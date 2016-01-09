#include "parking.h"
#include "phoenix_CC2016_service/phoenix_CC2016_service.h"


bool Parking::initialize() {

    cycleCounter = 0;
    numEdges = 0;
    lastVelocity = 0.0;

    state.priority = 10;
    state.name = "DEFAULT";

    currentState = ParkingState::SEARCHING;
    firstCircleArc = true;
    drivenArcLength = 0;
    parkingSpaceSize = 0;

    mavlinkChannel = readChannel<Mavlink::Data>("MAVLINK_IN");
    currentXPosition = 0;
    lastTimeStamp = -1;
    lastValidMeasurement = 0.5;

    car = writeChannel<sensor_utils::Car>("CAR");

    //edgePosition.assign(100, 0);
    //edgeType.assign(100, 0);

    //myfile.open ("parkingData.csv");

    return true;
}

bool Parking::deinitialize() {
    distanceMeasurement.clear();
    xPosition.clear();
    return true;
}

bool Parking::cycle() {

    /*if(getService<phoenix_CC2016_service::Phoenix_CC2016Service>("PHOENIX_SERVICE")->driveMode() != phoenix_CC2016_service::CCDriveMode::PARKING){
        //TODO remove parking car-control-state
        return true;
    }*/

    ++cycleCounter;

    //usleep(5000);

    switch (currentState) {

    case ParkingState::SEARCHING: {

        /*
         *  get velocity from odometer to calculate current x positions
         */
        double velocity = 0;
        int velCount = 0;

        for( const auto& msg : *mavlinkChannel )
        {
            if (msg.msgid == MAVLINK_MSG_ID_ODOMETER_DELTA) {
                mavlink_odometer_delta_t velocityMesage;                
                mavlink_msg_odometer_delta_decode(&msg, &velocityMesage);                
                if (msg.compid == 0) { //0 = Hall
                   velocity += velocityMesage.xvelocity;
                   ++velCount;
                }
            }
        }
        if (velCount > 0) velocity /= velCount; //average velocity in one timestep of the framework

        double k= 0.1;
        //velocity = k*velocity + (1-k)*lastVelocity;
        lastVelocity = velocity;

        //velocity = 1.0;

        //HACK wegen Hallsensor Ausfall:
        //velocity *= 3;

        /*
         *  get distance measurements from lidar sensor
         */
        for( const auto& msg : *mavlinkChannel )
        {
            if (msg.msgid == MAVLINK_MSG_ID_PROXIMITY && cycleCounter > 5) {  //cycleCounter > 1 nur für debugging, da extrem viele messwerte in den ersten paar frames auftauchen
                mavlink_proximity_t distanceMsg;
                mavlink_msg_proximity_decode(&msg, &distanceMsg);
                double distance = distanceMsg.distance;
                if (distance > 0.6) distance = 0.6; //limit max distance
                if (distance < 0.09) distance = lastValidMeasurement; //ignore measurements < 9 cm (artifacts)

                distanceMeasurement.push_back(distance); //add the measurement to the distance vector
                logger.debug("lidar measurement") << distance;
                lastValidMeasurement = distance;

                if (lastTimeStamp < 0) {
                    xPosition.push_back(currentXPosition); //at first iteration of framework
                    lastTimeStamp = distanceMsg.timestamp;
                }
                else {
                    currentXPosition += velocity*(distanceMsg.timestamp - lastTimeStamp)/1000000.0; //update x-position (timestamp is in us)
                    lastTimeStamp = distanceMsg.timestamp; //update timestamp
                    xPosition.push_back(currentXPosition); //add x-position to the position vector
                }
            }
        }

        //if (xPosition.size() > 0) myfile << xPosition.at(xPosition.size()-1) << "," << distanceMeasurement.at(distanceMeasurement.size()-1) << "," << velocity << std::endl;

        /*
         *  find big enough jumps (=beginning or end of parking space) in the distance measurement data
         */        

        /*
         * output
         * uint numEdges:               number of edges found
         * vector<double> edgePosition: x-positions of edge
         * vector<int> edgePosition:    type of edge (+1 or -1)
         */
        Parking::findEdges(); //find the x-location of big enough "jumps" in distance measurements

        std::ostringstream s;
        for (uint i=0; i < numEdges; ++i)
        {
            s << edgePosition.at(i);
            s << ", ";
        }
        logger.debug("edgePositions") << s.str();


        /*
         *  logic for extracting a valid parking space from edge positions
         */
        bool spaceStarted = false;
        double size;
        for (int i=0; i < numEdges; ++i)
        {
            if (edgeType.at(i) == -1)
            {
                spaceStarted = true;
                startX = edgePosition.at(i);
            }
            if (edgeType.at(i) == 1 && spaceStarted)
            {
                size = edgePosition.at(i) - startX;
                if (size >= config().get("minParkingSpaceSize", 0.45) && size <= config().get("maxParkingSpaceSize", 0.75))
                {                    
                    endX =  edgePosition.at(i);
                    parkingSpaceSize = size;
                    lastTimeStamp = -1;
                    currentState = ParkingState::STOPPING;
                    logger.debug("praking") << "valid space: size=" << size << ", startX=" << startX << ", endX=" << endX;
                    break;
                }
                else spaceStarted = false;
            }
        }

        return true;
    }

    case ParkingState::STOPPING: {       

        state.targetSpeed = 0;
        car->putState(state);

        if (car->velocity() < 0.02) currentState = ParkingState::ENTERING;

    }
    case ParkingState::ENTERING: {

        state.targetSpeed = 0.4;

        double velocity = 0;
        int velCount = 0;
        double xNow = currentXPosition;
        double timePassed = 0.0;

        for( const auto& msg : *mavlinkChannel )
        {
            if (msg.msgid == MAVLINK_MSG_ID_ODOMETER_DELTA) {
                mavlink_odometer_delta_t velocityMesage;
                mavlink_msg_odometer_delta_decode(&msg, &velocityMesage);
                if (msg.compid == 0) { //0 = Hall
                   velocity += velocityMesage.xvelocity;
                   ++velCount;

                   if (lastTimeStamp < 0) {
                       lastTimeStamp = velocityMesage.timestamp;
                   }
                   else {
                       timePassed += (velocityMesage.timestamp - lastTimeStamp)/1000000.0;
                       lastTimeStamp = velocityMesage.timestamp;
                   }
                }
            }
        }
        if (velCount > 0) velocity /= velCount; //average velocity in one timestep of the framework
        double deltaX = velocity*timePassed;
        currentXPosition += deltaX;

        /*for( const auto& msg : *mavlinkChannel )
        {
            if (msg.msgid == MAVLINK_MSG_ID_PROXIMITY && cycleCounter > 5) {  //cycleCounter > 1 nur für debugging, da extrem viele messwerte in den ersten paar frames auftauchen
                mavlink_proximity_t distanceMsg;
                mavlink_msg_proximity_decode(&msg, &distanceMsg);
                double distance = distanceMsg.distance;
                if (distance > 0.6) distance = 0.6; //limit max distance
                if (distance < 0.09) distance = lastValidMeasurement; //ignore measurements < 9 cm (artifacts)

                //distanceMeasurement.push_back(distance); //add the measurement to the distance vector
                //logger.debug("lidar measurement") << distance;
                //lastValidMeasurement = distance;

                if (lastTimeStamp < 0) {
                    //xPosition.push_back(currentXPosition); //at first iteration of framework
                    lastTimeStamp = distanceMsg.timestamp;
                }
                else {
                    currentXPosition += velocity*(distanceMsg.timestamp - lastTimeStamp)/1000000.0; //update x-position (timestamp is in us)
                    lastTimeStamp = distanceMsg.timestamp; //update timestamp
                    //xPosition.push_back(currentXPosition); //add x-position to the position vector
                }
            }
        }*/

        logger.debug("velocity") << velocity;
        logger.debug("currentX") << currentXPosition;


        /*
         * Einparkmethode mit 2 Kreisboegen
         */

        //HACK, ist nicht konstant
        parkingSpaceSize = 0.55;

        double y0 = 0.20;  //TODO: distance from second box to right side of car
        double lr = config().get("wheelbase", 0.21); //Radstand
        double l = config().get("carLength", 0.32);  //Fahrzeuglänge
        double b = config().get("carWidth", 0.2); //Fahrzeugbreite
        double delta_max = config().get("maxSteeringAngleDegrees", 20)*M_PI/180; //maximum steering angle

        double r = lr/2*tan(M_PI/2 - delta_max); //Radius des Wendekreises bei Volleinschlag beider Achsen

        double k = config().get("k", 0.05); //Sicherheitsabstand zur Ecke der 2. Box
        double d = config().get("d", 0.03); //Sicherheitsabstand zur 1. Box im eingeparkten Zustand

        double R = sqrt(l*l/4 + (r+b/2)*(r+b/2)); //Radius den das äußerste Eck des Fahrzeugs bei volleingeschlagenen Rädern zurücklegt
        double s = sqrt((R+k)*(R+k) - (parkingSpaceSize - d - l/2)*(parkingSpaceSize - d - l/2));
        double alpha = acos((r-y0+s)/(2*r)); //Winkel (in rad) der 2 Kreisboegen, die zum einfahren genutzt werden
        double x0 = d + l/2 + 2*r*sin(alpha) - parkingSpaceSize + endX; //Abstand vom Ende der 2. Box zur Mitte des Fahrzeugs bei Lenkbeginn (Anfang erster Kreisbogen)

        double d_mid2lidar = config().get("distanceMid2Lidar", 0.1); //Abstand von Fahrzeugmitte zum Lidar
        double x_begin_steering = x0 - d_mid2lidar;

        logger.debug("x_begin_steering") << x_begin_steering;

        double x_now = currentXPosition; //TODO: Momentante x-Position entlang der Straße (ausgehend vom Parkbeginn x=0)
        if (x_now <= x_begin_steering) {

            //drivenArcLength -= (currentXPosition - xNow);
            drivenArcLength -= deltaX;
            double max_steering = 18;

             if (firstCircleArc) {
                 // set servos to max steering angle
                 state.steering_front = -max_steering*M_PI/180;
                 state.steering_rear = max_steering*M_PI/180;

                 logger.warn("firstCircleArc") << "alpha*r=" << alpha*r << ", drivenArcLength=" << drivenArcLength;
                 if (drivenArcLength >= alpha*r) {
                     firstCircleArc = false;
                     drivenArcLength = 0;
                 }
             }
             else {
                 // set servos to max steering angle in other direction
                 state.steering_front = max_steering*M_PI/180;
                 state.steering_rear = -max_steering*M_PI/180;
                 logger.error("secondCircleArc") << "alpha*r=" << alpha*r << ", drivenArcLength=" << drivenArcLength;
                 if (drivenArcLength >= alpha*r) {
                     state.steering_front = 0.0; // * 180. / M_PI;
                     state.steering_rear = 0.0; // * 180. / M_PI;
                     currentState = ParkingState::CORRECTING;

                 }
             }

             car->putState(state);
        }
        else {
            // -> drive straight backwards
        }



    }
    case ParkingState::CORRECTING: {

        // TODO: test if we need more than one correcting movement

    }
    case ParkingState::FINISHED: {

        // TODO

    }

    }


    return true;
}

/*
 * input
 * dst:             distance measurements from lidar sensor
 * x:               corresponding x-position for each value in dst
 *
 * output
 * edgePosition:    x-position of the "jumps" in distance measurements bigger than a threshold value
 * edgeType:        1 => end of potential parking space (jump in positive y-direction); -1 => start of ...
 * numEdges:        number of edges found
 */
void Parking::findEdges()
{
    std::vector<double> *dst = &distanceMeasurement;
    std::vector<double> *x = &xPosition;

    uint res = config().get("resolutionGrob", 20); //skip 'res' values in first coarse search for edges

    if (dst->size() < 4*res) return;

    double gradientThreshold = config().get("gradientThreshold", 0.16); //jumps bigger than 'gradientThreshold' [m] are treated as edges

    uint numEdge = 0;
    uint i, m;
    double grad, maxGrad;

    for(i=res; i < dst->size()-res; i += res)
    {
        grad = dst->at(i+res) - dst->at(i-res);

        if (fabs(grad) > gradientThreshold)
        {
            maxGrad = 0.0;
            for (m=i-res+1; m<i+res-1; ++m)
            {
                grad = dst->at(m+1) - dst->at(m-1);
                maxGrad = fabs(grad) > fabs(maxGrad) ? grad : maxGrad;
            }

            if (edgePosition.size() <= numEdge)
            {
                edgePosition.push_back(x->at(m));
                edgeType.push_back(maxGrad > 0 ? -1 : 1);
            }
            else
            {
                edgePosition.at(numEdge) = x->at(m);
                edgeType.at(numEdge) = maxGrad > 0 ? -1 : 1;
            }

            ++numEdge;
            i += res;
        }
    }

    numEdges = numEdge;
}

