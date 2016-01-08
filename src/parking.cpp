#include "parking.h"
#include "phoenix_CC2016_service/phoenix_CC2016_service.h"


bool Parking::initialize() {

    cycleCounter = 0;

    currentState = ParkingState::SEARCHING;
    firstCircleArc = true;
    drivenArcLength = 0;
    parkingSpaceSize = 0;

    mavlinkChannel = readChannel<Mavlink::Data>("MAVLINK_IN");
    currentXPosition = 0;
    lastTimeStamp = -1;
    lastValidMeasurement = 0.5;

    //edgePosition.assign(100, 0);
    //edgeType.assign(100, 0);

    //myfile.open ("data.csv");

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

    //usleep(3000);

    switch (currentState) {

    case ParkingState::SEARCHING: {

        ++cycleCounter;

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


        /*
         *  get distance measurements from lidar sensor
         */
        for( const auto& msg : *mavlinkChannel )
        {
            if (msg.msgid == MAVLINK_MSG_ID_PROXIMITY && cycleCounter > 5) {  //cycleCounter > 1 nur für debugging, da extrem viele messwerte in den ersten paar frames auftauchen
                mavlink_proximity_t distanceMsg;
                mavlink_msg_proximity_decode(&msg, &distanceMsg);
                double distance = distanceMsg.distance;
                if (distance > 0.5) distance = 0.5;
                if (distance < 0.09) distance = lastValidMeasurement;

                distanceMeasurement.push_back(distance); //add the measurement to the distance vector
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


        /*
         *  find big enough jumps (=beginning or end of parking space) in the distance measurement data
         */
        int numEdges = 0;

        Parking::findEdges(&distanceMeasurement, &xPosition, &edgePosition, &edgeType, &numEdges); //find the x-location of big enough "jumps" in distance measurements

        if (edgePosition.size()>5)
        {
            logger.debug("parking") << edgePosition.at(0) << ", " << edgePosition.at(1) << ", " << edgePosition.at(2) << ", " << edgePosition.at(3) << ", " << edgePosition.at(4)<< ", " << edgePosition.at(5);
            logger.debug("parking") << edgeType.at(0) << ", " << edgeType.at(1) << ", " << edgeType.at(2) << ", " << edgeType.at(3) << ", " << edgeType.at(4)<< ", " << edgeType.at(5);
        }


        /*
         *  logic for extracting a valid parking space from edge positions
         */
        bool spaceStarted = false;
        //bool validSpaceDetected = false;
        double size, startX, endX;
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
                if (size >= config().get("minParkingSpaceSize", 0.50) && size <= config().get("maxParkingSpaceSize", 0.65))
                {
                    //validSpaceDetected = true;
                    endX =  edgePosition.at(i);
                    //currentState = ParkingState::STOPPING;
                    logger.debug("praking") << "valid space: size=" << size << ", startX=" << startX << ", endX=" << endX;
                    break;
                }
            }
        }

        return true;
    }

    case ParkingState::STOPPING: {

        //TODO
        // if (car has stopped) currentState = ParkingState::ENTERING;
        currentState = ParkingState::ENTERING;

    }
    case ParkingState::ENTERING: {
        /*
         * Einfache Methode mit gleichsinnigem Einschlag beider Achsen
         *
         * double y0 = distance from second box to right side of car
         * double k = 0.05; //safety distance to corner of second box
         * double delta_max = 32*pi/180; //maximum steering angle
         * double l = 0.25;
         * double x0 = y0/tan(delta_max) - k; //distance from end of parking spot (detected edge) to front of car
         * double x_begin_steering = x0 - l;
         *
         * if (x_now <= x_begin_steering) -> set servos to max steering angle
         * else -> drive straight backwards still
         *
         * if (ToF_back <= 0.05) {
         *    -> stop moving         *    
         *    currentState = ParkingState::CORRECTING;
         * }
         */

        /*
         * Bessere Methode mit 2 Kreisboegen
         */
        double y0 = 0.2;  //TODO: distance from second box to right side of car
        double lr = config().get("wheelbase", 0.21); //Radstand
        double l = config().get("carLength", 0.32);  //Fahrzeuglänge
        double b = config().get("carWidth", 0.2); //Fahrzeugbreite
        double delta_max = config().get("maxSteeringAngle", 24)*M_PI/180; //maximum steering angle
        double r = lr/2*tan(M_PI/2 - delta_max); //Radius des Wendekreises bei Volleinschlag beider Achsen

        double k = config().get("k", 0.05);
        double d = config().get("d", 0.03);

        double R = sqrt(l*l/4 + (r+b/2)*(r+b/2)); //Radius den das äußerste Eck des Fahrzeugs bei volleingeschlagenen Rädern zurücklegt
        double s = sqrt((R+k)*(R+k) - (parkingSpaceSize - d - l/2)*(parkingSpaceSize - d - l/2));
        double alpha = acos((r-y0+s)/(2*r)); //Winkel (in rad) der 2 Kreisboegen, die zum einfahren genutzt werden
        double x0 = d + l/2 + 2*r*sin(alpha) - parkingSpaceSize; //Abstand vom Ende der 2. Box zur Mitte des Fahrzeugs bei Lenkbeginn (Anfang erster Kreisbogen)

        double d_mid2lidar = config().get("distanceMid2Lidar", 0.04); //Abstand von Fahrzeugmitte zur Lidar
        double x_begin_steering = x0 - d_mid2lidar;

        double x_now = 0; //TODO: Momentante x-Position entlang der Straße (ausgehend vom Parkbeginn x=0)
        if (x_now <= x_begin_steering) {
             if (firstCircleArc) {
                 //-> set servos to max steering angle
                 drivenArcLength += 0; //Distanzschritt vom Hall/Drehgeber;
                 if (drivenArcLength >= alpha) {
                     firstCircleArc = false;
                     drivenArcLength = 0;
                 }
             }
             else {
                 //-> set servos to max steering angle in other direction
                 drivenArcLength += 0; //Distanzschritt vom Hall/Drehgeber;
                 if (drivenArcLength >= alpha) {
                     currentState = ParkingState::CORRECTING;
                 }
             }
        }
        else {} // -> drive straight backwards still



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
void Parking::findEdges(std::vector<double> *dst, std::vector<double> *x, std::vector<double> *edgePosition, std::vector<double> *edgeType, int *numEdges)
{
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

            if (edgePosition->size() < numEdge-1)
            {
                edgePosition->push_back(x->at(m));
                edgeType->push_back(maxGrad > 0 ? -1 : 1);
            }
            else
            {
                edgePosition->at(numEdge) = x->at(m);
                edgeType->at(numEdge) = maxGrad > 0 ? -1 : 1;
            }

            ++numEdge;
            i += res;
        }
    }

    *numEdges = numEdge;
}

