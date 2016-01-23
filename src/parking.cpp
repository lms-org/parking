#include "parking.h"
#include "phoenix_CC2016_service/phoenix_CC2016_service.h"


bool Parking::initialize() {

    cycleCounter = 0;
    numEdges = 0;
    car_yawAngle = 0.0;
    car_velocity = 0.0;
    car_xPosition = 0.0;
    yawAngleStartEntering = 0.0;
    endX = 0.0;
    y0_dynamic = 0.0;
    ind_end = 0.0;

    state.priority = 100;
    state.name = "PARKING";

    currentState = ParkingState::SEARCHING;
    firstCircleArc = true;
    parkingSpaceSize = 0;

    mavlinkChannel = readChannel<Mavlink::Data>("MAVLINK_IN");
    sensors = readChannel<sensor_utils::SensorContainer>("SENSORS");
    currentXPosition = 0;
    lastTimeStamp = -1;
    lastImuTimeStamp = -1;
    lastValidMeasurement = 0.5;

    car = writeChannel<sensor_utils::Car>("CAR");

    //edgePosition.assign(100, 0);
    //edgeType.assign(100, 0);

    myfile.open ("parkingData.csv");

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

    if (cycleCounter > 10)
    {
        updateYawAngle();
        updateVelocity();
    }

    switch (currentState)
    {

    case ParkingState::SEARCHING:
    {
        logger.info("SEARCHING");

         /***************************************************
         * drive straight along the middle of the right lane
         ***************************************************/

        //the desired state is such that phi=0 (straight driving) and y=0.2 (middle of right lane) with respect to the middle lane        
        setSteeringAngles(0.2, 0.0, DrivingMode::FORWARD);

        //set velocity and send state to car
        state.targetSpeed = config().get<float>("velocitySearching", 1.0);
        car->putState(state);


         /***************************************************
         * process lidar measurements and detect a valid parking space
         ***************************************************/

        //update current x-position, distance measurement vector and x-position vector
        updateXPosition(true);

        //find the x-positions of big enough "jumps" in distance measurements
        Parking::findEdges();

        //find a valid (according to size) parking space
        bool spaceFound = Parking::findValidParkingSpace(config().get<float>("minParkingSpaceSize", 0.3), config().get<float>("maxParkingSpaceSize", 0.75));

        if (spaceFound)
        {
            logger.info("valid parking space") << "size=" << parkingSpaceSize << ", startX=" << startX << ", endX=" << endX;
            timeSpaceWasFound = lms::Time::now();
            currentState = ParkingState::STOPPING;
        }

        break;       
    }

    case ParkingState::STOPPING:
    {
        logger.info("STOPPING");

        /***************************************************
        * come to a stop, but dont slip (otherwise the position measurements are crap)
        ***************************************************/

        //drive straight still
        setSteeringAngles(-0.2, 0.0, DrivingMode::FORWARD);

        //update current x-position
        updateXPosition(true);

        //set target speed as if the car was decelerating constantly
        if (config().get<float>("decelerationStopping", 3.0)) {
            state.targetSpeed = config().get("velocitySearching", 1.0) - timeSpaceWasFound.since().toFloat()*config().get<float>("decelerationStopping", 3.0);
        }
        else
        {
            state.targetSpeed = 0.0;
        }

        car->putState(state);

        int num_y_vals = config().get<float>("numberOfY0measurements", 20);
        if (car->velocity() < config().get<float>("minVelocityBeforeDrivingBackwards", 0.4)) {

            if (distanceMeasurement.size() > ind_end + num_y_vals && config().get<bool>("useYmeasurementOfSecondBox", false)) {
                std::nth_element(distanceMeasurement.begin() + ind_end, distanceMeasurement.begin() + ind_end + num_y_vals/2, distanceMeasurement.begin() + ind_end + num_y_vals);
                double median = distanceMeasurement.at(ind_end + num_y_vals/2);
                y0_dynamic = median - 0.06 + config().get<float>("distanceMidLidarY", 0.08); //0.06 lidar offset, 0.08 from lidar to middle of car
            }
            else y0_dynamic = 0.25;

            currentState = ParkingState::ENTERING;
        }

        break;
    }

    case ParkingState::ENTERING:
    {
        logger.info("ENTERING");

        updateXPosition(false);

        state.targetSpeed = -config().get<float>("velocityEntering", 0.5);

        double y0 = y0_dynamic;

        /***************************************************
        * calculate parameters for entering maneuver
        ***************************************************/

        double lr = config().get<float>("wheelbase", 0.21); //Radstand
        double l = config().get<float>("carLength", 0.32);  //Fahrzeuglänge
        double b = config().get<float>("carWidth", 0.2); //Fahrzeugbreite
        double delta_max = config().get("maxSteeringAngle", 22)*M_PI/180; //maximum steering angle
        double k = config().get<float>("k", 0.03); //Sicherheitsabstand zur Ecke der 2. Box
        double d = config().get<float>("d", 0.05); //Sicherheitsabstand zur 1. Box im eingeparkten Zustand

        double r = lr/2*tan(M_PI/2 - delta_max); //Radius des Wendekreises (bezogen auf den Fahrzeugmittelpunkt) bei Volleinschlag beider Achsen
        double R = sqrt(l*l/4 + (r+b/2)*(r+b/2)); //Radius den das äußerste Eck des Fahrzeugs bei volleingeschlagenen Rädern zurücklegt
        double s = sqrt((R+k)*(R+k) - (parkingSpaceSize - d - l/2)*(parkingSpaceSize - d - l/2));
        double alpha = acos((r-y0+s)/(2*r)); //Winkel (in rad) der 2 Kreisboegen, die zum einfahren genutzt werden
        double x0 = d + l/2 + 2*r*sin(alpha) - parkingSpaceSize; //Abstand vom Ende der 2. Box zur Mitte des Fahrzeugs bei Lenkbeginn (Anfang erster Kreisbogen)        
        double x_begin_steering = config().get<float>("xDistanceCorrection",0.09) + endX + x0 - config().get<float>("distanceMidLidarX", 0.09);

        logger.debug("params") << "x0=" << x0 << ", y0=" << y0_dynamic << ", size=" << parkingSpaceSize << ", endX=" << endX << ", currentX=" << currentXPosition;

        if (currentXPosition <= x_begin_steering) //begin steering into parking space
        {
            if (yawAngleStartEntering == 0.0) yawAngleStartEntering = car_yawAngle;

            double drivenArc = car_yawAngle - yawAngleStartEntering;

             if (firstCircleArc) {
                 // set servos to max steering angle
                 state.steering_front = -delta_max;
                 state.steering_rear = delta_max;

                 logger.debug("firstCircleArc") << "alpha=" << alpha << ", drivenArc=" << drivenArc;

                 if (drivenArc >= alpha) {
                     firstCircleArc = false;                    
                 }
             }
             else {
                 // set servos to max steering angle in other direction
                 state.steering_front = delta_max;
                 state.steering_rear = -delta_max;

                 logger.debug("secondCircleArc") << "alpha=" << alpha << ", drivenArc=" << drivenArc;

                 if (drivenArc <= 0.0) {
                     state.steering_front = 0.0; // * 180. / M_PI;
                     state.steering_rear = 0.0; // * 180. / M_PI;
                     state.targetSpeed = 0.0;

                     currentXPosition = 0;
                     currentState = ParkingState::CORRECTING;
                 }
             }

        }
        else //drive straight backwards
        {            
            setSteeringAngles(-0.2, 0.0, DrivingMode::BACKWARDS);
        }

        break;

    }
    case ParkingState::CORRECTING:
    {
        logger.info("SEARCHING");

        //set target state such that orientation (phi) is kept near 0 and y gets controlled by nearly the full steering angle
        double phi_ist = car_yawAngle - yawAngleStartEntering;
        setSteeringAngles(-0.3, 0.0, 0.0, phi_ist, DrivingMode::FORWARD);

        updateXPosition(false);

        if (getDistanceToMiddleLane() >= 58) {
            currentState = ParkingState::FINISHED;
        }

        if (currentXPosition < config().get<float>("correctingDistance", 0.04))
        {
            state.targetSpeed = config().get<float>("velocityCorrecting", 0.5);
        }
        else
        {
            state.targetSpeed = 0.0;
            state.steering_front = 0.0;
            state.steering_rear = 0.0;
        }

        break;
    }
    case ParkingState::FINISHED: {
        state.targetSpeed = 0.0;
        state.steering_front = 0.0;
        state.steering_rear = 0.0;

        //TODO flash lights :)
        break;
    }

    }

    car->putState(state);
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

    uint res = config().get<float>("resolutionGrob", 10); //skip 'res' values in first coarse search for edges

    if (dst->size() < 4*res) return;

    double gradientThreshold = config().get<float>("gradientThreshold", 0.16); //jumps bigger than 'gradientThreshold' [m] are treated as edges

    uint numEdge = 0;
    uint i, m;
    double gradGrob, maxGrad, grad;

    for(i=res; i < dst->size()-res; i += res)
    {
        gradGrob = dst->at(i+res) - dst->at(i-res);

        if (fabs(gradGrob) > gradientThreshold)
        {
            maxGrad = 0.0;
            uint indMax = 0;
            for (m=i-res+1; m<i+res-1; ++m)
            {
                grad = dst->at(m+1) - dst->at(m-1);
                if (fabs(grad) > fabs(maxGrad))
                {
                    maxGrad = grad;
                    indMax = m;
                }
            }
            ind_end = indMax; //index where the end of the second box is
            if (edgePosition.size() <= numEdge)
            {
                edgePosition.push_back(x->at(indMax));
                edgeType.push_back(gradGrob > 0 ? -1 : 1);
            }
            else
            {
                edgePosition.at(numEdge) = x->at(indMax);
                edgeType.at(numEdge) = gradGrob > 0 ? -1 : 1;
            }

            ++numEdge;
            i += res;
        }
    }
    numEdges = numEdge;

    //DEBUG print edge positions
    if (true)
    {
        std::ostringstream s;
        for (uint i=0; i < numEdges; ++i)
        {
            s << edgePosition.at(i);
            s << ", ";
        }
        logger.debug("edgePositions") << s.str();
    }
}

void Parking::updateXPosition(bool adjustVectors)
{
    if (true) {
    for( const auto& msg : *mavlinkChannel)
    {

        if (msg.msgid == MAVLINK_MSG_ID_PROXIMITY && cycleCounter > 5) {  //cycleCounter > 1 nur für debugging, da extrem viele messwerte in den ersten paar frames auftauchen
            mavlink_proximity_t distanceMsg;
            mavlink_msg_proximity_decode(&msg, &distanceMsg);
            double distance = distanceMsg.distance;
            double maxDistance = config().get<float>("maxDistanceLidar", 0.6);
            if (distance > maxDistance) distance = maxDistance; //limit max distance
            if (distance < 0.09) distance = lastValidMeasurement; //ignore measurements < 9 cm (artifacts)

            if (adjustVectors) distanceMeasurement.push_back(distance); //add the measurement to the distance vector
            //logger.debug("lidar measurement") << distance;
            lastValidMeasurement = distance;

            if (lastTimeStamp < 0) {
                if (adjustVectors) xPosition.push_back(currentXPosition); //at first iteration of framework
                lastTimeStamp = distanceMsg.timestamp;
            }
            else {
                currentXPosition += car_velocity*(distanceMsg.timestamp - lastTimeStamp)/1000000.0; //update x-position (timestamp is in us)
                lastTimeStamp = distanceMsg.timestamp; //update timestamp
                if (adjustVectors) xPosition.push_back(currentXPosition); //add x-position to the position vector

                myfile << currentXPosition << "," << distance << "," << car_velocity << std::endl;
            }
        }
        }

    }
    else  {       
        currentXPosition += car->movedDistance()*(car->velocity() > 0 ? 1 : -1);
        logger.debug("currentXPosition") << currentXPosition;

    }
    //if (distanceMeasurement.size() > 0) logger.debug("lidar measurement") << distanceMeasurement.at(distanceMeasurement.size()-1);
}

void Parking::updateYawAngle()
{
    car_yawAngle += car->deltaPhi();
    return;   
}


void Parking::updateVelocity()
{
    car_velocity = car->velocity();
    return;

    //old function using raw measurements
    /*car_velocity = 0.0;
    int velCount = 0;

    for( const auto& msg : *mavlinkChannel )
    {
        if (msg.msgid == MAVLINK_MSG_ID_ODOMETER_DELTA) {
            mavlink_odometer_delta_t velocityMesage;
            mavlink_msg_odometer_delta_decode(&msg, &velocityMesage);
            if (msg.compid == 0) { //0 = Hall
               car_velocity += velocityMesage.xvelocity;
               ++velCount;
            }
        }
    }
    if (velCount > 0) car_velocity /= velCount; //average velocity in one timestep of the framework
    */
}

void Parking::fitLineToMiddleLane(double *oM, double *oB)
{
    //get points from middle lane
    street_environment::RoadLane middleLane = getService<local_course::LocalCourse>("LOCAL_COURSE_SERVICE")->getCourse();
    std::vector<double> iX;
    std::vector<double> iY;

    for (uint i=config().get<int>("lineFitStartPoint", 2); i<=config().get<int>("lineFitEndPoint", 5); ++i) //get points 1 to 4
    {
        iX.push_back(middleLane.points().at(i).x);
        iY.push_back(middleLane.points().at(i).y);
    }

    if (iX.size() != iY.size())
    {
        *oM = 0.0;
        *oB = 0.0;
        return;
    }

    double xMean = 0.0, yMean = 0.0;
    for (uint i=0; i < iX.size(); ++i)
    {
        xMean += iX.at(i);
        yMean += iY.at(i);
    }
    xMean /= iX.size();
    yMean /= iX.size();

    double nom = 0.0, den = 0.0;
    for (uint i=0; i < iX.size(); ++i)
    {
        nom += (iX.at(i)-xMean)*(iY.at(i)-yMean);
        den += (iX.at(i)-xMean)*(iX.at(i)-xMean);
    }

    // regression line: y = m*x + b
    *oM = nom/den; //slope of line
    *oB = yMean - *oM*xMean; //y-intercept

    return;
}

bool Parking::findValidParkingSpace(double sizeMin, double sizeMax)
{
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

            if (size >= sizeMin && size <= sizeMax)
            {
                endX =  edgePosition.at(i);
                parkingSpaceSize = size;
                lastTimeStamp = -1;
                return true;
            }
            else spaceStarted = false;
        }
    }
    return false;
}

//current values are calculated with respect to the (straight) middle lane
void Parking::setSteeringAngles(double y_soll, double phi_soll, int drivingMode)
{

    //eigenvalues [-2, -2]
    double R_forward[] = {2.000, 1.420, 2.000, 1.000};
    double F_forward[] = {2.000, 0.420, 2.000, 0.000};
    double R_backwards[] = {-0.100, 0.979, -0.100, 1.000};
    double F_backwards[] = {2.000, 0.420, 2.000, 0.000};

    //find regression line y=mx+b through some points of the middle lane (not necessary but contributes to better stability in straight line performance)
    double m, b;
    fitLineToMiddleLane(&m, &b);

    //the current vehicle state is [y_ist; phi_ist]
    double y_ist = -b*cos(atan(m));
    double phi_ist = -atan(m);

    // define the controller gain R and the pre-filter F such that: u = [delta_v; delta_h] = F*[y_soll; phi_soll] - R*[y_ist; phi_ist]
    double delta_v, delta_h;
    if (drivingMode == DrivingMode::FORWARD)
    {
        delta_v = F_forward[0]*y_soll + F_forward[1]*phi_soll - (R_forward[0]*y_ist + R_forward[1]*phi_ist);
        delta_h = F_forward[2]*y_soll + F_forward[3]*phi_soll - (R_forward[2]*y_ist + R_forward[3]*phi_ist);
    }
    else
    {
        delta_v = F_backwards[0]*y_soll + F_backwards[1]*phi_soll - (R_backwards[0]*y_ist + R_backwards[1]*phi_ist);
        delta_h = F_backwards[2]*y_soll + F_backwards[3]*phi_soll - (R_backwards[2]*y_ist + R_backwards[3]*phi_ist);
    }

    //set the desired steering angles
    state.steering_front = delta_v;
    state.steering_rear = delta_h;

}

void Parking::setSteeringAngles(double y_soll, double phi_soll, double y_ist, double phi_ist, int drivingMode)
{

    //eigenvalues [-2, -2]
    double R_forward[] = {2.000, 1.420, 2.000, 1.000};
    double F_forward[] = {2.000, 0.420, 2.000, 0.000};
    double R_backwards[] = {-0.100, 0.979, -0.100, 1.000};
    double F_backwards[] = {2.000, 0.420, 2.000, 0.000};

    // define the controller gain R and the pre-filter F such that: u = [delta_v; delta_h] = F*[y_soll; phi_soll] - R*[y_ist; phi_ist]
    double delta_v, delta_h;
    if (drivingMode == DrivingMode::FORWARD)
    {
        delta_v = F_forward[0]*y_soll + F_forward[1]*phi_soll - (R_forward[0]*y_ist + R_forward[1]*phi_ist);
        delta_h = F_forward[2]*y_soll + F_forward[3]*phi_soll - (R_forward[2]*y_ist + R_forward[3]*phi_ist);
    }
    else
    {
        delta_v = F_backwards[0]*y_soll + F_backwards[1]*phi_soll - (R_backwards[0]*y_ist + R_backwards[1]*phi_ist);
        delta_h = F_backwards[2]*y_soll + F_backwards[3]*phi_soll - (R_backwards[2]*y_ist + R_backwards[3]*phi_ist);
    }

    //set the desired steering angles
    state.steering_front = delta_v;
    state.steering_rear = delta_h;

}

double Parking::getDistanceToMiddleLane()
{    
    double m, b;
    fitLineToMiddleLane(&m, &b);

    return b;
}


