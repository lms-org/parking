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
    straightMove = false;
    correctingCounter = 0;
    yawAngleSet = false;


    state.priority = 100;
    state.name = "PARKING";

    currentState = ParkingState::SEARCHING;
    firstCircleArc = true;
    parkingSpaceSize = 0.0;

    mavlinkChannel = readChannel<Mavlink::Data>("MAVLINK_IN");
    sensors = readChannel<sensor_utils::SensorContainer>("SENSORS");
    car = writeChannel<sensor_utils::Car>("CAR");

    currentXPosition = 0;
    lastTimeStamp = -1;
    lastImuTimeStamp = -1;
    lastValidMeasurement = config().get<float>("maxDistanceLidar", 0.6);

    //myfile.open("parkingData.csv");
    //myfile.open(saveLogDir("parking") + "/parkingData.csv" );
    return true;
}

bool Parking::deinitialize() {
    //myfile.open("parkingData.csv");
    if(isEnableSave()){
        const std::string filename = saveLogDir("parking") + "/parkingData_" + std::to_string(fileCounter++) + ".csv";
        std::ofstream myfile(filename);
        logger.info("file saved") << "filecount: " << fileCounter << ", vectorSize: " << xPosition.size();
        for (int i = 0; i < xPosition.size(); ++i)
        {
            myfile << xPosition.at(i) << "," << distanceMeasurement.at(i) << "," << distanceMeasurement2.at(i) <<std::endl;
        }
        myfile.flush();
        myfile.close();
    }

    //reset data vectors
    distanceMeasurement.clear();
    distanceMeasurement.shrink_to_fit();
    xPosition.clear();
    xPosition.shrink_to_fit();
    edgePosition.clear();
    edgePosition.shrink_to_fit();
    edgeType.clear();
    edgeType.shrink_to_fit();
    return true;
}

bool Parking::cycle() {
    lms::ServiceHandle<phoenix_CC2016_service::Phoenix_CC2016Service> phoenixService= getService<phoenix_CC2016_service::Phoenix_CC2016Service>("PHOENIX_SERVICE");
    if((phoenixService->driveMode() != phoenix_CC2016_service::CCDriveMode::PARKING)|| phoenixService->rcStateChanged()){
        //remove parking car-control-state
        car->removeState("PARKING");
        deinitialize();
        initialize();
        logger.info("reset parking");
        return true;
    }

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

         /***************************************************
         * drive straight along the middle of the right lane
         ***************************************************/

        //the desired state is such that phi=0 (straight driving) and y=0.2 (middle of right lane) with respect to the middle lane        
        setSteeringAngles(-0.2, config().get<float>("searchingPhiFactor"), DrivingMode::FORWARD);

        state.targetSpeed = config().get<float>("velocitySearching", 1.0);


         /***************************************************
         * process lidar measurements and detect a valid parking space
         ***************************************************/

        //update current x-position, distance measurement vector and x-position vector
        updatePositionAndDistance();

        //find the x-positions of big enough "jumps" in distance measurements
        Parking::findEdges();

        //find a valid (according to size) parking space
        bool spaceFound = Parking::findValidParkingSpace(config().get<float>("minParkingSpaceSize", 0.3), config().get<float>("maxParkingSpaceSize", 0.75));

        if (spaceFound)
        {
            logger.error("valid parking space") << "size=" << parkingSpaceSize << ", startX=" << startX << ", endX=" << endX;
            timeSpaceWasFound = lms::Time::now();
            currentState = ParkingState::STOPPING;
        }

        //worst case: reached end of parking lane without finding a valid space --> drive backwards and try again
        if (currentXPosition > config().get<float>("xMaxBeforeWorstCase", 6.0)) // TODO: && foundObstacle
        {
            currentState = ParkingState::WORST_CASE_BACKWARDS;
        }

        break;       
    }

    //Anhalten nach Searching
    case ParkingState::STOPPING:
    {

        /***************************************************
        * come to a stop, but dont slip (otherwise the position measurements are crap)
        ***************************************************/

        //drive straight still
        setSteeringAngles(-0.2, config().get<float>("searchingPhiFactor"), DrivingMode::FORWARD);

        //update current x-position
        updatePositionFromHall();

        //set target speed as if the car was decelerating constantly
        if (config().get<float>("decelerationStopping", 3.0)) {
            state.targetSpeed = config().get<float>("velocitySearching", 1.0) - timeSpaceWasFound.since().toFloat()*config().get<float>("decelerationStopping", 3.0);
        }
        else
        {
            state.targetSpeed = 0.0;
        }

        //int num_y_vals = config().get("numberOfY0measurements", 20);
        if (car->velocity() < config().get("minVelocityBeforeDrivingBackwards", 0.4)) {

           y0_dynamic = config().get<float>("y0_worstCase", 0.23);

            currentState = ParkingState::ENTERING;
        }

        break;
    }

    //Wir fahren in die Parklücke
    case ParkingState::ENTERING:
    {

        state.indicatorRight = true;

        updatePositionFromHall();

        //TODO nicht fest
        parkingSpaceSize = 0.55;

        /***************************************************
        * calculate parameters for entering maneuver
        ***************************************************/

        double lr = config().get<float>("wheelbase", 0.21); //Radstand
        double l = config().get<float>("carLength", 0.32);  //Fahrzeuglänge
        double b = config().get<float>("carWidth", 0.2); //Fahrzeugbreite
        double delta_max = config().get("maxSteeringAngle", 24)*M_PI/180; //maximum steering angle
        double k = config().get<float>("k", 0.03); //Sicherheitsabstand zur Ecke der 2. Box
        double d = config().get<float>("d", 0.05); //Sicherheitsabstand zur 1. Box im eingeparkten Zustand
        double y0 = config().get<float>("y0_worstCase", 0.24); //seitlicher Abstand zur zweiten Box

        //TODO delta_max + 1*pi/180;
        double r = lr/2*tan(M_PI/2 - (delta_max-1*M_PI/180)); //Radius des Wendekreises (bezogen auf den Fahrzeugmittelpunkt) bei Volleinschlag beider Achsen
        double R = sqrt(l*l/4 + (r+b/2)*(r+b/2)); //Radius den das äußerste Eck des Fahrzeugs bei volleingeschlagenen Rädern zurücklegt
        double s = sqrt((R+k)*(R+k) - (parkingSpaceSize - d - l/2)*(parkingSpaceSize - d - l/2));
        double alpha = acos((r-y0+s)/(2*r)); //Winkel (in rad) der 2 Kreisboegen, die zum einfahren genutzt werden
        double x0 = d + l/2 + 2*r*sin(alpha) - parkingSpaceSize; //Abstand vom Ende der 2. Box zur Mitte des Fahrzeugs bei Lenkbeginn (Anfang erster Kreisbogen)        
        double x_begin_steering = config().get<float>("xDistanceCorrection",0.09) + endX + x0 - config().get<float>("distanceMidLidarX", 0.09);

        if (currentXPosition > x_begin_steering) //drive straight backwards
        {
            setSteeringAngles(-0.2, config().get<float>("searchingPhiFactor"), DrivingMode::BACKWARDS);

            double brakingDistance = config().get<float>("brakingDistanceUntilSteering", 0.15);
            //TODO evtl
            if (false && currentXPosition < x_begin_steering + brakingDistance)
            {
                double deltaX = currentXPosition - x_begin_steering; //distance until steering
                double deltaV = config().get<float>("velocityApproaching", 1.0) - config().get<float>("velocityEntering", 0.5);
                state.targetSpeed = -config().get<float>("velocityApproaching", 0.5)
                        +  (1.0-deltaX/brakingDistance) * deltaV;
            }
            else
            {
                state.targetSpeed = -config().get<float>("velocityApproaching", 0.5);
            }

        }
        else //begin steering into parking space
        {            
            state.targetSpeed = -config().get<float>("velocityEntering", 0.5);

            if (! yawAngleSet)
            {
                yawAngleSet = true;
                yawAngleStartEntering = car_yawAngle; //set yawAngle reference
            }

            double drivenArc = car_yawAngle - yawAngleStartEntering;

            if (firstCircleArc) {
                // set servos to max steering angle
                state.steering_front = -delta_max;
                state.steering_rear = delta_max;

                if (drivenArc >= alpha) {
                    firstCircleArc = false;
                }
            }
            else {
                // set servos to max steering angle in other direction
                state.steering_front = delta_max;
                state.steering_rear = -delta_max;

                if (drivenArc <= 0.0 + config().get<float>("alphaOffset",0.0)) {
                    state.steering_front = 0.0; // * 180. / M_PI;
                    state.steering_rear = 0.0; // * 180. / M_PI;
                    state.targetSpeed = 0.0; //config().get<float>("velocityCorrecting", 0.5);

                    currentXPosition = 0;
                    currentState = ParkingState::CORRECTING;
                }
            }

        }

        break;

    }
    case ParkingState::CORRECTING:
    {

        updatePositionFromHall();

        std::vector<float> correctingDistances = config().getArray<float>("correctingDistances");

        if (correctingCounter >= correctingDistances.size())
        {
            currentState = ParkingState::FINISHED;
        }
        else
        {
            double phi_ist = car_yawAngle - yawAngleStartEntering;

            if (correctingCounter%2 == 0) //forward
            {
                state.targetSpeed = config().get<float>("velocityCorrecting", 0.5);
                setSteeringAngles(-0.29, 0.0, 0.0, 6.0*phi_ist, DrivingMode::FORWARD);

                if (currentXPosition >= correctingDistances.at(correctingCounter))
                {
                    ++correctingCounter;
                    currentXPosition = 0.0;
                }

            }
            else //backwards
            {
                state.targetSpeed = -config().get<float>("velocityCorrecting", 0.5);
                setSteeringAngles(-0.29, 0.0, 0.0, 6.0*phi_ist, DrivingMode::BACKWARDS);

                if (-currentXPosition >= correctingDistances.at(correctingCounter))
                {
                    ++correctingCounter;
                    currentXPosition = 0.0;
                }

            }

        }

        break;
    }
    case ParkingState::FINISHED: {

        state.targetSpeed = 0.0;
        state.steering_front = 0.0;
        state.steering_rear = 0.0;

        state.indicatorLeft = false;
        state.indicatorRight = false;

        break;
    }
    case ParkingState::WORST_CASE_BACKWARDS: {

        updatePositionFromHall();

        setSteeringAngles(-0.2, config().get<float>("searchingPhiFactor"), DrivingMode::BACKWARDS);
        state.targetSpeed = -config().get<float>("velocitySearching", 1.0);

        if (currentXPosition <= config().get<float>("xStartSearchingAgain", 0.8))
        {
            //reset parking module and try again
            deinitialize();
            initialize();
        }

        break;
    }

    }

    state.indicatorRight = true;


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
    auto *dst = &distanceMeasurement;
    auto *x = &xPosition;

    /*formula for rough resolution: res = l*b_min/(v*s)
    l: lidar measurements per second
    b_min: minimum object width that has to be detected
    v: velocity
    s: safety factor*/

    uint res = config().get<int>("lidarMeasurementsPerSecond", 300) * 0.1 / (config().get<float>("velocitySearching", 1.0) * 2.0);

    if (dst->size() < 4*res) return;

    double gradientThreshold = config().get<float>("gradientThreshold", 0.16); //jumps bigger than 'gradientThreshold' [m] are treated as edges

    uint numEdge = 0;
    uint i, m;
    double gradGrob, maxGrad, grad;

    for(i=res; i < dst->size()-res-ceil(config().get<int>("medianFilterSize", 3)/2.0); i += res)
    {
        gradGrob = dst->at(i+res) - dst->at(i-res);

        if (fabs(gradGrob) > gradientThreshold)
        {
            maxGrad = 0.0;
            uint indMax = 0;
            for (m=i-res+1; m<i+res; ++m)
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

    //DEBUG print edges
    if (config().get<bool>("debugPrintEdges", true))
    {
        std::ostringstream s;
        for (uint i=0; i < numEdges; ++i)
        {
            s << edgePosition.at(i);
            s << ", ";
        }
        logger.error("edgePositions") << s.str();

        std::ostringstream s2;
        for (uint i=0; i < numEdges; ++i)
        {
            s2 << edgeType.at(i);
            s2 << ", ";
        }
        logger.error("edgeTypes") << s2.str();
    }
}

void Parking::updatePositionAndDistance()
{

    for( const auto& msg : *mavlinkChannel)
    {
        if (msg.msgid == MAVLINK_MSG_ID_PROXIMITY && msg.compid == 0 && cycleCounter > 5) {  //cycleCounter > 1 nur für debugging, da extrem viele messwerte in den ersten paar frames auftauchen
            mavlink_proximity_t distanceMsg;
            mavlink_msg_proximity_decode(&msg, &distanceMsg);
            double distance = distanceMsg.distance;
            double maxDistance = config().get<float>("maxDistanceLidar", 0.6);
            if (distance > maxDistance) distance = maxDistance; //limit max distance
            if (distance < 0.05) distance = lastValidMeasurement; //ignore measurements < 9 cm (artifacts)

            if (distanceMeasurement.size() < 5) {
                distanceMeasurement.push_back(config().get<float>("maxDistanceLidar", 0.6)); //prevent possible spikes at the beginning
                distanceMeasurement2.push_back(config().get<float>("maxDistanceLidar", 0.6));
            }
            else
            {
                distanceMeasurement.push_back(distance); //add the measurement to the distance vector
                distanceMeasurement2.push_back(distance);
            }

            int medianSize = config().get<int>("medianFilterSize", 3);
            if (distanceMeasurement.size() >= medianSize && medianSize > 0) //median filter the last elements
            {
                std::nth_element(distanceMeasurement.end()-medianSize, distanceMeasurement.end()-ceil(medianSize/2.0), distanceMeasurement.end());
            }

            //logger.debug("lidar measurement") << distance;
            lastValidMeasurement = distance;

            if (lastTimeStamp < 0) {
                xPosition.push_back(currentXPosition); //at first iteration of framework
                lastTimeStamp = distanceMsg.timestamp;
            }
            else {
                currentXPosition += car_velocity*(distanceMsg.timestamp - lastTimeStamp)/1000000.0; //update x-position (timestamp is in us)
                lastTimeStamp = distanceMsg.timestamp; //update timestamp
                xPosition.push_back(currentXPosition); //add x-position to the position vector
            }
        }
    }   

}

void Parking::updatePositionFromHall()
{
    if(sensors->hasSensor("HALL")) {
        auto hall = sensors->sensor<sensor_utils::Odometer>("HALL");
        auto dst = hall->distance.x();
        currentXPosition += dst;
        //logger.debug("currentXPosition") << currentXPosition;
    }
}

void Parking::updateYawAngle()
{
    //upate from ego estimator
    car_yawAngle += car->deltaPhi();

    //old function using raw measurements
    /*double yawRate = 0.0;
    int yawCount = 0;
    for( const auto& msg : *mavlinkChannel )
    {
        if (msg.msgid == MAVLINK_MSG_ID_IMU && cycleCounter > 5) {
            mavlink_imu_t imuMessage;
            mavlink_msg_imu_decode(&msg, &imuMessage);
            double offset = 0.0299405;
            yawRate += imuMessage.zgyro-offset;
            ++yawCount;
            if (lastImuTimeStamp < 0) {
                lastImuTimeStamp = imuMessage.timestamp;
            }
            else {
                //double yawAngleDiffGrad = 10*(imuMessage.zgyro-offset)*(imuMessage.timestamp - lastTimeStamp)/1000000.0;
                //car_yawAngle += yawAngleDiffGrad*M_PI/180.0;
                //car_yawAngle += (imuMessage.zgyro-offset)*(imuMessage.timestamp - lastTimeStamp)/1000000.0;
                lastImuTimeStamp = imuMessage.timestamp;
            }
            //logger.debug("car_yawAngle") << car_yawAngle;
        }
    }
    if (yawCount > 0) yawRate /= yawCount;
    car_yawAngle += 0.01*yawRate;*/

    return;
}


void Parking::updateVelocity()
{
    car_velocity = car->velocity();
    /*if(sensors->hasSensor("HALL")) {
        auto hall = sensors->sensor<sensor_utils::Odometer>("HALL");
        car_velocity = hall->velocity.x();
    }*/
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
    double R_backwards[] = {-2.000, 0.580, -2.000, 1.000};
    double F_backwards[] = {-2.000, -0.420, -2.000, 0.000};

    //find regression line y=mx+b through some points of the middle lane (not necessary but contributes to better stability in straight line performance)
    double m, b;
    fitLineToMiddleLane(&m, &b);

    //the current vehicle state is [y_ist; phi_ist]
    double y_ist = -b*cos(-atan(m));
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
    double R_backwards[] = {-2.000, 0.580, -2.000, 1.000};
    double F_backwards[] = {-2.000, -0.420, -2.000, 0.000};

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


