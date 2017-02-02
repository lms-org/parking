#include "parking.h"
#include "phoenix_CC2016_service/phoenix_CC2016_service.h"



bool Parking::initialize() {

    car_yawAngle = 0.0;
    car_xPosition = 0.0;
    yawAngleStartEntering = 0.0;
    endX = 0.0;
    y0_dynamic = 0.0;
    ind_end = 0.0;
    straightMove = false;
    correctingCounter = 0;
    yawAngleSet = false;
    finishCounter = 0;


    state.priority = 100;
    state.name = "PARKING";

    currentState = ParkingState::SEARCHING;
    firstCircleArc = true;
    parkingSpaceSize = 0.0;

    sensors = readChannel<sensor_utils::SensorContainer>("SENSORS");
    car = writeChannel<street_environment::CarCommand>("CAR");
    laser_data = writeChannel<lms::math::PointCloud2f>("HOKUYO_LIDAR_DATA");

    currentXPosition = 0;
    lastTimeStamp = -1;
    lastImuTimeStamp = -1;


    return true;
}

bool Parking::deinitialize() {

    return true;
}

bool Parking::cycle() {
    {
        lms::ServiceHandle<phoenix_CC2016_service::Phoenix_CC2016Service> phoenixService = getService<phoenix_CC2016_service::Phoenix_CC2016Service>("PHOENIX_SERVICE");
        logger.info("[PARKING]") << "drive mode: " << static_cast<int>(phoenixService->driveMode());
         if((phoenixService->driveMode() != phoenix_CC2016_service::CCDriveMode::PARKING)|| phoenixService->rcStateChanged()){
            //remove parking car-control-state
            car->removeState("PARKING");
            deinitialize();
            initialize();
            state.indicatorLeft = false;
            state.indicatorRight = false;
            logger.debug("reset parking");
            return true;
        }
    }
    logger.info("parking");
    float distanceToObstacleFront = 0;
    bool validDistanceToObstacleFront = false;
    if(laser_data->points().size() > 0){
        validDistanceToObstacleFront = true;
        distanceToObstacleFront = laser_data->points()[laser_data->points().size()/2].x;
    }else{

    }

    switch (currentState)
    {

    case ParkingState::SEARCHING:
    {

        // turn of indicators
        state.indicatorLeft = false;
        state.indicatorRight = false;

         /***************************************************
         * drive straight along the middle of the right lane
         ***************************************************/

        //the desired state is such that phi=0 (straight driving) and y=0.2 (middle of right lane) with respect to the middle lane
        setSteeringAngles(-0.2, config().get<float>("searchingPhiFactor"), DrivingMode::FORWARD);

        state.targetSpeed = config().get<float>("velocitySearching", 1.0);




        // check if we detected a valid parking space
        bool spaceFound = checkForGap();
        logger.debug("searching for gap, found ")<<spaceFound;

        if(spaceFound){
            timeSpaceWasFound = lms::Time::now();
            currentState = ParkingState::STOPPING;
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

        state.indicatorLeft = false;
        state.indicatorRight = true;

        updatePositionFromHall();

        /***************************************************
        * calculate parameters for entering maneuver
        ***************************************************/
        const double lr = config().get<float>("wheelbase", 0.21); //Radstand
        const double l = config().get<float>("carLength", 0.32);  //Fahrzeuglänge
        const double b = config().get<float>("carWidth", 0.2); //Fahrzeugbreite
        const double delta_max = config().get("maxSteeringAngle", 24)*M_PI/180; //maximum steering angle
        const double k = config().get<float>("k", 0.03); //Sicherheitsabstand zur Ecke der 2. Box
        const double d = config().get<float>("d", 0.05); //Sicherheitsabstand zur 1. Box im eingeparkten Zustand
        const double y0 = config().get<float>("y0_worstCase", 0.24); //seitlicher Abstand zur zweiten Box

        //TODO delta_max + 1*pi/180;
        const double r = lr/2*tan(M_PI/2 - (delta_max-1*M_PI/180)); //Radius des Wendekreises (bezogen auf den Fahrzeugmittelpunkt) bei Volleinschlag beider Achsen
        const double R = sqrt(l*l/4 + (r+b/2)*(r+b/2)); //Radius den das äußerste Eck des Fahrzeugs bei volleingeschlagenen Rädern zurücklegt
        const double s = sqrt((R+k)*(R+k) - (parkingSpaceSize - d - l/2)*(parkingSpaceSize - d - l/2));
        const double alpha = acos((r-y0+s)/(2*r)); //Winkel (in rad) der 2 Kreisboegen, die zum einfahren genutzt werden
        const double x0 = d + l/2 + 2*r*sin(alpha) - parkingSpaceSize; //Abstand vom Ende der 2. Box zur Mitte des Fahrzeugs bei Lenkbeginn (Anfang erster Kreisbogen)
        const double x_begin_steering = config().get<float>("xDistanceCorrection",0.09) + endX + x0 - config().get<float>("distanceMidLidarX", 0.09);

        if (currentXPosition > x_begin_steering) //drive straight backwards
        {
            //setSteeringAngles(-0.2, config().get<float>("searchingPhiFactor"), DrivingMode::BACKWARDS);
            state.steering_front = 0.0;
            state.steering_rear = 0.0;

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
                logger.debug("driven arc")<<drivenArc<<" target: "<<alpha;
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
        state.indicatorLeft = false;
        state.indicatorRight = false;

        updatePositionFromHall();

        std::vector<float> correctingDistances = config().getArray<float>("correctingDistances");

        if (correctingCounter >= (int)correctingDistances.size()){
            currentState = ParkingState::FINISHED;
        }else{
            double phi_ist = car_yawAngle - yawAngleStartEntering;

            if (correctingCounter%2 == 0) //forward
            {
                state.targetSpeed = config().get<float>("velocityCorrecting", 0.5);
                setSteeringAngles(-0.29, 0.0, 0.0, 8.0*phi_ist, DrivingMode::FORWARD);

                logger.error("forward Correcting");
                //mit dem lidar den Frontabstand Messen
                float correctingDistance = correctingDistances.at(correctingCounter);
                if(validDistanceToObstacleFront){
                    correctingDistance = distanceToObstacleFront-0.25; //TODO
                }
                if (currentXPosition >= correctingDistances.at(correctingCounter))
                {
                    ++correctingCounter;
                    currentXPosition = 0.0;
                }

            }
            else //backwards
            {
                state.targetSpeed = -config().get<float>("velocityCorrecting", 0.5);
                setSteeringAngles(-0.29, 0.0, 0.0, 8.0*phi_ist, DrivingMode::BACKWARDS);

                logger.error("backwards Correcting");
                float correctingDistance = correctingDistances.at(correctingCounter);
                if(validDistanceToObstacleFront){
                    correctingDistance = 53-distanceToObstacleFront-5; //TODO
                }

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

        if (finishCounter > 10)
        {
            state.indicatorLeft = false;
            state.indicatorRight = false;
        }
        else
        {
            state.indicatorLeft = true;
            state.indicatorRight = true;
        }

        ++finishCounter;


        break;
    }
    case ParkingState::WORST_CASE_BACKWARDS: {
        // TODO

        break;
    }

    }


    car->putState(state);
    return true;
}

bool Parking::checkForGap()
{
    if(sensors->hasSensor("PARKINGLOT_" + std::to_string(0))){
        auto parking = sensors->sensor<sensor_utils::ParkingSensor>("PARKINGLOT_" + std::to_string(0));
        auto size = parking->size;

        if(size > config().get<float>("minParkingSpaceSize", 0.3) &&
           size < config().get<float>("maxParkingSpaceSize", 0.75)){
            logger.info("valid parking space found! ") << "size=" << size << ", at=" << parking->position;
            posXGap = parking->position;
            return true;
        }else{
            logger.info("invalid parking space found! ") << "size=" << size << ", at=" << parking->position;
        }
    }
    return false;
}

void Parking::updatePositionFromHall()
{
    if(sensors->hasSensor("HALL")) {
        auto hall = sensors->sensor<sensor_utils::Odometer>("HALL");
        auto dst = hall->distance.x();
        currentXPosition += dst;
    }
}

void Parking::fitLineToMiddleLane(double *oM, double *oB)
{
    //get points from middle lane
    street_environment::RoadLane middleLane = getService<local_course::LocalCourse>("LOCAL_COURSE_SERVICE")->getCourse();
    std::vector<double> iX;
    std::vector<double> iY;
    int lineFitStartPoint = config().get<int>("lineFitStartPoint", 2);
    int lineFitEndPoint = config().get<int>("lineFitEndPoint", 5);
    if(middleLane.points().size() <= lineFitEndPoint){
        logger.error("invalid lineFitEndPoint")<<lineFitEndPoint<<" ,middle has only "<<middleLane.points().size()<<" elements";
        lineFitEndPoint = ((int)middleLane.points().size()) -1;
    }
    for (int i=lineFitStartPoint; i<=lineFitEndPoint; ++i) //get points 1 to 4
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


