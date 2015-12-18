#include "parking.h"
#include "phoenix_CC2016_service/phoenix_CC2016_service.h"


bool Parking::initialize() {    

    currentState = ParkingState::SEARCHING;
    firstCircleArc = true;
    drivenArcLength = 0;
    parkingSpaceSize = 0;

    mavlinkChannel = readChannel<Mavlink::Data>("MAVLINK_IN");
    currentXPosition = 0;
    lastTimeStamp = -1;
    lastValidMeasurement = 0.5;



    logger.debug("init") << "init";


    myfile.open ("data.csv");

    return true;
}

bool Parking::deinitialize() {
    distanceMeasurement.clear();
    xPosition.clear();
    return true;
}

bool Parking::cycle() {

    //logger.debug("cycle") << "cycle";


    /*if(getService<phoenix_CC2016_service::Phoenix_CC2016Service>("PHOENIX_SERVICE")->driveMode() != phoenix_CC2016_service::CCDriveMode::PARKING){
        //TODO remove parking car-control-state
        return true;
    }*/

    usleep(2000);

    switch (currentState) {

    case ParkingState::SEARCHING: {

        ++counter;      

        double velocity = 0; //konstate Testgeschwindigkeit, TODO
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

        for( const auto& msg : *mavlinkChannel )
        {
            if (msg.msgid == MAVLINK_MSG_ID_PROXIMITY && counter > 5) {  //counter > 1 nur für debugging, da extrem viele messwerte in den ersten paar frames auftauchen
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
                //myfile << currentXPosition << ", " << distance << std::endl;
            }
        }
        //std::cout << std::endl;

        //cut size of vectors
        if (distanceMeasurement.size() > 5000) {
            distanceMeasurement.erase(distanceMeasurement.begin(), distanceMeasurement.begin() + distanceMeasurement.size()-5000);
            xPosition.erase(xPosition.begin(), xPosition.begin() + xPosition.size()-5000); // has the same size as distanceMeasurement
        }



        /*if (counter > 1000) {
            convolution.assign(distanceMeasurement.size(), 0);
            convolve1D(&distanceMeasurement, &convolution, (int)distanceMeasurement.size(), &gaussWin, (int)gaussWin.size());
            for (int i = 0; i < convolution.size(); ++i) {
                myfile << xPosition[i] << "," << distanceMeasurement[i] << ","<< convolution[i]  << std::endl;
            }

            usleep(20000000);
        }*/


        //Parklueckenerkennung
        bool foundParkingSpace = Parking::parkingSpaceDetection(&xPosition, &distanceMeasurement, &ps_x_start, &ps_x_end);

        if (foundParkingSpace) {

            parkingSpaceSize = ps_x_end - ps_x_start;

            logger.debug("parking") << "parking space detected: ps_x_start=" << ps_x_start << "\t ps_x_end=" << ps_x_end << "\t size=" << parkingSpaceSize;

            if (parkingSpaceSize > 0.45 && parkingSpaceSize < 0.6) ;//currentState = ParkingState::STOPPING;
        }
        else {
            //logger.debug("parking") << "ps_x_start=" << ps_x_start << " \t ps_x_end=" << ps_x_end;
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


bool Parking::parkingSpaceDetection(std::vector<double> *x_pos, std::vector<double> *dst, double *x_start, double *x_end) {

    /* algorithm:
     * 1. moving average with window size s
     * 2. calculate gradients
     * 3. set all values which are too close at 0 to 0 (with threshold gradient_thresh)
     * 4. search for connected regions and extract their middle point as an edge
     * -> all these operations are done in a single for loop
     */

    int s = 3; //smoothing (ungerade Zahl >= 3)
    double gradient_thresh = 0.17; //Threshold für die Erkennung von Bergen im Gradientenverlauf (dort sind die Kanten)

    *x_start = 0;
    *x_end = 0;

    if (dst->size() < 5) return false;

    int side_step = (s-1)/2;
    int starter = -1;
    int edge_type = 0;

    double grad=0, d0=0, d1=0, sz=0;

    //remove outliers and limit distance
    /*double maxDst = config().get("maxLidarDistance", 0.5);
    double lastValid = 0;
    for (unsigned int i=0; i < dst->size()-1; ++i) {
        if (dst->at(i) > 0.5) dst->at(i) = 0.5; //cut big distances

        if (dst->at(i+1) < 0.05) { //invalid too small measurements
            //dst->at(i) = lastValid;
            //std::cout << dst->at(i) << std::endl;
            dst->at(i+1) = dst->at(i);
        }
        else {
            //lastValid = dst->at(i);
        }
        //if (dst->at(i) < 0.5 && dst->at(i) > 0.05) std::cout << dst->at(i) << " ";
    }*/
    //std::cout << std::endl;

    //main worker loop
    for (unsigned int i=side_step; i < dst->size()-side_step-1; ++i) {

        //smoothing
        /*for (unsigned int k=i-side_step; k <= i + side_step; ++k) {
            d0 += dst->at(k);
            d1 += dst->at(k+1);
        }
        d0 /= s;
        d1 /= s;*/
        d0 = dst->at(i);
        d1 = dst->at(i+1);


        //gradient
        //grad = (d1-d0)/(x_pos->at(i+1) - x_pos->at(i));
        grad = d1-d0;
        if (fabs(grad) > gradient_thresh) {
            //std::cout << "grad: " << grad << std::endl;
        }



        //if ((dst->at(i+1) - dst->at(i)) > 0 )std::cout << " " << (dst->at(i+1) - dst->at(i));

        if (fabs(grad) > gradient_thresh) {
            if (starter == -1) {
                starter = i;
                sz = x_pos->at(i+1) - x_pos->at(i);
                if (grad > 0) edge_type = 1; //positive Kante: Beginn einer Lücke
                else edge_type = -1; // negative Kante: Ende einer Lücke
            }
            else {
                sz += x_pos->at(i+1) - x_pos->at(i);
            }
        }
        else {
            if (starter > -1) {
                if (edge_type == 1) {
                    *x_start = x_pos->at(starter) + sz/(i-starter);
                }
                else {
                    *x_end = x_pos->at(starter) + sz/(i-starter);
                }
                starter = -1;
            }
        }
    }

    if ((*x_start != 0 && *x_end != 0) && (*x_start < *x_end)) return true;

    return false;

}

void Parking::convolve1D(std::vector<double> *in, std::vector<double> *out, int dataSize, std::vector<double> *kernel, int kernelSize)
{
    int i, j, k;

    // check validity of params
    //if(!in || !out || !kernel) return false;
    //if(dataSize <=0 || kernelSize <= 0) return false;

    // start convolution from out[kernelSize-1] to out[dataSize-1] (last)
    for(i = kernelSize-1; i < dataSize; ++i)
    {
        out->at(i) = 0;                             // init to 0 before accumulate

        for(j = i, k = 0; k < kernelSize; --j, ++k)
            //out[i] += in[j] * kernel[k];
            out->at(i) += in->at(j) * kernel->at(k);
    }

    // convolution from out[0] to out[kernelSize-2]
    for(i = 0; i < kernelSize - 1; ++i)
    {
        out->at(i) = 0;                             // init to 0 before sum

        for(j = i, k = 0; j >= 0; --j, ++k)
            //out[i] += in[j] * kernel[k];
            out->at(i) += in->at(j) * kernel->at(k);
    }

}

double Parking::fakeLaserDistanceSensor() {

    if (counter > 20 && counter <= 40){
        return 60 + rand()%3;
    }
    else if (counter > 60 && counter <= 90){
        return 50 + rand()%3;
    }
    else {
        return 20 + rand()%3;
    }

}
