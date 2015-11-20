#include "parking.h"


bool Parking::initialize() {    

    currentState = ParkingState::SEARCHING;
    firstCircleArc = true;
    drivenArcLength = 0;
    parkingSpaceSize = 0;

    return true;
}

bool Parking::deinitialize() {
    measured_distance.clear();
    x_position.clear();
    return true;
}

bool Parking::cycle() {

    switch (currentState) {

    case ParkingState::SEARCHING: {

        ++counter;

        // Fake Messwerte holen
        double dst = Parking::fakeLaserDistanceSensor();

        measured_distance.push_back(dst);
        x_position.push_back(0.01*counter); //simulierte Fahrt mit 0.01m Inkrement pro Zyklus (entspricht 1m/s bei 100Hz)

        //Parklueckenerkennung
        bool foundParkingSpace = Parking::parkingSpaceDetection(&x_position, &measured_distance, &ps_x_start, &ps_x_end);

        if (foundParkingSpace) {

            parkingSpaceSize = ps_x_end - ps_x_start;

            logger.debug("parking") << "parking space detected: ps_x_start=" << ps_x_start << " \t ps_x_end=" << ps_x_end << "size=" << parkingSpaceSize;

            if (parkingSpaceSize > 0.45 && parkingSpaceSize < 0.6) currentState = ParkingState::STOPPING;
        }
        else {
            logger.debug("parking") << "ps_x_start=" << ps_x_start << " \t ps_x_end=" << ps_x_end;
        }
    }

    case ParkingState::STOPPING: {

        // if (car has stopped) currentState = ParkingState::ENTERING;

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
        double y0 = 0;  //distance from second box to right side of car
        double lr = 0.21; //Radstand
        double l = 0.3; //Fahrzeuglänge
        double b = 0.2; //Fahrzeugbreite
        double delta_max = 32*M_PI/180; //maximum steering angle
        double r = lr/2*tan(M_PI/2 - delta_max); //Radius des Wendekreises bei Volleinschlag beider Achsen

        double k = 0.05;
        double d = 0.03;

        double R = sqrt(l*l/4 + (r+b/2)*(r+b/2));
        double s = sqrt((R+k)*(R+k) - (parkingSpaceSize - d - l/2)*(parkingSpaceSize - d - l/2));
        double alpha = acos((r-y0+s)/(2*r)); //Winkel (in rad) der 2 Kreisboegen, die zum einfahren genutzt werden
        double x0 = d + l/2 + 2*r*sin(alpha) - parkingSpaceSize; //Abstand vom Ende der 2. Box zur Mitte des Fahrzeugs bei Lenkbeginn (Anfang erster Kreisbogen)

        double d_mid2cam = lr/2; //Abstand von Fahrzeugmitte zur Kamera
        double x_begin_steering = x0 - d_mid2cam;

        double x_now = 0; //Momentante x-Position entlang der Straße (ausgehend vom Parkbeginn x=0)
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
    double gradient_thresh = 200; //Threshold für die Erkennung von Bergen im Gradientenverlauf (dort sind die Kanten)

    *x_start = 0;
    *x_end = 0;

    if (dst->size() < 5) return false;

    int side_step = (s-1)/2;
    int starter = -1;
    int edge_type = 0;

    double grad=0, d0=0, d1=0, sz=0;

    //remove outliers
    for (unsigned int i=0; i < dst->size()-1; ++i) {
        if (dst->at(i) < 7) dst->at(i) = dst->at(i+1); //works if the outlier is only a single measurement
    }

    //main worker loop
    for (unsigned int i=side_step; i < dst->size()-side_step-1; ++i) {

        //smoothing
        for (unsigned int k=i-side_step; k <= i + side_step; ++k) {
            d0 += dst->at(k);
            d1 += dst->at(k+1);
        }
        d0 /= s;
        d1 /= s;

        //gradient
        grad = (d1-d0)/(x_pos->at(i+1) - x_pos->at(i));

        if (counter == 50) std::cout << " " << grad;

        if (abs(grad) > gradient_thresh) {
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
