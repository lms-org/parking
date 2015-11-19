#ifndef PARKING_H
#define PARKING_H

#include <lms/datamanager.h>
#include <lms/module.h>
#include <cmath>

class Parking : public lms::Module {
public:
    bool initialize();
    bool deinitialize();
    bool cycle();

    bool parkingSpaceDetection(std::vector<double> *x_pos, std::vector<double> *dst, double *x_start, double *x_end);
    double fakeLaserDistanceSensor();

    int counter = 0;
    std::vector<double> measured_distance;
    std::vector<double> x_position;
    enum class ParkingState {SEARCHING, STOPPING, ENTERING, CORRECTING, FINISHED};
    ParkingState currentState;
    double ps_x_start, ps_x_end, y0, x0;

};

#endif // PARKING_H
