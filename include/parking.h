#ifndef PARKING_H
#define PARKING_H

#include <lms/module.h>
#include <cmath>
#include <mavlink/lms/data.h>
#include <mavlink/CC2016/mavlink.h>
#include <fstream>
#include "sensor_utils/car.h"
#include "local_course/local_course.h"
#include "street_environment/road.h"
#include "street_environment/street_environment.h"


class Parking : public lms::Module {
public:
    bool initialize();
    bool deinitialize();
    bool cycle();

    void findEdges();
    void fitLine(std::vector<double> *iX, std::vector<double> *iY, double *oM, double *oB);

    int cycleCounter;
    std::vector<double> distanceMeasurement;
    std::vector<double> xPosition;
    std::vector<double> edgePosition;
    std::vector<int> edgeType;
    uint numEdges;
    enum class ParkingState {SEARCHING, STOPPING, ENTERING, CORRECTING, FINISHED};
    ParkingState currentState;
    double ps_x_start, ps_x_end, y0, x0, parkingSpaceSize;
    bool firstCircleArc; //true: Fahrzeug befindet sich im ersten Kreisbogen

    lms::ReadDataChannel<Mavlink::Data> mavlinkChannel;
    double lastTimeStamp, lastImuTimeStamp, currentXPosition, lastValidMeasurement;

    std::ofstream myfile;
    double startX, endX;

    lms::WriteDataChannel<sensor_utils::Car> car;
    sensor_utils::Car::State state;
    double car_yawAngle, car_velocity, car_xPosition;
    double yawAngleStartEntering;
    double velocity_temp;

    void updateYawAngle();
    void updateXPosition(bool adjustVectors);
    void updateVelocity();
};

#endif // PARKING_H
