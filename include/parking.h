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
#include <sensor_utils/sensor.h>
#include <sensor_utils/odometer.h>


class Parking : public lms::Module {
public:
    bool initialize();
    bool deinitialize();
    bool cycle();

    void findEdges();
    void fitLine(std::vector<double> *iX, std::vector<double> *iY, double *oM, double *oB);
    bool findValidParkingSpace(double sizeMin, double sizeMax);
    void setSteeringAngles(double y_soll, double phi_soll, int drivingMode);
    void setSteeringAngles(double y_soll, double phi_soll, double y_ist, double phi_ist, int drivingMode);

    lms::ReadDataChannel<sensor_utils::SensorContainer> sensors;

    int cycleCounter;
    std::vector<double> distanceMeasurement;
    std::vector<double> xPosition;
    std::vector<double> edgePosition;
    std::vector<int> edgeType;
    uint numEdges;
    enum DrivingMode {FORWARD, BACKWARDS};
    enum ParkingState {SEARCHING, STOPPING, ENTERING, BACKING_UP, CORRECTING, FINISHED};
    ParkingState currentState;
    double ps_x_start, ps_x_end, y0, x0, parkingSpaceSize;
    bool firstCircleArc; //true: Fahrzeug befindet sich im ersten Kreisbogen

    lms::ReadDataChannel<Mavlink::Data> mavlinkChannel;
    double lastTimeStamp, lastImuTimeStamp, currentXPosition, lastValidMeasurement;
    lms::Time tSpaceWasFound;

    std::ofstream myfile;
    double startX, endX, y0_dynamic, ind_end;

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
