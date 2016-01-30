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

    void updateYawAngle();
    void updateXPosition(bool adjustVectors);
    void updateXPositionFromHall();
    void updateVelocity();

    void findEdges();
    void fitLineToMiddleLane(double *oM, double *oB);
    bool findValidParkingSpace(double sizeMin, double sizeMax);
    void setSteeringAngles(double y_soll, double phi_soll, int drivingMode);
    void setSteeringAngles(double y_soll, double phi_soll, double y_ist, double phi_ist, int drivingMode);
    double getDistanceToMiddleLane();

    lms::ReadDataChannel<sensor_utils::SensorContainer> sensors;
    lms::WriteDataChannel<sensor_utils::Car> car;
    lms::ReadDataChannel<Mavlink::Data> mavlinkChannel;

    int cycleCounter;
    std::vector<double> distanceMeasurement;
    std::vector<double> xPosition;
    std::vector<double> edgePosition;
    std::vector<int> edgeType;
    uint numEdges;
    enum DrivingMode {FORWARD, BACKWARDS};
    enum ParkingState {SEARCHING, STOPPING, ENTERING, CORRECTING, CORRECTING2, FINISHED};
    ParkingState currentState;
    bool firstCircleArc;
    double parkingSpaceSize, lastTimeStamp, lastImuTimeStamp, currentXPosition, lastValidMeasurement;
    double startX, endX, y0_dynamic, ind_end;
    lms::Time timeSpaceWasFound;
    bool logThings;
    bool straightMove;

    sensor_utils::Car::State state;
    double car_yawAngle, car_velocity, car_xPosition;
    double yawAngleStartEntering;

    std::ofstream myfile;
    int fileCounter;
    int lidarCount = 0;

};

#endif // PARKING_H
