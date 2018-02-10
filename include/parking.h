#ifndef PARKING_H
#define PARKING_H

#include <lms/module.h>
#include <cmath>
#include <mavlink/lms/data.h>
#include <mavlink/CC2016/mavlink.h>
#include <fstream>
#include "street_environment/car.h"
#include "local_course/local_course.h"
#include "street_environment/road.h"
#include "street_environment/street_environment.h"
#include <sensor_utils/sensor.h>
#include <sensor_utils/odometer.h>
#include <sensor_utils/parking_sensor.h>
#include <lms/math/point_cloud.h>
#include <float.h>


class Parking : public lms::Module {
public:
    enum DrivingMode {FORWARD, BACKWARDS};
    enum ParkingState {SEARCHING, STOPPING, ENTERING, CORRECTING, FINISHED, WORST_CASE_BACKWARDS, PULLOUT};
    enum PullOutState {PULLBACK, TURN_LEFT, MOVE_STRAIGHT,  TURN_RIGHT, BACK_ON_TRACK};

    struct ParkingStateContainer{
        ParkingState state;
        DrivingMode driveMode;
    };

public:
    bool initialize();
    bool deinitialize();
    bool cycle();

    bool checkForGap();
    void updatePositionFromHall();

    void fitLineToMiddleLane(double *oM, double *oB);
    void setSteeringAngles(double y_soll, double phi_soll, int drivingMode);
    void setSteeringAngles(double y_soll, double phi_soll, double y_ist, double phi_ist, int drivingMode);
    double getDistanceToMiddleLane();

    lms::ReadDataChannel<sensor_utils::SensorContainer> sensors;
    lms::WriteDataChannel<street_environment::CarCommand> car;
    lms::WriteDataChannel<lms::math::PointCloud2f> laser_data;
    PullOutState pulloutstate;
    ParkingState currentState;
    bool firstCircleArc;
    double lastTimeStamp, lastImuTimeStamp, currentXPosition;
    double y0_dynamic, ind_end, endX;
    lms::Time timeSpaceWasFound;
    bool straightMove;
    bool yawAngleSet;
    int finishCounter;

    street_environment::CarCommand::State state;
    double car_yawAngle, car_xPosition;
    double yawAngleStartEntering;
    int correctingCounter;

    double posXGap;
    double parkingSpaceSize;

    //hack
    int m_cycleCounter;
    double move_straight_start_pos;
    double finished_pos;
    float initial_phi;

    double oldDeltaPhi;

};

#endif // PARKING_H
