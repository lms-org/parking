#ifndef PARKING_H
#define PARKING_H

#include <lms/module.h>
#include <cmath>
#include <mavlink/lms/data.h>
#include <mavlink/CC2016/mavlink.h>

class Parking : public lms::Module {
public:
    bool initialize();
    bool deinitialize();
    bool cycle();

    void findEdges();

    int cycleCounter;
    std::vector<double> distanceMeasurement;
    std::vector<double> xPosition;
    std::vector<double> edgePosition;
    std::vector<int> edgeType;
    uint numEdges;
    enum class ParkingState {SEARCHING, STOPPING, ENTERING, CORRECTING, FINISHED};
    ParkingState currentState;
    double ps_x_start, ps_x_end, y0, x0, drivenArcLength, parkingSpaceSize;
    bool firstCircleArc; //true: Fahrzeug befindet sich im ersten Kreisbogen

    lms::ReadDataChannel<Mavlink::Data> mavlinkChannel;
    double lastTimeStamp, currentXPosition, lastValidMeasurement;



};

#endif // PARKING_H
