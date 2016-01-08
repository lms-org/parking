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

    void findEdges(std::vector<double> *dst, std::vector<double> *x, std::vector<double> *edgePosition, std::vector<double> *edgeType, int *numEdges);

    int cycleCounter;
    std::vector<double> distanceMeasurement;
    std::vector<double> xPosition;
    std::vector<double> edgePosition;
    std::vector<double> edgeType;
    enum class ParkingState {SEARCHING, STOPPING, ENTERING, CORRECTING, FINISHED};
    ParkingState currentState;
    double ps_x_start, ps_x_end, y0, x0, drivenArcLength, parkingSpaceSize;
    bool firstCircleArc; //true: Fahrzeug befindet sich im ersten Kreisbogen

    lms::ReadDataChannel<Mavlink::Data> mavlinkChannel;
    double lastTimeStamp, currentXPosition, lastValidMeasurement;



};

#endif // PARKING_H
