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

    bool parkingSpaceDetection(std::vector<double> *x_pos, std::vector<double> *dst, double *x_start, double *x_end);
    void convolve1D(std::vector<double> *in, std::vector<double> *out, int dataSize, std::vector<double> *kernel, int kernelSize);
    double fakeLaserDistanceSensor();

    int counter = 0;
    std::vector<double> distanceMeasurement;
    std::vector<double> xPosition;
    std::vector<double> convolution;
    //std::vector<double> gaussWin {0.6627, 0, -0.6627}; //3
    std::vector<double> gaussWin {0.1071, 0.2302, 0.3255, 0.2555, 0, -0.2555, -0.3255, -0.2302, -0.1071}; //9, gradient threshold 0.11
    //std::vector<double> gaussWin {0.0380,    0.0607 ,   0.0890 ,   0.1193  ,  0.1452 ,   0.1583   , 0.1511 ,   0.1196 ,   0.0663 ,  0  , -0.0663  , -0.1196  , -0.1511  , -0.1583,  -0.1452 ,  -0.1193  , -0.0890  , -0.0607 ,  -0.0380};
    enum class ParkingState {SEARCHING, STOPPING, ENTERING, CORRECTING, FINISHED};
    ParkingState currentState;
    double ps_x_start, ps_x_end, y0, x0, drivenArcLength, parkingSpaceSize;
    bool firstCircleArc; //true: Fahrzeug befindet sich im ersten Kreisbogen

    lms::ReadDataChannel<Mavlink::Data> mavlinkChannel;
    double lastTimeStamp, currentXPosition, lastValidMeasurement;

    //debug
    double maxGrad = 0;
    std::ofstream myfile;

};

#endif // PARKING_H
