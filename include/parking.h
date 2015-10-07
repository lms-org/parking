#ifndef PARKING_H
#define PARKING_H

#include <lms/datamanager.h>
#include <lms/module.h>

class Parking : public lms::Module {
public:
    bool initialize();
    bool deinitialize();
    bool cycle();
};

#endif // PARKING_H
