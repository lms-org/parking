#include "parking.h"

extern "C" {
void* getInstance () {
    return new Parking();
}
}
