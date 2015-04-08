#ifndef _ADDED4VC10_H
#define _ADDED4VC10_H

#include <math.h>

inline double round(double number) {
    return number < 0.0 ? ceil(number - 0.5) : floor(number + 0.5);
}

#endif
