#pragma once
#include "vec.h"

struct Ray {
    Vec3 o, d;
    Vec3 at(float t) const { return o + d*t; }
};