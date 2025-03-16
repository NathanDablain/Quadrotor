#pragma once

// Coordinate conversions taken from https://en.wikipedia.org/wiki/Geographic_coordinate_conversion

#include <cstdlib>
#include <cmath>
#include "Linear_Algebra.hpp"

Vec3 NED2Body(Vec3 &NED_vec, Vec q);

Vec3 Body2NED(Vec3 &Body_vec, Vec q);

Vec3 lla2ecef(Vec3 lla);

Vec3 ecef2lla(Vec3 P_ecef);

Mat3 NED2ecefmat(Vec3 lla);

Mat3 ecef2NEDmat(Vec3 lla);

Vec3 NED2ecef(Vec3 P_NED, Vec3 P_ref_ecef, Vec3 lla);

Vec3 ecef2NED(Vec3 P_ecef, Vec3 P_ref_ecef, Vec3 lla);