#pragma once

#include "Linear_Algebra.hpp"

Vec3 NED2Body(Vec3 &NED_vec, Vec q){
    // Rotates a vector from the drone NED frame to the drone's body frame
    // It is assumed that the quaternion, q, rotates from the NED to body frame
    // u = q(2:4);
    // t = 2*cross(u, g);
    // g_rot = g + q(1)*t + cross(u, t);
    Vec3 u_loc = {q.data[1], q.data[2], q.data[3]};
    Vec3 t_loc = u_loc.cross(NED_vec)*2.0;
    Vec3 Body_vec = NED_vec + t_loc*q.data[0] + u_loc.cross(t_loc);
    return Body_vec;
}

Vec3 Body2NED(Vec3 &Body_vec, Vec q){
    // Rotates a vector from the drone's body frame to the drone NED frame
    // It is assumed that the quaternion, q, rotates from the NED to body frame
    // u = -1*q(2:4);
    // t = 2*cross(u, v);
    // v_rot = v + q(1)*t + cross(u, t);
    Vec3 u_loc = {-q.data[1], -q.data[2], -q.data[3]};
    Vec3 t_loc = u_loc.cross(Body_vec)*2.0;
    Vec3 NED_vec = Body_vec + t_loc*q.data[0] + u_loc.cross(t_loc);
    return NED_vec;
}