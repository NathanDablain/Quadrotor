// Coordinate conversions taken from https://en.wikipedia.org/wiki/Geographic_coordinate_conversion

#include "Coordinate_Frames.h"

Vec3 NED2Body(Vec3 NED_vec, Vec q){
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

Vec3 Body2NED(Vec3 Body_vec, Vec q){
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

Vec3 lla2ecef(Vec3 lla){
    // Equator radius in (m)
    const double a_c = 6378137.0;
    // Polar radius in (m)
    const double b_c = 6356752.314;
    // Earth eccentricity
    const double e_c = sqrt((pow(a_c,2) - pow(b_c, 2))/pow(a_c,2));
    
    double N_phi = a_c/sqrt(1.0 - (pow(e_c, 2)*pow(sin(lla.data[0]), 2)));

    Vec3 P_ecef = {(N_phi + lla.data[2])*cos(lla.data[0])*cos(lla.data[1]),
                   (N_phi + lla.data[2])*cos(lla.data[0])*sin(lla.data[1]),
                   ((1.0 - pow(e_c, 2))*N_phi + lla.data[2])*sin(lla.data[0])};
    
    return P_ecef;
}

Vec3 ecef2lla(Vec3 P_ecef){
    Vec3 lla;
    // Equator radius in (m)
    const double a = 6378137.0;
    // Polar radius in (m)
    const double b = 6356752.314;
    // Earth eccentricity
    const double e = sqrt((pow(a,2) - pow(b, 2))/pow(a,2));

    double X = P_ecef.data[0];
    double Y = P_ecef.data[1];
    double Z = P_ecef.data[2];
    double e_r2 = (pow(a, 2)-pow(b, 2))/pow(b, 2);
    double p = sqrt(pow(X, 2) + pow(Y, 2));
    double F = 54.0*pow(b, 2)*pow(Z, 2);
    double G = pow(p, 2) + (1.0 - pow(e, 2))*pow(Z, 2) - pow(e, 2)*(pow(a, 2) - pow(b, 2));
    double c = (pow(e, 4)*F*pow(p, 2))/pow(G, 3);
    double s = pow(1 + c + sqrt(pow(c, 2) + 2.0*c), 1.0/3.0);
    double k = s + 1.0 + (1.0/s);
    double P = F/(3*pow(k, 2)*pow(G, 2));
    double Q = sqrt(1 + (2.0*pow(e, 4)*P));
    double r0_p1 = (-P*pow(e, 2)*p)/(1+Q);
    double r0_p2 = 0.5*pow(a, 2)*(1 + (1/Q));
    double r0_p3 = (P*(1 - pow(e, 2))*pow(Z, 2))/(Q*(1.0 + Q));
    double r0_p4 = 0.5*P*pow(p, 2);
    double r0 = r0_p1 + sqrt(r0_p2 - r0_p3 - r0_p4);
    double U = sqrt(pow(p - (pow(e, 2)*r0), 2) + pow(Z, 2));
    double V = sqrt(pow(p - (pow(e, 2)*r0), 2) + (1 - pow(e, 2))*pow(Z, 2));
    double z0 = (pow(b, 2)*Z)/(a*V);
    
    lla.data = {atan((Z + e_r2*z0)/p),
                atan2(P_ecef.data[1], P_ecef.data[0]),
                U*(1 - (pow(b, 2)/(a*V)))};
    return lla;
}

Mat3 NED2ecefmat(Vec3 lla){
    // Here lla is of the reference position
    double c_lat = cos(lla.data[0]);
    double s_lat = sin(lla.data[0]);
    double c_lon = cos(lla.data[1]);
    double s_lon = sin(lla.data[1]);

    Mat3 Rotation_matrix;
    Rotation_matrix.data[0] = { c_lon, -s_lat*s_lon, c_lat*s_lon};
    Rotation_matrix.data[1] = {-s_lon, -s_lat*c_lon, c_lat*c_lon};
    Rotation_matrix.data[2] = {   0.0,        c_lat,       s_lat};

    return Rotation_matrix;
}

Mat3 ecef2NEDmat(Vec3 lla){
    // Here lla is of the reference position
    Mat3 Rotation_matrix;
    Rotation_matrix = NED2ecefmat(lla);

    return Rotation_matrix.inv();
}

Vec3 NED2ecef(Vec3 P_NED, Vec3 P_ref_ecef, Vec3 lla){
    // Rotates a vector from local NED frame to ecef given a reference position in ecef and lla
    Vec3 P_ecef = NED2ecefmat(lla)*P_NED + P_ref_ecef;
    return P_ecef;
}

Vec3 ecef2NED(Vec3 P_ecef, Vec3 P_ref_ecef, Vec3 lla){
    //  Rotates a vector from ecef to NED coordinate frame given a reference position in ecef and lla
    Vec3 P_NED = ecef2NEDmat(lla)*(P_ecef - P_ref_ecef);
    return P_NED;
}