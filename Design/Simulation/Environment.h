#pragma once

#include "vector"

class Environment{
    private:
        double gravity;
        double lla[3];
        double height;
        double pressure;
        double earth_semi_major_axis = 6378137.0;
        double earth_semi_minor_axis = 6356752.314;
        
    public:
        Environment(double LLA[3]);
        void Apply_forces_moments(vector<double> &Forces_NED, vector<double> &Moments_NED);
};