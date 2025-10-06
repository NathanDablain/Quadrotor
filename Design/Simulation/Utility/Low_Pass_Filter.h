#pragma once

class Low_Pass_Filter{
    private:
        double x;
        double Bandwidth;
        double Beta;
    public:
        Low_Pass_Filter();
        Low_Pass_Filter(double bandwidth, double initial_value);
        void Initialize(double bandwidth, double initial_value);
        double Update(double x_in, double d_t, bool passthrough);
        void Change_Bandwidth(double new_bandwidth);
};