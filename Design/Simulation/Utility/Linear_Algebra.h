#pragma once

#include <cstdint>
#include <array>
#include <vector>
#include <cstdint>

using namespace std;

struct Vec3{
    array<double, 3> data {0.0};

    void operator=(Vec3 const& v1);

    Vec3 operator+(Vec3 const& v1);

    Vec3 operator-(Vec3 const& v1);

    Vec3 operator*(double const& c);

    Vec3 operator/(double const& c);
    
    Vec3 cross(Vec3 const& v1);

    double dot(Vec3 const& v1);
};

struct Mat3{
    array<array<double, 3>, 3> data {0.0};

    void operator=(Mat3 m1);

    Mat3 operator+(Mat3 m1);

    Mat3 operator-(Mat3 m1);

    Mat3 operator*(Mat3 m1);

    Vec3 operator*(Vec3 v1);

    Mat3 inv();

    double det2(array<double, 4> m);

    double det3();
};

struct Vec{
    vector<double> data {0.0};

    void operator=(vector<double> v1);

    Vec operator+(Vec v1);

    Vec operator-(Vec v1);

    Vec operator*(double c);

    Vec operator/(double c);

    double dot(Vec v1);

    double magnitude();
};

struct Mat{
    vector<vector<double>> data;

    Mat(uint32_t height, uint32_t width);

    Mat operator+(Mat m1);

    Mat operator-(Mat m1);

    Mat operator*(Mat m1);

    Vec operator*(Vec v1);
};
