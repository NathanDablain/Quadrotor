#pragma once

#include <array>
#include <vector>
#include <cstdint>

using namespace std;

class Vec3{
    public:
        array<double, 3> data {0};

        void operator=(const Vec3 &v1){
            data[0] = v1.data[0];
            data[1] = v1.data[1];
            data[2] = v1.data[2];
        }

        Vec3 operator+(const Vec3 &v1){
            Vec3 v2;
            for (uint8_t i = 0; i < 3; i++){
                v2.data[i] = data[i] + v1.data[i];
            }

            return v2;
        }

        Vec3 operator-(const Vec3 &v1){
            Vec3 v2;
            for (uint8_t i = 0; i < 3; i++){
                v2.data[i] = data[i] - v1.data[i];
            }

            return v2;
        }

        Vec3 operator*(const double c){
            Vec3 v2;
            for (uint8_t i = 0; i < 3; i++){
                v2.data[i] = data[i] * c;
            }

            return v2;
        }

        Vec3 operator/(const double c){
            Vec3 v2;
            for (uint8_t i = 0; i < 3; i++){
                v2.data[i] = data[i] / c;
            }

            return v2;
        }
        
        Vec3 cross(const Vec3 &v1){ // CROSS PRODUCT
            Vec3 v2;
            v2.data[0] = data[1]*v1.data[2] - data[2]*v1.data[1];
            v2.data[1] = data[2]*v1.data[0] - data[0]*v1.data[2];
            v2.data[2] = data[0]*v1.data[1] - data[1]*v1.data[0];

            return v2;
        }

        double dot(const Vec3 &v1){ // DOT PRODUCT
            return data[0]*v1.data[0] + data[1]*v1.data[1] + data[2]*v1.data[2];
        }
};

class Mat3{
    public:
        array<array<double, 3>, 3> data {0};

        void operator=(const Mat3 &m1){
            for (uint8_t i = 0; i < 3; i++){
                for (uint8_t j = 0 ; j < 3; j++){
                    data[i][j] = m1.data[i][j];
                }
            }
        }

        Mat3 operator+(const Mat3 &m1){
            Mat3 m2;
            for (uint8_t i = 0; i < 3; i++){
                for (uint8_t j = 0 ; j < 3; j++){
                    m2.data[i][j] = data[i][j] + m1.data[i][j];
                }
            }
        
            return m2;
        };

        Mat3 operator-(const Mat3 &m1){
            Mat3 m2;
            for (uint8_t i = 0; i < 3; i++){
                for (uint8_t j = 0 ; j < 3; j++){
                    m2.data[i][j] = data[i][j] - m1.data[i][j];
                }
            }
        
            return m2;
        };

        Mat3 operator*(const Mat3 &m1){
            Mat3 m2;
            for (uint8_t i = 0; i < 3; i++){
                for (uint8_t j = 0; j < 3; j++){
                    for (uint8_t k = 0; k < 3; k++){
                        m2.data[i][j] += data[i][k]*m1.data[k][j];
                    }
                }
            }

            return m2;
        }

        Vec3 operator*(const Vec3 &v1){
            Vec3 v2;
            for (uint8_t i = 0; i < 3; i++){
                for (uint8_t j = 0; j < 3; j++){
                    v2.data[i] += data[i][j]*v1.data[j];
                }
            }

            return v2;
        }

        Mat3 inv(){
            // Computes inverse of data, -> adj(data)/det(data)
            Mat3 inv_m;
            // Check the det before calling this method
            double det = det3();
            array<double, 4> m2 = {data[1][1], data[1][2], data[2][1], data[2][2]};
            inv_m.data[0][0] = det2(m2)/det;
            m2 = {data[0][1], data[0][2], data[2][1], data[2][2]};
            inv_m.data[0][1] = -det2(m2)/det;
            m2 = {data[0][1], data[0][2], data[1][1], data[1][2]};
            inv_m.data[0][2] = det2(m2)/det;
            m2 = {data[1][0], data[1][2], data[2][0], data[2][2]};
            inv_m.data[1][0] = -det2(m2)/det;
            m2 = {data[0][0], data[0][2], data[2][0], data[2][2]};
            inv_m.data[1][1] = det2(m2)/det;
            m2 = {data[0][0], data[0][2], data[1][0], data[1][2]};
            inv_m.data[1][2] = -det2(m2)/det;
            m2 = {data[1][0], data[1][1], data[2][0], data[2][1]};
            inv_m.data[2][0] = det2(m2)/det;
            m2 = {data[0][0], data[0][1], data[2][0], data[2][1]};
            inv_m.data[2][1] = -det2(m2)/det;
            m2 = {data[0][0], data[0][1], data[1][0], data[1][1]};
            inv_m.data[2][2] = det2(m2)/det;

            return inv_m;
        }

        double det2(array<double, 4> &m){
            return m[0]*m[3] - m[1]*m[2];
        }

        double det3(){
            array<double, 4> m1 = {data[1][1], data[1][2], data[2][1], data[2][2]};
            array<double, 4> m2 = {data[1][0], data[1][2], data[2][0], data[2][2]};
            array<double, 4> m3 = {data[1][0], data[1][1], data[2][0], data[2][1]};
            return data[0][0]*det2(m1) - data[0][1]*det2(m2) + data[0][2]*det2(m3);
        }
};

class Vec{
    public:
        vector<double> data;

        void operator=(const vector<double> &v1){
            data.resize(v1.size(), 0.0);
            for (size_t i = 0; i < v1.size(); i++){
                data[i] = v1[i];
            }
        }

        Vec operator+(const Vec &v1){
            Vec v2;
            v2.data.resize(v1.data.size(), 0.0);
            for (size_t i = 0; i < data.size(); i++){
                v2.data[i] = data[i] + v1.data[i];
            }

            return v2;
        }

        Vec operator-(const Vec &v1){
            Vec v2;
            v2.data.resize(v1.data.size(), 0.0);
            for (size_t i = 0; i < data.size(); i++){
                v2.data[i] = data[i] - v1.data[i];
            }

            return v2;
        }

        Vec operator*(const double c){
            Vec v2;
            v2.data.resize(data.size(), 0.0);
            for (size_t i = 0; i < data.size(); i++){
                v2.data[i] = data[i] * c;
            }

            return v2;
        }

        Vec operator/(const double c){
            Vec v2;
            v2.data.resize(data.size(), 0.0);
            for (size_t i = 0; i < data.size(); i++){
                v2.data[i] = data[i] / c;
            }

            return v2;
        }

        double dot(const Vec &v1){ // DOT PRODUCT
            double result = 0.0;
            for (size_t i = 0; i < data.size(); i++){
                result += data[i] * v1.data[i];
            }

            return result;
        }
};

class Mat{
    public:
        vector<vector<double>> data;
        Mat(size_t height, size_t width){
            data.resize(height, vector<double>(width, 0.0));
        }

        Mat operator+(const Mat &m1){
            Mat m2(data.size(), data[0].size());
            for (size_t i = 0; i < data.size(); i++){
                for (uint8_t j = 0 ; j < data[0].size(); j++){
                    m2.data[i][j] = data[i][j] + m1.data[i][j];
                }
            }
        
            return m2;
        };

        Mat operator-(const Mat &m1){
            Mat m2(data.size(), data[0].size());
            for (size_t i = 0; i < data.size(); i++){
                for (uint8_t j = 0 ; j < data[0].size(); j++){
                    m2.data[i][j] = data[i][j] - m1.data[i][j];
                }
            }
        
            return m2;
        };

        Mat operator*(const Mat &m1){
            Mat m2(data.size(), m1.data[0].size());
            for (size_t i = 0; i < data.size(); i++){
                for (size_t j = 0; j < m1.data[0].size(); j++){
                    for (size_t k = 0; k < data[0].size(); k++){
                        m2.data[i][j] += data[i][k]*m1.data[k][j];
                    }
                }
            }

            return m2;
        }

        Vec operator*(const Vec &v1){
            Vec v2;
            v2.data.resize(data.size(), 0.0);
            for (size_t i = 0; i < data.size(); i++){
                for (size_t j = 0; j < data[0].size(); j++){
                    v2.data[i] += data[i][j]*v1.data[j];
                }
            }

            return v2;
        }

};
