#include "Linear_Algebra.h"

using namespace std;

//--------Vec3 Methods----------------//
Vec3 Vec3::operator=(Vec3 const& v1){
    this->data[0] = v1.data[0];
    this->data[1] = v1.data[1];
    this->data[2] = v1.data[2];

    return *this;
}

Vec3 Vec3::operator+(Vec3 const& v1){
    //Vec3 v2;
    for (uint8_t i = 0; i < 3; i++){
        this->data[i] += v1.data[i];
    }

    return *this;
}

Vec3 Vec3::operator-(Vec3 const& v1){
    // Vec3 v2;
    for (uint8_t i = 0; i < 3; i++){
        this->data[i] -= v1.data[i];
    }

    return *this;
}

Vec3 Vec3::operator*(double const& c){
    // Vec3 v2;
    for (uint8_t i = 0; i < 3; i++){
        this->data[i] *= c;
    }

    return *this;
}

Vec3 Vec3::operator/(double const& c){
    // Vec3 v2;
    for (uint8_t i = 0; i < 3; i++){
        this->data[i] /= c;
    }

    return *this;
}

Vec3 Vec3::cross(Vec3 const& v1){ // CROSS PRODUCT
    Vec3 v2;
    v2.data[0] = data[1]*v1.data[2] - data[2]*v1.data[1];
    v2.data[1] = data[2]*v1.data[0] - data[0]*v1.data[2];
    v2.data[2] = data[0]*v1.data[1] - data[1]*v1.data[0];

    return v2;
}

double Vec3::dot(Vec3 const& v1){ // DOT PRODUCT
    return data[0]*v1.data[0] + data[1]*v1.data[1] + data[2]*v1.data[2];
}
//------------------------------------//
//--------Mat3 Methods----------------//
void Mat3::operator=(Mat3 m1){
    for (uint8_t i = 0; i < 3; i++){
        for (uint8_t j = 0 ; j < 3; j++){
            data[i][j] = m1.data[i][j];
        }
    }
}

Mat3 Mat3::operator+(Mat3 m1){
    Mat3 m2;
    for (uint8_t i = 0; i < 3; i++){
        for (uint8_t j = 0 ; j < 3; j++){
            m2.data[i][j] = data[i][j] + m1.data[i][j];
        }
    }

    return m2;
};

Mat3 Mat3::operator-(Mat3 m1){
    Mat3 m2;
    for (uint8_t i = 0; i < 3; i++){
        for (uint8_t j = 0 ; j < 3; j++){
            m2.data[i][j] = data[i][j] - m1.data[i][j];
        }
    }

    return m2;
};

Mat3 Mat3::operator*(Mat3 m1){
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

Vec3 Mat3::operator*(Vec3 v1){
    Vec3 v2;
    for (uint8_t i = 0; i < 3; i++){
        for (uint8_t j = 0; j < 3; j++){
            v2.data[i] += data[i][j]*v1.data[j];
        }
    }

    return v2;
}

Mat3 Mat3::inv(){
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

double Mat3::det2(array<double, 4> m){
    return m[0]*m[3] - m[1]*m[2];
}

double Mat3::det3(){
    array<double, 4> m1 = {data[1][1], data[1][2], data[2][1], data[2][2]};
    array<double, 4> m2 = {data[1][0], data[1][2], data[2][0], data[2][2]};
    array<double, 4> m3 = {data[1][0], data[1][1], data[2][0], data[2][1]};
    return data[0][0]*det2(m1) - data[0][1]*det2(m2) + data[0][2]*det2(m3);
}
//------------------------------------//
//--------Vec Methods----------------//
void Vec::operator=(vector<double> v1){
    data.resize(v1.size(), 0.0);
    for (uint32_t i = 0; i < v1.size(); i++){
        data[i] = v1[i];
    }
}

Vec Vec::operator+(Vec v1){
    Vec v2;
    v2.data.resize(v1.data.size(), 0.0);
    for (uint32_t i = 0; i < data.size(); i++){
        v2.data[i] = data[i] + v1.data[i];
    }

    return v2;
}

Vec Vec::operator-(Vec v1){
    Vec v2;
    v2.data.resize(v1.data.size(), 0.0);
    for (uint32_t i = 0; i < data.size(); i++){
        v2.data[i] = data[i] - v1.data[i];
    }

    return v2;
}

Vec Vec::operator*(double c){
    Vec v2;
    v2.data.resize(data.size(), 0.0);
    for (uint32_t i = 0; i < data.size(); i++){
        v2.data[i] = data[i] * c;
    }

    return v2;
}

Vec Vec::operator/(double c){
    Vec v2;
    v2.data.resize(data.size(), 0.0);
    for (uint32_t i = 0; i < data.size(); i++){
        v2.data[i] = data[i] / c;
    }

    return v2;
}

double Vec::dot(Vec v1){ // DOT PRODUCT
    double result = 0.0;
    for (uint32_t i = 0; i < data.size(); i++){
        result += data[i] * v1.data[i];
    }

    return result;
}

double Vec::magnitude(){
    double result = 0.0;
    for (uint32_t i = 0; i < data.size(); i++){
        result += data[i];
    }
    return result;
}
//------------------------------------//
//--------Mat Methods----------------//
Mat::Mat(uint32_t height, uint32_t width){
    data.resize(height, vector<double>(width, 0.0));
};

Mat Mat::operator+(Mat m1){
    Mat m2(data.size(), data[0].size());
    for (uint32_t i = 0; i < data.size(); i++){
        for (uint8_t j = 0 ; j < data[0].size(); j++){
            m2.data[i][j] = data[i][j] + m1.data[i][j];
        }
    }

    return m2;
};

Mat Mat::operator-(Mat m1){
    Mat m2(data.size(), data[0].size());
    for (uint32_t i = 0; i < data.size(); i++){
        for (uint8_t j = 0 ; j < data[0].size(); j++){
            m2.data[i][j] = data[i][j] - m1.data[i][j];
        }
    }

    return m2;
};

Mat Mat::operator*(Mat m1){
    Mat m2(data.size(), m1.data[0].size());
    for (uint32_t i = 0; i < data.size(); i++){
        for (uint32_t j = 0; j < m1.data[0].size(); j++){
            for (uint32_t k = 0; k < data[0].size(); k++){
                m2.data[i][j] += data[i][k]*m1.data[k][j];
            }
        }
    }

    return m2;
}

Vec Mat::operator*(Vec v1){
    Vec v2;
    v2.data.resize(data.size(), 0.0);
    for (uint32_t i = 0; i < data.size(); i++){
        for (uint32_t j = 0; j < data[0].size(); j++){
            v2.data[i] += data[i][j]*v1.data[j];
        }
    }

    return v2;
}