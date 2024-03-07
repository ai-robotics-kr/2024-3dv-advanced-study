#ifndef __POINT_H__
#define __POINT_H__

#include <cmath>
#include <stdexcept>

template <typename T>
struct Point {
public:
    Point() : x(0), y(0), z(0) {}
    Point(const T x, const T y, const T z) : x(x), y(y), z(z) {}

    Point operator+(const Point& other) const {
        return Point(x + other.x, y + other.y, z + other.z);
    }
    Point operator+(const T scalar) const {
        return Point(x + scalar, y + scalar, z + scalar);
    }
    Point operator-(const Point& other) const {
        return Point(x - other.x, y - other.y, z - other.z);
    }
    Point operator-(const T scalar) const {
        return Point(x - scalar, y - scalar, z - scalar);
    }
    Point operator*(const T scalar) const {
        return Point(x * scalar, y * scalar, z * scalar);
    }
    Point operator/(const T scalar) const {
        if(scalar == 0) {
            throw std::invalid_argument("Division by zero");
        }
        return Point(x / scalar, y / scalar, z /scalar);
    }
    Point operator*(const Point& other) const {
        return Point(x * other.x, y * other.y, z * other.z);
    }
    Point operator/(const Point& other) const {
        if(other.x == 0 || other.y == 0 || other.z == 0){
            throw std::invalid_argument("Division by zero");
        }
        return Point(x / other.x, y / other.y, z / other.z);
    }
    Point& operator=(const Point& other) {
        if (this != &other) {
            x = other.x;
            y = other.y;
            z = other.z;
        }
        return *this;
    }
    bool operator==(const Point& other) const {
        return (x == other.x) && (y == other.y) && (z == other.z);
    }
    bool operator!=(const Point& other) const {
        return !(*this == other);
    }

    Point floor() {
        return Point(std::floor(x), std::floor(y), std::floor(z));
    }
    Point ceil() {
        return Point(std::ceil(x), std::ceil(y), std::ceil(z));
    }
    
    template <typename TargetType>
    Point<TargetType> convertTo() const {
        return Point<TargetType>(static_cast<TargetType>(x), static_cast<TargetType>(y), static_cast<TargetType>(z));
    }

    Point<int32_t> sign() const {
        return Point<int32_t>(x >= 0 ? 1 : -1, y >= 0 ? 1 : -1, z >= 0 ? 1 : -1 );
    }

    T x, y, z;
};

inline double logOdd(const double probability) {
    if (probability >= 0.99999) return 10;
    if (probability <= 1e-5) return -10;
    return std::log(probability / (1 - probability));
}

inline double probability(const double logodd) {
    return std::exp(logodd) / (1 + std::exp(logodd));
}


#endif // __POINT_H__