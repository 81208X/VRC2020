#include "struct.hpp"
#include "util.hpp"
#include <cmath>


namespace util{

    /*-----WheelSpeed-------*/
    WheelSpeed& WheelSpeed::normalize(double mx)
    {
        double maxMagnitude = abs(lf);
        maxMagnitude = std::max(maxMagnitude, abs(lr));
        maxMagnitude = std::max(maxMagnitude, abs(rf));
        maxMagnitude = std::max(maxMagnitude, abs(rr));

        double adjustment = mx / maxMagnitude;

        if (maxMagnitude > mx){
            // pros::lcd::print(GYRO_LCD_LINE, "Clamped %lld", pros::millis());
            lf *= adjustment;
            lr *= adjustment;
            rf *= adjustment;
            rr *= adjustment;
        }

        return *this;
    }

    /*-----ChassisSpeed-------*/
    Pos2d ChassisSpeed::getClosestPointAsHeading(Pos2d target){
        //https://www.desmos.com/calculator/3lze2ajlnz
        Pos2d heading = {sin(angle), cos(angle)};
        Pos2d cur = {x,y};
        Pos2d diff = target - cur;
        double d = diff.x * heading.x + diff.y * heading.y;
        return {x + heading.x * d, y + heading.y * d};
    }

    double ChassisSpeed::getAngleToAsHeading(Pos2d target){
        double diffAngle = atan2(target.x-x, target.y-y) - angle;
        return util::wrapAngle(diffAngle);
    }

    double ChassisSpeed::distance(Pos2d target){
        Pos2d a = {x,y};
        return target.distance(a);
    }


    /*-----Pos2d-------*/
    Pos2d::Pos2d()
    {
        x = 0;
        y = 0;
    }

    Pos2d::Pos2d(double x, double y)
    {
        this->x = x;
        this->y = y;
    }

    double Pos2d::distance(Pos2d &other)
    {
        return std::sqrt(pow(x - other.x, 2) + pow(y - other.y, 2));
    }

    double Pos2d::distance(double x, double y)
    {
        return std::sqrt(pow(this->x - x, 2) + pow(this->y - y, 2));
    }

    double Pos2d::norm()
    {
        return std::sqrt(pow(x, 2) + pow(y, 2));
    }

    Pos2d Pos2d::operator+(Pos2d &other)
    {
        return {x + other.x, y + other.y};
    }

    Pos2d &Pos2d::operator+=(Pos2d &other)
    {
        x += other.x;
        y += other.y;
        return *this;
    }

    Pos2d Pos2d::operator-(Pos2d &other)
    {
        return {x - other.x, y - other.y};
    }

    Pos2d &Pos2d::operator-=(Pos2d &other)
    {
        x -= other.x;
        y -= other.y;
        return *this;
    }

    Pos2d Pos2d::operator-() { return {-x, -y}; }

    Pos2d Pos2d::operator*(double scalar)
    {
        return {scalar * x, scalar * y};
    }

    Pos2d &Pos2d::operator*=(double scalar)
    {
        x *= scalar;
        y *= scalar;
        return *this;
    }

    Pos2d Pos2d::operator/(double scalar)
    {
        return *this * (1.0 / scalar);
    }

    bool Pos2d::operator==(Pos2d &other)
    {
        return abs(x - other.x) < 1e-9 &&
            abs(y - other.y) < 1e-9;
    }

    bool Pos2d::operator!=(Pos2d &other)
    {
        return !operator==(other);
    }

    Pos2d &Pos2d::operator/=(double scalar)
    {
        *this *= (1.0 / scalar);
        return *this;
    }
}