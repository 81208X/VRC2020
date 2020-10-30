#ifndef _UTIL_STRUCT_HPP_INCLUDED
#define _UTIL_STRUCT_HPP_INCLUDED

namespace util
{
  struct WheelSpeed
  {
    double lf;
    double lr;
    double rf;
    double rr;

    /**
     * Normalizes the wheelspeed to ensure all wheel speed is lower than a set value.
     * \param maxSpeed the maximum wheel speed for any single wheel.
     */
    WheelSpeed &normalize(double);
  };

  /// Stores a set of 2d coordinates and supports for a variety of useful math operations.
  class Pos2d
  {
  public:
    Pos2d();

    Pos2d(double x, double y);

    double distance(Pos2d &other);

    double distance(double x, double y);

    double norm();

    //Isn't needed in our program yet.
    //TODO: Pos2d RotateBy(Rotation? & other);

    Pos2d operator+(Pos2d &other);

    Pos2d &operator+=(Pos2d &other);

    Pos2d operator-(Pos2d &other);

    Pos2d &operator-=(Pos2d &other);

    Pos2d operator-();

    Pos2d operator*(double scalar);

    Pos2d &operator*=(double scalar);

    Pos2d operator/(double scalar);

    bool operator==(Pos2d &other);

    bool operator!=(Pos2d &other);

    Pos2d &operator/=(double scalar);

    double x = 0;
    double y = 0;
  };

  struct ChassisSpeed
  {
    double x;     // inches per second
    double y;     // inches per second
    double angle; // radians per second

    Pos2d getClosestPointAsHeading(Pos2d target);
    double getAngleToAsHeading(Pos2d target);
    double distance(Pos2d target);
  };


  ///The data structure and related functions are similar for both chassis speed and position, so the struct is reused.
  typedef ChassisSpeed ChassisPos;
} // namespace util
#endif /* _UTIL_STRUCT_HPP_INCLUDED */
