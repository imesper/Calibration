#ifndef ROBPOINT_H
#define ROBPOINT_H

class RobPoint
{
public:
  RobPoint();
  RobPoint(float x, float y, float z, float q1, float q2, float q3, float q4)
    : x(x)
    , y(y)
    , z(z)
    , q1(q1)
    , q2(q2)
    , q3(q3)
    , q4(q4)
  {}

  float x;
  float y;
  float z;
  float q1;
  float q2;
  float q3;
  float q4;
};

#endif // ROBPOINT_H
