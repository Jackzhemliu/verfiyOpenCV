//
// Created by liuzm on 9/13/17.
//

#ifndef QUATERNION_H
#define QUATERNION_H

#include <Eigen/Dense>
#include <math.h>
#include <iostream>

void QuaternionInv(const Eigen::Vector4d &q, Eigen::Vector4d &output)
{
  output = -q;
  output(3) *= -1;
}

void QuaternionLeftTransform(const Eigen::Vector4d &q, Eigen::Matrix4d &output)
{
  output << q(3),  q(2),  -q(1),  q(0),
           -q(2),  q(3),   q(0),  q(1),
            q(1), -q(0),   q(3),  q(2),
           -q(0), -q(1),  -q(2),  q(3);
}

void QuaternionRightTransform(const Eigen::Vector4d &q, Eigen::Matrix4d &output)
{
  output << q(3), -q(2),   q(1),  q(0),
            q(2),  q(3),  -q(0),  q(1),
           -q(1),  q(0),   q(3),  q(2),
           -q(0), -q(1),  -q(2),  q(3);
}

void QuaternionMul(const Eigen::Vector4d &left, const Eigen::Vector4d &right,
                   Eigen::Vector4d &output)
{
  Eigen::Matrix4d Q1_MATR;
  QuaternionLeftTransform(left, Q1_MATR);
  output = Q1_MATR * right;
}

void QuaternionMatCrossProd(const Eigen::Vector4d &q, Eigen::Matrix3d &output)
{
  output << 0, -q(2),   q(1),
         q(2),     0,  -q(0),
        -q(1),  q(0),     0;
}

void QuaternionToRotMat(const Eigen::Vector4d &q, Eigen::Matrix3d &output)
{
  output(0,0) = q(0)*q(0) - q(1)*q(1) - q(2)*q(2) + q(3)*q(3);
  output(0,1) = 2*( q(0)*q(1) + q(2)*q(3));
  output(0,2) = 2*( q(0)*q(2) - q(1)*q(3));

  output(1,0) = 2*( q(0)*q(1) - q(2)*q(3));
  output(1,1) = -q(0)*q(0) + q(1)*q(1) - q(2)*q(2) + q(3)*q(3);
  output(1,2) = 2*( q(1)*q(2) + q(0)*q(3));

  output(2,0) = 2*( q(0)*q(2) + q(1)*q(3) );
  output(2,1) = 2*( q(1)*q(2) - q(0)*q(3) );
  output(2,2) = -q(0)*q(0) - q(1)*q(1) + q(2)*q(2) + q(3)*q(3);
}

void RotMatToQuaternion(const Eigen::Matrix3d &rotation, Eigen::Vector4d &q)
{
  double minimal(1.0e-12);
  double sum_diag = 1 + rotation(0,0) + rotation(1,1) + rotation(2,2);

  if (fabs(sum_diag) > minimal)
  {

    q(3) = 1/2.0f * sqrt(fabs(sum_diag));
    q(0) = 1/(4.0f * q(3)) * (rotation(1,2) - rotation(2,1));
    q(1) = 1/(4.0f * q(3)) * (rotation(2,0) - rotation(0,2));
    q(2) = 1/(4.0f * q(3)) * (rotation(0,1) - rotation(1,0));
    std::cout << q << std::endl;
  }
  else
  {
    q(3) = 0;
    sum_diag = -1/2.0f *(rotation(0,0) + rotation(1,1));
    if (fabs(sum_diag) > minimal)
    {
      q(2)=sqrt( fabs((-1/2.0f)*(rotation(0,0)+rotation(1,1))) );
      q(0)=(1/(4.0f*q(2)))*(rotation(0,2)+rotation(2,0));
      q(1)=(1/(4.0f*q(2)))*(rotation(1,2)+rotation(2,1));
    }
    else {
      q(2) = 0;
      sum_diag = (-1/2.0f)*(rotation(1,1) + rotation(2,2));
      if (fabs(sum_diag) > minimal)
      {
        q(0)=sqrt( fabs((-1/2.0f)*(rotation(1,1)+ rotation(2,2))) );
        q(1)=(1/(2*q(0)))*rotation(1,0);
      }
      else {
        q(0) = 0;
        q(1) = sqrt(fabs(rotation(1,1)));
      }
    }
  }
}











#endif //EXTRINSIC_CALIBRATION_QUATERNIONCONVERT_H
