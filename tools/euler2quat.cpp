/*
 * =====================================================================================
 *
 *       Filename:  qua2euler.cpp
 *
 *    Description:
 *
 *        Version:  1.0
 *        Created:  10/24/2018 03:15:44 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (),
 *   Organization:
 *
 * =====================================================================================
 */
#include <stdlib.h>

#include <Eigen/Geometry>
#include <iostream>
#include <math.h>
#include <pcl/common/angles.h>

#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <string>

std::string g_unit = "deg";
std::vector<float> g_values;
using namespace pcl;
using namespace pcl::console;
Eigen::Quaterniond toQuaternion(double roll, double pitch, double yaw)
{
  Eigen::Quaterniond q;
  // Abbreviations for the various angular functions
  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);
  double cr = cos(roll * 0.5);
  double sr = sin(roll * 0.5);
  double cp = cos(pitch * 0.5);
  double sp = sin(pitch * 0.5);

  q.w() = cy * cr * cp + sy * sr * sp;
  q.x() = cy * sr * cp - sy * cr * sp;
  q.y() = cy * cr * sp + sy * sr * cp;
  q.z() = sy * cr * cp - cy * sr * sp;
  return q;
}
void toEulerAngle(const Eigen::Quaterniond& q, double& roll, double& pitch, double& yaw)
{
  // roll (x-axis rotation)
  double sinr = +2.0 * (q.w() * q.x() + q.y() * q.z());
  double cosr = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
  roll = atan2(sinr, cosr);

  // pitch (y-axis rotation)
  double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
  if (fabs(sinp) >= 1)
    pitch = copysign(M_PI / 2, sinp);  // use 90 degrees if out of range
  else
    pitch = asin(sinp);

  // yaw (z-axis rotation)
  double siny = +2.0 * (q.w() * q.z() + q.x() * q.y());
  double cosy = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
  yaw = atan2(siny, cosy);
}
void printHelp(int, char** argv)
{
  print_error("Syntax is: %s  -axisangle roll pitch yaw "
              "<options> [optional_arguments]\n",
              argv[0]);
  print_info("  where options are:\n");
  print_info("                     -unit X = deg(ree) or rad(ius) "
             "(default: ");
  print_value("%s", g_unit.c_str());
  print_info(")\n");
}

int main(int argc, char** argv)
{
  // euler to matrix
  if (argc < 3)
  {
    printHelp(argc, argv);
    return -1;
  }
  double roll = 0, pitch = 0, yaw = 0;
  parse_3x_arguments(argc, argv, "-axisangle", roll, pitch, yaw);
  pcl::console::parse_argument(argc, argv, "-unit", g_unit);
  if (g_unit != "rad")
  {
    roll = DEG2RAD(roll), pitch = DEG2RAD(pitch), yaw = DEG2RAD(yaw);
  }
  else
  {
    std::cout << "degree: " << RAD2DEG(roll) << "," << RAD2DEG(pitch) << "," << RAD2DEG(yaw) << std::endl;
  }
  Eigen::Affine3d trans = Eigen::Affine3d::Identity();
  trans.prerotate(Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
  trans.prerotate(Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()));
  trans.prerotate(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
  // trans.pretranslate(Eigen::Vector3d(x, y, z));
  std::cout << "trans\n" << trans.matrix() << std::endl;

  Eigen::Quaterniond q = toQuaternion(roll, pitch, yaw);
  std::cout << "quat = (" << q.w() << "," << q.x() << "," << q.y() << "," << q.z() << ")" << std::endl;
}
