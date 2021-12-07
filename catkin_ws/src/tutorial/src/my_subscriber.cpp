#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <math.h>
#define _USE_MATH_DEFINES

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    // cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::imwrite("view.jpg", cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::waitKey(10);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

tuple<int, int> MinMax(cv::Mat x) 
{
  tuple<int, int> minmax;
  int min;
  int max;
  cv::Point minLoc;
  cv::Point maxLoc;

  cv::minMaxLoc(x, &minVal, &maxVal, &minLoc, &maxLoc);

  minmax = make_tuple(min, max);
  return minmax;
}

double toRad(double x) 
{
  return x/180.0*M_PI;
}

cv::Mat tranform(cv::Mat pc, cv::Mat T) 
{
  cv::Mat T_sub = cv::Mat(T, cv::rowRange(0, 2), cv::colRange(0, 2));
  cv::Mat T_vec = cv::Mat(T, cv::rowRange(0, 2), cv::colRange(0, 0));

  return T_sub*pc + T_vec;
}

int main(int argc, char **argv)
{
  /*
   * Camera Intrinsic Matrix K
   */
  float K_arr[9] = { 1508.41265658058, 0                , 1016.85392397151,
                     0               , 1508.41328115334 , 1010.74419398673,
                     0               , 0                , 1               };

  cv::Mat K = cv::Mat(3, 3, CV_32F, K_arr);
  
  /*
   * Camera Distortion Coefficient Vector D
   */
  float D_arr[4] = { -0.165919118220638,0.103174388708803,0,0 };
  cv::Mat D = cv::Mat(1, 4, CV_32F, D_arr);

  /*
   * Transformation Matrix from Right Lidar to Left Lidar
   */
  float T_RL_arr[16] = {   0.974989004547664,   0.057888450527916,   0.214581845240068,   0.569917003996719,
                          -0.055903198202045,   0.998318742519729,  -0.015314070804663,   0.365951434873460,
                          -0.215107585737784,   0.002935239224394,   0.976585946514058,  -0.405460968057564,
                                           0,                   0,                   0,   1.000000000000000 };
  cv::Mat T_RL = cv::Mat(4, 4, CV_32F, T_RL_arr);

  float T_LC_arr[16] = {  -0.009946680215331,  -0.999928983625514,  -0.006564393219536,  -0.029710425546210,
                          -0.132985914239468,   0.007829209175149,  -0.991087004302641,  -0.320434885044933,
                          -0.215107585737784,   0.002935239224394,   0.976585946514058,  -0.405460968057564,
                                           0,                   0,                   0,   1.000000000000000 };
  cv::Mat T_LC = cv::Mat(4, 4, CV_32F, T_LC_arr);

  std::cout << K.at<float>(1,2) << std::endl;
  std::cout << K << std::endl; 
  std::cout << D << std::endl; 
  std::cout << T_RL << std::endl;
  std::cout << T_LC << std::endl;
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/front_camera/image_raw", 1, imageCallback);
  ros::spin();
  cv::destroyWindow("view");

}
