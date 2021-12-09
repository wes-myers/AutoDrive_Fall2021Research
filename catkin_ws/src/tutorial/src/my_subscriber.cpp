#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <math.h>
#include <vector>
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


std::tuple<int, int> MinMax(cv::Mat x) 
{
  std::tuple<int, int> minmax;
  double min;
  double max;
  cv::Point minLoc;
  cv::Point maxLoc;

  cv::minMaxLoc(x, &min, &max, &minLoc, &maxLoc);

  minmax = std::make_tuple(min, max);
  return minmax;
}


double toRad(double x) 
{
  return x/180.0*M_PI;
}

cv::Mat tranform(cv::Mat pc, cv::Mat T) 
{
  cv::Mat T_sub =  T.cv::Mat::rowRange(0, 2).cv::Mat::colRange(0, 2);
  cv::Mat T_vec =  T.cv::Mat::rowRange(0, 2).cv::Mat::colRange(0, 0);

  cv::Mat output = T_sub.cv::Mat::mul(pc);
  return output + T_vec;
}


cv::Mat frontObjPoints(cv::Mat pc) 
{
    std::vector<float> frontObjList;
    for(int i = 0; i < pc.rows; i++)
    {

      float x = pc.at<float>(i, 0);
      float y = pc.at<float>(i, 1);
      float z = pc.at<float>(i, 2);
      if ((x > 2.3) && ((y > -5.0) && (y < 5.0)) && (z > -1.5)) {
        frontObjList.push_back(x);
        frontObjList.push_back(y);
        frontObjList.push_back(z);
      }
    }

    int num_rows = frontObjList.size() / 3;
    if (num_rows != 0)
    {
      float* finalList = new float[num_rows];
      for (int i{}; i < frontObjList.size(); i++) 
      {
        finalList[i] = frontObjList.at(i);
      }
      
      return cv::Mat(num_rows, 3, CV_32F, finalList);
    }
}


/*
 * Assuming this function checks each set of points in pointcloud to check if they are within distance 
 * to the origin
 */
cv::Mat pointsinRange(cv::Mat pc, int d) 
{
  std::vector<float> pointList;
  for(int i{}; i < pc.rows; i++) 
  {

    std::cout << pc.row(i) << std::endl;
    std::cout << "norm " << cv::norm(pc.row(i), 1) << std::endl;
    if (cv::norm(pc.row(i), 1) < d)
    {
      pointList.push_back(pc.at<float>(i, 0));
      pointList.push_back(pc.at<float>(i, 1));
      pointList.push_back(pc.at<float>(i, 2));
    }
  }
  
  int num_rows = pointList.size() / 3;
    if (num_rows != 0)
    {
      float* finalList = new float[num_rows];
      for (int i{}; i < pointList.size(); i++) 
      {
        finalList[i] = pointList.at(i);
      }
      
      return cv::Mat(num_rows, 3, CV_32F, finalList);
    }
}


float getPadding(float theta, float x, float c, float f)
{
  return c + f*tan(toRad(theta) + atan((x-c)/f));
}

std::tuple<cv::Mat, cv::Mat> project(cv::Mat &pc, cv::Mat K, cv::Mat &T, int w = 2048, int h = 2048) 
{
  // cv::Mat T_sub = T.col(3).rowRange(0,3);
  cv::Mat T_sub = cv::Mat(3,3,CV_32F);
  for(int i{}; i < T_sub.cols; i++)
  {
    T.col(3).rowRange(0,3).copyTo(T_sub.col(i));
  }
  cv::Mat x = T.colRange(0, 3).rowRange(0,3) * pc;
  x += T_sub;
  x = K * x;
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

  // std::cout << K.at<float>(1,2) << std::endl;
  // std::cout << K << std::endl; 
  // std::cout << D << std::endl; 
  // std::cout << T_RL << std::endl;
  // std::cout << T_LC << std::endl;
  float example[12] = {   2.5,    40,    3,
                          2.1,   -1.6,    3,
                          2.5,    -1.3,   3,
                          2.6,    3.3,    -5.1 };
  float pcList[12] = {100.0, 50.0, 40.5, 105,
                  50.1, 53.2,25, 100,
                  25.6, 32.1,45.7, 40
  };
  cv::Mat example_mat = cv::Mat(4, 3, CV_32F, example);
  cv::Mat pc = cv::Mat(3, 4, CV_32F, pcList);

  std::cout << frontObjPoints(example_mat) << std::endl;
  std::cout << pointsinRange(example_mat, 5) << std::endl;
  std::cout << T_RL.rowRange(0,3).colRange(0,3) * pc.colRange(0,3).t() << std::endl;


  cv::Mat T_sub = cv::Mat(3,3,CV_32F);
  for(int i{}; i < T_sub.cols; i++)
  {
    T_RL.col(3).rowRange(0,3).copyTo(T_sub.col(i));
  }
  std::cout << "T_RL.colRange(0, 3).rowRange(0,3)\n" << T_sub << "pc.t()\n" << pc.t() << std::endl;
  cv::Mat x = T_RL.colRange(0, 3).rowRange(0,3) * pc.colRange(0,3);
  std::cout << "x:\n" << x << std::endl;
  x = x + T_sub;
  x = K * x;

  std::cout << "x:\n" << x << std::endl;

  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/front_camera/image_raw", 1, imageCallback);
  ros::spin();
  cv::destroyWindow("view");

}
