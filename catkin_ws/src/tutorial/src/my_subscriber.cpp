#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <math.h>
#include <vector>
#define _USE_MATH_DEFINES

/*
 * For use in Project() function
 *
 */
std::tuple<cv::Mat, cv::Mat> projTuple;

/*
 * Camera Intrinsic Matrix K
 *
 */
float K_arr[9] = { 1508.41265658058, 0                , 1016.85392397151,
                   0               , 1508.41328115334 , 1010.74419398673,
                   0               , 0                , 1               };

cv::Mat K = cv::Mat(3, 3, CV_32F, K_arr);

/*
 * Camera Distortion Coefficient Vector D
 *
 */
float D_arr[4] = { -0.165919118220638,0.103174388708803,0,0 };
cv::Mat D = cv::Mat(1, 4, CV_32F, D_arr);

/*
 * Transformation Matrix from Right Lidar to Left Lidar
 *
 */
float T_RL_arr[16] = {   0.974989004547664,   0.057888450527916,   0.214581845240068,   0.569917003996719,
                        -0.055903198202045,   0.998318742519729,  -0.015314070804663,   0.365951434873460,
                        -0.215107585737784,   0.002935239224394,   0.976585946514058,  -0.405460968057564,
                                         0,                   0,                   0,   1.000000000000000 };
cv::Mat T_RL = cv::Mat(4, 4, CV_32F, T_RL_arr);

float T_LC_arr[16] = {  
  -0.009946680215331,  -0.999928983625514,  -0.006564393219536,  -0.029710425546210,
  -0.132985914239468,   0.007829209175149,  -0.991087004302641,  -0.320434885044933,
   0.991068014904421,  -0.008985053663641,  -0.133054344701985,  -0.774930599969263,
                   0,                   0,                   0,   1.000000000000000
};
cv::Mat T_LC = cv::Mat(4, 4, CV_32F, T_LC_arr);

float pcLeft_arr[50] = {
  -2.0208149,   -7.73037004,  -2.14095116,   0.,           0.,        
  -6.655056,   -25.43992233,   0.45899829,   7.,           8.,        
  -2.36001849,  -9.01508427,  -2.1514318 ,   0.,           1.,        
  -6.88560915, -26.28376007,   1.42395687,   4.,           9.,       
  -2.61102915,  -9.96682549,  -2.00273132,   1.,           2.,        
  -6.53083897, -24.91179466,   2.25315022,   5.,          10.,        
  -3.46826959, -13.22025967,  -2.16474032,   2.,           3.,       
  -9.2395649,  -35.19411087,   4.46773005,  16.,          11.,        
  -4.5310545,  -17.24682045,  -2.18950486,   1.,           4.,        
  -9.78518105, -37.21949768,   6.09531307,  37.,          12.        
};
cv::Mat pcLeft = cv::Mat(10, 5, CV_32F, pcLeft_arr);
float pcRight_arr[50] = {
  -2.4547646 ,   9.01016331, -2.50226259 ,   2.,           0.,        
  -2.99385595, 11.00403023 , -2.63282704 ,   1.,           1.,        
  -3.83885479,  14.11958122,  -2.84419918,   1.,           2.,       
  -5.31772041,  19.58595848,  -3.21441579,   2.,           3.,        
  -2.41126895,   8.97389984,  -2.48983932,   2.,           0.,       
  -2.94128275,  10.96171379,  -2.62022996,   1.,           1.,        
  -3.76310802,  14.03428841,  -2.82435489,   1.,           2.,        
  -5.19311285,  19.39450455,  -3.18000031,   2.,           3.,        
  -2.36854696,   8.93931198,  -2.47793365,   2.,           0.,        
  -2.88954544,  10.92104721,  -2.60808253,   1.,           1.         
};
cv::Mat pcRight = cv::Mat(10, 5, CV_32F, pcRight_arr);

// void imageCallback(const sensor_msgs::ImageConstPtr& msg)

/*
 *
 */
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

/*
 *
 */
double toRad(double x) 
{
  return x/180.0*M_PI;
}

/*
 * Apply Transformation Matrix to 3D Point Cloud pc
 *
 */
cv::Mat Transform(cv::Mat pc, cv::Mat T) 
{
  std::cout  << "Transfrom: input pc\n" << pc << std::endl;

  cv::Mat T_sub = T.rowRange(0, 3).colRange(0, 3);
  cv::Mat T_vec =  T.rowRange(0, 3).colRange(3,3);
  std::cout << "yes" << std::endl;
  pc = T.rowRange(0, 3).colRange(0, 3) * pc;
  pc = pc + T_vec;
  std::cout  << "Transform: output pc\n" << pc.t() << std::endl;
  return pc.t();
}

/*
 * Filter 3D Point Cloud 
 * above the ground (z>-1.5),
 * in front of the vehicle front bumper (x>2.3),
 * in camera field of view (-5<y<5)
 *
 */
cv::Mat frontObjPoints(cv::Mat pc) 
{

    std::vector<float> frontObjList;
    for(int i = 0; i < pc.rows; i++)
    {
      float x = pc.at<float>(i, 0);
      float y = pc.at<float>(i, 1);
      float z = pc.at<float>(i, 2);
      std::cout << "frontObjPoints x: " << x << " y: " << y << " z: " << z << std::endl;
      if ((x > 2.3) && ((y > -5.0) && (y < 5.0)) && (z > -1.5)) {
        std::cout << "frontObjPoints: row.(" << i << ")\n" << pc.row(i) << std::endl;
        frontObjList.push_back(x);
        frontObjList.push_back(y);
        frontObjList.push_back(z);
      }
    }

    int num_rows = frontObjList.size() / 3;
    float* finalList = new float[num_rows * 3];
    if (num_rows == 0)
    {
      return cv::Mat::zeros(0,0,CV_32F);
    }
    for (int i{}; i < frontObjList.size(); i++) 
    {
      finalList[i] = frontObjList.at(i);
      std::cout << finalList[i];
    }
    
    std::cout << "frontObjPoints: output: \n" << cv::Mat(num_rows, 3, CV_32F, finalList) << "\n\n\n";
    return cv::Mat(num_rows, 3, CV_32F, finalList);
}


/*
 * Filter 3D Point Cloud within distance d 
 *
 */
cv::Mat pointsinRange(cv::Mat pc, int d) 
{
  std::vector<float> pointList;
  std::cout << "pointsinRange: input pc: \n" << pc << std::endl;
  for(int i{}; i < pc.rows; i++) 
  {
    if (cv::norm(pc.row(i)) < d)
    {
      std::cout << "pointsinRange: pc.row(" << i << "):\n" << pc.row(i) << std::endl;

      pointList.push_back(pc.at<float>(i, 0));
      pointList.push_back(pc.at<float>(i, 1));
      pointList.push_back(pc.at<float>(i, 2));
    }
  }
  std::cout << "pointsinRange: pointList.size() = " << pointList.size() << std::endl;
  int num_rows = pointList.size() / 3;
  std::cout << "pointsinRange: num_rows: " << num_rows << std::endl;
    if (num_rows == 0)
    {
      return cv::Mat::zeros(0,0,CV_32F);
    }
    float* finalList = new float[num_rows * 3];
    for (int i{}; i < pointList.size(); i++) 
    {
      finalList[i] = pointList.at(i);
    }
    std::cout << "\npointsinRange: output:\n" << cv::Mat(num_rows, 3, CV_32F, finalList) << "\n\n\n";
    return cv::Mat(num_rows, 3, CV_32F, finalList);

}

/*
 *
 */
float getPadding(float theta, float x, float c, float f)
{
  return c + f*tan(toRad(theta) + atan((x-c)/f));
}

/*
 * Get the pixel-wise laser point coverage area
 *
 */
std::tuple<cv::Mat, cv::Mat> Project(cv::Mat pc, cv::Mat K, cv::Mat T, int w = 2048, int h = 2048) 
{
  std::cout << "Project: T[:3,[-1]]:\n" << T.col(3).rowRange(0,3) << std::endl;
  cv::Mat x =  T.colRange(0, 3).rowRange(0,3).clone() * pc;
  for (int i{}; i < x.cols; i++) 
  {
    x.col(i) +=  T.col(3).rowRange(0,3).clone();
  }
  x = K * x;
  std::cout << "Project: x: \n" << x << std::endl;
  cv::Mat x_2D = cv::Mat::zeros(2, x.cols, CV_32F);
  for (int i{}; i < x.cols; i++) 
  {
    cv::Mat item = x.rowRange(0,2).col(i).clone();
    std::cout << "Project:  before item(" << i << "): \n" << item << std::endl;
    item /= x.at<float>(2, i);
    std::cout << "Project:  after item(" << i << "): \n" << item << std::endl;
    item.copyTo(x_2D.col(i));
  }

  cv::Mat projObjPoints = cv::Mat::zeros(2, x.cols, CV_32F);
  cv::Mat ObjPoints = cv::Mat::zeros(3, x.cols, CV_32F);
  int iter = 0;
  for (int i{}; i < x.cols; i++) 
  {
    
    if (((x_2D.at<float>(i, 0) >= 0) && (x_2D.at<float>(i, 0) < w)) &&
        ((x_2D.at<float>(i, 1) >= 0) && (x_2D.at<float>(i, 1) < h)))
    {
      std::cout << "Project: x_2D.col(" << i << ")\n" << x_2D.at<float>(0,i) << " " <<x_2D.at<float>(1,i) << std::endl;
      pc.col(i).copyTo(ObjPoints.col(iter));
      x_2D.col(i).copyTo(projObjPoints.col(iter));
      ++iter;
    }
  }
  projTuple = std::make_tuple<cv::Mat, cv::Mat>(projObjPoints.colRange(0, iter).t(), 
                                           ObjPoints.colRange(0, iter).t());
  return projTuple;
}

/*
 * Main function
 */
void imageCallback(cv::Mat pcRight, cv::Mat pcLeft)
{
  std::cout << cv::Mat::zeros(5, 3, CV_32F) <<std::endl;
  cv::Mat pcRight_points = pcRight.colRange(0,3);
  pcRight_points = Transform(pcRight_points.colRange(0,3).t(), T_RL.clone());
  cv::Mat pc = pcLeft;
  pc.push_back(pcRight);
  
  // pc sample data of points that are in range of camera and LiDar
  float pcInRange_arr[50] = {
  9.63046265,  4.56052542,  2.46006489, 4, 2,
  9.72741413,  4.35943651,  2.46096444, 2, 2,
  9.64061356,  4.15988588,  2.42407274, 2, 2,
  9.63804913,  3.72868013,  2.38583088, 2, 2,
  9.68922424,  3.70964861,  2.39527893, 2, 2,
  9.77860832,  3.66969633,  2.03020787, 2, 2,
  9.75619316,  3.3898325 ,  2.38448119, 2, 2,
  9.76796532,  3.35575843,  2.38448119, 2, 2,
  9.77962017,  3.32163858,  2.38448119, 2, 2,
  9.79115677,  3.28747869,  2.38448119, 2, 2
  };
  cv::Mat pcInRange = cv::Mat(10, 5, CV_32F, pcInRange_arr);
  cv::Mat objPoints = pointsinRange(frontObjPoints(pcInRange), 40);
  std::cout << "Callback : objPoints\n" << objPoints << std::endl;
  if (objPoints.cols == 0) {
    return;
  }
  cv::Mat objPoints_transpose = objPoints.t();
  std::tuple<cv::Mat, cv::Mat> project = Project(objPoints.clone().t(), K.clone(), T_LC.clone());
  cv::Mat projObjPoints = std::get<1>(project);
  objPoints = std::get<0>(project);
  std::cout << "Callback : objPoints\n" << objPoints << std::endl << "projObjPoints\n" << projObjPoints << std::endl;


  std::tuple<int, int> xMinMax = MinMax(objPoints.col(0));
  std::tuple<int, int> yMinMax = MinMax(objPoints.col(1));

  cv::Mat topDownMap = cv::Mat::zeros((std::get<1>(xMinMax) - std::get<0>(xMinMax) +1)
                                      ,(std::get<1>(yMinMax) - std::get<0>(yMinMax) + 1), CV_8UC1);


  // try
  // {
  //   cv::imwrite("view.jpg", cv_bridge::toCvShare(msg, "bgr8")->image);
  //   cv::waitKey(10);
  // }
  // catch (cv_bridge::Exception& e)
  // {
  //   ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  // }
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


  imageCallback(pcRight, pcLeft);

  // std::cout << frontObjPoints(example_mat) << std::endl;
  // std::cout << pointsinRange(example_mat, 5) << std::endl;



  cv::Mat T_sub = cv::Mat(3,3,CV_32F);
  for(int i{}; i < T_sub.cols; i++)
  {
    T_RL.col(3).rowRange(0,3).copyTo(T_sub.col(i));
  }

  cv::Mat x =  T_RL.colRange(0, 3).rowRange(0,3) * pc;

  for (int i{}; i < x.cols; i++) 
  {
    x.col(i) +=  T_RL.col(3).rowRange(0,3);
  }
  x = K * x;
  // std::cout << "x:\n" << x << std::endl;
  cv::Mat x_2D = cv::Mat(2, x.cols, CV_32F);
  for (int i{}; i < x.cols; i++) 
  {
    cv::Mat item = x.rowRange(0,2).col(i).clone();
    // std::cout << "item before " << item << std::endl;

    item /= x.at<float>(2, i);
    // std::cout << "item after " << item << std::endl;
    item.copyTo(x_2D.col(i));
  }
  x_2D = x_2D.t();
  // std::cout << "x_2D:\n" << x_2D << std::endl;

  // std::cout << "x:\n" << x << std::endl;
  int w = 6000;
  int h = 6000;
  cv::Mat projObjPoints = cv::Mat::zeros(x.cols, 2, CV_32F);
  cv::Mat ObjPoints = cv::Mat::zeros(3,x.cols, CV_32F);
  int iter = 0;

  for (int i{}; i < x.cols; i++) 
  {
    
    if (((x_2D.at<float>(i, 0) >= 0) && (x_2D.at<float>(i, 0) < w)) &&
        ((x_2D.at<float>(i, 1) >= 0) && (x_2D.at<float>(i, 1) < h)))
    {
      x.col(i).copyTo(ObjPoints.col(iter));
      x_2D.row(i).copyTo(projObjPoints.row(iter));
      iter++;
    }
  }
  
  // std::cout << "projObjPoints\n" << projObjPoints.rowRange(0,iter) << "\nObjPoints\n" << ObjPoints.colRange(0,iter).t() << std::endl;
  

  // ros::init(argc, argv, "image_listener");
  // ros::NodeHandle nh;
  // cv::namedWindow("view");
  // cv::startWindowThread();
  // image_transport::ImageTransport it(nh);
  // image_transport::Subscriber sub = it.subscribe("/front_camera/image_raw", 1, imageCallback);
  // ros::spin();
  // cv::destroyWindow("view");

}
