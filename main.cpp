#include <Eigen/Dense> 
#include <iostream> 
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp> 
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector> 
#include <Quaternion.h> 

int main() 
{
   std::vector<cv::Point2f> points1; 
   std::vector<cv::Point2f> points2;
   std::vector<cv::Point3f> points3d;  
  // generate 16 points in the first frame coordinate 
  // z = 1.2; 
  double z = 0.5; 
  for (int i = 0; i < 4; i++) 
  {
    for (int j =0; j < 4; j++) 
    {
      points3d.push_back(cv::Point3f(i+0.05, j+0.03, z)); 
    }   	
  }
    
  cv::Mat R_1_cv = cv::Mat::eye(3, 3, CV_32F); 
  cv::Mat R_2_cv = cv::Mat::zeros(3, 3, CV_32F);
  cv::Point3f t_2_cv(0.02, 0.0, 0.06);
  R_2_cv.at<float>(0, 0) = 0.5; 
  R_2_cv.at<float>(0, 2) = 0.866; 
  R_2_cv.at<float>(1, 1) = 1.0; 
  R_2_cv.at<float>(2, 0) = -0.866; 
  R_2_cv.at<float>(2, 2) = 0.5; 

  double focal = 150; 
  // Project points to cam 
  for (int i = 0; i < 16; i++) 
  {  
    cv::Point3f Point = points3d[i]; 
    points1.push_back(cv::Point2f(focal*Point.x/Point.z, focal*Point.y/Point.z));
    
    // R_world_in_cam2 = R_2_cv';
    cv::Mat R_world_in_cam2; 
    cv::transpose(R_2_cv, R_world_in_cam2); 	
    cv::Mat relative_point = cv::Mat::zeros(3, 1, CV_32F); 
    relative_point.at<float>(0, 0) = Point.x - t_2_cv.x; 
    relative_point.at<float>(1, 0) = Point.y - t_2_cv.y; 
    relative_point.at<float>(2, 0) = Point.z - t_2_cv.z; 

    cv::Mat Point_in_cam2 = R_world_in_cam2 * relative_point; 
     
    points2.push_back(cv::Point2f(focal*Point_in_cam2.at<float>(0,0)/ Point_in_cam2.at<float>(2,0), 
                                  focal*Point_in_cam2.at<float>(1,0)/ Point_in_cam2.at<float>(2,0))); 
  }
  
  // from 1 to 2
  cv::Mat E_2_on_1 = cv::findEssentialMat(points2, points1, focal);
  cv::Mat R_2_on_1, t_2_on_1;
  cv::recoverPose(E_2_on_1, points2, points1, R_2_on_1, t_2_on_1, focal);
  //std::cout << E << std::endl;
  std::cout <<"comparing R 2 on 1: " << std::endl;
  std::cout << R_2_cv << std::endl;
  std::cout << R_2_on_1 << std::endl;

  std::cout << "------" << std::endl;
  std::cout << "Comparing T: "<< std::endl;
  double scale = sqrt(0.02*0.02 + 0.06*0.06);
  std::cout << 0.02/scale << " " << 0.06/scale << std::endl;
  std::cout << t_2_on_1 << std::endl;



  return 0; 
}
