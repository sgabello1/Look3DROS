#include <iostream>
#include <fstream>
#include <Eigen/Core>

// opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <look3d/track/plane_tracker.h>
#include "sensor_msgs/Image.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

//#include "config.h"

#include "ros/ros.h"

using namespace std;
using namespace cv;
using namespace look3d;

/*
bool token = false;
bool stop = false;
int id = 0;

Mat imageCamera;
cv_bridge::CvImagePtr cv_ptr;
PlaneTracker tracker;


void on_mouse( int e, int x, int y, int d, void *ptr )
{//for now just keep it
}

// Gets current projection matrix (= PerspectiveMatrix * CameraPoseMatrix)
Eigen::Matrix<double, 3, 4> get_projection(look3d::PlaneTracker& tracker) {
    std::vector<double> cam_params = tracker.GetCameraParams();
    Eigen::Matrix3d intrinsics = Eigen::Matrix3d::Identity();
    intrinsics(0, 0) = cam_params[0];
    intrinsics(1, 1) = cam_params[1];
    intrinsics(0, 2) = cam_params[2];
    intrinsics(1, 2) = cam_params[3];

    std::vector<double> m = tracker.GetCurrentPose();
    Eigen::Matrix<double, 4, 4> mtr;
        mtr << m[0], m[1], m[2], m[3],
            m[4], m[5], m[6], m[7],
            m[8], m[9], m[10], m[11],
            m[12], m[13], m[14], m[15];

    Eigen::Matrix<double, 3, 4> pose;
    pose.setZero();
    pose.block<3, 3>(0, 0) = mtr.block<3, 3>(0, 0);
    pose.block<3, 1>(0, 3) = mtr.block<3, 1>(0, 3);

    Eigen::Matrix<double, 3, 4> projection = intrinsics * pose;
    return projection;
}

// Draws desirable target in world coordinate to current color image
void draw_target(cv::Mat& rgb_img, look3d::PlaneTracker& tracker) {
  

  const Eigen::Vector4d point_target(0.2, 0.2, 0, 1);
  Eigen::Matrix<double, 3, 4> proj = get_projection(tracker);

  Eigen::Vector3d pointA = proj * point_target;
  pointA /= pointA[2]; // point A

  Eigen::Vector3d pointB = proj * (point_target + Eigen::Vector4d(0.7, 0.2, 0, 1));
  pointB /= pointB[2]; // point B

  Eigen::Vector3d pointC = proj * (point_target + Eigen::Vector4d(0.2, 0.7, 0, 1));
  pointC /= pointC[2]; // point C

  Eigen::Vector3d pointD = proj * (point_target + Eigen::Vector4d(0.7,0.7, 0, 1));
  pointD /= pointD[2]; // point D
  
  circle(rgb_img, cv::Point(pointA[0], pointA[1]), 7, Scalar(0,0,255),-1); // image coordinates

  circle(rgb_img, cv::Point(pointB[0], pointB[1]), 7, Scalar(0,0,255),-1);

  circle(rgb_img, cv::Point(pointC[0], pointC[1]), 7, Scalar(0,0,255),-1);

  circle(rgb_img, cv::Point(pointD[0], pointD[1]), 7, Scalar(0,0,255),-1);

}

void receivedImage(const sensor_msgs::Image::ConstPtr& img)
{
    if(token ){
    try
    {

    token = false;
    cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    imageCamera=cv_ptr->image; // fire fly

    cv::Mat gray_img;
    cv::cvtColor(imageCamera, gray_img, CV_BGR2GRAY);
    PlaneTracker::TrackResult res = tracker.Track(gray_img);
    std::cout << "Tracking result:" << tracker.TrackResultToStr(res) << std::endl;

    cv::Mat cloned_img = imageCamera.clone();
    draw_target(cloned_img, tracker);

    cv::imshow("Panoramic Tracker", cloned_img);

    char c = cv::waitKey(20);
    switch (c) {
      case 27:
      stop = true;
      break;
      case 'r':
      id = 0;
      tracker.RestartTracker();
      break;
      }
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  
     
    imshow("Camera", imageCamera);
    setMouseCallback("Camera",on_mouse, NULL );
    token = true;

    }
}*/

using namespace look3d;
int main(int argc, char ** argv){

  std::string conf = argv[1];
  cout << " Hey conf file is " << conf << endl;


  /// initialize the tracker
  // remember to put plane_target.png in the ini file
  const std::string config_file =
      std::string("/home/sgabello/catkin_ws/src/look3d_ros/src/plane_tracker.ini");

  // ros::init(argc, argv, "spatialar_ros");

  // ros::NodeHandle n;

  // ros::Subscriber sub_image = n.subscribe("/image_rect_color", 3, receivedImage);

  // if(! tracker.Configure(config_file))
  // {
  //     printf("Some problems with config file \n");
  //     return -1;
  // }
 
  // tracker.RestartTracker();

  // int key = 0;
  // token = true;

  // while((char)key != 'q' )
  // {
  //   key = cvWaitKey(10);
  //   ros::spinOnce();
  // }

  return 0;
  
}
