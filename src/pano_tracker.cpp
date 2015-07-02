#include <iostream>
#include <fstream>
#include <Eigen/Core>
#include "ros/ros.h"
// opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <look3d/track/panoramic_tracker.h>

#include <geometry_msgs/PoseWithCovariance.h>

geometry_msgs::PoseWithCovariance poseMsg;

// Gets current projection matrix (= PerspectiveMatrix * CameraPoseMatrix)
Eigen::Matrix<double, 3, 4> get_projection(look3d::PanoramicTracker& tracker) {
    std::vector<double> cam_params = tracker.GetCameraParams();
    Eigen::Matrix3d intrinsics = Eigen::Matrix3d::Identity();
    intrinsics(0, 0) = cam_params[0];
    intrinsics(1, 1) = cam_params[1];
    intrinsics(0, 2) = cam_params[2];
    intrinsics(1, 2) = cam_params[3];

    std::vector<double> rot = tracker.GetCurrentPose();
    Eigen::Matrix<double, 3, 3> mrot;
    mrot << rot[0], rot[1], rot[2],
            rot[3], rot[4], rot[5],
            rot[6], rot[7], rot[8];

    Eigen::Matrix<double, 3, 1> mtra;
    mtra << rot[9],
            rot[10],
            rot[11];

    poseMsg.pose.position.x = rot[9];
    poseMsg.pose.position.y = rot[10];
    poseMsg.pose.position.z = rot[11];

    Eigen::Matrix<double, 3, 4> pose;
    pose.setZero();
    pose.block<3, 3>(0, 0) = mrot;

    Eigen::Matrix<double, 3, 4> projection = intrinsics * pose;
    return projection;
}

// Draws desirable target in world coordinate to current color image
void draw_target(cv::Mat& rgb_img, look3d::PanoramicTracker& tracker) {
  const Eigen::Vector4d point_target(0, 0, 1, 1);
  Eigen::Matrix<double, 3, 4> proj = get_projection(tracker);
  Eigen::Vector3d point_cam = proj * point_target;
  point_cam /= point_cam[2];
  Eigen::Vector3d pointx_cam = proj * (point_target + Eigen::Vector4d(0.1, 0, 0, 1));
  pointx_cam /= pointx_cam[2];
  Eigen::Vector3d pointy_cam = proj * (point_target + Eigen::Vector4d(0, 0.1, 0, 1));
  pointy_cam /= pointy_cam[2];
  Eigen::Vector3d pointz_cam = proj * (point_target + Eigen::Vector4d(0, 0, 0.1, 1));
  pointz_cam /= pointz_cam[2];
  cv::line(rgb_img, cv::Point(point_cam[0], point_cam[1]),
           cv::Point(pointx_cam[0], pointx_cam[1]), cv::Scalar(0, 0, 255), 3);
  cv::line(rgb_img, cv::Point(point_cam[0], point_cam[1]),
           cv::Point(pointy_cam[0], pointy_cam[1]), cv::Scalar(0, 255, 0), 3);
  cv::line(rgb_img, cv::Point(point_cam[0], point_cam[1]),
           cv::Point(pointz_cam[0], pointz_cam[1]), cv::Scalar(255, 0, 0), 3);
}


using namespace look3d;
int main(int argc, char ** argv) {
  /// initialize camera capture

  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher posePub = n.advertise<geometry_msgs::PoseWithCovariance>("/look3d/pose", 1);

  cv::VideoCapture capture;
#ifdef ANDROID
  if (!capture.open(CV_CAP_ANDROID + 0)) {
    LOGE("ERROR: Cannot open cv::VideoCapture");
    return -1;
  }
#else
  if (!capture.open(0)) {
    printf("ERROR: Cannot open cv::VideoCapture\n");
    return -1;
  }
#endif
  capture.set(CV_CAP_PROP_FRAME_WIDTH, 640);
  capture.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
  
  /// initialize the tracker
  PanoramicTracker tracker;
  std::string config_file = (argc > 1) ? argv[1] :
    //std::string(Look3D_ROOT) + "/look3d/examples/pano_tracker/pano_tracker.ini";
  std::string("pano_tracker.ini");
  if (!tracker.Configure(config_file)) {
    printf("Configuration fails (file:%s)\n", config_file.c_str());
    return -1;
  }
  tracker.RestartTracker();

  bool stop = false;
  while (!stop) {
    static cv::Mat color_img;
    capture.read(color_img);
    static cv::Mat gray_img;
    cv::cvtColor(color_img, gray_img, CV_BGR2GRAY);
    PanoramicTracker::TrackResult res = tracker.Track(gray_img);
    std::cout << "Tracking result:" << tracker.TrackResultToStr(res) << std::endl;

    cv::Mat cloned_img = color_img.clone();
    draw_target(cloned_img, tracker);
    cv::imshow("Panoramic Tracker", cloned_img);

    posePub.publish(poseMsg);

    char c = cv::waitKey(1);
    switch (c) {
      case 27:
      stop = true;
      break;
      case 'r':
      tracker.RestartTracker();
      break;
    }
  }

  return 0;
}
