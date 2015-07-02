// Look3D for tracking a picture-marker
// adjusting the projection with 2 homographies : 1 is the homography (fundamental matrix) the other is just for the scale
// the projection part is taken by the sar_chessboard3 idea

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Polygon.h"
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>


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

#define CHESSBOARD_WIDTH 6
#define CHESSBOARD_HEIGHT 5

#include "ros/ros.h"

using namespace std;
using namespace cv;
using namespace look3d;

bool token = false;
bool stop = false;
int id = 0;

cv_bridge::CvImagePtr cv_ptr;

PlaneTracker tracker;
vector<Point2f> vClick,vDisplay;    // stores 4 points that the user clicks(mouse left click) in the main image.
Mat image,displayWarped,displayWarped_t;
Mat display;
Mat  imageDesktop(800,1280, CV_8UC3,Scalar(0,0,255)), imageChessBoard;

Mat Hpc, Hcb, Htot;
vector<Point2f> dst,markers;      // Destination Points to transform overlay image  
vector<Point2f> corners;  
Mat imageCamera;
Mat imageChessBoardGray;

vector<Point2f> cbCornersProjector;
vector<Point2f> cornersTemplate;

bool go = false;
bool iWantCorners = false;

Size board_size(CHESSBOARD_WIDTH-1, CHESSBOARD_HEIGHT-1);


bool bTemplate,bTemplate2;

vector<Point2f> projectedPoints(4);      
Mat imageRect;



void findCamera2ProjectorHomography(){
   
    //iWantCorners = true;
    Mat gray;

    cvtColor(imageChessBoard, imageChessBoardGray, CV_BGR2GRAY);
    cvtColor(imageCamera, gray, CV_BGR2GRAY);

    bool flag = findChessboardCorners(imageCamera, board_size, corners);


    if(flag == 1)
    {            

      // This function identifies the chessboard pattern from the gray image, saves the valid group of corners
      cornerSubPix(gray, corners, Size(11,11), Size(-1,-1), TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1));

      // chess board corners in the camera space
      dst.push_back(corners[0]);
      dst.push_back(corners[CHESSBOARD_WIDTH-2]);
      dst.push_back(corners[(CHESSBOARD_WIDTH-1)*(CHESSBOARD_HEIGHT-1)-1]);
      dst.push_back(corners[(CHESSBOARD_WIDTH-1)*(CHESSBOARD_HEIGHT-2)]);

      cout << "Founded  chess board corners in the camera space "<< dst.size() << "\n";

    }

    bTemplate2 = findChessboardCorners(imageChessBoard, board_size, cornersTemplate, CV_CALIB_CB_ADAPTIVE_THRESH);

    if (bTemplate2 == 1){

    cout << "Founded corners in the template "<< cornersTemplate.size() << "\n";

    if(cornersTemplate.size() > 1)
    {
    cornerSubPix(imageChessBoardGray, cornersTemplate, Size(11,11), Size(-1,-1),TermCriteria( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1 ));
    cout << "Subpixel founded\n";
    }
      

    // corners detected in the projector cess board (template image)
    cbCornersProjector.push_back(cornersTemplate[0]);
    cbCornersProjector.push_back(cornersTemplate[CHESSBOARD_WIDTH-2]);
    cbCornersProjector.push_back(cornersTemplate[(CHESSBOARD_WIDTH-1)*(CHESSBOARD_HEIGHT-1)-1]);
    cbCornersProjector.push_back(cornersTemplate[(CHESSBOARD_WIDTH-1)*(CHESSBOARD_HEIGHT-2)]);

    cout << "Finded template corners...\n";
    cout << cbCornersProjector;
    cout << "\n";

    if(dst.size() == 4) {
    Hpc = findHomography(dst,cbCornersProjector,0); //src dst -> chessboard u see, chessboard you want to project

    
    go = true;

    cout << "Hpc found " << Hpc << "\n now going for online warping\n";
    }
    else
    {

        cout << "dst is " << dst << "\n";

        cout << "Projected chessboard (dst) NOT found "<< dst.size() << " and cbCornersProjector " << cbCornersProjector.size() << "\n";
        cbCornersProjector.clear();

    }
    }
    else{

        cout << "Corners in the template NOT founded return "<< bTemplate2 << "\n";

    }

  }

void on_mouse( int e, int x, int y, int d, void *ptr )
{
    if (e == EVENT_LBUTTONDOWN )
    {
         
         cout <<" just go for calibration "<<endl;
         findCamera2ProjectorHomography();   

    }  
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
  

  const Eigen::Vector4d point_target(0.1, 0.1, 0, 1);
  Eigen::Matrix<double, 3, 4> proj = get_projection(tracker);

  Eigen::Vector3d pointA = proj * point_target;
  pointA /= pointA[2]; // point A

  Eigen::Vector3d pointB = proj * (point_target + Eigen::Vector4d(0.2, 0.1, 0, 1));
  pointB /= pointB[2]; // point B

  Eigen::Vector3d pointC = proj * (point_target + Eigen::Vector4d(0.2, 0.2, 0, 1));
  pointC /= pointC[2]; // point C

  Eigen::Vector3d pointD = proj * (point_target + Eigen::Vector4d(0.1,0.2, 0, 1));
  pointD /= pointD[2]; // point D
  
  circle(rgb_img, cv::Point(pointA[0], pointA[1]), 7, Scalar(0,0,255),-1); // image coordinates

  circle(rgb_img, cv::Point(pointB[0], pointB[1]), 7, Scalar(0,0,255),-1);

  circle(rgb_img, cv::Point(pointC[0], pointC[1]), 7, Scalar(0,0,255),-1);

  circle(rgb_img, cv::Point(pointD[0], pointD[1]), 7, Scalar(0,0,255),-1);

  markers.push_back(Point2f(pointA[0], pointA[1])); // from one marker I get 4 virtual markers where to project the image
  markers.push_back(Point2f(pointB[0], pointB[1]));
  markers.push_back(Point2f(pointC[0], pointC[1]));
  markers.push_back(Point2f(pointD[0], pointD[1]));

  if(go)
  {
    if(markers.size() == 4 && vDisplay.size() == 4)
    {

    Mat cpy_image(imageCamera.rows, imageCamera.cols, imageCamera.type());
    Mat neg_image(imageCamera.rows, imageCamera.cols, imageCamera.type());
    Mat gray;
    Mat blank(display.rows, display.cols, display.type());  
    
    Mat Hcb = findHomography(vDisplay, markers,0);

    blank = Scalar(0);
    neg_image = Scalar(0);                // Image is white when pixel values are zero
    cpy_image = Scalar(0);                // Image is white when pixel values are zero

    bitwise_not(blank,blank);

    warpPerspective(display, displayWarped_t, Hcb, imageDesktop.size());

    Mat image2;
    warpPerspective(display, neg_image, Hcb, Size(neg_image.cols, neg_image.rows)); // Transform overlay Image to the position  - [ITEM1]
    warpPerspective(blank, cpy_image, Hcb, Size(cpy_image.cols, neg_image.rows));   // Transform a blank overlay image to position  
    bitwise_not(cpy_image, cpy_image);              // Invert the copy paper image from white to black
    bitwise_and(cpy_image, imageCamera, cpy_image);           // Create a "hole" in the Image to create a "clipping" mask - [ITEM2]           
    bitwise_or(cpy_image, neg_image, image2);            // Finally merge both items [ITEM1 & ITEM2]

    imshow("Camera 2", image2);

    Htot = Hpc*Hcb;
    warpPerspective(display, displayWarped, Htot, imageChessBoard.size()); //imageChessBoard, because we calibrated in this image
    imshow("Final",displayWarped);

    
    }

    }

    markers.clear();
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
    ROS_INFO("suca");
    }
}

using namespace look3d;
int main(int argc, char ** argv){

  /// initialize the tracker
  // remember to put plane_target.png in the ini file
  const std::string config_file =
      std::string("/home/sgabello/catkin_ws/src/look3d_ros/src/plane_tracker.ini");

  display = imread("/home/sgabello/catkin_ws/src/spatially_ar/src/cb.png");

  // this is the image that has to be reconized (cessboard projected) cb1280_800.png
  imageChessBoard = imread("/home/sgabello/catkin_ws/src/spatially_ar/src/cb652.png");


  if( !imageChessBoard.data )
  { 
    std::cout<< " --(!) Error reading images " << std::endl;  
    
    return -1;
    }
  else
  {
    std::cout<< "cb652.png ok " << std::endl;
    imshow("Final", imageChessBoard);
  }
 
  // corners of the image to be projected
  vDisplay.push_back(Point2f(float(0),float(0)));
  vDisplay.push_back(Point2f(float(display.cols),float(0)));
  vDisplay.push_back(Point2f(float(display.cols),float(display.rows)));
  vDisplay.push_back(Point2f(float(0),float(display.rows)));

  cout << vDisplay << " corners vDisplay " << endl;

  ros::init(argc, argv, "spatialar_ros");

  ros::NodeHandle n;

  ros::Subscriber sub_image = n.subscribe("/image_rect_color", 3, receivedImage);

  if(! tracker.Configure(config_file))
  {
      printf("Some problems with config file \n");
      return -1;
  }
 
  tracker.RestartTracker();

  int key = 0;
  token = true;

  while(!stop)
  {
    //key = cvWaitKey(10);
    ros::spinOnce();
  }

  return 0;
  
}
