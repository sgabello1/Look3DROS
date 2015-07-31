#include <iostream>
#include <Eigen/Dense>
#include <opencv2/highgui/highgui.hpp>

// #include "slick/scene/poli_camera.h"
#include "look3d/track/core/esm.h"

#include "config.h"

using namespace std;
using namespace look3d;

int main(int argc, char ** argv){
  cv::Mat m_ref = cv::imread(DATA_PATH("family.jpg"),0);
  cv::Mat m_query = cv::imread(DATA_PATH("family-near.jpg"),0);
  Eigen::VectorXf cam_params(8);
  cam_params << 752, 480, 613.5163950028584, 613.7638878332488,
    389.6645077689237, 240.2420430948173, 
    -0.3756451043471036, 0.1885368227301451;
  slick::PoliCamera<LookScalar> camera(cam_params);
  LookScalar resize_factor=1./(2*2*2);
  int width,height;
  
  camera.get_resolution(width, height);
  width *= resize_factor;
  height *= resize_factor;
  cv::Size size(width, height);

  cv::Mat m_ref_resized, m_query_resized;
  cv::resize(m_ref, m_ref_resized, size);
  cv::resize(m_query, m_query_resized, size);

  Homography<8> homo;
  StaticAppearance appearance;
  ESMEstimator<Homography<8>, StaticAppearance> esm(m_ref_resized);
  esm.appearance = appearance;
  esm.transform = homo;
  esm.max_iterations = 2;
  ESMResult result = esm.optimize(m_query_resized);

  cout << " esm  result: " << result << endl;

  Eigen::Matrix<LookScalar,4,1> v4_cam;
  v4_cam << 613.5163950028584, 613.7638878332488, 389.6645077689237, 240.2420430948173;
  RotationEstimator<StaticAppearance> rot_esm(v4_cam);
  rot_esm.set_image(m_ref_resized);
  rot_esm.appearance = appearance;
  result  = rot_esm.optimize(m_query_resized);

  cout << " rot_esm  result: " << result << endl;
  cout << " rot_esm transform:" << rot_esm.transform.get_rotation() << endl;

  return 0;
}

