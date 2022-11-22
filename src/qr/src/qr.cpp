// ros
#include <iostream>
#include <ros/ros.h>
// #include <Eigen/Eigen>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
// cv
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// srv
#include <qr_msgs/qr.h>

class qrController
{
public:
  qrController();
  void imgCallback(const sensor_msgs::Image::ConstPtr &img);
  bool serve(qr_msgs::qr::Request &req, qr_msgs::qr::Response &res);
  void body();

private:
  ros::NodeHandle nh_;
  ros::ServiceServer command_receive_;
  ros::Subscriber img_sub_;
  sensor_msgs::Image::ConstPtr img_;
};

qrController::qrController()
{
  img_sub_ = nh_.subscribe("cam", 1, &qrController::imgCallback, this);
  command_receive_ = nh_.advertiseService("/qr/scan_qr", &qrController::serve, this);
}

void qrController::imgCallback(const sensor_msgs::Image::ConstPtr &img)
{
  img_ = img;
}

bool qrController::serve(qr_msgs::qr::Request &req, qr_msgs::qr::Response &res)
{
  if (req.start_calling)
  {
    cv::Mat image;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    image = cv_bridge::toCvCopy(img_, "bgr8")->image;
    cv::aruco::detectMarkers(image, dictionary, corners, ids);
    if (ids.size() > 0)
    {
      ROS_INFO("===== GOT ID %d =====", ids[0]);
      res.id.data = ids[0];
      res.success = true;
    }
    else
    {
      res.success = false;
    }
  }
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "qr_node");
  ROS_INFO("qr node started");
  qrController node;
  ros::Rate loop_rate(20);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
}