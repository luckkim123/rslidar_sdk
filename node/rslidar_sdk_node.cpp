#include "manager/node_manager.hpp"
#include <rs_driver/macro/version.hpp>
#include <signal.h>
#include <mutex>
#include <condition_variable>

#ifdef ROS_FOUND
#include <ros/ros.h>
#include <ros/package.h>
#elif ROS2_FOUND
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#endif

using namespace robosense::lidar;
#ifdef ROS2_FOUND
std::mutex g_mtx;
std::condition_variable g_cv;
#endif

class rslidar : public rclcpp::Node
{
public:
    std::string path;
    rslidar() : Node("rslidar_sdk_node")
    {
        this->declare_parameter<std::string>("config_path", "/default/path/to/config");
        this->get_parameter("config_path", path);
        RCLCPP_INFO(this->get_logger(), "Config path: %s", path.c_str());
    }
};

static void sigHandler(int sig)
{
  RS_MSG << "RoboSense-LiDAR-Driver is stopping....." << RS_REND;
#ifdef ROS_FOUND
  ros::shutdown();
#elif ROS2_FOUND
  g_cv.notify_all();
#endif
}

int main(int argc, char** argv)
{
  signal(SIGINT, sigHandler);

  RS_TITLE << "********************************************************" << RS_REND;
  RS_TITLE << "**********                                    **********" << RS_REND;
  RS_TITLE << "**********    RSLidar_SDK Version: v" << RSLIDAR_VERSION_MAJOR 
           << "." << RSLIDAR_VERSION_MINOR 
           << "." << RSLIDAR_VERSION_PATCH << "     **********" << RS_REND;
  RS_TITLE << "**********                                    **********" << RS_REND;
  RS_TITLE << "********************************************************" << RS_REND;

#ifdef ROS_FOUND
  ros::init(argc, argv, "rslidar_sdk_node", ros::init_options::NoSigintHandler);
#elif ROS2_FOUND
  rclcpp::init(argc, argv);
#endif

  auto node = std::make_shared<rslidar>();
  std::string config_path = node->path;
  // config_path += "/config/config.yaml";

#ifdef ROS_FOUND
  ros::NodeHandle priv_nh("~");
  std::string path;
  priv_nh.param("config_path", path, std::string(""));
  if (!path.empty())
  {
    config_path = path;
  }
#endif

  YAML::Node config;
  try
  {
    config = YAML::LoadFile(config_path);
  }
  catch (...)
  {
    RS_ERROR << "The format of config file " << config_path 
             << " is wrong. Please check (e.g. indentation)." << RS_REND;
    return -1;
  }

  std::shared_ptr<NodeManager> demo_ptr = std::make_shared<NodeManager>();
  demo_ptr->init(config);
  demo_ptr->start();

  RS_MSG << "RoboSense-LiDAR-Driver is running....." << RS_REND;

#ifdef ROS_FOUND
  ros::spin();
#elif ROS2_FOUND
  std::unique_lock<std::mutex> lck(g_mtx);
  g_cv.wait(lck);
#endif

  return 0;
}
