#include <ros/ros.h>
#include <image_transport/image_transport.hpp>
#include "uvc_cam/uvc_cam.h"
#include <boost/thread.hpp>
#include <mutex>
#include <camera_info_manager/camera_info_manager.h>
#include <builtin_interfaces/msg/time.hpp>
#include <std_msgs/msg/float64.hpp>

namespace uvc_camera {

class Camera {
  public:
    Camera(ros::NodeHandle comm_nh, ros::NodeHandle param_nh);
    void onInit();
    void sendInfo(sensor_msgs::msg::ImagePtr::SharedPtr image, rclcpp::Time time);
    void feedImages();  
    ~Camera();

    void timeCb(builtin_interfaces::msg::Time time);
    void callBackExposure(std_msgs::msg::Float64 call_exposure_value);
    void callBackBrightness(std_msgs::msg::Float64 call_brightness_value);

  private:
    ros::NodeHandle node, pnode;
    image_transport::ImageTransport it;
    bool ok;

    int width, height, fps, skip_frames, frames_to_skip;
    std::string device, frame;
    int  exposure_value;
    int  brightness_value;
    bool rotate;

    camera_info_manager::CameraInfoManager info_mgr;

    image_transport::Publisher pub;
    ros::Publisher info_pub;
    ros::Publisher exposure_pub;
    ros::Publisher brightness_pub;

    ros::Subscriber time_sub;
    ros::Subscriber exposure_sub;
    ros::Subscriber brightness_sub;
    
    rclcpp::Time last_time;
    boost::mutex time_mutex_;

    uvc_cam::Cam *cam;
    std::thread image_thread;
};

};

