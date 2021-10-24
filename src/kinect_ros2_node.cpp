extern "C"
{
  #include "libfreenect/libfreenect.h"
}

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "opencv2/highgui.hpp"
#include "image_transport/image_transport.hpp"
#include "cv_bridge/cv_bridge.h"

using namespace std::chrono_literals;

static cv::Mat _depth_image(cv::Mat::zeros(cv::Size(640, 480), CV_16UC1));
static cv::Mat _rgb_image(cv::Mat::zeros(cv::Size(640, 480), CV_8UC3));

static uint16_t * _freenect_depth_pointer = nullptr;
static uint8_t * _freenect_rgb_pointer = nullptr;

static bool _depth_flag;
static bool _rgb_flag;

class KinectRos : public rclcpp::Node
{
public:
  KinectRos()
  : Node("kinect_ros2")
  {
    publisher_ = create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = create_wall_timer(1ms, std::bind(&KinectRos::timer_callback, this));

    depth_pub_ = image_transport::create_publisher(this, "depth/image_raw");
    rgb_pub_ = image_transport::create_publisher(this, "image_raw");

    int ret = freenect_init(&fn_ctx_, NULL);
    if (ret < 0) {
      RCLCPP_ERROR(get_logger(), "ERROR INIT");
      rclcpp::shutdown();
    }

    freenect_set_log_level(fn_ctx_, FREENECT_LOG_FATAL);
    freenect_select_subdevices(fn_ctx_, FREENECT_DEVICE_CAMERA);

    int num_devices = ret = freenect_num_devices(fn_ctx_);
    if (ret < 0) {
      RCLCPP_ERROR(get_logger(), "FREENECT - ERROR GET DEVICES");
      rclcpp::shutdown();
    }
    if (num_devices == 0) {
      RCLCPP_ERROR(get_logger(), "FREENECT - NO DEVICES");
      freenect_shutdown(fn_ctx_);
      rclcpp::shutdown();
    }
    ret = freenect_open_device(fn_ctx_, &fn_dev_, 0);
    if (ret < 0) {
      freenect_shutdown(fn_ctx_);
      RCLCPP_ERROR(get_logger(), "FREENECT - ERROR OPEN");
      rclcpp::shutdown();
    }
    ret =
      freenect_set_depth_mode(
      fn_dev_, freenect_find_depth_mode(
        FREENECT_RESOLUTION_MEDIUM,
        FREENECT_DEPTH_MM));
    if (ret < 0) {
      freenect_shutdown(fn_ctx_);
      RCLCPP_ERROR(get_logger(), "FREENECT - ERROR SET DEPTH");
      rclcpp::shutdown();
    }

    freenect_set_depth_callback(fn_dev_, depth_cb);
    freenect_set_video_callback(fn_dev_, rgb_cb);

    ret = freenect_start_depth(fn_dev_);
    if (ret < 0) {
      freenect_shutdown(fn_ctx_);
      RCLCPP_ERROR(get_logger(), "FREENECT - ERROR START DEPTH");
      rclcpp::shutdown();
    }

    ret = freenect_start_video(fn_dev_);
    if (ret < 0) {
      freenect_shutdown(fn_ctx_);
      RCLCPP_ERROR(get_logger(), "FREENECT - ERROR START RGB");
      rclcpp::shutdown();
    }
  }

  ~KinectRos()
  {
    RCLCPP_INFO(get_logger(), "stoping kinnect");
    freenect_stop_depth(fn_dev_);
    freenect_stop_video(fn_dev_);
    freenect_close_device(fn_dev_);
    freenect_shutdown(fn_ctx_);
  }

private:
  freenect_context * fn_ctx_;
  freenect_device * fn_dev_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

  image_transport::Publisher depth_pub_;
  image_transport::Publisher rgb_pub_;

  /* The freenect lib stores the depth data in a static region of the memory, so the best way to
  create a cv::Mat is passing the pointer of that region, avoiding the need of copying the data
  to a new cv::Mat. This way, the callback only used to set a flag that indicates that a new image
  has arrived. The flag is unset when a msg is published */
  static void depth_cb(freenect_device * dev, void * depth_ptr, uint32_t timestamp)
  {
    if (_depth_flag) {
      return;
    }

    if (_freenect_depth_pointer != (uint16_t *)depth_ptr) {
      _depth_image = cv::Mat(480, 640, CV_16UC1, depth_ptr);
      _freenect_depth_pointer = (uint16_t *)depth_ptr;
    }

    _depth_flag = true;
  }

  static void rgb_cb(freenect_device * dev, void * rgb_ptr, uint32_t timestamp)
  {
    if (_rgb_flag) {
      return;
    }

    if (_freenect_rgb_pointer != (uint8_t *)rgb_ptr) {
      _rgb_image = cv::Mat(480, 640, CV_8UC3, rgb_ptr);
      _freenect_rgb_pointer = (uint8_t *)rgb_ptr;
    }

    _rgb_flag = true;
  }

  void timer_callback()
  {
    freenect_process_events(fn_ctx_);

    if (_depth_flag) {
      auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono16", _depth_image).toImageMsg();
      depth_pub_.publish(msg);

      // cv::imshow("Depth", _depth_image);
      // cv::waitKey(1);
      _depth_flag = false;
    }

    if (_rgb_flag) {
      auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", _rgb_image).toImageMsg();
      rgb_pub_.publish(msg);

      // cv::imshow("RGB", _rgb_image);
      // cv::waitKey(1);
      _rgb_flag = false;
    }
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KinectRos>());
  rclcpp::shutdown();
  return 0;
}
