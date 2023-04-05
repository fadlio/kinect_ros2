#include "kinect_ros2/kinect_ros2_component.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std::chrono_literals;

namespace kinect_ros2
{
static cv::Mat _depth_image(cv::Mat::zeros(cv::Size(640, 480), CV_16UC1));
static cv::Mat _rgb_image(cv::Mat::zeros(cv::Size(640, 480), CV_8UC3));

static uint16_t * _freenect_depth_pointer = nullptr;
static uint8_t * _freenect_rgb_pointer = nullptr;

static bool _depth_flag;
static bool _rgb_flag;

KinectRosComponent::KinectRosComponent(const rclcpp::NodeOptions & options)
: Node("kinect_ros2", options)
{
  timer_ = create_wall_timer(1ms, std::bind(&KinectRosComponent::timer_callback, this));
  
  std::string pkg_share = ament_index_cpp::get_package_share_directory("kinect_ros2");

  //todo: use parameters
  depth_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(
    this, "kinect",
    "file://" + pkg_share + "/cfg/calibration_depth.yaml");

  rgb_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(
    this, "kinect",
    "file://" + pkg_share + "/cfg/calibration_rgb.yaml");

  rgb_info_ = rgb_info_manager_->getCameraInfo();
  rgb_info_.header.frame_id = "kinect_rgb";
  depth_info_ = depth_info_manager_->getCameraInfo();
  depth_info_.header.frame_id = "kinect_depth";

  depth_pub_ = image_transport::create_camera_publisher(this, "depth/image_raw");
  rgb_pub_ = image_transport::create_camera_publisher(this, "image_raw");

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

KinectRosComponent::~KinectRosComponent()
{
  RCLCPP_INFO(get_logger(), "stoping kinnect");
  freenect_stop_depth(fn_dev_);
  freenect_stop_video(fn_dev_);
  freenect_close_device(fn_dev_);
  freenect_shutdown(fn_ctx_);
}


/* The freenect lib stores the depth data in a static region of the memory, so the best way to
create a cv::Mat is passing the pointer of that region, avoiding the need of copying the data
to a new cv::Mat. This way, the callback only used to set a flag that indicates that a new image
has arrived. The flag is unset when a msg is published */
void KinectRosComponent::depth_cb(freenect_device * dev, void * depth_ptr, uint32_t timestamp)
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

void KinectRosComponent::rgb_cb(freenect_device * dev, void * rgb_ptr, uint32_t timestamp)
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

void KinectRosComponent::timer_callback()
{
  freenect_process_events(fn_ctx_);
  auto header = std_msgs::msg::Header();
  header.frame_id = "kinect_depth";

  auto stamp = now();
  header.stamp = stamp;
  depth_info_.header.stamp = stamp;

  if (_depth_flag) {
    //convert 16bit to 8bit mono
    // cv::Mat depth_8UC1(_depth_image, CV_16UC1);
    // depth_8UC1.convertTo(depth_8UC1, CV_8UC1);

    auto msg = cv_bridge::CvImage(header, "16UC1", _depth_image).toImageMsg();
    depth_pub_.publish(*msg, depth_info_);

    // cv::imshow("Depth", _depth_image);
    // cv::waitKey(1);
    _depth_flag = false;
  }

  if (_rgb_flag) {
    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", _rgb_image).toImageMsg();
    rgb_pub_.publish(*msg, rgb_info_);

    // cv::imshow("RGB", _rgb_image);
    // cv::waitKey(1);
    _rgb_flag = false;
  }
}
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(kinect_ros2::KinectRosComponent)
