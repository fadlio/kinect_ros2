extern "C"
{
  #include "libfreenect/libfreenect.h"
// #include "libfreenect/libfreenect_sync.h"
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

static cv::Mat depth_image(cv::Mat::zeros(cv::Size(640, 480), CV_16UC1));

class KinectRos : public rclcpp::Node
{
public:
  KinectRos()
  : Node("kinect_ros2")
  {
    publisher_ = create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = create_wall_timer(1ms, std::bind(&KinectRos::timer_callback, this));

    depth_pub_ = image_transport::create_publisher(this, "depth/image_raw");

    int ret = freenect_init(&fn_ctx, NULL);
    if (ret < 0) {
      RCLCPP_ERROR(get_logger(), "ERROR INIT");
      rclcpp::shutdown();
    }

    freenect_set_log_level(fn_ctx, FREENECT_LOG_FATAL);
    freenect_select_subdevices(fn_ctx, FREENECT_DEVICE_CAMERA);

    int num_devices = ret = freenect_num_devices(fn_ctx);
    if (ret < 0) {
      RCLCPP_ERROR(get_logger(), "ERROR DEVICES");
      rclcpp::shutdown();
    }
    if (num_devices == 0) {
      printf("No device found!\n");
      freenect_shutdown(fn_ctx);
      rclcpp::shutdown();
    }
    freenect_device * fn_dev;
    ret = freenect_open_device(fn_ctx, &fn_dev, 0);
    if (ret < 0) {
      freenect_shutdown(fn_ctx);
      RCLCPP_ERROR(get_logger(), "ERROR OPEN");
      rclcpp::shutdown();
    }
    ret =
      freenect_set_depth_mode(
      fn_dev, freenect_find_depth_mode(
        FREENECT_RESOLUTION_MEDIUM,
        FREENECT_DEPTH_MM));
    if (ret < 0) {
      freenect_shutdown(fn_ctx);
      RCLCPP_ERROR(get_logger(), "ERROR SET DEPTH");
      rclcpp::shutdown();
    }
    freenect_set_depth_callback(fn_dev, depth_cb);
    ret = freenect_start_depth(fn_dev);
    if (ret < 0) {
      freenect_shutdown(fn_ctx);
      RCLCPP_ERROR(get_logger(), "ERROR START DEPTH");
      rclcpp::shutdown();
    }
  }

private:
  static void depth_cb(freenect_device * dev, void *depth, uint32_t timestamp)
  {
    printf("cb\n");
    cv::Mat img = cv::Mat(640, 480, CV_16UC1, depth);

    std::memcpy(depth_image.data, img.data, 640 * 480 * sizeof(uint16_t));
    // for (int i = 0; i < 100; i++){
    //   printf("%u", img.data[i]);
    // }
    // printf("\n");
    // cv::imshow("Depth", img);
  }

  void timer_callback()
  {
    auto tick = now().nanoseconds();
    freenect_process_events(fn_ctx);
    // RCLCPP_INFO(get_logger(), "freenect ns: '%li'", now().nanoseconds() - tick);

    // static IplImage * image = 0;
    // static char * data = 0;
    // if (!image) {image = cvCreateImageHeader(cvSize(640, 480), 16, 1);}
    // unsigned int timestamp;
    // if (freenect_sync_get_depth((void **)&data, &timestamp, 0, FREENECT_DEPTH_11BIT)) {
    //   return;
    // }
    // cvSetData(image, data, 640 * 2);
    // return image;

    tick = now().nanoseconds();
    // auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", depth_image_).toImageMsg();
    // depth_pub_.publish(msg);
    // RCLCPP_INFO(get_logger(), "Publish ns: '%li'", now().nanoseconds() - tick);
    cv::imshow("Depth", depth_image);
  }
  freenect_context * fn_ctx;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

  //static cv::Mat depth_image(640, 480, CV_8UC1);// = cv::Mat::zeros(cv::Size(640, 480), CV_8UC1);
  image_transport::Publisher depth_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KinectRos>());
  rclcpp::shutdown();
  return 0;
}
