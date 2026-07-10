#include <chrono>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <rtabmap/core/CameraModel.h>
#include <rtabmap/core/IMU.h>
#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/OdometryInfo.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Transform.h>

namespace
{

double stampToSec(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

std::string parameterToString(const rclcpp::Parameter & parameter)
{
  std::ostringstream stream;
  stream << std::setprecision(16);

  switch (parameter.get_type()) {
    case rclcpp::ParameterType::PARAMETER_BOOL:
      return parameter.as_bool() ? "true" : "false";
    case rclcpp::ParameterType::PARAMETER_INTEGER:
      return std::to_string(parameter.as_int());
    case rclcpp::ParameterType::PARAMETER_DOUBLE:
      stream << parameter.as_double();
      return stream.str();
    case rclcpp::ParameterType::PARAMETER_STRING:
      return parameter.as_string();
    default:
      return std::string();
  }
}

cv::Mat matrixFromCameraInfoK(const sensor_msgs::msg::CameraInfo & info)
{
  cv::Mat k = cv::Mat::zeros(3, 3, CV_64FC1);
  for (int i = 0; i < 9; ++i) {
    k.at<double>(i / 3, i % 3) = info.k[i];
  }
  return k;
}

cv::Mat matrixFromCameraInfoR(const sensor_msgs::msg::CameraInfo & info)
{
  cv::Mat r = cv::Mat::eye(3, 3, CV_64FC1);
  for (int i = 0; i < 9; ++i) {
    r.at<double>(i / 3, i % 3) = info.r[i];
  }
  return r;
}

cv::Mat matrixFromCameraInfoP(const sensor_msgs::msg::CameraInfo & info)
{
  cv::Mat p = cv::Mat::zeros(3, 4, CV_64FC1);
  for (int i = 0; i < 12; ++i) {
    p.at<double>(i / 4, i % 4) = info.p[i];
  }
  return p;
}

cv::Mat matrixFromDistortion(const sensor_msgs::msg::CameraInfo & info)
{
  if (info.d.empty()) {
    return cv::Mat();
  }

  cv::Mat d(1, static_cast<int>(info.d.size()), CV_64FC1);
  for (size_t i = 0; i < info.d.size(); ++i) {
    d.at<double>(0, static_cast<int>(i)) = info.d[i];
  }
  return d;
}

cv::Mat covariance3x3(const std::array<double, 9> & covariance)
{
  if (covariance[0] < 0.0) {
    return cv::Mat();
  }

  cv::Mat output(3, 3, CV_64FC1);
  for (int i = 0; i < 9; ++i) {
    output.at<double>(i / 3, i % 3) = covariance[i];
  }
  return output;
}

rtabmap::Transform transformFromMsg(const geometry_msgs::msg::Transform & transform)
{
  return rtabmap::Transform(
    static_cast<float>(transform.translation.x),
    static_cast<float>(transform.translation.y),
    static_cast<float>(transform.translation.z),
    static_cast<float>(transform.rotation.x),
    static_cast<float>(transform.rotation.y),
    static_cast<float>(transform.rotation.z),
    static_cast<float>(transform.rotation.w));
}

void fillPose(const rtabmap::Transform & pose, geometry_msgs::msg::Pose & output)
{
  const Eigen::Quaterniond q = pose.getQuaterniond();
  output.position.x = pose.x();
  output.position.y = pose.y();
  output.position.z = pose.z();
  output.orientation.x = q.x();
  output.orientation.y = q.y();
  output.orientation.z = q.z();
  output.orientation.w = q.w();
}

void fillCovariance(
  const cv::Mat & covariance,
  std::array<double, 36> & output,
  double multiplier)
{
  output.fill(0.0);

  if (covariance.rows == 6 && covariance.cols == 6) {
    for (int r = 0; r < 6; ++r) {
      for (int c = 0; c < 6; ++c) {
        double value = 0.0;
        if (covariance.type() == CV_64FC1) {
          value = covariance.at<double>(r, c);
        } else if (covariance.type() == CV_32FC1) {
          value = covariance.at<float>(r, c);
        }
        output[static_cast<size_t>(r * 6 + c)] = value * multiplier;
      }
    }
  } else {
    // 没有协方差时给一个偏大的对角值，避免下游把未知协方差当成高置信度。
    for (int i = 0; i < 6; ++i) {
      output[static_cast<size_t>(i * 6 + i)] = 9999.0;
    }
  }
}

}  // namespace

class MonoOpenVinsOdometryNode : public rclcpp::Node
{
public:
  explicit MonoOpenVinsOdometryNode(const rclcpp::NodeOptions & options)
  : Node("openvins_mono_odometry", options),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_),
    tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this))
  {
    frame_id_ = getOrDeclare<std::string>("frame_id", "camera_link");
    odom_frame_id_ = getOrDeclare<std::string>("odom_frame_id", "odom");
    publish_tf_ = getOrDeclare<bool>("publish_tf", true);
    wait_for_transform_ = getOrDeclare<double>("wait_for_transform", 0.2);
    use_tf_transforms_ = getOrDeclare<bool>("use_tf_transforms", true);
    convert_16bit_to_8bit_ = getOrDeclare<bool>("convert_16bit_to_8bit", true);

    camera_local_fallback_ = parseTransformParam(
      "camera_local_transform",
      rtabmap::CameraModel::opticalRotation());
    imu_local_fallback_ = parseTransformParam(
      "imu_local_transform",
      rtabmap::Transform::getIdentity());

    auto parameters = collectRtabmapParameters();
    parameters[rtabmap::Parameters::kOdomStrategy()] = "10";
    parameters[rtabmap::Parameters::kOdomOpenVINSUseStereo()] = "false";
    parameters[rtabmap::Parameters::kVisDepthAsMask()] = "false";

    odometry_.reset(rtabmap::Odometry::create(parameters));
    if (!odometry_) {
      throw std::runtime_error("无法创建 RTAB-Map OpenVINS odometry，请确认 RTAB-Map 已启用 WITH_OPENVINS。");
    }

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    const int queue_size = getOrDeclare<int>("queue_size", 30);
    image_sub_.subscribe(this, "image");
    camera_info_sub_.subscribe(this, "camera_info");
    sync_ = std::make_shared<Synchronizer>(SyncPolicy(queue_size), image_sub_, camera_info_sub_);
    sync_->registerCallback(
      std::bind(&MonoOpenVinsOdometryNode::imageCallback, this, std::placeholders::_1, std::placeholders::_2));

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu",
      rclcpp::SensorDataQoS(),
      std::bind(&MonoOpenVinsOdometryNode::imuCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "mono_odometry started: image + camera_info + imu -> /%s, frame_id=%s, odom_frame_id=%s",
      odom_pub_->get_topic_name(), frame_id_.c_str(), odom_frame_id_.c_str());
  }

private:
  using ImageMsg = sensor_msgs::msg::Image;
  using CameraInfoMsg = sensor_msgs::msg::CameraInfo;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<ImageMsg, CameraInfoMsg>;
  using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

  template<typename T>
  T getOrDeclare(const std::string & name, const T & default_value)
  {
    if (!this->has_parameter(name)) {
      this->declare_parameter<T>(name, default_value);
    }
    return this->get_parameter(name).get_value<T>();
  }

  rtabmap::Transform parseTransformParam(
    const std::string & name,
    const rtabmap::Transform & default_value)
  {
    const std::string value = getOrDeclare<std::string>(name, "");
    if (value.empty()) {
      return default_value;
    }
    if (!rtabmap::Transform::canParseString(value)) {
      RCLCPP_WARN(
        get_logger(),
        "参数 %s=\"%s\" 不能解析为 Transform，使用默认值。",
        name.c_str(), value.c_str());
      return default_value;
    }
    return rtabmap::Transform::fromString(value);
  }

  rtabmap::ParametersMap collectRtabmapParameters()
  {
    rtabmap::ParametersMap output;
    const auto result = this->list_parameters({}, 10);
    for (const auto & name : result.names) {
      // RTAB-Map core 参数统一是 Group/Name 格式，wrapper 自己的 ROS 参数不放进去。
      if (name.find('/') == std::string::npos || !this->has_parameter(name)) {
        continue;
      }

      const rclcpp::Parameter parameter = this->get_parameter(name);
      const std::string value = parameterToString(parameter);
      if (!value.empty()) {
        output[name] = value;
      }
    }
    return output;
  }

  cv::Mat imageToMono8(const ImageMsg::ConstSharedPtr & image_msg)
  {
    namespace enc = sensor_msgs::image_encodings;

    if (image_msg->encoding == enc::MONO8 || image_msg->encoding == "8UC1") {
      return cv_bridge::toCvShare(image_msg)->image.clone();
    }

    if ((image_msg->encoding == enc::MONO16 || image_msg->encoding == "16UC1") && convert_16bit_to_8bit_) {
      cv::Mat mono16 = cv_bridge::toCvShare(image_msg)->image;
      cv::Mat mono8;
      mono16.convertTo(mono8, CV_8UC1, 1.0 / 256.0);
      return mono8;
    }

    // 其它编码统一转 mono8，保证 OdometryOpenVINS 收到 CV_8UC1。
    return cv_bridge::toCvCopy(image_msg, enc::MONO8)->image;
  }

  rtabmap::Transform lookupLocalTransform(
    const std::string & sensor_frame,
    const rclcpp::Time & stamp,
    const rtabmap::Transform & fallback,
    const char * label)
  {
    if (!use_tf_transforms_ || sensor_frame.empty() || sensor_frame == frame_id_) {
      return fallback;
    }

    try {
      const auto transform_msg = tf_buffer_.lookupTransform(
        frame_id_,
        sensor_frame,
        stamp,
        tf2::durationFromSec(wait_for_transform_));
      return transformFromMsg(transform_msg.transform);
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        3000,
        "无法查询 %s 外参 %s -> %s：%s。使用参数/默认外参。",
        label, frame_id_.c_str(), sensor_frame.c_str(), error.what());
      return fallback;
    }
  }

  rtabmap::CameraModel makeCameraModel(
    const CameraInfoMsg::ConstSharedPtr & info_msg,
    const ImageMsg::ConstSharedPtr & image_msg)
  {
    const cv::Size image_size(
      static_cast<int>(image_msg->width),
      static_cast<int>(image_msg->height));

    rtabmap::Transform local_transform = lookupLocalTransform(
      info_msg->header.frame_id.empty() ? image_msg->header.frame_id : info_msg->header.frame_id,
      image_msg->header.stamp,
      camera_local_fallback_,
      "camera");

    return rtabmap::CameraModel(
      info_msg->header.frame_id,
      image_size,
      matrixFromCameraInfoK(*info_msg),
      matrixFromDistortion(*info_msg),
      matrixFromCameraInfoR(*info_msg),
      matrixFromCameraInfoP(*info_msg),
      local_transform);
  }

  rtabmap::IMU makeImu(const sensor_msgs::msg::Imu & msg)
  {
    const rtabmap::Transform local_transform = lookupLocalTransform(
      msg.header.frame_id,
      msg.header.stamp,
      imu_local_fallback_,
      "imu");

    const cv::Vec3d angular_velocity(
      msg.angular_velocity.x,
      msg.angular_velocity.y,
      msg.angular_velocity.z);
    const cv::Vec3d linear_acceleration(
      msg.linear_acceleration.x,
      msg.linear_acceleration.y,
      msg.linear_acceleration.z);

    return rtabmap::IMU(
      angular_velocity,
      covariance3x3(msg.angular_velocity_covariance),
      linear_acceleration,
      covariance3x3(msg.linear_acceleration_covariance),
      local_transform);
  }

  void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(odometry_mutex_);

    rtabmap::SensorData data(makeImu(*msg), 0, stampToSec(msg->header.stamp));
    odometry_->process(data, nullptr);
  }

  void imageCallback(
    const ImageMsg::ConstSharedPtr image_msg,
    const CameraInfoMsg::ConstSharedPtr info_msg)
  {
    cv::Mat image;
    try {
      image = imageToMono8(image_msg);
    } catch (const cv_bridge::Exception & error) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "图像转换失败：%s", error.what());
      return;
    }

    rtabmap::OdometryInfo info;
    rtabmap::Transform pose;
    {
      std::lock_guard<std::mutex> lock(odometry_mutex_);
      rtabmap::SensorData data(
        image,
        makeCameraModel(info_msg, image_msg),
        0,
        stampToSec(image_msg->header.stamp));
      pose = odometry_->process(data, &info);
    }

    if (pose.isNull()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "OpenVINS 尚未初始化或当前帧失败，暂不发布 odom。features=%d local_map=%d",
        info.features, info.localMapSize);
      return;
    }

    publishOdometry(*image_msg, pose, info);
  }

  void publishOdometry(
    const ImageMsg & image_msg,
    const rtabmap::Transform & pose,
    const rtabmap::OdometryInfo & info)
  {
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = image_msg.header.stamp;
    odom_msg.header.frame_id = odom_frame_id_;
    odom_msg.child_frame_id = frame_id_;
    fillPose(pose, odom_msg.pose.pose);
    fillCovariance(info.reg.covariance, odom_msg.pose.covariance, 2.0);

    float vx = 0.0f;
    float vy = 0.0f;
    float vz = 0.0f;
    float vroll = 0.0f;
    float vpitch = 0.0f;
    float vyaw = 0.0f;
    const rtabmap::Transform & velocity = odometry_->getVelocityGuess();
    if (!velocity.isNull()) {
      velocity.getTranslationAndEulerAngles(vx, vy, vz, vroll, vpitch, vyaw);
    }

    odom_msg.twist.twist.linear.x = vx;
    odom_msg.twist.twist.linear.y = vy;
    odom_msg.twist.twist.linear.z = vz;
    odom_msg.twist.twist.angular.x = vroll;
    odom_msg.twist.twist.angular.y = vpitch;
    odom_msg.twist.twist.angular.z = vyaw;
    fillCovariance(info.reg.covariance, odom_msg.twist.covariance, 1.0);

    odom_pub_->publish(odom_msg);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped transform_msg;
      transform_msg.header = odom_msg.header;
      transform_msg.child_frame_id = odom_msg.child_frame_id;
      transform_msg.transform.translation.x = odom_msg.pose.pose.position.x;
      transform_msg.transform.translation.y = odom_msg.pose.pose.position.y;
      transform_msg.transform.translation.z = odom_msg.pose.pose.position.z;
      transform_msg.transform.rotation = odom_msg.pose.pose.orientation;
      tf_broadcaster_->sendTransform(transform_msg);
    }
  }

  std::string frame_id_;
  std::string odom_frame_id_;
  bool publish_tf_ = true;
  double wait_for_transform_ = 0.2;
  bool use_tf_transforms_ = true;
  bool convert_16bit_to_8bit_ = true;

  rtabmap::Transform camera_local_fallback_;
  rtabmap::Transform imu_local_fallback_;

  std::unique_ptr<rtabmap::Odometry> odometry_;
  std::mutex odometry_mutex_;

  message_filters::Subscriber<ImageMsg> image_sub_;
  message_filters::Subscriber<CameraInfoMsg> camera_info_sub_;
  std::shared_ptr<Synchronizer> sync_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<MonoOpenVinsOdometryNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
