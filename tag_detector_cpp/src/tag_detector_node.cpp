/*  tag_detector_node.cpp  */

#include <Eigen/Core>
#include <Eigen/Geometry>            // <-- Eigen FIRST

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <apriltag_msgs/msg/april_tag_detection.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

extern "C" {                       // standalone AprilTag C lib
  #include <apriltag.h>
  #include <tag36h11.h>
}

using apriltag_msgs::msg::AprilTagDetection;
using apriltag_msgs::msg::AprilTagDetectionArray;
using geometry_msgs::msg::PoseStamped;
using sensor_msgs::msg::CameraInfo;
using sensor_msgs::msg::Image;

class TagDetectorNode : public rclcpp::Node
{
public:
  TagDetectorNode() : Node("tag_detector")
  {
    tag_edge_     = declare_parameter("tag_edge_m", 0.09);   // 90 mm
    camera_frame_ = declare_parameter("camera_frame", std::string("camera"));

    /* ------------------------------------------------------------------ */
    tf_ = tag36h11_create();
    td_ = apriltag_detector_create();
    apriltag_detector_add_family_bits(td_, tf_, 1);
    td_->quad_decimate = 2.0;
    td_->nthreads      = 2;
    /* ------------------------------------------------------------------ */

    sub_info_ = create_subscription<CameraInfo>(
        "/camera/camera_info", rclcpp::SensorDataQoS(),
        [this](CameraInfo::SharedPtr m){ cameraInfoCb(std::move(m)); });

    sub_image_ = create_subscription<Image>(
        "/camera/image_raw", rclcpp::SensorDataQoS(),
        [this](Image::SharedPtr m){ imageCb(std::move(m)); });

    pub_dets_ = create_publisher<AprilTagDetectionArray>("/tag_detections", 10);
    pub_pose_ = create_publisher<PoseStamped>("/tag_pose", 10);
  }

  ~TagDetectorNode() override
  {
    apriltag_detector_destroy(td_);
    tag36h11_destroy(tf_);
  }

private:
  /* -----------------------  Callbacks  -------------------------------- */
  void cameraInfoCb(const CameraInfo::SharedPtr msg)
  {
    camera_frame_ = msg->header.frame_id;

    camera_matrix_ = (cv::Mat_<double>(3,3) <<
       msg->k[0], msg->k[1], msg->k[2],
       msg->k[3], msg->k[4], msg->k[5],
       msg->k[6], msg->k[7], msg->k[8]);

    dist_coeffs_ = (cv::Mat_<double>(1,5) <<
       msg->d[0], msg->d[1], msg->d[2], msg->d[3], msg->d[4]);
  }

  void imageCb(const Image::SharedPtr msg)
  {
    if (camera_matrix_.empty()) return;      // wait for intrinsics

    cv::Mat gray;
    try {
      gray = cv_bridge::toCvShare(msg, "mono8")->image;
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge: %s", e.what());
      return;
    }

    /* ---- AprilTag input image --------------------------------------- */
    image_u8_t img_hdr = {               //  <--  NO “const” here
        gray.cols,       // width
        gray.rows,       // height
        gray.cols,       // stride
        gray.data        // buffer pointer
    };

    zarray_t *detections = apriltag_detector_detect(td_, &img_hdr);   // ✅


    AprilTagDetectionArray det_array;
    det_array.header.stamp    = msg->header.stamp;
    det_array.header.frame_id = camera_frame_;

    const double half = tag_edge_ / 2.0;

    for (int i = 0; i < zarray_size(detections); ++i)
    {
      apriltag_detection_t *d;  zarray_get(detections, i, &d);

      std::vector<cv::Point2f> imgPts = {
        {float(d->p[0][0]), float(d->p[0][1])},
        {float(d->p[1][0]), float(d->p[1][1])},
        {float(d->p[2][0]), float(d->p[2][1])},
        {float(d->p[3][0]), float(d->p[3][1])}
      };
      std::vector<cv::Point3f> objPts = {
        {-float(half), -float(half), 0},
        { float(half), -float(half), 0},
        { float(half),  float(half), 0},
        {-float(half),  float(half), 0}
      };

      cv::Mat rvec, tvec;
      cv::solvePnP(objPts, imgPts, camera_matrix_, dist_coeffs_, rvec, tvec);

      cv::Mat R;  cv::Rodrigues(rvec, R);
      Eigen::Matrix3d Re;  cv::cv2eigen(R, Re);
      Eigen::Quaterniond q(Re);

      PoseStamped pose;
      pose.header = det_array.header;
      pose.pose.position.x = tvec.at<double>(0);
      pose.pose.position.y = tvec.at<double>(1);
      pose.pose.position.z = tvec.at<double>(2);
      pose.pose.orientation.x = q.x();
      pose.pose.orientation.y = q.y();
      pose.pose.orientation.z = q.z();
      pose.pose.orientation.w = q.w();
      pub_pose_->publish(pose);

      AprilTagDetection det;
      det.id = static_cast<std::int32_t>(d->id);
      det_array.detections.push_back(det);
    }

    apriltag_detections_destroy(detections);
    pub_dets_->publish(det_array);
  }

  /* -----------------------  Members  ---------------------------------- */
  double tag_edge_{0.09};
  std::string camera_frame_{"camera"};

  cv::Mat camera_matrix_, dist_coeffs_;

  apriltag_family_t  *tf_{nullptr};
  apriltag_detector_t* td_{nullptr};

  rclcpp::Subscription<CameraInfo>::SharedPtr sub_info_;
  rclcpp::Subscription<Image>::SharedPtr      sub_image_;
  rclcpp::Publisher<AprilTagDetectionArray>::SharedPtr pub_dets_;
  rclcpp::Publisher<PoseStamped>::SharedPtr            pub_pose_;
};

/* --------------------------------------------------------------------- */
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TagDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
