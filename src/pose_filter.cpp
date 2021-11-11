#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <eigen3/Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

#include "Iir.h"

#include <signal.h>

using namespace std::chrono_literals;

class ArucoFilterPose : public rclcpp::Node {

  using TransformStamped = geometry_msgs::msg::TransformStamped;
  using WrenchStamped = geometry_msgs::msg::WrenchStamped;

public:

  ArucoFilterPose(const std::string node_name, rclcpp::NodeOptions options)
  : Node(node_name, options)
  {
    aruco_pose_raw_topic_ = this->declare_parameter("subscribers.aruco_pose_raw_measures_prefix", "/target_tracking/camera_to_marker_transform/marker_");
    aruco_presence_topic_ = this->declare_parameter("subscribers.aruco_presence_prefix", "/target_tracking/camera_to_marker_presence/marker_");

    camera_link_frame = this->declare_parameter("frames.camera", "wrist_camera_link");

    stable_link_frame_ = this->declare_parameter("frames.stable_link_prefix", "stable_link_");
    
    link_base__T__wrist_camera_link = Eigen::Matrix4d::Identity();

    aruco_pose_filter_topic_ = this->declare_parameter("publishers.aruco_pose_filter_measures_prefix:", "/target_tracking/camera_to_marker_transform/filter/marker_");

    marker_id_ = this->declare_parameter("marker_id", "0");

    std::cout << std::endl << "Marker_id: " << marker_id_ << std::endl;

    aruco_pose_raw_topic_.append(marker_id_);
    std::cout << std::endl << "aruco_pose_raw_topic_: " << aruco_pose_raw_topic_ << std::endl;
    aruco_presence_topic_.append(marker_id_);
    stable_link_frame_.append(marker_id_);
    aruco_pose_filter_topic_.append(marker_id_);

    aruco_pose_filter_pub_ = this->create_publisher<TransformStamped>(aruco_pose_filter_topic_, 1);

    N_measures = this->declare_parameter<int>("median_filter.n_samples", 10);

    counter = 0;

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    for(int i = 0; i < N_measures; i++){
      last_N_measures_x.push_back(0.0);
      last_N_measures_y.push_back(0.0);
      last_N_measures_z.push_back(0.0);
      last_N_measures_rx.push_back(0.0);
      last_N_measures_ry.push_back(0.0);
      last_N_measures_rz.push_back(0.0);
      last_N_measures_rw.push_back(0.0);
    }
    
    use_median = this->declare_parameter<bool>("median_filter.use_median", true);

    std::cout << "use_median: " << use_median << std::endl;
    if(use_median)
      start_publishing = false;
    else
      start_publishing = true;
    publish = false;


    float sampling_rate = this->declare_parameter<float>("butterworth_filter.sampling_rate", 10.0);
    float cutoff_frequency = this->declare_parameter<float>("butterworth_filter.cutoff_frequency", 1.0);

    for (auto filter: filters){
      filter.setup(order, sampling_rate, cutoff_frequency);
    } 

    aruco_pose_raw_sub_ = this->create_subscription<TransformStamped>(aruco_pose_raw_topic_, 1, 
        std::bind(&ArucoFilterPose::aruco_pose_raw_callback, this, std::placeholders::_1));
      

    aruco_presence_sub_ = this->create_subscription<std_msgs::msg::Bool>(aruco_presence_topic_, 1, 
        std::bind(&ArucoFilterPose::aruco_presence_callback, this, std::placeholders::_1));

    if(use_median){
      hz_filter = this->declare_parameter<double>("filter_rate", 10.0);
      filter_publisher_timer_ = this->create_wall_timer(1000ms / hz_filter, 
        std::bind(&ArucoFilterPose::publish_filtering, this));
    }
    
    std::cout << "Aruco Pose Filter Start" << std::endl;
  }

private:


  void aruco_pose_raw_callback(const geometry_msgs::msg::TransformStamped::SharedPtr msg) {

    if (use_median)
    {
      last_N_measures_x[counter] = msg->transform.translation.x;
      last_N_measures_y[counter] = msg->transform.translation.y;
      last_N_measures_z[counter] = msg->transform.translation.z;
      last_N_measures_rx[counter] = msg->transform.rotation.x;
      last_N_measures_ry[counter] = msg->transform.rotation.y;
      last_N_measures_rz[counter] = msg->transform.rotation.z;
      last_N_measures_rw[counter] = msg->transform.rotation.w;
      counter = (counter + 1) % N_measures;

      if (counter == 0)
      {
        start_publishing = true;
      }
    }
    else if(publish){
      TransformStamped filtered_pose;
      filtered_pose.header.stamp = this->get_clock()->now();
      filtered_pose.header.frame_id = camera_link_frame;
      filtered_pose.child_frame_id = stable_link_frame_;

      filtered_pose.transform.translation.x = (float) filters[0].filter(msg->transform.translation.x);
      filtered_pose.transform.translation.y = (float) filters[1].filter(msg->transform.translation.y);
      filtered_pose.transform.translation.z = (float) filters[2].filter(msg->transform.translation.z);
      filtered_pose.transform.rotation.x = (float) filters[3].filter(msg->transform.rotation.x);
      filtered_pose.transform.rotation.y = (float) filters[4].filter(msg->transform.rotation.y);
      filtered_pose.transform.rotation.z = (float) filters[5].filter(msg->transform.rotation.z);
      filtered_pose.transform.rotation.w = (float) filters[6].filter(msg->transform.rotation.w);
      aruco_pose_filter_pub_->publish(filtered_pose);

      tf_broadcaster_->sendTransform(filtered_pose);
    }
  }

  void aruco_presence_callback(const std_msgs::msg::Bool::SharedPtr msg) {

      if(msg->data == true)
      {
        this->publish = true;
      }
      else
        this->publish = false;
  }


  void publish_filtering(){
    if(use_median and start_publishing and publish){
      TransformStamped filtered_pose;
      filtered_pose.header.stamp = this->get_clock()->now();
      filtered_pose.header.frame_id = camera_link_frame;
      filtered_pose.child_frame_id = stable_link_frame_;

      auto m = last_N_measures_x.begin() + last_N_measures_x.size()/2;
      std::nth_element(last_N_measures_x.begin(), m, last_N_measures_x.end());
      filtered_pose.transform.translation.x = (float) filters[0].filter(last_N_measures_x[last_N_measures_x.size()/2]);

      std::nth_element(last_N_measures_y.begin(), m, last_N_measures_y.end());
      filtered_pose.transform.translation.y = (float) filters[1].filter(last_N_measures_y[last_N_measures_y.size()/2]);

      std::nth_element(last_N_measures_z.begin(), m, last_N_measures_z.end());
      filtered_pose.transform.translation.z = (float) filters[2].filter(last_N_measures_z[last_N_measures_z.size()/2]);

      std::nth_element(last_N_measures_rx.begin(), m, last_N_measures_rx.end());
      filtered_pose.transform.rotation.x = (float) filters[3].filter(last_N_measures_rx[last_N_measures_rx.size()/2]);

      std::nth_element(last_N_measures_ry.begin(), m, last_N_measures_ry.end());
      filtered_pose.transform.rotation.y = (float) filters[4].filter(last_N_measures_ry[last_N_measures_ry.size()/2]);

      std::nth_element(last_N_measures_rz.begin(), m, last_N_measures_rz.end());
      filtered_pose.transform.rotation.z = (float) filters[5].filter(last_N_measures_rz[last_N_measures_rz.size()/2]);

      std::nth_element(last_N_measures_rw.begin(), m, last_N_measures_rw.end());
      filtered_pose.transform.rotation.w = (float) filters[6].filter(last_N_measures_rw[last_N_measures_rw.size()/2]);
    
      aruco_pose_filter_pub_->publish(filtered_pose);

      tf_broadcaster_->sendTransform(filtered_pose);
    }
    
  }



    

  std::string aruco_pose_raw_topic_;
  std::string aruco_presence_topic_;
  std::string aruco_pose_filter_topic_;
  std::string camera_link_frame = "wrist_camera_link";
  std::string base_link_frame_ = "link_base";
  std::string stable_link_frame_ = "stable_link";
  std::string marker_id_ = "0";

  Eigen::Matrix4d link_base__T__wrist_camera_link;

  geometry_msgs::msg::TransformStamped transformStamped;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_{nullptr};
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};

  rclcpp::Publisher<TransformStamped>::SharedPtr aruco_pose_filter_pub_{nullptr};
  rclcpp::Subscription<TransformStamped>::SharedPtr aruco_pose_raw_sub_{nullptr};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr aruco_presence_sub_{nullptr};

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::TimerBase::SharedPtr filter_publisher_timer_{nullptr};
  rclcpp::TimerBase::SharedPtr tf_listener_timer_{nullptr};

  std::vector<WrenchStamped> last_N_measures;
  std::vector<double> last_N_measures_x;
  std::vector<double> last_N_measures_y;
  std::vector<double> last_N_measures_z;
  std::vector<double> last_N_measures_rx;
  std::vector<double> last_N_measures_ry;
  std::vector<double> last_N_measures_rz;
  std::vector<double> last_N_measures_rw;

  int counter;
  int N_measures;

  double hz_filter;

  bool use_median;

  bool start_publishing;
  bool publish;

  static const int order = 3;

  std::array<Iir::Butterworth::LowPass<order>, 7> filters;

  
};



int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();

  rclcpp::NodeOptions node_options;

  node_options.use_intra_process_comms(true);
  auto node = std::make_shared<ArucoFilterPose>("pose_filter", node_options);

  executor->add_node(node);
  executor->spin(); 

  rclcpp::shutdown();

  return 0;
}
