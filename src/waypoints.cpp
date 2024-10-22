#include "waypoints.h"

#include <rclcpp/rclcpp.hpp>
#include <exception>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>

using namespace geometry_msgs::msg;
using namespace std::chrono_literals;

class Control : public rclcpp::Node
{
public:
  Control() : Node("waypoints")
  {
    set_parameter(rclcpp::Parameter("use_sim_time", true));

    // load waypoints and thresholds
    Waypoint::load(waypoints, position_thr, orientation_thr);

    pose_cmd.header.frame_id = "world";
    pub = create_publisher<PoseStamped>("/bluerov2/cmd_pose", 1);

    static auto timer{create_wall_timer(10ms, [&]()
      {
        if(!buffer.canTransform("world", "bluerov2/base_link", tf2::TimePointZero))
          return;
        const auto transform{buffer.lookupTransform("world", "bluerov2/base_link", tf2::TimePointZero)};
        trackWaypoint(transform.transform);
      })};
  }

private:

  std::vector<Waypoint> waypoints;
  double position_thr, orientation_thr;

  tf2_ros::Buffer buffer{get_clock()};
  tf2_ros::TransformListener tl{buffer};

  PoseStamped pose_cmd;
  rclcpp::Publisher<PoseStamped>::SharedPtr pub;


  void trackWaypoint(const Transform &pose)
  {
    static auto cur_wp{0};

    // TODO update cur_wp to cycle through the waypoints when the current one is reached
    auto x_error = waypoints[cur_wp].x - pose.translation.x;
    auto y_error = waypoints[cur_wp].y - pose.translation.y;
    auto z_error = waypoints[cur_wp].z - pose.translation.z;
    auto distance_error = std::sqrt(x_error*x_error + y_error*y_error + z_error*z_error);

    auto yaw_error = waypoints[cur_wp].theta - std::atan2(pose.rotation.z, pose.rotation.w)*2;

    yaw_error = std::fmod(yaw_error + M_PI, 2 * M_PI); // Shift to [0, 2*pi]
    if (yaw_error < 0) {
        yaw_error += 2 * M_PI; // Ensure positive
    }
    yaw_error -= M_PI; // Shift to [-pi, pi]

    // Going to 0 if the list of waypoints finishes
    if ( yaw_error < orientation_thr && distance_error < position_thr){
        if (cur_wp != waypoints.size() - 1){
            cur_wp +=1;
        }
        else{
            cur_wp = 0;
        }
     }


//    std::cout<<"cur_wp: "<<cur_wp<<"distance_error"


    waypoints[cur_wp].write(pose_cmd);
    pose_cmd.header.stamp = get_clock()->now();
    pub->publish(pose_cmd);    
  }
};



int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Control>());
  rclcpp::shutdown();
  return 0;
}
