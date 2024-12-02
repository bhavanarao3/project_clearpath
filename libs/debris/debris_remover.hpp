#pragma once

#include <chrono>
#include <fstream>
#include <functional>
#include <gazebo_msgs/srv/delete_entity.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <list>
#include <memory>
#include <rclcpp/client.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

using TwistVelPub = rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr;
using PoseSub = rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr;
using LaserSub = rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr;
using Timer = rclcpp::TimerBase::SharedPtr;
using namespace std::chrono_literals;

class DebrisRemover : public rclcpp::Node {
 public:
  DebrisRemover();

  virtual bool remove_debris(std::string);

  int get_recent_debris();

  // Getter for debris_counter
  int get_debris_counter() const { return debris_counter; }

  // Setter for debris_counter
  void set_debris_counter(int count) { debris_counter = count; }

  // Setter for unspawn_client
  void set_unspawn_client(rclcpp::Client<gazebo_msgs::srv::DeleteEntity>::SharedPtr client) {
    unspawn_client = client;
  }

  // Setter for remove_debris_node
  void set_remove_debris_node(rclcpp::Node::SharedPtr node) {
    remove_debris_node = node;
  }

  // Getter for remove_debris_node
  rclcpp::Node::SharedPtr get_remove_debris_node() const {
    return remove_debris_node;
  }

 private:
  sensor_msgs::msg::LaserScan scan_;
  rclcpp::Client<gazebo_msgs::srv::DeleteEntity>::SharedPtr unspawn_client;
  Timer timer_;
  rclcpp::Node::SharedPtr remove_debris_node;
  int debris_counter;
  std::list<int> debris_idx{};
};