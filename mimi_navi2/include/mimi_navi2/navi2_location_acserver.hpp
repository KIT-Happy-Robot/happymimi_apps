
#include <memory>
#include <chrono>
#include <iostream>
//#include <list>
#include <vector>
#include <algorithm> // for std::find
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/time.hpp"
#include <mimi_navi2/action/nav2_location_ac.hpp> //!!

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

//-----------

//-----------

enum NaviState {
  SUCCEEDED,
  ACTIVE,
  ABORTED,
  REJECTED,
  CANCELED,
  STUCKED
}

class Navi2LocationAcServer : public rclcpp::Node
{
private:
  // Alias
  using Nav2Pose = nav2_msgs::action::NavigateToPose;
  using Nav2PoseCGH = rclcpp_action::ClientGoalHandle<Nav2Pose>;
  using NavLocAc = mimi_navi2::action::Navi2LocationAc; //!!
  // ActionClient
  rclcpp_action::Client<Nav2Pose>::SharedPtr n2p_client;
  // ServiceServer
  rclcpp::Service<NavLoc>::SharedPtr navi2_srv;
  // clear costmap
  //rclcpp::Service<nav2_msgs::srv::ClearCostmapAroundRobot>::SharedPtr clear_around_srv_;
  // Publisher
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr current_location_pub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr head_pub;
  std::string location_name = "NULL";  //!!!


public:
  explicit Navi2LocationAcServer(): Node("navi2_location_acserver")
  {
    // ACTION
    this->n2p_client =
      rclcpp_action::create_client<Nav2Pose>(this,"navigate_to_pose");
    // SERVICE
    this->navi2_srv = this->create_service<NavLoc>(
      "/navi2_location_server", 
      std::bind(&Navi2LocationAcServer::searchLocationName, this, _1, _2) );
    // PUB
    this->current_location_pub =
      this->create_publisher<std_msgs::msg::String>("/current_location", 1);
    auto head_pub = 
      this->create_publisher<std_msgs::msg::Float64>("/servo/head", 1);
    // location_list
    this->declare_parameter("location_list"); // declare
    rclcpp::Parameter location_list_param = this->get_parameter("location_list"); //get
    std::vector<std::string> location_list = location_list_param.as_string_array(); // declare type
    //std::vector<double> coord_list
  }

};

