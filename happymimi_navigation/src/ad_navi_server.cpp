
//#include <list>
#include <vector>
#include <algorithm> // for std::find
// https://cpprefjp.github.io/reference/algorithm/find.html

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>

#include <happymimi_navigation/AdNaviSrv.h>
//#include <mimi_navi2/AdNaviAct.h>


// 受け取ったロケーションの座標をゴールポーズとしてMoveBaseへ送り、結果をレスポンス
class AdNaviServer
{
private:
  ros::NodeHandle nh;
  // ActionClient
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
  // ServiceServer
  rclcpp::Service<NavLoc>::SharedPtr navi_srv;
  // clear costmap
  //rclcpp::Service<nav2_msgs::srv::ClearCostmapAroundRobot>::SharedPtr clear_around_srv_;
  // Publisher
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr current_location_pub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr head_pub;
  //!!!
  std::string location_name = "NULL";
  //void init();

public:
  explicit Navi2LocationAcServer(): nh("")
  {
    // ACTION
    this->n2p_client =
      nh.actionClient<Nav2Pose>(this,"navigate_to_pose");
    // SERVICE
    this->navi2_srv = nh.adverticeServer<std_srvs::Empty>("/move_base/clear_costmaps");
    // PUB
    this->cur_loc_pub =
      this->advertisePublisher<std_msgs::msg::String>("/current_location", 1);
    //auto head_pub =
      this->create_publisher<std_msgs::msg::Float64>("/servo/head", 1);
    // location_list
    this->declare_parameter("location_list"); // declare
    rclcpp::Parameter location_list_param = this->get_parameter("location_list"); //get
    std::vector<std::string> location_list = location_list_param.as_string_array(); // declare type
    //std::vector<double> coord_list
  }

  bool searchLocationName(const std::shared_ptr<NavLoc> &request,
                          const std::shared_ptr<NavLoc> &response) {
    //!!!
    if ( std::find(location_list.begin(), location_list.end(),
                   request->location_name) != location_list.end() ){
      //location_name = request->location_name;
      RCLCPP_INFO(this->get_logger(), "%f", location_list[request->location_name]);
      return this->sendGoalPose(location_list[request->location_name]);
    } else {
      RCLCPP_INFO(this->get_logger(), "<%s> doesn't exist.", request->location_name);
      return response->result = false; //!!!
    }
  }

  void sendGoalPose(const std::vector<double> coord_list) {
    //　wait for action server
    while (!this->n2p_client->wait_for_action_server()) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        rclcpp::shutdown();
      } //!!!
      RCLCPP_INFO(get_logger(), "waiting for the action server...  nav2_msgs/NavigateToPose");
      rclcpp::sleep_for(500ms);
    }
    // set goal_pose
    auto goal = Nav2Pose::Goal();
    goal.pose.header.stamp = this->now();
    goal.pose.header.frame_id = "map"; //-------------
    goal.pose.pose.position.x = coord_list[0];
    goal.pose.pose.position.y = coord_list[1];
    goal.pose.pose.orientation.z = coord_list[2];
    goal.pose.pose.orientation.w = coord_list[3];
    // declare goal options
    // future...MultiThread
    auto goal_options = rclcpp_action::Client<Nav2Pose>::SendGoal Options();
    goal_options.goal_response_callback =
      std::bind(&Navi2LocationAcServer::naviResponseCB, this, _1);
    goal_options.feedback_callback =
      std::bind(&Navi2LocationAcServer::naviFeedbackCB, this, _1, _2);
    goal_options.result_callback =
      std::bind(&Navi2LocationAcServer::naviResultCB, this, _1);

    //!!! remaine absolute or relative movement distance

    //!!! clear costmap
    //RCLCPP_INFO(get_logger(), "clearing cost map...");

    // Start Navigation
    this->head_pub->publish(0.0);
    n2p_client->async_send_goal(goal, goal_options);
  }

  void naviResponseCB(std::shared_future<Nav2PoseCGH::SharedPtr> future) {
    auto gh = future.get();
    if (!gh) RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server :(");
    else  RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result :)");
  }
  void naviFeedbackCB(Nav2PoseCGH::SharedPtr,
                      const std::shared_ptr<const Nav2Pose::Feedback> feedback) {
    RCLCPP_INFO(this->get_logger(), "Distance remainig: %f,\n navi_time: %f ",
      feedback->distance_remaining, feedback->navigation_time);
    //!!! if its taking too long executeing time inspite so close to goal
  }
  void naviResultCB(const Nav2PoseCGH::WrappedResult &result) {
    switch (result.code){
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Navigation Success!");
        this->current_location_pub->publish(this->location_name); //!!!

        return result;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
  }

  //void execute(){
  //}
};

void execute(){
  //!!! do this part in above class space
  auto node = rclcpp::Node::make_shared<Navi2LocationAcServer>(); //type: std::shared_ptr<rclcpp::Node>
  rclcpp::WallRate rate(333ms);
  Navi2LocationAcServer nlas;
  while (rclcpp::ok()) {

    rclcpp::spin_some(node);
    rate.sleep(); //rclcpp::sleep_for(33)
  }

  rclcpp::shutdown();
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  execute();
  return 0;
}
