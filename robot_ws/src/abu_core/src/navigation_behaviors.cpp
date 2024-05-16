#include "tb3_autonomy/navigation_behaviors.h"

GoToPose::GoToPose(const std::string &name,
                   const BT::NodeConfiguration &config,
                   rclcpp::Node::SharedPtr node_ptr)
    : BT::StatefulActionNode(name, config), node_ptr_(node_ptr)
{
    action_client_ptr_ = rclcpp_action::create_client<NavigateToPose>(node_ptr_, "/navigate_to_pose");
    done_flag_ = false;
}

BT::PortsList GoToPose::providedPorts()
{
    return {BT::InputPort<std::string>("loc")};
}

BT::NodeStatus GoToPose::onStart()
{
    // BT::Expected<std::string> loc = getInput<std::string>("loc");
    BT::Optional<std::string> loc = getInput<std::string>("loc");
    const std::string location_file = node_ptr_->get_parameter("location_file").as_string();

    YAML::Node locations = YAML::LoadFile(location_file);

    std::vector<float> pose = locations[loc.value()].as<std::vector<float>>();

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(&GoToPose::nav_to_pose_callback, this, std::placeholders::_1);

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.pose.position.x = pose[0];
    goal_msg.pose.pose.position.y = pose[1];

    tf2::Quaternion q;
    q.setRPY(0, 0, pose[2]);
    q.normalize();
    goal_msg.pose.pose.orientation = tf2::toMsg(q);

    done_flag_ = false;
    action_client_ptr_->async_send_goal(goal_msg, send_goal_options);
    RCLCPP_INFO(node_ptr_->get_logger(), "Sent Goal to Nav2\n");
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GoToPose::onRunning()
{
    if (done_flag_)
    {
        RCLCPP_INFO(node_ptr_->get_logger(), "[%s] Goal reached\n", this->name());
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        return BT::NodeStatus::RUNNING;
    }
}

void GoToPose::nav_to_pose_callback(const GoalHandleNav::WrappedResult &result)
{
    if (result.result)
    {
        done_flag_ = true;
    }
}