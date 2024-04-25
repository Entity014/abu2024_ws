#include "tb3_autonomy/autonomy_node.h"
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
// #include <behaviortree_cpp/loggers/groot2_publisher.h>

using namespace std::chrono_literals;

const std::string bt_xml_dir =
    ament_index_cpp::get_package_share_directory("abu_core") + "/bt_xml";

AutonomyNode::AutonomyNode(const std::string &nodeName) : Node(nodeName)
{
    this->declare_parameter("location_file", "none");
    RCLCPP_INFO(get_logger(), "Init done");
}

void AutonomyNode::setup()
{
    create_behavior_tree();

    const auto timer_period = 100ms;
    timer_ = this->create_wall_timer(timer_period, std::bind(&AutonomyNode::update_behavior_tree, this));
}

void AutonomyNode::create_behavior_tree()
{
    BT::BehaviorTreeFactory factory;

    BT::NodeBuilder builder = [=](const std::string &name, const BT::NodeConfiguration &config)
    {
        return std::make_unique<GoToPose>(name, config, shared_from_this());
    };

    factory.registerBuilder<GoToPose>("GoToPose", builder);
    tree_ = factory.createTreeFromFile(bt_xml_dir + "/tree.xml");
    RCLCPP_INFO(this->get_logger(), "set ZMQ Publisher");
    // BT::Groot2Publisher publisher(tree_, 1667);
    BT::PublisherZMQ publisher_zmq(tree_);
}

void AutonomyNode::update_behavior_tree()
{
    // BT::NodeStatus tree_status = tree_.tickWhileRunning();
    BT::NodeStatus tree_status = tree_.tickRoot();

    if (tree_status == BT::NodeStatus::RUNNING)
    {
        return;
    }
    else if (tree_status == BT::NodeStatus::SUCCESS)
    {
        RCLCPP_INFO(this->get_logger(), "Finished Navigation");
    }
    else if (tree_status == BT::NodeStatus::FAILURE)
    {
        RCLCPP_INFO(this->get_logger(), "Navigation Failed");
        timer_->cancel();
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AutonomyNode>("autonomy_node");
    node->setup();

    rclcpp::spin(node);
    rclcpp::shutdown();
}