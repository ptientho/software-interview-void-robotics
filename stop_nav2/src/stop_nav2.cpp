#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nav2_msgs/action/follow_path.hpp"

class StopNav2: public rclcpp::Node{

public:
    using FollowPath = nav2_msgs::action::FollowPath;

    StopNav2(std::string node_name) : Node(node_name){

        this->wait_for_timeout_ = std::chrono::milliseconds(2000);
        this->callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
        this->callback_group_executor_.add_callback_group(this->callback_group_, this->get_node_base_interface());

        createActionClient("follow_path");

        // send cancel request
        this->timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&StopNav2::cancelGoal, this));
    }

    void createActionClient(const std::string& action_name){
        this->action_name_ = action_name;
        this->client_ptr_ = rclcpp_action::create_client<FollowPath>(this,action_name);

        // wait for action server running
        if (!this->client_ptr_->wait_for_action_server(this->wait_for_timeout_)){
            RCLCPP_ERROR(this->get_logger(), "%s action server not available after waiting for 2 s", this->action_name_.c_str());
            return;
        }
    }

    void cancelGoal() {
        auto future_cancel = this->client_ptr_->async_cancel_all_goals();
        this->timer_->cancel();

        RCLCPP_INFO(this->get_logger(), "Canceled all goals. Stop the robot.");

    }

private:
    rclcpp_action::Client<FollowPath>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::milliseconds wait_for_timeout_;
    std::string action_name_;

    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
};

int main(int argc, char** argv){

    rclcpp::init(argc, argv);
    auto node = std::make_shared<StopNav2>("stop_nav2_client");

    try
    {
        rclcpp::spin(node);
    }
    catch(std::exception& e)
    {
        RCLCPP_INFO(node->get_logger(), "Shutting down...");
    }
    
    rclcpp::shutdown();
    return 0;
}