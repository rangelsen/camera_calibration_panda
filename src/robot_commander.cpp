#include <queue>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <std_msgs/Int16.h>

////////////////////////////////////////////////////////////////////////////////
static std::queue<geometry_msgs::Pose> targets_;

////////////////////////////////////////////////////////////////////////////////
void update_pose(geometry_msgs::Pose new_target) {

    targets_.push(new_target);
}

////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv) {

    ros::init(argc, argv, "robot");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Subscriber pose_sub = nh.subscribe("pose", 100, update_pose);
    ros::Publisher panda_status_pub = nh.advertise<std_msgs::Int16>("panda_status", 100);

    static const std::string PLANNING_GROUP_ = "panda_arm";

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP_);
    move_group.startStateMonitor();

    while (true) {

        for (uint32_t i = 0; i < targets_.size(); i++) {

            std_msgs::Int16 status_msg;
            status_msg.data = 1;
            panda_status_pub.publish(status_msg);
            geometry_msgs::Pose target = targets_.front();
            targets_.pop();

            move_group.setPoseTarget(target);
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            moveit::planning_interface::MoveItErrorCode success = move_group.plan(my_plan);

            if (success == moveit::planning_interface::MoveItErrorCode::SUCCESS) {

                move_group.move();

                ROS_INFO("Signalling stop");
                std_msgs::Int16 status_msg;
                status_msg.data = 0;
                panda_status_pub.publish(status_msg);
            }
            else
                ROS_INFO("Error: No path available to target pose");
        }
    }

    return 0;
}
