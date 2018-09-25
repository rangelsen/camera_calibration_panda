#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Int16.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
// #include <opencv2/aruco/charuco.hpp>

////////////////////////////////////////////////////////////////////////////////
typedef enum {

    STOPPED = 0,
    MOVING = 1,
} PandaStatus;

static cv::Mat endeff_pose_;
static PandaStatus panda_status_;

////////////////////////////////////////////////////////////////////////////////
/*
void updateRobotState(*some state msg*) {

    set endeff_pose_
}
*/

////////////////////////////////////////////////////////////////////////////////
geometry_msgs::Pose generateNextTarget(cv::Mat current_pose) {

    geometry_msgs::Pose target;
    target.position.x = 0.3;
    target.position.y = 0.3;
    target.position.z = 0.3;
    target.orientation.x = 1.0;

    return target;
}

////////////////////////////////////////////////////////////////////////////////
void updatePandaStatus(std_msgs::Int16 status) {

    panda_status_ = (PandaStatus) status.data;
}

////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv) {

    ros::init(argc, argv, "main");
    ros::NodeHandle nh;

    /*
    ros::Subscriber panda_pose_sub = nh.subscribe("*some franka topic*",
        100, update_robot_state);
    */

    ros::Subscriber panda_status_sub = nh.subscribe("status", 100, updatePandaStatus);
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::Pose>("pose", 100);

    uint32_t i = 0;

    panda_status_ = MOVING;

    while (ros::ok()) {

        if (panda_status_ == STOPPED) {

            ROS_INFO("Capturing, estimating pose, and writing to file");
            /*
            cv::cvtColor(image, image_copy, cv::COLOR_GRAY2RGB);
            cv::Mat board_pose = calib.EstimateCharucoPose(image_copy, &camera);

            if (!board_pose.empty()) {

                cv::imwrite("res/calib-images/ir" + std::to_string(i) + ".png", image);

                Util::writeToFile("res/board-poses.csv", board_pose, i);
                Util::writeToFile("res/endeffector_poses.csv", cv_endeff_pose, i);

                i++;
            }
            */
            
            geometry_msgs::Pose next_target = generateNextTarget(endeff_pose_);
            pose_pub.publish(next_target);

            ROS_INFO("Published next target");
        }
        else if (panda_status_ = MOVING) { }

        ros::spinOnce();
    }

    return 0;
}

/// @file

