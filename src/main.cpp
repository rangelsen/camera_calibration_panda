#include <time.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Int16.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <Eigen/Geometry>

#include "panda_status.h"
#include "util.hpp"

#define DEBUG true

////////////////////////////////////////////////////////////////////////////////
static cv::Mat endeff_pose_;
static PandaStatus panda_status_;

////////////////////////////////////////////////////////////////////////////////
geometry_msgs::Pose generateNextTarget(cv::Mat current_pose) {

    double x, y, z;
    bool valid_pos = Util::getUserPosition(&x, &y, &z);

    geometry_msgs::Pose target;

    if (valid_pos) {

        target.position.x = x;
        target.position.y = y;
        target.position.z = z;

        Eigen::Quaterniond q_target = Util::UniformRandom();
        target.orientation.x = q_target.x();
        target.orientation.y = q_target.y();
        target.orientation.z = q_target.z();
        target.orientation.w = q_target.w();
    }
    else {

        target.position.x = current_pose.at<double>(0, 3);
        target.position.y = current_pose.at<double>(1, 3);
        target.position.z = current_pose.at<double>(2, 3);
    }

    return target;
}

////////////////////////////////////////////////////////////////////////////////
void updatePandaStatus(std_msgs::Int16 status) {

    panda_status_ = (PandaStatus) status.data;
}

////////////////////////////////////////////////////////////////////////////////
void calibrate() {

    ROS_INFO("Working");

#ifdef DEBUG

    sleep(10);
#else

    cv::cvtColor(image, image_copy, cv::COLOR_GRAY2RGB);
    cv::Mat board_pose = calib.EstimateCharucoPose(image_copy, &camera);

    if (!board_pose.empty()) {

        cv::imwrite("res/calib-images/ir" + std::to_string(i) + ".png", image);

        Util::writeToFile("res/board-poses.csv", board_pose, i);
        Util::writeToFile("res/endeffector_poses.csv", cv_endeff_pose, i);

        i++;
    }
#endif
}

////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv) {

    srand(time(NULL));

    ros::init(argc, argv, "main");
    ros::NodeHandle nh;

    ros::Subscriber panda_status_sub = nh.subscribe("panda_status", 100, updatePandaStatus);
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::Pose>("pose", 100);

    uint32_t i = 0;
    bool is_first = true;

    while (ros::ok()) {

        if (panda_status_ == PANDA_STOPPED) {

            if (is_first)
                is_first = false;
            else
                calibrate();

            geometry_msgs::Pose next_target = generateNextTarget(endeff_pose_);
            pose_pub.publish(next_target);

            ROS_INFO("Published next target");
        }

        ros::spinOnce();
    }

    return 0;
}

/// @file

