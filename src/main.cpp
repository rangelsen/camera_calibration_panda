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
#include "camerasensor.hpp"
// #include "calibration.hpp"

#define DEBUG 0

////////////////////////////////////////////////////////////////////////////////
static cv::Mat endeff_pose_;
static PandaStatus panda_status_;

////////////////////////////////////////////////////////////////////////////////
geometry_msgs::Pose generateNextTarget(cv::Mat current_pose, bool* should_quit) {

    double x, y, z;
    bool valid_pos = Util::getUserPosition(&x, &y, &z, should_quit);

    geometry_msgs::Pose target;

    if (!(*should_quit)) {
        if (valid_pos) {

            target.position.x = x;
            target.position.y = y;
            target.position.z = z;

            double sgn = 1.0;

            if ((rand() % 2) > 0)
                sgn = -1.0;

            Eigen::Quaterniond q_target = Util::UniformRandomQuat();
            target.orientation.x = q_target.x() * sgn;
            target.orientation.y = q_target.y() * sgn;
            target.orientation.z = q_target.z() * sgn;
            target.orientation.w = q_target.w() * sgn;
        }
        else {

            target.position.x = current_pose.at<double>(0, 3);
            target.position.y = current_pose.at<double>(1, 3);
            target.position.z = current_pose.at<double>(2, 3);
        }
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

#if DEBUG

    sleep(5);
#else

    /*
	static cv::Ptr<cv::aruco::Dictionary> dict = 
		cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

	static Calibration calib(dict);
    */

	static CameraSensor camera;

    /*
    static uint32_t i = 0;

	cv::Mat image_copy, image;
    cv::cvtColor(image, image_copy, cv::COLOR_GRAY2RGB);
    cv::Mat board_pose = calib.estimateCharucoPose(image_copy, camera);

    if (!board_pose.empty()) {

        cv::imwrite("res/calib-images/ir" + std::to_string(i) + ".png", image);

        Util::writeToFile("res/board-poses.csv", board_pose, i);
        Util::writeToFile("res/endeffector_poses.csv", endeff_pose_, i);

        (*i)++;
    }
    */
#endif
}

////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv) {

// TODO: Setup subscription to franka end-effector pose
// TODO: Generalize user input reading: getUserInput(std::string cmd, double* params, unit16_t n_params);

    ros::init(argc, argv, "calib");

    // Don't ask why
    ros::WallTime d = ros::WallTime::now();

    ros::NodeHandle nh;

    ros::Subscriber panda_status_sub = nh.subscribe("panda_status", 100,
		updatePandaStatus);

    ros::Publisher pose_pub = nh.advertise<geometry_msgs::Pose>("pose", 100);

    panda_status_ = PANDA_INIT;

    srand(time(NULL));
    bool is_first = true;
    bool should_quit = false;

    while (ros::ok()) {

        if (panda_status_ == PANDA_STOPPED) {

            if (is_first)
                is_first = false;
            else {
                calibrate();
			}

            geometry_msgs::Pose next_target =
                generateNextTarget(endeff_pose_, &should_quit);

            if (should_quit) {

                ros::shutdown();
                break;
            }
            else {

                pose_pub.publish(next_target);
                ROS_INFO("Published next target");
            }
        }

        ros::spinOnce();
    }

    return 0;
}

/// @file

