#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include "util.hpp"

////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv) {

    ros::init(argc, argv, "manual_user");
    ros::NodeHandle nh;

    ros::Publisher pose_pub = nh.advertise<geometry_msgs::Pose>("pose", 100);

    double x, y, z;

    while (ros::ok()) {

        bool should_quit = false;
        bool valid_pos = Util::getUserPosition(&x, &y, &z, &should_quit);

        if (valid_pos) {

            geometry_msgs::Pose target_pose;
            std::cout << "user position: " << x << ", " << y << ", " << z << std::endl;
            target_pose.position.x = x;
            target_pose.position.y = y;
            target_pose.position.z = z;
            target_pose.orientation.x = 1.0;

            pose_pub.publish(target_pose);
            std::cout << "published pose" << std::endl;
        }

        ros::spinOnce();
    }

    return 0;
}
