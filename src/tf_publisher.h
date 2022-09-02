#include "ros/ros.h"

#include "tf/tfMessage.h"
#include "geometry_msgs/TransformStamped.h"

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <math.h>


#include "Eigen/Dense"
#include "geometry_msgs/PoseStamped.h"
#include "utils.h"

#ifndef O_TF_PUBLISHER_H_
#define O_TF_PUBLISHER_H_

using namespace std;


class TF_PUBLISHER {
public:

	TF_PUBLISHER();
    void run();
    void tf_cb(tf2_msgs::TFMessage);
    void pose_publisher();

private:

	ros::NodeHandle _nh;
    ros::Subscriber _tf_sub;
    ros::Publisher _tf_pub;

    geometry_msgs::PoseStamped _quad_pose;
    Eigen::Vector3d _position_tf;
    Eigen::Vector4d _quaternion_tf;

    string _frame_id_ref = "map";
    string _chid_frame_id_ref = "d400_link";

};

#endif 
