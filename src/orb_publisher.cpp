#include "ros/ros.h"
//#include "tf_publisher.h"
#include "tf/transform_listener.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Odometry.h"
#include "utils.h"
#include "geometry_msgs/Vector3.h"

using namespace Eigen;
using namespace std;

class ORB_PUBLISHER {
    public:
        ORB_PUBLISHER();
        void publisher();
        void pose_cb( geometry_msgs::PoseStamped );
        void run();

    private:
        ros::NodeHandle nh;
        ros::Publisher orb_pub;
        ros::Subscriber orb_sub;
        geometry_msgs::PoseStamped quad_pose;
        Vector3d _local_pose;
        Vector4d _local_quat;

};

ORB_PUBLISHER::ORB_PUBLISHER() {
    orb_pub = nh.advertise<geometry_msgs::PoseStamped> ("/mavros/vision_pose/pose", 0);
    orb_sub = nh.subscribe("/orb_slam2_rgbd/pose", 1, &ORB_PUBLISHER::pose_cb, this );
}

void ORB_PUBLISHER::run() {
    boost::thread publisher_t( &ORB_PUBLISHER::publisher, this );
    ros::spin();
}

void ORB_PUBLISHER::pose_cb( geometry_msgs::PoseStamped p ) {
    _local_pose << p.pose.position.x, p.pose.position.y, p.pose.position.z;
    _local_quat << p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w;
}

void ORB_PUBLISHER::publisher() {
    ros::Rate rate(80.0);

    while( ros::ok() ) {
            quad_pose.header.stamp = ros::Time::now();
            quad_pose.pose.position.x = _local_pose[0]*1;
            quad_pose.pose.position.y = _local_pose[1]*1;
            quad_pose.pose.position.z = _local_pose[2]*1;

            quad_pose.pose.orientation.w = _local_quat[3];
            quad_pose.pose.orientation.x = _local_quat[0];
            quad_pose.pose.orientation.y = _local_quat[1];
            quad_pose.pose.orientation.z = _local_quat[2];
            orb_pub.publish(quad_pose);

            rate.sleep();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "orb_publisher");
    ORB_PUBLISHER orbpub;
    orbpub.run();

    return 0;
}