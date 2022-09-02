#include "ros/ros.h"
#include "utils.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"

using namespace Eigen;
using namespace std;


class GAZEBO_RTABMAP {
    public:
        GAZEBO_RTABMAP();
        void odom_cb ( nav_msgs::Odometry );
        void pub();
        void run();
        void tf_listner();

    private:
        ros::NodeHandle nh;
        tf::TransformListener listener;
        ros::Subscriber odom_sub;
        ros::Publisher pose_pub;
        Vector3d _local_pose;
        Vector4d _local_quat;
        tf::StampedTransform transform;
        geometry_msgs::PoseStamped quad_pose;
        Matrix3d rotation_x;

};

GAZEBO_RTABMAP::GAZEBO_RTABMAP() {
    // odom_sub = nh.subscribe("/rtabmap/odom", 1, &GAZEBO_RTABMAP::odom_cb, this);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped> ("/mavros/vision_pose/pose", 0);
}

void GAZEBO_RTABMAP::run() {
    boost::thread pub_t( &GAZEBO_RTABMAP::pub, this );
    boost::thread tf_listner_t( &GAZEBO_RTABMAP::tf_listner, this);
    ros::spin();
}

void GAZEBO_RTABMAP::tf_listner() {

    ros::Rate rate(100.0);
    while( ros::ok() ) {
        try{
            ros::Time now = ros::Time(0);
            listener.waitForTransform("/odom" ,"/camera_link", now, ros::Duration(3.0));
            cout<<"transform exists \n";
            listener.lookupTransform("/odom" ,"/camera_link", now, transform);
            
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        rate.sleep();
    }

}

// void GAZEBO_RTABMAP::odom_cb(nav_msgs::Odometry odom) {
//     _local_pose << odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z;
//     _local_quat << odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w;
// }

void GAZEBO_RTABMAP::pub() {
    ros::Rate rate(60.0);
    Eigen::Matrix3d orient;
    Eigen::Matrix3d or1;
    Vector3d pos;
    Vector4d quat;
    rotation_x = utilities::rotx(90);

    while( ros::ok() ) {
        geometry_msgs::PoseStamped quad_pose;
        quad_pose.header.frame_id = "odom";
        quad_pose.header.stamp = ros::Time::now();
        _local_pose <<  transform.getOrigin().x(),  transform.getOrigin().y(),  transform.getOrigin().z();
        _local_quat << transform.getRotation().w(), transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z();
        orient = utilities::QuatToMat(_local_quat);
        pos = rotation_x*_local_pose;
        or1 = orient;
        quat = utilities::MatToQuat(or1);
        quad_pose.pose.position.x = -pos[1];
        quad_pose.pose.position.y = -pos[0];
        quad_pose.pose.position.z = -pos[2];

        quad_pose.pose.orientation.w = quat[0];
        quad_pose.pose.orientation.x = -quat[2];
        quad_pose.pose.orientation.y = -quat[1];
        quad_pose.pose.orientation.z = -quat[3];

        pose_pub.publish(quad_pose);

        rate.sleep();   
    }
}

int main(int argc, char** argv) {
    ros::init(argc,argv, "gazebo_rtabmap");
    GAZEBO_RTABMAP gzb;
    gzb.run();

    return 0;
}