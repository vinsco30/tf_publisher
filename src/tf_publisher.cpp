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


class TF_PUBLISHER {
    public:
        TF_PUBLISHER();
        void odom_cb( nav_msgs::Odometry );
        void publisher();
        void run();
        void tf_listner();
    
    private:
        ros::NodeHandle nh;
        tf::TransformListener listener;
        ros::Publisher tf_pub;
        ros::Publisher rpy_pub;
        ros::Subscriber odom_sub;
        Vector3d _local_pose;
        Vector4d _local_quat;
        tf::StampedTransform transform;
        geometry_msgs::PoseStamped quad_pose;
        geometry_msgs::Vector3 rpy;
        Vector3d _rpy;
        Vector4d _quat;
};

TF_PUBLISHER::TF_PUBLISHER() {
    tf_pub = nh.advertise<geometry_msgs::PoseStamped> ("/mavros/vision_pose/pose", 0);
    rpy_pub = nh.advertise<geometry_msgs::Vector3> ("/rpy", 0);
    odom_sub = nh.subscribe("/t265/odom/sample", 1, &TF_PUBLISHER::odom_cb, this );
}

void TF_PUBLISHER::run() {
    boost::thread tf_listner_t( &TF_PUBLISHER::tf_listner, this);
    boost::thread publisher_t( &TF_PUBLISHER::publisher, this);
    ros::spin();
}

void TF_PUBLISHER::tf_listner() {

    ros::Rate rate(10.0);
    while( ros::ok() ) {
        try{
            ros::Time now = ros::Time(0);
            listener.waitForTransform("/map" ,"/camera_link", now, ros::Duration(5.0));
            cout<<"transform exists \n";
            listener.lookupTransform("/map" ,"/camera_link", now, transform);
            
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        rate.sleep();
    }

}

void TF_PUBLISHER::odom_cb( nav_msgs::Odometry t265_odom ) {
    _local_pose << t265_odom.pose.pose.position.x, t265_odom.pose.pose.position.y, t265_odom.pose.pose.position.z;
    _local_quat << t265_odom.pose.pose.orientation.x, t265_odom.pose.pose.orientation.y, t265_odom.pose.pose.orientation.z, t265_odom.pose.pose.orientation.w;
}

void TF_PUBLISHER::publisher() {
        ros::Rate rate(80.0);
        Matrix3d prova;
        while ( ros::ok() ) {

            quad_pose.header.stamp = ros::Time::now();
            quad_pose.pose.position.x = transform.getOrigin().x();
            quad_pose.pose.position.y = transform.getOrigin().y();
            quad_pose.pose.position.z = transform.getOrigin().z();

            quad_pose.pose.orientation.w = transform.getRotation().w();
            quad_pose.pose.orientation.x = transform.getRotation().x();
            quad_pose.pose.orientation.y = transform.getRotation().y();
            quad_pose.pose.orientation.z = transform.getRotation().z();
            tf_pub.publish(quad_pose);

            _quat[0] = transform.getRotation().w();
            _quat[1] = transform.getRotation().x();
            _quat[2] = transform.getRotation().y();
            _quat[3] = transform.getRotation().z();
            prova = utilities::QuatToMat(_quat);
            _rpy = utilities::R2XYZ(prova);
            rpy.x = _rpy[0];
            rpy.y = _rpy[1];
            rpy.z = _rpy[2];
            rpy_pub.publish(rpy);
            rate.sleep();
        }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "tf_publisher");
    TF_PUBLISHER tfpub;
    tfpub.run();

    return 0;  
}


