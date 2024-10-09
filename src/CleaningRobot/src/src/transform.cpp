#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "tf_listener");
    ros::NodeHandle nh;

    tf::TransformListener listener;
    ros::Rate rate(10.0);

    while (nh.ok()) {
        tf::StampedTransform transform;
        try {
            listener.lookupTransform("map", "base_link", ros::Time(0), transform);

            // 获取四元数
            tf::Quaternion q = transform.getRotation();
            double roll, pitch, yaw;
            tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
            
            double x,y,z;
            x = transform.getOrigin().getX();
            y = transform.getOrigin().getY();
            z = transform.getOrigin().getZ();

        

            ROS_INFO("x: %.2f, y: %.2f, z: %.2f, Roll: %f, Pitch: %f, Yaw: %f", x, y, z, roll, pitch, yaw);
        } catch (tf::TransformException &ex) {
            ROS_WARN("%s", ex.what());
        }

        rate.sleep();
    }
    return 0;
}
