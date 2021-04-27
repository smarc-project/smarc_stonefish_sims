#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Geometry>

class ImuRotater {
private:
    Eigen::Quaterniond rot;
    ros::Publisher pub;
    ros::Subscriber sub;
public:
    void callback(const sensor_msgs::Imu& msg)
    {
        sensor_msgs::Imu rot_msg = msg;
        Eigen::Quaterniond r(
            msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
        r = rot*r;
        rot_msg.orientation.w = r.w();
        rot_msg.orientation.x = r.x();
        rot_msg.orientation.y = r.y();
        rot_msg.orientation.z = r.z();
        pub.publish(rot_msg);
    }

    ImuRotater(ros::NodeHandle& nh)
    {
        double angle;
        ros::NodeHandle pn("~");
        pn.param<double>("angle", angle, .5*M_PI);
        rot = Eigen::Quaterniond(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()));
        pub  = nh.advertise<sensor_msgs::Imu>("imu_out/data", 1000);
        sub = nh.subscribe("imu_in/data", 1000, &ImuRotater::callback, this);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rotate_imu_orientation");

    ros::NodeHandle nh;

    ImuRotater rotater(nh);

    ros::spin();

    return 0;
}
