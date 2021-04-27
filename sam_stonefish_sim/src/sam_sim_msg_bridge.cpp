#include <ros/ros.h>

#include <std_msgs/Float64.h>
#include <std_srvs/SetBool.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Image.h>

#include <smarc_msgs/ThrusterRPM.h>
#include <smarc_msgs/ThrusterFeedback.h>
#include <smarc_msgs/DVL.h>
#include <smarc_msgs/DVLBeam.h>
#include <smarc_msgs/SensorStatus.h>
#include <smarc_msgs/Sidescan.h>
#include <sam_msgs/PercentStamped.h>
#include <sam_msgs/ThrusterAngles.h>

#include <cola2_msgs/Setpoints.h>
#include <cola2_msgs/DVL.h>
#include <cola2_msgs/DVLBeam.h>


class SamSimMsgBridge
{
private:

    // parameters
    std::string robot_name;
    double vbs_vol_min, vbs_vol_max;
    double lcg_joint_min, lcg_joint_max;

    // service to toggle sidescan
    ros::ServiceServer sidescan_toggle_service;
    bool sidescan_enabled;

    // messages to publish
    smarc_msgs::Sidescan sidescan_msg;
    smarc_msgs::SensorStatus sidescan_status_msg;
    cola2_msgs::Setpoints last_thruster_msg;
    cola2_msgs::Setpoints zero_thruster_msg;
    ros::Time last_thruster_msg_time;
    sensor_msgs::BatteryState battery_msg;

    // sensor outputs from stonefish
    ros::Subscriber sidescan_sub;
    ros::Subscriber pressure_sub;
    ros::Subscriber vbs_fb_sub;
    ros::Subscriber joint_states_fb_sub;
    ros::Subscriber dvl_sub;

    // sensor outputs from bridge
    ros::Publisher sidescan_status_pub;
    ros::Publisher sidescan_pub;
    ros::Publisher pressure_pub;
    ros::Publisher battery_pub;
    ros::Publisher vbs_fb_pub;
    ros::Publisher lcg_fb_pub;
    ros::Publisher thruster1_fb_pub;
    ros::Publisher thruster2_fb_pub;
    ros::Publisher dvl_pub;
    
    // command inputs to stonefish
    ros::Publisher thruster_cmd_pub;
    ros::Publisher vbs_cmd_pub;
    ros::Publisher joint_states_cmd_pub;

    // command inputs to bridge
    ros::Subscriber thruster1_cmd_sub;
    ros::Subscriber thruster2_cmd_sub;
    ros::Subscriber vbs_cmd_sub;
    ros::Subscriber angles_cmd_sub;
    ros::Subscriber lcg_cmd_sub;
    
    // publish timers
    ros::Timer battery_timer;
    ros::Timer thruster_timer;
    ros::Timer sidescan_status_timer;

public:

    void sidescan_status_timer_callback(const ros::TimerEvent& event)
    {
        if (sidescan_enabled) {
            sidescan_status_msg.sensor_status = smarc_msgs::SensorStatus::SENSOR_STATUS_ACTIVE;
            sidescan_status_msg.diagnostics_message = "sidescan is active!";
        } else {
            sidescan_status_msg.sensor_status = smarc_msgs::SensorStatus::SENSOR_STATUS_NOT_ACTIVE;
            sidescan_status_msg.diagnostics_message = "sidescan is NOT active!";
        }

        sidescan_status_pub.publish(sidescan_status_msg);
    }

    bool toggle_sidescan(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
    {
        sidescan_enabled = req.data;
        res.success = true;
        res.message = std::string("Toggle sidescan sonar: enabled = ") + (sidescan_enabled ? "true" : "false");
        return true;
    }

    void sidescan_callback(const sensor_msgs::Image& msg)
    {
        // Only publish sidescan message if the sidescan is enabled
        if (!sidescan_enabled)
        {
            return;
        }

        sidescan_msg.header.stamp = ros::Time::now();
        // The following fields are filled with dummy values
        sidescan_msg.type = 0xE2;
        sidescan_msg.time = 1;
        sidescan_msg.gain = 100;
        sidescan_msg.decimation = 1;
        sidescan_msg.max_duration = 1;

        // Stonefish publishes SSS signal as an Image of size (sss_bins, sss_lines)
        // Take the first row of the image and publish as sidescan message
        // Assumption: consecutive SSS signals differ by one row
        auto mid = msg.width/2;
        auto end = msg.width;
        sidescan_msg.port_channel = std::vector<uint8_t> (msg.data.begin(), msg.data.begin() + mid);
        std::reverse(sidescan_msg.port_channel.begin(), sidescan_msg.port_channel.end());
        sidescan_msg.starboard_channel = std::vector<uint8_t> (msg.data.begin() + mid, msg.data.begin() + end);

        sidescan_pub.publish(sidescan_msg);
    }

    void dvl_callback(const cola2_msgs::DVL& msg)
    {
        smarc_msgs::DVL dvl;
        dvl.header = msg.header;
        dvl.velocity = msg.velocity;
        dvl.velocity_covariance = msg.velocity_covariance;
        dvl.altitude = msg.altitude;
        for (const cola2_msgs::DVLBeam& beam : msg.beams) {
            smarc_msgs::DVLBeam b;
            b.range = beam.range;
            b.range_covariance = beam.range_covariance;
            b.velocity = beam.velocity;
            b.velocity_covariance = beam.velocity_covariance;
            b.pose = beam.pose;
            dvl.beams.push_back(b);
        }
        dvl_pub.publish(dvl);
    }

    void angles_cmd_callback(const sam_msgs::ThrusterAngles& msg)
    {
        sensor_msgs::JointState thruster_angles;
        thruster_angles.header.stamp = ros::Time::now();
        thruster_angles.name.push_back(robot_name + "/thruster_yaw_joint");
        thruster_angles.name.push_back(robot_name + "/thruster_pitch_joint");
        thruster_angles.position.push_back(msg.thruster_horizontal_radians);
        thruster_angles.position.push_back(msg.thruster_vertical_radians);
        joint_states_cmd_pub.publish(thruster_angles);
    }

    void joint_states_fb_callback(const sensor_msgs::JointState& msg)
    {
        if (msg.name[0] == "sam/lcg_joint") {
            sam_msgs::PercentStamped fb_msg;
            fb_msg.header.stamp = ros::Time::now();
            fb_msg.value = 100./(lcg_joint_max-lcg_joint_min)*(msg.position[0] - lcg_joint_min);
            lcg_fb_pub.publish(fb_msg);
        }
    }

    void lcg_cmd_callback(const sam_msgs::PercentStamped& msg)
    {
        sensor_msgs::JointState lcg_pos;
        lcg_pos.header.stamp = ros::Time::now();
        lcg_pos.name.push_back(robot_name + "/lcg_joint");
        lcg_pos.position.push_back(lcg_joint_min + 0.01*msg.value*(lcg_joint_max-lcg_joint_min));
        joint_states_cmd_pub.publish(lcg_pos);
    }

    void vbs_cmd_callback(const sam_msgs::PercentStamped& msg)
    {
        std_msgs::Float64 cmd;
        cmd.data = vbs_vol_min + 0.01*msg.value*(vbs_vol_max - vbs_vol_min);
        vbs_cmd_pub.publish(cmd);
    }

    void vbs_fb_callback(const std_msgs::Float64& msg)
    {
        sam_msgs::PercentStamped fb;
        fb.value = 100.*(.5+msg.data);
        vbs_fb_pub.publish(fb);
    }

    void pressure_callback(const sensor_msgs::FluidPressure& msg)
    {
        sensor_msgs::FluidPressure new_msg = msg;
        new_msg.fluid_pressure += 101325.;
        pressure_pub.publish(new_msg);
    }

    void thruster_cmd_callback(const smarc_msgs::ThrusterRPM::ConstPtr& msg, int thruster_ind)
    {
        last_thruster_msg_time = ros::Time::now();
        last_thruster_msg.header.stamp = last_thruster_msg_time;
        last_thruster_msg.setpoints[thruster_ind] = msg->rpm;
    }

    void thruster_timer_callback(const ros::TimerEvent& event)
    {
        ros::Time current_time = ros::Time::now();
        cola2_msgs::Setpoints msg;
        // must publish at 10Hz to get the thrusters going
        if ((current_time - last_thruster_msg_time).toSec() < 0.1) {
            msg = last_thruster_msg;
            msg.header.stamp = current_time;
        }
        else {
            msg = zero_thruster_msg;
        }
        thruster_cmd_pub.publish(msg);

        // publish feedback
        smarc_msgs::ThrusterFeedback fb;
        fb.rpm.rpm = msg.setpoints[0];
        fb.header.stamp = msg.header.stamp;
        thruster1_fb_pub.publish(fb);
        fb.rpm.rpm = msg.setpoints[1];
        thruster2_fb_pub.publish(fb);
    }

    void battery_timer_callback(const ros::TimerEvent& event)
    {
        battery_msg.header.stamp = ros::Time::now();
        battery_pub.publish(battery_msg);
    }

    void setup_messages()
    {
        last_thruster_msg.setpoints.push_back(0.);
        last_thruster_msg.setpoints.push_back(0.);
        zero_thruster_msg = last_thruster_msg;
        last_thruster_msg_time = ros::Time::now();
        battery_msg.voltage = 12.5;
        battery_msg.percentage = 81.;
        battery_msg.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
    }

    SamSimMsgBridge(ros::NodeHandle& nh)
    {
        ros::NodeHandle pn("~");
        pn.param<std::string>("robot_name", robot_name, "sam");
        pn.param<double>("lcg_joint_min", lcg_joint_min, -.01);
        pn.param<double>("lcg_joint_max", lcg_joint_max, .01);
        pn.param<double>("vbs_vol_min", vbs_vol_min, -.5);
        pn.param<double>("vbs_vol_max", vbs_vol_max, .5);

        setup_messages();

        sidescan_pub  = nh.advertise<smarc_msgs::Sidescan>("payload/sidescan", 1000);
        sidescan_sub = nh.subscribe("sim/sidescan/image", 1000, &SamSimMsgBridge::sidescan_callback, this);
        sidescan_enabled = false;
        sidescan_toggle_service = nh.advertiseService("payload/toggle_sidescan", &SamSimMsgBridge::toggle_sidescan, this);
        sidescan_status_msg.service_name = "/sam/payload/toggle_sidescan";
        sidescan_status_pub = nh.advertise<smarc_msgs::SensorStatus>("payload/sidescan_status", 1000);
        sidescan_status_timer = nh.createTimer(ros::Duration(1), &SamSimMsgBridge::sidescan_status_timer_callback, this);

        battery_pub = nh.advertise<sensor_msgs::BatteryState>("core/battery", 1000);
        pressure_pub = nh.advertise<sensor_msgs::FluidPressure>("core/pressure", 1000);
        pressure_sub = nh.subscribe("sim/pressure", 1000, &SamSimMsgBridge::pressure_callback, this);

        dvl_pub = nh.advertise<smarc_msgs::DVL>("core/dvl", 1000);
        dvl_sub = nh.subscribe("sim/dvl", 1000, &SamSimMsgBridge::dvl_callback, this);

        vbs_cmd_sub = nh.subscribe("core/vbs_cmd", 1000, &SamSimMsgBridge::vbs_cmd_callback, this);
        vbs_fb_sub = nh.subscribe("sim/vbs/volume_centered", 1000, &SamSimMsgBridge::vbs_fb_callback, this);
        vbs_fb_pub =  nh.advertise<sam_msgs::PercentStamped>("core/vbs_fb", 1000);
        vbs_cmd_pub = nh.advertise<std_msgs::Float64>("sim/vbs/setpoint", 1000); 

        joint_states_cmd_pub = nh.advertise<sensor_msgs::JointState>("desired_joint_states", 1000);
        joint_states_fb_sub =  nh.subscribe("joint_states", 1000, &SamSimMsgBridge::joint_states_fb_callback, this);
        lcg_fb_pub = nh.advertise<sam_msgs::PercentStamped>("core/lcg_fb", 1000);
        lcg_cmd_sub = nh.subscribe("core/lcg_cmd", 1000, &SamSimMsgBridge::lcg_cmd_callback, this);

        angles_cmd_sub = nh.subscribe("core/thrust_vector_cmd", 1000, &SamSimMsgBridge::angles_cmd_callback, this);
        thruster1_cmd_sub = nh.subscribe<smarc_msgs::ThrusterRPM>("core/thruster1_cmd", 1000, boost::bind(&SamSimMsgBridge::thruster_cmd_callback, this, _1, 0));
        thruster2_cmd_sub = nh.subscribe<smarc_msgs::ThrusterRPM>("core/thruster2_cmd", 1000, boost::bind(&SamSimMsgBridge::thruster_cmd_callback, this, _1, 1));
        thruster_cmd_pub = nh.advertise<cola2_msgs::Setpoints>("sim/thruster_setpoints", 1000);
        thruster1_fb_pub = nh.advertise<smarc_msgs::ThrusterFeedback>("core/thruster1_fb", 1000);
        thruster2_fb_pub = nh.advertise<smarc_msgs::ThrusterFeedback>("core/thruster2_fb", 1000);

        battery_timer = nh.createTimer(ros::Duration(1), &SamSimMsgBridge::battery_timer_callback, this);
        thruster_timer = nh.createTimer(ros::Duration(.1), &SamSimMsgBridge::thruster_timer_callback, this);
    }

};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "sam_sim_msg_bridge");

    ros::NodeHandle nh;

    SamSimMsgBridge bridge(nh);

    ros::spin();

    return 0;
}
