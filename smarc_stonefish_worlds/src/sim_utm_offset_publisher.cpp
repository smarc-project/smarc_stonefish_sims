#include <ros/ros.h>

#include <geodesy/utm.h>
#include <tf2_ros/static_transform_broadcaster.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sim_utm_offset_publisher");
    ros::NodeHandle nh;

    int utm_zone;
    std::string utm_band;
    if (!nh.hasParam("/utm_zone") || !nh.hasParam("/utm_band")) {
        ROS_ERROR("/utm_zone and /utm_band global parameters needed for UTM transform");
        return -1;
    }
    else {
        nh.getParam("/utm_zone", utm_zone);
        nh.getParam("/utm_band", utm_band);
    }

    std::string str_latitude, str_longitude;
    ros::NodeHandle pn("~");
    if (!pn.hasParam("latitude") || !pn.hasParam("longitude")) {
        ROS_ERROR("latitude and longitude parameters needed for UTM transform");
        return -1;
    }
    else {
        pn.getParam("latitude", str_latitude);
        pn.getParam("longitude", str_longitude);
    }
    double latitude = std::stod(str_latitude);
    double longitude = std::stod(str_longitude);

    ROS_INFO("Translating lat: %f, lon: %f to point in UTM zone: %d, band: %s", latitude, longitude, utm_zone, utm_band.c_str());
 
    geographic_msgs::GeoPoint lat_lon_point;
    lat_lon_point.latitude = latitude;
    lat_lon_point.longitude = longitude;
    lat_lon_point.altitude = 0.;
    geodesy::UTMPoint geo_utm_point;
    geodesy::fromMsg(lat_lon_point, geo_utm_point, true, utm_band[0], utm_zone);

    ROS_INFO("Got UTM coordinates easting: %f, northing: %f", geo_utm_point.easting, geo_utm_point.northing);

    tf2_ros::StaticTransformBroadcaster staticBroadcaster;

    geometry_msgs::TransformStamped transformStamped;
    transformStamped.transform.translation.x = geo_utm_point.easting;
    transformStamped.transform.translation.y = geo_utm_point.northing;
    transformStamped.transform.translation.z = 0.;
    transformStamped.transform.rotation.w = 1.;
    transformStamped.header.frame_id = "utm";
    transformStamped.child_frame_id = "map";
    transformStamped.header.stamp = ros::Time::now();
    staticBroadcaster.sendTransform(transformStamped);

    ros::spin();

    return 0;
}
