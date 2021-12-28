#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <robot_calibration_msgs/CalibrationData.h>
#include <visualization_msgs/Marker.h>

class PointPlotter
{
    public:
        PointPlotter();
        void callback(const robot_calibration_msgs::CalibrationData&);
        visualization_msgs::Marker create_arm_marker_message(const geometry_msgs::PointStamped&, const int);
        visualization_msgs::Marker create_camera_marker_message(const geometry_msgs::PointStamped&, const int);

    private:
        ros::NodeHandle n_;
        ros::Publisher pub_;
        ros::Subscriber sub_;
};

PointPlotter::PointPlotter()
{
    pub_ = n_.advertise<visualization_msgs::Marker>("/visualization_marker", 100);
    sub_ = n_.subscribe("/calibration_data", 5, &PointPlotter::callback, this);
}

void PointPlotter::callback(const robot_calibration_msgs::CalibrationData& msg)
{
    ROS_INFO("new calibration data message received.");

    for (auto obs : msg.observations)
    {
        int id = 0;

        // TODO: check with parameter name, not hard coded string
        if (obs.sensor_name == "arm")
        {
            for (auto feature : obs.features)
            {
                visualization_msgs::Marker marker_msg = create_arm_marker_message(feature, id);
                pub_.publish(marker_msg);
                ++id;
            }
        }

        // TODO: check with parameter name, not hard coded string
        else if (obs.sensor_name == "camera")
        {
            for (auto feature : obs.features)
            {
                visualization_msgs::Marker marker_msg = create_camera_marker_message(feature, id);
                pub_.publish(marker_msg);
                ++id;
            }
        }
    }
}

visualization_msgs::Marker PointPlotter::create_arm_marker_message(const geometry_msgs::PointStamped& feature, const int id)
{
    // TODO: adjust these parameters to make sense.
    visualization_msgs::Marker marker;
    marker.header.frame_id = "wrist_3_link";
    marker.header.stamp = ros::Time();
    marker.ns = "arm_view";
    marker.id = id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = feature.point.x;
    marker.pose.position.y = feature.point.y;
    marker.pose.position.z = feature.point.z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    return marker;
}

visualization_msgs::Marker PointPlotter::create_camera_marker_message(const geometry_msgs::PointStamped& feature, const int id)
{
    // TODO: adjust these parameters to make sense.
    visualization_msgs::Marker marker;
    marker.header.frame_id = "camera_color_optical_frame";
    marker.header.stamp = ros::Time();
    marker.ns = "camera_view";
    marker.id = id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = feature.point.x;
    marker.pose.position.y = feature.point.y;
    marker.pose.position.z = feature.point.z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;

    return marker;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "calibration_data_visualization");
    PointPlotter point_plotter;
    // ros::NodeHandle n;
    // ros::Subscriber sub = n.subscribe("/calibration_data", 5, visualizationCallback);
    ros::spin();
    return 0;
}

