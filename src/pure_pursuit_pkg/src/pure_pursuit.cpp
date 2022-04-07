
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <visualization_msgs/Marker.h>
// TODO: include ROS msg type headers and libraries you need

#include "pure_pursuit_pkg/csv_reader.h"
#include "pure_pursuit_pkg/pure_pursuit.h"

class PurePursuit {
private:
    ros::NodeHandle n;
    // TODO: create ROS subscribers and publishers
    ros::Subscriber pose_sub_;
    ros::Publisher drive_pub_;
    ros::Publisher way_point_viz_pub_;

    bool visualized_;
    size_t unique_marker_id_ = 0;
    
    size_t last_best_index_;

    double look_ahead_distance;
    int n_way_points_;
    std::vector<f110::WayPoint> way_points_data_list_;

    tf2_ros::TransformListener tf_listener_;
    tf2_ros::Buffer tf_buffer_;

public:
    PurePursuit() :
    n(ros::NodeHandle()),
    pose_sub_(n.subscribe("gt_pose",5, &PurePursuit::pose_callback,this)),
    drive_pub_(n.advertise<ackermann_msgs::AckermannDriveStamped>("nav",1)),
    way_point_viz_pub_(n.advertise<visualization_msgs::Marker>("waypoint_markers", 100)),
    visualized_(false),
    last_best_index_(0),
    tf_listener_(tf_buffer_)
    
    {
        look_ahead_distance = 0.5;
        n_way_points_ = 10000;
        f110::CSVReader csv_reader_("/workspace/src/waypoint_logger/scripts.csv");
        way_points_data_list_ = csv_reader_.getData(n_way_points_);
       
        visualize_waypoint_data(); //publish to visualize as Markers
        ros::Duration(1.0).sleep();
        
        
    }

        void add_way_point_visualization(const f110::WayPoint& way_point, const std::string& frame_id,
            double r, double g, double b, double transparency = 0.5, double scale_x=0.1, double scale_y=0.1,
            double scale_z=0.1)
    {
        //  std::cout << "get x " << way_points_data_list_[50].x << std::endl;
        // std::cout << "test";
        visualization_msgs::Marker way_point_marker;
        way_point_marker.header.frame_id = frame_id;
        way_point_marker.header.stamp = ros::Time();
        way_point_marker.ns = "pure_pursuit";
        way_point_marker.id = unique_marker_id_;
        way_point_marker.type = visualization_msgs::Marker::SPHERE;
        way_point_marker.action = visualization_msgs::Marker::ADD;
        way_point_marker.pose.position.x = way_point.x;
        way_point_marker.pose.position.y = way_point.y;
        way_point_marker.pose.position.z = 0;
        way_point_marker.pose.orientation.x = 0.0;
        way_point_marker.pose.orientation.y = 0.0;
        way_point_marker.pose.orientation.z = 0.0;
        way_point_marker.pose.orientation.w = 1.0;
        way_point_marker.scale.x = scale_x;
        way_point_marker.scale.y = scale_y;
        way_point_marker.scale.z = scale_z;
        way_point_marker.color.a = transparency;
        way_point_marker.color.r = r;
        way_point_marker.color.g = g;
        way_point_marker.color.b = b;
        way_point_viz_pub_.publish(way_point_marker);
        // ROS_INFO("pub");
        unique_marker_id_++;
    }

    void visualize_waypoint_data()
    {     visualization_msgs::Marker way_point_marker;
        way_point_marker.header.frame_id = "map";
        way_point_marker.header.stamp = ros::Time();
        way_point_marker.ns = "pure_pursuit";
        way_point_marker.id = 3452;
        way_point_marker.type = visualization_msgs::Marker::SPHERE;
        way_point_marker.action = visualization_msgs::Marker::ADD;
        way_point_marker.pose.position.x = way_points_data_list_[5].x;
        way_point_marker.pose.position.y = way_points_data_list_[5].y;
        way_point_marker.pose.position.z = 0;
        way_point_marker.pose.orientation.x = 0.0;
        way_point_marker.pose.orientation.y = 0.0;
        way_point_marker.pose.orientation.z = 0.0;
        way_point_marker.pose.orientation.w = 1.0;
        way_point_marker.scale.x = 0.1;
        way_point_marker.scale.y = 0.1;
        way_point_marker.scale.z = 0.1;
        way_point_marker.color.a = 0.5;
        way_point_marker.color.r = 0.0;
        way_point_marker.color.g = 0.0;
        way_point_marker.color.b = 1.0;
        way_point_viz_pub_.publish(way_point_marker);

        std::cout << " x " << way_points_data_list_[5].x << std::endl;

        const int increment = int(way_points_data_list_.size()/50);
        for(int i=0, j=0; i<way_points_data_list_.size(); i=i+increment, j++)
        {
            add_way_point_visualization(way_points_data_list_[i], "map", 0.0, 0.0, 1.0, 0.5);
            // add_way_point_visualization(way_points_data_list_[i],"map",1.0,1.0,0.5,0.5,0.1,0.1,0.1);
            std::cout << " x "<< i << " " << way_points_data_list_[i].x << std::endl;
        }
    }

    void pose_callback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg) {
        // TODO: find the current waypoint to track using methods mentioned in lecture

        // Convert pose_msg to WayPoint
        const auto current_way_point = f110::WayPoint(pose_msg);

        // Transform Points
        const auto transformed_way_points = transform(way_points_data_list_, current_way_point, tf_buffer_, tf_listener_);
        
        //find goal point
        const auto goal_way_point_index = f110::get_best_track_point_index(transformed_way_points, look_ahead_distance, last_best_index_);
       
       
        // TODO: transform goal point to vehicle frame of reference
        geometry_msgs::TransformStamped map_to_base_link;
        map_to_base_link = tf_buffer_.lookupTransform("base_link", "map", ros::Time(0));
        geometry_msgs::Pose goal_way_point;
        goal_way_point.position.x = way_points_data_list_[goal_way_point_index].x;
        goal_way_point.position.y = way_points_data_list_[goal_way_point_index].y;
        goal_way_point.position.z = 0;
        goal_way_point.orientation.x = 0;
        goal_way_point.orientation.y = 0;
        goal_way_point.orientation.z = 0;
        goal_way_point.orientation.w = 1;
        tf2::doTransform(goal_way_point, goal_way_point, map_to_base_link);

        add_way_point_visualization(goal_way_point, "base_link", 1.0, 0.0, 0.0, 0.3, 0.2, 0.2, 0.2);
        
        
        // TODO: calculate curvature/steering angle
         const double steering_angle = 2*(goal_way_point.position.y)/(look_ahead_distance*look_ahead_distance);
        
        // TODO: publish drive message, don't forget to limit the steering angle between -0.4189 and 0.4189 radians
        ackermann_msgs::AckermannDriveStamped drive_msg;
        drive_msg.header.stamp = ros::Time::now();
        drive_msg.header.frame_id = "base_link"; 
        drive_msg.drive.steering_angle = steering_angle; 
        if (steering_angle > 0.4)
            {
                drive_msg.drive.steering_angle = 0.4;
            }
        else if (steering_angle < -0.4)
        {
            drive_msg.drive.steering_angle = -0.4;
        }
        drive_msg.drive.speed = 1.0;
        drive_pub_.publish(drive_msg);
        
    }

};
int main(int argc, char ** argv) {
    ros::init(argc, argv, "pure_pursuit_node");
    PurePursuit pp;
    ros::spin();
    return 0;
}