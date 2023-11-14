#include <homing_local_planner/visualization.h>

namespace homing_local_planner
{
    HomingVisualization::HomingVisualization() : initialized_(false)
    {
    }

    HomingVisualization::HomingVisualization(ros::NodeHandle &nh, std::string frame_id) : initialized_(false), frame_id_(frame_id)
    {
        initialize(nh);
    }
    void HomingVisualization::initialize(ros::NodeHandle &nh)
    {
        if (initialized_)
            ROS_WARN("HomingVisualization already initialized. Reinitalizing...");

        homing_marker_pub_ = nh.advertise<visualization_msgs::Marker>("homing_markers", 100);
        local_plan_pub_ = nh.advertise<nav_msgs::Path>("local_plan", 1);
        global_plan_pub_ = nh.advertise<nav_msgs::Path>("global_plan", 1);
        homing_marker_pub_ = nh.advertise<visualization_msgs::Marker>("teb_markers", 1000);
        initialized_ = true;
    }
    void HomingVisualization::publishViaPoints(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &via_points,
                                               const std::string &ns) const
    {
        if (via_points.empty() || printErrorWhenNotInitialized())
            return;

        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id_;
        marker.header.stamp = ros::Time::now();
        marker.ns = ns;
        marker.id = 0;
        marker.type = visualization_msgs::Marker::POINTS;
        marker.action = visualization_msgs::Marker::ADD;
        marker.lifetime = ros::Duration(2.0);

        for (std::size_t i = 0; i < via_points.size(); ++i)
        {
            geometry_msgs::Point point;
            point.x = via_points[i][0];
            point.y = via_points[i][1];
            point.z = 0;
            marker.points.push_back(point);
        }

        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;

        homing_marker_pub_.publish(marker);
    }

    void HomingVisualization::publishLocalPlan(const std::vector<geometry_msgs::PoseStamped> &local_plan) const
    {
        if (printErrorWhenNotInitialized())
            return;
        base_local_planner::publishPlan(local_plan, local_plan_pub_);
    }

    void HomingVisualization::publishGlobalPlan(const std::vector<geometry_msgs::PoseStamped> &global_plan) const
    {
        if (printErrorWhenNotInitialized())
            return;
        base_local_planner::publishPlan(global_plan, global_plan_pub_);
    }

    bool HomingVisualization::printErrorWhenNotInitialized() const
    {
        if (!initialized_)
        {
            ROS_ERROR("HomingVisualization class not initialized. You must call initialize or an appropriate constructor");
            return true;
        }
        return false;
    }

    void HomingVisualization::publishObstacles(const ObstContainer &obstacles, double scale) const
    {
        if (obstacles.empty() || printErrorWhenNotInitialized())
            return;

        // Visualize point obstacles
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = frame_id_;
            marker.header.stamp = ros::Time::now();
            marker.ns = "PointObstacles";
            marker.id = 0;
            marker.type = visualization_msgs::Marker::POINTS;
            marker.action = visualization_msgs::Marker::ADD;
            marker.lifetime = ros::Duration(2.0);

            for (ObstContainer::const_iterator obst = obstacles.begin(); obst != obstacles.end(); ++obst)
            {
                boost::shared_ptr<PointObstacle> pobst = boost::dynamic_pointer_cast<PointObstacle>(*obst);
                if (!pobst)
                    continue;

                if (true)
                {
                    geometry_msgs::Point point;
                    point.x = pobst->x();
                    point.y = pobst->y();
                    point.z = 0;
                    marker.points.push_back(point);
                }
            }

            marker.scale.x = scale;
            marker.scale.y = scale;
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;

            homing_marker_pub_.publish(marker);
        }
    }

    void HomingVisualization::publishRobotFootprintModel(const PoseSE2 &current_pose, const BaseRobotFootprintModel &robot_model,
                                                         const std::string &ns, const std_msgs::ColorRGBA &color)
    {
        if (printErrorWhenNotInitialized())
            return;

        std::vector<visualization_msgs::Marker> markers;
        robot_model.visualizeRobot(current_pose, markers, color);
        if (markers.empty())
            return;

        int idx = 0;
        for (std::vector<visualization_msgs::Marker>::iterator marker_it = markers.begin(); marker_it != markers.end(); ++marker_it, ++idx)
        {
            marker_it->header.frame_id = frame_id_;
            marker_it->header.stamp = ros::Time::now();
            marker_it->action = visualization_msgs::Marker::ADD;
            marker_it->ns = "RobotFootprintModel";
            marker_it->id = idx;
            marker_it->lifetime = ros::Duration(2.0);
            marker_it->scale.x = 0.1;
            marker_it->scale.y = 0.1;
            homing_marker_pub_.publish(*marker_it);
        }
    }

    std_msgs::ColorRGBA HomingVisualization::toColorMsg(double a, double r, double g, double b)
    {
        std_msgs::ColorRGBA color;
        color.a = a;
        color.r = r;
        color.g = g;
        color.b = b;
        return color;
    }

}
