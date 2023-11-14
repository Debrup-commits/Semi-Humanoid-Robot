#ifndef VISUALIZATION_H_
#define VISUALIZATION_H_
#include <visualization_msgs/Marker.h>
#include <ros/ros.h>
#include <ros/publisher.h>
#include "Eigen/Core"
#include <string>
// boost
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <nav_msgs/Path.h>
#include <base_local_planner/goal_functions.h>
#include <homing_local_planner/obstacles.h>
#include <homing_local_planner/pose_se2.h>
#include <homing_local_planner/robot_footprint_model.h>

namespace homing_local_planner
{
    class HomingVisualization
    {
    public:
        HomingVisualization();
        HomingVisualization(ros::NodeHandle &nh, std::string frame_id);
        void initialize(ros::NodeHandle &nh);
        void publishViaPoints(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &via_points,
                              const std::string &ns = "ViaPoints") const;
        void publishLocalPlan(const std::vector<geometry_msgs::PoseStamped> &local_plan) const;
        void publishGlobalPlan(const std::vector<geometry_msgs::PoseStamped> &global_plan) const;
        void publishObstacles(const ObstContainer &obstacles, double scale = 0.1) const;

        /**
         * @brief Publish the visualization of the robot model
         *
         * @param current_pose Current pose of the robot
         * @param robot_model Subclass of BaseRobotFootprintModel
         * @param ns Namespace for the marker objects
         * @param color Color of the footprint
         */
        void publishRobotFootprintModel(const PoseSE2 &current_pose, const BaseRobotFootprintModel &robot_model, const std::string &ns = "RobotFootprintModel",
                                        const std_msgs::ColorRGBA &color = toColorMsg(0.5, 0.0, 0.8, 0.0));
        /**
         * @brief Helper function to generate a color message from single values
         * @param a Alpha value
         * @param r Red value
         * @param g Green value
         * @param b Blue value
         * @return Color message
         */
        static std_msgs::ColorRGBA toColorMsg(double a, double r, double g, double b);

    protected:
        bool printErrorWhenNotInitialized() const;
        ros::Publisher homing_marker_pub_; //!< Publisher for visualization markers
        ros::Publisher local_plan_pub_;    //!< Publisher for the local plan
        ros::Publisher global_plan_pub_;

        bool initialized_;
        std::string frame_id_;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
    //! Abbrev. for shared instances of the TebVisualization
    typedef boost::shared_ptr<HomingVisualization> HomingVisualizationPtr;
}
#endif /* VISUALIZATION_H_ */