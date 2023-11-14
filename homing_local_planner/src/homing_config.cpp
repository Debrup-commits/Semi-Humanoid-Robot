#include <homing_local_planner/homing_config.h>
namespace homing_local_planner
{
    void HomingConfig::loadRosParamFromNodeHandle(const ros::NodeHandle &nh)
    {
        // Robot
        nh.param("max_vel_x", robot.max_vel_x, robot.max_vel_x);
        nh.param("max_vel_x_backwards", robot.max_vel_x_backwards, robot.max_vel_x_backwards);
        nh.param("max_vel_theta", robot.max_vel_theta, robot.max_vel_theta);
        nh.param("acc_lim_x", robot.acc_lim_x, robot.acc_lim_x);
        nh.param("acc_lim_theta", robot.acc_lim_theta, robot.acc_lim_theta);

        // GoalTolerance
        nh.param("xy_goal_tolerance", goal_tolerance.xy_goal_tolerance, goal_tolerance.xy_goal_tolerance);
        nh.param("yaw_goal_tolerance", goal_tolerance.yaw_goal_tolerance, goal_tolerance.yaw_goal_tolerance);

        // Trajectory
        nh.param("max_global_plan_lookahead_dist", trajectory.max_global_plan_lookahead_dist, trajectory.max_global_plan_lookahead_dist);
        nh.param("global_plan_viapoint_sep", trajectory.global_plan_viapoint_sep, trajectory.global_plan_viapoint_sep);
        nh.param("global_plan_goal_sep", trajectory.global_plan_goal_sep, trajectory.global_plan_goal_sep);
        nh.param("global_plan_prune_distance", trajectory.global_plan_prune_distance, trajectory.global_plan_prune_distance);

        // Optimization
        nh.param("k_rho", optimization.k_rho, optimization.k_rho);
        nh.param("k_alpha", optimization.k_alpha, optimization.k_alpha);
        nh.param("k_phi", optimization.k_phi, optimization.k_phi);
    }

    void HomingConfig::reconfigure(HomingLocalPlannerReconfigureConfig &cfg)
    {
        boost::mutex::scoped_lock l(config_mutex_);
        // Robot
        robot.max_vel_x = cfg.max_vel_x;
        robot.max_vel_x_backwards = cfg.max_vel_x_backwards;
        robot.max_vel_theta = cfg.max_vel_theta;
        robot.acc_lim_x = cfg.acc_lim_x;
        robot.acc_lim_theta = cfg.acc_lim_theta;

        // GoalTolerance
        goal_tolerance.xy_goal_tolerance = cfg.xy_goal_tolerance;
        goal_tolerance.yaw_goal_tolerance = cfg.yaw_goal_tolerance;

        // Trajectory
        trajectory.max_global_plan_lookahead_dist = cfg.max_global_plan_lookahead_dist;
        trajectory.global_plan_viapoint_sep = cfg.global_plan_viapoint_sep;
        trajectory.global_plan_goal_sep = cfg.global_plan_goal_sep;
        trajectory.global_plan_prune_distance = cfg.global_plan_prune_distance;

        // Optimization
        optimization.k_rho = cfg.k_rho;
        optimization.k_alpha = cfg.k_alpha;
        optimization.k_phi = cfg.k_phi;
    }
}