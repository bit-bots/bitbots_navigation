#ifndef bitbots_local_planner_FTC_PLANNER_H_
#define bitbots_local_planner_FTC_PLANNER_H_

#include <nav_msgs/Odometry.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>
#include <Eigen/Core>
#include <ros/ros.h>
#include <cmath>
#include <dynamic_reconfigure/server.h>
#include <bitbots_local_planner/BBPlannerConfig.h>
#include <bitbots_local_planner/transform_global_plan.h>
#include <nav_core/base_local_planner.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <string>
#include <angles/angles.h>
#include <costmap_2d/costmap_2d.h>

namespace bitbots_local_planner
{

    class BBPlanner : public nav_core::BaseLocalPlanner
    {

    public:
        BBPlanner();
        /**
         * @brief  Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base
         * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
         * @return True if a valid velocity command was found, false otherwise
         */
        bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

        /**
         * @brief  Check if the goal pose has been achieved by the local planner
         * @return True if achieved, false otherwise
         */
        bool isGoalReached();

        /**
         * @brief  Set the plan that the local planner is following
         * @param plan The plan to pass to the local planner
         * @return True if the plan was updated successfully, false otherwise
         */
        bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);

        /**
         * @brief Constructs the local planner
         * @param name The name to give this instance of the local planner
         * @param tf A pointer to a transform buffer
         * @param costmap_ros The cost map to use for assigning costs to local plans
         */
        void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

        ~BBPlanner();

    private:

        /**
        *@brief Reconfigure config_
        */
        void reconfigureCB(BBPlannerConfig &config, uint32_t level);

        /**
        *@brief Publish the global plan for visulatation.
        *@param points where jused to calculate plan.
        */
        void publishPlan(int max_point);

        int checkMaxDistance(tf::Stamped<tf::Pose> current_pose);

        //used for transformation
        tf2_ros::Buffer* tf_;
        //global plan which we run along
        std::vector<geometry_msgs::PoseStamped> global_plan_;
        //transformed global plan in global frame with only the points with are needed for calculation (max_points)
        std::vector<geometry_msgs::PoseStamped> transformed_global_plan_;
        //last point of the global plan in global frame
        tf::Stamped<tf::Pose> goal_pose_;
        // true if the robot should rotate to gobal plan if new global goal set
        tf::Stamped<tf::Pose> old_goal_pose_;
        // The final pose
        tf::Stamped<tf::Pose> end_pose_;
        //for dynamic reconfigure
        dynamic_reconfigure::Server<BBPlannerConfig> *dsrv_;
        //start config
        bitbots_local_planner::BBPlannerConfig default_config_;
        //reconfigure config
        bitbots_local_planner::BBPlannerConfig config_;
        //true if the goal point is reache and orientation of goal is reached
        bool goal_reached_;
        //opublisher where the local plan for visulatation is published
        ros::Publisher local_plan_publisher_;

        costmap_2d::Costmap2DROS* costmap_ros_;
    };
};
#endif