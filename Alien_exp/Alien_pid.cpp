/************************************************************************
Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <unistd.h>
#include <string.h>

#include <Eigen/Dense>
#include <zmq.h>
#include <nlohmann/json.hpp>
#define DEBUG_MODE true
using json = nlohmann::json;
using namespace UNITREE_LEGGED_SDK;

template <typename T>
T clamp(T val, T low, T high) {
    return std::max(low, std::min(val, high));
}


// ##########################################################################################
// ####################################### PID Controller ###################################
// ##########################################################################################
struct Pose {
	Eigen::Vector3d position;
	Eigen::Quaterniond orientation;

	double yaw;
	
	Pose() : position(0, 0, 0), orientation(Eigen::Quaterniond::Identity()) {}
	
	Pose(const Eigen::Vector3d& pos, const Eigen::Quaterniond& orient) 
			: position(pos), orientation(orient) {}
	
	Pose(double x, double y, double z, const Eigen::Quaterniond& orient) 
			: position(x, y, z), orientation(orient) {}
};

class Waypoints {
private:
	struct Point2D {
		double x, y, yaw;
		
		Point2D() : x(0.0), y(0.0), yaw(0.0) {}
		Point2D(double _x, double _y, double _yaw = 0.0) : x(_x), y(_y), yaw(_yaw) {}
		
		Eigen::Vector2d position() const {
			return Eigen::Vector2d(x, y);
		}
	};
	
	std::vector<Point2D> points;
	
public:
	Waypoints() {}
	
	void addPoint(double x, double y, double yaw = 0.0) {
		points.emplace_back(x, y, yaw);
	}
	
	const Point2D& getPoint(size_t index) const {
		if (index < points.size()) {
			return points[index];
		}
		static Point2D defaultPoint;
		return defaultPoint;
	}
	
	Eigen::Vector2d getPosition(size_t index) const {
		return getPoint(index).position();
	}
	
	double getYaw(size_t index) const {
		return getPoint(index).yaw;
	}
	
	size_t size() const {
		return points.size();
	}
	
	bool empty() const {
		return points.empty();
	}
	
	void clear() {
		points.clear();
	}
	
	const Point2D& operator[](size_t index) const {
		return points[index];
	}
	
	const Point2D& back() const {
		return points.back();
	}
	
	typename std::vector<Point2D>::iterator begin() { return points.begin(); }
	typename std::vector<Point2D>::iterator end() { return points.end(); }
	typename std::vector<Point2D>::const_iterator begin() const { return points.begin(); }
	typename std::vector<Point2D>::const_iterator end() const { return points.end(); }
};

// Single-dimension PID controller for x, y, or yaw control
class SingleDimensionPID {
private:
    double kp_, ki_, kd_;    // PID gains
    double min_output_;      // Min output value
    double max_output_;      // Max output value
    double error_sum_;       // Accumulated error (for I term)
    double prev_error_;      // Previous error (for D term)
    double dt_;              // Time step
    
public:
    SingleDimensionPID(
        double kp = 1.0, double ki = 0.0, double kd = 0.0,
        double min_output = -1.0, double max_output = 1.0,
        double dt = 0.01
    ) : kp_(kp), ki_(ki), kd_(kd),
        min_output_(min_output), max_output_(max_output),
        error_sum_(0.0), prev_error_(0.0), dt_(dt) {}
    
    // Reset the controller state
    void reset() {
        error_sum_ = 0.0;
        prev_error_ = 0.0;
    }
    
    // Set the time step
    void setTimeStep(double dt) {
        dt_ = dt;
    }
    
    // Set the output limits
    void setOutputLimits(double min_output, double max_output) {
        min_output_ = min_output;
        max_output_ = max_output;
    }
    
    // Calculate control output based on error
    double calculate(double error) {
        // P term
        double p_term = kp_ * error;
        
        // I term (with anti-windup)
        error_sum_ += error * dt_;
        error_sum_ = clamp(error_sum_, -10.0, 10.0);  // Anti-windup
        double i_term = ki_ * error_sum_;
        
        // D term
        double error_rate = (error - prev_error_) / dt_;
        double d_term = kd_ * error_rate;
        
        // Save error for next iteration
        prev_error_ = error;
        
        // Sum all terms and apply output limits
        double output = p_term + i_term + d_term;
        return clamp(output, min_output_, max_output_);
    }
};

// Main controller that uses three separate PID controllers
class ThreeDimensionPIDController {
public:
    // Constructor with separate gains for each dimension
    ThreeDimensionPIDController(
        // X controller params
        double x_p = 2.0, double x_i = 0.0, double x_d = 0.0,
        // Y controller params
        double y_p = 1.0, double y_i = 0.0, double y_d = 0.0,
        // Yaw controller params
        double yaw_p = 1.0, double yaw_i = 0.0, double yaw_d = 0.0,
        // Limits and tolerances
        double max_linear_vel = 0.3,    // Max linear velocity
        double max_angular_vel = 0.3,   // Max angular velocity
        double position_tolerance = 0.3, // Position tolerance
        double angle_tolerance = 0.2    // Angle tolerance
    ) : max_linear_vel_(max_linear_vel),
        max_angular_vel_(max_angular_vel),
        pos_tol_(position_tolerance),
        ang_tol_(angle_tolerance),
        current_waypoint_idx_(0),
        goal_reached_(true),
        dt_(0.01),  // Time step, adjust as needed
        cmd_linear_x_(0.0),
        cmd_linear_y_(0.0),
        cmd_angular_(0.0)
    {
        // Create the three PID controllers
        x_controller_ = SingleDimensionPID(x_p, x_i, x_d, -max_linear_vel, max_linear_vel, dt_);
        y_controller_ = SingleDimensionPID(y_p, y_i, y_d, -max_linear_vel, max_linear_vel, dt_);
        yaw_controller_ = SingleDimensionPID(yaw_p, yaw_i, yaw_d, -max_angular_vel, max_angular_vel, dt_);
    }
    
    // Set new path
    void setPath(const Waypoints& new_path) {
        path_ = new_path;
        current_waypoint_idx_ = 0;
        
        // Reset PID controllers
        x_controller_.reset();
        y_controller_.reset();
        yaw_controller_.reset();
        
        if (!new_path.empty()) {
            goal_reached_ = false;
        } else {
            goal_reached_ = true;
            std::cout << "Received empty path!" << std::endl;
        }
    }
    
    // Transform a target position from world frame to robot frame
    Eigen::Vector3d getPositionInRobotFrame(const Eigen::Vector3d& world_point, 
        double robot_x, double robot_y, double robot_yaw) {
        // Calculate the transformation
        double cos_yaw = cos(robot_yaw);
        double sin_yaw = sin(robot_yaw);

        // Step 1: Translate - Subtract robot position from world point
        double translated_x = world_point.x() - robot_x;
        double translated_y = world_point.y() - robot_y;

        // Step 2: Rotate - Apply rotation matrix to transform to robot's frame
        double robot_frame_x = translated_x * cos_yaw + translated_y * sin_yaw;
        double robot_frame_y = -translated_x * sin_yaw + translated_y * cos_yaw;

        // Return the point in robot frame coordinates
        return Eigen::Vector3d(
            robot_frame_x,
            robot_frame_y,
            world_point.z()  // Z coordinate typically remains unchanged for 2D transformations
        );
    }

    double normalize_angle(double angle) {
        return std::atan2(std::sin(angle), std::cos(angle));
    }
    
    // // Compute velocities based on current pose
    // void computeVelocities(const Pose& current_pose) {
    //     // Store current pose for use in current_yaw() method
    //     current_pose_ = current_pose;
        
    //     // Check if we have a path and haven't reached the goal
    //     if (path_.empty() || goal_reached_) {
    //         cmd_linear_x_ = 0.0;
    //         cmd_linear_y_ = 0.0;
    //         cmd_angular_ = 0.0;
    //         return;
    //     }
        
    //     // Get current waypoint in world frame
    //     const auto& target = path_[current_waypoint_idx_];
        
    //     // Create 3D point for target (z=0 for 2D navigation)
    //     Eigen::Vector3d target_position(target.x, target.y, 0.0);
        
    //     // Transform target point to robot frame
    //     Eigen::Vector3d target_in_robot_frame = getPositionInRobotFrame(
    //         target_position, 
    //         current_pose.position.x(), 
    //         current_pose.position.y(), 
    //         current_pose.yaw
    //     );
        
    //     // Extract x and y components in robot frame
    //     double dx_robot = target_in_robot_frame.x();
    //     double dy_robot = target_in_robot_frame.y();
        
    //     // Distance to target (same in both frames)
    //     double distance = std::sqrt(dx_robot*dx_robot + dy_robot*dy_robot);
        
    //     // Check if we've reached the current waypoint
    //     if (distance <= pos_tol_) {
    //         // Move to the next waypoint
    //         current_waypoint_idx_++;
            
    //         // Reset PID controllers when switching waypoints
    //         x_controller_.reset();
    //         y_controller_.reset();
    //         yaw_controller_.reset();
            
    //         // Check if we've reached the end of the path
    //         if (current_waypoint_idx_ >= path_.size()) {
    //             goal_reached_ = true;
    //             cmd_linear_x_ = 0.0;
    //             cmd_linear_y_ = 0.0;
    //             cmd_angular_ = 0.0;
    //             std::cout << "Goal reached!" << std::endl;
    //             return;
    //         }
            
    //         // Get new target waypoint
    //         const auto& new_target = path_[current_waypoint_idx_];
            
    //         // Recalculate for new target
    //         Eigen::Vector3d new_target_position(new_target.x, new_target.y, 0.0);
    //         Eigen::Vector3d new_target_in_robot_frame = getPositionInRobotFrame(
    //             new_target_position, 
    //             current_pose.position.x(), 
    //             current_pose.position.y(), 
    //             current_pose.yaw
    //         );

    //         dx_robot = new_target_in_robot_frame.x();
    //         dy_robot = new_target_in_robot_frame.y();
    //         distance = std::sqrt(dx_robot*dx_robot + dy_robot*dy_robot);
    //     }
        
    //     // Calculate errors for each dimension
    //     // For x and y, the error is the difference in robot frame
    //     double x_error = dx_robot;
    //     double y_error = dy_robot;
        
    //     // For yaw, calculate the difference between target yaw and current yaw
    //     double yaw_error = normalize_angle(target.yaw - current_pose_.yaw);
        
    //     // Feed errors to each PID controller
    //     cmd_linear_x_ = x_controller_.calculate(x_error);
    //     cmd_linear_y_ = y_controller_.calculate(y_error);
    //     cmd_angular_ = yaw_controller_.calculate(yaw_error);
        
    //     // Dynamic speed reduction based on heading error or proximity
    //     // If we're significantly off-heading, prioritize turning over moving
    //     if (std::abs(yaw_error) > 0.5) {  // ~30 degrees
    //         cmd_linear_x_ *= 0.5;  // Reduce forward speed to focus on turning
    //         cmd_linear_y_ *= 0.5;  // Reduce lateral speed as well
    //     }
        
    //     // Debug output
    //     if (DEBUG_MODE) {
    //         std::cout << "=========================================================" << std::endl;
    //         std::cout << "Target world: (" << target.x << ", " << target.y << ")" << std::endl;
    //         std::cout << "Target robot: (" << dx_robot << ", " << dy_robot << ")" << std::endl;
    //         std::cout << "Current: (" << current_pose.position.x() << ", " 
    //                   << current_pose.position.y() << "), yaw: " << current_yaw() << std::endl;
    //         std::cout << "Distance: " << distance << ", Yaw error: " << yaw_error << " rad" << std::endl;
    //         std::cout << "Cmd: x=" << cmd_linear_x_ << ", y=" << cmd_linear_y_ << ", angular=" << cmd_angular_ << std::endl;
    //         std::cout << "=========================================================" << std::endl;
    //     }
    // }

	// Compute velocities based on current pose
	void computeVelocities(const Pose& current_pose) {
		// Store current pose for use in current_yaw() method
		current_pose_ = current_pose;
		
		// Check if we have a path and haven't reached the goal
		if (path_.empty() || goal_reached_) {
			cmd_linear_x_ = 0.0;
			cmd_linear_y_ = 0.0;
			cmd_angular_ = 0.0;
			return;
		}
		
		// Get current waypoint in world frame
		const auto& target = path_[current_waypoint_idx_];
		
		// Create 3D point for target (z=0 for 2D navigation)
		Eigen::Vector3d target_position(target.x, target.y, 0.0);
		
		// Transform target point to robot frame
		Eigen::Vector3d target_in_robot_frame = getPositionInRobotFrame(
			target_position, 
			current_pose.position.x(), 
			current_pose.position.y(), 
			current_pose.yaw
		);
		
		// Extract x and y components in robot frame
		double dx_robot = target_in_robot_frame.x();
		double dy_robot = target_in_robot_frame.y();
		
		// Distance to target (same in both frames)
		double distance = std::sqrt(dx_robot*dx_robot + dy_robot*dy_robot);
		
		// Calculate yaw error early so we can use it in the waypoint reached condition
		double yaw_error = normalize_angle(target.yaw - current_pose_.yaw);
		
		// Check if we've reached the current waypoint (both position AND orientation)
		if (distance <= pos_tol_ && std::abs(yaw_error) <= ang_tol_) {
			// Move to the next waypoint
			current_waypoint_idx_++;
			
			// Reset PID controllers when switching waypoints
			x_controller_.reset();
			y_controller_.reset();
			yaw_controller_.reset();
			
			// Check if we've reached the end of the path
			if (current_waypoint_idx_ >= path_.size()) {
				goal_reached_ = true;
				cmd_linear_x_ = 0.0;
				cmd_linear_y_ = 0.0;
				cmd_angular_ = 0.0;
				std::cout << "Goal reached!" << std::endl;
				return;
			}
			
			// Get new target waypoint
			const auto& new_target = path_[current_waypoint_idx_];
			
			// Recalculate for new target
			Eigen::Vector3d new_target_position(new_target.x, new_target.y, 0.0);
			Eigen::Vector3d new_target_in_robot_frame = getPositionInRobotFrame(
				new_target_position, 
				current_pose.position.x(), 
				current_pose.position.y(), 
				current_pose.yaw
			);

			dx_robot = new_target_in_robot_frame.x();
			dy_robot = new_target_in_robot_frame.y();
			distance = std::sqrt(dx_robot*dx_robot + dy_robot*dy_robot);
			
			// Recalculate yaw error for the new target
			yaw_error = normalize_angle(new_target.yaw - current_pose_.yaw);
		}
		
		// Calculate errors for each dimension
		// For x and y, the error is the difference in robot frame
		double x_error = dx_robot;
		double y_error = dy_robot;
		
		// For yaw, we already calculated the error above
		// No need to recalculate: double yaw_error = normalize_angle(target.yaw - current_pose_.yaw);
		
		// Feed errors to each PID controller
		cmd_linear_x_ = x_controller_.calculate(x_error);
		cmd_linear_y_ = y_controller_.calculate(y_error);
		cmd_angular_ = yaw_controller_.calculate(yaw_error);
		
		// Dynamic speed reduction based on heading error or proximity
		// If we're significantly off-heading, prioritize turning over moving
		if (std::abs(yaw_error) > 0.5) {  // ~30 degrees
			cmd_linear_x_ *= 0.5;  // Reduce forward speed to focus on turning
			cmd_linear_y_ *= 0.5;  // Reduce lateral speed as well
		}
		
		// Additional strategy: When close to the waypoint, prioritize orientation
		if (distance < pos_tol_ * 2 && std::abs(yaw_error) > ang_tol_) {
			// We're close to the waypoint but orientation is off
			// Reduce linear velocities even more to focus on orientation
			cmd_linear_x_ *= 0.3;
			cmd_linear_y_ *= 0.3;
		}
		
		// Debug output
		if (DEBUG_MODE) {
			std::cout << "=========================================================" << std::endl;
			std::cout << "Target world: (" << target.x << ", " << target.y << "), yaw: " << target.yaw << std::endl;
			std::cout << "Target robot: (" << dx_robot << ", " << dy_robot << ")" << std::endl;
			std::cout << "Current: (" << current_pose.position.x() << ", " 
					<< current_pose.position.y() << "), yaw: " << current_yaw() << std::endl;
			std::cout << "Distance: " << distance << ", Yaw error: " << yaw_error << " rad" << std::endl;
			std::cout << "Position tolerance: " << pos_tol_ << ", Angle tolerance: " << ang_tol_ << std::endl;
			std::cout << "Cmd: x=" << cmd_linear_x_ << ", y=" << cmd_linear_y_ << ", angular=" << cmd_angular_ << std::endl;
			std::cout << "=========================================================" << std::endl;
		}
	}
    
    // Helper to extract yaw angle from pose
    double current_yaw() const {
        return current_pose_.yaw;
    }
    
    // Get current velocity commands
    double getLinearVelocityX() const {
        return cmd_linear_x_;
    }
    
    double getLinearVelocityY() const {
        return cmd_linear_y_;
    }
    
    double getAngularVelocity() const {
        return cmd_angular_;
    }
    
    // Check if goal has been reached
    bool isGoalReached() const {
        return goal_reached_;
    }
    
    // Set the position tolerance
    void setPositionTolerance(double tolerance) {
        pos_tol_ = tolerance;
    }
    
    // Set the angle tolerance
    void setAngleTolerance(double tolerance) {
        ang_tol_ = tolerance;
    }
    
    // Set the time step
    void setTimeStep(double dt) {
        dt_ = dt;
        x_controller_.setTimeStep(dt);
        y_controller_.setTimeStep(dt);
        yaw_controller_.setTimeStep(dt);
    }
    
    // Public access to current pose (needed for Custom class)
    Pose current_pose_;
    
private:
    // Individual PID controllers for each dimension
    SingleDimensionPID x_controller_;
    SingleDimensionPID y_controller_;
    SingleDimensionPID yaw_controller_;
    
    // Path
    Waypoints path_;
    
    // Control limits
    double max_linear_vel_;
    double max_angular_vel_;
    
    // Tolerances
    double pos_tol_;
    double ang_tol_;
    
    // Path following state
    size_t current_waypoint_idx_;
    bool goal_reached_;
    double dt_;
    
    // Command outputs
    double cmd_linear_x_;
    double cmd_linear_y_;
    double cmd_angular_;
};


class Custom
{
public:
    Custom(uint8_t level): safe(LeggedType::A1), udp(level){
        udp.InitCmdData(cmd);

        try {
			// zmq_context = new zmq::context_t();
			// zmq_socket = new zmq::socket_t(*zmq_context, zmq::socket_type::sub);
			
			// zmq_socket->connect("tcp://128.61.44.239:5555");
			// zmq_socket->set(zmq::sockopt::subscribe, "");
			// std::cout << "Connected to MoCap server" << std::endl;

			// zmq_planner_context = new zmq::context_t();
			// zmq_planner_socket = new zmq::socket_t(*zmq_planner_context, zmq::socket_type::sub);
			
			// zmq_planner_socket->connect("tcp://128.61.21.180:5556");
			// zmq_planner_socket->set(zmq::sockopt::subscribe, "");
			// std::cout << "Connected to planner server" << std::endl;

            my_zmq_ctx = zmq_ctx_new();
            // Create zmq socket
            my_zmq_sock = zmq_socket(my_zmq_ctx, ZMQ_SUB);
            
            // Connect to MoCap server
            zmq_connect(my_zmq_sock, "tcp://128.61.44.239:5555");
            // Set subscription filter to receive everything
            zmq_setsockopt(my_zmq_sock, ZMQ_SUBSCRIBE, "", 0);
            std::cout << "Connected to MoCap server" << std::endl;

            // Create planner context
            my_zmq_planner_ctx = zmq_ctx_new();
            // Create planner socket
            my_zmq_planner_sock = zmq_socket(my_zmq_planner_ctx, ZMQ_SUB);
            
            // Connect to planner server
            zmq_connect(my_zmq_planner_sock, "tcp://128.61.21.180:5556");
            // Set subscription filter to receive everything
            zmq_setsockopt(my_zmq_planner_sock, ZMQ_SUBSCRIBE, "", 0);
            std::cout << "Connected to planner server" << std::endl;
		} catch (const std::exception& e) {
			std::cerr << "Failed to connect to MoCap server: " << e.what() << std::endl;
		}
		
		robot_pose = Pose();
        
		// Create three-dimension PID controller with tuned parameters
		// X PID: P=1.0, I=0.05, D=0.1
		// Y PID: P=1.0, I=0.05, D=0.1
		// Yaw PID: P=1.0, I=0.0, D=0.3
		controller = ThreeDimensionPIDController(
		    1.0, 0.05, 0.1,  // X PID gains
		    6.0, 0.05, 0.1,  // Y PID gains
		    0.5, 0.0, 0.3,   // Yaw PID gains
		    0.3, 0.3, 0.3, 0.2);  // Max velocities and tolerances
		
		// Set the sampling time for the controller
		controller.setTimeStep(dt);

		// Wait a moment to get initial pose
		sleep(1);
		updatePoseFromMocap();

		// Set waypoints for testing
		setWaypoints();
    }

    ~Custom(){
		// delete zmq_socket;
		// delete zmq_context;
		// delete zmq_planner_socket;
		// delete zmq_planner_context;

        zmq_close(my_zmq_sock);
        zmq_ctx_destroy(my_zmq_ctx);

        zmq_close(my_zmq_planner_sock);
        zmq_ctx_destroy(my_zmq_planner_ctx);

	}

    void UDPRecv();
    void UDPSend();
    void RobotControl();

    // ************************************
    void setTargetPoint(double x, double y, double yaw = 0.0);
    void wayppointInterpolation();
    void updatePoseFromMocap();
    void updateTargetPointFromPlanner();
    void RobotFollower();
    void setWaypoints();

    Safety safe;
    UDP udp;
    HighCmd cmd = {0};
    HighState state = {0};
    int motiontime = 0;
    float dt = 0.002;     // 0.001~0.01

    // ****************************
    ThreeDimensionPIDController controller;
	void* my_zmq_ctx;
    void* my_zmq_sock;
    void* my_zmq_planner_ctx;
    void* my_zmq_planner_sock;

    double last_timestamp = 0.0; 

    Pose robot_pose;
	std::vector<Pose> obstacles_pose;
    Eigen::Vector3d goal_position;
    Waypoints path;
};


void Custom::setTargetPoint(double x, double y, double yaw) {
    // Add the target point
    goal_position.x() = x;
    goal_position.y() = y;
    goal_position.z() = yaw;
    
    if(DEBUG_MODE) {
        std::cout << "Set new target point: (" << x << ", " << y << ", " << yaw << ")" << std::endl;
    }
}

void Custom::setWaypoints(){
    std::vector<Eigen::Vector3d> waypoint_pos;
    
    // Example rectangular path with orientation
    // waypoint_pos.push_back(Eigen::Vector3d(2.0, 0.6, 0));
    // waypoint_pos.push_back(Eigen::Vector3d(4.0, 0.6, 1.57));
    // waypoint_pos.push_back(Eigen::Vector3d(4.0, 1.6, 3.14));
    // waypoint_pos.push_back(Eigen::Vector3d(2.0, 1.6, -1.57));
    // waypoint_pos.push_back(Eigen::Vector3d(2.0, 0.6, 0));

    waypoint_pos.push_back(Eigen::Vector3d(3.0, 0.6, 1.57));
    waypoint_pos.push_back(Eigen::Vector3d(3.0, 2.7, 3.14));
    waypoint_pos.push_back(Eigen::Vector3d(-0.8, 2.7, -2.9));
    waypoint_pos.push_back(Eigen::Vector3d(-0.8, 1.0, 0));
    waypoint_pos.push_back(Eigen::Vector3d(0.7, 1.0, 0));
    waypoint_pos.push_back(Eigen::Vector3d(0.7, 0.5, 0));
    waypoint_pos.push_back(Eigen::Vector3d(1.2, 0.5, 0));
    waypoint_pos.push_back(Eigen::Vector3d(3.0, 0.6, 0));

    for(auto waypoint:waypoint_pos){
        path.addPoint(waypoint.x(), waypoint.y(), waypoint.z());
    }

    controller.setPath(path);
}

void Custom::wayppointInterpolation(){
    path.clear();
        
    // Get current position from robot_pose
    Eigen::Vector2d start(robot_pose.position.x(), robot_pose.position.y());
    
    // Get target position
    Eigen::Vector2d end(goal_position.x(), goal_position.y());
    
    // Calculate distance between current position and target
    double distance = (end - start).norm();
    
    // If distance is below threshold, just add the target point
    const double DISTANCE_THRESHOLD = 0.5; // meters
    if (distance <= DISTANCE_THRESHOLD) {
        path.addPoint(goal_position.x(), goal_position.y(), goal_position.z()); // Use target's yaw value
        return;
    }
    
    // For longer distances, create interpolated waypoints
    // Calculate number of points based on distance
    const double POINT_SPACING = 0.5; // meters between points
    std::cout<<static_cast<int>(distance / POINT_SPACING)<<std::endl;
    int num_points = std::min(30, static_cast<int>(distance / POINT_SPACING));
    
    // Direction vector
    Eigen::Vector2d direction = (end - start).normalized();
    
    // Calculate heading/yaw from direction vector
    double path_yaw = std::atan2(direction.y(), direction.x());
    
    // Get current robot's yaw angle
    Eigen::Matrix3d rot_matrix = robot_pose.orientation.toRotationMatrix();
    double current_yaw = std::atan2(rot_matrix(1, 0), rot_matrix(0, 0));
    
    // Target yaw angle (stored in goal_position.z())
    double target_yaw = goal_position.z();
    
    // Add intermediate points
    for (int i = 0; i < num_points; i++) {
        double ratio = static_cast<double>(i) / (num_points - 1);
        
        // Linear position interpolation
        Eigen::Vector2d point = start + ratio * (end - start);
        
        // Yaw angle linear interpolation
        // Normalize angle difference to ensure shortest path rotation
        double yaw_diff = target_yaw - current_yaw;
        
        // Normalize angle difference to [-π, π] range
        while (yaw_diff > M_PI) yaw_diff -= 2 * M_PI;
        while (yaw_diff < -M_PI) yaw_diff += 2 * M_PI;
        
        // Calculate interpolated yaw
        double interpolated_yaw = current_yaw + ratio * yaw_diff;
        
        // Add point to path
        path.addPoint(point.x(), point.y(), interpolated_yaw);
    }
    
    if (DEBUG_MODE) {
        std::cout << "Generated " << path.size() << " waypoints for a " 
                << distance << "m path" << std::endl;
        std::cout << "Yaw interpolation: start=" << current_yaw * 180/M_PI 
                << "°, end=" << target_yaw * 180/M_PI << "°" << std::endl;
    }
}

void Custom::updatePoseFromMocap() {
    try {
        // zmq::message_t message;
        char buffer[4096] = {0};
        // auto result = zmq_socket->recv(message, zmq::recv_flags::dontwait);
        // int recv_len = zmq_recv(zmq_socket, buffer, sizeof(buffer) - 1, ZMQ_DONTWAIT);
        
        // char buffer[4096] = {0};
        int recv_len = zmq_recv(my_zmq_sock, buffer, sizeof(buffer) - 1, ZMQ_DONTWAIT);
        
        if (recv_len > 0) {
            // std::string msg_str = message.to_string();
            buffer[recv_len] = '\0';
            std::string msg_str(buffer);
            
            if(DEBUG_MODE) {
                std::cout << "Received MoCap message" << std::endl;
            }

            json msg_json = json::parse(msg_str);
            
            // Clear obstacles vector to refresh with new data
            obstacles_pose.clear();
            
            if (msg_json.contains("robots")) {
                auto& robots = msg_json["robots"];
                
                // Process robot pose
                if (robots.contains("Go1")) {
                    auto& robot = robots["Go1"];
                    
                    if (robot.contains("position") && robot.contains("rotation")) {
                        auto& pos = robot["position"];
                        auto& rot = robot["rotation"];
                        auto& yaw = robot["yaw"];
                        
                        // Update robot pose
                        robot_pose.position = Eigen::Vector3d(
                            pos[0].get<double>(),
                            pos[1].get<double>(),
                            pos[2].get<double>()
                        );
                        
                        robot_pose.orientation = Eigen::Quaterniond(
                            rot[3].get<double>(),  // w    
                            rot[0].get<double>(),  // x
                            rot[1].get<double>(),  // y
                            rot[2].get<double>()   // z
                        );
                        robot_pose.yaw = yaw;
                    }
                }
                
                // Process all obstacles
                for (auto& [name, data] : robots.items()) {
                    // Skip our robot
                    if (name == "Go1" or name == "Go2") continue;
                    
                    // Process any object with "obstacle" in the name
                    if (name.find("obstacle") != std::string::npos) {
                        if (data.contains("position") && data.contains("rotation")) {
                            auto& pos = data["position"];
                            auto& rot = data["rotation"];
                            
                            // Create new Pose for this obstacle
                            Pose obstacle_pose;
                            obstacle_pose.position = Eigen::Vector3d(
                                pos[0].get<double>(),
                                pos[1].get<double>(),
                                pos[2].get<double>()
                            );
                            
                            obstacle_pose.orientation = Eigen::Quaterniond(
                                rot[3].get<double>(),  // w    
                                rot[0].get<double>(),  // x
                                rot[1].get<double>(),  // y
                                rot[2].get<double>()   // z
                            );
                            
                            // Add to obstacles vector
                            obstacles_pose.push_back(obstacle_pose);
                        }
                    }
                }
                
                // Record timestamp
                if (msg_json.contains("timestamp")) {
                    last_timestamp = msg_json["timestamp"].get<double>();
                }
            }
            
            if(DEBUG_MODE){
                // Print robot position
                std::cout << "Robot position: x:" << robot_pose.position.x() << " y:" 
                << robot_pose.position.y() << " orientation: " 
                << robot_pose.orientation.x() << " " << robot_pose.orientation.y() << " " 
                << robot_pose.orientation.z() << " " << robot_pose.orientation.w() << std::endl;
    
                // Print obstacle information
                std::cout << "Found " << obstacles_pose.size() << " obstacles" << std::endl;
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Error getting MoCap data: " << e.what() << std::endl;
    }
}

void Custom::updateTargetPointFromPlanner() {
    try {
        // zmq::message_t message;
        // auto result = zmq_planner_socket->recv(message, zmq::recv_flags::dontwait);

        char buffer[4096] = {0};
        // int recv_len = zmq_recv(zmq_planner_socket, buffer, sizeof(buffer) - 1, ZMQ_DONTWAIT);
        int recv_len = zmq_recv(my_zmq_planner_sock, buffer, sizeof(buffer) - 1, ZMQ_DONTWAIT);
        
        
        
        if (recv_len > 0) {
            // std::string msg_str = message.to_string();

            buffer[recv_len] = '\0';
            std::string msg_str(buffer);
            
            if(DEBUG_MODE) {
                std::cout << "Received Planner message: " << msg_str << std::endl;
            }

            json msg_json = json::parse(msg_str);
            
            // Check if the message contains waypoints
            if (msg_json.contains("waypoints") && msg_json["waypoints"].is_array()) {
                // Clear existing path
                path.clear();
                
                // Access the waypoints array
                auto& waypoints_array = msg_json["waypoints"];
                
                if(DEBUG_MODE) {
                    std::cout << "Found " << waypoints_array.size() << " waypoints" << std::endl;
                }
                
                // Process each waypoint
                for (size_t i = 1; i < waypoints_array.size()-2; i++) {

                    // Each waypoint is an array [x, y]
                    if (waypoints_array[i].is_array() && waypoints_array[i].size() >= 2) {
                        double x = waypoints_array[i][0].get<double>();
                        double y = waypoints_array[i][1].get<double>();
                        
                        // Calculate yaw (heading) if not the last point
                        double yaw = 0.0;
                        if (i < waypoints_array.size() - 1 && waypoints_array[i+1].size() >= 2) {
                            double next_x = waypoints_array[i+1][0].get<double>();
                            double next_y = waypoints_array[i+1][1].get<double>();
                            yaw = std::atan2(next_y - y, next_x - x);
                            if(i == waypoints_array.size()-3)	yaw = 0.0;
                        }
                        
                        // Add to the path
                        path.addPoint(x, y, yaw);
                        
                        if(DEBUG_MODE) {
                            std::cout << "  Waypoint " << i << ": (" << x << ", " << y 
                                    << "), yaw: " << yaw << std::endl;
                        }
                    }
                }
                
                // After processing all waypoints, set the path in the controller
                controller.setPath(path);
                
                if(DEBUG_MODE) {
                    std::cout << "Set " << path.size() << " waypoints in the controller" << std::endl;
                }
            } else {
                if(DEBUG_MODE) {
                    std::cout << "Message doesn't contain valid waypoints array" << std::endl;
                }
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Error processing planner data: " << e.what() << std::endl;
    }
}

void Custom::UDPRecv()
{
    udp.Recv();
}

void Custom::UDPSend()
{  
    udp.Send();
}

void Custom::RobotFollower(){
    motiontime += 2;
    udp.GetRecv(state);

    // cmd.speedLevel = 0;

    updatePoseFromMocap();
    // updateTargetPointFromPlanner();
    // Use PID controller to compute control commands
    controller.computeVelocities(robot_pose);
    
    // If the goal hasn't been reached, send control commands
    if (!controller.isGoalReached()) {
        cmd.mode = 2;  // Velocity control mode
        // cmd.gaitType = 1;  // Walking gait

        // Get computed linear and angular velocities
        // double linear_vel = controller.getLinearVelocity();
        double angular_vel = controller.getAngularVelocity();

        double vx = controller.getLinearVelocityX();
        double vy = controller.getLinearVelocityY();
        
        // Send control commands to the robot
        // sport_client.Move(vx, vy, angular_vel);

        vx *= 10;
        vy*= 10;
        angular_vel *= 10;

        if(vx > 0.3)	vx = 0.3;
        if(vx < -0.3)	vx = -0.3;

        if(vy > 0.3)	vy = 0.3;
        if(vy < -0.3)	vy = -0.3;

        if(angular_vel > 0.3)	angular_vel = 0.3;
        if(angular_vel < -0.3)	angular_vel = -0.3;
        std::cout<<"vx: "<<vx<<"vy: "<<vy<<"angular vel:"<<angular_vel<<std::endl;

        cmd.forwardSpeed = vx;
        cmd.sideSpeed = vy;  // No lateral movement
        cmd.rotateSpeed = angular_vel;
        
        if(DEBUG_MODE){
            std::cout << "PID Control: vx=" << vx << "vy "<<vy <<", w=" << angular_vel << std::endl;
            std::cout << "Position: " << robot_pose.position.x() << ", " 
                                    << robot_pose.position.y() << ", " 
                                    << robot_pose.position.z() << std::endl;
        }
    } else {
        // Goal reached, stop moving
        // sport_client.StopMove();
        cmd.mode = 0;
        // cmd.gaitType = 0;
        cmd.forwardSpeed = 0.0f;
        cmd.sideSpeed = 0.0f;
        cmd.rotateSpeed = 0.0f;
        std::cout << "PID Controller: Goal reached!" << std::endl;
    }
    udp.SetSend(cmd);
}


void Custom::RobotControl() 
{
    motiontime += 2;
    udp.GetRecv(state);
    printf("%d   %f\n", motiontime, state.imu.quaternion[2]);

    cmd.forwardSpeed = 0.0f;
    cmd.sideSpeed = 0.0f;
    cmd.rotateSpeed = 0.0f;
    cmd.bodyHeight = 0.0f;

    cmd.mode = 0;      // 0:idle, default stand      1:forced stand     2:walk continuously
    cmd.roll  = 0;
    cmd.pitch = 0;
    cmd.yaw = 0;

    if(motiontime>1000 && motiontime<1500){
        cmd.mode = 1;
        cmd.roll = 0.5f;
    }

    if(motiontime>1500 && motiontime<2000){
        cmd.mode = 1;
        cmd.pitch = 0.3f;
    }

    if(motiontime>2000 && motiontime<2500){
        cmd.mode = 1;
        cmd.yaw = 0.3f;
    }

    if(motiontime>2500 && motiontime<3000){
        cmd.mode = 1;
        cmd.bodyHeight = -0.3f;
    }

    if(motiontime>3000 && motiontime<3500){
        cmd.mode = 1;
        cmd.bodyHeight = 0.3f;
    }

    if(motiontime>3500 && motiontime<4000){
        cmd.mode = 1;
        cmd.bodyHeight = 0.0f;
    }

    if(motiontime>4000 && motiontime<5000){
        cmd.mode = 2;
    }

    if(motiontime>5000 && motiontime<8500){
        cmd.mode = 2;
        cmd.forwardSpeed = 0.1f; // -1  ~ +1
    }

    if(motiontime>8500 && motiontime<12000){
        cmd.mode = 2;
        cmd.forwardSpeed = -0.2f; // -1  ~ +1
    }

    if(motiontime>12000 && motiontime<16000){
        cmd.mode = 2;
        cmd.rotateSpeed = 0.3f;   // turn
    }

    if(motiontime>16000 && motiontime<20000){
        cmd.mode = 2;
        cmd.rotateSpeed = -0.3f;   // turn
    }

    if(motiontime>20000 ){
        cmd.mode = 1;
    }

    udp.SetSend(cmd);
}

int main(void) 
{
    std::cout << "Communication level is set to HIGH-level." << std::endl
              << "WARNING: Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    Custom custom(HIGHLEVEL);
    // InitEnvironment();
    // LoopFunc loop_control("control_loop", custom.dt,    boost::bind(&Custom::RobotControl, &custom));
    LoopFunc loop_control("control_loop", custom.dt,    boost::bind(&Custom::RobotFollower, &custom));
    LoopFunc loop_udpSend("udp_send",     custom.dt, 3, boost::bind(&Custom::UDPSend,      &custom));
    LoopFunc loop_udpRecv("udp_recv",     custom.dt, 3, boost::bind(&Custom::UDPRecv,      &custom));

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();

    while(1){
        sleep(10);
    };

    return 0; 
}
