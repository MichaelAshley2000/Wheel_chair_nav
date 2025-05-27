#include <ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

// Wheelchair Physical Parameters
const float WHEEL_BASE = 0.55;          // Distance between wheels in meters
const float WHEEL_RADIUS = 0.15;        // Wheel radius in meters
const float MAX_LINEAR_SPEED = 0.5;     // Maximum linear speed in m/s
const float MAX_ANGULAR_SPEED = 0.8;    // Maximum angular speed in rad/s

// Global variables for odometry
float x = 0.0;
float y = 0.0;
float theta = 0.0;
unsigned long last_odom_update = 0;

// ROS messages
nav_msgs::Odometry odom_msg;
ros::Publisher odom_pub("odom", &odom_msg);
geometry_msgs::TransformStamped odom_trans;
tf::TransformBroadcaster odom_broadcaster;

// Function to convert motor speeds to real-world velocities
void calculateRealVelocities(int16_t leftSpeed, int16_t rightSpeed, float& linear_vel, float& angular_vel) {
    // Convert from motor commands (-200 to 200) to actual velocities
    float left_speed_mps = (float)leftSpeed / 200.0 * MAX_LINEAR_SPEED;
    float right_speed_mps = (float)rightSpeed / 200.0 * MAX_LINEAR_SPEED;
    
    // Calculate robot's linear and angular velocity
    linear_vel = (right_speed_mps + left_speed_mps) / 2.0;
    angular_vel = (right_speed_mps - left_speed_mps) / WHEEL_BASE;
}

// Update and publish odometry
void updateOdometry() {
    unsigned long current_time = millis();
    float dt = (current_time - last_odom_update) / 1000.0;  // Convert to seconds
    
    if (dt < 0.01) return;  // Minimum update interval (10ms)
    
    // Calculate real-world velocities
    float linear_vel, angular_vel;
    calculateRealVelocities(currentLeftSpeed, currentRightSpeed, linear_vel, angular_vel);
    
    // Update position and orientation
    float delta_x = linear_vel * cos(theta) * dt;
    float delta_y = linear_vel * sin(theta) * dt;
    float delta_th = angular_vel * dt;
    
    x += delta_x;
    y += delta_y;
    theta += delta_th;
    
    // Normalize theta between -π and π
    if (theta > PI) theta -= 2 * PI;
    if (theta < -PI) theta += 2 * PI;
    
    // Create quaternion from theta
    geometry_msgs::Quaternion odom_quat;
    odom_quat.z = sin(theta/2.0);
    odom_quat.w = cos(theta/2.0);
    
    // Publish transform first
    odom_trans.header.stamp = nh.now();
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    
    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    
    odom_broadcaster.sendTransform(odom_trans);
    
    // Publish odometry message
    odom_msg.header.stamp = nh.now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";
    
    // Set position
    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = odom_quat;
    
    // Set velocity
    odom_msg.twist.twist.linear.x = linear_vel;
    odom_msg.twist.twist.angular.z = angular_vel;
    
    // Set covariance (simplified)
    for(int i = 0; i < 36; i++) {
        odom_msg.pose.covariance[i] = 0.0;
        odom_msg.twist.covariance[i] = 0.0;
    }
    // Add small uncertainty to position and velocity
    odom_msg.pose.covariance[0] = 0.01;   // x
    odom_msg.pose.covariance[7] = 0.01;   // y
    odom_msg.pose.covariance[35] = 0.01;  // theta
    
    odom_pub.publish(&odom_msg);
    
    last_odom_update = current_time;
}

// Modify your existing setup() function
void setup() {
    // ... existing setup code ...
    
    // Initialize ROS publishers
    nh.advertise(odom_pub);
    
    // Initialize variables
    last_odom_update = millis();
}

// Modify your existing loop() function
void loop() {
    nh.spinOnce();
    
    // Update motor speeds (your existing rampMotorSpeeds function)
    rampMotorSpeeds();
    
    // Update and publish odometry
    updateOdometry();
    
    delay(10);
}