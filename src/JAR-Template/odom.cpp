#include "vex.h"

/**
 * Setter method for tracker center distances.
 * The forward tracker center distance is the horizontal distance from the 
 * center of the robot to the center of the wheel the sensor is measuring.
 * The sideways tracker center distance is the vertical distance from the 
 * center of the robot to the center of the sideways wheel being measured.
 * If there's really no sideways wheel we set the center distance to 0 and
 * pretend the wheel never spins, which is equivalent to a no-drift robot.
 * 
 * @param ForwardTracker_center_distance A horizontal distance to the wheel center in inches.
 * @param SidewaysTracker_center_distance A vertical distance to the wheel center in inches.
 */

void Odom::set_physical_distances(float ForwardTracker_center_distance, float SidewaysTracker_center_distance){
  this->ForwardTracker_center_distance = ForwardTracker_center_distance;
  this->SidewaysTracker_center_distance = SidewaysTracker_center_distance;
}

Odom::Odom(){
  ForwardTracker_center_distance = 0;
  SidewaysTracker_center_distance = 0;
  ForwardTracker_position = 0;
  SideWaysTracker_position = 0;
  X_position = 0;
  Y_position = 0;
  orientation_deg = 0;
  kalman_X_position = 0;
  kalman_Y_position = 0;
  kalman_Px = 1;
  kalman_Py = 1;
  kalman_initialized = false;
  prev_velocity_x = 0;
  prev_velocity_y = 0;
  last_update_time = 0;
}

/**
 * Resets the position, including tracking wheels.
 * Position is field-centric, and orientation is such that 0 degrees
 * is in the positive Y direction. Orientation can be provided with 
 * some flexibility, including less than 0 and greater than 360.
 * 
 * @param X_position Field-centric x position of the robot.
 * @param Y_position Field-centric y position of the robot.
 * @param orientation_deg Field-centered, clockwise-positive, orientation.
 * @param ForwardTracker_position Current position of the sensor in inches.
 * @param SidewaysTracker_position Current position of the sensor in inches.
 */

void Odom::set_position(float X_position, float Y_position, float orientation_deg, float ForwardTracker_position, float SidewaysTracker_position){
  this->ForwardTracker_position = ForwardTracker_position;
  this->SideWaysTracker_position = SidewaysTracker_position;
  this->X_position = X_position;
  this->Y_position = Y_position;
  this->orientation_deg = orientation_deg;
}

/**
 * Does the odometry math to update position
 * Uses the Pilons arc method outline here: https://wiki.purduesigbots.com/software/odometry
 * All the deltas are done by getting member variables and comparing them to 
 * the input. Ultimately this all works to update the public member variable
 * X_position. This function needs to be run at 200Hz or so for best results.
 * 
 * @param ForwardTracker_position Current position of the sensor in inches.
 * @param SidewaysTracker_position Current position of the sensor in inches.
 * @param orientation_deg Field-centered, clockwise-positive, orientation.
 */

void Odom::update_position(float ForwardTracker_position, float SidewaysTracker_position, float orientation_deg){
  // Calculate time delta for adaptive filtering (in milliseconds)
  uint32_t current_time = vex::timer::system();
  float dt = 0.005f; // Default 5ms (200Hz) if first run
  if (last_update_time > 0) {
    dt = (current_time - last_update_time) / 1000.0f; // Convert to seconds
    // Clamp dt to reasonable values to prevent instability
    if (dt < 0.001f) dt = 0.001f; // Min 1ms
    if (dt > 0.1f) dt = 0.1f;     // Max 100ms
  }
  last_update_time = current_time;

  // Calculate deltas
  float Forward_delta = ForwardTracker_position - this->ForwardTracker_position;
  float Sideways_delta = SidewaysTracker_position - this->SideWaysTracker_position;
  this->ForwardTracker_position = ForwardTracker_position;
  this->SideWaysTracker_position = SidewaysTracker_position;
  
  float orientation_rad = to_rad(orientation_deg);
  float prev_orientation_rad = to_rad(this->orientation_deg);
  float orientation_delta_rad = orientation_rad - prev_orientation_rad;
  this->orientation_deg = orientation_deg;

  // Calculate local position changes using arc method
  float local_X_position;
  float local_Y_position;

  // Use small angle approximation for better accuracy when angle change is tiny
  const float SMALL_ANGLE_THRESHOLD = 0.001f; // ~0.057 degrees
  if (fabs(orientation_delta_rad) < SMALL_ANGLE_THRESHOLD) {
    local_X_position = Sideways_delta;
    local_Y_position = Forward_delta;
  } else {
    // Pilons arc method for curved motion
    float sin_half_delta = sin(orientation_delta_rad / 2.0f);
    float arc_radius_multiplier = 2.0f * sin_half_delta;
    local_X_position = arc_radius_multiplier * ((Sideways_delta / orientation_delta_rad) + SidewaysTracker_center_distance); 
    local_Y_position = arc_radius_multiplier * ((Forward_delta / orientation_delta_rad) + ForwardTracker_center_distance);
  }

  // Convert local displacement to global coordinates
  float local_polar_angle;
  float local_polar_length;

  // Use squared length comparison to avoid unnecessary sqrt
  float local_length_squared = local_X_position * local_X_position + local_Y_position * local_Y_position;
  if (local_length_squared < 0.000001f) { // Effectively zero movement
    local_polar_angle = 0;
    local_polar_length = 0;
  } else {
    local_polar_angle = atan2(local_Y_position, local_X_position); 
    local_polar_length = sqrt(local_length_squared); // Only compute sqrt when needed
  }

  float global_polar_angle = local_polar_angle - prev_orientation_rad - (orientation_delta_rad / 2.0f);

  float X_position_delta = local_polar_length * cos(global_polar_angle); 
  float Y_position_delta = local_polar_length * sin(global_polar_angle);

  // Outlier rejection: check for unreasonably large jumps (> 12 inches in one update)
  const float MAX_POSITION_JUMP = 12.0f;
  float jump_magnitude_squared = X_position_delta * X_position_delta + Y_position_delta * Y_position_delta;
  if (jump_magnitude_squared > MAX_POSITION_JUMP * MAX_POSITION_JUMP) {
    // Likely a sensor glitch, skip this update
    return;
  }

  // Raw (unsmoothed) position update
  X_position += X_position_delta;
  Y_position += Y_position_delta;

  // Calculate current velocity for smoothing
  float curr_velocity_x = X_position_delta / dt;
  float curr_velocity_y = Y_position_delta / dt;

  // Adaptive Kalman filter with velocity-aware process noise
  // Tunable parameters
  const float base_process_variance = 0.005f;        // Reduced for smoother tracking
  const float measurement_variance = 0.15f;          // Reduced for more responsive tracking
  const float velocity_process_factor = 0.002f;      // Scale process noise with velocity
  const float velocity_smoothing_alpha = 0.3f;       // Low-pass filter on velocity changes

  if (!kalman_initialized) {
    kalman_X_position = X_position;
    kalman_Y_position = Y_position;
    kalman_Px = 1.0f;
    kalman_Py = 1.0f;
    prev_velocity_x = curr_velocity_x;
    prev_velocity_y = curr_velocity_y;
    kalman_initialized = true;
  } else {
    // Smooth velocity changes to reduce jitter
    float smoothed_velocity_x = velocity_smoothing_alpha * curr_velocity_x + (1.0f - velocity_smoothing_alpha) * prev_velocity_x;
    float smoothed_velocity_y = velocity_smoothing_alpha * curr_velocity_y + (1.0f - velocity_smoothing_alpha) * prev_velocity_y;
    prev_velocity_x = smoothed_velocity_x;
    prev_velocity_y = smoothed_velocity_y;

    // Adaptive process variance based on velocity magnitude
    float velocity_magnitude = sqrt(smoothed_velocity_x * smoothed_velocity_x + smoothed_velocity_y * smoothed_velocity_y);
    float adaptive_process_variance = base_process_variance + velocity_process_factor * velocity_magnitude * dt;

    // Predict step with adaptive process noise
    kalman_Px += adaptive_process_variance;
    kalman_Py += adaptive_process_variance;

    // Update step with measurement = raw odom position
    float Kx = kalman_Px / (kalman_Px + measurement_variance);
    float Ky = kalman_Py / (kalman_Py + measurement_variance);

    kalman_X_position += Kx * (X_position - kalman_X_position);
    kalman_Y_position += Ky * (Y_position - kalman_Y_position);

    kalman_Px *= (1.0f - Kx);
    kalman_Py *= (1.0f - Ky);

    // Prevent covariance from becoming too small (maintain some adaptability)
    const float min_covariance = 0.01f;
    if (kalman_Px < min_covariance) kalman_Px = min_covariance;
    if (kalman_Py < min_covariance) kalman_Py = min_covariance;
  }

  X_position = kalman_X_position;
  Y_position = kalman_Y_position;
}
