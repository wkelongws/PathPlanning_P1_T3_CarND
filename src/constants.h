#ifndef CONSTANTS
#define CONSTANTS

#define VEHICLE_RADIUS 1.25              // meters
#define FOLLOW_DISTANCE 10.0              // distance to keep behind leading cars

#define PREVIOUS_PATH_POINTS_TO_KEEP 20
#define NUM_PATH_POINTS 50
#define PATH_DT 0.02                    // seconds

#define TRACK_LENGTH 6945.554           // meters

// number of waypoints to use for interpolation
#define NUM_WAYPOINTS_BEHIND 5
#define NUM_WAYPOINTS_AHEAD 5

// for trajectory generation/evaluation and non-ego car predictions
#define N_SAMPLES 15
#define DT 0.1           				// seconds
#define PLAN_AHEAD_DURATION 1.5           // seconds

#define SPEED_LIMIT 21.3                // m/s
#define VELOCITY_INCREMENT_LIMIT 0.125

// cost function weights
#define COLLISION_COST_WEIGHT 99999
#define IN_LANE_BUFFER_COST_WEIGHT 10
#define EFFICIENCY_COST_WEIGHT 100
#define NOT_MIDDLE_LANE_COST_WEIGHT 1

// Other
#define MAX_INSTANTANEOUS_JERK 10       // m/s/s/s
#define MAX_INSTANTANEOUS_ACCEL 10      // m/s/s

#define PERCENT_V_DIFF_TO_MAKE_UP 0.5   // the percent difference between current velocity and target velocity to allow ego car to make up in a single trajectory  

#endif