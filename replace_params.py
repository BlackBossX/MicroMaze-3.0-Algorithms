import re

with open("src/following3.cpp", "r") as f:
    content = f.read()

target = """// ================= Odometry Variables =================
float ltm_s = 7.52; float ltm_f[3] = {7.52, 7.52, 7.52};
float rtm_s = 7.52; float rtm_f[3] = {7.52, 7.52, 7.52};
float tpd_s = 3.36; float tpd_f[3] = {3.36, 3.36, 3.36};
volatile long leftTicks = 0;
volatile long rightTicks = 0;
float lastCmdDist = 0, lastCmdAngle = 0;

// ================= Global Parameters =================
float Ki = 0.0;   
float EMA_ALPHA = 1.0; 
float SIDE_WALL_THRESHOLD = 155.0;     
float FRONT_WALL_THRESHOLD = 160.0;
float TARGET_45_DIST = 126.0;        
float FRONT_SLOW_DIST = 80.0, FRONT_CRASH_DIST = 45.0; 
float PID_DEADBAND = 2.0;  
float REVERSE_BRAKE_MS = 25.0; 

// ================= Phase & Speed Profiles =================
// [0] = Search Phase
float Kp_s = 0.25, Kd_s = 0.02, cs_s = 180.0, tm_s = 1.0, ec_s = 47.5;
float o9i_s = 58.0, o9o_s = 120.0, o18i_s = 50.0, o18o_s = 110.0;

// [0] = Fast 120, [1] = Fast 180, [2] = Fast 220
float Kp_f[3] =   {0.25, 0.2, 0.2};
float Kd_f[3] =   {0.05, 0.05, 0.05};
float cs_f[3] =   {192.0, 192.0, 192.0};
float tm_f[3] =   {1.0, 1.2, 1.4}; 
float ec_f[3] =   {47.5, 45.0, 41.0}; 
float o9i_f[3] =  {45.0, 58.0, 58.0};
float o9o_f[3] =  {110.0, 110.0, 110.0};
float o18i_f[3] = {50.0, 50.0, 50.0};
float o18o_f[3] = {110.0, 110.0, 110.0};"""

replacement = """// ================= Odometry Variables =================
float ltm_s = 7.66; float ltm_f[3] = {7.66, 7.52, 7.52};
float rtm_s = 7.66; float rtm_f[3] = {7.66, 7.52, 7.52};
float tpd_s = 4.17; float tpd_f[3] = {4.17, 3.36, 3.36};
volatile long leftTicks = 0;
volatile long rightTicks = 0;
float lastCmdDist = 0, lastCmdAngle = 0;

// ================= Global Parameters =================
float Ki = 0.0;   
float EMA_ALPHA = 1.0; 
float SIDE_WALL_THRESHOLD = 155.0;     
float FRONT_WALL_THRESHOLD = 165.0;
float TARGET_45_DIST = 126.0;        
float FRONT_SLOW_DIST = 80.0, FRONT_CRASH_DIST = 45.0; 
float PID_DEADBAND = 2.0;  
float REVERSE_BRAKE_MS = 25.0; 

// ================= Phase & Speed Profiles =================
// [0] = Search Phase
float Kp_s = 0.25, Kd_s = 0.02, cs_s = 180.0, tm_s = 1.0, ec_s = 47.5;
float o9i_s = 50.0, o9o_s = 100.0, o18i_s = 50.0, o18o_s = 110.0;

// [0] = Fast 120, [1] = Fast 180, [2] = Fast 220
float Kp_f[3] =   {0.25, 0.2, 0.2};
float Kd_f[3] =   {0.05, 0.05, 0.05};
float cs_f[3] =   {192.0, 192.0, 192.0};
float tm_f[3] =   {1.0, 1.2, 1.4}; 
float ec_f[3] =   {47.5, 45.0, 41.0}; 
float o9i_f[3] =  {45.0, 58.0, 58.0};
float o9o_f[3] =  {110.0, 110.0, 110.0};
float o18i_f[3] = {50.0, 50.0, 50.0};
float o18o_f[3] = {110.0, 110.0, 110.0};"""

if target in content:
    content = content.replace(target, replacement)
    with open("src/following3.cpp", "w") as f:
        f.write(content)
    print("Updated successfully")
else:
    print("Error: Target not found")

