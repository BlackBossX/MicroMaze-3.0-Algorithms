#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// ================= Pin Definitions =================
const int PWMA = 19, AIN2 = 5, AIN1 = 4;
const int PWMB = 18, BIN2 = 15, BIN1 = 2;
const int ENC_R_C1 = 32, ENC_R_C2 = 33; 
const int ENC_L_C1 = 34, ENC_L_C2 = 13;
const int XSHUT_RF = 14, XSHUT_LF = 25;
const int XSHUT_R45 = 27, XSHUT_L45 = 26;

// ================= ToF Sensor Setup =================
#define ADDR_RF  0x30
#define ADDR_LF  0x31
#define ADDR_R45 0x32
#define ADDR_L45 0x29

Adafruit_VL53L0X sensorRF = Adafruit_VL53L0X();
Adafruit_VL53L0X sensorLF = Adafruit_VL53L0X();
Adafruit_VL53L0X sensorR45 = Adafruit_VL53L0X();
Adafruit_VL53L0X sensorL45 = Adafruit_VL53L0X();

// ================= Calibration =================
const float SCALE_LF = 1.0;  const float OFFSET_LF = 0.0;
const float SCALE_RF = 1.0;  float OFFSET_RF = -20.0; 
const float SCALE_L45 = 1.0; const float OFFSET_L45 = 0.0;
const float SCALE_R45 = 1.0; const float OFFSET_R45 = 0.0;
const float MAX_TOF_DIST = 2000.0; 

float distLF = MAX_TOF_DIST, distRF = MAX_TOF_DIST;
float distL45 = MAX_TOF_DIST, distR45 = MAX_TOF_DIST;

// ================= Odometry Variables =================
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
float o18o_f[3] = {110.0, 110.0, 110.0};

int currentPhase = 0; // 0 = Search, 1 = Fast Run
int fastSpeedIdx = 0; // 0, 1, or 2

// ================= Drive & State Variables =================
enum DriveState { IDLE, DRIVE_DIST, TURN_ANGLE, WALL_FOLLOW, REVERSE_BRAKING };
volatile DriveState currentState = IDLE;

long targetLeft = 0, targetRight = 0;
int basePwm = 150, driveDir = 1, turnDir = 1;
unsigned long reverseBrakeStartTime = 0;

float prevErrorL = 0, prevErrorR = 0;
float prevDerivL = 0, prevDerivR = 0;
unsigned long lastPidTime = 0;

WebServer server(80);

// ================= Flood Fill Map Variables =================
#define MAP_WIDTH 16
#define MAP_HEIGHT 16
unsigned char walls[MAP_WIDTH][MAP_HEIGHT];
int distances[MAP_WIDTH][MAP_HEIGHT];
bool visited[MAP_WIDTH][MAP_HEIGHT];
int m_x = 0, m_y = 0, m_dir = 0; 
volatile bool waitingForFastRun = false;
volatile bool skipSearch = false; 

typedef struct { int x; int y; } Point;
Point queue[MAP_WIDTH * MAP_HEIGHT];
int q_head = 0, q_tail = 0;

TaskHandle_t mazeTaskHandle = NULL;
TaskHandle_t driveTaskHandle = NULL;

// ================= 4x Quadrature Encoder ISRs =================
void IRAM_ATTR leftEncoderC1ISR() { if (digitalRead(ENC_L_C1) == digitalRead(ENC_L_C2)) leftTicks++; else leftTicks--; }
void IRAM_ATTR leftEncoderC2ISR() { if (digitalRead(ENC_L_C1) != digitalRead(ENC_L_C2)) leftTicks++; else leftTicks--; }
void IRAM_ATTR rightEncoderC1ISR() { if (digitalRead(ENC_R_C1) == digitalRead(ENC_R_C2)) rightTicks++; else rightTicks--; }
void IRAM_ATTR rightEncoderC2ISR() { if (digitalRead(ENC_R_C1) != digitalRead(ENC_R_C2)) rightTicks++; else rightTicks--; }

// ================= Motors & Sensors =================
void controlLeftMotor(int speed, int dir) {
  if (dir == 1) { digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); analogWrite(PWMB, speed); } 
  else if (dir == -1) { digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH); analogWrite(PWMB, speed); } 
  else { digitalWrite(BIN1, HIGH); digitalWrite(BIN2, HIGH); analogWrite(PWMB, 255); }
}

void controlRightMotor(int speed, int dir) {
  if (dir == 1) { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); analogWrite(PWMA, speed); } 
  else if (dir == -1) { digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH); analogWrite(PWMA, speed); } 
  else { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, HIGH); analogWrite(PWMA, 255); }
}

void initToFSensors() {
  pinMode(XSHUT_RF, OUTPUT); pinMode(XSHUT_LF, OUTPUT); pinMode(XSHUT_R45, OUTPUT); pinMode(XSHUT_L45, OUTPUT);
  digitalWrite(XSHUT_RF, LOW); digitalWrite(XSHUT_LF, LOW); digitalWrite(XSHUT_R45, LOW); digitalWrite(XSHUT_L45, LOW); delay(10);
  digitalWrite(XSHUT_RF, HIGH); digitalWrite(XSHUT_LF, HIGH); digitalWrite(XSHUT_R45, HIGH); digitalWrite(XSHUT_L45, HIGH); delay(10);
  digitalWrite(XSHUT_LF, LOW); digitalWrite(XSHUT_R45, LOW); digitalWrite(XSHUT_L45, LOW); delay(10); sensorRF.begin(ADDR_RF);
  digitalWrite(XSHUT_LF, HIGH); delay(10); sensorLF.begin(ADDR_LF);
  digitalWrite(XSHUT_R45, HIGH); delay(10); sensorR45.begin(ADDR_R45);
  digitalWrite(XSHUT_L45, HIGH); delay(10); sensorL45.begin(ADDR_L45);
}

void updateSensor(Adafruit_VL53L0X &sensor, float scale, float offset, float &filtered) {
  VL53L0X_RangingMeasurementData_t measure;
  sensor.rangingTest(&measure, false);
  if (measure.RangeStatus != 4 && measure.RangeMilliMeter > 0 && measure.RangeMilliMeter < 2000) {
    float cal = (measure.RangeMilliMeter * scale) + offset;
    filtered = (filtered == MAX_TOF_DIST) ? cal : (EMA_ALPHA * cal) + ((1.0 - EMA_ALPHA) * filtered);
  } else { filtered = MAX_TOF_DIST; }
}

void executeWallFollowingPID(int activePwm) {
  unsigned long currentTime = millis();
  float dt = (currentTime - lastPidTime) / 1000.0;
  if (dt <= 0.005) return; 
  lastPidTime = currentTime;

  bool hasLeft = distL45 < SIDE_WALL_THRESHOLD;
  bool hasRight = distR45 < SIDE_WALL_THRESHOLD;

  float active_Kp = (currentPhase == 0) ? Kp_s : Kp_f[fastSpeedIdx];
  float active_Kd = (currentPhase == 0) ? Kd_s : Kd_f[fastSpeedIdx];

  float correctionL = 0, correctionR = 0;

  if (hasLeft) {
    float errorL = TARGET_45_DIST - distL45; 
    if (abs(errorL) < PID_DEADBAND) errorL = 0;
    errorL = constrain(errorL, -25.0, 25.0);
    float derivL = (0.5 * ((errorL - prevErrorL) / dt)) + (0.5 * prevDerivL);
    prevDerivL = derivL; prevErrorL = errorL;
    correctionL = (active_Kp * errorL) + (active_Kd * derivL);
  } else { prevErrorL = 0; prevDerivL = 0; }

  if (hasRight) {
    float errorR = TARGET_45_DIST - distR45; 
    if (abs(errorR) < PID_DEADBAND) errorR = 0;
    errorR = constrain(errorR, -25.0, 25.0);
    float derivR = (0.5 * ((errorR - prevErrorR) / dt)) + (0.5 * prevDerivR);
    prevDerivR = derivR; prevErrorR = errorR;
    correctionR = (active_Kp * errorR) + (active_Kd * derivR);
  } else { prevErrorR = 0; prevDerivR = 0; }

  float netSteer = constrain(correctionL - correctionR, -(activePwm * 0.8), (activePwm * 0.8));

  controlLeftMotor(constrain(activePwm + netSteer, 0, 255), 1);
  controlRightMotor(constrain(activePwm - netSteer, 0, 255), 1);
}

// ================= CORE 0: Real-Time Drive Task =================
void driveControlTask(void * pvParameters) {
  for(;;) {
    static int tofState = 0;
    switch(tofState) {
      case 0: updateSensor(sensorLF, SCALE_LF, OFFSET_LF, distLF); break;
      case 1: updateSensor(sensorRF, SCALE_RF, OFFSET_RF, distRF); break;
      case 2: updateSensor(sensorL45, SCALE_L45, OFFSET_L45, distL45); break;
      case 3: updateSensor(sensorR45, SCALE_R45, OFFSET_R45, distR45); break;
    }
    tofState = (tofState + 1) % 4;
    
    if (currentState == WALL_FOLLOW) {
      float minFront = min(distLF, distRF);
      bool leftDone = (abs(leftTicks) >= targetLeft);
      bool rightDone = (abs(rightTicks) >= targetRight);

      if (minFront < FRONT_CRASH_DIST) {
        if (REVERSE_BRAKE_MS > 0) {
          currentState = REVERSE_BRAKING; reverseBrakeStartTime = millis();
          controlLeftMotor(255, -1); controlRightMotor(255, -1);
        } else { currentState = IDLE; controlLeftMotor(0, 0); controlRightMotor(0, 0); }
      } 
      else if (leftDone && rightDone) {
        currentState = IDLE; controlLeftMotor(0, 0); controlRightMotor(0, 0);
      }
      else {
        int dynamicPwm = basePwm;
        if (minFront < FRONT_SLOW_DIST) {
          float speedFactor = constrain((minFront - FRONT_CRASH_DIST) / (FRONT_SLOW_DIST - FRONT_CRASH_DIST), 0.0, 1.0);
          dynamicPwm = 80 + (basePwm - 80) * speedFactor;
        }
        executeWallFollowingPID(dynamicPwm); 
      }
    }
    else if (currentState == REVERSE_BRAKING) {
      if (millis() - reverseBrakeStartTime >= REVERSE_BRAKE_MS) {
        currentState = IDLE; controlLeftMotor(0, 0); controlRightMotor(0, 0); 
      }
    }
    else if (currentState == DRIVE_DIST) {
      bool leftDone = (abs(leftTicks) >= targetLeft);
      bool rightDone = (abs(rightTicks) >= targetRight);
      if (leftDone && rightDone) { currentState = IDLE; controlLeftMotor(0, 0); controlRightMotor(0, 0); } 
      else {
        if (leftDone) controlLeftMotor(0, 0); else controlLeftMotor(basePwm, driveDir);
        if (rightDone) controlRightMotor(0, 0); else controlRightMotor(basePwm, driveDir);
      }
    }
    else if (currentState == TURN_ANGLE) {
      bool leftDone = (abs(leftTicks) >= targetLeft);
      bool rightDone = (abs(rightTicks) >= targetRight);
      
      if (leftDone && rightDone) { 
        currentState = IDLE; controlLeftMotor(0, 0); controlRightMotor(0, 0); 
      } 
      else {
        int activeTurnPwm = basePwm;
        if (currentPhase == 1 && targetLeft > 0) {
          float progress = (float)abs(leftTicks) / targetLeft;
          int minTurnPwm = 80; 
          float rampFactor = 1.0;

          if (progress < 0.25) { 
            rampFactor = progress / 0.25; 
          } else if (progress > 0.75) { 
            rampFactor = (1.0 - progress) / 0.25; 
          }
          
          rampFactor = constrain(rampFactor, 0.0, 1.0);
          activeTurnPwm = minTurnPwm + (basePwm - minTurnPwm) * rampFactor;
        }

        if (leftDone) controlLeftMotor(0, 0); else controlLeftMotor(activeTurnPwm, turnDir);
        if (rightDone) controlRightMotor(0, 0); else controlRightMotor(activeTurnPwm, -turnDir);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1)); 
  }
}

// ================= Robot Motion Wrappers =================
void waitForMove() {
  while (currentState != IDLE) { vTaskDelay(pdMS_TO_TICKS(5)); }
  vTaskDelay(pdMS_TO_TICKS(50)); 
}

void r_turn(float angle) {
  turnDir = (angle > 0) ? 1 : -1;
  float base_tpd = (currentPhase == 0) ? tpd_s : tpd_f[fastSpeedIdx];
  float active_tm = (currentPhase == 0) ? tm_s : tm_f[fastSpeedIdx];
  float active_tpd = base_tpd * active_tm;
  
  targetLeft = abs(angle * active_tpd);
  targetRight = abs(angle * active_tpd);
  leftTicks = 0; rightTicks = 0;
  currentState = TURN_ANGLE;
  waitForMove();
}

void r_move(float dist, bool usePID) {
  if (dist <= 0) return;
  driveDir = (dist >= 0) ? 1 : -1;
  float active_ltm = (currentPhase == 0) ? ltm_s : ltm_f[fastSpeedIdx];
  float active_rtm = (currentPhase == 0) ? rtm_s : rtm_f[fastSpeedIdx];
  targetLeft = abs(dist * active_ltm);
  targetRight = abs(dist * active_rtm);
  leftTicks = 0; rightTicks = 0;
  if (usePID && dist > 0) {
    prevErrorL = 0; prevErrorR = 0; prevDerivL = 0; prevDerivR = 0; lastPidTime = millis();
    currentState = WALL_FOLLOW;
  } else {
    currentState = DRIVE_DIST;
  }
  waitForMove();
}

// ================= Flood Fill & Logic =================
bool is_center_pos(int cx, int cy) {
  return ((cx == 7 && cy == 7) || (cx == 7 && cy == 8) || (cx == 8 && cy == 7) || (cx == 8 && cy == 8));
}
bool in_center() { return is_center_pos(m_x, m_y); }

void init_maze() {
  for (int i = 0; i < MAP_WIDTH; i++) {
    for (int j = 0; j < MAP_HEIGHT; j++) {
      walls[i][j] = 0; visited[i][j] = false;
      if (i == 0) walls[i][j] |= 8;
      if (i == MAP_WIDTH - 1) walls[i][j] |= 2;
      if (j == 0) walls[i][j] |= 4;
      if (j == MAP_HEIGHT - 1) walls[i][j] |= 1;
    }
  }
}

bool update_walls() {
  if (visited[m_x][m_y]) { return false; }
  vTaskDelay(pdMS_TO_TICKS(50)); 

  visited[m_x][m_y] = true;
  bool wall_front = (min(distLF, distRF) < FRONT_WALL_THRESHOLD);
  bool wall_right = (distR45 < SIDE_WALL_THRESHOLD);
  bool wall_left  = (distL45 < SIDE_WALL_THRESHOLD);

  if (m_dir == 0) {
    if (wall_front) { walls[m_x][m_y] |= 1; if (m_y < 15) walls[m_x][m_y+1] |= 4; }
    if (wall_right) { walls[m_x][m_y] |= 2; if (m_x < 15) walls[m_x+1][m_y] |= 8; }
    if (wall_left)  { walls[m_x][m_y] |= 8; if (m_x > 0)  walls[m_x-1][m_y] |= 2; }
  } else if (m_dir == 1) {
    if (wall_front) { walls[m_x][m_y] |= 2; if (m_x < 15) walls[m_x+1][m_y] |= 8; }
    if (wall_right) { walls[m_x][m_y] |= 4; if (m_y > 0)  walls[m_x][m_y-1] |= 1; }
    if (wall_left)  { walls[m_x][m_y] |= 1; if (m_y < 15) walls[m_x][m_y+1] |= 4; }
  } else if (m_dir == 2) {
    if (wall_front) { walls[m_x][m_y] |= 4; if (m_y > 0)  walls[m_x][m_y-1] |= 1; }
    if (wall_right) { walls[m_x][m_y] |= 8; if (m_x > 0)  walls[m_x-1][m_y] |= 2; }
    if (wall_left)  { walls[m_x][m_y] |= 2; if (m_x < 15) walls[m_x+1][m_y] |= 8; }
  } else if (m_dir == 3) {
    if (wall_front) { walls[m_x][m_y] |= 8; if (m_x > 0)  walls[m_x-1][m_y] |= 2; }
    if (wall_right) { walls[m_x][m_y] |= 1; if (m_y < 15) walls[m_x][m_y+1] |= 4; }
    if (wall_left)  { walls[m_x][m_y] |= 4; if (m_y > 0)  walls[m_x][m_y-1] |= 1; }
  }
  return true; 
}

void flood_fill(int target_mode, bool strict_mode) {
  for (int i = 0; i < MAP_WIDTH; i++) {
    for (int j = 0; j < MAP_HEIGHT; j++) { distances[i][j] = 9999; }
  }
  q_head = 0; q_tail = 0;

  if (target_mode == 0) {
    distances[7][7] = 0; queue[q_tail++] = (Point){7, 7};
    distances[7][8] = 0; queue[q_tail++] = (Point){7, 8};
    distances[8][7] = 0; queue[q_tail++] = (Point){8, 7};
    distances[8][8] = 0; queue[q_tail++] = (Point){8, 8};
  } else {
    distances[0][0] = 0; queue[q_tail++] = (Point){0, 0};
  }

  while (q_head < q_tail) {
    Point p = queue[q_head++];
    int curr_dist = distances[p.x][p.y];

    if (!(walls[p.x][p.y] & 1) && p.y < 15 && distances[p.x][p.y + 1] == 9999) {
      if (!strict_mode || visited[p.x][p.y + 1]) { distances[p.x][p.y + 1] = curr_dist + 1; queue[q_tail++] = (Point){p.x, p.y + 1}; }
    }
    if (!(walls[p.x][p.y] & 2) && p.x < 15 && distances[p.x + 1][p.y] == 9999) {
      if (!strict_mode || visited[p.x + 1][p.y]) { distances[p.x + 1][p.y] = curr_dist + 1; queue[q_tail++] = (Point){p.x + 1, p.y}; }
    }
    if (!(walls[p.x][p.y] & 4) && p.y > 0 && distances[p.x][p.y - 1] == 9999) {
      if (!strict_mode || visited[p.x][p.y - 1]) { distances[p.x][p.y - 1] = curr_dist + 1; queue[q_tail++] = (Point){p.x, p.y - 1}; }
    }
    if (!(walls[p.x][p.y] & 8) && p.x > 0 && distances[p.x - 1][p.y] == 9999) {
      if (!strict_mode || visited[p.x - 1][p.y]) { distances[p.x - 1][p.y] = curr_dist + 1; queue[q_tail++] = (Point){p.x - 1, p.y}; }
    }
  }
}

int get_next_dir(int cx, int cy, int cdir) {
  int d_n = (!(walls[cx][cy] & 1) && cy < 15) ? distances[cx][cy + 1] : 9999;
  int d_e = (!(walls[cx][cy] & 2) && cx < 15) ? distances[cx + 1][cy] : 9999;
  int d_s = (!(walls[cx][cy] & 4) && cy > 0)  ? distances[cx][cy - 1] : 9999;
  int d_w = (!(walls[cx][cy] & 8) && cx > 0)  ? distances[cx - 1][cy] : 9999;

  int options[4] = {d_n, d_e, d_s, d_w};
  int min_d = options[cdir];
  int best_dir = cdir;

  for (int i = 1; i <= 3; i++) {
    int test_dir = (cdir + i) % 4;
    if (options[test_dir] < min_d) { min_d = options[test_dir]; best_dir = test_dir; }
  }
  return best_dir;
}

void move_to_optimal_neighbor() {
  int best_dir = get_next_dir(m_x, m_y, m_dir);
  int turn = (best_dir - m_dir + 4) % 4;
  float active_cs = (currentPhase == 0) ? cs_s : cs_f[fastSpeedIdx];
  
  if (turn == 0) {
    r_move(active_cs, true); 
  } else {
    float in_offset, out_offset;
    if (currentPhase == 0) {
      in_offset  = (turn == 2) ? o18i_s : o9i_s;
      out_offset = (turn == 2) ? o18o_s : o9o_s;
    } else {
      in_offset  = (turn == 2) ? o18i_f[fastSpeedIdx] : o9i_f[fastSpeedIdx];
      out_offset = (turn == 2) ? o18o_f[fastSpeedIdx] : o9o_f[fastSpeedIdx];
    }

    r_move(in_offset, false); 
    if (turn == 1) { r_turn(90); }
    else if (turn == 2) { r_turn(90); r_turn(90); }
    else if (turn == 3) { r_turn(-90); }
    r_move(out_offset, true); 
  }
  
  m_dir = best_dir;
  if (m_dir == 0) m_y++; else if (m_dir == 1) m_x++; else if (m_dir == 2) m_y--; else if (m_dir == 3) m_x--;
}

// ================= CORE 1: Maze Solving Task =================
void mazeTask(void * pvParameters) {
  if (!skipSearch) {
    init_maze();
    
    // Phase 1: Search In
    while (1) {
      if (update_walls()) { flood_fill(0, false); }
      if (in_center()) break;
      move_to_optimal_neighbor();
    }
    
    flood_fill(1, false);
    
    // Phase 2: Search Out
    while (1) {
      if (update_walls()) { flood_fill(1, false); }
      if (m_x == 0 && m_y == 0) break;
      move_to_optimal_neighbor();
    }

    r_move(ec_s, false); 
  }

  // Phase 3: Fast Run Loop
  while (1) {
    waitingForFastRun = true;
    while(waitingForFastRun) { vTaskDelay(pdMS_TO_TICKS(100)); } 

    m_x = 0; m_y = 0; m_dir = 0; 
    float active_cs = cs_f[fastSpeedIdx];

    flood_fill(0, true);
    
    while (1) {
      if (in_center()) break;

      int best_dir = get_next_dir(m_x, m_y, m_dir);
      int turn = (best_dir - m_dir + 4) % 4;

      if (turn == 0) {
        int straight_cells = 0;
        int px = m_x, py = m_y;
        while (!is_center_pos(px, py)) {
          int next_d = get_next_dir(px, py, m_dir);
          if (next_d != m_dir) break;
          straight_cells++;
          if (m_dir == 0) py++; else if (m_dir == 1) px++; else if (m_dir == 2) py--; else if (m_dir == 3) px--;
        }
        r_move(straight_cells * active_cs, true); 
        m_x = px; m_y = py;
      } 
      else {
        float in_offset  = (turn == 2) ? o18i_f[fastSpeedIdx] : o9i_f[fastSpeedIdx];
        float out_offset = (turn == 2) ? o18o_f[fastSpeedIdx] : o9o_f[fastSpeedIdx];

        r_move(in_offset, false); 
        if (turn == 1) { r_turn(90); }
        else if (turn == 2) { r_turn(90); r_turn(90); }
        else if (turn == 3) { r_turn(-90); }
        m_dir = best_dir;

        int straight_cells = 0;
        int px = m_x, py = m_y;
        
        if (m_dir == 0) py++; else if (m_dir == 1) px++; else if (m_dir == 2) py--; else if (m_dir == 3) px--;
        
        while (!is_center_pos(px, py)) {
          int next_d = get_next_dir(px, py, m_dir);
          if (next_d != m_dir) break;
          straight_cells++;
          if (m_dir == 0) py++; else if (m_dir == 1) px++; else if (m_dir == 2) py--; else if (m_dir == 3) px--;
        }
        
        r_move(out_offset + (straight_cells * active_cs), true); 
        m_x = px; m_y = py;
      }
    }
    
    r_move(ec_f[fastSpeedIdx], false);

    currentState = IDLE;
    controlLeftMotor(0, 0); controlRightMotor(0, 0);
  }
}

// ================= Web Server HTML =================
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Micromouse Dashboard</title>
  <style>
    body { font-family: Arial; margin: 10px; background: #121212; color: #ffffff; font-size: 14px;}
    .card { background: #1e1e1e; padding: 15px; border-radius: 8px; margin-bottom: 15px; }
    input[type=range], input[type=number] { width: 100%; box-sizing: border-box; margin-bottom: 8px; }
    input[type=number], button { padding: 6px; font-size: 13px; background: #333; color: white; border: none; border-radius: 4px; }
    button { cursor: pointer; width: 100%; font-weight: bold; border-radius: 6px; margin-bottom: 5px;}
    h3 { margin-top: 0; color: #00cc66; }
    .sensor-map { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; text-align: center; margin-bottom: 5px;}
    .sensor-box { background: #2a2a2a; padding: 10px; border-radius: 6px; border-top: 3px solid #00cc66; }
    .val { font-size: 20px; font-weight: bold; margin-top: 5px; }
    .wall-ind-grid { display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 5px; text-align: center; margin-bottom: 10px;}
    .wall-ind { padding: 8px; background: #333; font-weight: bold; border-radius: 4px; color: #555; }
    .wall-ind.active { background: #dc3545; color: white; }
    .grid-2 { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; }
    .grid-4 { display: grid; grid-template-columns: repeat(4, 1fr); gap: 5px; }
    label { display: block; font-weight: bold; font-size: 11px; margin-bottom: 2px; color: #aaa;}
    table { width: 100%; border-collapse: collapse; margin-bottom: 10px; font-size: 12px; }
    th, td { border: 1px solid #444; padding: 4px; text-align: center; }
    td input { margin: 0; padding: 4px; font-size: 12px; }
    .action-row { display: flex; gap: 5px; align-items: center; }
    .action-row input, .action-row button { flex: 1; margin: 0; padding: 10px;}
    canvas { background: #000; border: 2px solid #333; border-radius: 4px; display: block; margin: 0 auto; }
  </style>
</head>
<body onload="fetchParams()">
  <h2>🤖 Dual-Core Dashboard</h2>
  
  <div class="card">
    <h3>Sensors & Odometry</h3>
    <div class="sensor-map">
      <div class="sensor-box">L-45&deg;<div class="val" id="tl45">--</div></div>
      <div class="sensor-box">R-45&deg;<div class="val" id="tr45">--</div></div>
      <div class="sensor-box">L-Front<div class="val" id="tlf">--</div></div>
      <div class="sensor-box">R-Front<div class="val" id="trf">--</div></div>
    </div>
    <div class="wall-ind-grid">
      <div id="w_l" class="wall-ind">LEFT WALL</div>
      <div id="w_f" class="wall-ind">FRONT WALL</div>
      <div id="w_r" class="wall-ind">RIGHT WALL</div>
    </div>
    <div class="grid-2">
      <div>L: <span id="lt">0</span> (<b id="lm">0.0</b> mm)</div>
      <div>R: <span id="rt">0</span> (<b id="rm">0.0</b> mm)</div>
    </div>
  </div>
  
  <div class="card">
    <h3>Speed & Offset Profiles Matrix</h3>
    <table>
      <tr><th>Param</th><th>Search</th><th>F-120</th><th>F-180</th><th>F-220</th></tr>
      <tr><td>Kp</td><td><input type="number" id="kps" step="0.05"></td><td><input type="number" id="kp0" step="0.05"></td><td><input type="number" id="kp1" step="0.05"></td><td><input type="number" id="kp2" step="0.05"></td></tr>
      <tr><td>Kd</td><td><input type="number" id="kds" step="0.1"></td><td><input type="number" id="kd0" step="0.1"></td><td><input type="number" id="kd1" step="0.1"></td><td><input type="number" id="kd2" step="0.1"></td></tr>
      <tr><td>Cell Size</td><td><input type="number" id="css" step="0.5"></td><td><input type="number" id="cs0" step="0.5"></td><td><input type="number" id="cs1" step="0.5"></td><td><input type="number" id="cs2" step="0.5"></td></tr>
      <tr><td>Turn Mult</td><td><input type="number" id="tms" step="0.01"></td><td><input type="number" id="tm0" step="0.01"></td><td><input type="number" id="tm1" step="0.01"></td><td><input type="number" id="tm2" step="0.01"></td></tr>
      <tr><td>90-IN</td><td><input type="number" id="o9is" step="0.5"></td><td><input type="number" id="o9i0" step="0.5"></td><td><input type="number" id="o9i1" step="0.5"></td><td><input type="number" id="o9i2" step="0.5"></td></tr>
      <tr><td>90-OUT</td><td><input type="number" id="o9os" step="0.5"></td><td><input type="number" id="o9o0" step="0.5"></td><td><input type="number" id="o9o1" step="0.5"></td><td><input type="number" id="o9o2" step="0.5"></td></tr>
      <tr><td>180-IN</td><td><input type="number" id="o18is" step="0.5"></td><td><input type="number" id="o18i0" step="0.5"></td><td><input type="number" id="o18i1" step="0.5"></td><td><input type="number" id="o18i2" step="0.5"></td></tr>
      <tr><td>180-OUT</td><td><input type="number" id="o18os" step="0.5"></td><td><input type="number" id="o18o0" step="0.5"></td><td><input type="number" id="o18o1" step="0.5"></td><td><input type="number" id="o18o2" step="0.5"></td></tr>
      <tr><td>End Center</td><td><input type="number" id="ecs" step="0.5"></td><td><input type="number" id="ec0" step="0.5"></td><td><input type="number" id="ec1" step="0.5"></td><td><input type="number" id="ec2" step="0.5"></td></tr>
      <tr><td>L Tck/mm</td><td><input type="number" id="ltms" step="0.01"></td><td><input type="number" id="ltm0" step="0.01"></td><td><input type="number" id="ltm1" step="0.01"></td><td><input type="number" id="ltm2" step="0.01"></td></tr>
      <tr><td>R Tck/mm</td><td><input type="number" id="rtms" step="0.01"></td><td><input type="number" id="rtm0" step="0.01"></td><td><input type="number" id="rtm1" step="0.01"></td><td><input type="number" id="rtm2" step="0.01"></td></tr>
      <tr><td>Tck/Deg</td><td><input type="number" id="tpds" step="0.01"></td><td><input type="number" id="tpd0" step="0.01"></td><td><input type="number" id="tpd1" step="0.01"></td><td><input type="number" id="tpd2" step="0.01"></td></tr>
    </table>
    
    <h3 style="margin-top:15px;">Global Variables</h3>
    <div class="grid-4">
      <div><label>Ki:</label><input type="number" id="p_ki" step="0.01"></div>
      <div><label>Target 45:</label><input type="number" id="p_t45"></div>
      <div><label>Side Wall Th:</label><input type="number" id="p_swt"></div>
      <div><label>Front Wall Th:</label><input type="number" id="p_fwt"></div>
      <div><label>Slow Dist:</label><input type="number" id="p_fsd"></div>
      <div><label>Rev Brake:</label><input type="number" id="p_rb"></div>
      <div><label>Deadband:</label><input type="number" id="p_db" step="0.5"></div>
    </div>
    <button onclick="updateParams()" style="background: #6f42c1; padding: 10px; margin-top:10px; border:none; color:white;">SAVE ALL PARAMETERS</button>
  </div>

  <div class="card">
    <h3>Live Graphical Maze Map</h3>
    <canvas id="mazeCanvas" width="320" height="320"></canvas>
  </div>

  <div class="card">
    <h3>Autonomous Control</h3>
    <div style="display:flex; gap:10px;">
      <button onclick="cmd('solve_maze')" style="background: #e83e8c; padding: 15px; font-size:16px; color:white; border:none;">▶ SEARCH MAZE</button>
      <button onclick="cmd('stop')" style="background: #dc3545; padding: 15px; font-size:16px; color:white; border:none;">EMERGENCY STOP</button>
    </div>
    
    <h3 style="margin-top: 20px; font-size:13px; color:#aaa;">Fast Run Speeds (Run multiple times)</h3>
    <div style="display:flex; gap:10px;">
      <button onclick="cmd('fast_run', 120)" style="background: #28a745; padding: 12px; border:none; color:white;">FAST 120</button>
      <button onclick="cmd('fast_run', 180)" style="background: #ffc107; color:#000; padding: 12px; border:none;">FAST 180</button>
      <button onclick="cmd('fast_run', 220)" style="background: #dc3545; padding: 12px; border:none; color:white;">FAST 220</button>
    </div>
  </div>
  
  <div class="card">
    <h3>Manual Calibration</h3>
    <label>Base Speed (PWM): <span id="ap_val">150</span></label>
    <input type="range" id="ap" min="0" max="255" value="150" oninput="document.getElementById('ap_val').innerText=this.value">
    <div class="action-row" style="margin-bottom:10px;">
      <input type="number" id="dist" placeholder="Cmd (mm)">
      <button onclick="cmd('dist')" style="background:#555; color:white; border:none;">Drive</button>
      <input type="number" id="actual_dist" placeholder="Act (mm)">
      <button onclick="calibrate('dist')" style="background:#17a2b8; color:white; border:none;">Calc & Save</button>
    </div>
    <div class="action-row" style="margin-bottom:10px;">
      <input type="number" id="angle" placeholder="Cmd (&deg;)">
      <button onclick="cmd('turn')" style="background:#555; color:white; border:none;">Turn</button>
      <input type="number" id="actual_angle" placeholder="Act (&deg;)">
      <button onclick="calibrate('turn')" style="background:#17a2b8; color:white; border:none;">Calc & Save</button>
    </div>
  </div>

  <script>
    let swt = 160, fwt = 150; 
    function formatDist(val) { return val >= 2000 ? "CLEAR" : val.toFixed(1); }
    
    function fetchParams() {
      fetch('/get_params').then(r => r.json()).then(d => {
        ['kps','kp0','kp1','kp2','kds','kd0','kd1','kd2','css','cs0','cs1','cs2','tms','tm0','tm1','tm2','o9is','o9i0','o9i1','o9i2','o9os','o9o0','o9o1','o9o2','o18is','o18i0','o18i1','o18i2','o18os','o18o0','o18o1','o18o2','ecs','ec0','ec1','ec2','ltms','ltm0','ltm1','ltm2','rtms','rtm0','rtm1','rtm2','tpds','tpd0','tpd1','tpd2'].forEach(k => { document.getElementById(k).value = d[k]; });
        ['ki','t45','swt','fwt','rb','fsd','db'].forEach(k => { document.getElementById('p_' + k).value = d[k]; });
        swt = d.swt; fwt = d.fwt; 
      });
    }

    function updateParams() {
      let ids = ['kps','kp0','kp1','kp2','kds','kd0','kd1','kd2','css','cs0','cs1','cs2','tms','tm0','tm1','tm2','o9is','o9i0','o9i1','o9i2','o9os','o9o0','o9o1','o9o2','o18is','o18i0','o18i1','o18i2','o18os','o18o0','o18o1','o18o2','ecs','ec0','ec1','ec2','ltms','ltm0','ltm1','ltm2','rtms','rtm0','rtm1','rtm2','tpds','tpd0','tpd1','tpd2'].map(k => `${k}=${document.getElementById(k).value}`);
      let globals = ['ki','t45','swt','fwt','rb','fsd','db'].map(k => `${k}=${document.getElementById('p_'+k).value}`);
      fetch(`/set_params?${ids.join('&')}&${globals.join('&')}`).then(() => { alert("Saved!"); fetchParams(); });
    }

    function cmd(action, directVal=0) {
      let pwm = document.getElementById('ap').value;
      let val = directVal;
      if (action === 'dist') val = document.getElementById('dist').value;
      if (action === 'turn') val = document.getElementById('angle').value;
      fetch(`/cmd?action=${action}&val=${val}&pwm=${pwm}`);
    }

    function calibrate(type) {
      let actual = document.getElementById(type === 'dist' ? 'actual_dist' : 'actual_angle').value;
      if (actual) { fetch(`/cal_${type}?actual=${actual}`).then(r => r.text()).then(res => { alert(res); }); }
    }

    function drawMaze(x, y, d, w) {
      const canvas = document.getElementById('mazeCanvas');
      const ctx = canvas.getContext('2d');
      ctx.clearRect(0, 0, 320, 320);
      const cs = 20; 
      
      ctx.strokeStyle = '#00cc66'; ctx.lineWidth = 2;
      for(let j=0; j<16; j++) {
        for(let i=0; i<16; i++) {
          let cx = i * cs;
          let cy = 320 - (j * cs) - cs; 
          let wall = w[j*16 + i];
          if(wall & 1) { ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(cx+cs, cy); ctx.stroke(); } 
          if(wall & 2) { ctx.beginPath(); ctx.moveTo(cx+cs, cy); ctx.lineTo(cx+cs, cy+cs); ctx.stroke(); } 
          if(wall & 4) { ctx.beginPath(); ctx.moveTo(cx, cy+cs); ctx.lineTo(cx+cs, cy+cs); ctx.stroke(); } 
          if(wall & 8) { ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(cx, cy+cs); ctx.stroke(); } 
        }
      }
      
      ctx.fillStyle = 'red'; ctx.strokeStyle = 'white'; ctx.lineWidth = 2;
      let rx = x * cs + cs/2; let ry = 320 - (y * cs) - cs/2;
      ctx.beginPath(); ctx.arc(rx, ry, 5, 0, 2*Math.PI); ctx.fill();
      ctx.beginPath(); ctx.moveTo(rx, ry);
      if(d==0) ctx.lineTo(rx, ry-10); 
      if(d==1) ctx.lineTo(rx+10, ry); 
      if(d==2) ctx.lineTo(rx, ry+10); 
      if(d==3) ctx.lineTo(rx-10, ry); 
      ctx.stroke();
    }

    setInterval(() => {
      fetch('/data').then(r => r.json()).then(d => {
        document.getElementById('lt').innerText = d.lt; document.getElementById('lm').innerText = d.lm.toFixed(1);
        document.getElementById('rt').innerText = d.rt; document.getElementById('rm').innerText = d.rm.toFixed(1);
        document.getElementById('tlf').innerText = formatDist(d.tlf); document.getElementById('trf').innerText = formatDist(d.trf);
        document.getElementById('tl45').innerText = formatDist(d.tl45); document.getElementById('tr45').innerText = formatDist(d.tr45);
        
        document.getElementById('w_l').className = (d.tl45 < swt) ? "wall-ind active" : "wall-ind";
        document.getElementById('w_r').className = (d.tr45 < swt) ? "wall-ind active" : "wall-ind";
        document.getElementById('w_f').className = (Math.min(d.tlf, d.trf) < fwt) ? "wall-ind active" : "wall-ind";
      });
      fetch('/maze_data').then(r => r.json()).then(d => { drawMaze(d.x, d.y, d.d, d.w); });
    }, 300);
  </script>
</body>
</html>
)rawliteral";

// ================= Setup =================
void setup() {
  Serial.begin(115200); Wire.begin(); Wire.setClock(400000); 

  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(ENC_L_C1, INPUT_PULLUP); pinMode(ENC_L_C2, INPUT_PULLUP);
  pinMode(ENC_R_C1, INPUT_PULLUP); pinMode(ENC_R_C2, INPUT_PULLUP);
  controlLeftMotor(0, 0); controlRightMotor(0, 0);

  attachInterrupt(digitalPinToInterrupt(ENC_L_C1), leftEncoderC1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_L_C2), leftEncoderC2ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_C1), rightEncoderC1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_C2), rightEncoderC2ISR, CHANGE);

  initToFSensors();
  WiFi.softAP("Micromouse_Test", "12345678");

  server.on("/", []() { server.send(200, "text/html", index_html); });
  
  server.on("/data", []() {
    float active_ltm = (currentPhase == 0) ? ltm_s : ltm_f[fastSpeedIdx];
    float active_rtm = (currentPhase == 0) ? rtm_s : rtm_f[fastSpeedIdx];
    String json = "{";
    json += "\"lt\":" + String(leftTicks) + ",\"lm\":" + String(leftTicks / active_ltm) + ",";
    json += "\"rt\":" + String(rightTicks) + ",\"rm\":" + String(rightTicks / active_rtm) + ",";
    json += "\"tlf\":" + String(distLF) + ",\"trf\":" + String(distRF) + ",\"tl45\":" + String(distL45) + ",\"tr45\":" + String(distR45) + "}";
    server.send(200, "application/json", json);
  });

  server.on("/maze_data", []() {
    String json; json.reserve(2048); 
    json = "{\"x\":" + String(m_x) + ",\"y\":" + String(m_y) + ",\"d\":" + String(m_dir) + ",\"w\":[";
    for(int j=0; j<16; j++) {
      for(int i=0; i<16; i++) {
        json += String(walls[i][j]);
        if(i!=15 || j!=15) json += ",";
      }
    }
    json += "]}";
    server.send(200, "application/json", json);
  });
  
  server.on("/get_params", []() {
    String json; json.reserve(2048);
    json = "{\"kps\":" + String(Kp_s) + ",\"kp0\":" + String(Kp_f[0]) + ",\"kp1\":" + String(Kp_f[1]) + ",\"kp2\":" + String(Kp_f[2]) + ",";
    json += "\"kds\":" + String(Kd_s) + ",\"kd0\":" + String(Kd_f[0]) + ",\"kd1\":" + String(Kd_f[1]) + ",\"kd2\":" + String(Kd_f[2]) + ",";
    json += "\"css\":" + String(cs_s) + ",\"cs0\":" + String(cs_f[0]) + ",\"cs1\":" + String(cs_f[1]) + ",\"cs2\":" + String(cs_f[2]) + ",";
    json += "\"tms\":" + String(tm_s) + ",\"tm0\":" + String(tm_f[0]) + ",\"tm1\":" + String(tm_f[1]) + ",\"tm2\":" + String(tm_f[2]) + ",";
    json += "\"o9is\":" + String(o9i_s) + ",\"o9i0\":" + String(o9i_f[0]) + ",\"o9i1\":" + String(o9i_f[1]) + ",\"o9i2\":" + String(o9i_f[2]) + ",";
    json += "\"o9os\":" + String(o9o_s) + ",\"o9o0\":" + String(o9o_f[0]) + ",\"o9o1\":" + String(o9o_f[1]) + ",\"o9o2\":" + String(o9o_f[2]) + ",";
    json += "\"o18is\":" + String(o18i_s) + ",\"o18i0\":" + String(o18i_f[0]) + ",\"o18i1\":" + String(o18i_f[1]) + ",\"o18i2\":" + String(o18i_f[2]) + ",";
    json += "\"o18os\":" + String(o18o_s) + ",\"o18o0\":" + String(o18o_f[0]) + ",\"o18o1\":" + String(o18o_f[1]) + ",\"o18o2\":" + String(o18o_f[2]) + ",";
    json += "\"ecs\":" + String(ec_s) + ",\"ec0\":" + String(ec_f[0]) + ",\"ec1\":" + String(ec_f[1]) + ",\"ec2\":" + String(ec_f[2]) + ",";
    json += "\"ltms\":" + String(ltm_s) + ",\"ltm0\":" + String(ltm_f[0]) + ",\"ltm1\":" + String(ltm_f[1]) + ",\"ltm2\":" + String(ltm_f[2]) + ",";
    json += "\"rtms\":" + String(rtm_s) + ",\"rtm0\":" + String(rtm_f[0]) + ",\"rtm1\":" + String(rtm_f[1]) + ",\"rtm2\":" + String(rtm_f[2]) + ",";
    json += "\"tpds\":" + String(tpd_s) + ",\"tpd0\":" + String(tpd_f[0]) + ",\"tpd1\":" + String(tpd_f[1]) + ",\"tpd2\":" + String(tpd_f[2]) + ",";
    json += "\"ki\":" + String(Ki) + ",";
    json += "\"t45\":" + String(TARGET_45_DIST) + ",\"swt\":" + String(SIDE_WALL_THRESHOLD) + ",\"fwt\":" + String(FRONT_WALL_THRESHOLD) + ",";
    json += "\"rb\":" + String(REVERSE_BRAKE_MS) + ",\"fsd\":" + String(FRONT_SLOW_DIST) + ",\"db\":" + String(PID_DEADBAND) + "}";
    server.send(200, "application/json", json);
  });

  server.on("/set_params", []() {
    if (server.hasArg("kps")) Kp_s = server.arg("kps").toFloat();
    if (server.hasArg("kp0")) Kp_f[0] = server.arg("kp0").toFloat();
    if (server.hasArg("kp1")) Kp_f[1] = server.arg("kp1").toFloat();
    if (server.hasArg("kp2")) Kp_f[2] = server.arg("kp2").toFloat();
    
    if (server.hasArg("kds")) Kd_s = server.arg("kds").toFloat();
    if (server.hasArg("kd0")) Kd_f[0] = server.arg("kd0").toFloat();
    if (server.hasArg("kd1")) Kd_f[1] = server.arg("kd1").toFloat();
    if (server.hasArg("kd2")) Kd_f[2] = server.arg("kd2").toFloat();

    if (server.hasArg("css")) cs_s = server.arg("css").toFloat();
    if (server.hasArg("cs0")) cs_f[0] = server.arg("cs0").toFloat();
    if (server.hasArg("cs1")) cs_f[1] = server.arg("cs1").toFloat();
    if (server.hasArg("cs2")) cs_f[2] = server.arg("cs2").toFloat();

    if (server.hasArg("tms")) tm_s = server.arg("tms").toFloat();
    if (server.hasArg("tm0")) tm_f[0] = server.arg("tm0").toFloat();
    if (server.hasArg("tm1")) tm_f[1] = server.arg("tm1").toFloat();
    if (server.hasArg("tm2")) tm_f[2] = server.arg("tm2").toFloat();

    if (server.hasArg("o9is")) o9i_s = server.arg("o9is").toFloat();
    if (server.hasArg("o9i0")) o9i_f[0] = server.arg("o9i0").toFloat();
    if (server.hasArg("o9i1")) o9i_f[1] = server.arg("o9i1").toFloat();
    if (server.hasArg("o9i2")) o9i_f[2] = server.arg("o9i2").toFloat();

    if (server.hasArg("o9os")) o9o_s = server.arg("o9os").toFloat();
    if (server.hasArg("o9o0")) o9o_f[0] = server.arg("o9o0").toFloat();
    if (server.hasArg("o9o1")) o9o_f[1] = server.arg("o9o1").toFloat();
    if (server.hasArg("o9o2")) o9o_f[2] = server.arg("o9o2").toFloat();

    if (server.hasArg("o18is")) o18i_s = server.arg("o18is").toFloat();
    if (server.hasArg("o18i0")) o18i_f[0] = server.arg("o18i0").toFloat();
    if (server.hasArg("o18i1")) o18i_f[1] = server.arg("o18i1").toFloat();
    if (server.hasArg("o18i2")) o18i_f[2] = server.arg("o18i2").toFloat();

    if (server.hasArg("o18os")) o18o_s = server.arg("o18os").toFloat();
    if (server.hasArg("o18o0")) o18o_f[0] = server.arg("o18o0").toFloat();
    if (server.hasArg("o18o1")) o18o_f[1] = server.arg("o18o1").toFloat();
    if (server.hasArg("o18o2")) o18o_f[2] = server.arg("o18o2").toFloat();

    if (server.hasArg("ecs")) ec_s = server.arg("ecs").toFloat();
    if (server.hasArg("ec0")) ec_f[0] = server.arg("ec0").toFloat();
    if (server.hasArg("ec1")) ec_f[1] = server.arg("ec1").toFloat();
    if (server.hasArg("ec2")) ec_f[2] = server.arg("ec2").toFloat();

    if (server.hasArg("ltms")) ltm_s = server.arg("ltms").toFloat();
    if (server.hasArg("ltm0")) ltm_f[0] = server.arg("ltm0").toFloat();
    if (server.hasArg("ltm1")) ltm_f[1] = server.arg("ltm1").toFloat();
    if (server.hasArg("ltm2")) ltm_f[2] = server.arg("ltm2").toFloat();

    if (server.hasArg("rtms")) rtm_s = server.arg("rtms").toFloat();
    if (server.hasArg("rtm0")) rtm_f[0] = server.arg("rtm0").toFloat();
    if (server.hasArg("rtm1")) rtm_f[1] = server.arg("rtm1").toFloat();
    if (server.hasArg("rtm2")) rtm_f[2] = server.arg("rtm2").toFloat();

    if (server.hasArg("tpds")) tpd_s = server.arg("tpds").toFloat();
    if (server.hasArg("tpd0")) tpd_f[0] = server.arg("tpd0").toFloat();
    if (server.hasArg("tpd1")) tpd_f[1] = server.arg("tpd1").toFloat();
    if (server.hasArg("tpd2")) tpd_f[2] = server.arg("tpd2").toFloat();

    if (server.hasArg("ki")) Ki = server.arg("ki").toFloat();
    if (server.hasArg("t45")) TARGET_45_DIST = server.arg("t45").toFloat();
    if (server.hasArg("swt")) SIDE_WALL_THRESHOLD = server.arg("swt").toFloat();
    if (server.hasArg("fwt")) FRONT_WALL_THRESHOLD = server.arg("fwt").toFloat();
    if (server.hasArg("rb")) REVERSE_BRAKE_MS = server.arg("rb").toFloat();
    if (server.hasArg("fsd")) FRONT_SLOW_DIST = server.arg("fsd").toFloat();
    if (server.hasArg("db")) PID_DEADBAND = server.arg("db").toFloat();
    server.send(200, "text/plain", "OK");
  });

  server.on("/cmd", []() {
    String action = server.arg("action");
    float val = server.arg("val").toFloat();
    
    if (action == "stop") {
      waitingForFastRun = false;
      if(mazeTaskHandle != NULL) { vTaskDelete(mazeTaskHandle); mazeTaskHandle = NULL; }
      currentState = IDLE; controlLeftMotor(0, 0); controlRightMotor(0, 0);
    } 
    else if (action == "fast_run") {
      basePwm = (int)val;
      currentPhase = 1;
      if (basePwm == 120) fastSpeedIdx = 0;
      else if (basePwm == 180) fastSpeedIdx = 1;
      else if (basePwm == 220) fastSpeedIdx = 2;
      
      if (waitingForFastRun) {
        waitingForFastRun = false; 
      } 
      else if (mazeTaskHandle == NULL) {
        skipSearch = true;
        xTaskCreatePinnedToCore(mazeTask, "MazeSolver", 10000, NULL, 1, &mazeTaskHandle, 1);
      }
    }
    else if (action == "solve_maze") {
      basePwm = server.arg("pwm").toInt();
      currentPhase = 0; m_x = 0; m_y = 0; m_dir = 0; waitingForFastRun = false; skipSearch = false;
      if(mazeTaskHandle == NULL) {
        xTaskCreatePinnedToCore(mazeTask, "MazeSolver", 10000, NULL, 1, &mazeTaskHandle, 1);
      }
    }
    else if (action == "dist") { basePwm = server.arg("pwm").toInt(); lastCmdDist = abs(val); r_move(val, false); }
    else if (action == "turn") { basePwm = server.arg("pwm").toInt(); lastCmdAngle = abs(val); r_turn(val); }
    server.send(200, "text/plain", "OK");
  });

  server.on("/cal_dist", []() {
    float actual = abs(server.arg("actual").toFloat());
    if (actual > 0 && lastCmdDist > 0) {
      float factor = lastCmdDist / actual;
      ltm_s *= factor; rtm_s *= factor;
      for (int i = 0; i < 3; i++) { ltm_f[i] *= factor; rtm_f[i] *= factor; }
      server.send(200, "text/plain", "Distance parameters updated.");
    } else { server.send(200, "text/plain", "Error."); }
  });

  server.on("/cal_turn", []() {
    float actual = abs(server.arg("actual").toFloat());
    if (actual > 0 && lastCmdAngle > 0) {
      float factor = lastCmdAngle / actual;
      tpd_s *= factor;
      for (int i = 0; i < 3; i++) { tpd_f[i] *= factor; }
      server.send(200, "text/plain", "Turn parameter updated.");
    } else { server.send(200, "text/plain", "Error."); }
  });

  server.begin();

  xTaskCreatePinnedToCore(driveControlTask, "DriveControl", 8192, NULL, 2, &driveTaskHandle, 0);
}

// ================= CORE 1: Web Server Loop =================
void loop() {
  server.handleClient();
  vTaskDelay(pdMS_TO_TICKS(10)); 
}