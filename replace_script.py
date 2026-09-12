import re

with open("src/following2.cpp", "r") as f:
    content = f.read()

# Chunk 1
c1_target = """// ================= Odometry Variables =================
float LEFT_TICKS_PER_MM = 7.45;  
float RIGHT_TICKS_PER_MM = 7.46; 
volatile long leftTicks = 0;
volatile long rightTicks = 0;
float lastCmdDist = 0, lastCmdAngle = 0;

// ================= Global Parameters =================
float Ki = 0.0;   
float EMA_ALPHA = 1.0; 
float SIDE_WALL_THRESHOLD = 155.0;     
float FRONT_WALL_THRESHOLD = 160.0;
float TARGET_45_DIST = 126.0;        
float FRONT_SLOW_DIST = 150.0, FRONT_CRASH_DIST = 45.0; 
float PID_DEADBAND = 2.0;  
float REVERSE_BRAKE_MS = 25.0, TICKS_PER_DEGREE = 4.86; 

// ================= Phase & Speed Profiles =================
// [0] = Search Phase
float Kp_s = 0.25, Kd_s = 0.02, cs_s = 185.0, tm_s = 1.0, ec_s = 47.5;
float o9i_s = 58.0, o9o_s = 110.0, o18i_s = 50.0, o18o_s = 110.0;

// [0] = Fast 120, [1] = Fast 180, [2] = Fast 220
float Kp_f[3] =   {0.25, 0.2, 0.2};
float Kd_f[3] =   {0.05, 0.05, 0.05};
float cs_f[3] =   {192.0, 192.0, 192.0};
float tm_f[3] =   {1.0, 1.2, 1.4}; 
float ec_f[3] =   {47.5, 45.0, 41.0}; 
float o9i_f[3] =  {58.0, 58.0, 58.0};
float o9o_f[3] =  {110.0, 110.0, 110.0};
float o18i_f[3] = {50.0, 50.0, 50.0};
float o18o_f[3] = {110.0, 110.0, 110.0};"""

c1_replacement = """// ================= Odometry Variables =================
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

if c1_target in content: content = content.replace(c1_target, c1_replacement)
else: print("C1 FAIL")

# Chunk 2
c2_target = """void r_turn(float angle) {
  turnDir = (angle > 0) ? 1 : -1;
  float active_tpd = TICKS_PER_DEGREE * ((currentPhase == 1) ? tm_f[fastSpeedIdx] : tm_s);
  
  targetLeft = abs(angle * active_tpd);
  targetRight = abs(angle * active_tpd);
  leftTicks = 0; rightTicks = 0;
  currentState = TURN_ANGLE;
  waitForMove();
}

void r_move(float dist, bool usePID) {
  if (dist <= 0) return;
  driveDir = (dist >= 0) ? 1 : -1;
  targetLeft = abs(dist * LEFT_TICKS_PER_MM);
  targetRight = abs(dist * RIGHT_TICKS_PER_MM);
  leftTicks = 0; rightTicks = 0;"""

c2_replacement = """void r_turn(float angle) {
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
  leftTicks = 0; rightTicks = 0;"""

if c2_target in content: content = content.replace(c2_target, c2_replacement)
else: print("C2 FAIL")

# Chunk 3
c3_target = """      <tr><td>180-OUT</td><td><input type="number" id="o18os" step="0.5"></td><td><input type="number" id="o18o0" step="0.5"></td><td><input type="number" id="o18o1" step="0.5"></td><td><input type="number" id="o18o2" step="0.5"></td></tr>
      <tr><td>End Center</td><td><input type="number" id="ecs" step="0.5"></td><td><input type="number" id="ec0" step="0.5"></td><td><input type="number" id="ec1" step="0.5"></td><td><input type="number" id="ec2" step="0.5"></td></tr>
    </table>
    
    <h3 style="margin-top:15px;">Global Variables</h3>
    <div class="grid-4">
      <div><label>Ki:</label><input type="number" id="p_ki" step="0.01"></div>
      <div><label>L Tcks/mm:</label><input type="number" id="p_ltm" step="0.01"></div>
      <div><label>R Tcks/mm:</label><input type="number" id="p_rtm" step="0.01"></div>
      <div><label>Ticks/Deg:</label><input type="number" id="p_tpd" step="0.1"></div>
      <div><label>Target 45:</label><input type="number" id="p_t45"></div>
      <div><label>Side Wall Th:</label><input type="number" id="p_swt"></div>
      <div><label>Front Wall Th:</label><input type="number" id="p_fwt"></div>
      <div><label>Slow Dist:</label><input type="number" id="p_fsd"></div>
      <div><label>Rev Brake:</label><input type="number" id="p_rb"></div>
      <div><label>Deadband:</label><input type="number" id="p_db" step="0.5"></div>
    </div>"""

c3_replacement = """      <tr><td>180-OUT</td><td><input type="number" id="o18os" step="0.5"></td><td><input type="number" id="o18o0" step="0.5"></td><td><input type="number" id="o18o1" step="0.5"></td><td><input type="number" id="o18o2" step="0.5"></td></tr>
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
    </div>"""

if c3_target in content: content = content.replace(c3_target, c3_replacement)
else: print("C3 FAIL")

# Chunk 4
c4_target = """    function fetchParams() {
      fetch('/get_params').then(r => r.json()).then(d => {
        ['kps','kp0','kp1','kp2','kds','kd0','kd1','kd2','css','cs0','cs1','cs2','tms','tm0','tm1','tm2','o9is','o9i0','o9i1','o9i2','o9os','o9o0','o9o1','o9o2','o18is','o18i0','o18i1','o18i2','o18os','o18o0','o18o1','o18o2','ecs','ec0','ec1','ec2'].forEach(k => { document.getElementById(k).value = d[k]; });
        ['ki','ltm','rtm','tpd','t45','swt','fwt','rb','fsd','db'].forEach(k => { document.getElementById('p_' + k).value = d[k]; });
        swt = d.swt; fwt = d.fwt; 
      });
    }

    function updateParams() {
      let ids = ['kps','kp0','kp1','kp2','kds','kd0','kd1','kd2','css','cs0','cs1','cs2','tms','tm0','tm1','tm2','o9is','o9i0','o9i1','o9i2','o9os','o9o0','o9o1','o9o2','o18is','o18i0','o18i1','o18i2','o18os','o18o0','o18o1','o18o2','ecs','ec0','ec1','ec2'].map(k => `${k}=${document.getElementById(k).value}`);
      let globals = ['ki','ltm','rtm','tpd','t45','swt','fwt','rb','fsd','db'].map(k => `${k}=${document.getElementById('p_'+k).value}`);
      fetch(`/set_params?${ids.join('&')}&${globals.join('&')}`).then(() => { alert("Saved!"); fetchParams(); });
    }"""

c4_replacement = """    function fetchParams() {
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
    }"""

if c4_target in content: content = content.replace(c4_target, c4_replacement)
else: print("C4 FAIL")

# Chunk 5
c5_target = """  server.on("/data", []() {
    String json = "{";
    json += "\\"lt\\":" + String(leftTicks) + ",\\"lm\\":" + String(leftTicks / LEFT_TICKS_PER_MM) + ",";
    json += "\\"rt\\":" + String(rightTicks) + ",\\"rm\\":" + String(rightTicks / RIGHT_TICKS_PER_MM) + ",";
    json += "\\"tlf\\":" + String(distLF) + ",\\"trf\\":" + String(distRF) + ",\\"tl45\\":" + String(distL45) + ",\\"tr45\\":" + String(distR45) + "}";
    server.send(200, "application/json", json);
  });"""

c5_replacement = """  server.on("/data", []() {
    float active_ltm = (currentPhase == 0) ? ltm_s : ltm_f[fastSpeedIdx];
    float active_rtm = (currentPhase == 0) ? rtm_s : rtm_f[fastSpeedIdx];
    String json = "{";
    json += "\\"lt\\":" + String(leftTicks) + ",\\"lm\\":" + String(leftTicks / active_ltm) + ",";
    json += "\\"rt\\":" + String(rightTicks) + ",\\"rm\\":" + String(rightTicks / active_rtm) + ",";
    json += "\\"tlf\\":" + String(distLF) + ",\\"trf\\":" + String(distRF) + ",\\"tl45\\":" + String(distL45) + ",\\"tr45\\":" + String(distR45) + "}";
    server.send(200, "application/json", json);
  });"""

if c5_target in content: content = content.replace(c5_target, c5_replacement)
else: print("C5 FAIL")

# Chunk 6
c6_target = """    json += "\\"ecs\\":" + String(ec_s) + ",\\"ec0\\":" + String(ec_f[0]) + ",\\"ec1\\":" + String(ec_f[1]) + ",\\"ec2\\":" + String(ec_f[2]) + ",";
    json += "\\"ki\\":" + String(Ki) + ",\\"ltm\\":" + String(LEFT_TICKS_PER_MM) + ",\\"rtm\\":" + String(RIGHT_TICKS_PER_MM) + ",\\"tpd\\":" + String(TICKS_PER_DEGREE) + ",";
    json += "\\"t45\\":" + String(TARGET_45_DIST) + ",\\"swt\\":" + String(SIDE_WALL_THRESHOLD) + ",\\"fwt\\":" + String(FRONT_WALL_THRESHOLD) + ",";"""

c6_replacement = """    json += "\\"ecs\\":" + String(ec_s) + ",\\"ec0\\":" + String(ec_f[0]) + ",\\"ec1\\":" + String(ec_f[1]) + ",\\"ec2\\":" + String(ec_f[2]) + ",";
    json += "\\"ltms\\":" + String(ltm_s) + ",\\"ltm0\\":" + String(ltm_f[0]) + ",\\"ltm1\\":" + String(ltm_f[1]) + ",\\"ltm2\\":" + String(ltm_f[2]) + ",";
    json += "\\"rtms\\":" + String(rtm_s) + ",\\"rtm0\\":" + String(rtm_f[0]) + ",\\"rtm1\\":" + String(rtm_f[1]) + ",\\"rtm2\\":" + String(rtm_f[2]) + ",";
    json += "\\"tpds\\":" + String(tpd_s) + ",\\"tpd0\\":" + String(tpd_f[0]) + ",\\"tpd1\\":" + String(tpd_f[1]) + ",\\"tpd2\\":" + String(tpd_f[2]) + ",";
    json += "\\"ki\\":" + String(Ki) + ",";
    json += "\\"t45\\":" + String(TARGET_45_DIST) + ",\\"swt\\":" + String(SIDE_WALL_THRESHOLD) + ",\\"fwt\\":" + String(FRONT_WALL_THRESHOLD) + ",";"""

if c6_target in content: content = content.replace(c6_target, c6_replacement)
else: print("C6 FAIL")

# Chunk 7
c7_target = """    if (server.hasArg("ecs")) ec_s = server.arg("ecs").toFloat();
    if (server.hasArg("ec0")) ec_f[0] = server.arg("ec0").toFloat();
    if (server.hasArg("ec1")) ec_f[1] = server.arg("ec1").toFloat();
    if (server.hasArg("ec2")) ec_f[2] = server.arg("ec2").toFloat();

    if (server.hasArg("ki")) Ki = server.arg("ki").toFloat();
    if (server.hasArg("ltm")) LEFT_TICKS_PER_MM = server.arg("ltm").toFloat();
    if (server.hasArg("rtm")) RIGHT_TICKS_PER_MM = server.arg("rtm").toFloat();
    if (server.hasArg("tpd")) TICKS_PER_DEGREE = server.arg("tpd").toFloat();
    if (server.hasArg("t45")) TARGET_45_DIST = server.arg("t45").toFloat();"""

c7_replacement = """    if (server.hasArg("ecs")) ec_s = server.arg("ecs").toFloat();
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
    if (server.hasArg("t45")) TARGET_45_DIST = server.arg("t45").toFloat();"""

if c7_target in content: content = content.replace(c7_target, c7_replacement)
else: print("C7 FAIL")

# Chunk 8
c8_target = """  server.on("/cal_dist", []() {
    float actual = abs(server.arg("actual").toFloat());
    if (actual > 0 && lastCmdDist > 0) {
      float factor = lastCmdDist / actual;
      LEFT_TICKS_PER_MM *= factor; RIGHT_TICKS_PER_MM *= factor;
      server.send(200, "text/plain", "Distance parameters updated.");
    } else { server.send(200, "text/plain", "Error."); }
  });

  server.on("/cal_turn", []() {
    float actual = abs(server.arg("actual").toFloat());
    if (actual > 0 && lastCmdAngle > 0) {
      float factor = lastCmdAngle / actual;
      TICKS_PER_DEGREE *= factor;
      server.send(200, "text/plain", "Turn parameter updated.");
    } else { server.send(200, "text/plain", "Error."); }
  });"""

c8_replacement = """  server.on("/cal_dist", []() {
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
  });"""

if c8_target in content: content = content.replace(c8_target, c8_replacement)
else: print("C8 FAIL")

with open("src/following2.cpp", "w") as f:
    f.write(content)
print("Done!")
