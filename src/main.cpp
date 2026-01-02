#include <Arduino.h>
#include <Ps3Controller.h>

/* ================= DEBUG ================= */

/* ================= BTS7960 PIN DEFINITIONS ================= */

// LEFT SIDE (2 motors in parallel)
#define L_RPWM 25
#define L_LPWM 26
#define L_REN  14
#define L_LEN  27

// RIGHT SIDE (2 motors in parallel)
#define R_RPWM 13
#define R_LPWM 12
#define R_REN  32
#define R_LEN  33

/* ================= PWM SETTINGS ================= */
#define PWM_FREQ 20000
#define PWM_RES  8

#define L_RPWM_CH 0
#define L_LPWM_CH 1
#define R_RPWM_CH 2
#define R_LPWM_CH 3

#define DEADZONE 10

/* ================= SAFETY FLAGS ================= */
bool robotArmed = false;
bool killActive = false;

/* ---- Button edge detection ---- */
bool prevStart  = false;
bool prevSelect = false;

/* ================= FUNCTION PROTOTYPES ================= */
void stopMotors();
void driveBTS(int rpwm_ch, int lpwm_ch, int speed);

/* ================= PS3 CALLBACK ================= */
void onConnect() {
    static bool alreadyConnected = false;
    if (!alreadyConnected) {
        Serial.println("✅ PS3 CONTROLLER CONNECTED - READY TO ARM!");
        Serial.println("Press START button to ARM the robot");
        alreadyConnected = true;
    }
}

/* ================= SETUP ================= */
void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("Booting Robowar Controller...");
    Serial.println("Waiting for PS3 controller...");

    /* ---- Enable pins ---- */
    pinMode(L_REN, OUTPUT);
    pinMode(L_LEN, OUTPUT);
    pinMode(R_REN, OUTPUT);
    pinMode(R_LEN, OUTPUT);

    digitalWrite(L_REN, HIGH);
    digitalWrite(L_LEN, HIGH);
    digitalWrite(R_REN, HIGH);
    digitalWrite(R_LEN, HIGH);

    /* ---- PWM setup ---- */
    ledcSetup(L_RPWM_CH, PWM_FREQ, PWM_RES);
    ledcSetup(L_LPWM_CH, PWM_FREQ, PWM_RES);
    ledcSetup(R_RPWM_CH, PWM_FREQ, PWM_RES);
    ledcSetup(R_LPWM_CH, PWM_FREQ, PWM_RES);

    ledcAttachPin(L_RPWM, L_RPWM_CH);
    ledcAttachPin(L_LPWM, L_LPWM_CH);
    ledcAttachPin(R_RPWM, R_RPWM_CH);
    ledcAttachPin(R_LPWM, R_LPWM_CH);

    stopMotors();

    /* ---- PS3 INIT (NEW ESP32 MAC) ---- */
    Ps3.attach(onConnect);
    Ps3.begin("50:F1:CD:C2:DD:BC");   // YOUR NEW ESP32 BASE MAC

    Serial.println("Ps3.begin() called with ESP32 MAC");
    Serial.println("🎮 Waiting for PS3 Controller connection...");
    Serial.println("Use SELECT + START to control:");
    Serial.println("  START: Arm/Disarm robot");
    Serial.println("  SELECT: Kill switch (emergency stop)");
    Serial.println("  Left stick (LX): Turn left/right");
    Serial.println("  Right stick (RY): Forward/Backward");
    Serial.print("  Deadzone: "); Serial.println(DEADZONE);
    
    delay(2000);
    Serial.println("\n=== ROBOWAR SYSTEM READY ===");
    Serial.println("Waiting for PS3 Controller...");
    Serial.println("Once connected:");
    Serial.println("  1. Press START to ARM robot");
    Serial.println("  2. Move joysticks to control");
    Serial.println("  3. Press SELECT to KILL (emergency stop)");
    Serial.println("===========================\n");
}
/* ================= MOTOR CONTROL ================= */

void driveBTS(int rpwm_ch, int lpwm_ch, int speed) {
    int pwm = map(abs(speed), 0, 127, 0, 255);

    if (speed > 0) {
        ledcWrite(rpwm_ch, pwm);
        ledcWrite(lpwm_ch, 0);
    }
    else if (speed < 0) {
        ledcWrite(rpwm_ch, 0);
        ledcWrite(lpwm_ch, pwm);
    }
    else {
        ledcWrite(rpwm_ch, 0);
        ledcWrite(lpwm_ch, 0);
    }
}

void stopMotors() {
    ledcWrite(L_RPWM_CH, 0);
    ledcWrite(L_LPWM_CH, 0);
    ledcWrite(R_RPWM_CH, 0);
    ledcWrite(R_LPWM_CH, 0);
}

/* ================= LOOP ================= */
void loop() {

    /* ---- CONNECTION STATUS ---- */
    static bool lastConn = false;
    bool nowConn = Ps3.isConnected();

    if (nowConn != lastConn) {
        Serial.println(nowConn ? "🔵 PS3 CONNECTED" : "🔴 PS3 DISCONNECTED");
        lastConn = nowConn;
        Serial.print("Robot Status - ARMED: "); Serial.print(robotArmed);
        Serial.print(" | KILL: "); Serial.println(killActive);
    }

    if (!Ps3.isConnected()) {
        stopMotors();
        Serial.println("Waiting for controller...");
        delay(500);
        return;
    }

    /* ---- READ BUTTON STATES ---- */
    bool startNow  = Ps3.data.button.start;
    bool selectNow = Ps3.data.button.select;

    /* ---- KILL SWITCH (EDGE DETECT) ---- */
    if (selectNow && !prevSelect) {
        Serial.println("❌ KILL SWITCH PRESSED (SELECT)");
        killActive = true;
        robotArmed = false;
        stopMotors();
        Serial.println("  Motors stopped - Kill active");
    }

    /* ---- ARM / DISARM (EDGE DETECT) ---- */
    if (startNow && !prevStart) {

        if (killActive) {
            killActive = false;
            Serial.println("⚠️ KILL RESET — ROBOT DISARMED");
        } else {
            robotArmed = !robotArmed;
            Serial.println(robotArmed ? "🟢 ROBOT ARMED" : "🟡 ROBOT DISARMED");
        }

        stopMotors();
    }
    
    /* ---- LOG BUTTON PRESSES ---- */
    static uint32_t btnDbgT = 0;
    if (millis() - btnDbgT > 1000) {
        if (Ps3.data.button.cross) Serial.println("  [BUTTON] Cross pressed");
        if (Ps3.data.button.circle) Serial.println("  [BUTTON] Circle pressed");
        if (Ps3.data.button.square) Serial.println("  [BUTTON] Square pressed");
        if (Ps3.data.button.triangle) Serial.println("  [BUTTON] Triangle pressed");
        if (Ps3.data.button.l1) Serial.println("  [BUTTON] L1 pressed");
        if (Ps3.data.button.r1) Serial.println("  [BUTTON] R1 pressed");
        btnDbgT = millis();
    }

    /* ---- UPDATE PREVIOUS BUTTON STATES ---- */
    prevStart  = startNow;
    prevSelect = selectNow;

    /* ---- BLOCK MOVEMENT ---- */
    if (!robotArmed || killActive) {
        stopMotors();
        return;
    }

    /* ---- JOYSTICK INPUT ---- */
    int forward = Ps3.data.analog.stick.ry;  // Forward / Backward
    int turn    = Ps3.data.analog.stick.lx;  // Left / Right

    if (abs(forward) < DEADZONE) forward = 0;
    if (abs(turn) < DEADZONE)    turn = 0;

    int leftSpeed  = constrain(forward + turn,  -127, 127);
    int rightSpeed = constrain(forward - turn, -127, 127);

    /* ---- DEBUG VALUES ---- */
    static uint32_t dbgT = 0;
    if (millis() - dbgT > 400) {
        Serial.println("=== JOYSTICK & MOTOR DEBUG ===");
        Serial.print("  Raw LX (turn): "); Serial.print(Ps3.data.analog.stick.lx);
        Serial.print("  | Raw RY (forward): "); Serial.println(Ps3.data.analog.stick.ry);
        
        Serial.print("  After Deadzone - Forward: "); Serial.print(forward);
        Serial.print("  | Turn: "); Serial.println(turn);
        
        Serial.print("  Calculated Motor Speeds - Left: "); Serial.print(leftSpeed);
        Serial.print("  | Right: "); Serial.println(rightSpeed);
        
        Serial.print("  Status - ARMED: "); Serial.print(robotArmed);
        Serial.print("  | KILL: "); Serial.print(killActive);
        Serial.print("  | Connected: "); Serial.println(Ps3.isConnected());
        Serial.println("=============================");
        dbgT = millis();
    }

    /* ---- DRIVE MOTORS ---- */
    driveBTS(L_RPWM_CH, L_LPWM_CH, leftSpeed);
    driveBTS(R_RPWM_CH, R_LPWM_CH, rightSpeed);
}

