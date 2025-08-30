#include "main.h"  // Assuming this contains your pin definitions and other global includes

#include <Arduino.h>
#include <Servo.h>

#define CONFIGURE 1  // Set to 1 to enable homing routine in setup
#define TEST_RELOADING 0

// ======================= Pins ===========================
const int enPin_x = 2;
const int dirPin_x = 3;
const int stepPin_x = 4;

const int enPin_y = 5;
const int dirPin_y = 6;
const int stepPin_y = 7;

Servo gun_enable_servo; /* Pin 8 */
Servo gun_shoot_servo;  /* Pin 9 */

const int limitSW_back = 10;   // Limit switch for backward/downward Y movement
const int limitSW_front = 11;  // Limit switch for forward/upward Y movement

// ======================= Motion =========================
const int STEP_DELAY_US = 100;                                       // Base delay for one step
const int STEPS_PER_REV = 3200;                                      // 1/16 Microstep resolution for now
const unsigned long STEP_INTERVAL_MICROSECONDS = STEP_DELAY_US * 4;  // Constant speed for all movements

// ==================== Gun Mechanism =====================
bool gunReloaded = false;
bool gunEnabled = false;  // Global variable for gun enable/disable state
unsigned long gunLastShot = 0;
const unsigned long GUN_RELOAD_TIME_MS = 400UL;

// ================= Stepper Control Variables ============
// X-axis (Left/Right)
volatile long targetStepsX = 0;
volatile long currentStepsX = 0;
volatile unsigned long lastStepTimeX = 0;

// Y-axis (Up/Down)
volatile long targetStepsY = 0;
volatile long currentStepsY = 0;
volatile unsigned long lastStepTimeY = 0;

// =================== Serial Protocol Definitions ========
const byte START_BYTE = 0xFF;
const byte END_BYTE = 0xFE;

const byte CMD_MOVE_X = 0x01;
const byte CMD_MOVE_Y = 0x02;
const byte CMD_SHOOT_GUN = 0x03;
const byte CMD_ENABLE_GUN = 0x04;
const byte CMD_DISABLE_GUN = 0x05;

// =================== Serial Receive Buffer ==============
// Simple state machine for robust packet reception
enum ReceiveState {
    WAITING_FOR_START,
    WAITING_FOR_CMD_TYPE,
    WAITING_FOR_DATA_MSB,
    WAITING_FOR_DATA_LSB,
    WAITING_FOR_END
};

ReceiveState currentReceiveState = WAITING_FOR_START;
byte currentCommandType = 0;
int16_t receivedStepValue = 0;  // For 16-bit signed step count

// ========================================================

void gun_shoot();
void handleByteCommand(byte commandType, int16_t value);
void processIncomingSerialByte(int incomingByte);
// Updated executeStepperStep signature: removed minLimit, maxLimit, and axis parameters
void executeStepperStep(int pin_step, int pin_dir, int pin_limit_back, int pin_limit_front,
                        volatile long* currentSteps, volatile long* targetSteps,
                        volatile unsigned long* lastStepTime);

void setup()
{
    Serial.begin(115200);
    Serial.println("STARTING\n");

    pinMode(dirPin_x, OUTPUT);
    pinMode(stepPin_x, OUTPUT);
    pinMode(enPin_x, OUTPUT);

    pinMode(dirPin_y, OUTPUT);
    pinMode(stepPin_y, OUTPUT);
    pinMode(enPin_y, OUTPUT);

    // Limit switches are typically wired as Normally Closed (NC) and pull-up resistor,
    // so they read LOW when pressed, HIGH when not pressed.
    pinMode(limitSW_back, INPUT_PULLUP);
    pinMode(limitSW_front, INPUT_PULLUP);

    gun_enable_servo.attach(8);
    gun_shoot_servo.attach(9);

    gun_enable_servo.write(30);  // GUN OFF (initial state)
    gunEnabled = false;

    gun_shoot_servo.write(180);  // GUN RELOADED
    gunReloaded = true;

    digitalWrite(enPin_x, LOW);  // LOW = enabled (steppers are enabled by default)
    digitalWrite(enPin_y, LOW);

#if CONFIGURE == 1
    // Initial configuration: Home the Y-axis

    long maxYLimitSteps = 0;

    // 1. Move Y-axis to the back limit switch (downward/backward direction)
    // This defines our '0' or 'min' position.
    digitalWrite(dirPin_y, HIGH);               // Assuming HIGH for 'back/down'
    while (digitalRead(limitSW_back) != LOW) {  // While switch is NOT pressed (HIGH)
        digitalWrite(stepPin_y, HIGH);
        delayMicroseconds(STEP_INTERVAL_MICROSECONDS);
        digitalWrite(stepPin_y, LOW);
        delayMicroseconds(STEP_INTERVAL_MICROSECONDS);
    }
    delay(100);  // Debounce delay

    // 2. Move Y-axis to the front limit switch (upward/forward direction)
    // This defines our 'max' position and total travel range.
    digitalWrite(dirPin_y, LOW);                 // Assuming LOW for 'front/up'
    currentStepsY = 0;                           // Set current position to the minimum
    while (digitalRead(limitSW_front) != LOW) {  // While switch is NOT pressed (HIGH)
        digitalWrite(stepPin_y, HIGH);
        delayMicroseconds(STEP_INTERVAL_MICROSECONDS);
        digitalWrite(stepPin_y, LOW);
        delayMicroseconds(STEP_INTERVAL_MICROSECONDS);
        currentStepsY++;  // Increment current position while moving forward
    }
    delay(100);                      // Debounce delay
    maxYLimitSteps = currentStepsY;  // This is our absolute maximum position

    // 3. Move to approximately 50% of the range (center) for starting position
    long homeOffsetSteps = (maxYLimitSteps) / 2;

    // We want currentStepsY to move towards homeOffsetSteps
    targetStepsY = homeOffsetSteps;  // Set the target

    Serial.print("Moving Y-axis to center (");
    Serial.print(homeOffsetSteps);
    Serial.println(" steps)...");
    while (currentStepsY != targetStepsY) {
        // Determine direction
        if (currentStepsY < targetStepsY) {
            digitalWrite(dirPin_y, LOW);  // Move front/up
            currentStepsY++;
        }
        else {
            digitalWrite(dirPin_y, HIGH);  // Move back/down
            currentStepsY--;
        }
        digitalWrite(stepPin_y, HIGH);
        delayMicroseconds(STEP_INTERVAL_MICROSECONDS);
        digitalWrite(stepPin_y, LOW);
        delayMicroseconds(STEP_INTERVAL_MICROSECONDS);
    }

    gun_enable_servo.write(15);  // GUN ON
    delay(200);
    gun_enable_servo.write(30);  // GUN OFF
    delay(200);
#endif
#if TEST_RELOADING == 1
    // Perform two initial (blocking) test shots during setup so we see movement
    gun_shoot_servo.write(30);  // GUN SHOOT
    delay(400);
    gun_shoot_servo.write(180);  // GUN NOT SHOOT -- RELOAD
    delay(400);
    gun_shoot_servo.write(30);  // GUN SHOOT
    delay(400);
    gun_shoot_servo.write(180);  // GUN NOT SHOOT -- RELOAD
    gunReloaded = true;
#endif

    Serial.println("READY NOW");
}

void loop()
{
    // Process incoming serial data byte by byte
    while (Serial.available()) {
        processIncomingSerialByte(Serial.read());
    }

    // Handle non-blocking gun reload
    if (!gunReloaded) {
        const unsigned long now = millis();
        if ((now - gunLastShot) >= GUN_RELOAD_TIME_MS) {
            gun_shoot_servo.write(180);  // reload
            gunReloaded = true;
        }
    }

    // Handle non-blocking stepper movement for X-axis (no limits implemented for X-axis in this version)
    // Updated executeStepperStep call: removed minLimit, maxLimit, and axis arguments
    executeStepperStep(stepPin_x, dirPin_x, -1, -1,  // Placeholder for X-axis limits (not used here)
                       &currentStepsX, &targetStepsX, &lastStepTimeX);

    // Handle non-blocking stepper movement for Y-axis with HARDWARE limit checks
    // Updated executeStepperStep call: removed minLimit, maxLimit, and axis arguments
    executeStepperStep(stepPin_y, dirPin_y, limitSW_back, limitSW_front,
                       &currentStepsY, &targetStepsY, &lastStepTimeY);
}

// Function to execute one step if needed, non-blocking
// Now includes HARDWARE limit switch checks for axes that need them (like Y-axis)
void executeStepperStep(int pin_step, int pin_dir, int pin_limit_back, int pin_limit_front,
                        volatile long* currentSteps, volatile long* targetSteps,
                        volatile unsigned long* lastStepTime)
{
    if (*currentSteps != *targetSteps) {
        unsigned long now = micros();  // Use micros() for finer timing for steppers
        if ((now - *lastStepTime) >= STEP_INTERVAL_MICROSECONDS) {
            *lastStepTime = now;

            // Determine planned direction for the next step
            bool movingPositive = (*currentSteps < *targetSteps);

            // --- Apply Hardware Limit Checks (for Y-axis using specific pin_limit values) ---
            // We assume pin_limit_back and pin_limit_front are valid pin numbers for Y-axis,
            // and -1 for X-axis where limits are not implemented yet.
            if (pin_limit_back != -1 && pin_limit_front != -1) {  // This means it's an axis with limits (Y-axis)
                if (movingPositive && digitalRead(pin_limit_front) == LOW) {
                    // Moving positive (up/front) but front limit switch is active
                    *targetSteps = *currentSteps;  // Stop at current position
                    return;                        // Do not take a step
                }
                else if (!movingPositive && digitalRead(pin_limit_back) == LOW) {
                    // Moving negative (down/back) but back limit switch is active
                    *targetSteps = *currentSteps;  // Stop at current position
                    return;                        // Do not take a step
                }
            }
            // --- End Hardware Limit Checks ---

            // Set direction and increment/decrement step count
            // Note: Direction is HIGH for "back/down" and LOW for "front/up" in this setup
            if (movingPositive) {            // If target is greater than current (moving "up" or "left")
                digitalWrite(pin_dir, LOW);  // Move in positive direction (e.g., Left for X, Up for Y)
                (*currentSteps)++;
            }
            else {                            // If target is less than current (moving "down" or "right")
                digitalWrite(pin_dir, HIGH);  // Move in negative direction (e.g., Right for X, Down for Y)
                (*currentSteps)--;
            }

            digitalWrite(pin_step, HIGH);
            delayMicroseconds(1);  // Short pulse
            digitalWrite(pin_step, LOW);
        }
    }
}

void gun_shoot()
{
    // Non-blocking shoot: only fire if currently reloaded AND gun is enabled.
    if (!(gunReloaded && gunEnabled)) return;

    gun_shoot_servo.write(30);  // GUN SHOOT position
    gunLastShot = millis();
    gunReloaded = false;
}

// Handles a complete command once parsed from serial
void handleByteCommand(byte commandType, int16_t value)
{
    switch (commandType) {
        case CMD_MOVE_X:
            // Set the target steps for X-axis movement as ABSOLUTE.
            // Treat current position as zero now, and move to `value` pulses.
            currentStepsX = 0;
            targetStepsX = value;
            break;
        case CMD_MOVE_Y:
            // Set the target steps for Y-axis movement as ABSOLUTE.
            // Treat current position as zero now, and move to `value` pulses.
            currentStepsY = 0;
            targetStepsY = value;
            break;
        case CMD_SHOOT_GUN:
            gun_shoot();
            break;
        case CMD_ENABLE_GUN:
            gun_enable_servo.write(15);  // GUN ON position
            gunEnabled = true;           // Update gunEnabled state
            break;
        case CMD_DISABLE_GUN:
            gun_enable_servo.write(30);  // GUN OFF position
            gunEnabled = false;          // Update gunEnabled state
            break;
        default:
            break;
    }
}

// State machine to parse incoming serial bytes
void processIncomingSerialByte(int incomingByte)
{
    static byte msb = 0;
    static byte lsb = 0;

    switch (currentReceiveState) {
        case WAITING_FOR_START:
            if (incomingByte == START_BYTE) {
                currentReceiveState = WAITING_FOR_CMD_TYPE;
            }
            break;
        case WAITING_FOR_CMD_TYPE:
            currentCommandType = incomingByte;
            if (currentCommandType == CMD_MOVE_X || currentCommandType == CMD_MOVE_Y) {
                currentReceiveState = WAITING_FOR_DATA_MSB;
            }
            else {
                receivedStepValue = 0;                  // Commands without data bytes don't have a step value
                currentReceiveState = WAITING_FOR_END;  // Commands without data bytes
            }
            break;
        case WAITING_FOR_DATA_MSB:
            msb = incomingByte;
            currentReceiveState = WAITING_FOR_DATA_LSB;
            break;
        case WAITING_FOR_DATA_LSB:
            lsb = incomingByte;
            receivedStepValue = (msb << 8) | lsb;  // Reconstruct 16-bit signed integer
            currentReceiveState = WAITING_FOR_END;
            break;
        case WAITING_FOR_END:
            if (incomingByte == END_BYTE) {
                handleByteCommand(currentCommandType, receivedStepValue);
            }
            // Reset for next command
            currentReceiveState = WAITING_FOR_START;
            currentCommandType = 0;
            receivedStepValue = 0;
            break;
    }
}
