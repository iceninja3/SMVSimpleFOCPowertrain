
#include <SimpleFOC.h>

// --- MOTOR 1 CONFIG ---
BLDCMotor motor1 = BLDCMotor(46);
BLDCDriver6PWM driver1 = BLDCDriver6PWM(13, 14, 11, 12, 9, 10);
float target1 = 10.0;

// --- MOTOR 2 CONFIG (New Pins) ---
// Pins: 21, 47, 35, 48, 37, 26
BLDCMotor motor2 = BLDCMotor(46); 
BLDCDriver6PWM driver2 = BLDCDriver6PWM(21, 47, 35, 48, 37, 36);
float target2 = 10.0;

// Commander setup
Commander command = Commander(Serial);
void doTarget1(char* cmd) { command.scalar(&target1, cmd); }
void doTarget2(char* cmd) { command.scalar(&target2, cmd); }

void setup() {
  Serial.begin(115200);
  SimpleFOCDebug::enable(&Serial);

  // Initialize Driver 1
  driver1.voltage_power_supply = 48;
  driver1.voltage_limit = 24;
  driver1.init();
  motor1.linkDriver(&driver1);
  motor1.voltage_limit = 5; // Safety limit
  motor1.controller = MotionControlType::velocity_openloop;
  motor1.init();

  // Initialize Driver 2
  driver2.voltage_power_supply = 48;
  driver2.voltage_limit = 24;
  driver2.init();
  motor2.linkDriver(&driver2);
  motor2.voltage_limit = 5; // Safety limit
  motor2.controller = MotionControlType::velocity_openloop;
  motor2.init();

  // Commander commands
  // Use 'A' for motor 1 and 'B' for motor 2 in Serial Monitor
  command.add('A', doTarget1, "target velocity M1");
  command.add('B', doTarget2, "target velocity M2");

  Serial.println("Dual Motors Ready (Open Loop)");
  _delay(1000);
}

void loop() {
  // Run both motors
  motor1.move(target1);
  motor2.move(target2);

  command.run();
}

// // Open loop motor control example
// #include <SimpleFOC.h>

// // BLDC motor instance - 46 pole pairs
// BLDCMotor motor = BLDCMotor(46);

// // Driver instance: 6PWM pins
// BLDCDriver6PWM driver = BLDCDriver6PWM(13, 14, 11, 12, 9, 10);

// // Target velocity variable
// float target_velocity = 10 * 3.14; // ~5 Hz rotation

// // Commander instance
// Commander command = Commander(Serial);
// void doTarget(char* cmd) { command.scalar(&target_velocity, cmd); }
// void doLimit(char* cmd) { command.scalar(&motor.voltage_limit, cmd); }

// void setup() {
//   Serial.begin(115200);
//   SimpleFOCDebug::enable(&Serial);

//   // 1. Driver config
//   driver.voltage_power_supply = 48;
//   driver.voltage_limit = 24;
//   if(!driver.init()){
//     Serial.println("Driver init failed!");
//     return;
//   }

//   // 2. Link driver to motor
//   motor.linkDriver(&driver);

//   // 3. Set control type to OPEN LOOP velocity
//   motor.controller = MotionControlType::velocity_openloop;

//   // 4. Limits
//   motor.voltage_limit = 5;   // Start LOW to avoid overheating (Open loop draws constant current)
//   motor.velocity_limit = 20; // [rad/s]

//   // 5. Init motor hardware
//   if(!motor.init()){
//     Serial.println("Motor init failed!");
//     return;
//   }

//   // Add commands
//   command.add('T', doTarget, "target velocity");
//   command.add('L', doLimit, "voltage limit");

//   Serial.println("Motor ready! (Open Loop)");
//   _delay(1000);
// }

// void loop() {
//   // In open loop, we ONLY call move(). 
//   // motor.loopFOC() is NOT used because there is no feedback loop to run.
//   motor.move(target_velocity);

//   // User communication
//   command.run();
// }
