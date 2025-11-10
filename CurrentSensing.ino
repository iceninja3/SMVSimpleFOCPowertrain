// #vibecoded

// TODO:
// 1. Change Hall sensor sensitivity from it's datasheet. Convert their units which are prob mV/A to V/A
// 2.Change Current Limit What is the motors stall current or max continous current? 
// 3. Change Sensor's Out pins (A0 and A1)
// 4. Change power supply voltage accordingly from 48V (fully charged it will be like 54V). Better number is more efficient.

// Open loop motor control example
#include <SimpleFOC.h>
//#include <SimpleFOCDrivers.h> // --- NEW --- (May be needed for current sense)

// BLDC motor & driver instance
// BLDCMotor motor = BLDCMotor(pole pair number);
BLDCMotor motor = BLDCMotor(46);

//  BLDCDriver6PWM( int phA_h, int phA_l, int phB_h, int phB_l, int phC_h, int phC_l, int en)
BLDCDriver6PWM driver = BLDCDriver6PWM(13,14, 11,12, 9,10);

//HallSensor sensor = HallSensor(HALL_A, HALL_B, HALL_C, POLE_PAIRS);
HallSensor sensor = HallSensor(42, 41, 40, 46); // 46 is pole pair count

// // --- NEW ---
// // InlineCurrentSense(float shunt_resistor, float gain, int pinA, int pinB, int pinC (optional))
// //   - shunt_resistor: Shunt resistance in Ohms (e.g., 0.01)
// //   - gain:           Amplifier gain (e.g., 50)
// //   - pinA, pinB:     MCU Analog (ADC) pins reading the amplified voltage for phase A and B
// // ** YOU MUST CHANGE THESE VALUES TO MATCH YOUR HARDWARE **
// float shunt_resistance = 0.01; // Ohms
// float amp_gain = 50.0;         // V/V
// int pin_current_A = A0;        // Example analog pin 
// int pin_current_B = A1;        // Example analog pin
// InlineCurrentSense current_sense = InlineCurrentSense(shunt_resistance, amp_gain, pin_current_A, pin_current_B);
// // --- END NEW ---

// HallCurrentSense(int pinA, int pinB, float sensitivity, int pinC (optional))
//
//   - pinA, pinB:   MCU Analog (ADC) pins reading the sensor's OUT pin
//   - sensitivity:  The sensor's sensitivity in Volts per Amp (V/A)
//
// Example: An ACS758-050B has a sensitivity of 0.04V/A (or 40mV/A)

float sensor_sensitivity = 0.04; // Example: 40mV/A = 0.04V/A
int pin_current_A = A0;          // Example analog pin 
int pin_current_B = A1;          // Example analog pin
HallCurrentSense current_sense = HallCurrentSense(pin_current_A, pin_current_B, sensor_sensitivity);


//target variable
float target_velocity = 10 * 3.14; // --- CHANGED --- (Moved from loop)

// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_velocity, cmd); }
void doLimit(char* cmd) { command.scalar(&motor.voltage_limit, cmd); }
void doCurrentLimit(char* cmd) { command.scalar(&motor.current_limit, cmd); } // --- NEW ---

void setup() {
  pinMode(42, INPUT);
  pinMode(41, INPUT);
  pinMode(40, INPUT);
  
  Serial.begin(115200);
  SimpleFOCDebug::enable(&Serial);

  // driver config
  driver.voltage_power_supply = 48;
  driver.voltage_limit = 24; 
  if(!driver.init()){
    Serial.println("Driver init failed!");
    return;
  }
  Serial.println("Driver init success.");

  // --- NEW ---
  // Initialise and calibrate the current sense
  // This finds the zero-current "offset" voltage for each ADC
  if(!current_sense.init()){
    Serial.println("Current Sense init failed!");
    return;
  }
  Serial.println("Current Sense init success.");
  // Link the current sense to the driver
  current_sense.linkDriver(&driver);
  // --- END NEW ---

  // --- CHANGED ---
  // Add a CURRENT limit for motor protection
  // This is the entire reason for adding current sensing!
  motor.voltage_limit = 24;     // [V] Still good to have
  motor.current_limit = 5;      // [Amps] - ** SET A SAFE VALUE FOR YOUR MOTOR **
  // --- END CHANGED ---
  
  //CLOSED LOOP control config
  motor.controller = MotionControlType::velocity;
  
  sensor.init();
  motor.linkSensor(&sensor);
  
  motor.linkDriver(&driver);

  // --- NEW ---
  // Link the current sense to the motor
  motor.linkCurrentSense(&current_sense);
  // Tell the FOC algorithm to use the current-based torque control
  motor.torque_controller = TorqueControlType::foc_current;
  // --- END NEW ---

  pinMode(38, OUTPUT);
  // init motor hardware
  if(!motor.init()){
    Serial.println("Motor init failed!");
    digitalWrite(38, HIGH);
    return;
  }

  // --- CHANGED ---
  // Re-enable commander and add a current limit command
  command.add('T', doTarget, "target velocity");
  command.add('L', doLimit, "voltage limit");
  command.add('C', doCurrentLimit, "current limit"); // --- NEW ---

  Serial.println("Motor ready!");
  Serial.println("Set target velocity [rad/s]");
  // --- END CHANGED ---

  // align sensor and set zero electric angle
  motor.initFOC();
  
  Serial.println("Motor initFOC success.");
}

void loop() {
  // This function runs all the FOC math
  // It now runs BOTH the velocity loop AND the inner current loop
  motor.loopFOC();

  // The velocity controller will now automatically use the current loop
  // to achieve its target velocity.
  motor.move(target_velocity);

  // user communication
  command.run();
}