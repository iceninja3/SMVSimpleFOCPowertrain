// Open loop motor control example
#include <SimpleFOC.h>
// BLDC motor & driver instance
// BLDCMotor motor = BLDCMotor(pole pair number);
int sensorPin = 16;   // ESP32 GPIO16 (ADC2_CH0)
int neutral = 1197;
int sensorValue = 0;
double max_velocity = 2*3.14;
double max_acceleration = 1;
//double cur

BLDCMotor motor = BLDCMotor(46);

//  BLDCDriver6PWM( int phA_h, int phA_l, int phB_h, int phB_l, int phC_h, int phC_l, int en)
//  - phA_h, phA_l - A phase pwm pin high/low pair 
//  - phB_h, phB_l - B phase pwm pin high/low pair
//  - phB_h, phC_l - C phase pwm pin high/low pair
//  - enable pin    - (optional input)
BLDCDriver6PWM driver = BLDCDriver6PWM(13,14, 11,12, 9,10);


//HallSensor sensor = HallSensor(HALL_A, HALL_B, HALL_C, POLE_PAIRS);
HallSensor sensor = HallSensor(42, 41, 40, 46); // 46 is pole pair count--------------------------------------- CHANGE THIS

// Stepper motor & driver instance
//StepperMotor motor = StepperMotor(50);
//StepperDriver4PWM driver = StepperDriver4PWM(9, 5, 10, 6,  8);


//target variable
float target_velocity = 10*3.14;

// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_velocity, cmd); }
void doLimit(char* cmd) { command.scalar(&motor.voltage_limit, cmd); }

void setup() {
  pinMode(42, INPUT);
  pinMode(41, INPUT);
  pinMode(40, INPUT);

  pinMode(16, INPUT);
  // use monitoring with serial 
  Serial.begin(115200);
  // enable more verbose output for debugging
  // comment out if not needed
  SimpleFOCDebug::enable(&Serial);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 48;
  // limit the maximal dc voltage the driver can set
  // as a protection measure for the low-resistance motors
  // this value is fixed on startup
  driver.voltage_limit = 24;
  if(!driver.init()){
    Serial.println("Driver init failed!");
    return;
  }

  // limiting motor movements
  // limit the voltage to be set to the motor
  // start very low for high resistance motors
  // current = voltage / resistance, so try to be well under 1Amp
  motor.voltage_limit = 24; // [V]
 
  // open loop control config
  //motor.controller = MotionControlType::velocity_openloop;

  //CLOSED LOOP control config
  motor.controller = MotionControlType::velocity;
  sensor.init();
  motor.linkSensor(&sensor);
    // link the motor and the driver
  motor.linkDriver(&driver);

  pinMode(38, OUTPUT);
  // init motor hardware
  if(!motor.init()){
    Serial.println("Motor init failed!");
    digitalWrite(38, HIGH);
    return;
  }

  // add target command T
  // command.add('T', doTarget, "target velocity");
  // command.add('L', doLimit, "voltage limit");

  // Serial.println("Motor ready!");
  // Serial.println("Set target velocity [rad/s]");
  // _delay(1000);

  //Initializations MOTOR + SENSOR 
  // align sensor and set zero electric angle
  motor.initFOC();
}

void loop() {

  // sensorValue = analogRead(sensorPin);
  // double curVelocity = (sensorValue - neutral)*max_velocity/(4095-neutral);
  // //double c

  // if(curVelocity < 0) {
  //   curVelocity = 0;
  // }
  // // open loop velocity movement
  // // using motor.voltage_limit and motor.velocity_limit
  // // to turn the motor "backwards", just set a negative target_velocity
  // motor.loopFOC();
  // motor.move(curVelocity);

  // target_velocity = 10*3.14;

  // sensorValue = analogRead(sensorPin);
  // double delta = sensorValue - neutral;
  // if (fabs(delta) < 50 || delta < 0) {   // adjustable deadband
  //   if (motor.enabled) motor.disable();
  // } 
  // else {
  //   if (!motor.enabled) motor.enable();
  //   double curVelocity = delta * max_velocity / (4095 - neutral);
  //   motor.move(curVelocity);
  // }

double curSense = analogRead(sensorPin);
double change = curSense - sensorValue;
sensorValue = curSense;

double delta = sensorValue - neutral;
const double deadband = 50;          // adjust as needed
const double brake_velocity = -5.0;  // plug braking strength (tune this)

// CASE 1: Throttle above neutral (forward)
if (delta > deadband) {
  if (!motor.enabled) motor.enable();
  double curVelocity = delta * max_velocity / (4095 - neutral);
  motor.move(curVelocity);
}

// CASE 2: Throttle released / below neutral
else if (delta < 0) {
  if (!motor.enabled) motor.enable();
  motor.move(brake_velocity);  // plug braking command
}

// CASE 3: Neutral / small deadband zone
else {
  if (motor.enabled) motor.disable();  // disable driver for idle
}

  motor.loopFOC();
  //target_velocity = abs(0.5*3.14*sin(millis()/1000));
  // user communication
  command.run();
}



// int sensorPin = 16;   // ESP32 GPIO16 (ADC2_CH0)
// int sensorValue = 0;

// void setup() {
//   Serial.begin(115200);  // Faster baud rate is fine on ESP32
// }

// void loop() {
//   // Read analog value (0–4095 on 12-bit ADC by default)
//   sensorValue = analogRead(sensorPin);

//   Serial.println(sensorValue);
//   delay(200);
// }