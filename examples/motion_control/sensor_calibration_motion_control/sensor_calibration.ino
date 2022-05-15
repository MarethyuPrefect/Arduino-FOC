/**
 * Torque control example using voltage control loop.
 * 
 * Most of the low-end BLDC driver boards doesn't have current measurement therefore SimpleFOC offers 
 * you a way to control motor torque by setting the voltage to the motor instead hte current. 
 * 
 * This makes the BLDC motor effectively a DC motor, and you can use it in a same way.
 */
#include <SimpleFOC.h>

// magnetic sensor instance - SPI
MagneticSensorSPI sensor = MagneticSensorSPI(AS5147_SPI, 10);
// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(11);
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 5, 6, 8);

// instantiate the calibrated sensor object
CalibratedSensor sensor_calibrated  = CalibratedSensor(sensor);

// inline current sensor instance
InlineCurrentSense current_sense = InlineCurrentSense(0.001, 50.0, A0, A1);

// voltage set point variable
float target_voltage = 2;
// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_voltage, cmd); }

void setup() {
  sensor.init();

  // Link motor to sensor
  motor.linkSensor(&sensor);
  // power supply voltage
  driver.voltage_power_supply = 12;
  driver.init();
  motor.linkDriver(&driver);
  // aligning voltage 
  motor.voltage_sensor_align = 5;
  // set motion control loop to be used
  motor.controller = MotionControlType::torque;
  // use monitoring with serial 
  Serial.begin(115200);
  // comment out if not needed
  motor.useMonitoring(Serial);
  // initialize motor
  motor.init();

  // do sensor eccentricity calibration
  sensor_calibrated.calibrate(motor);
  // initalize calibrated sensor. 
  sensor_calibrated.init();
  motor.linkSensor(sensor_calibrated);

  // align sensor and start FOC
  motor.initFOC();
  
  // add target command T
  command.add('T', doTarget, "target voltage");
  
  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target voltage using serial terminal:"));
  _delay(1000);
}

void loop() {

}