#include <SimpleFOC.h>

BLDCMotor motor = BLDCMotor(4); // motor instance
BLDCDriver3PWM driver = BLDCDriver3PWM(27, 33, 15, 12); // driver instance
MagneticSensorI2C as5600 = MagneticSensorI2C(AS5600_I2C); // sensor instance

void setup() {
  
  Serial.begin(115200);

  // as5600
  as5600.init();
  Serial.println("AS5600 ready");

  // driver
  driver.voltage_power_supply = 12; //27.6
  driver.voltage_limit = 12; // max voltage
  driver.init();
  Serial.println("Driver ready");

  // motor
  motor.linkDriver(&driver);
  motor.linkSensor(&as5600);
  motor.voltage_sensor_align = 3; // adjust the voltage

  motor.controller = MotionControlType::angle; // postion control

  // velocity PID control - default P=0.2 I = 0.005 D = 0.05
  motor.PID_velocity.P = 0.4;
  motor.PID_velocity.I = 0.002;
  motor.PID_velocity.D = 0.05;
  motor.PID_velocity.output_ramp = 100; // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor.LPF_velocity.Tf = 5; // default 5ms

  // angle P controller -  default P=20
  motor.P_angle.P = 10;
  motor.P_angle.I = 0;
  motor.P_angle.D = 0;
  motor.P_angle.output_ramp = 10000; // default is 1e6
  motor.LPF_angle.Tf = 0;

  // maximal velocity of the position control
  motor.velocity_limit = 5; // default 20
  motor.voltage_limit = 24; // default voltage_power_supply
  
  motor.useMonitoring(Serial);
  
  motor.init(); // initialize motor
  motor.initFOC(); // align encoder and start FOC

  Serial.println("Motor ready.");
  _delay(1000);

}

float target_angle = 0; // angle set point variable
long timestamp_us = _micros(); // timestamp for changing direction

float offset_angle = 0; //3.73(RIGHR) //0.76(LEFT)

void loop() {
  as5600.update();
  // display the angle and the angular velocity to the terminal
  Serial.print(as5600.getAngle());
  Serial.print("\t");
  Serial.println(as5600.getVelocity());

  // if(_micros() - timestamp_us > 10e6) {
  //       timestamp_us = _micros();
  //       target_angle = -target_angle;   
  // }
  
  // Check for serial input
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n'); // Read the input until newline
    if (input.startsWith("T")) { // Check if the input starts with 'T'
      float new_angle = input.substring(1).toFloat(); // Convert the rest of the string to float
      target_angle = new_angle; // Set the new target angle
      Serial.print("New target angle: ");
      Serial.println(target_angle);
    }
  }
  
  motor.loopFOC();
  motor.move(target_angle + offset_angle);
}
