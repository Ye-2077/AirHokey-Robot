#include <WiFi.h>
#include <SimpleFOC.h>

///////////////////////  WIFI  /////////////////////////
// const char* ssid = "daisy_wifi";
// const char* password = "daisy1209";
const char* ssid = "Chenyn";
const char* password = "cyn123456@";
// const IPAddress serverIP(172, 20, 10, 12);
const IPAddress serverIP(172, 20, 10, 4);
uint16_t serverPort = 59630;
WiFiClient client;
///////////////////////////////////////////////////////


///////////////////////  MOTOR & SENSOR  /////////////////////////
BLDCMotor motor = BLDCMotor(4); // motor instance
BLDCDriver3PWM driver = BLDCDriver3PWM(27, 33, 15, 12); // driver instance
MagneticSensorI2C as5600 = MagneticSensorI2C(AS5600_I2C); // sensor instance
//////////////////////////////////////////////////////////////////

float target_angle = 0; // angle set point variable
float offset_angle = 1.9; //3.8(RIGHR) //5.19(LEFT)


void setup() {
  Serial.begin(115200);

  /////////////////// WIFI Configuration ///////////////////
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        }
  Serial.println("Connected to WiFi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
 
  /////////////////// AS5600 Configuration ////////////////////
  as5600.init();
  Serial.println("AS5600 ready");

  /////////////////// Driver Configuration ///////////////////
  driver.voltage_power_supply = 12; //27.6
  driver.voltage_limit = 12; // max voltage
  driver.init();
  Serial.println("Driver ready");

  /////////////////// Motor Configuration ///////////////////
  motor.linkDriver(&driver);
  motor.linkSensor(&as5600);
  motor.voltage_sensor_align = 3; // adjust the voltage

  motor.controller = MotionControlType::angle; // postion control

  // velocity PID control - default P=0.5 I = 10 D =0 
  // motor.PID_velocity.P = 0.2;
  // motor.PID_velocity.I = 0.0; //0.005;
  // motor.PID_velocity.D = 0.05;
  motor.PID_velocity.P = 0.25;
  motor.PID_velocity.I = 0.002;
  motor.PID_velocity.D = 0.08;
  
  // motor.PID_velocity.P = 0.2;
  // motor.PID_velocity.I = 0.002;
  // motor.PID_velocity.D = 0.05;

  motor.PID_velocity.output_ramp = 200; // default value is 20
  motor.LPF_velocity.Tf = 5; // default 5ms

  // angle P controller -  default P=20
  // motor.P_angle.P = 10;
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

void loop() {
  // AS5600 update
  as5600.update();
  as5600.getAngle();
  as5600.getVelocity();
  // Serial.print(as5600.getAngle());
  // Serial.print("\t");
  // Serial.println(as5600.getVelocity());

  // Handle WiFi connection without blocking
  if (!client.connected()) {
    if (client.connect(serverIP, serverPort)) {
      // Serial.println("Connected to server");
      String identifier = "ESP32_A\n";//left
      // String identifier = "ESP32_B\n";//right
      client.println(identifier);
    } else {
      Serial.println("Failed to connect to server");
    }
  }

  if (client.available()) { // Check if there are incoming bytes available from the server
    String instruction = client.readStringUntil('\n');
    target_angle = instruction.toFloat(); // Set new target angle from network
    Serial.print("Received target: ");
    Serial.println(target_angle);
  }

  motor.loopFOC();
  motor.move(target_angle+offset_angle);
}

