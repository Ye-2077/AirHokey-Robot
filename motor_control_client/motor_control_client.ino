#include <WiFi.h>
#include <SimpleFOC.h>

const char* ssid = "TP-Link_285B";
const char* password = "82165147";
const IPAddress serverIP(192, 168, 0, 103);
uint16_t serverPort = 59630;
WiFiClient client;

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(4);
BLDCDriver3PWM driver = BLDCDriver3PWM(27, 33, 15, 12);

void setup() {
    Serial.begin(115200);
    // Initialize WiFi
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("Connected to WiFi");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    // Motor driver setup
    driver.voltage_power_supply = 24;
    driver.init();
    motor.linkDriver(&driver);

    // Motor configuration
    motor.voltage_limit = 5;   // [V]
    motor.velocity_limit = 12.56; // [rad/s]
    motor.controller = MotionControlType::angle_openloop;
    motor.init();
}

void loop() {
    if (client.connect(serverIP, serverPort)) {
        Serial.println("Connected to server");

        // Waiting for the instruction
        while (!client.available()) {
            delay(1);
        }

        // Reading the instruction
        String instruction = client.readStringUntil('\n');
        float target = instruction.toFloat();
        Serial.print("Received target: ");
        Serial.println(target);

        // Applying the instruction to the motor
        motor.target = target;
        motor.move();

        // client.stop();
    } else {
        Serial.println("Failed to connect to server");
    }
    // delay(5000); // Wait for a bit before trying again
}
