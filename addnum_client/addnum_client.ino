#include <WiFi.h>

const char *ssid = "TP-Link_285B";
const char *password = "82165147";

const IPAddress serverIP(192,168,0,103);
uint16_t serverPort = 59630;

WiFiClient client;

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("Connected");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
}

void loop() {
    if (client.connect(serverIP, serverPort)) {
        Serial.println("Connected to server");
        
        // 等待服务器发送数据
        while (!client.available()) {
            delay(1);
        }
        
        String received = client.readStringUntil('\n');
        Serial.print("Received: ");
        Serial.println(received);

        // 分割字符串，计算两数之和
        int commaIndex = received.indexOf(',');
        int num1 = received.substring(0, commaIndex).toInt();
        int num2 = received.substring(commaIndex + 1).toInt();
        int sum = num1 + num2;
        
        // 发送计算结果回服务器
        client.println(sum);
        Serial.print("Sent sum: ");
        Serial.println(sum);
        
        client.stop();
    } else {
        Serial.println("Connection to server failed");
    }
    delay(5000); // 等待一段时间再次尝试
}
