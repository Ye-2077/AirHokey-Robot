// #include <WiFi.h>

// const char *ssid = "TP-Link_285B"; //wifi name (do not use 5G wifi)
// const char *password = "82165147"; //wifi password

// void setup()
// {
//   Serial.begin(115200);
//   Serial.println();

//   WiFi.begin(ssid, password); //连接网络

//   while (WiFi.status() != WL_CONNECTED) //等待网络连接成功
//   {
//     delay(500);
//     Serial.print(".");
//   }
//   Serial.println("WiFi connected!");

//   Serial.println("IP address: ");
//   Serial.println(WiFi.localIP()); //打印模块IP
// }

// void loop()
// {
// }
#include <WiFi.h>

const char *ssid = "TP-Link_285B"; //wifi名
const char *password = "82165147";//wifi密码

const IPAddress serverIP(192,168,0,103); //欲访问的服务端IP地址
uint16_t serverPort = 59630;         //服务端口号

WiFiClient client; //声明一个ESP32客户端对象，用于与服务器进行连接

void setup()
{
    Serial.begin(115200);
    Serial.println();

    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false); //关闭STA模式下wifi休眠，提高响应速度
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    Serial.println("Connected");
    Serial.print("IP Address:");
    Serial.println(WiFi.localIP());
}

void loop()
{
    Serial.println("尝试访问服务器");
    if (client.connect(serverIP, serverPort)) //尝试访问目标地址
    {
        Serial.println("访问成功");
        client.print("Hello world!");                    //向服务器发送数据
        while (client.connected() || client.available()) //如果已连接或有收到的未读取的数据
        {
            if (client.available()) //如果有数据可读取
            {
                String line = client.readStringUntil('\n'); //读取数据到换行符
                Serial.print("读取到数据：");
                Serial.println(line);
                client.write(line.c_str()); //将收到的数据回发
            }
        }
        Serial.println("关闭当前连接");
        client.stop(); //关闭客户端
    }
    else
    {
        Serial.println("访问失败");
        client.stop(); //关闭客户端
    }
    delay(5000);
}

