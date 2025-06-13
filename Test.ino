// // Cấu hình Blynk
// #define BLYNK_TEMPLATE_ID "TMPL6eL8SB4vv"
// #define BLYNK_TEMPLATE_NAME "IoT and application"
// #define BLYNK_AUTH_TOKEN "HJM6xzTvvXUqODvLACls9v32e7VHkc_5"
#define BLYNK_TEMPLATE_ID "TMPL6VLBLn5wf"
#define BLYNK_TEMPLATE_NAME "Control Motor"
#define BLYNK_AUTH_TOKEN "2K6l9xL-6ELElgmyOYwJHV0HQKRzv4_-"

#include <Arduino.h>
#include <ArduinoBLE.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

// Cấu hình WiFi
const char* ssid = "Thobaymau ";
const char* password = "19102000";

// Định nghĩa UUID
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define PULSE_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define SETPOINT_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26aa"
#define DISTANCE_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a9"

// Địa chỉ MAC của server (thay bằng MAC thực tế của server)
const char* serverMacAddress = "14:33:5c:2f:c6:c6";

// Biến toàn cục
int motorSpeed = 0;
float distance = 0.0;
BLECharacteristic pulseCharacteristic;
BLECharacteristic setpointCharacteristic;
BLECharacteristic distanceCharacteristic;

// Hàm gọi khi Blynk gửi setpoint qua Virtual Pin V2
BLYNK_WRITE(V2) {
  int setpoint = param.asInt();
  if (setpoint >= 0 && setpointCharacteristic && setpointCharacteristic.canWrite()) {
    String setpointStr = String(setpoint);
    setpointCharacteristic.writeValue(setpointStr.c_str());
    Serial.print("Sent setpoint to server from Blynk: ");
    Serial.println(setpoint);
  } else {
    Serial.println("Invalid setpoint or setpoint characteristic not writable!");
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // Kết nối WiFi
  Serial.print("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("Connected to WiFi!");

  // Khởi động Blynk
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, password);
  Serial.println("Connected to Blynk!");

  // Khởi tạo BLE
  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }

  // Bắt đầu quét thiết bị BLE
  BLE.scan();
  Serial.println("Scanning for BLE server by MAC address...");
}

void loop() {
  // Chạy Blynk
  Blynk.run();

  // Kiểm tra nếu tìm thấy thiết bị BLE
  BLEDevice peripheral = BLE.available();

  if (peripheral) {
    // Kiểm tra địa chỉ MAC
    if (peripheral.address() == String(serverMacAddress)) {
      BLE.stopScan();
      Serial.print("Found server with MAC: ");
      Serial.println(peripheral.address());

      // Kết nối đến peripheral
      if (peripheral.connect()) {
        Serial.println("Connected to server!");
      } else {
        Serial.println("Failed to connect to server!");
        BLE.scan();
        return;
      }

      // Tìm service
      if (peripheral.discoverAttributes()) {
        BLEService service = peripheral.service(SERVICE_UUID);
        if (service) {
          // Tìm characteristics
          pulseCharacteristic = service.characteristic(PULSE_CHARACTERISTIC_UUID);
          setpointCharacteristic = service.characteristic(SETPOINT_CHARACTERISTIC_UUID);
          distanceCharacteristic = service.characteristic(DISTANCE_CHARACTERISTIC_UUID);

          if (pulseCharacteristic && setpointCharacteristic && distanceCharacteristic) {
            // Đăng ký thông báo
            if (pulseCharacteristic.canSubscribe()) {
              pulseCharacteristic.subscribe();
              Serial.println("Subscribed to pulse characteristic.");
            } else {
              Serial.println(" banssPulse characteristic does not support notify!");
            }
            if (distanceCharacteristic.canSubscribe()) {
              distanceCharacteristic.subscribe();
              Serial.println("Subscribed to distance characteristic.");
            } else {
              Serial.println("Distance characteristic does not support notify!");
            }

            // Vòng lặp xử lý khi kết nối
            while (peripheral.connected()) {
              Blynk.run(); // Tiếp tục chạy Blynk

              // Đọc dữ liệu từ pulse characteristic
              if (pulseCharacteristic.valueUpdated()) {
                int length = pulseCharacteristic.valueLength();
                char data[length + 1];
                pulseCharacteristic.readValue(data, length);
                data[length] = '\0';
                motorSpeed = atoi(data);
                Serial.print("Received motorSpeed: ");
                Serial.println(motorSpeed);
                Blynk.virtualWrite(V0, motorSpeed); // Gửi tốc độ lên V0
              }

              // Đọc dữ liệu từ distance characteristic
              if (distanceCharacteristic.valueUpdated()) {
                int length = distanceCharacteristic.valueLength();
                char data[length + 1];
                distanceCharacteristic.readValue(data, length);
                data[length] = '\0';
                distance = atof(data);
                Serial.print("Received distance: ");
                Serial.println(distance);
                Blynk.virtualWrite(V1, distance); // Gửi khoảng cách lên V1
              }

              // Gửi setpoint từ Serial Monitor (tùy chọn)
              if (Serial.available()) {
                String input = Serial.readStringUntil('\n');
                input.trim();
                int setpoint = input.toInt();
                if (setpoint >= 0 && setpointCharacteristic.canWrite()) {
                  String setpointStr = String(setpoint);
                  setpointCharacteristic.writeValue(setpointStr.c_str());
                  Serial.print("Sent setpoint to server from Serial: ");
                  Serial.println(setpoint);
                  Blynk.virtualWrite(V2, setpoint); // Cập nhật setpoint trên Blynk
                } else {
                  Serial.println("Invalid setpoint or setpoint characteristic not writable!");
                }
              }
            }

            Serial.println("Disconnected from server!");
            peripheral.disconnect();
          } else {
            Serial.println("Characteristics not found!");
            peripheral.disconnect();
          }
        } else {
          Serial.println("Service not found!");
          peripheral.disconnect();
        }
      } else {
        Serial.println("Failed to discover attributes!");
        peripheral.disconnect();
      }
    }
    BLE.scan();
  }
}