#include <Arduino.h>
#include <PID_v1_bc.h>
#include <driver/ledc.h>
#include <ArduinoBLE.h>

// Định nghĩa UUID
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define PULSE_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define SETPOINT_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26aa"
#define DISTANCE_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a9"
#define RPM_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26ab" // UUID mới cho RPM

// Khai báo BLE service và characteristics
BLEService pService(SERVICE_UUID);
BLECharacteristic pulseCharacteristic(PULSE_CHARACTERISTIC_UUID, BLERead | BLENotify, 4);
BLECharacteristic setpointCharacteristic(SETPOINT_CHARACTERISTIC_UUID, BLEWrite, 4);
BLECharacteristic distanceCharacteristic(DISTANCE_CHARACTERISTIC_UUID, BLERead | BLENotify, 4);
BLECharacteristic rpmCharacteristic(RPM_CHARACTERISTIC_UUID, BLERead | BLENotify, 4); // Characteristic mới cho RPM

// Chân kết nối L298N
#define ENA 18  // PWM
#define IN1 19
#define IN2 21

// Chân encoder
#define ENC_A 15  // Encoder A (interrupt)

// Chân SRF05
#define SRF05_TRIG 2
#define SRF05_ECHO 4

// PID parameters
double Kp = 1.2, Ki = 0.5, Kd = 0.1;

// Encoder variables
volatile long encoderPulse = 0;
const int PPR = 360; // Số xung trên mỗi vòng quay (thay đổi theo encoder của bạn)

// PID variables
double Setpoint, Input, Output;

// Timer interrupt flag
hw_timer_t* timer = NULL;
volatile bool flagTimer = false;

// PWM settings
const int pwmFreq = 20000;    // 20kHz
const int pwmChannel = 0;
const int pwmResolution = 8;  // 8 bit (0-255)

// Tần số cập nhật PID (ms)
const int sampleTime = 100;

// Hàm đo khoảng cách SRF05
long getDistanceCM() {
  digitalWrite(SRF05_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(SRF05_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(SRF05_TRIG, LOW);
  long duration = pulseIn(SRF05_ECHO, HIGH, 25000); // timeout 25ms ~ 4m
  long distance = duration / 58; // Công thức cm = duration (us) / 58
  return distance;
}

// Ngắt encoder tăng xung
void IRAM_ATTR handleEncoderA() {
  encoderPulse++;
}

// Ngắt timer
void IRAM_ATTR onTimer() {
  flagTimer = true;
}

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // Khởi tạo BLE
  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }

  // In địa chỉ MAC của server
  String macAddress = BLE.address();
  Serial.print("Server MAC Address: ");
  Serial.println(macAddress);

  // Đặt tên thiết bị
  BLE.setDeviceName("BLE_SERVER");
  BLE.setLocalName("BLE_SERVER");

  // Thiết lập service và characteristics
  BLE.setAdvertisedService(pService);
  pService.addCharacteristic(pulseCharacteristic);
  pService.addCharacteristic(setpointCharacteristic);
  pService.addCharacteristic(distanceCharacteristic);
  pService.addCharacteristic(rpmCharacteristic); // Thêm characteristic cho RPM
  BLE.addService(pService);

  // Gửi giá trị khởi tạo
  pulseCharacteristic.writeValue("0");
  setpointCharacteristic.writeValue("0");
  distanceCharacteristic.writeValue("0");
  rpmCharacteristic.writeValue("0");

  // Bắt đầu quảng bá
  BLE.advertise();
  Serial.println("BLE server is advertising");

  // Setup L298N
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  // Setup PWM
  ledcSetup(pwmChannel, pwmFreq, pwmResolution);
  ledcAttachPin(ENA, pwmChannel);

  // Setup Encoder
  pinMode(ENC_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), handleEncoderA, RISING);

  // Setup SRF05
  pinMode(SRF05_TRIG, OUTPUT);
  pinMode(SRF05_ECHO, INPUT);

  // Setup PID
  Setpoint = 20;
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(sampleTime);
  myPID.SetOutputLimits(-255, 255);

  // Setup timer interrupt (100ms)
  timer = timerBegin(0, 80, true); // Timer 0, 1 tick = 1us
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, sampleTime * 1000, true);
  timerAlarmEnable(timer);
}

// Hàm điều khiển động cơ
void setMotor(int pwm) {
  if (pwm > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    ledcWrite(pwmChannel, pwm);
  } else if (pwm < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    ledcWrite(pwmChannel, -pwm);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    ledcWrite(pwmChannel, 0);
  }
}

void loop() {
  BLEDevice central = BLE.central();

  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());

    while (central.connected()) {
      // Nhận setpoint từ setpointCharacteristic
      if (setpointCharacteristic.written()) {
        int length = setpointCharacteristic.valueLength();
        char data[length + 1];
        setpointCharacteristic.readValue(value, length);
        data[length] = '\0';
        int newSetpoint = atoi(data);
        if (newSetpoint >= 0) {
          Setpoint = newSetpoint;
          Serial.print("Received setpoint from central: ");
          Serial.println(newSetpoint);
        }
      }

      // Xử lý PID và gửi dữ liệu khi timer ngắt
      if (flagTimer) {
        flagTimer = false;

        // Đọc khoảng cách SRF05
        long distanceCM = getDistanceCM();

        // Nếu khoảng cách ≤ 10cm, dừng động cơ
        if (distanceCM > 0 && distanceCM <= 10) {
          if (Setpoint != 0) {
            Setpoint = 0;
          }
        }

        // Đọc xung encoder
        noInterrupts();
        long pulse = encoderPulse;
        encoderPulse = 0;
        interrupts();

        // Tính RPM
        float rpm = (pulse * 280)/600;

        Input = pulse;

        // Tính toán PID
        myPID.Compute();
        setMotor((int)Output);

        // Gửi dữ liệu qua BLE
        char pulseData[20];
        snprintf(pulseData, sizeof(pulseData), "%ld", pulse);
        pulseCharacteristic.writeValue(pulseData);

        char distanceData[20];
        snprintf(distanceData, sizeof(distanceData), "%ld", distanceCM);
        distanceCharacteristic.writeValue(distanceData);

        char rpmData[20];
        snprintf(rpmData, sizeof(rpmData), "%.2f", rpm);
        rpmCharacteristic.writeValue(rpmData);

        // In dữ liệu lên Serial
        Serial.print("Sent pulse: ");
        Serial.println(pulseData);
        Serial.print("Sent distance: ");
        Serial.println(distanceData);
        Serial.print("Sent RPM: ");
        Serial.println(rpmData);
      }
    }

    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}
