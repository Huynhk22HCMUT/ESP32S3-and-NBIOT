#include "esp_camera.h"
#include <Arduino.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <PCF8563.h>
#include "esp_sleep.h"
#include <driver/rtc_io.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <DNSServer.h> // Thư viện để làm Captive Portal

Preferences preferences;
WebServer server(80);
DNSServer dnsServer;
const byte DNS_PORT = 53;

// Các biến cấu hình MQTT thay thế cho hardcode
String mqtt_server = "";
int mqtt_port;
String mqtt_client_id = "";
String mqtt_username = "";
String mqtt_password = "";
String mqtt_topic = "";

bool isAPMode = false;
SemaphoreHandle_t i2cMutex;
// ===== OLED SSD1306 + RTC PCF8563 =====
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE); // Khởi tạo đối tượng màn hình OLED SSD1306 sử dụng giao tiếp I2C phần cứng
PCF8563 rtc; // Khởi tạo đối tượng RTC (Real-Time Clock) PCF8563
// ===== GPIO =====
#define USER_BUTTON_PIN D1 // Định nghĩa chân GPIO cho nút nhấn người dùng
#define BUZZER_PIN A3 // Định nghĩa chân GPIO cho buzzer
#define DISPLAY_TIMEOUT 5000 // Thời gian timeout để tắt màn hình OLED (5 giây)
#define BATTERY_PIN A0
#define USB_PIN A3
#define FLASH_LED A8
unsigned long lastDisplayTime = 0; // Biến lưu thời gian lần cuối màn hình được bật
bool isDisplayOn = false; // Cờ kiểm tra xem màn hình OLED có đang bật không
// ===== SIM7020 CONFIG =====
#define SIM_TX_PIN 43 // Chân TX cho module SIM7020
#define SIM_RX_PIN 44 // Chân RX cho module SIM7020
#define SIM_BAUD 115200 // Tốc độ baud cho giao tiếp với SIM7020
#define APN "internet" // Tên APN (Access Point Name) cho kết nối mạng
// ===== MQTT CONFIG =====
// const char* MQTT_SERVER = "test.mosquitto.org"; // Địa chỉ server MQTT
// const int MQTT_PORT = 1883; // Cổng MQTT
// const char* MQTT_CLIENT_ID = "nb_iot_cam_device"; // ID client MQTT
// const char* MQTT_USERNAME = ""; // Tên đăng nhập MQTT (rỗng nếu không cần)
// const char* MQTT_TOPIC = "nb-iot/image/upload"; // Topic MQTT để publish ảnh
const unsigned long SEND_INTERVAL_MS = 300000UL; // Khoảng thời gian gửi ảnh (5 phút = 300000 ms)
const unsigned long SLEEP_INTERVAL_MS = 30000UL; // Khoảng thời gian ngủ mặc định (30 giây, nhưng sẽ được điều chỉnh)
HardwareSerial sim(1); // Khởi tạo đối tượng Serial cho SIM7020 (sử dụng UART1)
unsigned long lastSend = 0; // Biến lưu thời gian lần gửi ảnh cuối cùng
bool mqttSubscribed = false; // Cờ kiểm tra xem đã subscribe topic MQTT chưa
// ================== CAMERA CONFIG ==================
#define CAMERA_MODEL_XIAO_ESP32S3 // Định nghĩa model camera cho Xiao ESP32S3
#include "camera_pins.h" // Include file định nghĩa các chân camera
// ================== SLEEP VARIABLES ==================
RTC_DATA_ATTR int bootCount = 0; // Biến lưu số lần boot, lưu trữ trong RTC memory để giữ giá trị qua deep sleep
RTC_DATA_ATTR Time sleepStartTime;
RTC_DATA_ATTR unsigned long sleepDurationMs = 0;
// ================== READY FLAGS ==================
bool camReady = false; // Cờ kiểm tra camera đã sẵn sàng chưa
bool simReady = false; // Cờ kiểm tra SIM đã sẵn sàng chưa
// ================== BUZZER ==================
void beep(unsigned int frequency, unsigned int durationMs)
{
  pinMode(BUZZER_PIN, OUTPUT);   // ✅ ép mode output

  ledcSetup(0, frequency, 8);
  ledcAttachPin(BUZZER_PIN, 0);

  ledcWrite(0, 128);
  delay(durationMs);

  ledcWrite(0, 0);
  ledcDetachPin(BUZZER_PIN);     // ✅ giải phóng pin
}
// ================== OLED HIỂN THỊ PIN ==================
void drawBatteryIcon(int x, int y, int width, int height, int level) { // Hàm vẽ icon pin trên OLED
  u8g2.drawFrame(x, y, width, height); // Vẽ khung ngoài icon pin
  int capWidth = width / 10; // Tính chiều rộng phần nắp pin
  u8g2.drawBox(x + width, y + height / 4, capWidth, height / 2); // Vẽ phần nắp pin
  level = constrain(level, 0, 100); // Giới hạn mức pin từ 0 đến 100
  int innerWidth = width - 2; // Chiều rộng bên trong khung
  int innerHeight = height - 2; // Chiều cao bên trong khung
  int fillWidth = map(level, 0, 100, 0, innerWidth); // Tính chiều rộng phần đầy pin
  u8g2.drawBox(x + 1, y + 1, fillWidth, innerHeight); // Vẽ phần đầy pin
}
float readUSBVoltage()
{
  ledcDetachPin(BUZZER_PIN);        // ✅ NGẮT PWM
  pinMode(USB_PIN, INPUT);          // ✅ chuyển về ADC
  delay(10);                        // ✅ ổn định pin
  const int samples = 100;
  int values[samples];

  for(int i=0;i<samples;i++)
  {
    values[i] = analogRead(USB_PIN);
    delay(5);   // lấy mẫu nhanh
  }

  // sắp xếp mảng
  for(int i=0;i<samples-1;i++)
  {
    for(int j=i+1;j<samples;j++)
    {
      if(values[j] < values[i])
      {
        int t = values[i];
        values[i] = values[j];
        values[j] = t;
      }
    }
  }

  // bỏ 20 giá trị thấp nhất và cao nhất
  long total = 0;
  for(int i=20;i<samples-20;i++)
  {
    total += values[i];
  }

  float adc = total / (float)(samples - 40);

  float v_adc = (adc / 4095.0) * 3.3; 

  float voltage = v_adc * 5.0;

  return voltage;
}
void showBatteryScreen(int batteryLevel) { // Hàm hiển thị màn hình pin trên OLED
xSemaphoreTake(i2cMutex, portMAX_DELAY);

u8g2.clearBuffer();

Time nowTime = rtc.getTime();

char dateStr[20];
sprintf(dateStr, "%02d/%02d/20%02d", nowTime.day, nowTime.month, nowTime.year);

u8g2.setFont(u8g2_font_ncenB08_tr);
u8g2.drawStr(25, 15, dateStr);

float usbVoltage = readUSBVoltage();

if(usbVoltage > 3.0)
{
  u8g2.drawStr(25,40,"USB POWER");
}
else
{
  int x = 30, y = 25, w = 60, h = 30;
  drawBatteryIcon(x, y, w, h, batteryLevel);
}

u8g2.sendBuffer();

xSemaphoreGive(i2cMutex);
}
void turnOffDisplay() { // Hàm tắt màn hình OLED
  u8g2.clearDisplay(); // Xóa màn hình
  isDisplayOn = false; // Đặt cờ màn hình tắt
}

float readBatteryVoltage()
{ ledcDetachPin(BUZZER_PIN);        // ✅ tránh xung đột
  pinMode(BATTERY_PIN, INPUT);
  delay(10);
  const int samples = 50;
  int values[samples];

  for(int i=0;i<samples;i++)
  {
    values[i] = analogRead(BATTERY_PIN);
    delay(2);   // lấy mẫu nhanh
  }

  // sắp xếp mảng
  for(int i=0;i<samples-1;i++)
  {
    for(int j=i+1;j<samples;j++)
    {
      if(values[j] < values[i])
      {
        int t = values[i];
        values[i] = values[j];
        values[j] = t;
      }
    }
  }

  // bỏ 10 giá trị thấp nhất và cao nhất
  long total = 0;
  for(int i=10;i<samples-10;i++)
  {
    total += values[i];
  }

  float adc = total / (float)(samples - 20);

  float v_adc = (adc / 4095.0) * 3.3; 

  float voltage = v_adc * 5.0;

  return voltage;
}




int voltageToPercent(float v)
{
  float percent = (v) / (4.2) * 100.0;

  percent = constrain(percent, 0, 100); 

  return (int)percent;
}
// ================== CAMERA ==================
bool initCamera() { // Hàm khởi tạo camera
  camera_config_t config; // Khởi tạo cấu hình camera
  config.ledc_channel = LEDC_CHANNEL_0; // Kênh LEDC cho clock
  config.ledc_timer = LEDC_TIMER_0; // Timer LEDC
  config.pin_d0 = Y2_GPIO_NUM; // Chân dữ liệu D0
  config.pin_d1 = Y3_GPIO_NUM; // Chân dữ liệu D1
  config.pin_d2 = Y4_GPIO_NUM; // Chân dữ liệu D2
  config.pin_d3 = Y5_GPIO_NUM; // Chân dữ liệu D3
  config.pin_d4 = Y6_GPIO_NUM; // Chân dữ liệu D4
  config.pin_d5 = Y7_GPIO_NUM; // Chân dữ liệu D5
  config.pin_d6 = Y8_GPIO_NUM; // Chân dữ liệu D6
  config.pin_d7 = Y9_GPIO_NUM; // Chân dữ liệu D7
  config.pin_xclk = XCLK_GPIO_NUM; // Chân clock XCLK
  config.pin_pclk = PCLK_GPIO_NUM; // Chân clock pixel
  config.pin_vsync = VSYNC_GPIO_NUM; // Chân đồng bộ vertical
  config.pin_href = HREF_GPIO_NUM; // Chân đồng bộ horizontal
  config.pin_sscb_sda = SIOD_GPIO_NUM; // Chân SDA cho SSCB
  config.pin_sscb_scl = SIOC_GPIO_NUM; // Chân SCL cho SSCB
  config.pin_pwdn = PWDN_GPIO_NUM; // Chân power down
  config.pin_reset = RESET_GPIO_NUM; // Chân reset
  config.xclk_freq_hz = 20000000; // Tần số clock 20MHz
  config.pixel_format = PIXFORMAT_JPEG; // Định dạng pixel JPEG
  config.frame_size = FRAMESIZE_QVGA; // Kích thước frame 160x120
  config.jpeg_quality = 12; // Chất lượng JPEG (thấp hơn để chất lượng cao hơn)
  config.fb_count = 2; // Số lượng frame buffer
  config.grab_mode = CAMERA_GRAB_LATEST; // Chế độ lấy frame mới nhất
  config.fb_location = CAMERA_FB_IN_PSRAM; // Vị trí frame buffer trong PSRAM
  esp_err_t err = esp_camera_init(&config); // Khởi tạo camera với cấu hình
  if (err != ESP_OK) { // Kiểm tra lỗi khởi tạo
    Serial.printf(" Camera init failed! Error 0x%x\n", err); // In lỗi nếu thất bại
    return false; // Trả về false nếu thất bại
  }
  // ================== Tinh chỉnh sensor ==================
  sensor_t* s = esp_camera_sensor_get(); // Lấy con trỏ sensor camera
  if (s) { // Nếu sensor tồn tại
    s->set_brightness(s, 1); // Đặt độ sáng
    s->set_contrast(s, 2); // Đặt độ tương phản
    s->set_saturation(s, 1); // Đặt độ bão hòa màu
    s->set_sharpness(s, 2); // Đặt độ nét
    s->set_gainceiling(s, (gainceiling_t)GAINCEILING_16X); // Đặt giới hạn gain
    s->set_exposure_ctrl(s, 1); // Bật auto exposure
    s->set_whitebal(s, 1); // Bật auto white balance
    s->set_awb_gain(s, 1); // Bật AWB gain
    s->set_lenc(s, 1); // Bật lens correction
    s->set_vflip(s, 0); // Không lật dọc
    s->set_hmirror(s, 1); // Lật ngang
  }
  Serial.println(" Camera initialized successfully (High Quality Mode)"); // In thông báo thành công
  return true; // Trả về true nếu thành công
}
// ================== SIM FUNCTION ==================
String waitResponse(uint32_t timeout = 5000) { // Hàm chờ phản hồi từ SIM với timeout
  String resp; // Chuỗi lưu phản hồi
  uint32_t start = millis(); // Lấy thời gian bắt đầu
  while (millis() - start < timeout) { // Lặp trong thời gian timeout
    while (sim.available()) { // Nếu có dữ liệu từ SIM
      resp += (char)sim.read(); // Đọc và thêm vào chuỗi
    }
    if (resp.indexOf("OK") != -1 || resp.indexOf("ERROR") != -1 || resp.indexOf("+CMQ") != -1) // Kiểm tra nếu có OK, ERROR hoặc +CMQ
      break; // Thoát vòng lặp nếu có
    delay(10); // Chờ 10ms
  }
  if (resp.length()) Serial.println("<<< " + resp); // In phản hồi nếu có
  return resp; // Trả về phản hồi
}
String sendAT(const String& cmd, uint32_t timeout = 5000) { // Hàm gửi lệnh AT và chờ phản hồi
  Serial.println(">> " + cmd); // In lệnh gửi
  sim.println(cmd); // Gửi lệnh đến SIM
  return waitResponse(timeout); // Chờ và trả về phản hồi
}
bool initSIM() { // Hàm khởi tạo SIM
  sendAT("AT"); // Kiểm tra kết nối
  sendAT("ATE0"); // Tắt echo
  sendAT("AT+CMEE=2"); // Bật báo lỗi chi tiết
  sendAT("AT+CFUN=1", 3000); // Bật full functionality
  delay(2000); // Chờ 2 giây
  String simStatus = sendAT("AT+CPIN?", 2000); // Kiểm tra SIM PIN
  if (!simStatus.indexOf("OK")) { // Nếu không có OK
    Serial.println(" SIM chưa sẵn sàng!"); // In lỗi
    return false; // Thất bại
  }
  for (int i = 0; i < 10; i++) { // Thử đăng ký mạng tối đa 10 lần
    String reg = sendAT("AT+CEREG?", 2000); // Kiểm tra đăng ký mạng
    if (reg.indexOf(",1") != -1 || reg.indexOf(",5") != -1) { // Nếu đăng ký thành công
      Serial.println(" Đã đăng ký mạng NB-IoT"); // In thành công
      break; // Thoát vòng lặp
    }
    Serial.println(" Đang chờ đăng ký mạng..."); // In chờ
    delay(2000); // Chờ 2 giây
  }
  sendAT(String("AT+CGDCONT=5,\"IP\",\"") + APN + "\""); // Thiết lập PDP context
  String cgatt = sendAT("AT+CGATT?"); // Kiểm tra attach GPRS
  String cgact = sendAT("AT+CGACT?"); // Kiểm tra activate PDP
  Serial.println(" Gắn mạng: " + cgatt); // In trạng thái attach
  Serial.println(" PDP context: " + cgact); // In trạng thái PDP
  if (cgatt.indexOf("+CGATT: 1") == -1 || cgact.indexOf("+CGACT: 1,1") == -1) { // Nếu không attach hoặc activate
    Serial.println(" Không gắn được mạng!"); // In lỗi
    return false; // Thất bại
  }
  String pdp = sendAT("AT+CGCONTRDP", 3000); // Kiểm tra PDP details
  if (pdp.indexOf("0.0.0.0") != -1 || pdp.indexOf("ERROR") != -1) { // Nếu PDP không hoạt động
    Serial.println(" PDP chưa hoạt động!"); // In lỗi
    return false; // Thất bại
  }
  Serial.println(" PDP đã sẵn sàng!"); // In thành công
  return true; // Thành công
}
// ================== MQTT ==================
bool mqttConnect() { // Hàm kết nối MQTT
  Serial.println(" Bắt đầu kết nối MQTT (CMQ mode)..."); // In bắt đầu kết nối
  // Bật CMQTSYNC
  String syncResp = sendAT("AT+CMQTSYNC=1", 2000); // Bật chế độ sync MQTT
  if (syncResp.indexOf("OK") == -1) // Nếu thất bại
    Serial.println(" Không thể bật chế độ CMQTSYNC. Tiếp tục thử..."); // In cảnh báo
  else
    Serial.println(" Đã bật chế độ đồng bộ MQTT (CMQTSYNC=1)"); // In thành công
  //  Đảm bảo PDP sẵn sàng thật sự
  String pdp = sendAT("AT+CGCONTRDP", 5000); // Kiểm tra PDP lại
  if (pdp.indexOf("0.0.0.0") != -1 || pdp.indexOf("ERROR") != -1) { // Nếu chưa ổn định
    Serial.println(" PDP chưa ổn định, chờ 3s rồi thử lại..."); // In cảnh báo
    delay(3000); // Chờ 3 giây
  }
  delay(3000); //  Cho module ổn định socket NB-IoT
  // ===== CMQNEW với retry =====
  const int MAX_NEW_RETRY = 10; // Số lần thử tối đa cho CMQNEW
  //String cmd = String("AT+CMQNEW=\"") + MQTT_SERVER + "\"," + MQTT_PORT + ",12000,1024"; // Lệnh tạo kết nối MQTT mới
  String cmd = String("AT+CMQNEW=\"") + mqtt_server + "\"," + String(mqtt_port) + ",12000,1024";
  String resp; // Biến lưu phản hồi
  int newRetry = 0; // Biến đếm retry
  while (newRetry < MAX_NEW_RETRY) { // Lặp thử lại
    resp = sendAT(cmd, 8000); // Gửi lệnh
    if (resp.indexOf("OK") != -1 || resp.indexOf("+CMQNEW:") != -1) { // Nếu thành công
      Serial.println(" CMQNEW OK"); // In thành công
      break; // Thoát
    } else {
      Serial.printf(" CMQNEW thất bại! Thử lại %d/%d sau 3s...\n", newRetry + 1, MAX_NEW_RETRY); // In thất bại
      newRetry++; // Tăng retry
      delay(3000); // Chờ 3 giây
    }
  }
  if (newRetry >= MAX_NEW_RETRY) { // Nếu hết retry
    Serial.println(" CMQNEW thất bại nhiều lần → dừng kết nối MQTT!"); // In dừng
    return false; // Thất bại
  }
  // ===== CMQCON với retry =====
  const int MAX_CON_RETRY = 10; // Số lần thử tối đa cho CMQCON
  // String conCmd = String("AT+CMQCON=0,3,\"") + MQTT_CLIENT_ID +
  //                 "\",120,1,0,\"" + MQTT_USERNAME + "\",\"\""; // Lệnh kết nối MQTT
  String conCmd = String("AT+CMQCON=0,3,\"") + mqtt_client_id +
                  "\",120,1,0,\"" + mqtt_username + "\",\"" + mqtt_password + "\"";
  int conRetry = 0; // Biến đếm retry
  while (conRetry < MAX_CON_RETRY) { // Lặp thử lại
    resp = sendAT(conCmd, 10000); // Gửi lệnh
    if (resp.indexOf("OK") != -1) { // Nếu thành công
      Serial.println(" CMQCON OK (đã kết nối broker)"); // In thành công
      break; // Thoát
    } else {
      Serial.printf(" CMQCON thất bại! Thử lại %d/%d sau 3s...\n", conRetry + 1, MAX_CON_RETRY); // In thất bại
      sendAT("AT+CMQDISCON=0", 2000); // Reset kết nối trước khi thử lại
      conRetry++; // Tăng retry
      delay(3000); // Chờ 3 giây
    }
  }
  if (conRetry >= MAX_CON_RETRY) { // Nếu hết retry
    Serial.println(" CMQCON thất bại nhiều lần → dừng kết nối MQTT!"); // In dừng
    return false; // Thất bại
  }
  mqttSubscribed = false; // Reset cờ subscribe
  return true; // Thành công
}
bool mqttSubscribe(const String& topic) { // Hàm subscribe topic MQTT
  Serial.println(" Đăng ký topic..."); // In bắt đầu subscribe
  const int MAX_SUB_RETRY = 5; // Số lần thử tối đa
  int subRetry = 0; // Biến đếm retry
  String cmd = String("AT+CMQSUB=0,\"") + topic + "\",1"; // Lệnh subscribe
  String resp; // Biến lưu phản hồi
  while (subRetry < MAX_SUB_RETRY) { // Lặp thử lại
    resp = sendAT(cmd, 8000); // Gửi lệnh
    if (resp.indexOf("OK") != -1) { // Nếu thành công
      Serial.println(" Đăng ký topic thành công!"); // In thành công
      mqttSubscribed = true; // Đặt cờ subscribe
      return true; // Thành công
    }
    Serial.printf(" Đăng ký topic thất bại! Thử lại %d/%d sau 2s...\n", subRetry + 1, MAX_SUB_RETRY); // In thất bại
    Serial.println(resp); // giữ nguyên debug gốc để dễ theo dõi phản hồi module
    subRetry++; // Tăng retry
    delay(2000); // Chờ 2 giây
  }
  Serial.println(" Đăng ký topic thất bại nhiều lần → dừng đăng ký!"); // In dừng
  mqttSubscribed = false; // Reset cờ
  return false; // Thất bại
}
// ================== MQTT RESET SESSION ==================
bool mqttResetSession(uint8_t retries = 5) { // Hàm reset session MQTT
  for (uint8_t i = 1; i <= retries; i++) { // Lặp số lần thử
    Serial.printf(" (%d/%d) Đóng kết nối MQTT...\n", i, retries); // In tiến trình
    String resp1 = sendAT("AT+CMQDISCON=0", 2000); // Ngắt kết nối
    String resp2 = sendAT("AT+CMQDEL=0", 2000); // Xóa session
    // Kiểm tra xem cả hai lệnh đều OK
    if (resp1.indexOf("OK") != -1 || resp2.indexOf("OK") != -1) { // Nếu ít nhất một lệnh OK
      Serial.println(" Đã đóng và xóa session MQTT cũ thành công!"); // In thành công
      return true; // Thành công
    }
    Serial.println(" Đóng/Xóa session thất bại, thử lại sau 2s..."); // In thất bại
    delay(2000); // Chờ 2 giây
  }
  Serial.println(" Không thể reset session MQTT sau nhiều lần thử!"); // In lỗi cuối
  return false; // Thất bại
}
// ================== MQTT PUBLISH LARGE ==================
bool mqttPublishLarge(const String& topic, const String& payload) { // Hàm publish payload lớn theo chunk
  const size_t CHUNK_SIZE = 500; // Kích thước mỗi chunk (500 bytes)
  Serial.println(" Bắt đầu gửi payload lớn theo từng phần..."); // In bắt đầu
  size_t i = 0; // Chỉ số bắt đầu payload
  int retryCount = 0; // Đếm retry cho chunk
  const int MAX_RETRY = 5; // Thử lại tối đa 5 lần cho mỗi chunk
  while (i < payload.length()) { // Lặp qua toàn bộ payload
    String chunk = payload.substring(i, min(i + CHUNK_SIZE, payload.length())); // Lấy chunk
    // Chuyển chunk sang HEX
    String hex; // Chuỗi HEX
    const char hexChars[] = "0123456789ABCDEF"; // Bảng ký tự HEX
    for (size_t j = 0; j < chunk.length(); j++) { // Chuyển từng byte sang HEX
      uint8_t c = chunk[j]; // Lấy byte
      hex += hexChars[c >> 4]; // Phần cao
      hex += hexChars[c & 0xF]; // Phần thấp
    }
    String cmd = String("AT+CMQPUB=0,\"") + topic + "\",0,0,0," +
                 String(chunk.length() * 2) + ",\"" + hex + "\""; // Lệnh publish chunk HEX
    Serial.printf(" Gửi chunk %u / %u (size=%u bytes)\n",
                  (unsigned)(i / CHUNK_SIZE + 1),
                  (unsigned)((payload.length() + CHUNK_SIZE - 1) / CHUNK_SIZE),
                  (unsigned)chunk.length()); // In thông tin chunk
    String resp = sendAT(cmd, 8000); // Gửi lệnh
    // Nếu lỗi khi gửi chunk
    if (resp.indexOf("OK") == -1 && resp.indexOf("+CMQPUB:") == -1) { // Nếu không OK hoặc +CMQPUB
      Serial.printf(" Lỗi khi gửi chunk tại byte %u → thử lại (lần %d/%d)\n",
                    (unsigned)i, retryCount + 1, MAX_RETRY); // In lỗi
      retryCount++; // Tăng retry
      if (retryCount >= MAX_RETRY) { // Nếu hết retry
        Serial.println(" Gửi lại nhiều lần vẫn lỗi → dừng gửi tiếp!"); // In dừng
        return false; // Thất bại
      }
      delay(2000); // chờ ngắn trước khi thử lại
      continue; // thử lại chunk hiện tại
    }
    // Chunk gửi thành công → reset retry và chuyển tiếp chunk tiếp theo
    retryCount = 0; // Reset retry
    i += CHUNK_SIZE; // Chuyển đến chunk tiếp
    delay(800); // Chờ 800ms giữa các chunk
  }
  Serial.println(" Gửi toàn bộ ảnh hoàn tất!"); // In hoàn tất
  return true; // Thành công
}
// ================== CAPTURE + PAYLOAD ==================
String makePayload() { // Hàm chụp ảnh và tạo payload JSON
  Serial.println(" Capturing image..."); // In bắt đầu chụp
  digitalWrite(FLASH_LED, HIGH);
  delay(200);
  camera_fb_t* fb = esp_camera_fb_get(); // Lấy frame buffer từ camera
  if (!fb) { // Nếu thất bại
    Serial.println(" Camera capture failed!"); // In lỗi
    return "{}"; // Trả về JSON rỗng
  }
  else {
    //  Buzzer kêu “bíp bíp” ba lần
    for (int i = 0; i < 3; i++) { // Phát buzzer 3 lần
      beep(2000, 500); // 2kHz trong 0.5s
      delay(500); // Chờ giữa các beep
    }
  }
  Serial.printf(" Image captured, size: %d bytes\n", fb->len); // In kích thước ảnh
  String hexImg; // Chuỗi HEX
  const char hexChars[] = "0123456789ABCDEF"; // Bảng ký tự HEX
  for (size_t j = 0; j < fb->len; j++) { // Chuyển từng byte sang HEX
    uint8_t c = fb->buf[j]; // Lấy byte
    hexImg += hexChars[c >> 4]; // Phần cao
    hexImg += hexChars[c & 0xF]; // Phần thấp
  }
  esp_camera_fb_return(fb); // Trả frame buffer
  digitalWrite(FLASH_LED, LOW);
  String json = "{\"image\":\"" + hexImg + "\"}"; // Tạo JSON với ảnh hex
  return json; // Trả về JSON
}
long time_diff_seconds(Time t2, Time t1) {
  long hdiff = t2.hour - t1.hour;
  long mdiff = t2.minute - t1.minute;
  long sdiff = t2.second - t1.second;
  return hdiff * 3600 + mdiff * 60 + sdiff;
}
// ================== SETUP ==================
String payload; // Biến lưu payload ảnh
unsigned long afterPhotoTime = 0; // Biến lưu thời gian sau khi chụp ảnh
void ButtonTask(void *pvParameters) {
  for (;;) {
    int buttonState = digitalRead(USER_BUTTON_PIN); // Đọc trạng thái nút
    if (buttonState == LOW && !isDisplayOn) { // Nếu nút nhấn và màn hình tắt
      Serial.println("Button pressed → Bật OLED + buzzer"); // In thông báo
      isDisplayOn = true; // Bật cờ màn hình
      lastDisplayTime = millis(); // Lưu thời gian bật
      float usbVoltage = readUSBVoltage();

    if(usbVoltage > 3.0)
    {
      showBatteryScreen(0);
    }
    else
    {
      float voltage = readBatteryVoltage();

      int batteryLevel = voltageToPercent(voltage);

      showBatteryScreen(batteryLevel);
    }
    }
    if (isDisplayOn && millis() - lastDisplayTime > DISPLAY_TIMEOUT) { // Nếu hết timeout
      Serial.println("Tắt OLED sau 5s"); // In tắt
      turnOffDisplay(); // Tắt màn hình
    }
    vTaskDelay(pdMS_TO_TICKS(20)); // Chờ 20ms
  }
}
void MainTask(void *pvParameters) {
  camReady = false; // Reset cờ camera
  simReady = false; // Reset cờ SIM
  while (!camReady) { // Lặp đến khi camera sẵn sàng
    if (initCamera()) { // Thử khởi tạo camera
      camReady = true; // Đặt cờ sẵn sàng
    } else {
      Serial.println("❌ Không thể khởi tạo camera! Thử lại sau 2 giây..."); // In lỗi
      vTaskDelay(pdMS_TO_TICKS(2000)); // chờ trước khi thử lại
    }
  }
  vTaskDelay(pdMS_TO_TICKS(30000));
  payload = makePayload(); // Chụp ảnh và tạo payload ngay sau khi khởi tạo camera
  afterPhotoTime = millis(); // Lưu thời gian sau chụp
  lastSend = afterPhotoTime - SEND_INTERVAL_MS - 1; // Đặt lastSend để kích hoạt gửi ngay
  sim.begin(SIM_BAUD, SERIAL_8N1, SIM_RX_PIN, SIM_TX_PIN); // Khởi tạo Serial cho SIM
  vTaskDelay(pdMS_TO_TICKS(1000)); // Chờ 1 giây
  while (!simReady) { // Lặp đến khi SIM sẵn sàng
    if (initSIM()) { // Thử khởi tạo SIM
      simReady = true; // Đặt cờ sẵn sàng
    } else {
      Serial.println(" Lỗi khởi tạo SIM! Thử lại sau 2 giây..."); // In lỗi
      vTaskDelay(pdMS_TO_TICKS(2000)); // chờ trước khi thử lại
    }
  }
  while(!mqttConnect()) { // Lặp đến khi kết nối MQTT thành công
    Serial.println(" Kết nối MQTT thất bại! Thử lại sau 10 giây..."); // In lỗi
    vTaskDelay(pdMS_TO_TICKS(10000)); // Chờ 10 giây
  }
  while (!mqttSubscribe(mqtt_topic)) { // Lặp đến khi subscribe thành công
    Serial.println(" Thử đăng ký topic lại sau 10 giây..."); // In thử lại
    vTaskDelay(pdMS_TO_TICKS(10000)); // Chờ 10 giây
  }
  for (;;) {
    if (camReady && simReady && mqttSubscribed && (millis() - lastSend > SEND_INTERVAL_MS)) { // Kiểm tra điều kiện gửi
      Serial.println(" " + payload); // In payload
      while (!mqttPublishLarge(mqtt_topic, payload)) { // Lặp đến khi publish thành công
        Serial.println(" Publish lỗi, thử reconnect..."); // In lỗi
        mqttConnect(); // Thử reconnect
        while (!mqttSubscribe(mqtt_topic)) { // Thử subscribe lại
          Serial.println(" Thử đăng ký topic lại sau 10 giây..."); // In thử lại
          vTaskDelay(pdMS_TO_TICKS(10000)); // Chờ 10 giây
        }
      }
      lastSend = millis(); // Cập nhật thời gian gửi cuối
      unsigned long endSend = millis(); // Lấy thời gian kết thúc gửi
      unsigned long t_process = endSend - afterPhotoTime; // Tính thời gian xử lý sau chụp
      unsigned long t_boot_to_photo = afterPhotoTime; // Tính thời gian từ boot đến sau chụp
      unsigned long total_awake = t_process + t_boot_to_photo;
      unsigned long sleepTime = (total_awake < SEND_INTERVAL_MS) ? (SEND_INTERVAL_MS - total_awake) : 1000UL; // Tính thời gian ngủ = 5p - total_awake
      // Reset session MQTT trước khi ngủ
      while (!mqttResetSession()) { // Lặp đến khi reset thành công
        Serial.println(" Không thể reset session MQTT, thử lại sau 5s..."); // In lỗi
        vTaskDelay(pdMS_TO_TICKS(5000)); // Chờ 5 giây
      }
      for (int i = 0; i < 3; i++) { beep(2000, 500); vTaskDelay(pdMS_TO_TICKS(500)); } // beep trước khi ngủ
      //  Deep Sleep sau khi gửi xong
      Serial.println(" Đã sử dụng hết " + String( total_awake/ 1000) + " giây...");
      Serial.println(" Đi vào Deep Sleep trong " + String(sleepTime / 1000) + " giây..."); // In đi ngủ
      sleepStartTime = rtc.getTime();
      sleepDurationMs = sleepTime;
      rtc_gpio_pullup_en(GPIO_NUM_2);
      rtc_gpio_pulldown_dis(GPIO_NUM_2);
      esp_sleep_enable_ext0_wakeup(GPIO_NUM_2, 0);
      esp_sleep_enable_timer_wakeup(sleepTime * 1000ULL); // Thiết lập wakeup timer (microseconds)
      Serial.flush(); // Flush Serial
      esp_deep_sleep_start(); // Bắt đầu deep sleep
    }
    // Xử lý mất kết nối MQTT
    if (sim.available()) { // Nếu có dữ liệu từ SIM
      String line = sim.readString(); // Đọc dòng
      if (line.indexOf("+CMQDISCON") != -1) { // Nếu mất kết nối MQTT
        Serial.println(" MQTT bị ngắt! Reconnect..."); // In cảnh báo
        while(!mqttConnect()) { // Thử reconnect
          Serial.println(" Kết nối MQTT thất bại! Thử lại sau 10 giây..."); // In lỗi
          vTaskDelay(pdMS_TO_TICKS(10000)); // Chờ
        }
        while (!mqttSubscribe(mqtt_topic)) { // Thử subscribe lại
          Serial.println(" Thử đăng ký topic lại sau 10 giây..."); // In thử lại
          vTaskDelay(pdMS_TO_TICKS(10000)); // Chờ
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(20)); // Chờ 20ms mỗi loop
  }
}
void loadConfig() {
  preferences.begin("config", false);
  mqtt_server = preferences.getString("mqtt_server", "");
  mqtt_port = preferences.getInt("mqtt_port", 0);
  mqtt_client_id = preferences.getString("client_id", "");
  mqtt_username = preferences.getString("username", "");
  mqtt_password = preferences.getString("password", "");
  mqtt_topic = preferences.getString("topic", "");
  preferences.end();
}
const char* html_page = R"rawliteral(
<!DOCTYPE html>
<html lang="vi">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Cấu Hình Đồng Hồ Nước NB-IoT</title>
  <style>
    :root {
      --primary-color: #0056b3;
      --primary-hover: #004494;
      --bg-gradient-1: #e0eafc;
      --bg-gradient-2: #cfdef3;
      --text-color: #333;
    }
    body { 
      font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; 
      margin: 0; 
      padding: 0; 
      background: linear-gradient(135deg, var(--bg-gradient-1) 0%, var(--bg-gradient-2) 100%); 
      color: var(--text-color);
      min-height: 100vh;
      display: flex;
      justify-content: center;
      align-items: center;
    }
    .container { 
      background: #ffffff; 
      width: 100%; 
      max-width: 420px; 
      margin: 20px;
      padding: 30px 25px; 
      border-radius: 12px; 
      box-shadow: 0 10px 30px rgba(0,0,0,0.1); 
      box-sizing: border-box;
    }
    .header {
      text-align: center;
      margin-bottom: 25px;
    }
    .logo {
      width: 70px;
      height: 70px;
      margin: 0 auto 10px auto;
      background: #f0f4f8;
      border-radius: 50%;
      display: flex;
      justify-content: center;
      align-items: center;
      box-shadow: inset 0 2px 5px rgba(0,0,0,0.05);
    }
    .logo svg {
      width: 40px;
      height: 40px;
      fill: var(--primary-color);
    }
    h2 { 
      margin: 0; 
      color: var(--primary-color); 
      font-size: 22px;
      font-weight: 600;
    }
    p.subtitle {
      margin: 5px 0 0 0;
      font-size: 13px;
      color: #666;
    }
    .form-group {
      margin-bottom: 15px;
    }
    label { 
      display: block; 
      font-size: 14px; 
      font-weight: 600; 
      margin-bottom: 6px; 
      color: #555;
    }
    input[type=text], input[type=number], input[type=password] { 
      width: 100%; 
      padding: 12px; 
      box-sizing: border-box; 
      border: 1px solid #ced4da; 
      border-radius: 6px; 
      font-size: 14px; 
      transition: all 0.3s ease;
      background-color: #fcfcfc;
    }
    input[type=text]:focus, input[type=number]:focus, input[type=password]:focus {
      border-color: var(--primary-color);
      outline: none;
      background-color: #fff;
      box-shadow: 0 0 0 3px rgba(0, 86, 179, 0.15);
    }
    input[type=submit] { 
      background-color: var(--primary-color); 
      color: white; 
      padding: 14px; 
      border: none; 
      width: 100%; 
      border-radius: 6px; 
      font-size: 16px; 
      font-weight: 600;
      cursor: pointer; 
      margin-top: 10px;
      transition: background-color 0.3s ease, transform 0.1s ease; 
    }
    input[type=submit]:hover { 
      background-color: var(--primary-hover); 
    }
    input[type=submit]:active {
      transform: scale(0.98);
    }
    .footer {
      text-align: center;
      margin-top: 20px;
      font-size: 12px;
      color: #888;
    }
  </style>
</head>
<body>
  <div class="container">
    <div class="header">
      <div class="logo">
        <svg viewBox="0 0 24 24" xmlns="http://www.w3.org/2000/svg">
          <path d="M12,20A6,6 0 0,1 6,14C6,10 12,3.25 12,3.25C12,3.25 18,10 18,14A6,6 0 0,1 12,20M12,4.82C10.74,6.77 7.5,10.66 7.5,14A4.5,4.5 0 0,0 12,18.5A4.5,4.5 0 0,0 16.5,14C16.5,10.66 13.26,6.77 12,4.82M4.08,10.42C4.16,10.09 4.4,9.84 4.73,9.76L6.68,9.26C7,9.18 7.35,9.36 7.43,9.68C7.5,10 7.35,10.35 7.03,10.43L5.07,10.93C4.75,11.03 4.4,10.84 4.33,10.5C4.26,10.5 4.19,10.5 4.08,10.42M19.92,10.42C19.84,10.09 19.6,9.84 19.27,9.76L17.32,9.26C17,9.18 16.65,9.36 16.57,9.68C16.5,10 16.65,10.35 16.97,10.43L18.93,10.93C19.25,11.03 19.6,10.84 19.67,10.5C19.74,10.5 19.81,10.5 19.92,10.42M2.5,14A9.5,9.5 0 0,0 12,23.5A9.5,9.5 0 0,0 21.5,14C21.5,13.62 21.19,13.31 20.81,13.31C20.43,13.31 20.12,13.62 20.12,14A8.12,8.12 0 0,1 12,22.12A8.12,8.12 0 0,1 3.88,14C3.88,13.62 3.57,13.31 3.19,13.31C2.81,13.31 2.5,13.62 2.5,14Z" />
        </svg>
      </div>
      <h2>Cấu Hình Thiết Bị</h2>
      <p class="subtitle">Smart Water Meter - NB-IoT</p>
    </div>

    <form action="/save" method="POST">
      <div class="form-group">
        <label>MQTT Server:</label>
        <input type="text" name="server" value="%s" placeholder="Ví dụ: broker.hivemq.com" required>
      </div>
      
      <div class="form-group">
        <label>MQTT Port:</label>
        <input type="number" name="port" value="%s" placeholder="Ví dụ: 1883" required>
      </div>

      <div class="form-group">
        <label>Client ID:</label>
        <input type="text" name="client_id" value="%s" placeholder="Nhập Client ID của thiết bị" required>
      </div>

      <div class="form-group">
        <label>Username:</label>
        <input type="text" name="username" value="%s" placeholder="Bỏ trống nếu không có">
      </div>

      <div class="form-group">
        <label>Password:</label>
        <input type="password" name="password" value="%s" placeholder="Bỏ trống nếu không có">
      </div>

      <div class="form-group">
        <label>Topic Gửi Data:</label>
        <input type="text" name="topic" value="%s" placeholder="Ví dụ: watermeter/data" required>
      </div>

      <input type="submit" value="Lưu Cấu Hình & Khởi Động Lại">
    </form>
    
    <div class="footer">
      &copy; 2026 AI OCR Water Meter Project
    </div>
  </div>
</body>
</html>
)rawliteral";
void handleRoot() {
  char *buffer = (char *)malloc(8192);
  if (buffer == NULL) {
    server.send(500, "text/plain", "Loi het bo nho RAM ESP32!");
    return;
  }

  // MẸO Ở ĐÂY: Nếu port là 0 thì biến thành chuỗi rỗng "", ngược lại thì chuyển thành chuỗi số
  String portStr = (mqtt_port == 0) ? "" : String(mqtt_port);

  // Điền dữ liệu an toàn
  snprintf(buffer, 8192, html_page, 
           mqtt_server.c_str(), 
           portStr.c_str(),     // <--- Đã thay thế biến mqtt_port bằng chuỗi portStr
           mqtt_client_id.c_str(), 
           mqtt_username.c_str(), 
           mqtt_password.c_str(), 
           mqtt_topic.c_str());
           
  server.send(200, "text/html; charset=UTF-8", buffer);
  free(buffer); 
}

void handleSave() {
  if (server.hasArg("server") && server.hasArg("port")) {
    preferences.begin("config", false);
    preferences.putString("mqtt_server", server.arg("server"));
    preferences.putInt("mqtt_port", server.arg("port").toInt());
    preferences.putString("client_id", server.arg("client_id"));
    preferences.putString("username", server.arg("username"));
    preferences.putString("password", server.arg("password"));
    preferences.putString("topic", server.arg("topic"));
    preferences.end();
    
    server.send(200, "text/html; charset=UTF-8", "<!DOCTYPE html><html><head><meta charset='UTF-8'></head><body><h2 style='text-align:center; color:green; margin-top:50px;'>Đã lưu cấu hình! Hệ thống đang khởi động lại...</h2></body></html>");
    delay(2000);
    ESP.restart(); // Khởi động lại vi điều khiển
  }
}

// Chuyển hướng mọi request sai địa chỉ về trang chủ (Captive Portal mechanism)
void handleNotFound() {
  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
}

void startCaptivePortal() {
  isAPMode = true;
  Serial.println("\n[!] VÀO CHẾ ĐỘ CẤU HÌNH AP");
  
  // Phát WiFi
  WiFi.mode(WIFI_AP);
  WiFi.softAP("ESP32_Water_Meter", "12345678"); 
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  // Khởi chạy DNS Server để bẻ lái mọi traffic về ESP32
  dnsServer.start(DNS_PORT, "*", IP);

  // Cấu hình các đường dẫn Web
  server.on("/", HTTP_GET, handleRoot);
  server.on("/save", HTTP_POST, handleSave);
  server.onNotFound(handleNotFound); // Phục vụ Captive Portal
  server.begin();
  
  // Hiển thị lên OLED cho kỹ thuật viên biết
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(5, 20, "Access Point Mode:");
  u8g2.drawStr(5, 40, "ESP32_Water_Meter");
  u8g2.drawStr(5, 60, "IP: 192.168.4.1");
  u8g2.sendBuffer();
}
void setup() { 
  // 1. MỞ SERIAL VÀ I2C ĐẦU TIÊN (Cho OLED và RTC)
  Serial.begin(115200);
  Wire.begin(5,6); // Dùng chân 5, 6 cho OLED và PCF8563
  Wire.setClock(100000);
  i2cMutex = xSemaphoreCreateMutex();

  // 2. KHỞI TẠO RTC SAU KHI I2C ĐÃ MỞ
  rtc.init();
  Time currentTime = rtc.getTime();
  
  esp_sleep_wakeup_cause_t wakeup_cause = esp_sleep_get_wakeup_cause();

  if (wakeup_cause == ESP_SLEEP_WAKEUP_EXT0) {
    // =========================================================
    // NHÁNH 1: THỨC DẬY TỪ NÚT NHẤN (CHỈ ĐỂ XEM PIN)
    // =========================================================
    Serial.println("\n=== Woken by button ===");
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
    
    u8g2.begin();
    u8g2.setContrast(255);
    
    float usbVoltage = readUSBVoltage();
    if(usbVoltage > 3.0) {
      showBatteryScreen(0);
    } else {
      float voltage = readBatteryVoltage();
      int batteryLevel = voltageToPercent(voltage);
      showBatteryScreen(batteryLevel);
    }
    
    isDisplayOn = true;
    unsigned long startWait = millis();
    while (millis() - startWait < DISPLAY_TIMEOUT) {
      delay(10);
    }
    turnOffDisplay();
    
    long passed_sec = time_diff_seconds(currentTime, sleepStartTime);
    unsigned long passed_ms = passed_sec * 1000UL;
    unsigned long remaining_ms = (passed_ms < sleepDurationMs) ? sleepDurationMs - passed_ms : 1000UL;
    
    rtc_gpio_pullup_en(GPIO_NUM_2);
    rtc_gpio_pulldown_dis(GPIO_NUM_2);
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_2, 0);
    esp_sleep_enable_timer_wakeup(remaining_ms * 1000ULL);
    esp_deep_sleep_start();
    
  } else {
    // =========================================================
    // NHÁNH 2: KHỞI ĐỘNG BÌNH THƯỜNG / VÀO CHẾ ĐỘ AP
    // =========================================================
    pinMode(USER_BUTTON_PIN, INPUT_PULLUP);
    
    // Đọc cấu hình từ Flash
    loadConfig();

    // CHO NGƯỜI DÙNG 2 GIÂY ĐỂ BẤM NÚT
    Serial.println("\n[INFO] Đang chờ 2 giây... Nhấn GIỮ nút ngay bây giờ để vào cấu hình AP!");
    delay(2000); 

    // Kiểm tra xem nút có đang bị giữ không
    if (digitalRead(USER_BUTTON_PIN) == LOW) {
      Serial.println("\n[!] Đã phát hiện nút nhấn, hãy giữ thêm 3 giây...");
      delay(3000); // Xác nhận cố tình giữ
      
      if (digitalRead(USER_BUTTON_PIN) == LOW) {
        // Bật OLED để báo hiệu đang ở chế độ AP (Dùng cấu hình I2C chân 5,6 ở trên cùng)
        u8g2.begin();
        u8g2.setContrast(255);
        
        startCaptivePortal(); // Chạy Web Server
        return; // Cắt đứt luồng setup, không chạy Camera và SIM
      }
    }

    // --- TIẾP TỤC QUÁ TRÌNH KHỞI ĐỘNG CHÍNH ---
    ++bootCount; 
    if(bootCount > 1) { 
      for (int i = 0; i < 2; i++) { beep(2000, 500); delay(500); } 
    }
    delay(1000); 
    
    Serial.println("\n=== ESP32S3 + SIM7020G MQTT (Send Image) ===");
    Serial.println("Boot number: " + String(bootCount)); 
    
    pinMode(BUZZER_PIN, OUTPUT); 
    digitalWrite(BUZZER_PIN, LOW); 
    analogSetPinAttenuation(BATTERY_PIN, ADC_11db);
    analogSetPinAttenuation(USB_PIN, ADC_11db);
    analogReadResolution(12);

    pinMode(BATTERY_PIN, INPUT);
    pinMode(USB_PIN, INPUT);
    pinMode(FLASH_LED, OUTPUT);
    
    u8g2.begin(); 
    u8g2.setContrast(255); 
    turnOffDisplay(); 
    
    // Đồng bộ RTC (Chỉ cần chạy 1 lần rồi có thể comment lại)
    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    rtc.stopClock(); 
    rtc.setYear(25); 
    rtc.setMonth(10); 
    rtc.setDay(29); 
    rtc.setHour(10); 
    rtc.setMinut(45); 
    rtc.setSecond(0); 
    rtc.startClock(); 
    xSemaphoreGive(i2cMutex);
    
    xTaskCreate(ButtonTask, "ButtonTask", 4096, NULL, 10, NULL); 
    xTaskCreate(MainTask, "MainTask", 8192, NULL, 1, NULL); 
  }
}
void loop() {
  if (isAPMode) {
    dnsServer.processNextRequest(); // Phản hồi DNS cho Captive Portal
    server.handleClient();          // Phản hồi HTTP
    delay(10);                      // Nhường CPU cho các tác vụ nền của WiFi
  } else {
    vTaskDelete(NULL); 
  }
}

  






