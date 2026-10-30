#include <Arduino.h>
#include "esp_camera.h"
#include <WiFi.h>
#include "esp_http_server.h"
#include "img_converters.h"

#define WIDTH 160
#define HEIGHT 120


// Region of Interest (ROI): Ignore ceiling lights and floor reflections
#define ROI_Y_START 15
#define ROI_Y_END   105


// Tuned for 200cm distance
#define MIN_SAT 50
#define MIN_VAL 45
#define HUE_TOL 10
#define MIN_PIXELS 10
#define CENTER_OF_FRAME  WIDTH/2.0f
#define ALMOST_CENTERED 0.08f

struct ColorDef {
  uint8_t maxHue;
  const char* name;
};

const ColorDef COLORS[] = {
  {15,  "RED"},     // 0 - 15: Lower Red + Reddish-Orange
  {38,  "YELLOW"},  // 16 - 38: Yellowish-Orange + Yellow
  {90,  "GREEN"},   // 39 - 90: Green
  {145, "BLUE"},    // 91 - 145: Blue
  {179, "RED"}      // 146 - 179: Upper Red
};

// Unpacks 16-bit RGB565 into 8-bit R, G, B using bit replication
inline void unpackRGB565(uint16_t p, uint8_t &r, uint8_t &g, uint8_t &b) {
  uint8_t r5 = (p >> 11) & 0x1F;
  uint8_t g6 = (p >> 5) & 0x3F;
  uint8_t b5 = p & 0x1F;
  r = (r5 << 3) | (r5 >> 2);
  g = (g6 << 2) | (g6 >> 4);
  b = (b5 << 3) | (b5 >> 2);
}

// Fast integer RGB to Hue (0-179), returns -1 if pixel is dull or dark
inline int rgbToHue(uint8_t r, uint8_t g, uint8_t b) {
  uint8_t mx = max(r, max(g, b));
  if (mx < MIN_VAL) return -1;

  uint8_t mn = min(r, min(g, b));
  uint8_t delta = mx - mn;
  if (delta == 0) return -1;

  int sat = ((int)delta * 255) / mx;
  if (sat < MIN_SAT) return -1;

  int h = 0;
  if (mx == r) h = 30 * ((int)g - (int)b) / delta;
  else if (mx == g) h = 60 + 30 * ((int)b - (int)r) / delta;
  else h = 120 + 30 * ((int)r - (int)g) / delta;

  if (h < 0) h += 180;
  return h % 180;
}
const int NUM_COLORS = sizeof(COLORS) / sizeof(COLORS[0]);
const char* getColorName(int hue) {
  for (int i = 0; i <NUM_COLORS ; i++) {
    if (hue <= COLORS[i].maxHue) return COLORS[i].name;
  }
  return "UNKNOWN";
}

// Single-pass scan inside the horizon ROI to find dominant color and steering error
bool detectFlagRealTime(uint8_t *buf, int &outHue, float &outError) {
  uint16_t histCount[180] = {0};
  uint32_t histSumX[180] = {0};
  int validPixels = 0;

  // Scan every pixel inside ROI
  for (int y = ROI_Y_START; y < ROI_Y_END; y++) {
    int rowOffset = y * WIDTH * 2;
    for (int x = 0; x < WIDTH; x++) {
      int idx = rowOffset + (x * 2);
      uint16_t p = ((uint16_t)buf[idx] << 8) | buf[idx + 1];
      uint8_t r, g, b;
      unpackRGB565(p, r, g, b);
      int h = rgbToHue(r, g, b);
      if (h >= 0) {
        histCount[h]++;
        histSumX[h] += x;
        validPixels++;
      }
    }
  }
  if (validPixels < MIN_PIXELS) 
  { Serial.printf("valid=%d", validPixels);
    return false;}

  // Find dominant hue peak
  int bestHue = -1, maxCount = 0;
  for (int h = 0; h < 180; h++) {
    int sum = 0;
    for (int k = -2; k <= 2; k++) sum += histCount[(h + k + 180) % 180]; // sum of votes for each hue in the hist
    if (sum > maxCount) {
      maxCount = sum;
      bestHue = h;
    }
  }
  if (maxCount < MIN_PIXELS) 
  { Serial.printf("valid=%d peak=%d  hue=%d\n", validPixels, maxCount,  bestHue);
    return false;}

  // Compute Center of Mass directly from histogram bins
  long totalX = 0;
  int totalMatched = 0;
  for (int k = -HUE_TOL; k <= HUE_TOL; k++) {
    int bin = (bestHue + k + 180) % 180;
    totalX += histSumX[bin];  // sum of all cols of the pixels of the hue range [besthue-k , besthue+k] 
    totalMatched += histCount[bin]; // total count of pixels of the hue range [besthue-k , besthue+k]
  }

  if (totalMatched < MIN_PIXELS) 
  { Serial.printf("valid=%d peak=%d matched=%d hue=%d\n", validPixels, maxCount, totalMatched, bestHue);
    return false;}

  outHue = bestHue;
  float centerX = (float)totalX / totalMatched;
  outError = (centerX - CENTER_OF_FRAME) / (CENTER_OF_FRAME);
  return true;
}

/////////////////////////////////////////////////

esp_err_t capture_handler(httpd_req_t *req) {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  int hue = -1;
  float error = 0.0f;
  bool found = detectFlagRealTime(fb->buf, hue, error);

  // Draw white horizontal lines at ROI_Y_START and ROI_Y_END
  for (int x = 0; x < WIDTH; x++) {
    int topIdx = (ROI_Y_START * WIDTH + x) * 2;
    int botIdx = (ROI_Y_END * WIDTH + x) * 2;
    fb->buf[topIdx] = 0xFF; fb->buf[topIdx + 1] = 0xFF;
    fb->buf[botIdx] = 0xFF; fb->buf[botIdx + 1] = 0xFF;
  }

  // If a flag is detected, draw a vertical green line at the detected center X
  if (found) {
    int centerX = (int)((error * CENTER_OF_FRAME) + CENTER_OF_FRAME);
    centerX = constrain(centerX, 0, WIDTH - 1);
    for (int y = ROI_Y_START; y < ROI_Y_END; y++) {
      int idx = (y * WIDTH + centerX) * 2;
      fb->buf[idx] = 0x07; fb->buf[idx + 1] = 0xE0; // Green in RGB565
    }
  }

  // Convert the RGB565 frame into a JPEG so your browser can display it
  uint8_t *jpg_buf = NULL;
  size_t jpg_len = 0;
  bool converted = fmt2jpg(fb->buf, fb->len, WIDTH, HEIGHT, PIXFORMAT_RGB565, 80, &jpg_buf, &jpg_len);
  esp_camera_fb_return(fb);

  if (!converted) {
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  esp_err_t res = httpd_resp_send(req, (const char *)jpg_buf, jpg_len);
  free(jpg_buf);
  return res;
}

void startDebugServer() {
  WiFi.softAP("ESP32-Cam-Debug", "12345678");

  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  httpd_handle_t server = NULL;
  if (httpd_start(&server, &config) == ESP_OK) {
    httpd_uri_t capture_uri = {
      .uri       = "/",
      .method    = HTTP_GET,
      .handler   = capture_handler,
      .user_ctx  = NULL
    };
    httpd_register_uri_handler(server, &capture_uri);
  }
}




////////////////////////////////////////






void initCamera() {
  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = 5; config.pin_d1 = 18; config.pin_d2 = 19; config.pin_d3 = 21;
  config.pin_d4 = 36; config.pin_d5 = 39; config.pin_d6 = 34; config.pin_d7 = 35;
  config.pin_xclk = 0; config.pin_pclk = 22; config.pin_vsync = 25; config.pin_href = 23;
  config.pin_sccb_sda = 26; config.pin_sccb_scl = 27; config.pin_pwdn = 32; config.pin_reset = -1;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_RGB565;
  config.frame_size = FRAMESIZE_QQVGA;
  config.fb_count = 1;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_DRAM;

  if (esp_camera_init(&config) != ESP_OK) {
    while (true) delay(1000);
  }

  sensor_t *s = esp_camera_sensor_get();
  s->set_whitebal(s, 1);
  s->set_awb_gain(s, 1);
  s->set_wb_mode(s, 0);
  s->set_saturation(s, 2);
  s->set_contrast(s, 1);
}

void setup() {
  Serial.begin(115200);
  initCamera();
  startDebugServer();
}


void loop() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return;

  int hue = -1;
  float error = 0.0f;

  if (detectFlagRealTime(fb->buf, hue, error)) {
    if (abs(error) < ALMOST_CENTERED) error = 0.0f;

      Serial.printf("1,%.2f,%s\n", error, getColorName(hue));

  } else {
      Serial.println("0,0.00,NONE");

  }

  esp_camera_fb_return(fb);
}



// void loop() {
//   camera_fb_t *fb = esp_camera_fb_get();
//   if (!fb) return;

//   int hue = -1;
//   float error = 0.0f;


//   if (detectFlagRealTime(fb->buf, hue, error)) {
//     if (abs(error) < ALMOST_CENTERED) error = 0.0f;
//       Serial.printf("1,%.2f,%s\n", error, getColorName(hue));
//   } else {
//     Serial.println("0,0.00,NONE");
//   }

//   esp_camera_fb_return(fb);
// }
