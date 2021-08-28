#include <driver/adc.h> // sleep adc
#include <esp_bt.h> // sleep bt
#include <esp_wifi.h> // sleep wifi
#include <esp_sleep.h> // sleep
#include <esp_camera.h> // camera
#include <SD_MMC.h> // SD
#include <soc/rtc_cntl_reg.h> // sleep RTC
#include <driver/rtc_io.h> // RTC
#include <soc/rtc_wdt.h> // Watchdog
#include <LittleFS.h> // Internal storage
#include <EEPROM.h> // Storage for config
#include <ESP32Time.h> // Used to set time in RTC
#include <ESPmDNS.h> // WiFi web server with DNS
#include <WiFi.h> // WiFi web server
#include <AsyncTCP.h> // Async TCP connection
#include <ESPAsyncWebServer.h> // Async web server

#define US_TO_S_FACTOR 1000000
#define TIMEZONE 3600 * 2
#define null nullptr

// Pin definition for CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

//#define RESET_CONFIG // uncomment to put default Config struct and CURRENT_EPOCH in EEPROM
#define CURRENT_EPOCH 1629880670 + 10 + TIMEZONE
#define WIFI_SSID "HiHiHi"
#define WIFI_PASSWORD "U5z^26Gi9B&un4J7"

typedef struct
{
  unsigned int timeSleepDay;
  unsigned int timeSleepNight;
  
  unsigned long epoch;
  
  unsigned int hourStartNight;
  unsigned int hourStopNight;
  
  unsigned int hourStartWifi;
  unsigned int hourStopWifi;
  unsigned int minuteModuloWifi;
  unsigned int minuteStartWifi;
  unsigned int minuteStopWifi;
  
  bool forceWifi;
} Config;

String outputHtml;
unsigned long lastPhotoTakenTime = 0;
unsigned long lastPhotoSize = 0;
String lastPhotoPath, tempPath;
bool isDoingSomething = false;
ESP32Time rtc;
AsyncWebServer server(80);

Config config = {
  .timeSleepDay = 50,
  .timeSleepNight = 3590,
  .epoch = CURRENT_EPOCH,
  .hourStartNight = 21,
  .hourStopNight = 6,
  .hourStartWifi = 8,
  .hourStopWifi = 10,
  .minuteModuloWifi = 30,
  .minuteStartWifi = 0,
  .minuteStopWifi = 10,
  .forceWifi = true
};

/*
 * Watchdog functions
 * I don't know if it works properly
 * 
 * https://stackoverflow.com/a/58116274/7088874
 * https://www.esp32.com/viewtopic.php?p=56880#p56880
 */
void disableWatchdog() {
  rtc_wdt_protect_off();      //Disable RTC WDT write protection
  rtc_wdt_disable();
}

void enableWatchdog() {
  rtc_wdt_enable();           //Start the RTC WDT timer
  rtc_wdt_protect_on();       //Enable RTC WDT write protection
}

void printConfig() {
  Serial.println(F("Config :"));
  Serial.print(F("\ttimeSleepDay : ")); Serial.println(config.timeSleepDay);
  Serial.print(F("\ttimeSleepNight : ")); Serial.println(config.timeSleepNight);
  Serial.print(F("\tepoch : ")); Serial.println(config.epoch);
  Serial.print(F("\thourStartNight : ")); Serial.println(config.hourStartNight);
  Serial.print(F("\thourStopNight : ")); Serial.println(config.hourStopNight);
  Serial.print(F("\thourStartWifi : ")); Serial.println(config.hourStartWifi);
  Serial.print(F("\thourStopWifi : ")); Serial.println(config.hourStopWifi);
  Serial.print(F("\tminuteModuloWifi : ")); Serial.println(config.minuteModuloWifi);
  Serial.print(F("\tminuteStartWifi : ")); Serial.println(config.minuteStartWifi);
  Serial.print(F("\tminuteStopWifi : ")); Serial.println(config.minuteStopWifi);
  Serial.print(F("\tforceWifi : ")); Serial.println(config.forceWifi);
}

void getConfig() {
  if (!EEPROM.begin(sizeof(Config))) {
    Serial.println(F("Failed to initialise EEPROM"));
  }

#ifdef RESET_CONFIG
  config.forceWifi = true;
  saveConfig();
#else
  Config savedConfig;
  EEPROM.get(0, savedConfig);

  if (savedConfig.epoch >= CURRENT_EPOCH) {
    config = savedConfig;
  }
#endif

  printConfig();
}

void saveConfig() {
  EEPROM.put(0, config);
  EEPROM.commit();
}

void goToSleep(unsigned int timeToSleep) {
  Serial.println(rtc.getTimeDate());
  Serial.print(F("Going to sleep now for : ")); Serial.println(timeToSleep);

  // Turns off the ESP32-CAM white on-board LED (flash) connected to GPIO 4
  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);
  rtc_gpio_hold_en(GPIO_NUM_4);
  
  SD_MMC.end();
  LittleFS.end();

  adc_power_release();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  btStop();
  esp_bt_controller_disable();
  
  esp_sleep_enable_timer_wakeup(timeToSleep * US_TO_S_FACTOR);

  delay(1000);
  Serial.flush();

  config.epoch = rtc.getEpoch() + timeToSleep + 1; // next startup
  saveConfig();
    
  esp_deep_sleep_start();
}

void cleanPhotosLittleFS(unsigned long sizeNeeded) {
  File root = LittleFS.open("/");
  if(!root) {
    Serial.println(F("Failed to open LittleFS root directory"));
    return;
  }
  
  unsigned long spaceAvailable = LittleFS.totalBytes() - LittleFS.usedBytes();
  bool shouldRemove = spaceAvailable <= sizeNeeded;
    
  while(shouldRemove) {
    Serial.print(F("LittleFS space available : ")); Serial.println(spaceAvailable);
    Serial.print(F("Should remove for size ")); Serial.println(sizeNeeded);
  
    File file = root.openNextFile();
    if (!file) {
      break;
    } else {
      while(file) {
        if(!file.isDirectory()) {
          tempPath.remove(0);
          tempPath += String("/") + file.name();
          Serial.print(F("Delete file from LittleFS : ")); Serial.print(tempPath); Serial.print(F(" for size ")); Serial.println(file.size());
          file.close();
          if (!LittleFS.remove(tempPath)) {
            Serial.println(F("Error during deletion"));
          } else {
            break;
          }
        }
        file = root.openNextFile();
        yield();
      }
    }
    
    spaceAvailable = LittleFS.totalBytes() - LittleFS.usedBytes();
    shouldRemove = spaceAvailable <= sizeNeeded;
  }
  
  root.close();
}

unsigned long takePhoto() {
  Serial.println(rtc.getTimeDate());
  Serial.println(F("Taking photo"));
  
  if (isDoingSomething) {
    Serial.println(F("Something already running"));
    return 0;
  }

  isDoingSomething = true;
  
  camera_fb_t *fb = esp_camera_fb_get();  
  if(!fb) {
    Serial.println(F("Camera capture failed"));
    goToSleep(config.timeSleepDay);
  }
  
  Serial.print(F("File size : ")); Serial.println(fb->len);

  lastPhotoPath.remove(0);
  lastPhotoPath += "/" + rtc.getTime("%Y-%m-%d_%H-%M-%S") + ".jpg";
  bool hasError = true;
 
  Serial.print(F("Picture file name: ")); Serial.println(lastPhotoPath.c_str());
  
  File file = SD_MMC.open(lastPhotoPath, FILE_WRITE);
  if(!file) {
    Serial.println(F("Failed to open SD file in writing mode"));
  } else {
    if (file.write(fb->buf, fb->len) != fb->len) {
      Serial.println(F("Failed to write on SD Card"));
    } else {
      Serial.println(F("Saved on SD"));
      hasError = false;
    }
    file.close();
  }
  yield();

  cleanPhotosLittleFS(fb->len);
  file = LittleFS.open(lastPhotoPath, FILE_WRITE);
  if(!file) {
    Serial.println(F("Failed to open LittleFS file in writing mode"));
  } else {
    if (file.write(fb->buf, fb->len) != fb->len) {
      Serial.println(F("Failed to write on LittleFS"));
    } else {
      Serial.println(F("Saved on LittleFS"));
      hasError = false;
    }
    file.close();
  }
  yield();

  if (!hasError) {
    lastPhotoTakenTime = rtc.getEpoch();
    lastPhotoSize = fb->len;
  }
  
  esp_camera_fb_return(fb);

  isDoingSomething = false;
  
  return lastPhotoSize;
}

void handleFileList(AsyncWebServerRequest *request) {
  Serial.print(F("GET ")); Serial.println(request->url());
  
  if (isDoingSomething) {
    Serial.println(F("Something already running"));
    request->send(503);
    return;
  }

  isDoingSomething = true;

  outputHtml.remove(0);
  
  outputHtml += "<html><head><meta charset=\"utf-8\">";

  outputHtml += "<script type=\"text/javascript\">function addtimestamp() { const link = document.getElementById('timeLink'); const now = Math.round(new Date().getTime() / 1000); link.setAttribute(\"href\", \"/time?epoch=\" + now); }</script>";
  
  outputHtml += "<title>HiHiHi Camera</title>";
  
  outputHtml += "</head><body><h1>HiHiHi Camera</h1>";

  outputHtml += "<h2>Actions</h2><a target=\"_blank\" href=\"/take\">Prendre une photo</a>&nbsp;<a target=\"\" href=\"/wifi\">Basculer forcage WiFi</a>&nbsp;<a onclick=\"addtimestamp()\" href=\"#\" id=\"timeLink\">R&eacute;gler l'heure</a>&nbsp;<a target=\"\" target=\"_blank\" href=\"/wget\">Liste photos seulement</a>&nbsp;<a target=\"\" href=\"/format\">Formatter LittleFS + SD</a><br /><br />";

  outputHtml += "<h2>Photos SD</h2>";
  unsigned int nbPhotos = 0;
  File root = SD_MMC.open("/");
  if(root) {
    File file = root.openNextFile();
    while(file) {
      if(!file.isDirectory() && file.size() > 0) {
        //outputHtml += String("<a target=\"_blank\" href=\"/photos_sd/") + file.name() + String("\">") + file.name() + String(" - ") + String(file.size()) + String(" - SD</a><br />");
        nbPhotos++;
      }
      file.close();
      file = root.openNextFile();
      yield();
    }
    root.close();
  
    outputHtml += "<br />Nombre de photos SD : " + String(nbPhotos);
  } else {
    Serial.println(F("Failed to open SD root directory"));
    outputHtml += "<br />Impossible d'ouvrir le stockage";
  }
  
  outputHtml += "<h2>Photos internes</h2>";
  nbPhotos = 0;
  root = LittleFS.open("/");
  if(root) {
    File file = root.openNextFile();
    while(file) {
      if(!file.isDirectory() && file.size() > 0) {
        outputHtml += String("<a target=\"_blank\" href=\"/photos_internal/") + file.name() + String("\">") + file.name() + String(" - ") + String(file.size()) + String(" - Interne</a><br />");
        nbPhotos++;
      }
      file.close();
      file = root.openNextFile();
      yield();
    }
    root.close();
  
    outputHtml += "<br />Nombre de photos internes : " + String(nbPhotos);
  } else {
    Serial.println(F("Failed to open LittleFS root directory"));
    outputHtml += "<br />Impossible d'ouvrir le stockage";
  }

  outputHtml += "<h2>Configuration</h2>";
  outputHtml += "<form method=\"POST\" action=\"/set\">";

  outputHtml += "<div><label for=\"timeSleepDay\">Pause journ&eacute;e (s)</label>&nbsp;<input type=\"number\" value=\"" + String(config.timeSleepDay) + "\" name=\"timeSleepDay\" id=\"timeSleepDay\" /></div><br />";
  outputHtml += "<div><label for=\"timeSleepNight\">Pause nuit (s)</label>&nbsp;<input type=\"number\" value=\"" + String(config.timeSleepNight) + "\" name=\"timeSleepNight\" id=\"timeSleepNight\" /></div><br />";
  outputHtml += "<div><label for=\"hourStartNight\">Heure d&eacute;but nuit (soir)</label>&nbsp;<input type=\"number\" value=\"" + String(config.hourStartNight) + "\" name=\"hourStartNight\" id=\"hourStartNight\" /></div><br />";
  outputHtml += "<div><label for=\"hourStopNight\">Heure fin nuit (matin)</label>&nbsp;<input type=\"number\" value=\"" + String(config.hourStopNight) + "\" name=\"hourStopNight\" id=\"hourStopNight\" /></div><br />";
  outputHtml += "<div><label for=\"hourStartWifi\">Heure d&eacute;but WiFi</label>&nbsp;<input type=\"number\" value=\"" + String(config.hourStartWifi) + "\" name=\"hourStartWifi\" id=\"hourStartWifi\" /></div><br />";
  outputHtml += "<div><label for=\"hourStopWifi\">Heure fin WiFi</label>&nbsp;<input type=\"number\" value=\"" + String(config.hourStopWifi) + "\" name=\"hourStopWifi\" id=\"hourStopWifi\" /></div><br />";
  outputHtml += "<div><label for=\"minuteModuloWifi\">Minute module WiFi</label>&nbsp;<input type=\"number\" value=\"" + String(config.minuteModuloWifi) + "\" name=\"minuteModuloWifi\" id=\"minuteModuloWifi\" /></div><br />";
  outputHtml += "<div><label for=\"minuteStartWifi\">Minute d&eacutebut; WiFi</label>&nbsp;<input type=\"number\" value=\"" + String(config.minuteStartWifi) + "\" name=\"minuteStartWifi\" id=\"minuteStartWifi\" /></div><br />";
  outputHtml += "<div><label for=\"minuteStopWifi\">Minute fin WiFi</label>&nbsp;<input type=\"number\" value=\"" + String(config.minuteStopWifi) + "\" name=\"minuteStopWifi\" id=\"minuteStopWifi\" /></div><br />";
  outputHtml += "<div><label for=\"forceWifi\">Forcer le WiFi</label>&nbsp;<input type=\"number\" value=\"" + String(config.forceWifi) + "\" name=\"forceWifi\" id=\"forceWifi\" /></div><br />";

  outputHtml += "<div>Date et heure : <strong>" + rtc.getDateTime() + "</strong></div>";
  
  outputHtml += "<button type=\"submit\">Mettre à jour</button></form>";

  outputHtml += "</body></html>";
  
  request->send(200, PSTR("text/html"), outputHtml);

  isDoingSomething = false;
}

// can trigger watchdog if too much files   !
void handleFileWget(AsyncWebServerRequest *request) {
  disableWatchdog();
  
  Serial.print(F("GET ")); Serial.println(request->url());

  if (isDoingSomething) {
    Serial.println(F("Something already running"));
    request->send(503);
    return;
  }

  isDoingSomething = true;
  unsigned int nbPhotos = 0;
  
  outputHtml.remove(0);
  outputHtml += "<html><head><meta charset=\"utf-8\">";

  outputHtml += "<title>HiHiHi Camera - Photos</title>";
  
  outputHtml += "</head><body><h1>Listes photos</h1>";
  
  outputHtml += "<h2>Photos SD</h2>";
  File root = SD_MMC.open("/");
  if(root) {
    File file = root.openNextFile();
    while(file) {
      if(!file.isDirectory() && file.size() > 0) {
        outputHtml += String("<a target=\"_blank\" href=\"/photos_sd/") + file.name() + String("\">") + file.name() + String(" - ") + String(file.size()) + String("- SD</a><br />");
        nbPhotos++;
      }
      file.close();
      file = root.openNextFile();
      yield();
    }
    root.close();
  
    outputHtml += "<br />Nombre de photos SD : " + String(nbPhotos);
  } else {
    Serial.println(F("Failed to open SD root directory"));
    outputHtml += "<br />Impossible d'ouvrir le stockage";
  }

  outputHtml += "<h2>Photos internes</h2>";
  root = LittleFS.open("/");
  if(root) {
    File file = root.openNextFile();
    while(file) {
      if(!file.isDirectory() && file.size() > 0) {
        outputHtml += String("<a target=\"_blank\" href=\"/photos_internal/") + file.name() + String("\">") + file.name() + String(" - ") + String(file.size()) + String(" - Interne</a><br />");
        nbPhotos++;
      }
      file.close();
      file = root.openNextFile();
      yield();
    }
    root.close();
    
    outputHtml += "<br />Nombre de photos internes : " + String(nbPhotos);
  } else {
    Serial.println(F("Failed to open LittleFS root directory"));
    outputHtml += "<br />Impossible d'ouvrir le stockage";
  }
  outputHtml += "</body></html>";
  
  request->send(200, PSTR("text/html"), outputHtml);
  
  isDoingSomething = false;

  enableWatchdog();
}

void handleTakePhoto(AsyncWebServerRequest *request) {
  Serial.print(F("GET ")); Serial.println(request->url());
  
  if (takePhoto() > 0) {
    request->redirect(String("/photos_sd") + lastPhotoPath);
  } else {
    request->send_P(500, PSTR("text/plain"), PSTR("Failed to capture"));
  }
}

void handleSetTime(AsyncWebServerRequest *request) {
  Serial.print(F("GET ")); Serial.println(request->url());

  if (isDoingSomething) {
    Serial.println(F("Something already running"));
    request->send(503);
    return;
  }

  isDoingSomething = true;

  unsigned long newEpoch = request->hasArg(F("epoch")) ? request->arg(F("epoch")).toInt() : 0;
  if (newEpoch <= 0) {
    Serial.println(F("No epoch received"));
    request->send_P(400, PSTR("text/plain"), PSTR("No epoch received"));    
  }
  newEpoch += TIMEZONE;

  Serial.print(F("Set EEPROM epoch to ")); Serial.println(newEpoch);
  
  config.epoch = newEpoch;
  saveConfig();

  initTime();
  
  request->send_P(200, PSTR("text/html"), PSTR("<b>OK !</b><a href=\"/\">Retour</a>"));

  isDoingSomething = false;
}

void handleSetConfig(AsyncWebServerRequest *request) {
  Serial.print(F("POST ")); Serial.println(request->url());

  if (isDoingSomething) {
    Serial.println(F("Something already running"));
    request->send(503);
    return;
  }

  isDoingSomething = true;

  byte params = request->params();
  for(byte i = 0; i < params; i++) {
    AsyncWebParameter* p = request->getParam(i);
    Serial.printf("POST[%s]: %s\n", p->name().c_str(), p->value().c_str());
  }

  config.timeSleepDay = request->arg(F("timeSleepDay")).toInt();
  config.timeSleepNight = request->arg(F("timeSleepNight")).toInt();
  config.hourStartNight = request->arg(F("hourStartNight")).toInt();
  config.hourStopNight = request->arg(F("hourStopNight")).toInt();
  config.hourStartWifi = request->arg(F("hourStartWifi")).toInt();
  config.hourStopWifi = request->arg(F("hourStopWifi")).toInt();
  config.minuteModuloWifi = request->arg(F("minuteModuloWifi")).toInt();
  config.minuteStartWifi = request->arg(F("minuteStartWifi")).toInt();
  config.minuteStopWifi = request->arg(F("minuteStopWifi")).toInt();
  config.forceWifi = request->arg(F("forceWifi")) == "1";

  printConfig();

  saveConfig();
  initTime();
  
  request->send_P(200, PSTR("text/html"), PSTR("<b>OK !</b><a href=\"/\">Retour</a>"));

  isDoingSomething = false;
}

void handleToggleWifi(AsyncWebServerRequest *request) {
  Serial.print(F("GET ")); Serial.println(request->url());

  if (isDoingSomething) {
    Serial.println(F("Something already running"));
    request->send(503);
    return;
  }

  isDoingSomething = true;

  config.forceWifi = !config.forceWifi;
  saveConfig();
  
  request->redirect(PSTR("/"));

  isDoingSomething = false;
}

void handleFormat(AsyncWebServerRequest *request) {
  Serial.print(F("GET ")); Serial.println(request->url());
  
  if (isDoingSomething) {
    Serial.println(F("Something already running"));
    request->send(503);
    return;
  }

  isDoingSomething = true;

  removeAllLittleFS();
  removeAllSD();
  
  request->redirect(PSTR("/"));

  isDoingSomething = false;
}

void removeAllLittleFS() {
  File root = LittleFS.open("/");
  if(!root) {
    Serial.println(F("Failed to open LittleFS root directory"));
  }

  File file = root.openNextFile();
  while(file) {
    if(!file.isDirectory()) {
      tempPath.remove(0);
      tempPath += String("/") + file.name();
      Serial.print(F("Delete file from LittleFS : ")); Serial.println(tempPath);
      file.close();
      if (!LittleFS.remove(tempPath)) {
        Serial.println(F("Error during deletion"));
      }
    }
    file = root.openNextFile();
    yield();
  }
  root.close();
}

// can trigger watchdog if too much files   !
void removeAllSD() {
  disableWatchdog();
  
  File root = SD_MMC.open("/");
  if(!root) {
    Serial.println(F("Failed to open SD root directory"));
  }

  File file = root.openNextFile();
  while(file) {
    if(!file.isDirectory()) {
      tempPath.remove(0);
      tempPath += String("/") + file.name();
      Serial.print(F("Delete file from SD : ")); Serial.println(tempPath);
      file.close();
      if (!SD_MMC.remove(tempPath)) {
        Serial.println(F("Error during deletion"));
      }
    }
    file = root.openNextFile();
    yield();
  }
  root.close();
  
  enableWatchdog();
}

bool isOkForWifi() {
  return config.forceWifi || (rtc.getHour(true) >= config.hourStartWifi && rtc.getHour(true) <= config.hourStopWifi && rtc.getMinute() % config.minuteModuloWifi >= config.minuteStartWifi && rtc.getMinute() % config.minuteModuloWifi <= config.minuteStopWifi);
}

bool isNight() {
  return rtc.getHour(true) >= config.hourStartNight || rtc.getHour(true) <= config.hourStopNight;
}

bool needToTakePhoto() {
  return rtc.getEpoch() - lastPhotoTakenTime >= config.timeSleepDay;
}

void initCam() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
  
  camera_config_t configCamera;
  configCamera.ledc_channel = LEDC_CHANNEL_0;
  configCamera.ledc_timer = LEDC_TIMER_0;
  configCamera.pin_d0 = Y2_GPIO_NUM;
  configCamera.pin_d1 = Y3_GPIO_NUM;
  configCamera.pin_d2 = Y4_GPIO_NUM;
  configCamera.pin_d3 = Y5_GPIO_NUM;
  configCamera.pin_d4 = Y6_GPIO_NUM;
  configCamera.pin_d5 = Y7_GPIO_NUM;
  configCamera.pin_d6 = Y8_GPIO_NUM;
  configCamera.pin_d7 = Y9_GPIO_NUM;
  configCamera.pin_xclk = XCLK_GPIO_NUM;
  configCamera.pin_pclk = PCLK_GPIO_NUM;
  configCamera.pin_vsync = VSYNC_GPIO_NUM;
  configCamera.pin_href = HREF_GPIO_NUM;
  configCamera.pin_sscb_sda = SIOD_GPIO_NUM;
  configCamera.pin_sscb_scl = SIOC_GPIO_NUM;
  configCamera.pin_pwdn = PWDN_GPIO_NUM;
  configCamera.pin_reset = RESET_GPIO_NUM;
  configCamera.xclk_freq_hz = 20000000;
  configCamera.pixel_format = PIXFORMAT_JPEG; 
  
  if(psramFound()) {
    configCamera.frame_size = FRAMESIZE_UXGA;
    configCamera.jpeg_quality = 8;
    configCamera.fb_count = 2;
  } else {
    configCamera.frame_size = FRAMESIZE_SVGA;
    configCamera.jpeg_quality = 12;
    configCamera.fb_count = 1;
  }
  
  pinMode(4, INPUT);
  digitalWrite(4, LOW);
  rtc_gpio_hold_dis(GPIO_NUM_4);
  
  // Init Camera
  esp_err_t err = esp_camera_init(&configCamera);
  if (err != ESP_OK) {
    Serial.print(F("Camera init failed with error 0x")); Serial.println(err, HEX);
    goToSleep(config.timeSleepDay);
  }

  sensor_t * s = esp_camera_sensor_get();
  s->set_whitebal(s, 1);
  s->set_awb_gain(s, 1);
  s->set_wb_mode(s, 0);

  delay(5000);
}

void initTime() {
  long eepromTime = config.epoch;

  Serial.print(F("RTC epoch : ")); Serial.println(rtc.getEpoch());
  Serial.print(F("EEPROM epoch : ")); Serial.println(eepromTime);
  Serial.print(F("CURRENT_EPOCH : ")); Serial.println(CURRENT_EPOCH);

  if (eepromTime > CURRENT_EPOCH) {
    Serial.print(F("Set time from EEPROM to ")); Serial.println(eepromTime);
    rtc.setTime(eepromTime);
  } else if (rtc.getEpoch() < CURRENT_EPOCH) {
    Serial.print(F("Set time from build to ")); Serial.println(CURRENT_EPOCH);
    rtc.setTime(CURRENT_EPOCH);
  }
  
  Serial.println(rtc.getTimeDate());
}

void initWifi() {
  Serial.println(F("Start WiFi SSID : HiHiHi"));

  adc_power_acquire();
  WiFi.disconnect(false);
  
  WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);
  Serial.print(F("AP IP address: ")); Serial.println(WiFi.softAPIP());

  if (!MDNS.begin("hihihi")) {
    Serial.println(F("Error setting up MDNS responder!"));
  }
  Serial.println(F("mDNS started hihihi.local"));

  server.on("/take", HTTP_GET, handleTakePhoto);
  server.on("/time", HTTP_GET, handleSetTime);
  server.on("/wifi", HTTP_GET, handleToggleWifi);
  server.on("/set", HTTP_POST, handleSetConfig);
  server.on("/wget", HTTP_GET, handleFileWget);
  server.on("/format", HTTP_GET, handleFormat);
  server.serveStatic("/photos_sd", SD_MMC, "/");
  server.serveStatic("/photos_internal", LittleFS, "/");
  server.onNotFound(handleFileList);
  server.begin();
}

void initStorage() {
  if(!SD_MMC.begin()) {
    Serial.println(F("SD Card Mount Failed"));
  } else {  
    if(SD_MMC.cardType() == CARD_NONE) {
      Serial.println(F("No SD Card attached"));
    }
  }
  
  if (!LittleFS.begin(true)) {
    Serial.println(F("Error during LittleFS init"));
  }
}

void setup() {
  lastPhotoPath.reserve(64);
  tempPath.reserve(64);
  outputHtml.reserve(4096);
  
  Serial.begin(115200);
  Serial.println(F("Starting"));
  
  getConfig();
  
  initTime();

  initCam();
  initStorage();

  if (isOkForWifi()) {
    initWifi();
  }
}

void loop() {
  if (needToTakePhoto()) {
    takePhoto();
  }
  
  if (!isOkForWifi()) {
    if (isNight()) {
      Serial.println(F("It's night !"));
      goToSleep(config.timeSleepNight);
    } else {
      goToSleep(config.timeSleepDay);
    }
  }
}
