#include <M5Stack.h>
#include <WiFi.h>
#include "time.h"

const char* ssid       = "Buffalo-G-AC8E";
const char* password   = "5aidas7biba8u";

const char* ntpServer =  "ntp.jst.mfeed.ad.jp";
const long  gmtOffset_sec = 9 * 3600;
const int   daylightOffset_sec = 0;
  
uint8_t hh = 00, mm = 00, ss = 00; // Get H, M, S from compile time

byte xcolon = 0, xsecs = 0;

void setup(void) {
  
  Serial.begin(115200);
  M5.begin();
  M5.Lcd.fillScreen(TFT_BLACK);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setTextColor(TFT_YELLOW, TFT_BLACK);

  //connect to WiFi
  M5.Lcd.printf("Connecting to %s ", ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    M5.Lcd.printf(".");
  }
  M5.Lcd.println(" CONNECTED!!");
  delay(500);
  M5.Lcd.clear();
  
  //init and get the time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    M5.Lcd.println("Failed to obtain time");
    return;
  }
  //disconnect WiFi as it's no longer needed
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  

}

void loop() {
  
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    M5.Lcd.println("Failed to obtain time");
    return;
  }
  hh = timeinfo.tm_hour;
  mm = timeinfo.tm_min;
  ss = timeinfo.tm_sec;
  
  // Update digital time
  int xpos = 0;
  int ypos = 85; // Top left corner ot clock text, about half way down
  int ysecs = ypos + 24;

  // Draw hours and minutes
  if (hh < 10) xpos += M5.Lcd.drawChar('0', xpos, ypos, 8); // Add hours leading zero for 24 hr clock
  xpos += M5.Lcd.drawNumber(hh, xpos, ypos, 8);             // Draw hours
  xcolon = xpos; // Save colon coord for later to flash on/off later
  xpos += 29;
  if (mm < 10) xpos += M5.Lcd.drawChar('0', xpos, ypos, 8); // Add minutes leading zero
  xpos += M5.Lcd.drawNumber(mm, xpos, ypos, 8);             // Draw minutes
  xsecs = xpos; // Sae seconds 'x' position for later display updates
 
  M5.Lcd.drawChar(':', xcolon, ypos - 8, 8);     // Hour:minute colon
  xpos += M5.Lcd.drawChar(':', xsecs, ysecs, 6); // Seconds colon
  
  //Draw seconds
  if (ss < 10) xpos += M5.Lcd.drawChar('0', xpos, ysecs, 6); // Add leading zero
  M5.Lcd.drawNumber(ss, xpos, ysecs, 6);                     // Draw seconds
}
