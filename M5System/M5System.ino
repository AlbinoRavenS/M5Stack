#include <M5Stack.h>

// Used AI
#include "utility/MPU9250.h"
#include "utility/quaternionFilters.h"
MPU9250 IMU;
int biasflag = 0;

// Used Clock
#include "time.h"

//Used WifiConnection
#include <WiFi.h>
const char* ssid       = "Buffalo-G-AC8E";
const char* password   = "5aidas7biba8u";

////////////
//LoopTask//
////////////
//ButtonControl
char* btnAstr = "---";
char* btnBstr = "Act/Set";
char* btnCstr = "Mode";

//ModeList
char modeList[][6] = {
  "None",
  "AI",
  "Clock",
  "Rtn" //loop end control
};

void setup() {

  // Initialize the M5Stack object
  M5.begin();
  Wire.begin();
  Serial.begin(115200);

  M5.Lcd.setTextSize(2);

  // LCD display
  menuButton();
  M5.Lcd.printf("Starting... Check OK.");
}

// the loop routine runs over and over again forever
int runMode = 0;
int ModeInitialize = 0;
void loop() {
  switch (runMode) {
    case 0: //None
      ButtonCheck();
      break;

    case 1: //AI
      AI_Loop();
      ButtonCheck();
      break;

    case 2: //Clock
      Clock_Loop();
      ButtonCheck();
      break;

    case 3: //Rtn
      ButtonCheck();
      break;

    default:
      ButtonCheck();
      break;
  }
}

void menuButton() {
  M5.Lcd.clear();
  WriteButtonAction();
}

void WriteButtonAction() {
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(TFT_GREEN);
  M5.Lcd.setCursor(0, 220);
  M5.Lcd.printf("%s", btnAstr);
  M5.Lcd.setCursor(110, 220);
  M5.Lcd.printf("%s", btnBstr);
  M5.Lcd.setCursor(240, 200);
  M5.Lcd.printf("%s", btnCstr);
  M5.Lcd.setCursor(300, 200);
  M5.Lcd.printf("%d", runMode);
  M5.Lcd.setCursor(220, 220);
  M5.Lcd.printf("%s", modeList[runMode]);
  M5.Lcd.setCursor(0, 0);
}
void ButtonCheck() {
  M5.update();

  if (M5.BtnC.pressedFor(3000)) {
    ESP.restart();
  }

  switch (runMode) {
    case 0: //None
      if (M5.BtnC.wasPressed()) {
        ChangeMode();
      }
      break;

    case 1: //AI
      if (M5.BtnB.wasPressed()) {
        biasflag = 1;
      }
      if (M5.BtnB.pressedFor(3000)) {
        AI_Setting();
      }
      if (M5.BtnC.wasPressed()) {
        ChangeMode();
      }
      break;

    case 2: //Clock
      if (M5.BtnC.wasPressed()) {
        ChangeMode();
      }
      break;

    case 3: //Rtn
      if (M5.BtnC.wasPressed()) {
        ChangeMode();
      }
      break;

    default:
      break;
  }
}

void ChangeMode() {
  runMode++;
  if (strcmp(modeList[runMode], "Rtn") == 0) {
    runMode = 0;
  }
  menuButton();
  ModeInitialize = 0;
}

////////////
//LoopTask//
////////////
//setup Initialize & loop
///////////
//MODE_AI//
///////////
float lastroll, lastpitch, lastacx, lastacy;
float accelbias[3] = {0, 0, 0};
float mag_max[3] = {0, 0, 0};
float mag_min[3] = {0, 0, 0};
float pitchbias = 0;


void AI_Initialize() {
  ModeInitialize = 1;
  M5.Lcd.setTextSize(2);
  IMU.initMPU9250();
  IMU.initAK8963(IMU.magCalibration);
  biasflag = 1;
}

void AI_Setting() {
  M5.Lcd.clear();
  M5.Lcd.println("calibration mode");
  M5.Lcd.println("\nset display side up.");
  M5.Lcd.println("keep level");
  M5.Lcd.println("push left button");
  M5.Lcd.setCursor(0, 220);
  M5.Lcd.println("calibrate");
  M5.Lcd.setCursor(220, 220);
  M5.Lcd.println("cancel");
  while (1) {
    delay(1);
    M5.update();
    if (M5.BtnA.wasPressed()) {
      IMU.calibrateMPU9250(IMU.gyroBias, IMU.accelBias);
      M5.Lcd.clear();
      break;
    }
    if (M5.BtnC.wasPressed()) {
      M5.Lcd.clear();
      break;
    }
  }
  biasflag = 1;
}

void AI_Loop() {
  if (ModeInitialize == 0) {
    AI_Initialize();
  }

  // atitude indicator outline
  M5.Lcd.drawCircle(160, 120, 110, TFT_GREEN);
  M5.Lcd.fillRect(158, 0, 4, 10, TFT_GREEN);
  M5.Lcd.drawLine(139, 2, 141, 12, TFT_GREEN);
  M5.Lcd.drawLine(119, 7, 122, 17, TFT_GREEN);
  M5.Lcd.drawLine(100, 16, 105, 25, TFT_GREEN);
  M5.Lcd.drawLine(56, 60, 65, 65, TFT_GREEN);
  M5.Lcd.drawLine(181, 2, 179, 12, TFT_GREEN);
  M5.Lcd.drawLine(201, 7, 198, 17, TFT_GREEN);
  M5.Lcd.drawLine(220, 16, 215, 25, TFT_GREEN);
  M5.Lcd.drawLine(264, 60, 255, 65, TFT_GREEN);

  // put your main code here, to run repeatedly:
  float acx, acy, acz, gyx, gyy, gyz, magx, magy, magz;
  if (IMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {
    IMU.readAccelData(IMU.accelCount);
    IMU.readGyroData(IMU.gyroCount);
    IMU.readMagData(IMU.magCount);

    IMU.getAres();
    IMU.getGres();
    IMU.getMres();


    acx = IMU.accelCount[0] * IMU.aRes;
    acy = IMU.accelCount[1] * IMU.aRes;
    acz = IMU.accelCount[2] * IMU.aRes;

    gyx = IMU.gyroCount[0] * IMU.gRes;
    gyy = IMU.gyroCount[1] * IMU.gRes;
    gyz = IMU.gyroCount[2] * IMU.gRes;

    magx = (float)IMU.magCount[0] * IMU.mRes * IMU.magCalibration[0];
    magy = (float)IMU.magCount[1] * IMU.mRes * IMU.magCalibration[1];
    magz = (float)IMU.magCount[2] * IMU.mRes * IMU.magCalibration[2];

    if (magx > mag_max[0]) {
      mag_max[0] = magx;
    }
    if (magy > mag_max[1]) {
      mag_max[1] = magy;
    }
    if (magz > mag_max[2]) {
      mag_max[2] = magz;
    }
    if (magx < mag_min[0]) {
      mag_min[0] = magx;
    }
    if (magy < mag_min[1]) {
      mag_min[1] = magy;
    }
    if (magz < mag_min[2]) {
      mag_min[2] = magz;
    }

    IMU.magbias[0] = (mag_max[0] + mag_min[0]) / 2;
    IMU.magbias[1] = (mag_max[1] + mag_min[1]) / 2;
    IMU.magbias[2] = (mag_max[2] + mag_min[2]) / 2;

    magx -= IMU.magbias[0];
    magy -= IMU.magbias[1];
    magz -= IMU.magbias[2];

    IMU.updateTime();

    MahonyQuaternionUpdate(acz, acy, acx,
                           gyz * DEG_TO_RAD, gyy * DEG_TO_RAD, gyx * DEG_TO_RAD,
                           magz, magy, magx
                           , IMU.deltat);

    //IMU.count = millis();
  }

  M5.Lcd.setCursor(0, 0);
  //M5.Lcd.printf("ac x:%+2.2f y:%+2.2f z:%+2.2f\n",acx,acy,acz);
  //M5.Lcd.printf("gy x:%+4.f y:%+4.f z:%+4.f\n",gyx,gyy,gyz);

  IMU.delt_t = millis() - IMU.count;

  //IMU.yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ() **(getQ()+3)),
  //                *getQ() * *getQ() + *(getQ()+1) * *(getQ()+1)- *(getQ()+2) * *(getQ()+2) - *(getQ()+3) * *(getQ()+3))
  //                *RAD_TO_DEG;

  IMU.yaw = atan2(2.0f * (*(getQ()) * *(getQ() + 3) + * (getQ() + 1) * *(getQ() + 2)), 1.0f - 2.0f * (*(getQ() + 2)**(getQ() + 2) + * (getQ() + 3)**(getQ() + 3))) * RAD_TO_DEG;

  IMU.roll  = atan2(2.0f * (*getQ() * *(getQ() + 1) + * (getQ() + 2) **(getQ() + 3)),
                    *getQ() * *getQ() - * (getQ() + 1) * *(getQ() + 1) - * (getQ() + 2) * *(getQ() + 2) + * (getQ() + 3) * *(getQ() + 3));

  IMU.pitch = -asin(2.0f * (*(getQ() + 1) * *(getQ() + 3) - *getQ() **(getQ() + 2))) * RAD_TO_DEG;

  if (biasflag) {
    accelbias[0] = acx;
    accelbias[1] = acy;
    accelbias[2] = acz;
    pitchbias = IMU.pitch;
    biasflag = 0;
  }

  IMU.pitch -= pitchbias;
  //M5.Lcd.printf("%.+04.0f,%+04.0f,%+04.0f",IMU.pitch,IMU.yaw*RAD_TO_DEG,IMU.roll*RAD_TO_DEG);

  float lastbar =  sqrt(abs(12100 - lastpitch * lastpitch * 16));
  float pitchbar = sqrt(abs(12100 - IMU.pitch * IMU.pitch * 16));
  //yoku tsukau
  float coslroll = cos(lastroll);
  float sinlroll = sin(lastroll);

  float cosroll = cos(IMU.roll);
  float sinroll = sin(IMU.roll);


  //Horizon bar
  M5.Lcd.drawLine(160 - lastpitch * 4 * coslroll - lastbar * sinlroll,
                  120 + lastpitch * 4 * sinlroll - lastbar * coslroll,
                  160 - lastpitch * 4 * coslroll + lastbar * sinlroll,
                  120 + lastpitch * 4 * sinlroll + lastbar * coslroll, TFT_BLACK);
  if (abs(IMU.pitch * 4) < 100) {
    M5.Lcd.drawLine(160 - IMU.pitch * 4 * cosroll - pitchbar * sinroll,
                    120 + IMU.pitch * 4 * sinroll - pitchbar * cosroll,
                    160 - IMU.pitch * 4 * cosroll + pitchbar * sinroll,
                    120 + IMU.pitch * 4 * sinroll + pitchbar * cosroll, TFT_GREEN);
  }
  for (int i = -9; i <= 9; i++) {
    //M5.Lcd.drawLine(160-30,120+40*i+lastpitch*4,160+30,120+40*i+lastpitch*4,TFT_BLACK);
    //M5.Lcd.drawLine(160-15,120-20+40*i+lastpitch*4,160+15,120-20+40*i+lastpitch*4,TFT_BLACK);
    M5.Lcd.setCursor(160 - (lastpitch * 4 - 40 * i)*coslroll + 30 * sinlroll + 5,
                     120 + (lastpitch * 4 - 40 * i)*sinlroll + 30 * coslroll - 5);
    M5.Lcd.setTextColor(TFT_BLACK);
    M5.Lcd.printf("%d", i * 10);

    //Pitch scale 10deg 60px 5deg 30px
    M5.Lcd.drawLine(160 - (lastpitch * 4 - 40 * i)*coslroll - 30 * sinlroll,
                    120 + (lastpitch * 4 - 40 * i)*sinlroll - 30 * coslroll,
                    160 - (lastpitch * 4 - 40 * i)*coslroll + 30 * sinlroll,
                    120 + (lastpitch * 4 - 40 * i)*sinlroll + 30 * coslroll,
                    TFT_BLACK);
    M5.Lcd.drawLine(160 - (lastpitch * 4 - 40 * i - 20)*coslroll - 15 * sinlroll,
                    120 + (lastpitch * 4 - 40 * i - 20)*sinlroll - 15 * coslroll,
                    160 - (lastpitch * 4 - 40 * i - 20)*coslroll + 15 * sinlroll,
                    120 + (lastpitch * 4 - 40 * i - 20)*sinlroll + 15 * coslroll,
                    TFT_BLACK);

    if (abs(IMU.pitch * 4 - i * 40) < 100) {

      M5.Lcd.drawLine(160 - (IMU.pitch * 4 - 40 * i)*cosroll - 30 * sinroll,
                      120 + (IMU.pitch * 4 - 40 * i)*sinroll - 30 * cosroll,
                      160 - (IMU.pitch * 4 - 40 * i)*cosroll + 30 * sinroll,
                      120 + (IMU.pitch * 4 - 40 * i)*sinroll + 30 * cosroll,
                      TFT_GREEN);
      M5.Lcd.drawLine(160 - (IMU.pitch * 4 - 40 * i - 20)*cosroll - 15 * sinroll,
                      120 + (IMU.pitch * 4 - 40 * i - 20)*sinroll - 15 * cosroll,
                      160 - (IMU.pitch * 4 - 40 * i - 20)*cosroll + 15 * sinroll,
                      120 + (IMU.pitch * 4 - 40 * i - 20)*sinroll + 15 * cosroll,
                      TFT_GREEN);
      M5.Lcd.setCursor(160 - (IMU.pitch * 4 - 40 * i)*cosroll + 30 * sinroll + 5,
                       120 + (IMU.pitch * 4 - 40 * i)*sinroll + 30 * cosroll - 5);
      M5.Lcd.setTextColor(TFT_GREEN);
      M5.Lcd.printf("%d", i * 10);
    }
  }

  //mini aircraft
  //M5.Lcd.drawLine(160-80*sin(lastroll),120+80*cos(lastroll),160+80*sin(lastroll),120-80*cos(lastroll),TFT_BLACK);
  M5.Lcd.drawLine(160 - 50, 120, 160 + 50, 120, TFT_ORANGE);
  M5.Lcd.drawRect(158, 118, 5, 5, TFT_ORANGE);

  //bank bar
  M5.Lcd.drawLine(160 + 110 * coslroll, 120 - 110 * sinlroll, 160 + 80 * coslroll, 120 - 80 * sinlroll, TFT_BLACK);
  M5.Lcd.drawLine(160 + 110 * cosroll, 120 - 110 * sinroll, 160 + 80 * cosroll, 120 - 80 * sinroll, TFT_GREEN);

  M5.Lcd.setCursor(0, 0);
  M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);

  // G Bowl
  M5.Lcd.drawCircle(280, 40, 35, TFT_GREEN);
  M5.Lcd.drawLine(245, 40, 315, 40, TFT_GREEN);
  M5.Lcd.drawLine(280, 5, 280, 75, TFT_GREEN);

  int bowlx = (acx - accelbias[0]) * 20;
  int bowly = (acy - accelbias[1]) * 20;
  M5.Lcd.fillRect(277 - lastacx, 37 + lastacy, 6, 6, TFT_BLACK);
  M5.Lcd.fillRect(277 - bowlx, 37 + bowly, 6, 6, TFT_ORANGE);

  // G Meter
  float g = sqrt(acx * acx + acy * acy + acz * acz);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);
  M5.Lcd.printf("%+1.2f", g);

  lastroll = IMU.roll;
  lastpitch = IMU.pitch;

  lastacx = bowlx;
  lastacy = bowly;

  WriteButtonAction();
  delay(5);
}

////////////
//MODE_CLOCK//
////////////
const char* ntpServer =  "ntp.jst.mfeed.ad.jp";
const long  gmtOffset_sec = 9 * 3600;
const int   daylightOffset_sec = 0;
uint8_t hh = 00, mm = 00, ss = 00; // Get H, M, S from compile time
byte xcolon = 0, xsecs = 0;

void Clock_Initialize() {
  ModeInitialize = 1;
  M5.Lcd.println("Initialize Clock...");
  ConnectWifi();

  //init and get the time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    M5.Lcd.println("Failed to obtain time");
    return;
  }

  DisconnectWifi();
  M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);
}

void Clock_Loop() {
  if (ModeInitialize == 0) {
    Clock_Initialize();
  }

  M5.Lcd.setTextSize(1);
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

////////////
//Function//
////////////

//connect to WiFi
void ConnectWifi() {
  M5.Lcd.printf("Connecting to %s ", ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    M5.Lcd.printf(".");
    ButtonCheck();
  }
  M5.Lcd.println(" CONNECTED!!");
  delay(500);
}

//disconnect WiFi as it's no longer needed
void DisconnectWifi() {
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
}
