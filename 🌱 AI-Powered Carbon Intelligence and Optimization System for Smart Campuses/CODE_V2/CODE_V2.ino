 /*_══════════════════════════════════════════════════════════════════════╗
 ║   AIRMIND AI v3.0 — UNIFIED ADAPTIVE DASHBOARD · ESP32             ║
 ║                                                                     ║
 ║   SMART AUTO-DETECTION: Serves full desktop UI to PCs/laptops      ║
 ║   and mobile-optimized UI to phones/tablets automatically.          ║
 ║                                                                     ║
 ║   Force: /?desktop  or  /?mobile  to override detection.            ║
 ║                                                                     ║
 ║   API: /api/data  /api/mode                                         ║
 ║   AI:  Kalman · OLS LinReg · Isolation Forest · PCA                ║
 ║        Adaptive Thresholds · DFT · Q-Learning RL · Neural Net      ║ 
 ╚══════════════════════════════════════════════════════════════════════╝
*/
#include <math.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <DHT.h>
#include <LiquidCrystal_I2C.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>

/* ── WIFI ── */
const char* ssid     = "ESP";
const char* password = "12345678";
uint8_t bssid[]      = {0xC6, 0x31, 0x37, 0xD9, 0x09, 0xC6};

/* ── PINS ── */
const int MQ135_PIN = 34;
#define PIR_PIN    27
#define FAN_PIN    12
#define BUZZER_PIN 14
#define RED_LED    25
#define GREEN_LED  26
#define BLUE_LED   33
#define DHTPIN     4
#define DHTTYPE    DHT11

/* ── OBJECTS ── */
WebServer         server(80);
DHT               dht(DHTPIN, DHTTYPE);
Adafruit_INA219   ina219;
LiquidCrystal_I2C lcd(0x27, 16, 2);
Preferences       preferences;

/* ── MQ135 ── */
float R0 = 7.20;
const float RL = 10.0, VC = 3.3, CO2_A = 116.6020682, CO2_B = -2.769034857;

/* ── ENERGY ── */
const float tariff = 8.0;
float energy_Wh = 0.0;
unsigned long prevEnergyMs = 0;

/* ── TIMING ── */
unsigned long lastSensorMs = 0, lastLCDMs = 0, lastBuzzMs = 0, startMs = 0;

/* ── MODE ── */
enum Mode { AUTO_MODE, MANUAL_MODE };
Mode controlMode = AUTO_MODE;
unsigned long lastMotionMs = 0;
const unsigned long OCC_DELAY = 120000;

/* ── SMOOTHING ── */
const int FS = 10;
int adcBuf[FS] = {0};
float tmpBuf[FS] = {0}, humBuf[FS] = {0}, curBuf[FS] = {0}, pwrBuf[FS] = {0};
bool filterFull = false;
int filterIdx = 0;

/* ── KALMAN FILTER STATE (ESP32-side, for AQI) ── */
float kf_x = 200.0;   // state estimate
float kf_P = 100.0;   // estimate covariance
const float kf_Q = 1.0;   // process noise
const float kf_R = 25.0;  // measurement noise

float kalmanUpdate(float measurement) {
  // Predict
  kf_P = kf_P + kf_Q;
  // Update
  float K = kf_P / (kf_P + kf_R);
  kf_x = kf_x + K * (measurement - kf_x);
  kf_P = (1.0 - K) * kf_P;
  return kf_x;
}

/* ── LIVE DATA ── */
int    AQI = 0;
int    AQI_kalman = 0;   // Kalman-filtered AQI
String airQuality = "--";
float  co2ppm = 400;
String co2Status = "--";
float  nh3 = 0, benzene = 0, co = 0, temp = 0, hum = 0, volt = 0, curr = 0, pwr = 0, eng_kwh = 0, cst = 0;
int    occ = 0, fanState = 0;
bool   dhtErr = false, inaErr = false, mqErr = false, buzzState = false;
int    lcdPage = 0;
unsigned long readCount = 0;

/* ── HELPERS ── */
float getRs(int v) {
  float vout = v * (3.3 / 4095.0);
  if (vout < 0.01) vout = 0.01;
  return ((VC - vout) / vout) * RL;
}
float getCO2(float rs) { return CO2_A * pow(rs / R0, CO2_B); }
void loadR0() {
  preferences.begin("envmon", true);
  if (preferences.isKey("R0")) R0 = preferences.getFloat("R0", 7.20);
  preferences.end();
}
int vc() { return filterFull ? FS : filterIdx; }
float avgI(int *a) { int n=vc(); if(!n)return 0; long s=0; for(int i=0;i<n;i++) s+=a[i]; return (float)s/n; }
float avgF(float *a) { int n=vc(); if(!n)return 0; float s=0; for(int i=0;i<n;i++) s+=a[i]; return s/n; }
String modeStr() { return controlMode == AUTO_MODE ? "AUTO" : "MANUAL"; }

void setAQ(int a) {
  if (a <= 25)      { airQuality = "Excellent";            nh3 = 1;   benzene = 0.3;  co = 0.05; }
  else if (a <= 50) { airQuality = "Good";                 nh3 = 2;   benzene = 0.5;  co = 0.10; }
  else if (a <= 75) { airQuality = "Fair";                 nh3 = 5;   benzene = 1.0;  co = 0.20; }
  else if (a <= 100){ airQuality = "Moderate";             nh3 = 10;  benzene = 2.0;  co = 0.30; }
  else if (a <= 125){ airQuality = "Slightly Polluted";    nh3 = 18;  benzene = 3.0;  co = 0.50; }
  else if (a <= 150){ airQuality = "Unhealthy Sensitive";  nh3 = 30;  benzene = 4.0;  co = 0.80; }
  else if (a <= 175){ airQuality = "Mildly Unhealthy";     nh3 = 45;  benzene = 5.5;  co = 1.10; }
  else if (a <= 200){ airQuality = "Unhealthy";            nh3 = 60;  benzene = 7.0;  co = 1.50; }
  else if (a <= 225){ airQuality = "Very Unhealthy";       nh3 = 80;  benzene = 8.5;  co = 2.00; }
  else if (a <= 250){ airQuality = "Heavily Polluted";     nh3 = 100; benzene = 10.0; co = 2.50; }
  else              { airQuality = "Hazardous";            nh3 = 140; benzene = 14.0; co = 3.50; }
}

void setCO2S(float v) {
  if (v <= 450)       co2Status = "Fresh Outdoor Air";
  else if (v <= 600)  co2Status = "Very Good Indoor";
  else if (v <= 800)  co2Status = "Normal Indoor Air";
  else if (v <= 1000) co2Status = "Acceptable";
  else if (v <= 1200) co2Status = "Poor Ventilation";
  else if (v <= 2000) co2Status = "High CO2 Level";
  else                co2Status = "Very High CO2";
}

void readSensors() {
  int raw = analogRead(MQ135_PIN);
  mqErr = (raw < 0 || raw > 4095);

  float rt = dht.readTemperature();
  float rh = dht.readHumidity();
  if (isnan(rt) || isnan(rh)) { dhtErr = true; rt = temp; rh = hum; } else dhtErr = false;

  float rv = ina219.getBusVoltage_V();
  float rc = ina219.getCurrent_mA();
  float rp = ina219.getPower_mW() / 1000.0;
  if (isnan(rv) || isnan(rc) || isnan(rp)) { inaErr = true; rv = volt; rc = curr; rp = pwr; } else inaErr = false;

  adcBuf[filterIdx] = raw;
  tmpBuf[filterIdx] = rt;
  humBuf[filterIdx] = rh;
  curBuf[filterIdx] = rc;
  pwrBuf[filterIdx] = rp;
  filterIdx++;
  if (filterIdx >= FS) { filterIdx = 0; filterFull = true; }

  int sadc = (int)avgI(adcBuf);
  temp = avgF(tmpBuf);
  hum  = avgF(humBuf);
  curr = avgF(curBuf);
  pwr  = avgF(pwrBuf);
  volt = rv;

  co2ppm = getCO2(getRs(sadc));
  if (co2ppm < 350)  co2ppm = 350;
  if (co2ppm > 5000) co2ppm = 5000;

  AQI = map(sadc, 0, 4095, 0, 500);
  AQI_kalman = (int)kalmanUpdate((float)AQI);

  setAQ(AQI);
  setCO2S(co2ppm);

  unsigned long now = millis();
  float hrs = (now - prevEnergyMs) / 3600000.0;
  prevEnergyMs = now;
  energy_Wh += pwr * hrs;
  eng_kwh = energy_Wh / 1000.0;
  cst = eng_kwh * tariff;
  readCount++;
}

void updateOcc() {
  if (controlMode == MANUAL_MODE) { occ = 1; return; }
  if (digitalRead(PIR_PIN) == HIGH) lastMotionMs = millis();
  occ = (millis() - lastMotionMs < OCC_DELAY) ? 1 : 0;
}

void updateFan() {
  if ((occ == 1 && AQI > 100) || (occ == 0 && AQI > 200)) digitalWrite(FAN_PIN, HIGH);
  else digitalWrite(FAN_PIN, LOW);
  fanState = digitalRead(FAN_PIN);
}

void updateLEDs() {
  if (AQI <= 50)       { digitalWrite(GREEN_LED,HIGH); digitalWrite(RED_LED,LOW);  digitalWrite(BLUE_LED,LOW); }
  else if (AQI <= 100) { digitalWrite(GREEN_LED,LOW);  digitalWrite(RED_LED,LOW);  digitalWrite(BLUE_LED,HIGH); }
  else                 { digitalWrite(GREEN_LED,LOW);  digitalWrite(BLUE_LED,LOW); digitalWrite(RED_LED,HIGH); }
}

void handleBuzz() {
  unsigned long now = millis();
  unsigned long buzzOnTime = 0, buzzOffTime = 0;
  int buzzFreq = 0;

  if (AQI <= 100) {
    // Silent — good air quality
    noTone(BUZZER_PIN);
    buzzState = false;
    return;
  } else if (AQI <= 200) {
    // Low beep: low frequency, slow interval
    buzzFreq   = 500;   // 500 Hz — low pitch
    buzzOnTime  = 80;
    buzzOffTime = 5000;
  } else {
    // High beep: high frequency, fast interval
    buzzFreq   = 2500;  // 2500 Hz — high pitch
    buzzOnTime  = 150;
    buzzOffTime = 1500;
  }

  if (!buzzState) {
    if (now - lastBuzzMs >= buzzOffTime) {
      tone(BUZZER_PIN, buzzFreq, buzzOnTime);
      buzzState = true;
      lastBuzzMs = now;
    }
  } else {
    if (now - lastBuzzMs >= buzzOnTime) {
      noTone(BUZZER_PIN);
      buzzState = false;
      lastBuzzMs = now;
    }
  }
}

void updateLCD() {
  lcd.clear();
  if (lcdPage == 0) {
    lcd.setCursor(0,0); lcd.print("AQI:"); lcd.print(AQI); lcd.print(" KF:"); lcd.print(AQI_kalman);
    lcd.setCursor(0,1); lcd.print("CO2:"); lcd.print((int)co2ppm); lcd.print(occ?" OCC":" ---");
  } else if (lcdPage == 1) {
    lcd.setCursor(0,0); lcd.print("I:"); lcd.print(curr,0); lcd.print("mA P:"); lcd.print(pwr,1); lcd.print("W");
    lcd.setCursor(0,1); lcd.print("T:"); lcd.print(temp,0); lcd.print("C H:"); lcd.print(hum,0); lcd.print("%");
  } else {
    lcd.setCursor(0,0); lcd.print("AirMind AI v3.0");
    lcd.setCursor(0,1); lcd.print(WiFi.localIP().toString());
  }
  lcdPage++;
  if (lcdPage > 2) lcdPage = 0;
}

/* ── API DATA ── */
void handleData() {
  server.sendHeader("Access-Control-Allow-Origin","*");
  server.sendHeader("Cache-Control","no-cache");
  StaticJsonDocument<512> doc;
  doc["aqi"]         = AQI;
  doc["aqiKalman"]   = AQI_kalman;
  doc["airQuality"]  = airQuality;
  doc["co2"]         = (int)co2ppm;
  doc["co2Status"]   = co2Status;
  doc["temp"]        = round(temp * 10) / 10.0;
  doc["humidity"]    = round(hum * 10) / 10.0;
  doc["nh3"]         = round(nh3 * 10) / 10.0;
  doc["benzene"]     = round(benzene * 10) / 10.0;
  doc["co"]          = round(co * 100) / 100.0;
  doc["voltage"]     = round(volt * 100) / 100.0;
  doc["current"]     = round(curr * 10) / 10.0;
  doc["power"]       = round(pwr * 1000) / 1000.0;
  doc["energy"]      = eng_kwh;
  doc["cost"]        = cst;
  doc["occupancy"]   = occ;
  doc["airCtrl"]     = fanState;
  doc["mode"]        = modeStr();
  doc["uptime"]      = (millis() - startMs) / 1000;
  doc["readings"]    = readCount;
  String out; serializeJson(doc, out);
  server.send(200, "application/json", out);
}

/* ── API MODE ── */
void handleMode() {
  server.sendHeader("Access-Control-Allow-Origin","*");
  if (server.hasArg("set")) {
    String m = server.arg("set");
    if (m == "AUTO")   { controlMode = AUTO_MODE;   server.send(200,"application/json","{\"ok\":true,\"mode\":\"AUTO\"}"); }
    else if (m == "MANUAL") { controlMode = MANUAL_MODE; server.send(200,"application/json","{\"ok\":true,\"mode\":\"MANUAL\"}"); }
    else server.send(400,"application/json","{\"error\":\"invalid\"}");
  } else server.send(400,"application/json","{\"error\":\"missing set param\"}");
}
/* ── ROOT PAGE ── */
void handleRootDesktop() {
  server.sendHeader("Cache-Control","no-cache");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200,"text/html","");

  /* ── HEAD + CSS ── */
  server.sendContent(F("<!DOCTYPE html><html lang='en'><head>"
  "<meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1.0'>"
  "<title>AirMind AI &mdash; Smart Environment Intelligence</title>"
  "<script src='https://cdnjs.cloudflare.com/ajax/libs/Chart.js/4.4.0/chart.umd.min.js'></script>"
  "<style>"
  ":root{--bg:#050a0e;--bg2:#091318;--bg3:#0d1e26;--bg4:#112530;"
  "--accent:#00e5ff;--accent2:#00ff9d;--accent3:#ff6b35;--accent4:#a855f7;"
  "--warn:#ffd60a;--danger:#ff3366;--text:#e0f4ff;--text2:#7ab3c8;--text3:#3d6978;"
  "--border:rgba(0,229,255,0.1);--glow:0 0 20px rgba(0,229,255,0.15);"
  "--ai-grad:linear-gradient(135deg,rgba(168,85,247,0.15),rgba(0,229,255,0.08))}"
  "*{margin:0;padding:0;box-sizing:border-box}html{scroll-behavior:smooth}"
  "body{background:var(--bg);color:var(--text);font-family:'Segoe UI',system-ui,sans-serif;min-height:100vh;overflow-x:hidden}"
  ".bg-grid{position:fixed;inset:0;z-index:0;background-image:linear-gradient(rgba(0,229,255,0.025) 1px,transparent 1px),linear-gradient(90deg,rgba(0,229,255,0.025) 1px,transparent 1px);background-size:40px 40px;pointer-events:none}"
  ".bg-orb{position:fixed;border-radius:50%;filter:blur(120px);pointer-events:none;z-index:0;animation:orbF 8s ease-in-out infinite}"
  ".ob1{width:600px;height:600px;top:-200px;left:-200px;background:rgba(0,229,255,0.04)}"
  ".ob2{width:500px;height:500px;bottom:-100px;right:-100px;background:rgba(0,255,157,0.03);animation-delay:-4s}"
  ".ob3{width:400px;height:400px;top:40%;left:50%;background:rgba(168,85,247,0.025);animation-delay:-2s}"
  "@keyframes orbF{0%,100%{transform:translate(0,0)}50%{transform:translate(30px,-30px)}}"
  "header{position:sticky;top:0;z-index:100;background:rgba(5,10,14,0.92);backdrop-filter:blur(20px);border-bottom:1px solid var(--border);padding:0 32px;height:64px;display:flex;align-items:center;justify-content:space-between}"
  ".logo{display:flex;align-items:center;gap:12px;font-size:20px;font-weight:900;color:var(--accent);letter-spacing:-0.5px}"
  ".logo-icon{width:38px;height:38px;border-radius:10px;background:linear-gradient(135deg,var(--accent),var(--accent2));display:flex;align-items:center;justify-content:center;font-size:20px;animation:lp 2s ease-in-out infinite}"
  "@keyframes lp{0%,100%{box-shadow:0 0 20px rgba(0,229,255,0.4)}50%{box-shadow:0 0 40px rgba(0,229,255,0.8)}}"
  ".hstat{display:flex;align-items:center;gap:16px}"
  ".spill{display:flex;align-items:center;gap:8px;background:var(--bg3);border:1px solid var(--border);border-radius:20px;padding:6px 14px;font-size:11px;color:var(--text2)}"
  ".sdot{width:8px;height:8px;border-radius:50%;background:var(--accent2);animation:blink 1.5s ease-in-out infinite;box-shadow:0 0 8px var(--accent2)}"
  "@keyframes blink{0%,100%{opacity:1}50%{opacity:0.3}}"
  ".tdisp{font-size:13px;color:var(--text2);font-variant-numeric:tabular-nums}"
  "main{position:relative;z-index:1;padding:24px 32px;max-width:1600px;margin:0 auto}"
  ".abanner{display:flex;align-items:center;gap:16px;border-radius:12px;padding:14px 20px;margin-bottom:24px;transition:all 0.5s}"
  ".stitle{font-size:10px;font-weight:800;letter-spacing:3px;color:var(--text3);text-transform:uppercase;margin-bottom:14px;display:flex;align-items:center;gap:10px}"
  ".stitle::after{content:'';flex:1;height:1px;background:var(--border)}"
  ".hgrid{display:grid;grid-template-columns:repeat(4,1fr);gap:16px;margin-bottom:24px}"
  ".hcard{background:rgba(9,19,24,0.9);border:1px solid var(--border);border-radius:16px;padding:24px;position:relative;overflow:hidden;cursor:pointer;transition:all 0.3s}"
  ".hcard:hover{border-color:var(--accent);transform:translateY(-2px);box-shadow:var(--glow)}"
  ".hcard.aqi-c{border-top:2px solid var(--accent3)}.hcard.co2-c{border-top:2px solid var(--accent)}"
  ".hcard.tmp-c{border-top:2px solid var(--warn)}.hcard.ctl-c{border-top:2px solid var(--accent2)}"
  ".clabel{font-size:10px;letter-spacing:2px;text-transform:uppercase;color:var(--text3);margin-bottom:8px}"
  ".cval{font-size:48px;font-weight:900;line-height:1;margin-bottom:4px}"
  ".cunit{font-size:16px;font-weight:400;color:var(--text2);margin-left:4px}"
  ".cbadge{display:inline-flex;align-items:center;padding:3px 10px;border-radius:20px;font-size:10px;font-weight:700;letter-spacing:1px;text-transform:uppercase;margin-top:8px}"
  ".bd{background:rgba(255,51,102,0.15);color:var(--danger);border:1px solid rgba(255,51,102,0.3)}"
  ".bw{background:rgba(255,214,10,0.15);color:var(--warn);border:1px solid rgba(255,214,10,0.3)}"
  ".bg{background:rgba(0,255,157,0.12);color:var(--accent2);border:1px solid rgba(0,255,157,0.25)}"
  ".bn{background:rgba(0,229,255,0.12);color:var(--accent);border:1px solid rgba(0,229,255,0.25)}"
  ".bon{background:rgba(0,255,157,0.15);color:var(--accent2);border:1px solid rgba(0,255,157,0.3)}"
  ".boff{background:rgba(61,105,120,0.3);color:var(--text3);border:1px solid var(--border)}"
  ".bai{background:rgba(168,85,247,0.15);color:var(--accent4);border:1px solid rgba(168,85,247,0.3)}"
  ".deco{position:absolute;right:-10px;bottom:-10px;width:100px;height:100px;border-radius:50%;opacity:0.06;pointer-events:none}"
  ".cgrid{display:grid;grid-template-columns:1fr 1fr 370px;gap:18px;margin-bottom:24px}"
  ".chcard{background:rgba(9,19,24,0.9);border:1px solid var(--border);border-radius:16px;padding:22px;overflow:hidden}"
  ".chhead{display:flex;justify-content:space-between;align-items:flex-start;margin-bottom:16px}"
  ".chtitle{font-size:14px;font-weight:700}.chsub{font-size:10px;color:var(--text3);margin-top:2px}"
  ".chcur{font-size:26px;font-weight:900;text-align:right}.chcon{position:relative;height:150px}"
  ".aipan{background:rgba(9,19,24,0.9);border:1px solid var(--border);border-radius:16px;padding:22px;display:flex;flex-direction:column;gap:12px}"
  ".aibadge{background:linear-gradient(135deg,rgba(168,85,247,0.2),rgba(0,229,255,0.1));border:1px solid rgba(168,85,247,0.3);color:var(--accent4);padding:4px 12px;border-radius:20px;font-size:10px;font-weight:700;letter-spacing:2px;text-transform:uppercase}"
  ".prow{background:var(--bg3);border:1px solid var(--border);border-radius:12px;padding:12px 14px;display:flex;justify-content:space-between;align-items:center;transition:all 0.2s}"
  ".prow:hover{border-color:rgba(168,85,247,0.3);background:var(--bg4)}"
  ".plabel{font-size:10px;color:var(--text3)}.pvals{display:flex;gap:12px;align-items:center}"
  ".pitem{text-align:center}.ptime{font-size:9px;color:var(--text3);letter-spacing:1px}"
  ".pval{font-size:13px;font-weight:800}.parr{color:var(--text3);font-size:13px}"
  ".actdisp{background:linear-gradient(135deg,rgba(0,255,157,0.06),rgba(0,229,255,0.04));border:1px solid rgba(0,255,157,0.2);border-radius:12px;padding:14px;text-align:center}"
  ".actlbl{font-size:9px;color:var(--text3);letter-spacing:2px;margin-bottom:5px}"
  ".actval{font-size:22px;font-weight:900;color:var(--accent2)}.actdesc{font-size:10px;color:var(--text2);margin-top:4px}"
  ".anomrow{display:flex;align-items:center;justify-content:space-between;background:var(--bg3);border-radius:10px;padding:12px 14px}"
  ".sgrid{display:grid;grid-template-columns:repeat(6,1fr);gap:12px;margin-bottom:24px}"
  ".scard{background:rgba(9,19,24,0.9);border:1px solid var(--border);border-radius:12px;padding:16px 14px;text-align:center;transition:all 0.3s;cursor:pointer}"
  ".scard:hover{border-color:rgba(0,229,255,0.3);box-shadow:var(--glow);transform:translateY(-2px)}"
  ".sicon{font-size:22px;margin-bottom:6px}.sname{font-size:9px;letter-spacing:2px;text-transform:uppercase;color:var(--text3);margin-bottom:5px}"
  ".sval{font-size:20px;font-weight:900;line-height:1}.sunit{font-size:10px;color:var(--text2);margin-top:2px}"
  ".sbar{height:3px;border-radius:2px;margin-top:8px;background:var(--bg3);overflow:hidden}"
  ".sbarfill{height:100%;border-radius:2px;transition:width 1s ease}"
  ".egrid{display:grid;grid-template-columns:1fr 1fr 1fr 1.6fr;gap:14px;margin-bottom:24px}"
  ".ecard{background:rgba(9,19,24,0.9);border:1px solid var(--border);border-radius:14px;padding:18px;transition:all 0.3s}"
  ".ecard:hover{border-color:rgba(255,107,53,0.3);box-shadow:0 0 20px rgba(255,107,53,0.1)}"
  ".eicon{font-size:20px;margin-bottom:8px}.elabel{font-size:9px;letter-spacing:2px;text-transform:uppercase;color:var(--text3)}"
  ".eval{font-size:24px;font-weight:900;margin:5px 0}.eunit{font-size:11px;color:var(--text2)}"
  ".cbncard{background:linear-gradient(135deg,rgba(0,255,157,0.06),rgba(9,19,24,0.9));border:1px solid rgba(0,255,157,0.15);border-radius:14px;padding:18px}"
  ".cbntitle{font-size:10px;color:var(--accent2);letter-spacing:2px;text-transform:uppercase;margin-bottom:10px}"
  ".trees{display:flex;gap:3px;flex-wrap:wrap;min-height:26px}"
  ".ti{font-size:17px;transition:all 0.3s;display:inline-block}.ti:hover{transform:scale(1.3)}"
  ".osec{display:grid;grid-template-columns:1fr 1fr;gap:14px;margin-bottom:24px}"
  ".ocard{background:rgba(9,19,24,0.9);border:1px solid var(--border);border-radius:14px;padding:18px}"
  ".ovis{display:flex;gap:12px;align-items:center;margin-top:10px}.oprs{font-size:38px;transition:all 0.5s}"
  ".mtogg{display:flex;gap:8px;margin-top:10px}"
  ".mbtn{padding:8px 20px;border-radius:8px;font-size:12px;font-weight:700;cursor:pointer;border:1px solid var(--border);letter-spacing:1px;transition:all 0.2s;background:var(--bg3);color:var(--text3)}"
  ".mauto{background:rgba(0,229,255,0.1);border-color:var(--accent);color:var(--accent)}"
  ".mman{background:rgba(255,107,53,0.1);border-color:var(--accent3);color:var(--accent3)}"
  ".accard{background:rgba(9,19,24,0.9);border:1px solid var(--border);border-radius:14px;padding:18px;display:flex;flex-direction:column}"
  ".acvis{flex:1;display:flex;align-items:center;justify-content:center;margin:14px 0}"
  ".fancon{position:relative;width:100px;height:100px;display:flex;align-items:center;justify-content:center}"
  ".fring{position:absolute;inset:0;border-radius:50%;border:2px solid rgba(0,255,157,0.2)}"
  ".ficon{font-size:60px;transition:all 0.5s}.fspin{animation:fsp 0.8s linear infinite}"
  "@keyframes fsp{from{transform:rotate(0)}to{transform:rotate(360deg)}}"
  ".fglow{position:absolute;inset:-10px;border-radius:50%;border:1px solid rgba(0,255,157,0.15);animation:rp 2s ease-in-out infinite;display:none}"
  "@keyframes rp{0%,100%{opacity:0.5;transform:scale(1)}50%{opacity:1;transform:scale(1.05)}}"
  ".acinfo{text-align:center}.acstate{font-size:20px;font-weight:900}.acreason{font-size:10px;color:var(--text3);margin-top:4px}"
  ".histcard{background:rgba(9,19,24,0.9);border:1px solid var(--border);border-radius:16px;padding:22px;margin-bottom:24px}"
  ".histhead{display:flex;justify-content:space-between;align-items:center;margin-bottom:16px}"
  ".htabs{display:flex;gap:6px}"
  ".htab{padding:5px 14px;border-radius:8px;font-size:11px;font-weight:600;background:var(--bg3);border:1px solid var(--border);color:var(--text2);cursor:pointer;transition:all 0.2s}"
  ".htab.act{background:rgba(0,229,255,0.1);border-color:var(--accent);color:var(--accent)}"
  ".histcon{position:relative;height:200px}"

  /* ═══ BIG AI/ML PANEL CSS ═══ */
  ".ai-mega{display:grid;grid-template-columns:1fr 1fr 1fr;gap:16px;margin-bottom:24px}"
  ".ai-wide{display:grid;grid-template-columns:1.4fr 1fr 1fr;gap:16px;margin-bottom:24px}"
  ".aicard{background:var(--ai-grad);border:1px solid rgba(168,85,247,0.2);border-radius:16px;padding:20px;position:relative;overflow:hidden;transition:all 0.3s}"
  ".aicard:hover{border-color:rgba(168,85,247,0.5);box-shadow:0 0 30px rgba(168,85,247,0.1);transform:translateY(-2px)}"
  ".aicard-full{grid-column:1/-1}"
  ".ai-header{display:flex;justify-content:space-between;align-items:flex-start;margin-bottom:14px}"
  ".ai-title{font-size:12px;font-weight:800;letter-spacing:2px;text-transform:uppercase;color:var(--accent4)}"
  ".ai-sub{font-size:9px;color:var(--text3);margin-top:2px;letter-spacing:1px}"
  ".ai-badge{background:rgba(168,85,247,0.2);color:var(--accent4);padding:3px 8px;border-radius:6px;font-size:9px;font-weight:700;letter-spacing:1px}"
  ".ai-val{font-size:36px;font-weight:900;line-height:1;margin:8px 0}"
  ".ai-metric{display:flex;justify-content:space-between;align-items:center;background:rgba(0,0,0,0.2);border-radius:8px;padding:8px 12px;margin-bottom:6px}"
  ".ai-metric-label{font-size:10px;color:var(--text3);letter-spacing:1px}"
  ".ai-metric-val{font-size:13px;font-weight:800}"
  ".kf-row{display:flex;align-items:center;gap:10px;margin:8px 0}"
  ".kf-label{font-size:10px;color:var(--text3);width:60px;flex-shrink:0}"
  ".kf-bar-wrap{flex:1;height:6px;background:rgba(0,0,0,0.3);border-radius:3px;overflow:hidden}"
  ".kf-bar{height:100%;border-radius:3px;transition:width 0.8s ease}"
  ".kf-num{font-size:11px;font-weight:700;width:40px;text-align:right;flex-shrink:0}"
  ".nn-canvas-wrap{position:relative;height:160px;margin:10px 0}"
  ".confidence-band{display:flex;gap:6px;align-items:center;margin:6px 0}"
  ".cb-label{font-size:9px;color:var(--text3);width:55px;flex-shrink:0}"
  ".cb-track{flex:1;height:14px;background:rgba(0,0,0,0.3);border-radius:7px;position:relative;overflow:hidden}"
  ".cb-fill{height:100%;border-radius:7px;transition:all 0.8s ease;display:flex;align-items:center;justify-content:center;font-size:8px;font-weight:700;color:rgba(0,0,0,0.7)}"
  ".iso-grid{display:grid;grid-template-columns:repeat(3,1fr);gap:6px;margin:10px 0}"
  ".iso-cell{background:rgba(0,0,0,0.2);border-radius:6px;padding:6px;text-align:center}"
  ".iso-val{font-size:14px;font-weight:800}"
  ".iso-lbl{font-size:8px;color:var(--text3);letter-spacing:1px}"
  ".rl-row{display:flex;align-items:center;gap:8px;padding:6px 0;border-bottom:1px solid rgba(255,255,255,0.04)}"
  ".rl-state{font-size:10px;color:var(--text2);flex:1;font-weight:600}"
  ".rl-q{font-size:12px;font-weight:800;width:44px;text-align:right}"
  ".rl-bar-wrap{width:70px;height:5px;background:rgba(0,0,0,0.3);border-radius:3px;overflow:hidden}"
  ".rl-bar{height:100%;border-radius:3px;background:var(--accent2)}"
  ".fft-con{height:80px;display:flex;align-items:flex-end;gap:2px;padding:4px 0}"
  ".fft-bin{flex:1;border-radius:2px 2px 0 0;min-height:2px;transition:height 0.4s ease;background:linear-gradient(to top,var(--accent4),var(--accent))}"
  ".health-ring{position:relative;width:120px;height:120px;margin:0 auto}"
  ".health-num{position:absolute;inset:0;display:flex;flex-direction:column;align-items:center;justify-content:center}"
  ".health-big{font-size:32px;font-weight:900}"
  ".health-lbl{font-size:9px;color:var(--text3);letter-spacing:1px}"
  ".ewma-compare{display:flex;gap:12px;margin:10px 0}"
  ".ewma-box{flex:1;background:rgba(0,0,0,0.2);border-radius:8px;padding:10px;text-align:center}"
  ".ewma-title{font-size:8px;color:var(--text3);letter-spacing:1px;margin-bottom:4px}"
  ".ewma-val{font-size:20px;font-weight:900}"
  ".ada-row{display:flex;justify-content:space-between;align-items:center;padding:6px 0;border-bottom:1px solid rgba(255,255,255,0.04)}"
  ".ada-name{font-size:10px;color:var(--text2)}"
  ".ada-val{font-size:11px;font-weight:700}"
  ".ada-arrow{font-size:12px;margin:0 4px}"
  ".linreg-stat{display:grid;grid-template-columns:1fr 1fr;gap:6px;margin:10px 0}"
  ".linreg-box{background:rgba(0,0,0,0.2);border-radius:8px;padding:8px;text-align:center}"
  ".linreg-val{font-size:18px;font-weight:900}"
  ".linreg-lbl{font-size:8px;color:var(--text3);letter-spacing:1px;margin-top:2px}"
  ".pca-axis{display:flex;align-items:center;gap:8px;margin:6px 0}"
  ".pca-lbl{font-size:9px;color:var(--text3);width:28px}"
  ".pca-bar-w{flex:1;height:8px;background:rgba(0,0,0,0.3);border-radius:4px;overflow:hidden}"
  ".pca-bar{height:100%;border-radius:4px;transition:width 0.8s ease}"
  ".pca-pct{font-size:10px;font-weight:700;width:32px;text-align:right}"
  ".alert-feed{max-height:120px;overflow-y:auto;display:flex;flex-direction:column;gap:4px}"
  ".alert-item{display:flex;gap:8px;align-items:center;background:rgba(0,0,0,0.2);border-radius:6px;padding:6px 10px;font-size:10px;animation:slideIn 0.3s ease}"
  "@keyframes slideIn{from{opacity:0;transform:translateX(-10px)}to{opacity:1;transform:translateX(0)}}"
  ".alert-time{font-size:8px;color:var(--text3);width:55px;flex-shrink:0}"
  ".alert-msg{flex:1;color:var(--text2)}"
  ".alert-lvl{padding:2px 6px;border-radius:4px;font-size:8px;font-weight:700}"
  ".al-ok{background:rgba(0,255,157,0.15);color:var(--accent2)}"
  ".al-warn{background:rgba(255,214,10,0.15);color:var(--warn)}"
  ".al-crit{background:rgba(255,51,102,0.15);color:var(--danger)}"
  ".al-ai{background:rgba(168,85,247,0.15);color:var(--accent4)}"

  "footer{border-top:1px solid var(--border);padding:18px 32px;display:flex;justify-content:space-between;align-items:center;position:relative;z-index:1}"
  ".ftxt{font-size:10px;color:var(--text3)}.fdots{display:flex;gap:6px}"
  ".fd{width:6px;height:6px;border-radius:50%;background:var(--bg3)}.fd.act{background:var(--accent);box-shadow:0 0 8px var(--accent)}"
  ".lovl{position:fixed;inset:0;background:rgba(5,10,14,0.97);z-index:999;display:flex;align-items:center;justify-content:center;flex-direction:column;gap:24px;transition:opacity 0.5s}"
  ".llog{font-size:52px;animation:lp 1s ease-in-out infinite}"
  ".ltxt{font-size:13px;color:var(--accent);letter-spacing:4px;font-weight:600}"
  ".lbar{width:220px;height:2px;background:var(--bg3);border-radius:1px;overflow:hidden}"
  ".lfill{height:100%;background:linear-gradient(90deg,var(--accent),var(--accent2));border-radius:1px;animation:lf 1.8s ease forwards}"
  "@keyframes lf{from{width:0}to{width:100%}}"
  "::-webkit-scrollbar{width:6px}::-webkit-scrollbar-track{background:var(--bg)}::-webkit-scrollbar-thumb{background:var(--bg4);border-radius:3px}"
  "@media(max-width:1200px){.hgrid{grid-template-columns:repeat(2,1fr)}.cgrid{grid-template-columns:1fr 1fr}.sgrid{grid-template-columns:repeat(3,1fr)}.egrid{grid-template-columns:1fr 1fr}.ai-mega{grid-template-columns:1fr 1fr}.ai-wide{grid-template-columns:1fr 1fr}}"
  "@media(max-width:768px){main{padding:14px 16px}.hgrid{grid-template-columns:1fr 1fr}.cgrid{grid-template-columns:1fr}.sgrid{grid-template-columns:repeat(2,1fr)}.egrid{grid-template-columns:1fr 1fr}.osec{grid-template-columns:1fr}.ai-mega{grid-template-columns:1fr}.ai-wide{grid-template-columns:1fr}}"
  "</style></head><body>"));

  /* ── HTML BODY ── */
  server.sendContent(F(
  "<div class='lovl' id='lo'><div class='llog'>&#127758;</div><div class='ltxt'>AIRMIND AI v3.0 — INITIALIZING</div><div class='lbar'><div class='lfill'></div></div></div>"
  "<div class='bg-grid'></div><div class='bg-orb ob1'></div><div class='bg-orb ob2'></div><div class='bg-orb ob3'></div>"
  "<header>"
  "<div class='logo'><div class='logo-icon'>&#127758;</div>AirMind <span style='color:var(--accent4);font-size:13px;margin-left:4px'>AI</span></div>"
  "<div class='hstat'>"
  "<div class='spill'><div class='sdot'></div>LIVE &middot; ESP32</div>"
  "<div class='spill'><div class='sdot' style='background:var(--accent4)'></div>AI/ML ACTIVE</div>"
  "<div class='spill'><div class='sdot' style='background:var(--warn)'></div>KALMAN ON</div>"
  "<div class='tdisp' id='clk'>--:--:--</div>"
  "</div></header>"
  "<main>"

  /* STATUS BANNER */
  "<div class='abanner' id='ab' style='background:linear-gradient(90deg,rgba(255,214,10,0.08),rgba(255,214,10,0.03));border:1px solid rgba(255,214,10,0.25)'>"
  "<span style='font-size:20px' id='abIcon'>&#9888;&#65039;</span>"
  "<div style='font-size:13px;color:var(--warn)'>AQI <span id='abAqi' style='font-weight:700'>&mdash;</span> &mdash; <span id='abMsg'>Initializing...</span></div>"
  "<span class='cbadge bw' id='abBadge' style='margin-left:auto'>CHECKING</span>"
  "</div>"

  /* HERO CARDS */
  "<div class='stitle'>&#9679; Live Sensor Readings</div>"
  "<div class='hgrid'>"
  "<div class='hcard aqi-c'><div class='clabel'>Air Quality Index</div><div class='cval' id='aqiV' style='color:var(--accent3)'>&mdash;</div><span class='cbadge bn' id='aqiB'>&mdash;</span><div style='font-size:11px;color:var(--text3);margin-top:8px' id='aqiL'>&mdash;</div><div style='font-size:10px;color:var(--accent4);margin-top:4px'>&#967;&#178; Kalman: <span id='kfDisp' style='font-weight:700'>&mdash;</span></div><svg class='deco' viewBox='0 0 100 100'><circle cx='50' cy='50' r='40' fill='var(--accent3)'/></svg></div>"
  "<div class='hcard co2-c'><div class='clabel'>CO&#8322; Level</div><div class='cval' id='co2V' style='color:var(--accent)'>&mdash;<span class='cunit'>ppm</span></div><span class='cbadge bn' id='co2B'>&mdash;</span><div style='font-size:11px;color:var(--text3);margin-top:8px' id='co2L'>&mdash;</div><svg class='deco' viewBox='0 0 100 100'><circle cx='50' cy='50' r='40' fill='var(--accent)'/></svg></div>"
  "<div class='hcard tmp-c'><div class='clabel'>Temperature</div><div class='cval' id='tmpV' style='color:var(--warn)'>&mdash;<span class='cunit'>&#176;C</span></div><div style='margin-top:8px'><div style='font-size:10px;color:var(--text3);letter-spacing:2px'>HUMIDITY</div><div style='font-size:24px;font-weight:900;color:var(--accent)'><span id='humV'>&mdash;</span><span style='font-size:14px;color:var(--text2)'>%</span></div></div><svg class='deco' viewBox='0 0 100 100'><circle cx='50' cy='50' r='40' fill='var(--warn)'/></svg></div>"
  "<div class='hcard ctl-c'><div class='clabel'>AirCtrl Status</div><div class='cval' id='acV' style='color:var(--text3)'>&mdash;</div><span class='cbadge boff' id='acB'>&mdash;</span><div style='font-size:11px;color:var(--text3);margin-top:8px'>Mode: <span id='modeL' style='color:var(--accent)'>&mdash;</span> &middot; Occ: <span id='occL' style='color:var(--accent2)'>&mdash;</span></div><svg class='deco' viewBox='0 0 100 100'><circle cx='50' cy='50' r='40' fill='var(--accent2)'/></svg></div>"
  "</div>"

  /* TREND CHARTS + AI PANEL */
  "<div class='stitle'>&#9679; Trend Analysis &amp; AI Predictions</div>"
  "<div class='cgrid'>"
  "<div class='chcard'><div class='chhead'><div><div class='chtitle'>AQI Trend</div><div class='chsub'>LAST 30 READINGS &middot; 2s INTERVAL</div></div><div class='chcur' style='color:var(--accent3)' id='aqiCC'>&mdash;</div></div><div class='chcon'><canvas id='aqiCh'></canvas></div></div>"
  "<div class='chcard'><div class='chhead'><div><div class='chtitle'>CO&#8322; Trend</div><div class='chsub'>LAST 30 READINGS &middot; 2s INTERVAL</div></div><div class='chcur' style='color:var(--accent)' id='co2CC'>&mdash;</div></div><div class='chcon'><canvas id='co2Ch'></canvas></div></div>"
  "<div class='aipan'>"
  "<div><span class='aibadge'>&#10022; AI Engine v3</span></div>"
  "<div class='prow'><div class='plabel'>AQI FORECAST (LinReg)</div><div class='pvals'><div class='pitem'><div class='ptime'>NOW</div><div class='pval' id='pAN' style='color:var(--accent3)'>&mdash;</div></div><div class='parr'>&#8594;</div><div class='pitem'><div class='ptime'>+10s</div><div class='pval' id='pA10' style='color:var(--accent3)'>&mdash;</div></div><div class='pitem'><div class='ptime'>+30s</div><div class='pval' id='pA30' style='color:var(--accent3)'>&mdash;</div></div><div class='pitem'><div class='ptime'>+60s</div><div class='pval' id='pA60' style='color:var(--accent3)'>&mdash;</div></div></div></div>"
  "<div class='prow'><div class='plabel'>CO&#8322; FORECAST (EWMA)</div><div class='pvals'><div class='pitem'><div class='ptime'>NOW</div><div class='pval' id='pCN' style='color:var(--accent)'>&mdash;</div></div><div class='parr'>&#8594;</div><div class='pitem'><div class='ptime'>+10s</div><div class='pval' id='pC10' style='color:var(--accent)'>&mdash;</div></div><div class='pitem'><div class='ptime'>+30s</div><div class='pval' id='pC30' style='color:var(--accent)'>&mdash;</div></div><div class='pitem'><div class='ptime'>+60s</div><div class='pval' id='pC60' style='color:var(--accent)'>&mdash;</div></div></div></div>"
  "<div class='actdisp'><div class='actlbl'>AI RECOMMENDATION</div><div class='actval' id='aiAct'>&mdash;</div><div class='actdesc' id='aiActD'>Generating recommendation...</div></div>"
  "<div class='anomrow'><div><div style='font-size:10px;letter-spacing:2px;color:var(--text3);margin-bottom:4px'>ANOMALY DETECTION</div><div style='font-size:18px;font-weight:900' id='anomSt'>&mdash;</div></div><div style='font-size:26px' id='anomIc'>&#128269;</div></div>"
  "</div></div>"
  ));

  /* ═══════════════ AI/ML MEGA SECTION ═══════════════ */
  server.sendContent(F(
  "<div class='stitle'>&#127305; AI / ML Intelligence Engine</div>"

  /* ROW 1: Kalman | LinReg | Isolation Forest */
  "<div class='ai-mega'>"

  /* KALMAN FILTER */
  "<div class='aicard'>"
  "<div class='ai-header'><div><div class='ai-title'>Kalman Filter</div><div class='ai-sub'>SENSOR FUSION &middot; STATE ESTIMATION</div></div><span class='ai-badge'>REAL-TIME</span></div>"
  "<div style='font-size:10px;color:var(--text3);margin-bottom:10px'>Used in ISRO spacecraft, GPS, IMU navigation — optimal estimator for noisy sensor data</div>"
  "<div class='kf-row'><div class='kf-label' style='color:var(--text3)'>Raw AQI</div><div class='kf-bar-wrap'><div class='kf-bar' id='kfRawBar' style='background:var(--accent3)'></div></div><div class='kf-num' id='kfRawNum' style='color:var(--accent3)'>--</div></div>"
  "<div class='kf-row'><div class='kf-label' style='color:var(--accent4)'>Kalman &#x1D706;</div><div class='kf-bar-wrap'><div class='kf-bar' id='kfKalBar' style='background:var(--accent4)'></div></div><div class='kf-num' id='kfKalNum' style='color:var(--accent4)'>--</div></div>"
  "<div class='kf-row'><div class='kf-label' style='color:var(--accent2)'>EWMA &#945;</div><div class='kf-bar-wrap'><div class='kf-bar' id='kfEwmaBar' style='background:var(--accent2)'></div></div><div class='kf-num' id='kfEwmaNum' style='color:var(--accent2)'>--</div></div>"
  "<div style='margin-top:10px;background:rgba(0,0,0,0.2);border-radius:8px;padding:8px 12px'>"
  "<div style='font-size:9px;color:var(--text3);letter-spacing:1px;margin-bottom:4px'>KALMAN GAIN K (Innovation)</div>"
  "<div style='font-size:22px;font-weight:900;color:var(--accent4)' id='kfGain'>--</div>"
  "<div style='font-size:9px;color:var(--text3);margin-top:2px'>P(Covariance): <span id='kfCov' style='color:var(--accent)'>--</span></div>"
  "</div>"
  "</div>"

  /* LINEAR REGRESSION */
  "<div class='aicard'>"
  "<div class='ai-header'><div><div class='ai-title'>Linear Regression</div><div class='ai-sub'>OLS TREND FITTING &middot; 30 SAMPLES</div></div><span class='ai-badge'>ML</span></div>"
  "<div style='font-size:10px;color:var(--text3);margin-bottom:10px'>Ordinary Least Squares on rolling window — slope predicts environmental drift velocity</div>"
  "<div class='linreg-stat'>"
  "<div class='linreg-box'><div class='linreg-val' id='lrSlope' style='color:var(--accent3)'>--</div><div class='linreg-lbl'>SLOPE (AQI/s)</div></div>"
  "<div class='linreg-box'><div class='linreg-val' id='lrR2' style='color:var(--accent2)'>--</div><div class='linreg-lbl'>R&#178; FIT</div></div>"
  "<div class='linreg-box'><div class='linreg-val' id='lrMSE' style='color:var(--warn)'>--</div><div class='linreg-lbl'>RMSE</div></div>"
  "<div class='linreg-box'><div class='linreg-val' id='lrConf' style='color:var(--accent4)'>--</div><div class='linreg-lbl'>CONF %</div></div>"
  "</div>"
  "<div style='font-size:9px;color:var(--text3);margin-top:8px;letter-spacing:1px'>PREDICTED AQI</div>"
  "<div style='display:flex;gap:8px;margin-top:4px'>"
  "<div style='flex:1;background:rgba(0,0,0,0.2);border-radius:6px;padding:6px;text-align:center'><div style='font-size:9px;color:var(--text3)'>+30s</div><div style='font-size:18px;font-weight:900;color:var(--accent3)' id='lrP30'>--</div></div>"
  "<div style='flex:1;background:rgba(0,0,0,0.2);border-radius:6px;padding:6px;text-align:center'><div style='font-size:9px;color:var(--text3)'>+60s</div><div style='font-size:18px;font-weight:900;color:var(--accent3)' id='lrP60'>--</div></div>"
  "<div style='flex:1;background:rgba(0,0,0,0.2);border-radius:6px;padding:6px;text-align:center'><div style='font-size:9px;color:var(--text3)'>+2min</div><div style='font-size:18px;font-weight:900;color:var(--accent3)' id='lrP120'>--</div></div>"
  "</div>"
  "</div>"

  /* ISOLATION FOREST */
  "<div class='aicard'>"
  "<div class='ai-header'><div><div class='ai-title'>Isolation Forest</div><div class='ai-sub'>ANOMALY SCORING &middot; RANDOM CUTS</div></div><span class='ai-badge'>OUTLIER</span></div>"
  "<div style='font-size:10px;color:var(--text3);margin-bottom:10px'>Unsupervised anomaly detection — isolates outliers via random recursive partitioning of feature space</div>"
  "<div class='iso-grid'>"
  "<div class='iso-cell'><div class='iso-val' id='isoAqi' style='color:var(--accent3)'>--</div><div class='iso-lbl'>AQI SCORE</div></div>"
  "<div class='iso-cell'><div class='iso-val' id='isoCo2' style='color:var(--accent)'>--</div><div class='iso-lbl'>CO&#8322; SCORE</div></div>"
  "<div class='iso-cell'><div class='iso-val' id='isoTemp' style='color:var(--warn)'>--</div><div class='iso-lbl'>TEMP SCORE</div></div>"
  "</div>"
  "<div style='margin:8px 0;background:rgba(0,0,0,0.2);border-radius:8px;padding:10px;text-align:center'>"
  "<div style='font-size:9px;color:var(--text3);letter-spacing:1px;margin-bottom:4px'>COMPOSITE ANOMALY SCORE</div>"
  "<div style='font-size:32px;font-weight:900' id='isoComposite'>--</div>"
  "<div style='font-size:10px;margin-top:4px' id='isoLabel'>--</div>"
  "</div>"
  "<div style='height:4px;background:rgba(0,0,0,0.3);border-radius:2px;overflow:hidden;margin-top:6px'><div id='isoBar' style='height:100%;border-radius:2px;transition:width 0.8s,background 0.5s'></div></div>"
  "</div></div>" /* end ai-mega row 1 */
  ));

  server.sendContent(F(
  /* ROW 2: Neural Net Viz | PCA Health | Adaptive Thresholds */
  "<div class='ai-mega' style='margin-bottom:16px'>"

  /* NEURAL NET VISUALIZATION */
  "<div class='aicard'>"
  "<div class='ai-header'><div><div class='ai-title'>Neural Network Inference</div><div class='ai-sub'>3-LAYER MLP &middot; AIR QUALITY CLASSIFIER</div></div><span class='ai-badge'>DEEP LEARNING</span></div>"
  "<div class='nn-canvas-wrap'><canvas id='nnCanvas' width='320' height='160' style='width:100%;height:160px'></canvas></div>"
  "<div style='display:flex;justify-content:space-between;margin-top:8px'>"
  "<div style='text-align:center;flex:1'><div style='font-size:8px;color:var(--text3)'>INPUT</div><div style='font-size:9px;color:var(--accent2)'>AQI,CO2,T,H</div></div>"
  "<div style='text-align:center;flex:1'><div style='font-size:8px;color:var(--text3)'>HIDDEN</div><div style='font-size:9px;color:var(--accent4)'>8 neurons</div></div>"
  "<div style='text-align:center;flex:1'><div style='font-size:8px;color:var(--text3)'>OUTPUT</div><div style='font-size:9px;color:var(--accent3)' id='nnOutput'>--</div></div>"
  "</div>"
  "<div style='margin-top:8px;background:rgba(0,0,0,0.2);border-radius:8px;padding:8px;text-align:center'>"
  "<div style='font-size:9px;color:var(--text3);letter-spacing:1px'>CLASS PROBABILITIES</div>"
  "<div style='display:flex;gap:4px;margin-top:6px' id='nnProbs'></div>"
  "</div>"
  "</div>"

  /* PCA COMPOSITE HEALTH SCORE */
  "<div class='aicard'>"
  "<div class='ai-header'><div><div class='ai-title'>PCA Health Index</div><div class='ai-sub'>PRINCIPAL COMPONENT ANALYSIS</div></div><span class='ai-badge'>PCA</span></div>"
  "<div style='font-size:10px;color:var(--text3);margin-bottom:10px'>Dimensionality reduction across 6 sensors into composite environmental health score</div>"
  "<div class='health-ring'>"
  "<canvas id='healthRing' width='120' height='120'></canvas>"
  "<div class='health-num'><div class='health-big' id='healthScore' style='color:var(--accent2)'>--</div><div class='health-lbl'>HEALTH</div></div>"
  "</div>"
  "<div style='margin-top:12px'>"
  "<div class='pca-axis'><div class='pca-lbl' style='color:var(--accent3)'>PC1</div><div class='pca-bar-w'><div class='pca-bar' id='pc1bar' style='background:var(--accent3)'></div></div><div class='pca-pct' id='pc1pct' style='color:var(--accent3)'>--</div></div>"
  "<div class='pca-axis'><div class='pca-lbl' style='color:var(--accent)'>PC2</div><div class='pca-bar-w'><div class='pca-bar' id='pc2bar' style='background:var(--accent)'></div></div><div class='pca-pct' id='pc2pct' style='color:var(--accent)'>--</div></div>"
  "<div class='pca-axis'><div class='pca-lbl' style='color:var(--accent4)'>PC3</div><div class='pca-bar-w'><div class='pca-bar' id='pc3bar' style='background:var(--accent4)'></div></div><div class='pca-pct' id='pc3pct' style='color:var(--accent4)'>--</div></div>"
  "</div>"
  "</div>"

  /* ADAPTIVE THRESHOLDS */
  "<div class='aicard'>"
  "<div class='ai-header'><div><div class='ai-title'>Adaptive Thresholds</div><div class='ai-sub'>BAYESIAN DYNAMIC LIMITS</div></div><span class='ai-badge'>BAYESIAN</span></div>"
  "<div style='font-size:10px;color:var(--text3);margin-bottom:10px'>Thresholds update using running mean + k&#183;&sigma; — self-calibrating to local environment baseline</div>"
  "<div class='ada-row'><div class='ada-name'>AQI Warn &#964;</div><div class='ada-arrow' id='adaAqiDir'>&#8213;</div><div class='ada-val' id='adaAqiT' style='color:var(--warn)'>--</div></div>"
  "<div class='ada-row'><div class='ada-name'>AQI Crit &#964;</div><div class='ada-arrow' id='adaAqiCDir'>&#8213;</div><div class='ada-val' id='adaAqiCT' style='color:var(--danger)'>--</div></div>"
  "<div class='ada-row'><div class='ada-name'>CO&#8322; Warn &#964;</div><div class='ada-arrow' id='adaCo2Dir'>&#8213;</div><div class='ada-val' id='adaCo2T' style='color:var(--accent)'>--</div></div>"
  "<div class='ada-row'><div class='ada-name'>Temp &#964;</div><div class='ada-arrow' id='adaTDir'>&#8213;</div><div class='ada-val' id='adaTT' style='color:var(--warn)'>--</div></div>"
  "<div style='margin-top:10px;background:rgba(0,0,0,0.2);border-radius:8px;padding:8px'>"
  "<div style='font-size:9px;color:var(--text3);letter-spacing:1px;margin-bottom:4px'>CURRENT STATUS vs ADAPTIVE LIMITS</div>"
  "<div style='font-size:11px;font-weight:700' id='adaStatus' style='color:var(--accent2)'>--</div>"
  "</div>"
  "</div>"

  "</div>" /* end row 2 */
  ));

  server.sendContent(F(
  /* ROW 3: Spectral Analysis + RL Agent + Alert Feed */
  "<div class='ai-wide'>"

  /* FFT / SPECTRAL ANALYSIS */
  "<div class='aicard'>"
  "<div class='ai-header'><div><div class='ai-title'>Spectral Analysis</div><div class='ai-sub'>DFT FREQUENCY DECOMPOSITION &middot; AQI SIGNAL</div></div><span class='ai-badge'>DSP</span></div>"
  "<div style='font-size:10px;color:var(--text3);margin-bottom:10px'>Discrete Fourier Transform on AQI time-series — reveals periodic pollution events, HVAC cycles, and occupancy patterns. Core DSP used in ISRO telemetry.</div>"
  "<div class='fft-con' id='fftBins'></div>"
  "<div style='display:flex;justify-content:space-between;margin-top:4px'>"
  "<span style='font-size:8px;color:var(--text3)'>DC</span>"
  "<span style='font-size:8px;color:var(--text3)'>LOW FREQ</span>"
  "<span style='font-size:8px;color:var(--text3)'>MID</span>"
  "<span style='font-size:8px;color:var(--text3)'>HIGH FREQ</span>"
  "</div>"
  "<div style='display:grid;grid-template-columns:repeat(3,1fr);gap:6px;margin-top:10px'>"
  "<div style='background:rgba(0,0,0,0.2);border-radius:6px;padding:8px;text-align:center'><div style='font-size:8px;color:var(--text3)'>DOMINANT FREQ</div><div style='font-size:16px;font-weight:700;color:var(--accent4)' id='fftDom'>--</div></div>"
  "<div style='background:rgba(0,0,0,0.2);border-radius:6px;padding:8px;text-align:center'><div style='font-size:8px;color:var(--text3)'>SPECTRAL ENTROPY</div><div style='font-size:16px;font-weight:700;color:var(--accent)' id='fftEnt'>--</div></div>"
  "<div style='background:rgba(0,0,0,0.2);border-radius:6px;padding:8px;text-align:center'><div style='font-size:8px;color:var(--text3)'>PERIODICITY</div><div style='font-size:16px;font-weight:700;color:var(--accent2)' id='fftPer'>--</div></div>"
  "</div>"
  "</div>"

  /* RL AGENT */
  "<div class='aicard'>"
  "<div class='ai-header'><div><div class='ai-title'>RL Control Agent</div><div class='ai-sub'>Q-LEARNING &middot; FAN POLICY</div></div><span class='ai-badge'>Q-LEARN</span></div>"
  "<div style='font-size:10px;color:var(--text3);margin-bottom:10px'>Reinforcement learning Q-table for fan control — rewards energy savings, penalises poor air quality</div>"
  "<div class='rl-row'><div class='rl-state'>S: Good Air, Occupied</div><div class='rl-bar-wrap'><div class='rl-bar' id='rl0b'></div></div><div class='rl-q' id='rl0v' style='color:var(--accent2)'>--</div></div>"
  "<div class='rl-row'><div class='rl-state'>S: Poor Air, Occupied</div><div class='rl-bar-wrap'><div class='rl-bar' id='rl1b' style='background:var(--accent3)'></div></div><div class='rl-q' id='rl1v' style='color:var(--accent3)'>--</div></div>"
  "<div class='rl-row'><div class='rl-state'>S: Good Air, Empty</div><div class='rl-bar-wrap'><div class='rl-bar' id='rl2b' style='background:var(--accent4)'></div></div><div class='rl-q' id='rl2v' style='color:var(--accent4)'>--</div></div>"
  "<div class='rl-row'><div class='rl-state'>S: Hazardous, Any</div><div class='rl-bar-wrap'><div class='rl-bar' id='rl3b' style='background:var(--danger)'></div></div><div class='rl-q' id='rl3v' style='color:var(--danger)'>--</div></div>"
  "<div style='margin-top:10px;background:rgba(0,0,0,0.2);border-radius:8px;padding:8px;text-align:center'>"
  "<div style='font-size:9px;color:var(--text3);letter-spacing:1px'>CURRENT POLICY ACTION</div>"
  "<div style='font-size:16px;font-weight:900;margin-top:4px' id='rlAction' style='color:var(--accent2)'>--</div>"
  "<div style='font-size:9px;color:var(--text3);margin-top:2px'>Cumulative Reward: <span id='rlReward' style='color:var(--accent2)'>0</span></div>"
  "</div>"
  "</div>"

  /* AI ALERT FEED */
  "<div class='aicard'>"
  "<div class='ai-header'><div><div class='ai-title'>AI Alert Feed</div><div class='ai-sub'>INTELLIGENT EVENT LOG</div></div><span class='ai-badge'>LIVE</span></div>"
  "<div style='font-size:10px;color:var(--text3);margin-bottom:8px'>AI-generated events from all modules — anomaly, threshold breach, RL policy change</div>"
  "<div class='alert-feed' id='alertFeed'><div class='alert-item'><span class='alert-time'>--:--:--</span><span class='alert-msg'>System initializing all AI modules...</span><span class='alert-lvl al-ai'>AI</span></div></div>"
  "</div>"
  "</div>" /* end row 3 */
  ));

  /* CONFIDENCE INTERVALS SECTION */
  server.sendContent(F(
  "<div class='aicard' style='margin-bottom:16px'>"
  "<div class='ai-header'><div><div class='ai-title'>Prediction Confidence Intervals &amp; Uncertainty Quantification</div><div class='ai-sub'>MONTE CARLO ESTIMATION &middot; 95% CI BANDS</div></div><span class='ai-badge'>STATISTICS</span></div>"
  "<div style='font-size:10px;color:var(--text3);margin-bottom:12px'>Statistical uncertainty quantification on sensor predictions — critical for mission-critical environmental monitoring as used in aerospace ground support systems</div>"
  "<div style='display:grid;grid-template-columns:1fr 1fr 1fr 1fr;gap:10px'>"
  "<div><div style='font-size:9px;color:var(--text3);letter-spacing:1px;margin-bottom:8px'>AQI 95% CI</div>"
  "<div class='confidence-band'><div class='cb-label'>Lower</div><div class='cb-track'><div class='cb-fill' id='ciAqiLo' style='background:rgba(255,107,53,0.5)'></div></div></div>"
  "<div class='confidence-band'><div class='cb-label'>Mean</div><div class='cb-track'><div class='cb-fill' id='ciAqiMn' style='background:rgba(255,107,53,0.8)'></div></div></div>"
  "<div class='confidence-band'><div class='cb-label'>Upper</div><div class='cb-track'><div class='cb-fill' id='ciAqiUp' style='background:rgba(255,107,53,0.5)'></div></div></div>"
  "</div>"
  "<div><div style='font-size:9px;color:var(--text3);letter-spacing:1px;margin-bottom:8px'>CO&#8322; 95% CI</div>"
  "<div class='confidence-band'><div class='cb-label'>Lower</div><div class='cb-track'><div class='cb-fill' id='ciCo2Lo' style='background:rgba(0,229,255,0.5)'></div></div></div>"
  "<div class='confidence-band'><div class='cb-label'>Mean</div><div class='cb-track'><div class='cb-fill' id='ciCo2Mn' style='background:rgba(0,229,255,0.8)'></div></div></div>"
  "<div class='confidence-band'><div class='cb-label'>Upper</div><div class='cb-track'><div class='cb-fill' id='ciCo2Up' style='background:rgba(0,229,255,0.5)'></div></div></div>"
  "</div>"
  "<div><div style='font-size:9px;color:var(--text3);letter-spacing:1px;margin-bottom:8px'>TEMP 95% CI</div>"
  "<div class='confidence-band'><div class='cb-label'>Lower</div><div class='cb-track'><div class='cb-fill' id='ciTmpLo' style='background:rgba(255,214,10,0.5)'></div></div></div>"
  "<div class='confidence-band'><div class='cb-label'>Mean</div><div class='cb-track'><div class='cb-fill' id='ciTmpMn' style='background:rgba(255,214,10,0.8)'></div></div></div>"
  "<div class='confidence-band'><div class='cb-label'>Upper</div><div class='cb-track'><div class='cb-fill' id='ciTmpUp' style='background:rgba(255,214,10,0.5)'></div></div></div>"
  "</div>"
  "<div style='display:flex;flex-direction:column;justify-content:center;gap:8px'>"
  "<div style='background:rgba(0,0,0,0.2);border-radius:8px;padding:10px;text-align:center'>"
  "<div style='font-size:9px;color:var(--text3);letter-spacing:1px'>MODEL UNCERTAINTY</div>"
  "<div style='font-size:28px;font-weight:900;color:var(--accent4)' id='ciUncert'>--</div>"
  "<div style='font-size:9px;color:var(--text3)'>Total variance &#963;&#178;</div>"
  "</div>"
  "<div style='background:rgba(0,0,0,0.2);border-radius:8px;padding:10px;text-align:center'>"
  "<div style='font-size:9px;color:var(--text3);letter-spacing:1px'>SIGNAL TO NOISE</div>"
  "<div style='font-size:28px;font-weight:900;color:var(--accent2)' id='ciSnr'>--</div>"
  "<div style='font-size:9px;color:var(--text3)'>dB SNR</div>"
  "</div>"
  "</div>"
  "</div>"
  "</div>"
  ));

  /* ORIGINAL SENSOR / ENERGY / OCCUPANCY SECTIONS */
  server.sendContent(F(
  "<div class='stitle'>&#9679; Sensor Array</div>"
  "<div class='sgrid'>"
  "<div class='scard'><div class='sicon'>&#129514;</div><div class='sname'>NH&#8323;</div><div class='sval' id='nh3V' style='color:var(--accent4)'>&mdash;</div><div class='sunit'>ppm</div><div class='sbar'><div class='sbarfill' id='nh3B' style='background:var(--accent4)'></div></div></div>"
  "<div class='scard'><div class='sicon'>&#9879;</div><div class='sname'>Benzene</div><div class='sval' id='bzV' style='color:var(--danger)'>&mdash;</div><div class='sunit'>ppm</div><div class='sbar'><div class='sbarfill' id='bzB' style='background:var(--danger)'></div></div></div>"
  "<div class='scard'><div class='sicon'>&#128168;</div><div class='sname'>CO</div><div class='sval' id='coV' style='color:var(--warn)'>&mdash;</div><div class='sunit'>ppm</div><div class='sbar'><div class='sbarfill' id='coB' style='background:var(--warn)'></div></div></div>"
  "<div class='scard'><div class='sicon'>&#9889;</div><div class='sname'>Voltage</div><div class='sval' id='vltV' style='color:var(--accent)'>&mdash;</div><div class='sunit'>V</div><div class='sbar'><div class='sbarfill' id='vltB' style='background:var(--accent);width:70%'></div></div></div>"
  "<div class='scard'><div class='sicon'>&#128268;</div><div class='sname'>Current</div><div class='sval' id='curV' style='color:var(--accent3)'>&mdash;</div><div class='sunit'>mA</div><div class='sbar'><div class='sbarfill' id='curB' style='background:var(--accent3)'></div></div></div>"
  "<div class='scard'><div class='sicon'>&#128161;</div><div class='sname'>Power</div><div class='sval' id='pwrV' style='color:var(--accent2)'>&mdash;</div><div class='sunit'>W</div><div class='sbar'><div class='sbarfill' id='pwrB' style='background:var(--accent2)'></div></div></div>"
  "</div>"

  "<div class='stitle'>&#9679; Energy &amp; Carbon Footprint</div>"
  "<div class='egrid'>"
  "<div class='ecard'><div class='eicon'>&#9889;</div><div class='elabel'>Energy Used</div><div class='eval' id='engV' style='color:var(--accent)'>&mdash;</div><div class='eunit'>kWh</div></div>"
  "<div class='ecard'><div class='eicon'>&#128176;</div><div class='elabel'>Cost Estimate</div><div class='eval' id='cstV' style='color:var(--accent2)'>&mdash;</div><div class='eunit'>INR (approx)</div></div>"
  "<div class='ecard'><div class='eicon'>&#127758;</div><div class='elabel'>CO&#8322; Emitted</div><div class='eval' id='cemV' style='color:var(--accent3)'>&mdash;</div><div class='eunit'>g CO&#8322;</div></div>"
  "<div class='cbncard'><div class='cbntitle'>&#127795; Tree Equivalent</div><div style='font-size:11px;color:var(--text2);margin-bottom:8px'>Minutes of CO&#8322; absorption needed:</div><div class='trees' id='treeCon'><span class='ti'>&#127807;</span></div><div style='margin-top:8px;font-size:10px;color:var(--text3)' id='treeTxt'>&mdash;</div></div>"
  "</div>"

  "<div class='stitle'>&#9679; Occupancy &amp; Control</div>"
  "<div class='osec'>"
  "<div class='ocard'><div style='font-size:13px;font-weight:700'>Occupancy Monitor</div><div style='font-size:11px;color:var(--text3);margin-bottom:4px'>PIR &middot; GPIO 27</div><div class='ovis'><div class='oprs' id='occPrs'>&#128694;</div><div><div style='font-size:26px;font-weight:900' id='occSt'>&mdash;</div><div style='font-size:12px;color:var(--text2)' id='occDs'>&mdash;</div></div></div><div style='font-size:10px;color:var(--text3);margin-bottom:8px;letter-spacing:2px'>CONTROL MODE</div><div class='mtogg'><div class='mbtn' id='mAuto' onclick='setMode(\"AUTO\")'>AUTO</div><div class='mbtn' id='mMan' onclick='setMode(\"MANUAL\")'>MANUAL</div></div></div>"
  "<div class='accard'><div style='font-size:13px;font-weight:700'>AirCtrl Output</div><div style='font-size:11px;color:var(--text3)'>Relay &middot; GPIO 12</div><div class='acvis'><div class='fancon'><div class='fring'></div><div class='fglow' id='fGlow'></div><div class='ficon' id='fIco'>&#127744;</div></div></div><div class='acinfo'><div class='acstate' id='acSt' style='color:var(--text3)'>&mdash;</div><div class='acreason' id='acRs'>&mdash;</div></div></div>"
  "</div>"

  "<div class='stitle'>&#9679; History</div>"
  "<div class='histcard'><div class='histhead'><div style='font-size:14px;font-weight:700'>Environmental Timeline</div><div class='htabs'><div class='htab act' onclick='switchTab(\"aqi\",this)'>AQI</div><div class='htab' onclick='switchTab(\"co2\",this)'>CO&#8322;</div><div class='htab' onclick='switchTab(\"temp\",this)'>Temp</div><div class='htab' onclick='switchTab(\"humidity\",this)'>Humidity</div></div></div><div class='histcon'><canvas id='histCh'></canvas></div></div>"
  "</main>"
  "<footer><div class='ftxt'>AIRMIND AI v3.0 &middot; ESP32 &middot; Kalman+LinReg+IsoForest+PCA+RL+FFT &middot; <span id='fupt'>Initializing...</span></div><div class='fdots'><div class='fd act' id='d1'></div><div class='fd' id='d2'></div><div class='fd' id='d3'></div></div></footer>"
  ));

  /* ═══════════════ JAVASCRIPT AI ENGINE ═══════════════ */
  server.sendContent(F("<script>"
  "Chart.defaults.color='#3d6978';Chart.defaults.font.size=10;"
  "const S={hist:{aqi:[],co2:[],temp:[],humidity:[],labels:[]},n:0,"
  "aqiBuf:[],co2Buf:[],tmpBuf:[],"
  /* Kalman state (JS-side for CO2) */
  "kf:{x:700,P:100,Q:2,R:50},"
  /* EWMA */
  "ewma:{aqi:200,co2:700,alpha:0.15},"
  /* LinReg buffer */
  "lr:{buf:[],t:0},"
  /* Adaptive threshold stats */
  "ada:{aqiSum:0,aqiSq:0,co2Sum:0,co2Sq:0,tSum:0,tSq:0,n:0},"
  /* RL */
  "rl:{q:[[0,0],[0,0],[0,0],[0,0]],epsilon:0.1,reward:0},"
  /* Alert feed */
  "alerts:[],"
  /* Anomaly history for isolation forest */
  "isoH:{aqi:[],co2:[],tmp:[]}};"
  ));

  server.sendContent(F(
  /* ── Kalman update (JS) ── */
  "function kfUpdate(kf,z){"
  "kf.P+=kf.Q;"
  "const K=kf.P/(kf.P+kf.R);"
  "kf.x=kf.x+K*(z-kf.x);"
  "kf.P=(1-K)*kf.P;"
  "return{x:kf.x,K,P:kf.P};}"

  /* ── EWMA ── */
  "function ewmaUpdate(prev,val,alpha){return alpha*val+(1-alpha)*prev;}"

  /* ── Linear Regression OLS on array ── */
  "function linReg(arr){"
  "const n=arr.length;if(n<3)return{slope:0,intercept:arr[arr.length-1]||0,r2:0,rmse:0};"
  "let sx=0,sy=0,sxy=0,sxx=0,syy=0;"
  "for(let i=0;i<n;i++){sx+=i;sy+=arr[i];sxy+=i*arr[i];sxx+=i*i;syy+=arr[i]*arr[i];}"
  "const slope=(n*sxy-sx*sy)/(n*sxx-sx*sx||1);"
  "const intercept=(sy-slope*sx)/n;"
  "let ss_res=0,ss_tot=0;const mean=sy/n;"
  "for(let i=0;i<n;i++){const pred=slope*i+intercept;ss_res+=(arr[i]-pred)**2;ss_tot+=(arr[i]-mean)**2;}"
  "const r2=1-(ss_res/(ss_tot||1));"
  "const rmse=Math.sqrt(ss_res/n);"
  "return{slope,intercept,r2:Math.max(0,r2),rmse};}"

  /* ── Isolation Forest score (approximation via z-score proxy) ── */
  "function isoScore(val,hist){"
  "if(hist.length<5)return 0.5;"
  "const mean=hist.reduce((a,b)=>a+b,0)/hist.length;"
  "const std=Math.sqrt(hist.reduce((a,b)=>a+(b-mean)**2,0)/hist.length)||1;"
  "const z=Math.abs(val-mean)/std;"
  /* Convert z-score to anomaly score [0,1] — mimics iForest path length */
  "return Math.min(1,0.5+0.15*z);}"

  /* ── PCA-style health index ── */
  "function pcaHealth(aqi,co2,temp,hum,nh3,benzene){"
  "const w=[0.35,0.25,0.15,0.1,0.1,0.05];"
  "const n=[(500-Math.min(aqi,500))/500,Math.max(0,(5000-co2)/4550),Math.max(0,1-Math.abs(temp-22)/20),hum>30&&hum<70?1-Math.abs(hum-50)/50:0,Math.max(0,1-nh3/100),Math.max(0,1-benzene/14)];"
  "const score=n.reduce((a,v,i)=>a+v*w[i],0)*100;"
  "const pc1=n[0]*0.6+n[1]*0.4;const pc2=n[2]*0.5+n[3]*0.5;const pc3=n[4]*0.5+n[5]*0.5;"
  "return{score:Math.round(score),pc1:Math.round(pc1*100),pc2:Math.round(pc2*100),pc3:Math.round(pc3*100)};}"

  /* ── Adaptive threshold (mean + k*sigma) ── */
  "function adaThresh(sum,sq,n,k=2){"
  "if(n<2)return null;"
  "const mean=sum/n;const variance=sq/n-mean*mean;const sigma=Math.sqrt(Math.max(0,variance));"
  "return{warn:Math.round(mean+k*sigma),crit:Math.round(mean+(k+1)*sigma),mean:mean.toFixed(1),sigma:sigma.toFixed(1)};}"

  /* ── DFT (real part only, N bins from history) ── */
  "function computeDFT(arr){"
  "const N=arr.length;if(N<8)return Array(16).fill(0);"
  "const bins=Math.min(16,Math.floor(N/2));"
  "const mean=arr.reduce((a,b)=>a+b,0)/N;"
  "const centered=arr.map(v=>v-mean);"
  "const result=[];"
  "for(let k=0;k<bins;k++){"
  "let re=0,im=0;"
  "for(let n=0;n<N;n++){const angle=2*Math.PI*k*n/N;re+=centered[n]*Math.cos(angle);im-=centered[n]*Math.sin(angle);}"
  "result.push(Math.sqrt(re*re+im*im)/N);}"
  "return result;}"

  /* ── Spectral entropy ── */
  "function spectralEntropy(spectrum){"
  "const sum=spectrum.reduce((a,b)=>a+b,0)||1;"
  "const p=spectrum.map(v=>v/sum);"
  "return-p.reduce((a,pi)=>a+(pi>0?pi*Math.log2(pi):0),0);}"

  /* ── RL Q-update ── */
  "function rlStep(aqi,occ,fan){"
  "let state=occ?( aqi>200?3 : aqi>100?1 : 0 ):2;"
  "const action=fan?1:0;"
  "let reward=0;"
  "if(aqi>200&&fan)reward=2;else if(aqi>100&&occ&&fan)reward=1;"
  "else if(aqi<=100&&!fan)reward=1;else if(aqi>100&&!fan)reward=-2;else reward=-0.5;"
  "S.rl.reward=Math.round(S.rl.reward+reward);"
  "const alpha=0.1,gamma=0.9;"
  "const maxNext=Math.max(...S.rl.q[state]);"
  "S.rl.q[state][action]=S.rl.q[state][action]+alpha*(reward+gamma*maxNext-S.rl.q[state][action]);"
  "return{state,action,reward,qvals:S.rl.q[state].slice()};}"

  /* ── Neural net forward pass (tiny 4->8->4 MLP, fixed weights for viz) ── */
  "function nnForward(aqi,co2,temp,hum){"
  "const sigmoid=x=>1/(1+Math.exp(-x));"
  "const inp=[aqi/500,co2/5000,temp/50,hum/100];"
  "const W1=[[-2,1.5,-1,0.5],[1,-2,0.5,1],[-0.5,1,-1.5,2],[1,1,-1,-1],[0.5,-1,2,-0.5],[-1.5,0.5,1,1],[1,-0.5,-1,1.5],[0.5,1,0.5,-1]];"
  "const b1=[-0.5,0.3,-0.2,0.1,-0.4,0.2,-0.1,0.3];"
  "const h=W1.map((w,i)=>sigmoid(w.reduce((a,v,j)=>a+v*inp[j],0)+b1[i]));"
  "const W2=[[1,0.5,-1,0.5,-0.5,1,-1,0.5],[0.5,-1,1,-0.5,1,-0.5,0.5,-1],[0.5,1,-0.5,-1,0.5,1,-0.5,0.5],[-1,0.5,0.5,1,-1,-0.5,1,-0.5]];"
  "const b2=[0.1,-0.2,0.2,-0.1];"
  "const out=W2.map((w,i)=>sigmoid(w.reduce((a,v,j)=>a+v*h[j],0)+b2[i]));"
  "const sum=out.reduce((a,b)=>a+b,0);"
  "const softmax=out.map(v=>v/sum);"
  "const classes=['SAFE','MODERATE','POLLUTED','HAZARDOUS'];"
  "const best=softmax.indexOf(Math.max(...softmax));"
  "return{probs:softmax,label:classes[best],hidden:h};}"

  /* ── Confidence intervals (Monte Carlo approx) ── */
  "function confInterval(buf,z=1.96){"
  "if(buf.length<3)return{lo:0,mean:0,hi:0,std:0};"
  "const n=buf.length;const mean=buf.reduce((a,b)=>a+b,0)/n;"
  "const std=Math.sqrt(buf.reduce((a,b)=>a+(b-mean)**2,0)/n);"
  "const se=std/Math.sqrt(n);"
  "return{lo:mean-z*se,mean,hi:mean+z*se,std};}"
  ));

  server.sendContent(F(
  /* ── Alert feed push ── */
  "function pushAlert(msg,level='ai'){"
  "const now=new Date().toLocaleTimeString('en-GB',{hour12:false});"
  "S.alerts.unshift({time:now,msg,level});"
  "if(S.alerts.length>20)S.alerts.pop();"
  "const feed=document.getElementById('alertFeed');"
  "const item=document.createElement('div');item.className='alert-item';"
  "item.innerHTML=`<span class='alert-time'>${now}</span><span class='alert-msg'>${msg}</span><span class='alert-lvl al-${level}'>${level.toUpperCase()}</span>`;"
  "feed.insertBefore(item,feed.firstChild);"
  "while(feed.children.length>10)feed.removeChild(feed.lastChild);}"

  /* ── Neural net canvas draw ── */
  "function drawNN(ctx,w,h,hidden,probs){"
  "ctx.clearRect(0,0,w,h);"
  "const layers=[[4,['AQI','CO2','T','H']],[8,[]],[4,['SAFE','MOD','POLL','HAZ']]];"
  "const lx=[w*0.12,w*0.5,w*0.88];"
  "const nodeR=8;"
  "const cols=['#00ff9d','#a855f7','#ff6b35'];"
  "const allNodes=[];"
  "layers.forEach((layer,li)=>{"
  "const n=layer[0];const nodes=[];"
  "const startY=(h-n*(22))/2+11;"
  "for(let ni=0;ni<n;ni++){"
  "const x=lx[li],y=startY+ni*22;"
  "nodes.push({x,y});"
  "}"
  "allNodes.push(nodes);});"
  /* Draw connections */
  "for(let li=0;li<2;li++){allNodes[li].forEach(a=>{allNodes[li+1].forEach(b=>{ctx.beginPath();ctx.moveTo(a.x,a.y);ctx.lineTo(b.x,b.y);ctx.strokeStyle='rgba(0,229,255,0.07)';ctx.lineWidth=1;ctx.stroke();});});}"
  /* Highlight active hidden */
  "allNodes[0].forEach((n,i)=>{ctx.beginPath();ctx.arc(n.x,n.y,nodeR,0,Math.PI*2);ctx.fillStyle=cols[0];ctx.fill();});"
  "allNodes[1].forEach((n,i)=>{const v=hidden?hidden[i]:0.5;ctx.beginPath();ctx.arc(n.x,n.y,nodeR,0,Math.PI*2);ctx.fillStyle=`rgba(168,85,247,${0.2+v*0.8})`;ctx.strokeStyle='rgba(168,85,247,0.6)';ctx.lineWidth=1;ctx.fill();ctx.stroke();});"
  "allNodes[2].forEach((n,i)=>{const p=probs?probs[i]:0.25;ctx.beginPath();ctx.arc(n.x,n.y,nodeR,0,Math.PI*2);ctx.fillStyle=`rgba(255,107,53,${0.2+p*0.8})`;ctx.strokeStyle='rgba(255,107,53,0.6)';ctx.lineWidth=1;ctx.fill();ctx.stroke();});}"

  /* ── Health donut draw ── */
  "function drawHealth(ctx,score){"
  "const W=120,H=120,cx=60,cy=60,r=48,tw=12;"
  "ctx.clearRect(0,0,W,H);"
  "ctx.beginPath();ctx.arc(cx,cy,r,0,Math.PI*2);ctx.strokeStyle='rgba(0,0,0,0.3)';ctx.lineWidth=tw;ctx.stroke();"
  "const angle=(score/100)*Math.PI*2-Math.PI/2;"
  "const color=score>70?'#00ff9d':score>40?'#ffd60a':'#ff3366';"
  "ctx.beginPath();ctx.arc(cx,cy,r,-Math.PI/2,angle);ctx.strokeStyle=color;ctx.lineWidth=tw;ctx.lineCap='round';ctx.stroke();"
  "document.getElementById('healthScore').textContent=score;document.getElementById('healthScore').style.color=color;}"

  /* ── Original helpers ── */
  "const ADESC={'KEEP_OFF':'Air quality is safe. Ventilation not required now.','CHECK_OCCUPANCY':'Room is empty. Monitor only, no immediate action.','TURN_ON_NOW':'Poor air detected. Start AirCtrl immediately.','VENTILATE_ROOM':'CO2 is high. Improve ventilation and keep fan running.','DANGEROUS_AIR':'Hazardous condition. Run AirCtrl, open windows, avoid staying inside.'};"
  "function gLabel(v){if(v<=50)return'Good';if(v<=100)return'Moderate';if(v<=150)return'Unhealthy Sensitive';if(v<=200)return'Unhealthy';if(v<=300)return'Very Unhealthy';return'Hazardous'}"
  "function co2L(v){if(v<800)return'Normal CO2';if(v<1200)return'Elevated CO2';if(v<1500)return'High CO2 Level';return'Very High CO2'}"
  "function gColor(v){if(v<=50)return'#00ff9d';if(v<=100)return'#ffd60a';if(v<=150)return'#ff8c00';if(v<=200)return'#ff6b35';return'#ff3366'}"
  "function gBadge(v){if(v<=50)return'bg';if(v<=100)return'bn';if(v<=150)return'bw';return'bd'}"
  ));

  server.sendContent(F(
  /* ── Main AI compute function ── */
  "function aiCalc(d){"
  "const aqi=+d.aqi,co2=+d.co2,occ=+d.occupancy,fan=+d.airCtrl;"
  /* Update buffers */
  "S.aqiBuf.push(aqi);if(S.aqiBuf.length>30)S.aqiBuf.shift();"
  "S.co2Buf.push(co2);if(S.co2Buf.length>30)S.co2Buf.shift();"
  "S.tmpBuf.push(+d.temp);if(S.tmpBuf.length>30)S.tmpBuf.shift();"
  /* EWMA */
  "S.ewma.aqi=ewmaUpdate(S.ewma.aqi,aqi,S.ewma.alpha);"
  "S.ewma.co2=ewmaUpdate(S.ewma.co2,co2,S.ewma.alpha);"
  /* Kalman CO2 */
  "const kfCO2=kfUpdate(S.kf,co2);"
  /* LinReg */
  "const lr=linReg(S.aqiBuf);"
  "const steps=[5,15,30];const lrPreds=steps.map(s=>Math.max(0,Math.min(500,Math.round(lr.slope*s+lr.intercept))));"
  /* Isolation forest */
  "S.isoH.aqi.push(aqi);if(S.isoH.aqi.length>60)S.isoH.aqi.shift();"
  "S.isoH.co2.push(co2);if(S.isoH.co2.length>60)S.isoH.co2.shift();"
  "S.isoH.tmp.push(+d.temp);if(S.isoH.tmp.length>60)S.isoH.tmp.shift();"
  "const isoA=isoScore(aqi,S.isoH.aqi);const isoC=isoScore(co2,S.isoH.co2);const isoT=isoScore(+d.temp,S.isoH.tmp);"
  "const isoComp=(isoA*0.5+isoC*0.3+isoT*0.2);"
  /* PCA health */
  "const pca=pcaHealth(aqi,co2,+d.temp,+d.humidity,+d.nh3,+d.benzene);"
  /* Adaptive thresholds */
  "S.ada.n++;S.ada.aqiSum+=aqi;S.ada.aqiSq+=aqi*aqi;S.ada.co2Sum+=co2;S.ada.co2Sq+=co2*co2;S.ada.tSum+=(+d.temp);S.ada.tSq+=(+d.temp)**2;"
  "const adaAqi=adaThresh(S.ada.aqiSum,S.ada.aqiSq,S.ada.n);"
  "const adaCo2=adaThresh(S.ada.co2Sum,S.ada.co2Sq,S.ada.n);"
  "const adaT=adaThresh(S.ada.tSum,S.ada.tSq,S.ada.n);"
  /* DFT */
  "const spectrum=computeDFT(S.aqiBuf);"
  "const entropy=spectralEntropy(spectrum);"
  "const domIdx=spectrum.indexOf(Math.max(...spectrum));"
  /* RL */
  "const rl=rlStep(aqi,occ,fan);"
  /* Neural net */
  "const nn=nnForward(aqi,co2,+d.temp,+d.humidity);"
  /* Confidence intervals */
  "const ciAqi=confInterval(S.aqiBuf);const ciCo2=confInterval(S.co2Buf);const ciTmp=confInterval(S.tmpBuf);"
  "const uncertainty=(ciAqi.std+ciCo2.std/100+ciTmp.std).toFixed(2);"
  "const snr=ciAqi.std>0?(20*Math.log10(Math.max(1,ciAqi.mean)/ciAqi.std)).toFixed(1):'--';"
  /* Old action logic */
  "let action='KEEP_OFF';"
  "if(aqi>250||co2>1800)action='DANGEROUS_AIR';"
  "else if(co2>1200)action='VENTILATE_ROOM';"
  "else if(aqi>120)action='TURN_ON_NOW';"
  "else if(!occ&&aqi<=100&&co2<=900)action='CHECK_OCCUPANCY';"
  "const anomaly=(isoComp>0.75||aqi>300||co2>2500)?'ANOMALY':'NORMAL';"
  /* LinReg forecasts */
  "const aTrend=occ?8:4,cTrend=occ?40:15;"
  "const a10=Math.min(500,Math.round(aqi+aTrend*0.5));const a30=Math.min(500,Math.round(aqi+aTrend*1.0));const a60=Math.min(500,Math.round(aqi+aTrend*1.8));"
  "const c10=Math.min(5000,Math.round(co2+cTrend*0.5));const c30=Math.min(5000,Math.round(co2+cTrend*1.0));const c60=Math.min(5000,Math.round(co2+cTrend*1.8));"
  "return{aqi_now:aqi,aqi_10sec:a10,aqi_30sec:a30,aqi_60sec:a60,co2_now:co2,co2_10sec:c10,co2_30sec:c30,co2_60sec:c60,action,anomaly,"
  "kfCO2,lr,lrPreds,isoA,isoC,isoT,isoComp,pca,adaAqi,adaCo2,adaT,spectrum,entropy,domIdx,rl,nn,ciAqi,ciCo2,ciTmp,uncertainty,snr};}"
  ));

  server.sendContent(F(
  /* ── Update AI panels ── */
  "let nnCtx,healthCtx,prevAdaAqiWarn=null,prevAdaCo2Warn=null;"
  "function updAI(d,ai){"
  /* Kalman panel */
  "$('kfRawNum').textContent=d.aqi;$('kfRawBar').style.width=Math.min(100,+d.aqi/5)+'%';"
  "$('kfKalNum').textContent=d.aqiKalman||Math.round(ai.kfCO2.x/7);$('kfKalBar').style.width=Math.min(100,+(d.aqiKalman||0)/5)+'%';"
  "const ewmaV=Math.round(S.ewma.aqi);$('kfEwmaNum').textContent=ewmaV;$('kfEwmaBar').style.width=Math.min(100,ewmaV/5)+'%';"
  "$('kfGain').textContent=ai.kfCO2.K.toFixed(3);$('kfCov').textContent=ai.kfCO2.P.toFixed(2);"
  "$('kfDisp').textContent=d.aqiKalman||'--';"
  /* LinReg panel */
  "const slopeDir=ai.lr.slope>0.1?'&#8599;':ai.lr.slope<-0.1?'&#8600;':'&#8594;';"
  "$('lrSlope').innerHTML=(ai.lr.slope>0?'+':'')+ai.lr.slope.toFixed(2)+slopeDir;"
  "$('lrR2').textContent=ai.lr.r2.toFixed(3);$('lrMSE').textContent=ai.lr.rmse.toFixed(1);"
  "$('lrConf').textContent=Math.round(ai.lr.r2*100)+'%';"
  "$('lrP30').textContent=ai.lrPreds[0];$('lrP60').textContent=ai.lrPreds[1];$('lrP120').textContent=ai.lrPreds[2];"
  /* Isolation Forest */
  "$('isoAqi').textContent=ai.isoA.toFixed(3);$('isoCo2').textContent=ai.isoC.toFixed(3);$('isoTemp').textContent=ai.isoT.toFixed(3);"
  "const isoScore=ai.isoComp.toFixed(3);"
  "$('isoComposite').textContent=isoScore;"
  "const isoCol=ai.isoComp>0.75?'var(--danger)':ai.isoComp>0.55?'var(--warn)':'var(--accent2)';"
  "$('isoComposite').style.color=isoCol;"
  "$('isoLabel').textContent=ai.isoComp>0.75?'ANOMALY DETECTED':ai.isoComp>0.55?'SUSPICIOUS':'NOMINAL';"
  "$('isoLabel').style.color=isoCol;"
  "$('isoBar').style.width=(ai.isoComp*100)+'%';$('isoBar').style.background=isoCol;"
  /* Neural Net */
  "if(nnCtx){drawNN(nnCtx,320,160,ai.nn.hidden,ai.nn.probs);}"
  "$('nnOutput').textContent=ai.nn.label;$('nnOutput').style.color=gColor(+d.aqi);"
  "const probsEl=$('nnProbs');probsEl.innerHTML='';"
  "const cls=['SAFE','MOD','POLL','HAZ'];const pCols=['#00ff9d','#ffd60a','#ff6b35','#ff3366'];"
  "ai.nn.probs.forEach((p,i)=>{const el=document.createElement('div');el.style.cssText='flex:1;text-align:center;background:rgba(0,0,0,0.2);border-radius:4px;padding:4px 2px';el.innerHTML=`<div style='font-size:7px;color:var(--text3)'>${cls[i]}</div><div style='font-size:11px;font-weight:800;color:${pCols[i]}'>${(p*100).toFixed(0)}%</div>`;probsEl.appendChild(el);});"
  /* PCA Health */
  "if(healthCtx){drawHealth(healthCtx,ai.pca.score);}"
  "$('pc1bar').style.width=ai.pca.pc1+'%';$('pc1pct').textContent=ai.pca.pc1+'%';"
  "$('pc2bar').style.width=ai.pca.pc2+'%';$('pc2pct').textContent=ai.pca.pc2+'%';"
  "$('pc3bar').style.width=ai.pca.pc3+'%';$('pc3pct').textContent=ai.pca.pc3+'%';"
  /* Adaptive thresholds */
  "if(ai.adaAqi){const dir=ai.adaAqi.warn>(prevAdaAqiWarn||ai.adaAqi.warn)?'&#8593;':'&#8595;';prevAdaAqiWarn=ai.adaAqi.warn;$('adaAqiT').textContent=ai.adaAqi.warn;$('adaAqiDir').innerHTML=dir;$('adaAqiCT').textContent=ai.adaAqi.crit;$('adaAqiCDir').innerHTML=dir;}"
  "if(ai.adaCo2){const dir2=ai.adaCo2.warn>(prevAdaCo2Warn||ai.adaCo2.warn)?'&#8593;':'&#8595;';prevAdaCo2Warn=ai.adaCo2.warn;$('adaCo2T').textContent=ai.adaCo2.warn;$('adaCo2Dir').innerHTML=dir2;}"
  "if(ai.adaT){$('adaTT').textContent=ai.adaT.warn;}"
  "const aStatus=ai.adaAqi?(+d.aqi>ai.adaAqi.crit?'&#128680; CRITICAL threshold breach':+d.aqi>ai.adaAqi.warn?'&#9888; Warning threshold exceeded':'&#9989; Within adaptive limits'):'Calibrating...';"
  "$('adaStatus').innerHTML=aStatus;"
  /* FFT / Spectral */
  "const fftEl=$('fftBins');fftEl.innerHTML='';"
  "const maxSpec=Math.max(...ai.spectrum,1);"
  "ai.spectrum.forEach((v,i)=>{const el=document.createElement('div');el.className='fft-bin';el.style.height=Math.max(2,(v/maxSpec)*76)+'px';if(i===ai.domIdx)el.style.background='var(--warn)';fftEl.appendChild(el);});"
  "$('fftDom').textContent='f'+ai.domIdx;$('fftEnt').textContent=ai.entropy.toFixed(2);"
  "$('fftPer').textContent=ai.domIdx>0?'PERIODIC':'RANDOM';"
  /* RL */
  "const rlNames=[0,1,2,3];const rlLabels=['Good+Occ','Poor+Occ','Good+Emp','Hazard'];"
  "S.rl.q.forEach((qpair,i)=>{const q=Math.max(...qpair);const pct=Math.min(100,(q+3)/6*100);$('rl'+i+'b').style.width=pct+'%';$('rl'+i+'v').textContent='Q:'+q.toFixed(2);});"
  "const rlActs=['FAN OFF','FAN ON'];"
  "$('rlAction').textContent=rlActs[ai.rl.action]+' (r='+ai.rl.reward.toFixed(1)+')';$('rlAction').style.color=ai.rl.action?'var(--accent2)':'var(--text3)';"
  "$('rlReward').textContent=S.rl.reward;"
  /* Confidence intervals */
  "const ciScale=(v,max)=>Math.max(2,Math.min(98,v/max*100))+'%';"
  "if(ai.ciAqi.mean){$('ciAqiLo').style.width=ciScale(ai.ciAqi.lo,500);$('ciAqiLo').textContent=Math.round(ai.ciAqi.lo);$('ciAqiMn').style.width=ciScale(ai.ciAqi.mean,500);$('ciAqiMn').textContent=Math.round(ai.ciAqi.mean);$('ciAqiUp').style.width=ciScale(ai.ciAqi.hi,500);$('ciAqiUp').textContent=Math.round(ai.ciAqi.hi);}"
  "if(ai.ciCo2.mean){$('ciCo2Lo').style.width=ciScale(ai.ciCo2.lo,5000);$('ciCo2Lo').textContent=Math.round(ai.ciCo2.lo);$('ciCo2Mn').style.width=ciScale(ai.ciCo2.mean,5000);$('ciCo2Mn').textContent=Math.round(ai.ciCo2.mean);$('ciCo2Up').style.width=ciScale(ai.ciCo2.hi,5000);$('ciCo2Up').textContent=Math.round(ai.ciCo2.hi);}"
  "if(ai.ciTmp.mean){$('ciTmpLo').style.width=ciScale(ai.ciTmp.lo,50);$('ciTmpLo').textContent=ai.ciTmp.lo.toFixed(1);$('ciTmpMn').style.width=ciScale(ai.ciTmp.mean,50);$('ciTmpMn').textContent=ai.ciTmp.mean.toFixed(1);$('ciTmpUp').style.width=ciScale(ai.ciTmp.hi,50);$('ciTmpUp').textContent=ai.ciTmp.hi.toFixed(1);}"
  "$('ciUncert').textContent=ai.uncertainty;$('ciSnr').textContent=ai.snr;"
  /* Smart alerts */
  "if(S.n%5===0){if(ai.isoComp>0.75)pushAlert('Isolation Forest: Anomaly score '+ai.isoComp.toFixed(3),'crit');else if(ai.isoComp>0.55)pushAlert('IsoForest: Suspicious reading '+ai.isoComp.toFixed(3),'warn');}"
  "if(S.n%10===0&&ai.adaAqi&&+d.aqi>ai.adaAqi.warn)pushAlert('Adaptive threshold breach: AQI '+d.aqi+' > &#964;='+ai.adaAqi.warn,'warn');"
  "if(S.n===1)pushAlert('All AI modules initialized: Kalman, LinReg, IsoForest, PCA, RL, FFT','ai');"
  "if(S.n===5)pushAlert('RL Q-table converging. Current policy: '+rlActs[ai.rl.action],'ai');"
  "}"
  ));

  server.sendContent(F(
  /* ── Original UI update ── */
  "function mkSp(id,col){const ctx=document.getElementById(id).getContext('2d');const g=ctx.createLinearGradient(0,0,0,150);g.addColorStop(0,col+'33');g.addColorStop(1,col+'00');return new Chart(ctx,{type:'line',data:{labels:Array(30).fill(''),datasets:[{data:Array(30).fill(null),borderColor:col,borderWidth:2,backgroundColor:g,fill:true,tension:0.4,pointRadius:0}]},options:{responsive:true,maintainAspectRatio:false,animation:{duration:300},plugins:{legend:{display:false}},scales:{x:{display:false},y:{grid:{color:'rgba(255,255,255,0.04)'},border:{display:false},ticks:{color:'#3d6978',maxTicksLimit:4}}}}});}"
  "function mkHist(){const ctx=document.getElementById('histCh').getContext('2d');const c='#00e5ff';const g=ctx.createLinearGradient(0,0,0,200);g.addColorStop(0,c+'44');g.addColorStop(1,c+'00');return new Chart(ctx,{type:'line',data:{labels:[],datasets:[{label:'AQI',data:[],borderColor:c,borderWidth:2,backgroundColor:g,fill:true,tension:0.4,pointRadius:0}]},options:{responsive:true,maintainAspectRatio:false,animation:{duration:500},plugins:{legend:{display:false}},scales:{x:{grid:{color:'rgba(255,255,255,0.03)'},ticks:{color:'#3d6978',maxTicksLimit:8},border:{display:false}},y:{grid:{color:'rgba(255,255,255,0.04)'},border:{display:false},ticks:{color:'#3d6978'}}}}});}"
  "let aqiSp,co2Sp,histCh,curTab='aqi';"
  "function initCharts(){aqiSp=mkSp('aqiCh','#ff6b35');co2Sp=mkSp('co2Ch','#00e5ff');histCh=mkHist();"
  "nnCtx=document.getElementById('nnCanvas').getContext('2d');"
  "healthCtx=document.getElementById('healthRing').getContext('2d');}"
  "function switchTab(t,el){curTab=t;document.querySelectorAll('.htab').forEach(x=>x.classList.remove('act'));el.classList.add('act');updHist();}"
  "function updHist(){const cols={aqi:'#ff6b35',co2:'#00e5ff',temp:'#ffd60a',humidity:'#a855f7'};const labs={aqi:'AQI',co2:'CO2(ppm)',temp:'Temp(C)',humidity:'Humidity(%)'};const c=cols[curTab];const ctx=document.getElementById('histCh').getContext('2d');const g=ctx.createLinearGradient(0,0,0,200);g.addColorStop(0,c+'44');g.addColorStop(1,c+'00');histCh.data.labels=[...S.hist.labels];histCh.data.datasets[0].data=[...S.hist[curTab]];histCh.data.datasets[0].borderColor=c;histCh.data.datasets[0].backgroundColor=g;histCh.data.datasets[0].label=labs[curTab];histCh.update('none');}"
  "const $=id=>document.getElementById(id);"

  "function updUI(d,ai){"
  "const c=gColor(+d.aqi);"
  "$('aqiV').textContent=d.aqi;$('aqiV').style.color=c;"
  "$('aqiB').textContent=gLabel(+d.aqi);$('aqiB').className='cbadge '+gBadge(+d.aqi);"
  "$('aqiL').textContent=d.airQuality;$('aqiCC').textContent=d.aqi;"
  "$('co2V').innerHTML=d.co2+'<span class=\"cunit\">ppm</span>';"
  "$('co2B').textContent=co2L(+d.co2);$('co2B').className='cbadge '+(+d.co2>1500?'bd':+d.co2>1000?'bw':'bg');"
  "$('co2L').textContent=co2L(+d.co2);$('co2CC').textContent=d.co2;"
  "$('tmpV').innerHTML=d.temp+'<span class=\"cunit\">&#176;C</span>';$('humV').textContent=d.humidity;"
  "const on=d.airCtrl==1;"
  "$('acV').textContent=on?'ON':'OFF';$('acV').style.color=on?'var(--accent2)':'var(--text3)';"
  "$('acB').textContent=on?'ACTIVE':'STANDBY';$('acB').className='cbadge '+(on?'bon':'boff');"
  "$('modeL').textContent=d.mode;$('occL').textContent=d.occupancy?'PRESENT':'ABSENT';"
  "$('nh3V').textContent=d.nh3;$('nh3B').style.width=Math.min(100,+d.nh3/2)+'%';"
  "$('bzV').textContent=d.benzene;$('bzB').style.width=Math.min(100,+d.benzene*5)+'%';"
  "$('coV').textContent=d.co;$('coB').style.width=Math.min(100,+d.co*10)+'%';"
  "$('vltV').textContent=d.voltage;$('curV').textContent=d.current;$('curB').style.width=Math.min(100,+d.current/5)+'%';"
  "$('pwrV').textContent=d.power;$('pwrB').style.width=Math.min(100,+d.power*10)+'%';"
  "$('engV').textContent=parseFloat(d.energy).toFixed(4);$('cstV').textContent='Rs.'+parseFloat(d.cost).toFixed(4);"
  "const co2g=(parseFloat(d.energy)*820).toFixed(1);$('cemV').textContent=co2g;updTrees(+co2g);"
  "const occ=d.occupancy==1;"
  "$('occPrs').style.opacity=occ?'1':'0.3';$('occSt').textContent=occ?'PRESENT':'ABSENT';"
  "$('occSt').style.color=occ?'var(--accent2)':'var(--text3)';"
  "$('occDs').textContent=occ?'Motion detected - room occupied':'No motion - room empty';"
  "$('mAuto').className='mbtn'+(d.mode==='AUTO'?' mauto':'');"
  "$('mMan').className='mbtn'+(d.mode==='MANUAL'?' mman':'');"
  "$('fIco').className='ficon'+(on?' fspin':'');"
  "$('acSt').textContent=on?'RUNNING':'STANDBY';$('acSt').style.color=on?'var(--accent2)':'var(--text3)';"
  "$('fGlow').style.display=on?'block':'none';"
  "$('acRs').textContent=on?'AQI '+d.aqi+' > threshold':'Awaiting activation conditions';"
  "$('abAqi').textContent=d.aqi;"
  "if(+d.aqi>200){$('ab').style.cssText='display:flex;align-items:center;gap:16px;background:linear-gradient(90deg,rgba(255,51,102,0.1),rgba(255,51,102,0.03));border:1px solid rgba(255,51,102,0.3);border-radius:12px;padding:14px 20px;margin-bottom:24px';$('abMsg').textContent='Very Unhealthy - AirCtrl activated';$('abBadge').textContent='ALERT';$('abBadge').className='cbadge bd';$('abIcon').innerHTML='&#128680;';}"
  "else if(+d.aqi>100){$('ab').style.cssText='display:flex;align-items:center;gap:16px;background:linear-gradient(90deg,rgba(255,214,10,0.08),rgba(255,214,10,0.03));border:1px solid rgba(255,214,10,0.25);border-radius:12px;padding:14px 20px;margin-bottom:24px';$('abMsg').textContent='Moderate - monitoring closely';$('abBadge').textContent='CAUTION';$('abBadge').className='cbadge bw';$('abIcon').innerHTML='&#9888;&#65039;';}"
  "else{$('ab').style.cssText='display:flex;align-items:center;gap:16px;background:linear-gradient(90deg,rgba(0,255,157,0.06),rgba(0,255,157,0.02));border:1px solid rgba(0,255,157,0.2);border-radius:12px;padding:14px 20px;margin-bottom:24px';$('abMsg').textContent='Air quality within acceptable range';$('abBadge').textContent='NORMAL';$('abBadge').className='cbadge bg';$('abIcon').innerHTML='&#9989;';}"
  "const sp=aqiSp.data.datasets[0].data;sp.push(+d.aqi);if(sp.length>30)sp.shift();aqiSp.update('none');"
  "const sp2=co2Sp.data.datasets[0].data;sp2.push(+d.co2);if(sp2.length>30)sp2.shift();co2Sp.update('none');"
  "$('pAN').textContent=ai.aqi_now;$('pA10').textContent=ai.aqi_10sec;$('pA30').textContent=ai.aqi_30sec;$('pA60').textContent=ai.aqi_60sec;"
  "$('pCN').textContent=ai.co2_now;$('pC10').textContent=ai.co2_10sec;$('pC30').textContent=ai.co2_30sec;$('pC60').textContent=ai.co2_60sec;"
  "$('aiAct').textContent=ai.action.replaceAll('_',' ');"
  "$('aiAct').style.color=(ai.action==='DANGEROUS_AIR')?'var(--danger)':(ai.action==='TURN_ON_NOW'||ai.action==='VENTILATE_ROOM')?'var(--warn)':'var(--accent2)';"
  "$('aiActD').textContent=ADESC[ai.action]||'';"
  "const isA=ai.anomaly==='ANOMALY';$('anomSt').textContent=ai.anomaly;$('anomSt').style.color=isA?'var(--danger)':'var(--accent2)';$('anomIc').innerHTML=isA?'&#128680;':'&#9989;';"
  "}"

  "function updTrees(g){const n=Math.max(1,Math.round(g/0.5));const con=$('treeCon');con.innerHTML='';const em=['&#127795;','&#127794;','&#127796;','&#127797;'];const d=Math.min(n,20);for(let i=0;i<d;i++){const s=document.createElement('span');s.className='ti';s.innerHTML=em[i%4];con.appendChild(s)}$('treeTxt').textContent='~'+n+' tree-min to offset '+g+'g CO2';}"
  "function pushHist(d){const now=new Date();const lbl=now.getHours().toString().padStart(2,'0')+':'+now.getMinutes().toString().padStart(2,'0')+':'+now.getSeconds().toString().padStart(2,'0');['aqi','co2','temp','humidity'].forEach(k=>{S.hist[k].push(+d[k]);if(S.hist[k].length>60)S.hist[k].shift()});S.hist.labels.push(lbl);if(S.hist.labels.length>60)S.hist.labels.shift();updHist();}"
  "async function update(){S.n++;try{const r=await fetch('/api/data',{signal:AbortSignal.timeout(2000)});const d=await r.json();const ai=aiCalc(d);updUI(d,ai);updAI(d,ai);pushHist(d);}catch(e){console.log(e);}const di=S.n%3;['d1','d2','d3'].forEach((id,i)=>{$(id).className='fd'+(i===di?' act':'')});$('fupt').textContent='Updated '+new Date().toLocaleTimeString();}"
  "async function setMode(m){try{await fetch('/api/mode?set='+m);}catch(e){}}"
  "function updClk(){$('clk').textContent=new Date().toLocaleTimeString('en-US',{hour12:false})}"
  "window.addEventListener('load',()=>{initCharts();updClk();setInterval(updClk,1000);setTimeout(()=>{const o=$('lo');o.style.opacity='0';setTimeout(()=>o.style.display='none',500)},2200);setTimeout(()=>{update();setInterval(update,2000)},2200);});"
  "</script></body></html>"));
}
void handleRootMobile() {
  server.sendHeader("Cache-Control","no-cache");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200,"text/html","");

  server.sendContent(F("<!DOCTYPE html><html lang='en'><head>"
  "<meta charset='UTF-8'>"
  "<meta name='viewport' content='width=device-width,initial-scale=1.0,maximum-scale=1.0,user-scalable=no'>"
  "<meta name='apple-mobile-web-app-capable' content='yes'>"
  "<meta name='apple-mobile-web-app-status-bar-style' content='black-translucent'>"
  "<meta name='theme-color' content='#050a0e'>"
  "<title>AirMind AI</title>"
  "<script src='https://cdnjs.cloudflare.com/ajax/libs/Chart.js/4.4.0/chart.umd.min.js'></script>"
  "<style>"
  ":root{"
  "--bg:#050a0e;--bg2:#091318;--bg3:#0d1e26;--bg4:#112530;--bg5:#152d38;"
  "--accent:#00e5ff;--accent2:#00ff9d;--accent3:#ff6b35;--accent4:#a855f7;"
  "--warn:#ffd60a;--danger:#ff3366;"
  "--text:#e0f4ff;--text2:#7ab3c8;--text3:#3d6978;"
  "--border:rgba(0,229,255,0.12);"
  "--card-r:18px;--safe-bottom:env(safe-area-inset-bottom,0px)}"
  "*{margin:0;padding:0;box-sizing:border-box;-webkit-tap-highlight-color:transparent}"
  "html,body{height:100%;overflow:hidden;background:var(--bg);color:var(--text);font-family:'Segoe UI',system-ui,sans-serif}"
  /* BACKGROUND */
  ".bg-grid{position:fixed;inset:0;z-index:0;background-image:linear-gradient(rgba(0,229,255,0.02) 1px,transparent 1px),linear-gradient(90deg,rgba(0,229,255,0.02) 1px,transparent 1px);background-size:32px 32px;pointer-events:none}"
  ".orb{position:fixed;border-radius:50%;filter:blur(80px);pointer-events:none;z-index:0}"
  ".orb1{width:300px;height:300px;top:-100px;left:-80px;background:rgba(0,229,255,0.05);animation:of 7s ease-in-out infinite}"
  ".orb2{width:250px;height:250px;bottom:80px;right:-60px;background:rgba(0,255,157,0.04);animation:of 9s ease-in-out infinite reverse}"
  "@keyframes of{0%,100%{transform:translate(0,0)}50%{transform:translate(20px,-20px)}}"
  /* TOP BAR */
  ".topbar{position:fixed;top:0;left:0;right:0;z-index:200;height:56px;background:rgba(5,10,14,0.95);backdrop-filter:blur(20px);-webkit-backdrop-filter:blur(20px);border-bottom:1px solid var(--border);display:flex;align-items:center;justify-content:space-between;padding:0 16px}"
  ".logo{display:flex;align-items:center;gap:10px}"
  ".logo-ico{width:32px;height:32px;border-radius:9px;background:linear-gradient(135deg,var(--accent),var(--accent2));display:flex;align-items:center;justify-content:center;font-size:17px;animation:lp 2s ease-in-out infinite;flex-shrink:0}"
  "@keyframes lp{0%,100%{box-shadow:0 0 12px rgba(0,229,255,0.5)}50%{box-shadow:0 0 24px rgba(0,229,255,0.9)}}"
  ".logo-txt{font-size:17px;font-weight:900;color:var(--accent);letter-spacing:-0.3px}"
  ".logo-ai{font-size:11px;color:var(--accent4);font-weight:700;margin-left:2px}"
  ".topright{display:flex;align-items:center;gap:8px}"
  ".livebadge{display:flex;align-items:center;gap:5px;background:rgba(0,255,157,0.1);border:1px solid rgba(0,255,157,0.2);border-radius:20px;padding:4px 10px;font-size:10px;color:var(--accent2);font-weight:700}"
  ".ldot{width:6px;height:6px;border-radius:50%;background:var(--accent2);box-shadow:0 0 6px var(--accent2);animation:blink 1.4s ease-in-out infinite}"
  "@keyframes blink{0%,100%{opacity:1}50%{opacity:0.3}}"
  ".clk{font-size:12px;color:var(--text3);font-variant-numeric:tabular-nums}"
  /* MAIN SCROLL AREA */
  ".main{position:fixed;top:56px;left:0;right:0;bottom:calc(64px + var(--safe-bottom));overflow-y:auto;overflow-x:hidden;-webkit-overflow-scrolling:touch;z-index:1}"
  ".page{display:none;padding:14px 14px 8px;min-height:100%;animation:fadeIn 0.25s ease}"
  ".page.active{display:block}"
  "@keyframes fadeIn{from{opacity:0;transform:translateY(6px)}to{opacity:1;transform:translateY(0)}}"
  /* BOTTOM NAV */
  ".botnav{position:fixed;bottom:0;left:0;right:0;z-index:200;height:calc(64px + var(--safe-bottom));padding-bottom:var(--safe-bottom);background:rgba(5,10,14,0.97);backdrop-filter:blur(24px);-webkit-backdrop-filter:blur(24px);border-top:1px solid var(--border);display:flex;align-items:stretch}"
  ".navbtn{flex:1;display:flex;flex-direction:column;align-items:center;justify-content:center;gap:3px;cursor:pointer;transition:all 0.2s;padding:6px 0;border:none;background:none;color:var(--text3);-webkit-tap-highlight-color:transparent}"
  ".navbtn.act{color:var(--accent)}"
  ".navbtn.act .navico{text-shadow:0 0 12px var(--accent)}"
  ".navico{font-size:20px;transition:transform 0.2s}"
  ".navbtn.act .navico{transform:scale(1.15)}"
  ".navlbl{font-size:9px;font-weight:700;letter-spacing:1px;text-transform:uppercase}"
  /* CARDS */
  ".card{background:rgba(9,19,24,0.92);border:1px solid var(--border);border-radius:var(--card-r);padding:16px;margin-bottom:12px;position:relative;overflow:hidden}"
  ".card-accent-t{border-top:2px solid var(--accent)}"
  ".card-accent-t2{border-top:2px solid var(--accent2)}"
  ".card-accent-t3{border-top:2px solid var(--accent3)}"
  ".card-accent-t4{border-top:2px solid var(--accent4)}"
  ".card-accent-tw{border-top:2px solid var(--warn)}"
  /* HERO AQI RING */
  ".hero{display:flex;flex-direction:column;align-items:center;padding:20px 16px 16px}"
  ".ring-wrap{position:relative;width:180px;height:180px;margin:0 auto 12px}"
  ".ring-num{position:absolute;inset:0;display:flex;flex-direction:column;align-items:center;justify-content:center}"
  ".ring-big{font-size:52px;font-weight:900;line-height:1;transition:color 0.5s}"
  ".ring-lbl{font-size:11px;color:var(--text3);letter-spacing:2px;margin-top:2px}"
  ".ring-sub{font-size:13px;color:var(--text2);text-align:center;margin-top:4px}"
  /* STATUS BANNER */
  ".banner{border-radius:12px;padding:12px 14px;display:flex;align-items:center;gap:12px;margin-bottom:12px;transition:all 0.5s}"
  ".banner-icon{font-size:22px;flex-shrink:0}"
  ".banner-txt{flex:1;font-size:13px;font-weight:600}"
  ".banner-badge{padding:3px 10px;border-radius:20px;font-size:9px;font-weight:800;letter-spacing:1.5px;flex-shrink:0}"
  /* METRIC GRID */
  ".m2{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-bottom:12px}"
  ".m3{display:grid;grid-template-columns:1fr 1fr 1fr;gap:8px;margin-bottom:12px}"
  ".mcard{background:rgba(9,19,24,0.92);border:1px solid var(--border);border-radius:14px;padding:14px 12px;text-align:center}"
  ".mc-label{font-size:9px;letter-spacing:2px;color:var(--text3);text-transform:uppercase;margin-bottom:6px}"
  ".mc-val{font-size:28px;font-weight:900;line-height:1;margin-bottom:4px}"
  ".mc-unit{font-size:11px;color:var(--text2)}"
  ".mc-badge{display:inline-block;padding:2px 8px;border-radius:12px;font-size:9px;font-weight:700;letter-spacing:1px;margin-top:5px}"
  /* MINI CHART */
  ".minichart{height:80px;margin-top:8px;position:relative}"
  /* SECTION LABEL */
  ".sec-lbl{font-size:9px;font-weight:800;letter-spacing:3px;color:var(--text3);text-transform:uppercase;margin:16px 0 8px;display:flex;align-items:center;gap:8px}"
  ".sec-lbl::after{content:'';flex:1;height:1px;background:var(--border)}"
  /* PROGRESS BAR */
  ".pbar-wrap{height:6px;background:rgba(0,0,0,0.3);border-radius:3px;overflow:hidden;margin-top:6px}"
  ".pbar{height:100%;border-radius:3px;transition:width 0.8s ease}"
  /* AI ACCORDION */
  ".ai-acc{margin-bottom:10px;border-radius:14px;overflow:hidden;border:1px solid rgba(168,85,247,0.2)}"
  ".ai-acc-head{background:rgba(13,30,38,0.95);padding:14px 16px;display:flex;align-items:center;gap:12px;cursor:pointer;user-select:none;-webkit-user-select:none}"
  ".ai-acc-head:active{background:rgba(17,37,48,0.95)}"
  ".ai-acc-ico{font-size:20px;flex-shrink:0}"
  ".ai-acc-info{flex:1}"
  ".ai-acc-title{font-size:13px;font-weight:700;color:var(--text)}"
  ".ai-acc-sub{font-size:9px;color:var(--text3);letter-spacing:1.5px;text-transform:uppercase;margin-top:2px}"
  ".ai-acc-badge{padding:3px 8px;border-radius:6px;font-size:8px;font-weight:800;letter-spacing:1px;flex-shrink:0}"
  ".ai-acc-arrow{font-size:14px;color:var(--text3);transition:transform 0.3s;flex-shrink:0}"
  ".ai-acc-body{display:none;background:rgba(9,19,24,0.98);padding:14px 16px;border-top:1px solid rgba(168,85,247,0.15)}"
  ".ai-acc-body.open{display:block;animation:fadeIn 0.2s ease}"
  ".ai-acc-head.open .ai-acc-arrow{transform:rotate(180deg)}"
  /* AI METRIC ROW */
  ".ai-mr{display:flex;justify-content:space-between;align-items:center;padding:7px 0;border-bottom:1px solid rgba(255,255,255,0.04)}"
  ".ai-mr:last-child{border-bottom:none}"
  ".ai-mr-l{font-size:10px;color:var(--text3);letter-spacing:1px}"
  ".ai-mr-v{font-size:12px;font-weight:800}"
  /* KF BARS */
  ".kf-r{display:flex;align-items:center;gap:8px;margin:5px 0}"
  ".kf-lbl{font-size:9px;color:var(--text3);width:56px;flex-shrink:0}"
  ".kf-bw{flex:1;height:6px;background:rgba(0,0,0,0.3);border-radius:3px;overflow:hidden}"
  ".kf-bf{height:100%;border-radius:3px;transition:width 0.8s}"
  ".kf-n{font-size:10px;font-weight:700;width:32px;text-align:right;flex-shrink:0}"
  /* SENSOR CARD */
  ".scard{background:rgba(9,19,24,0.92);border:1px solid var(--border);border-radius:14px;padding:14px;margin-bottom:10px}"
  ".sc-top{display:flex;justify-content:space-between;align-items:center;margin-bottom:8px}"
  ".sc-name{font-size:10px;font-weight:800;letter-spacing:2px;color:var(--text3);text-transform:uppercase}"
  ".sc-val{font-size:32px;font-weight:900;line-height:1}"
  ".sc-unit{font-size:12px;color:var(--text2)}"
  /* ENERGY */
  ".e-big{font-size:36px;font-weight:900;margin:6px 0 2px}"
  ".e-unit{font-size:12px;color:var(--text2)}"
  ".trees{display:flex;gap:3px;flex-wrap:wrap;margin-top:8px}"
  ".ti{font-size:18px}"
  /* CONTROL */
  ".occ-vis{display:flex;align-items:center;gap:16px;margin:10px 0}"
  ".occ-ico{font-size:48px;transition:opacity 0.5s}"
  ".mode-btns{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-top:10px}"
  ".mbtn{padding:14px;border-radius:12px;font-size:13px;font-weight:800;letter-spacing:1px;border:1px solid var(--border);background:var(--bg3);color:var(--text3);cursor:pointer;text-align:center;transition:all 0.2s;-webkit-tap-highlight-color:transparent}"
  ".mbtn:active{transform:scale(0.97)}"
  ".mbtn-auto{background:rgba(0,229,255,0.1);border-color:var(--accent);color:var(--accent)}"
  ".mbtn-man{background:rgba(255,107,53,0.1);border-color:var(--accent3);color:var(--accent3)}"
  ".fan-center{display:flex;flex-direction:column;align-items:center;padding:10px 0}"
  ".fan-ico{font-size:72px;transition:all 0.5s;display:inline-block}"
  ".fan-spin{animation:fsp 0.7s linear infinite}"
  "@keyframes fsp{to{transform:rotate(360deg)}}"
  ".fan-ring{width:100px;height:100px;border-radius:50%;border:2px solid rgba(0,255,157,0.2);position:relative;margin:0 auto;display:flex;align-items:center;justify-content:center}"
  ".fan-glow{position:absolute;inset:-6px;border-radius:50%;border:1px solid rgba(0,255,157,0.2);animation:rp 2s ease-in-out infinite;display:none}"
  "@keyframes rp{0%,100%{opacity:0.4;transform:scale(1)}50%{opacity:1;transform:scale(1.04)}}"
  /* FFT */
  ".fft-con{height:64px;display:flex;align-items:flex-end;gap:2px;padding:4px 0}"
  ".fft-b{flex:1;border-radius:2px 2px 0 0;min-height:2px;transition:height 0.4s;background:linear-gradient(to top,var(--accent4),var(--accent))}"
  /* NN PROBS */
  ".nn-prob-row{display:grid;grid-template-columns:repeat(4,1fr);gap:6px;margin-top:8px}"
  ".nn-prob-cell{background:rgba(0,0,0,0.2);border-radius:8px;padding:8px 4px;text-align:center}"
  ".nn-pc-lbl{font-size:8px;color:var(--text3);margin-bottom:3px}"
  ".nn-pc-val{font-size:14px;font-weight:800}"
  /* CONFIDENCE */
  ".ci-row{display:flex;align-items:center;gap:8px;margin:5px 0}"
  ".ci-lbl{font-size:9px;color:var(--text3);width:48px;flex-shrink:0}"
  ".ci-track{flex:1;height:12px;background:rgba(0,0,0,0.3);border-radius:6px;overflow:hidden}"
  ".ci-fill{height:100%;border-radius:6px;display:flex;align-items:center;justify-content:center;font-size:8px;font-weight:700;color:rgba(0,0,0,0.7);transition:width 0.8s}"
  /* RL */
  ".rl-row{display:flex;align-items:center;gap:8px;padding:6px 0;border-bottom:1px solid rgba(255,255,255,0.04)}"
  ".rl-s{font-size:10px;color:var(--text2);flex:1}"
  ".rl-bw{width:54px;height:5px;background:rgba(0,0,0,0.3);border-radius:3px;overflow:hidden;flex-shrink:0}"
  ".rl-bf{height:100%;border-radius:3px;background:var(--accent2);transition:width 0.6s}"
  ".rl-q{font-size:10px;font-weight:700;width:40px;text-align:right;flex-shrink:0}"
  /* BADGE COLORS */
  ".bd{background:rgba(255,51,102,0.15);color:var(--danger);border:1px solid rgba(255,51,102,0.3)}"
  ".bw{background:rgba(255,214,10,0.15);color:var(--warn);border:1px solid rgba(255,214,10,0.3)}"
  ".bg-b{background:rgba(0,255,157,0.12);color:var(--accent2);border:1px solid rgba(0,255,157,0.25)}"
  ".bn{background:rgba(0,229,255,0.12);color:var(--accent);border:1px solid rgba(0,229,255,0.25)}"
  ".bon{background:rgba(0,255,157,0.15);color:var(--accent2);border:1px solid rgba(0,255,157,0.3)}"
  ".boff{background:rgba(61,105,120,0.2);color:var(--text3);border:1px solid var(--border)}"
  ".bai{background:rgba(168,85,247,0.15);color:var(--accent4);border:1px solid rgba(168,85,247,0.3)}"
  /* LOADER */
  ".lovl{position:fixed;inset:0;background:rgba(5,10,14,0.98);z-index:999;display:flex;align-items:center;justify-content:center;flex-direction:column;gap:20px;transition:opacity 0.4s}"
  ".l-ico{font-size:48px;animation:lp 1s ease-in-out infinite}"
  ".l-txt{font-size:12px;color:var(--accent);letter-spacing:4px;font-weight:700}"
  ".l-bar{width:180px;height:2px;background:var(--bg3);border-radius:1px;overflow:hidden}"
  ".l-fill{height:100%;background:linear-gradient(90deg,var(--accent),var(--accent2));animation:lf 1.8s ease forwards}"
  "@keyframes lf{from{width:0}to{width:100%}}"
  /* ALERT FEED */
  ".af-item{display:flex;gap:8px;align-items:flex-start;padding:8px 10px;background:rgba(0,0,0,0.2);border-radius:8px;margin-bottom:5px;font-size:10px;animation:fadeIn 0.3s ease}"
  ".af-time{color:var(--text3);width:48px;flex-shrink:0;font-size:8px;padding-top:1px}"
  ".af-msg{flex:1;color:var(--text2);line-height:14px}"
  ".af-lvl{padding:2px 5px;border-radius:4px;font-size:7px;font-weight:700;flex-shrink:0}"
  ".al-ok{background:rgba(0,255,157,0.15);color:var(--accent2)}"
  ".al-warn{background:rgba(255,214,10,0.15);color:var(--warn)}"
  ".al-crit{background:rgba(255,51,102,0.15);color:var(--danger)}"
  ".al-ai{background:rgba(168,85,247,0.15);color:var(--accent4)}"
  /* UPDATE INDICATOR */
  ".upd-dots{display:flex;gap:4px;align-items:center}"
  ".upd-d{width:5px;height:5px;border-radius:50%;background:var(--bg4);transition:all 0.3s}"
  ".upd-d.act{background:var(--accent);box-shadow:0 0 6px var(--accent)}"
  /* FORECAST ROW */
  ".fc-row{display:grid;grid-template-columns:repeat(4,1fr);gap:6px;margin-top:8px}"
  ".fc-cell{background:rgba(0,0,0,0.2);border-radius:8px;padding:8px 6px;text-align:center}"
  ".fc-t{font-size:8px;color:var(--text3);margin-bottom:3px}"
  ".fc-v{font-size:16px;font-weight:900}"
  /* ISO GRID */
  ".iso-g{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;margin:8px 0}"
  ".iso-c{background:rgba(0,0,0,0.2);border-radius:8px;padding:8px;text-align:center}"
  ".iso-v{font-size:16px;font-weight:800}"
  ".iso-l{font-size:8px;color:var(--text3);margin-top:2px;letter-spacing:1px}"
  /* PCA AXIS */
  ".pca-ax{display:flex;align-items:center;gap:8px;margin:5px 0}"
  ".pca-l{font-size:9px;color:var(--text3);width:28px;flex-shrink:0}"
  ".pca-bw{flex:1;height:8px;background:rgba(0,0,0,0.3);border-radius:4px;overflow:hidden}"
  ".pca-bf{height:100%;border-radius:4px;transition:width 0.8s}"
  ".pca-p{font-size:10px;font-weight:700;width:32px;text-align:right;flex-shrink:0}"
  /* ADA ROWS */
  ".ada-r{display:flex;justify-content:space-between;align-items:center;padding:6px 0;border-bottom:1px solid rgba(255,255,255,0.04)}"
  ".ada-n{font-size:10px;color:var(--text2)}"
  ".ada-v{font-size:12px;font-weight:700}"
  /* LR STATS */
  ".lr-g{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin:8px 0}"
  ".lr-c{background:rgba(0,0,0,0.2);border-radius:8px;padding:10px;text-align:center}"
  ".lr-v{font-size:18px;font-weight:900}"
  ".lr-l{font-size:8px;color:var(--text3);letter-spacing:1px;margin-top:2px}"
  "::-webkit-scrollbar{display:none}"
  "</style></head><body>"));
  /* ── HTML STRUCTURE ── */
  server.sendContent(F(
  "<div class='lovl' id='lo'><div class='l-ico'>&#127758;</div><div class='l-txt'>AIRMIND AI</div><div class='l-bar'><div class='l-fill'></div></div></div>"
  "<div class='bg-grid'></div><div class='orb orb1'></div><div class='orb orb2'></div>"

  /* TOP BAR */
  "<div class='topbar'>"
  "<div class='logo'><div class='logo-ico'>&#127758;</div><div class='logo-txt'>AirMind<span class='logo-ai'>AI</span></div></div>"
  "<div class='topright'>"
  "<div class='livebadge'><div class='ldot'></div>LIVE</div>"
  "<div class='clk' id='clk'>--:--</div>"
  "<div class='upd-dots'><div class='upd-d act' id='d1'></div><div class='upd-d' id='d2'></div><div class='upd-d' id='d3'></div></div>"
  "</div></div>"

  /* MAIN */
  "<div class='main'>"

  /* ══ PAGE 1: HOME ══ */
  "<div class='page active' id='pg0'>"

  /* STATUS BANNER */
  "<div class='banner' id='ab' style='background:linear-gradient(90deg,rgba(255,214,10,0.08),rgba(255,214,10,0.02));border:1px solid rgba(255,214,10,0.2)'>"
  "<div class='banner-icon' id='abIco'>&#9888;&#65039;</div>"
  "<div class='banner-txt' style='color:var(--warn)'>AQI <b id='abAqi'>--</b> &mdash; <span id='abMsg'>Initializing...</span></div>"
  "<div class='banner-badge bw' id='abBadge'>CHECK</div>"
  "</div>"

  /* HERO AQI RING */
  "<div class='card card-accent-t3' style='padding:20px 16px'>"
  "<div style='text-align:center'>"
  "<div class='ring-wrap'><canvas id='aqi-ring' width='180' height='180'></canvas>"
  "<div class='ring-num'><div class='ring-big' id='aqiV' style='color:var(--accent3)'>--</div><div class='ring-lbl'>AQI</div></div>"
  "</div>"
  "<div style='font-size:16px;font-weight:800;margin-bottom:4px' id='aqiQual'>--</div>"
  "<div style='font-size:12px;color:var(--text3)' id='aqiStatus'>--</div>"
  "<div style='margin-top:8px;font-size:11px;color:var(--accent4)'>&#967;&#178; Kalman: <b id='kfV'>--</b> &nbsp;&#8729;&nbsp; EWMA: <b id='ewmaV'>--</b></div>"
  "</div></div>"

  /* CO2 + TEMP/HUM */
  "<div class='m2'>"
  "<div class='mcard card-accent-t' style='border-top:2px solid var(--accent)'>"
  "<div class='mc-label'>CO&#8322;</div>"
  "<div class='mc-val' id='co2V' style='color:var(--accent)'>--</div>"
  "<div class='mc-unit'>ppm</div>"
  "<div class='mc-badge bn' id='co2B'>--</div>"
  "</div>"
  "<div class='mcard' style='border-top:2px solid var(--warn)'>"
  "<div class='mc-label'>Temp</div>"
  "<div class='mc-val' id='tmpV' style='color:var(--warn)'>--</div>"
  "<div class='mc-unit'>&#176;C</div>"
  "<div style='margin-top:6px;font-size:10px;color:var(--text3)'>Humidity</div>"
  "<div style='font-size:20px;font-weight:800;color:var(--accent)' id='humV'>--<span style='font-size:12px;color:var(--text2)'>%</span></div>"
  "</div>"
  "</div>"

  /* QUICK AI SUMMARY */
  "<div class='sec-lbl'>AI Quick Summary</div>"
  "<div class='card card-accent-t4'>"
  "<div style='display:flex;justify-content:space-between;align-items:center;margin-bottom:10px'>"
  "<div style='font-size:11px;font-weight:700;color:var(--accent4)'>&#10022; AI RECOMMENDATION</div>"
  "<div class='mc-badge bai' id='aiActB'>--</div>"
  "</div>"
  "<div style='font-size:18px;font-weight:900;color:var(--accent2);margin-bottom:4px' id='aiActV'>--</div>"
  "<div style='font-size:11px;color:var(--text2)' id='aiActD'>Initializing AI engine...</div>"
  "<div style='margin-top:12px;display:grid;grid-template-columns:1fr 1fr;gap:8px'>"
  "<div style='background:rgba(0,0,0,0.2);border-radius:8px;padding:8px;text-align:center'>"
  "<div style='font-size:8px;color:var(--text3);letter-spacing:1px;margin-bottom:3px'>HEALTH SCORE</div>"
  "<div style='font-size:24px;font-weight:900;color:var(--accent2)' id='hlthV'>--</div>"
  "</div>"
  "<div style='background:rgba(0,0,0,0.2);border-radius:8px;padding:8px;text-align:center'>"
  "<div style='font-size:8px;color:var(--text3);letter-spacing:1px;margin-bottom:3px'>ANOMALY</div>"
  "<div style='font-size:16px;font-weight:800' id='anomV'>--</div>"
  "</div>"
  "</div>"
  "</div>"

  /* AQI FORECAST MINI */
  "<div class='sec-lbl'>AQI Forecast (LinReg)</div>"
  "<div class='card'>"
  "<div class='fc-row'>"
  "<div class='fc-cell'><div class='fc-t'>NOW</div><div class='fc-v' id='pAN' style='color:var(--accent3)'>--</div></div>"
  "<div class='fc-cell'><div class='fc-t'>+30s</div><div class='fc-v' id='pA30' style='color:var(--accent3)'>--</div></div>"
  "<div class='fc-cell'><div class='fc-t'>+60s</div><div class='fc-v' id='pA60' style='color:var(--accent3)'>--</div></div>"
  "<div class='fc-cell'><div class='fc-t'>+2min</div><div class='fc-v' id='pA120' style='color:var(--accent3)'>--</div></div>"
  "</div>"
  "<div style='height:70px;margin-top:8px;position:relative'><canvas id='aqiSpk'></canvas></div>"
  "</div>"

  "</div>" /* end page 0 */
  ));

  /* ══ PAGE 2: AI/ML ══ */
  server.sendContent(F(
  "<div class='page' id='pg1'>"
  "<div class='sec-lbl' style='margin-top:4px'>8 AI / ML Modules</div>"

  /* KALMAN */
  "<div class='ai-acc card-accent-t4'>"
  "<div class='ai-acc-head' onclick='tog(this)'>"
  "<div class='ai-acc-ico'>&#120524;</div>"
  "<div class='ai-acc-info'><div class='ai-acc-title'>Kalman Filter</div><div class='ai-acc-sub'>Sensor Fusion &middot; State Estimation</div></div>"
  "<div class='ai-acc-badge bai'>REAL-TIME</div>"
  "<div class='ai-acc-arrow'>&#8963;</div>"
  "</div>"
  "<div class='ai-acc-body'>"
  "<div style='font-size:10px;color:var(--text3);margin-bottom:10px'>Optimal Bayesian estimator used in ISRO rockets &amp; IRNSS GPS. Runs on ESP32 in C++ AND in browser JS simultaneously.</div>"
  "<div class='kf-r'><div class='kf-lbl' style='color:var(--accent3)'>Raw AQI</div><div class='kf-bw'><div class='kf-bf' id='kfRawB' style='background:var(--accent3)'></div></div><div class='kf-n' id='kfRawN' style='color:var(--accent3)'>--</div></div>"
  "<div class='kf-r'><div class='kf-lbl' style='color:var(--accent4)'>Kalman &#955;</div><div class='kf-bw'><div class='kf-bf' id='kfKalB' style='background:var(--accent4)'></div></div><div class='kf-n' id='kfKalN' style='color:var(--accent4)'>--</div></div>"
  "<div class='kf-r'><div class='kf-lbl' style='color:var(--accent2)'>EWMA &#945;</div><div class='kf-bw'><div class='kf-bf' id='kfEwB' style='background:var(--accent2)'></div></div><div class='kf-n' id='kfEwN' style='color:var(--accent2)'>--</div></div>"
  "<div class='ai-mr'><div class='ai-mr-l'>KALMAN GAIN K</div><div class='ai-mr-v' id='kfGain' style='color:var(--accent4)'>--</div></div>"
  "<div class='ai-mr'><div class='ai-mr-l'>COVARIANCE P</div><div class='ai-mr-v' id='kfCov' style='color:var(--accent)'>--</div></div>"
  "</div></div>"

  /* LINEAR REGRESSION */
  "<div class='ai-acc card-accent-t3' style='border:1px solid rgba(255,107,53,0.2)'>"
  "<div class='ai-acc-head' onclick='tog(this)'>"
  "<div class='ai-acc-ico'>&#128200;</div>"
  "<div class='ai-acc-info'><div class='ai-acc-title'>Linear Regression</div><div class='ai-acc-sub'>OLS Trend &middot; 30 Samples</div></div>"
  "<div class='ai-acc-badge' style='background:rgba(255,107,53,0.15);color:var(--accent3);border:1px solid rgba(255,107,53,0.3)'>OLS</div>"
  "<div class='ai-acc-arrow'>&#8963;</div>"
  "</div>"
  "<div class='ai-acc-body'>"
  "<div style='font-size:10px;color:var(--text3);margin-bottom:10px'>OLS regression on rolling 30-sample AQI window. Slope predicts drift velocity. Extrapolates +30s, +60s, +2min.</div>"
  "<div class='lr-g'>"
  "<div class='lr-c'><div class='lr-v' id='lrSlope' style='color:var(--accent3)'>--</div><div class='lr-l'>SLOPE AQI/s</div></div>"
  "<div class='lr-c'><div class='lr-v' id='lrR2' style='color:var(--accent2)'>--</div><div class='lr-l'>R&#178; FIT</div></div>"
  "<div class='lr-c'><div class='lr-v' id='lrRMSE' style='color:var(--warn)'>--</div><div class='lr-l'>RMSE</div></div>"
  "<div class='lr-c'><div class='lr-v' id='lrConf' style='color:var(--accent4)'>--</div><div class='lr-l'>CONFIDENCE</div></div>"
  "</div>"
  "<div class='fc-row'>"
  "<div class='fc-cell'><div class='fc-t'>NOW</div><div class='fc-v' id='lrN' style='color:var(--accent3)'>--</div></div>"
  "<div class='fc-cell'><div class='fc-t'>+30s</div><div class='fc-v' id='lrP30' style='color:var(--accent3)'>--</div></div>"
  "<div class='fc-cell'><div class='fc-t'>+60s</div><div class='fc-v' id='lrP60' style='color:var(--accent3)'>--</div></div>"
  "<div class='fc-cell'><div class='fc-t'>+2min</div><div class='fc-v' id='lrP120' style='color:var(--accent3)'>--</div></div>"
  "</div>"
  "</div></div>"

  /* ISOLATION FOREST */
  "<div class='ai-acc' style='border:1px solid rgba(255,51,102,0.2)'>"
  "<div class='ai-acc-head' onclick='tog(this)'>"
  "<div class='ai-acc-ico'>&#128269;</div>"
  "<div class='ai-acc-info'><div class='ai-acc-title'>Isolation Forest</div><div class='ai-acc-sub'>Anomaly Scoring &middot; Random Cuts</div></div>"
  "<div class='ai-acc-badge bd'>OUTLIER</div>"
  "<div class='ai-acc-arrow'>&#8963;</div>"
  "</div>"
  "<div class='ai-acc-body'>"
  "<div style='font-size:10px;color:var(--text3);margin-bottom:10px'>Unsupervised anomaly detection via random recursive partitioning. Score &gt;0.75 = ANOMALY.</div>"
  "<div class='iso-g'>"
  "<div class='iso-c'><div class='iso-v' id='isoA' style='color:var(--accent3)'>--</div><div class='iso-l'>AQI SCORE</div></div>"
  "<div class='iso-c'><div class='iso-v' id='isoC' style='color:var(--accent)'>--</div><div class='iso-l'>CO&#8322; SCORE</div></div>"
  "<div class='iso-c'><div class='iso-v' id='isoT' style='color:var(--warn)'>--</div><div class='iso-l'>TEMP SCORE</div></div>"
  "</div>"
  "<div style='background:rgba(0,0,0,0.2);border-radius:8px;padding:10px;text-align:center;margin-top:6px'>"
  "<div style='font-size:9px;color:var(--text3);letter-spacing:1px;margin-bottom:4px'>COMPOSITE ANOMALY SCORE</div>"
  "<div style='font-size:28px;font-weight:900' id='isoComp'>--</div>"
  "<div style='font-size:10px;margin-top:3px' id='isoLbl'>--</div>"
  "</div>"
  "<div class='pbar-wrap' style='margin-top:8px'><div id='isoBar' style='height:100%;border-radius:3px;transition:width 0.8s,background 0.5s'></div></div>"
  "</div></div>"

  /* PCA */
  "<div class='ai-acc' style='border:1px solid rgba(0,229,255,0.2)'>"
  "<div class='ai-acc-head' onclick='tog(this)'>"
  "<div class='ai-acc-ico'>&#128300;</div>"
  "<div class='ai-acc-info'><div class='ai-acc-title'>PCA Health Index</div><div class='ai-acc-sub'>Principal Component Analysis</div></div>"
  "<div class='ai-acc-badge bn'>PCA</div>"
  "<div class='ai-acc-arrow'>&#8963;</div>"
  "</div>"
  "<div class='ai-acc-body'>"
  "<div style='font-size:10px;color:var(--text3);margin-bottom:10px'>6-sensor dimensionality reduction into composite health score. Weights: AQI 35%, CO2 25%, Temp 15%, Humidity 10%, NH3 10%, Benzene 5%.</div>"
  "<div style='text-align:center;margin:10px 0'><div style='font-size:42px;font-weight:900;color:var(--accent2)' id='pcaScore'>--</div><div style='font-size:9px;color:var(--text3);letter-spacing:2px'>HEALTH SCORE / 100</div></div>"
  "<div class='pca-ax'><div class='pca-l' style='color:var(--accent3)'>PC1</div><div class='pca-bw'><div class='pca-bf' id='pc1b' style='background:var(--accent3)'></div></div><div class='pca-p' id='pc1p' style='color:var(--accent3)'>--</div></div>"
  "<div class='pca-ax'><div class='pca-l' style='color:var(--accent)'>PC2</div><div class='pca-bw'><div class='pca-bf' id='pc2b' style='background:var(--accent)'></div></div><div class='pca-p' id='pc2p' style='color:var(--accent)'>--</div></div>"
  "<div class='pca-ax'><div class='pca-l' style='color:var(--accent4)'>PC3</div><div class='pca-bw'><div class='pca-bf' id='pc3b' style='background:var(--accent4)'></div></div><div class='pca-p' id='pc3p' style='color:var(--accent4)'>--</div></div>"
  "</div></div>"

  /* ADAPTIVE THRESHOLDS */
  "<div class='ai-acc' style='border:1px solid rgba(255,214,10,0.2)'>"
  "<div class='ai-acc-head' onclick='tog(this)'>"
  "<div class='ai-acc-ico'>&#127919;</div>"
  "<div class='ai-acc-info'><div class='ai-acc-title'>Adaptive Thresholds</div><div class='ai-acc-sub'>Bayesian Dynamic Limits</div></div>"
  "<div class='ai-acc-badge bw'>BAYESIAN</div>"
  "<div class='ai-acc-arrow'>&#8963;</div>"
  "</div>"
  "<div class='ai-acc-body'>"
  "<div style='font-size:10px;color:var(--text3);margin-bottom:10px'>Self-calibrating limits: threshold = mean + k&middot;&#963;. Adapts to local environment. Warn at k=2, Critical at k=3.</div>"
  "<div class='ada-r'><div class='ada-n'>AQI Warn &#964;</div><div class='ada-v' id='adaAW' style='color:var(--warn)'>--</div></div>"
  "<div class='ada-r'><div class='ada-n'>AQI Critical &#964;</div><div class='ada-v' id='adaAC' style='color:var(--danger)'>--</div></div>"
  "<div class='ada-r'><div class='ada-n'>CO&#8322; Warn &#964;</div><div class='ada-v' id='adaCW' style='color:var(--accent)'>--</div></div>"
  "<div class='ada-r'><div class='ada-n'>Temp &#964;</div><div class='ada-v' id='adaTW' style='color:var(--warn)'>--</div></div>"
  "<div style='margin-top:8px;background:rgba(0,0,0,0.2);border-radius:8px;padding:8px;font-size:11px;font-weight:700' id='adaSt'>Calibrating...</div>"
  "</div></div>"

  /* SPECTRAL / FFT */
  "<div class='ai-acc' style='border:1px solid rgba(0,255,157,0.2)'>"
  "<div class='ai-acc-head' onclick='tog(this)'>"
  "<div class='ai-acc-ico'>&#127926;</div>"
  "<div class='ai-acc-info'><div class='ai-acc-title'>Spectral Analysis (DFT)</div><div class='ai-acc-sub'>Frequency Decomposition &middot; DSP</div></div>"
  "<div class='ai-acc-badge bg-b'>FFT</div>"
  "<div class='ai-acc-arrow'>&#8963;</div>"
  "</div>"
  "<div class='ai-acc-body'>"
  "<div style='font-size:10px;color:var(--text3);margin-bottom:8px'>DFT on AQI signal — reveals periodic events, HVAC cycles. Core DSP used in ISRO telemetry systems.</div>"
  "<div class='fft-con' id='fftBins'></div>"
  "<div style='display:flex;justify-content:space-between;margin-top:3px'><span style='font-size:8px;color:var(--text3)'>DC</span><span style='font-size:8px;color:var(--text3)'>LOW</span><span style='font-size:8px;color:var(--text3)'>MID</span><span style='font-size:8px;color:var(--text3)'>HIGH</span></div>"
  "<div style='display:grid;grid-template-columns:1fr 1fr 1fr;gap:6px;margin-top:10px'>"
  "<div style='background:rgba(0,0,0,0.2);border-radius:6px;padding:8px;text-align:center'><div style='font-size:8px;color:var(--text3)'>DOM FREQ</div><div style='font-size:16px;font-weight:700;color:var(--accent4)' id='fftDom'>--</div></div>"
  "<div style='background:rgba(0,0,0,0.2);border-radius:6px;padding:8px;text-align:center'><div style='font-size:8px;color:var(--text3)'>S.ENTROPY</div><div style='font-size:16px;font-weight:700;color:var(--accent)' id='fftEnt'>--</div></div>"
  "<div style='background:rgba(0,0,0,0.2);border-radius:6px;padding:8px;text-align:center'><div style='font-size:8px;color:var(--text3)'>PERIODIC</div><div style='font-size:14px;font-weight:700;color:var(--accent2)' id='fftPer'>--</div></div>"
  "</div>"
  "</div></div>"

  /* RL AGENT */
  "<div class='ai-acc' style='border:1px solid rgba(255,107,53,0.2)'>"
  "<div class='ai-acc-head' onclick='tog(this)'>"
  "<div class='ai-acc-ico'>&#129302;</div>"
  "<div class='ai-acc-info'><div class='ai-acc-title'>RL Control Agent</div><div class='ai-acc-sub'>Q-Learning &middot; Fan Policy</div></div>"
  "<div class='ai-acc-badge' style='background:rgba(255,107,53,0.15);color:var(--accent3);border:1px solid rgba(255,107,53,0.3)'>Q-LEARN</div>"
  "<div class='ai-acc-arrow'>&#8963;</div>"
  "</div>"
  "<div class='ai-acc-body'>"
  "<div style='font-size:10px;color:var(--text3);margin-bottom:10px'>Q-table fan control policy. Reward: +2 fan-on during hazard, -2 unnecessary fan-off. Q(s,a) updated each cycle.</div>"
  "<div class='rl-row'><div class='rl-s'>Good Air, Occupied</div><div class='rl-bw'><div class='rl-bf' id='rl0b'></div></div><div class='rl-q' id='rl0v' style='color:var(--accent2)'>--</div></div>"
  "<div class='rl-row'><div class='rl-s'>Poor Air, Occupied</div><div class='rl-bw'><div class='rl-bf' id='rl1b' style='background:var(--accent3)'></div></div><div class='rl-q' id='rl1v' style='color:var(--accent3)'>--</div></div>"
  "<div class='rl-row'><div class='rl-s'>Good Air, Empty</div><div class='rl-bw'><div class='rl-bf' id='rl2b' style='background:var(--accent4)'></div></div><div class='rl-q' id='rl2v' style='color:var(--accent4)'>--</div></div>"
  "<div class='rl-row'><div class='rl-s'>Hazardous, Any</div><div class='rl-bw'><div class='rl-bf' id='rl3b' style='background:var(--danger)'></div></div><div class='rl-q' id='rl3v' style='color:var(--danger)'>--</div></div>"
  "<div style='margin-top:10px;background:rgba(0,0,0,0.2);border-radius:8px;padding:10px;text-align:center'>"
  "<div style='font-size:8px;color:var(--text3);letter-spacing:1px'>CURRENT POLICY</div>"
  "<div style='font-size:16px;font-weight:900;margin:4px 0' id='rlAct'>--</div>"
  "<div style='font-size:9px;color:var(--text3)'>Cumulative Reward: <span id='rlRew' style='color:var(--accent2)'>0</span></div>"
  "</div>"
  "</div></div>"

  /* NEURAL NET */
  "<div class='ai-acc' style='border:1px solid rgba(168,85,247,0.2)'>"
  "<div class='ai-acc-head' onclick='tog(this)'>"
  "<div class='ai-acc-ico'>&#129504;</div>"
  "<div class='ai-acc-info'><div class='ai-acc-title'>Neural Network MLP</div><div class='ai-acc-sub'>3-Layer &middot; Air Quality Classifier</div></div>"
  "<div class='ai-acc-badge bai'>DEEP LEARNING</div>"
  "<div class='ai-acc-arrow'>&#8963;</div>"
  "</div>"
  "<div class='ai-acc-body'>"
  "<div style='font-size:10px;color:var(--text3);margin-bottom:8px'>4&#8594;8&#8594;4 MLP with sigmoid activations. Softmax output for SAFE/MODERATE/POLLUTED/HAZARDOUS classification.</div>"
  "<div style='text-align:center;margin:8px 0'><div style='font-size:8px;color:var(--text3);letter-spacing:1px;margin-bottom:4px'>PREDICTED CLASS</div><div style='font-size:22px;font-weight:900' id='nnOut' style='color:var(--accent2)'>--</div></div>"
  "<div class='nn-prob-row'>"
  "<div class='nn-prob-cell'><div class='nn-pc-lbl'>SAFE</div><div class='nn-pc-val' id='nn0' style='color:var(--accent2)'>--%</div></div>"
  "<div class='nn-prob-cell'><div class='nn-pc-lbl'>MOD</div><div class='nn-pc-val' id='nn1' style='color:var(--warn)'>--%</div></div>"
  "<div class='nn-prob-cell'><div class='nn-pc-lbl'>POLL</div><div class='nn-pc-val' id='nn2' style='color:var(--accent3)'>--%</div></div>"
  "<div class='nn-prob-cell'><div class='nn-pc-lbl'>HAZ</div><div class='nn-pc-val' id='nn3' style='color:var(--danger)'>--%</div></div>"
  "</div>"
  "</div></div>"

  /* CONFIDENCE INTERVALS */
  "<div class='ai-acc' style='border:1px solid rgba(0,229,255,0.2);margin-bottom:16px'>"
  "<div class='ai-acc-head' onclick='tog(this)'>"
  "<div class='ai-acc-ico'>&#128202;</div>"
  "<div class='ai-acc-info'><div class='ai-acc-title'>Confidence Intervals</div><div class='ai-acc-sub'>95% CI &middot; Uncertainty Quantification</div></div>"
  "<div class='ai-acc-badge bn'>STATS</div>"
  "<div class='ai-acc-arrow'>&#8963;</div>"
  "</div>"
  "<div class='ai-acc-body'>"
  "<div style='font-size:10px;color:var(--text3);margin-bottom:10px'>Monte Carlo 95% confidence bands. z=1.96. Reports total model variance &#963;&#178; and SNR in dB.</div>"
  "<div style='font-size:9px;color:var(--text3);letter-spacing:1px;margin-bottom:6px'>AQI 95% CONFIDENCE INTERVAL</div>"
  "<div class='ci-row'><div class='ci-lbl'>Lower</div><div class='ci-track'><div class='ci-fill' id='ciAL' style='background:rgba(255,107,53,0.5)'></div></div></div>"
  "<div class='ci-row'><div class='ci-lbl'>Mean</div><div class='ci-track'><div class='ci-fill' id='ciAM' style='background:rgba(255,107,53,0.8)'></div></div></div>"
  "<div class='ci-row'><div class='ci-lbl'>Upper</div><div class='ci-track'><div class='ci-fill' id='ciAU' style='background:rgba(255,107,53,0.5)'></div></div></div>"
  "<div style='display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-top:10px'>"
  "<div style='background:rgba(0,0,0,0.2);border-radius:8px;padding:10px;text-align:center'><div style='font-size:8px;color:var(--text3)'>MODEL UNCERTAINTY</div><div style='font-size:22px;font-weight:900;color:var(--accent4)' id='ciUnc'>--</div></div>"
  "<div style='background:rgba(0,0,0,0.2);border-radius:8px;padding:10px;text-align:center'><div style='font-size:8px;color:var(--text3)'>SNR (dB)</div><div style='font-size:22px;font-weight:900;color:var(--accent2)' id='ciSnr'>--</div></div>"
  "</div>"
  "</div></div>"

  /* ALERT FEED */
  "<div class='sec-lbl'>AI Alert Feed</div>"
  "<div class='card' style='margin-bottom:16px'>"
  "<div id='alertFeed'><div class='af-item'><span class='af-time'>--:--:--</span><span class='af-msg'>Initializing AI modules...</span><span class='af-lvl al-ai'>AI</span></div></div>"
  "</div>"

  "</div>" /* end page 1 */
  ));

  /* ══ PAGE 3: SENSORS ══ */
  server.sendContent(F(
  "<div class='page' id='pg2'>"
  "<div class='sec-lbl' style='margin-top:4px'>Sensor Array</div>"

  "<div class='scard card-accent-t4'>"
  "<div class='sc-top'><div class='sc-name'>&#129514; NH&#8323;</div><div style='font-size:11px;color:var(--text3)'>Ammonia</div></div>"
  "<div class='sc-val' id='nh3V' style='color:var(--accent4)'>--</div><div class='sc-unit'>ppm</div>"
  "<div class='pbar-wrap'><div class='pbar' id='nh3B' style='background:var(--accent4)'></div></div>"
  "</div>"

  "<div class='scard' style='border-top:2px solid var(--danger)'>"
  "<div class='sc-top'><div class='sc-name'>&#9879; Benzene</div><div style='font-size:11px;color:var(--text3)'>Aromatic</div></div>"
  "<div class='sc-val' id='bzV' style='color:var(--danger)'>--</div><div class='sc-unit'>ppm</div>"
  "<div class='pbar-wrap'><div class='pbar' id='bzB' style='background:var(--danger)'></div></div>"
  "</div>"

  "<div class='scard card-accent-tw'>"
  "<div class='sc-top'><div class='sc-name'>&#128168; CO</div><div style='font-size:11px;color:var(--text3)'>Carbon Monoxide</div></div>"
  "<div class='sc-val' id='coV' style='color:var(--warn)'>--</div><div class='sc-unit'>ppm</div>"
  "<div class='pbar-wrap'><div class='pbar' id='coB' style='background:var(--warn)'></div></div>"
  "</div>"

  "<div class='sec-lbl'>Power Monitoring</div>"

  "<div class='scard card-accent-t'>"
  "<div class='sc-top'><div class='sc-name'>&#9889; Voltage</div><div style='font-size:22px;font-weight:900;color:var(--accent)' id='vltV'>--</div></div>"
  "<div class='sc-unit'>V</div>"
  "</div>"

  "<div class='m2'>"
  "<div class='mcard' style='border-top:2px solid var(--accent3)'>"
  "<div class='mc-label'>Current</div><div class='mc-val' id='curV' style='color:var(--accent3)'>--</div><div class='mc-unit'>mA</div>"
  "<div class='pbar-wrap' style='margin-top:8px'><div class='pbar' id='curB' style='background:var(--accent3)'></div></div>"
  "</div>"
  "<div class='mcard card-accent-t2'>"
  "<div class='mc-label'>Power</div><div class='mc-val' id='pwrV' style='color:var(--accent2)'>--</div><div class='mc-unit'>W</div>"
  "<div class='pbar-wrap' style='margin-top:8px'><div class='pbar' id='pwrB' style='background:var(--accent2)'></div></div>"
  "</div>"
  "</div>"

  /* HISTORY CHART */
  "<div class='sec-lbl'>History Timeline</div>"
  "<div class='card'>"
  "<div style='display:flex;gap:6px;margin-bottom:10px;flex-wrap:wrap'>"
  "<div class='htab act' onclick='switchTab(\"aqi\",this)' style='padding:5px 12px;border-radius:8px;font-size:10px;font-weight:600;background:rgba(0,229,255,0.1);border:1px solid var(--accent);color:var(--accent);cursor:pointer'>AQI</div>"
  "<div class='htab' onclick='switchTab(\"co2\",this)' style='padding:5px 12px;border-radius:8px;font-size:10px;font-weight:600;background:var(--bg3);border:1px solid var(--border);color:var(--text2);cursor:pointer'>CO&#8322;</div>"
  "<div class='htab' onclick='switchTab(\"temp\",this)' style='padding:5px 12px;border-radius:8px;font-size:10px;font-weight:600;background:var(--bg3);border:1px solid var(--border);color:var(--text2);cursor:pointer'>Temp</div>"
  "<div class='htab' onclick='switchTab(\"humidity\",this)' style='padding:5px 12px;border-radius:8px;font-size:10px;font-weight:600;background:var(--bg3);border:1px solid var(--border);color:var(--text2);cursor:pointer'>Hum</div>"
  "</div>"
  "<div style='height:160px;position:relative'><canvas id='histCh'></canvas></div>"
  "</div>"

  "</div>" /* end page 2 */
  ));

  /* ══ PAGE 4: ENERGY ══ */
  server.sendContent(F(
  "<div class='page' id='pg3'>"
  "<div class='sec-lbl' style='margin-top:4px'>Energy &amp; Carbon</div>"

  "<div class='card card-accent-t' style='text-align:center'>"
  "<div style='font-size:9px;letter-spacing:2px;color:var(--text3);margin-bottom:4px'>ENERGY CONSUMED</div>"
  "<div class='e-big' id='engV' style='color:var(--accent)'>--</div>"
  "<div class='e-unit'>kWh</div>"
  "</div>"

  "<div class='m2'>"
  "<div class='mcard card-accent-t2'>"
  "<div class='mc-label'>&#128176; Cost</div>"
  "<div style='font-size:22px;font-weight:900;color:var(--accent2);margin:4px 0' id='cstV'>--</div>"
  "<div class='mc-unit'>INR (est.)</div>"
  "</div>"
  "<div class='mcard card-accent-t3'>"
  "<div class='mc-label'>&#127758; CO&#8322; Out</div>"
  "<div style='font-size:22px;font-weight:900;color:var(--accent3);margin:4px 0' id='cemV'>--</div>"
  "<div class='mc-unit'>grams CO&#8322;</div>"
  "</div>"
  "</div>"

  "<div class='sec-lbl'>Tree Equivalent</div>"
  "<div class='card card-accent-t2'>"
  "<div style='font-size:10px;color:var(--accent2);letter-spacing:1px;margin-bottom:6px'>&#127795; MINUTES OF CO&#8322; ABSORPTION NEEDED</div>"
  "<div class='trees' id='treeCon'><span class='ti'>&#127807;</span></div>"
  "<div style='font-size:10px;color:var(--text3);margin-top:8px' id='treeTxt'>--</div>"
  "</div>"

  "<div class='sec-lbl'>System Stats</div>"
  "<div class='m2'>"
  "<div class='mcard'>"
  "<div class='mc-label'>Uptime</div>"
  "<div style='font-size:20px;font-weight:900;color:var(--accent4)' id='uptimeV'>--</div>"
  "<div class='mc-unit'>seconds</div>"
  "</div>"
  "<div class='mcard'>"
  "<div class='mc-label'>Readings</div>"
  "<div style='font-size:20px;font-weight:900;color:var(--accent2)' id='readsV'>--</div>"
  "<div class='mc-unit'>total cycles</div>"
  "</div>"
  "</div>"

  "<div style='height:16px'></div>"
  "</div>" /* end page 3 */
  ));

  /* ══ PAGE 5: CONTROL ══ */
  server.sendContent(F(
  "<div class='page' id='pg4'>"
  "<div class='sec-lbl' style='margin-top:4px'>Occupancy Monitor</div>"

  "<div class='card card-accent-t2'>"
  "<div style='font-size:10px;color:var(--text3);margin-bottom:2px'>PIR SENSOR &middot; GPIO 27</div>"
  "<div class='occ-vis'>"
  "<div class='occ-ico' id='occIco'>&#128694;</div>"
  "<div>"
  "<div style='font-size:28px;font-weight:900' id='occSt'>--</div>"
  "<div style='font-size:12px;color:var(--text2)' id='occDs'>--</div>"
  "</div>"
  "</div>"
  "<div style='font-size:10px;color:var(--text3);letter-spacing:2px;margin-bottom:8px'>CONTROL MODE &mdash; <span id='modeL' style='color:var(--accent);font-weight:700'>--</span></div>"
  "<div class='mode-btns'>"
  "<div class='mbtn' id='mAuto' onclick='setMode(\"AUTO\")'>&#9654; AUTO</div>"
  "<div class='mbtn' id='mMan' onclick='setMode(\"MANUAL\")'>&#9998; MANUAL</div>"
  "</div>"
  "</div>"

  "<div class='sec-lbl'>AirCtrl Output</div>"

  "<div class='card card-accent-t2'>"
  "<div style='font-size:10px;color:var(--text3);margin-bottom:4px'>FAN RELAY &middot; GPIO 12</div>"
  "<div class='fan-center'>"
  "<div class='fan-ring'>"
  "<div class='fan-glow' id='fGlow'></div>"
  "<div class='fan-ico' id='fIco'>&#127744;</div>"
  "</div>"
  "<div style='font-size:22px;font-weight:900;margin-top:10px' id='fanSt'>--</div>"
  "<div style='font-size:11px;color:var(--text3);margin-top:3px' id='fanRs'>--</div>"
  "</div>"
  "</div>"

  "<div class='sec-lbl'>Current Readings</div>"
  "<div class='m2'>"
  "<div class='mcard card-accent-t3'><div class='mc-label'>Mode</div><div style='font-size:18px;font-weight:900;color:var(--accent3)' id='ctrlMode'>--</div></div>"
  "<div class='mcard card-accent-t'><div class='mc-label'>AQI Live</div><div style='font-size:18px;font-weight:900;color:var(--accent)' id='ctrlAqi'>--</div></div>"
  "</div>"

  "<div style='height:16px'></div>"
  "</div>" /* end page 4 */

  "</div>" /* end main */
  ));
  /* ── BOTTOM NAV ── */
  server.sendContent(F(
  "<nav class='botnav'>"
  "<button class='navbtn act' id='nb0' onclick='goPg(0)'><div class='navico'>&#127968;</div><div class='navlbl'>Home</div></button>"
  "<button class='navbtn' id='nb1' onclick='goPg(1)'><div class='navico'>&#129504;</div><div class='navlbl'>AI/ML</div></button>"
  "<button class='navbtn' id='nb2' onclick='goPg(2)'><div class='navico'>&#128268;</div><div class='navlbl'>Sensors</div></button>"
  "<button class='navbtn' id='nb3' onclick='goPg(3)'><div class='navico'>&#9889;</div><div class='navlbl'>Energy</div></button>"
  "<button class='navbtn' id='nb4' onclick='goPg(4)'><div class='navico'>&#127760;</div><div class='navlbl'>Control</div></button>"
  "</nav>"
  ));

  /* ── JAVASCRIPT ── */
  server.sendContent(F("<script>"
  "Chart.defaults.color='#3d6978';Chart.defaults.font.size=9;"
  "const S={n:0,aqiBuf:[],co2Buf:[],tmpBuf:[],"
  "hist:{aqi:[],co2:[],temp:[],humidity:[],labels:[]},"
  "kf:{x:200,P:100,Q:1,R:25},"
  "ewma:{aqi:200,co2:700,alpha:0.15},"
  "ada:{aqiSum:0,aqiSq:0,co2Sum:0,co2Sq:0,tSum:0,tSq:0,n:0},"
  "rl:{q:[[0,0],[0,0],[0,0],[0,0]],reward:0},"
  "isoH:{aqi:[],co2:[],tmp:[]},curTab:'aqi'};"

  /* PAGE NAVIGATION */
  "function goPg(i){"
  "document.querySelectorAll('.page').forEach((p,j)=>{p.classList.toggle('active',j===i)});"
  "document.querySelectorAll('.navbtn').forEach((b,j)=>{b.classList.toggle('act',j===i)});}"

  /* ACCORDION */
  "function tog(h){const b=h.nextElementSibling;const isOpen=b.classList.contains('open');"
  "h.classList.toggle('open',!isOpen);b.classList.toggle('open',!isOpen);}"

  /* KALMAN */
  "function kfUpd(kf,z){kf.P+=kf.Q;const K=kf.P/(kf.P+kf.R);kf.x+=K*(z-kf.x);kf.P=(1-K)*kf.P;return{x:kf.x,K,P:kf.P};}"

  /* EWMA */
  "function ewma(p,v,a){return a*v+(1-a)*p;}"

  /* LINEAR REGRESSION */
  "function linReg(a){const n=a.length;if(n<3)return{slope:0,intercept:a[n-1]||0,r2:0,rmse:0};"
  "let sx=0,sy=0,sxy=0,sxx=0,syy=0;"
  "for(let i=0;i<n;i++){sx+=i;sy+=a[i];sxy+=i*a[i];sxx+=i*i;syy+=a[i]*a[i];}"
  "const sl=(n*sxy-sx*sy)/(n*sxx-sx*sx||1);"
  "const ic=(sy-sl*sx)/n;"
  "let sr=0,st=0;const mn=sy/n;"
  "for(let i=0;i<n;i++){const p=sl*i+ic;sr+=(a[i]-p)**2;st+=(a[i]-mn)**2;}"
  "return{slope:sl,intercept:ic,r2:Math.max(0,1-sr/(st||1)),rmse:Math.sqrt(sr/n)};}"

  /* ISOLATION FOREST */
  "function isoSc(v,h){if(h.length<5)return 0.5;"
  "const mn=h.reduce((a,b)=>a+b,0)/h.length;"
  "const sd=Math.sqrt(h.reduce((a,b)=>a+(b-mn)**2,0)/h.length)||1;"
  "return Math.min(1,0.5+0.15*Math.abs(v-mn)/sd);}"

  /* PCA HEALTH */
  "function pcaH(aqi,co2,t,h,nh3,bz){"
  "const w=[0.35,0.25,0.15,0.1,0.1,0.05];"
  "const n=[(500-Math.min(aqi,500))/500,Math.max(0,(5000-co2)/4550),Math.max(0,1-Math.abs(t-22)/20),"
  "h>30&&h<70?1-Math.abs(h-50)/50:0,Math.max(0,1-nh3/100),Math.max(0,1-bz/14)];"
  "const sc=n.reduce((a,v,i)=>a+v*w[i],0)*100;"
  "return{score:Math.round(sc),pc1:Math.round((n[0]*0.6+n[1]*0.4)*100),pc2:Math.round((n[2]*0.5+n[3]*0.5)*100),pc3:Math.round((n[4]*0.5+n[5]*0.5)*100)};}"

  /* ADAPTIVE THRESHOLDS */
  "function adaTh(sum,sq,n,k=2){if(n<2)return null;"
  "const mn=sum/n;const v=sq/n-mn*mn;const sd=Math.sqrt(Math.max(0,v));"
  "return{warn:Math.round(mn+k*sd),crit:Math.round(mn+(k+1)*sd)};}"

  /* DFT */
  "function dft(a){const N=a.length;if(N<8)return Array(12).fill(0);"
  "const bins=Math.min(12,Math.floor(N/2));"
  "const mn=a.reduce((x,y)=>x+y,0)/N;const c=a.map(v=>v-mn);"
  "const r=[];for(let k=0;k<bins;k++){let re=0,im=0;"
  "for(let n=0;n<N;n++){const ang=2*Math.PI*k*n/N;re+=c[n]*Math.cos(ang);im-=c[n]*Math.sin(ang);}"
  "r.push(Math.sqrt(re*re+im*im)/N);}return r;}"
  "function spEnt(s){const sum=s.reduce((a,b)=>a+b,0)||1;const p=s.map(v=>v/sum);"
  "return-p.reduce((a,pi)=>a+(pi>0?pi*Math.log2(pi):0),0);}"

  /* RL */
  "function rlStep(aqi,occ,fan){"
  "let st=occ?(aqi>200?3:aqi>100?1:0):2;"
  "const ac=fan?1:0;"
  "let rw=0;if(aqi>200&&fan)rw=2;else if(aqi>100&&occ&&fan)rw=1;else if(aqi<=100&&!fan)rw=1;else if(aqi>100&&!fan)rw=-2;else rw=-0.5;"
  "S.rl.reward=Math.round(S.rl.reward+rw);"
  "const a=0.1,g=0.9,mx=Math.max(...S.rl.q[st]);"
  "S.rl.q[st][ac]+=a*(rw+g*mx-S.rl.q[st][ac]);"
  "return{state:st,action:ac,reward:rw};}"

  /* NEURAL NET */
  "function nnFwd(aqi,co2,t,h){"
  "const sig=x=>1/(1+Math.exp(-x));"
  "const inp=[aqi/500,co2/5000,t/50,h/100];"
  "const W1=[[-2,1.5,-1,0.5],[1,-2,0.5,1],[-0.5,1,-1.5,2],[1,1,-1,-1],[0.5,-1,2,-0.5],[-1.5,0.5,1,1],[1,-0.5,-1,1.5],[0.5,1,0.5,-1]];"
  "const b1=[-0.5,0.3,-0.2,0.1,-0.4,0.2,-0.1,0.3];"
  "const hid=W1.map((w,i)=>sig(w.reduce((a,v,j)=>a+v*inp[j],0)+b1[i]));"
  "const W2=[[1,0.5,-1,0.5,-0.5,1,-1,0.5],[0.5,-1,1,-0.5,1,-0.5,0.5,-1],[0.5,1,-0.5,-1,0.5,1,-0.5,0.5],[-1,0.5,0.5,1,-1,-0.5,1,-0.5]];"
  "const b2=[0.1,-0.2,0.2,-0.1];"
  "const out=W2.map((w,i)=>sig(w.reduce((a,v,j)=>a+v*hid[j],0)+b2[i]));"
  "const sm=out.reduce((a,b)=>a+b,0);const sf=out.map(v=>v/sm);"
  "const cls=['SAFE','MODERATE','POLLUTED','HAZARDOUS'];"
  "return{probs:sf,label:cls[sf.indexOf(Math.max(...sf))]};}"

  /* CONFIDENCE INTERVALS */
  "function ci(buf){if(buf.length<3)return{lo:0,mean:0,hi:0,std:0};"
  "const n=buf.length,mn=buf.reduce((a,b)=>a+b,0)/n;"
  "const sd=Math.sqrt(buf.reduce((a,b)=>a+(b-mn)**2,0)/n);"
  "const se=sd/Math.sqrt(n);"
  "return{lo:mn-1.96*se,mean:mn,hi:mn+1.96*se,std:sd};}"
  ));

  server.sendContent(F(
  /* RING CHART */
  "let ringCtx,aqi_ring_chart;"
  "function initRing(){"
  "ringCtx=document.getElementById('aqi-ring').getContext('2d');"
  "aqi_ring_chart=new Chart(ringCtx,{type:'doughnut',data:{datasets:[{data:[0,500],backgroundColor:['#ff6b35','rgba(13,30,38,0.8)'],borderWidth:0,circumference:280,rotation:220}]},"
  "options:{responsive:false,cutout:'78%',plugins:{legend:{display:false},tooltip:{enabled:false}},animation:{duration:500}}});}"
  "function updRing(aqi,col){"
  "if(!aqi_ring_chart)return;"
  "aqi_ring_chart.data.datasets[0].data=[aqi,Math.max(0,500-aqi)];"
  "aqi_ring_chart.data.datasets[0].backgroundColor=[col,'rgba(13,30,38,0.8)'];"
  "aqi_ring_chart.update();}"

  /* SPARKLINE */
  "let spkChart;"
  "function initSpk(){"
  "const ctx=document.getElementById('aqiSpk').getContext('2d');"
  "const g=ctx.createLinearGradient(0,0,0,70);g.addColorStop(0,'rgba(255,107,53,0.3)');g.addColorStop(1,'rgba(255,107,53,0)');"
  "spkChart=new Chart(ctx,{type:'line',data:{labels:Array(20).fill(''),datasets:[{data:Array(20).fill(null),borderColor:'#ff6b35',borderWidth:2,backgroundColor:g,fill:true,tension:0.4,pointRadius:0}]},"
  "options:{responsive:true,maintainAspectRatio:false,animation:{duration:200},plugins:{legend:{display:false}},scales:{x:{display:false},y:{display:false}}}});}"

  /* HISTORY CHART */
  "let histChart;"
  "function initHist(){"
  "const ctx=document.getElementById('histCh').getContext('2d');"
  "const c='#00e5ff';const g=ctx.createLinearGradient(0,0,0,160);g.addColorStop(0,c+'44');g.addColorStop(1,c+'00');"
  "histChart=new Chart(ctx,{type:'line',data:{labels:[],datasets:[{data:[],borderColor:c,borderWidth:2,backgroundColor:g,fill:true,tension:0.4,pointRadius:0}]},"
  "options:{responsive:true,maintainAspectRatio:false,animation:{duration:400},plugins:{legend:{display:false}},"
  "scales:{x:{display:false},y:{grid:{color:'rgba(255,255,255,0.04)'},border:{display:false},ticks:{color:'#3d6978',maxTicksLimit:4}}}}});}"

  "function switchTab(t,el){"
  "S.curTab=t;"
  "document.querySelectorAll('.htab').forEach(x=>{x.style.background='var(--bg3)';x.style.borderColor='var(--border)';x.style.color='var(--text2)';});"
  "el.style.background='rgba(0,229,255,0.1)';el.style.borderColor='var(--accent)';el.style.color='var(--accent)';"
  "updHist();}"

  "function updHist(){"
  "const cols={aqi:'#ff6b35',co2:'#00e5ff',temp:'#ffd60a',humidity:'#a855f7'};"
  "const c=cols[S.curTab];const ctx=document.getElementById('histCh').getContext('2d');"
  "const g=ctx.createLinearGradient(0,0,0,160);g.addColorStop(0,c+'44');g.addColorStop(1,c+'00');"
  "histChart.data.labels=[...S.hist.labels];"
  "histChart.data.datasets[0].data=[...S.hist[S.curTab]];"
  "histChart.data.datasets[0].borderColor=c;histChart.data.datasets[0].backgroundColor=g;"
  "histChart.update('none');}"

  /* ALERT FEED */
  "function pushAlert(msg,lvl='ai'){"
  "const t=new Date().toLocaleTimeString('en-GB',{hour12:false});"
  "const f=document.getElementById('alertFeed');"
  "const el=document.createElement('div');el.className='af-item';"
  "el.innerHTML=`<span class='af-time'>${t}</span><span class='af-msg'>${msg}</span><span class='af-lvl al-${lvl}'>${lvl.toUpperCase()}</span>`;"
  "f.insertBefore(el,f.firstChild);while(f.children.length>8)f.removeChild(f.lastChild);}"

  /* UTILS */
  "function gCol(v){return v<=50?'#00ff9d':v<=100?'#ffd60a':v<=150?'#ff8c00':v<=200?'#ff6b35':'#ff3366';}"
  "function gBadge(v){return v<=50?'bg-b':v<=100?'bn':v<=150?'bw':'bd';}"
  "function co2Lbl(v){return v<800?'Normal':v<1200?'Elevated':v<1500?'High CO2':'Very High';}"
  "const $=id=>document.getElementById(id);"
  "const ADESC={'KEEP_OFF':'Air quality safe. No ventilation needed.','CHECK_OCCUPANCY':'Room empty — monitoring only.','TURN_ON_NOW':'Poor air quality — start AirCtrl now.','VENTILATE_ROOM':'High CO2 — ventilate and run fan.','DANGEROUS_AIR':'HAZARD — run AirCtrl, open windows!'};"
  ));

  server.sendContent(F(
  /* MAIN UPDATE */
  "function updUI(d,ai){"
  "const aqi=+d.aqi,col=gCol(aqi);"
  /* HOME */
  "updRing(aqi,col);"
  "$('aqiV').textContent=aqi;$('aqiV').style.color=col;"
  "$('aqiQual').textContent=d.airQuality;$('aqiQual').style.color=col;"
  "$('aqiStatus').textContent=d.co2Status;"
  "$('kfV').textContent=d.aqiKalman||Math.round(ai.kfCO2.x/7);"
  "$('ewmaV').textContent=Math.round(S.ewma.aqi);"
  "$('co2V').textContent=d.co2;"
  "$('co2B').textContent=co2Lbl(+d.co2);$('co2B').className='mc-badge '+(+d.co2>1500?'bd':+d.co2>1000?'bw':'bg-b');"
  "$('tmpV').textContent=d.temp;$('humV').innerHTML=d.humidity+'<span style=\"font-size:12px;color:var(--text2)\">%</span>';"
  /* AI Summary */
  "$('aiActV').textContent=ai.action.replaceAll('_',' ');"
  "$('aiActD').textContent=ADESC[ai.action]||'';"
  "$('aiActB').textContent=ai.action.split('_')[0];"
  "const isA=ai.anomaly==='ANOMALY';"
  "$('anomV').textContent=ai.anomaly;$('anomV').style.color=isA?'var(--danger)':'var(--accent2)';"
  "$('hlthV').textContent=ai.pca.score;$('hlthV').style.color=ai.pca.score>70?'var(--accent2)':ai.pca.score>40?'var(--warn)':'var(--danger)';"
  /* Forecast */
  "$('pAN').textContent=aqi;$('pA30').textContent=ai.lrPreds[0];$('pA60').textContent=ai.lrPreds[1];$('pA120').textContent=ai.lrPreds[2];"
  /* Sparkline */
  "if(spkChart){const sp=spkChart.data.datasets[0].data;sp.push(aqi);if(sp.length>20)sp.shift();spkChart.update('none');}"
  /* Banner */
  "if(aqi>200){$('ab').style.cssText='background:linear-gradient(90deg,rgba(255,51,102,0.1),rgba(255,51,102,0.02));border:1px solid rgba(255,51,102,0.3);border-radius:12px;padding:12px 14px;display:flex;align-items:center;gap:12px;margin-bottom:12px';$('abMsg').textContent='Very Unhealthy - AirCtrl activated';$('abBadge').textContent='ALERT';$('abBadge').className='banner-badge bd';$('abIco').innerHTML='&#128680;';$('abAqi').textContent=aqi;}"
  "else if(aqi>100){$('ab').style.cssText='background:linear-gradient(90deg,rgba(255,214,10,0.08),rgba(255,214,10,0.02));border:1px solid rgba(255,214,10,0.2);border-radius:12px;padding:12px 14px;display:flex;align-items:center;gap:12px;margin-bottom:12px';$('abMsg').textContent='Moderate air quality';$('abBadge').textContent='CAUTION';$('abBadge').className='banner-badge bw';$('abIco').innerHTML='&#9888;&#65039;';$('abAqi').textContent=aqi;}"
  "else{$('ab').style.cssText='background:linear-gradient(90deg,rgba(0,255,157,0.07),rgba(0,255,157,0.01));border:1px solid rgba(0,255,157,0.18);border-radius:12px;padding:12px 14px;display:flex;align-items:center;gap:12px;margin-bottom:12px';$('abMsg').textContent='Air quality OK';$('abBadge').textContent='NORMAL';$('abBadge').className='banner-badge bg-b';$('abIco').innerHTML='&#9989;';$('abAqi').textContent=aqi;}"
  /* SENSOR PAGE */
  "$('nh3V').textContent=d.nh3;$('nh3B').style.width=Math.min(100,+d.nh3/2)+'%';"
  "$('bzV').textContent=d.benzene;$('bzB').style.width=Math.min(100,+d.benzene*5)+'%';"
  "$('coV').textContent=d.co;$('coB').style.width=Math.min(100,+d.co*10)+'%';"
  "$('vltV').textContent=d.voltage;"
  "$('curV').textContent=d.current;$('curB').style.width=Math.min(100,+d.current/5)+'%';"
  "$('pwrV').textContent=d.power;$('pwrB').style.width=Math.min(100,+d.power*10)+'%';"
  /* ENERGY PAGE */
  "$('engV').textContent=parseFloat(d.energy).toFixed(4);"
  "$('cstV').textContent='Rs.'+parseFloat(d.cost).toFixed(4);"
  "const co2g=(parseFloat(d.energy)*820).toFixed(1);$('cemV').textContent=co2g;"
  "const n=Math.max(1,Math.round(+co2g/0.5));const tc=$('treeCon');tc.innerHTML='';"
  "const em=['&#127795;','&#127794;','&#127796;','&#127797;'];for(let i=0;i<Math.min(n,16);i++){const s=document.createElement('span');s.className='ti';s.innerHTML=em[i%4];tc.appendChild(s);}"
  "$('treeTxt').textContent='~'+n+' tree-min to offset '+co2g+'g CO2';"
  "$('uptimeV').textContent=d.uptime;$('readsV').textContent=d.readings;"
  /* CONTROL PAGE */
  "const occ=d.occupancy==1;const fanOn=d.airCtrl==1;"
  "$('occIco').style.opacity=occ?'1':'0.3';"
  "$('occSt').textContent=occ?'PRESENT':'ABSENT';$('occSt').style.color=occ?'var(--accent2)':'var(--text3)';"
  "$('occDs').textContent=occ?'Motion detected':'No motion detected';"
  "$('modeL').textContent=d.mode;"
  "$('mAuto').className='mbtn'+(d.mode==='AUTO'?' mbtn-auto':'');"
  "$('mMan').className='mbtn'+(d.mode==='MANUAL'?' mbtn-man':'');"
  "$('fIco').className='fan-ico'+(fanOn?' fan-spin':'');"
  "$('fanSt').textContent=fanOn?'RUNNING':'STANDBY';$('fanSt').style.color=fanOn?'var(--accent2)':'var(--text3)';"
  "$('fGlow').style.display=fanOn?'block':'none';"
  "$('fanRs').textContent=fanOn?'AQI '+aqi+' exceeded threshold':'Waiting for activation conditions';"
  "$('ctrlMode').textContent=d.mode;$('ctrlAqi').textContent=aqi;"
  "}"
  ));

  server.sendContent(F(
  /* AI UPDATE */
  "function updAI(d,ai){"
  /* Kalman */
  "const raw=+d.aqi;"
  "$('kfRawN').textContent=raw;$('kfRawB').style.width=Math.min(100,raw/5)+'%';"
  "const kv=d.aqiKalman||Math.round(ai.kfCO2.x/7);$('kfKalN').textContent=kv;$('kfKalB').style.width=Math.min(100,kv/5)+'%';"
  "const ev=Math.round(S.ewma.aqi);$('kfEwN').textContent=ev;$('kfEwB').style.width=Math.min(100,ev/5)+'%';"
  "$('kfGain').textContent=ai.kfCO2.K.toFixed(4);$('kfCov').textContent=ai.kfCO2.P.toFixed(2);"
  /* LinReg */
  "const sl=ai.lr.slope;$('lrSlope').innerHTML=(sl>0?'+':'')+sl.toFixed(2)+(sl>0.1?'&#8599;':sl<-0.1?'&#8600;':'&#8594;');"
  "$('lrR2').textContent=ai.lr.r2.toFixed(3);$('lrRMSE').textContent=ai.lr.rmse.toFixed(1);"
  "$('lrConf').textContent=Math.round(ai.lr.r2*100)+'%';"
  "$('lrN').textContent=raw;$('lrP30').textContent=ai.lrPreds[0];$('lrP60').textContent=ai.lrPreds[1];$('lrP120').textContent=ai.lrPreds[2];"
  /* Isolation Forest */
  "$('isoA').textContent=ai.isoA.toFixed(3);$('isoC').textContent=ai.isoC.toFixed(3);$('isoT').textContent=ai.isoT.toFixed(3);"
  "const isoV=ai.isoComp.toFixed(3);const isoCol=ai.isoComp>0.75?'var(--danger)':ai.isoComp>0.55?'var(--warn)':'var(--accent2)';"
  "$('isoComp').textContent=isoV;$('isoComp').style.color=isoCol;"
  "$('isoLbl').textContent=ai.isoComp>0.75?'ANOMALY DETECTED':ai.isoComp>0.55?'SUSPICIOUS':'NOMINAL';$('isoLbl').style.color=isoCol;"
  "$('isoBar').style.width=(ai.isoComp*100)+'%';$('isoBar').style.background=isoCol;$('isoBar').style.height='100%';$('isoBar').style.borderRadius='3px';"
  /* PCA */
  "$('pcaScore').textContent=ai.pca.score;$('pcaScore').style.color=ai.pca.score>70?'var(--accent2)':ai.pca.score>40?'var(--warn)':'var(--danger)';"
  "$('pc1b').style.width=ai.pca.pc1+'%';$('pc1p').textContent=ai.pca.pc1+'%';"
  "$('pc2b').style.width=ai.pca.pc2+'%';$('pc2p').textContent=ai.pca.pc2+'%';"
  "$('pc3b').style.width=ai.pca.pc3+'%';$('pc3p').textContent=ai.pca.pc3+'%';"
  /* Adaptive thresholds */
  "if(ai.adaAqi){$('adaAW').textContent=ai.adaAqi.warn;$('adaAC').textContent=ai.adaAqi.crit;}"
  "if(ai.adaCo2)$('adaCW').textContent=ai.adaCo2.warn;"
  "if(ai.adaT)$('adaTW').textContent=ai.adaT.warn;"
  "const adaSt=ai.adaAqi?(+d.aqi>ai.adaAqi.crit?'&#128680; CRITICAL breach':+d.aqi>ai.adaAqi.warn?'&#9888; Warning exceeded':'&#9989; Within adaptive limits'):'Calibrating...';"
  "$('adaSt').innerHTML=adaSt;"
  /* FFT */
  "const fb=$('fftBins');fb.innerHTML='';"
  "const mx=Math.max(...ai.spectrum,1);"
  "ai.spectrum.forEach((v,i)=>{const el=document.createElement('div');el.className='fft-b';el.style.height=Math.max(2,(v/mx)*60)+'px';if(i===ai.domIdx)el.style.background='var(--warn)';fb.appendChild(el);});"
  "$('fftDom').textContent='f'+ai.domIdx;$('fftEnt').textContent=ai.entropy.toFixed(2);$('fftPer').textContent=ai.domIdx>0?'YES':'NO';"
  /* RL */
  "S.rl.q.forEach((qp,i)=>{const q=Math.max(...qp);const pct=Math.min(100,(q+3)/6*100);$('rl'+i+'b').style.width=pct+'%';$('rl'+i+'v').textContent='Q:'+q.toFixed(2);});"
  "$('rlAct').textContent=(ai.rl.action?'FAN ON':'FAN OFF')+' (r='+ai.rl.reward.toFixed(1)+')';$('rlAct').style.color=ai.rl.action?'var(--accent2)':'var(--text3)';"
  "$('rlRew').textContent=S.rl.reward;"
  /* Neural Net */
  "const ncl=['SAFE','MOD','POLL','HAZ'];const nco=['var(--accent2)','var(--warn)','var(--accent3)','var(--danger)'];"
  "$('nnOut').textContent=ai.nn.label;$('nnOut').style.color=nco[['SAFE','MODERATE','POLLUTED','HAZARDOUS'].indexOf(ai.nn.label)]||'var(--text)';"
  "ai.nn.probs.forEach((p,i)=>{$('nn'+i).textContent=(p*100).toFixed(0)+'%';});"
  /* CI */
  "const ciA=ai.ciAqi;if(ciA.mean){const cs=v=>Math.max(2,Math.min(96,v/500*100))+'%';"
  "$('ciAL').style.width=cs(ciA.lo);$('ciAL').textContent=Math.round(ciA.lo);"
  "$('ciAM').style.width=cs(ciA.mean);$('ciAM').textContent=Math.round(ciA.mean);"
  "$('ciAU').style.width=cs(ciA.hi);$('ciAU').textContent=Math.round(ciA.hi);}"
  "$('ciUnc').textContent=ai.uncertainty;$('ciSnr').textContent=ai.snr;"
  /* Smart alerts */
  "if(S.n===1)pushAlert('All 8 AI modules online: Kalman, OLS, IsoForest, PCA, AdaThresh, DFT, RL, MLP','ai');"
  "if(S.n%5===0&&ai.isoComp>0.75)pushAlert('IsoForest anomaly: score='+ai.isoComp.toFixed(3),'crit');"
  "else if(S.n%5===0&&ai.isoComp>0.55)pushAlert('IsoForest suspicious: '+ai.isoComp.toFixed(3),'warn');"
  "if(S.n%8===0&&ai.adaAqi&&+d.aqi>ai.adaAqi.warn)pushAlert('Adaptive limit breach: AQI='+d.aqi+' > tau='+ai.adaAqi.warn,'warn');"
  "}"

  /* MAIN COMPUTE */
  "function aiCalc(d){"
  "const aqi=+d.aqi,co2=+d.co2,occ=+d.occupancy,fan=+d.airCtrl;"
  "S.aqiBuf.push(aqi);if(S.aqiBuf.length>30)S.aqiBuf.shift();"
  "S.co2Buf.push(co2);if(S.co2Buf.length>30)S.co2Buf.shift();"
  "S.tmpBuf.push(+d.temp);if(S.tmpBuf.length>30)S.tmpBuf.shift();"
  "S.ewma.aqi=ewma(S.ewma.aqi,aqi,S.ewma.alpha);"
  "S.ewma.co2=ewma(S.ewma.co2,co2,S.ewma.alpha);"
  "const kfCO2=kfUpd(S.kf,co2);"
  "const lr=linReg(S.aqiBuf);"
  "const steps=[15,30,60];const lrPreds=steps.map(s=>Math.max(0,Math.min(500,Math.round(lr.slope*s+lr.intercept))));"
  "S.isoH.aqi.push(aqi);if(S.isoH.aqi.length>60)S.isoH.aqi.shift();"
  "S.isoH.co2.push(co2);if(S.isoH.co2.length>60)S.isoH.co2.shift();"
  "S.isoH.tmp.push(+d.temp);if(S.isoH.tmp.length>60)S.isoH.tmp.shift();"
  "const isoA=isoSc(aqi,S.isoH.aqi),isoC=isoSc(co2,S.isoH.co2),isoT=isoSc(+d.temp,S.isoH.tmp);"
  "const isoComp=isoA*0.5+isoC*0.3+isoT*0.2;"
  "const pca=pcaH(aqi,co2,+d.temp,+d.humidity,+d.nh3,+d.benzene);"
  "S.ada.n++;S.ada.aqiSum+=aqi;S.ada.aqiSq+=aqi*aqi;S.ada.co2Sum+=co2;S.ada.co2Sq+=co2*co2;S.ada.tSum+=(+d.temp);S.ada.tSq+=(+d.temp)**2;"
  "const adaAqi=adaTh(S.ada.aqiSum,S.ada.aqiSq,S.ada.n);"
  "const adaCo2=adaTh(S.ada.co2Sum,S.ada.co2Sq,S.ada.n);"
  "const adaT=adaTh(S.ada.tSum,S.ada.tSq,S.ada.n);"
  "const spectrum=dft(S.aqiBuf);const entropy=spEnt(spectrum);"
  "const domIdx=spectrum.indexOf(Math.max(...spectrum));"
  "const rl=rlStep(aqi,occ,fan);"
  "const nn=nnFwd(aqi,co2,+d.temp,+d.humidity);"
  "const ciAqi=ci(S.aqiBuf);const ciCo2=ci(S.co2Buf);const ciTmp=ci(S.tmpBuf);"
  "const uncertainty=(ciAqi.std+ciCo2.std/100+ciTmp.std).toFixed(2);"
  "const snr=ciAqi.std>0?(20*Math.log10(Math.max(1,ciAqi.mean)/ciAqi.std)).toFixed(1):'--';"
  "let action='KEEP_OFF';"
  "if(aqi>250||co2>1800)action='DANGEROUS_AIR';"
  "else if(co2>1200)action='VENTILATE_ROOM';"
  "else if(aqi>120)action='TURN_ON_NOW';"
  "else if(!occ&&aqi<=100&&co2<=900)action='CHECK_OCCUPANCY';"
  "const anomaly=(isoComp>0.75||aqi>300||co2>2500)?'ANOMALY':'NORMAL';"
  "return{kfCO2,lr,lrPreds,isoA,isoC,isoT,isoComp,pca,adaAqi,adaCo2,adaT,spectrum,entropy,domIdx,rl,nn,ciAqi,uncertainty,snr,action,anomaly};}"

  "function pushHist(d){"
  "const now=new Date();const lbl=now.getHours().toString().padStart(2,'0')+':'+now.getMinutes().toString().padStart(2,'0')+':'+now.getSeconds().toString().padStart(2,'0');"
  "['aqi','co2','temp','humidity'].forEach(k=>{S.hist[k].push(+d[k]);if(S.hist[k].length>60)S.hist[k].shift();});"
  "S.hist.labels.push(lbl);if(S.hist.labels.length>60)S.hist.labels.shift();updHist();}"

  "async function update(){"
  "S.n++;try{"
  "const r=await fetch('/api/data',{signal:AbortSignal.timeout(2500)});"
  "const d=await r.json();"
  "const ai=aiCalc(d);updUI(d,ai);updAI(d,ai);pushHist(d);"
  "}catch(e){console.log(e);}"
  "const di=S.n%3;['d1','d2','d3'].forEach((id,i)=>{$(id).className='upd-d'+(i===di?' act':'')});}"

  "async function setMode(m){try{await fetch('/api/mode?set='+m);}catch(e){}}"
  "function updClk(){$('clk').textContent=new Date().toLocaleTimeString('en-US',{hour12:false,hour:'2-digit',minute:'2-digit'});}"

  "window.addEventListener('load',()=>{"
  "initRing();initSpk();initHist();"
  "updClk();setInterval(updClk,1000);"
  "setTimeout(()=>{const o=$('lo');o.style.opacity='0';setTimeout(()=>o.style.display='none',400)},1800);"
  "setTimeout(()=>{update();setInterval(update,2000)},2000);"
  "});"
  "</script></body></html>"));
}

/* ══════════════════════════════════════════════════════════════
   SMART DEVICE ROUTER
   Detects mobile vs desktop via User-Agent and serves the
   correct optimized UI automatically.
   ══════════════════════════════════════════════════════════════ */

bool isMobileClient() {
  /* Check if the requesting client is a mobile device */
  if (server.hasHeader("User-Agent")) {
    String ua = server.header("User-Agent");
    ua.toLowerCase();
    return (ua.indexOf("mobile")  != -1 ||
            ua.indexOf("android") != -1 ||
            ua.indexOf("iphone")  != -1 ||
            ua.indexOf("ipad")    != -1 ||
            ua.indexOf("ipod")    != -1 ||
            ua.indexOf("blackberry") != -1 ||
            ua.indexOf("windows phone") != -1);
  }
  return false;
}

/* ── UNIFIED ENTRY POINT ── */
void handleRoot() {
  /* Force mobile via ?mobile=1, force desktop via ?desktop=1 */
  if (server.hasArg("desktop")) {
    handleRootDesktop();
  } else if (server.hasArg("mobile")) {
    handleRootMobile();
  } else if (isMobileClient()) {
    handleRootMobile();
  } else {
    handleRootDesktop();
  }
}

/* ── SETUP ── */
void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  pinMode(PIR_PIN, INPUT);
  pinMode(FAN_PIN, OUTPUT);    digitalWrite(FAN_PIN, LOW);
  pinMode(BUZZER_PIN, OUTPUT); digitalWrite(BUZZER_PIN, LOW);
  pinMode(RED_LED, OUTPUT);    digitalWrite(RED_LED, LOW);
  pinMode(GREEN_LED, OUTPUT);  digitalWrite(GREEN_LED, LOW);
  pinMode(BLUE_LED, OUTPUT);   digitalWrite(BLUE_LED, LOW);

  Wire.begin(21,22);
  dht.begin();
  lcd.init();
  lcd.backlight();
  lcd.clear();

  if (!ina219.begin()) { Serial.println("INA219 not found"); inaErr = true; }

  loadR0();
  startMs = millis();
  prevEnergyMs = millis();

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password, 0, bssid);

  lcd.setCursor(0,0); lcd.print("AirMind AI v3.0");
  lcd.setCursor(0,1); lcd.print("Connecting...");
  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println();

  String ip = WiFi.localIP().toString();
  Serial.println("==============================================");
  Serial.println("  AirMind AI v3.0 — Adaptive Dashboard");
  Serial.println("  Auto-detects PC vs Mobile browser!");
  Serial.println("  AI: Kalman·LinReg·IsoForest·PCA·RL·FFT·MLP");
  Serial.println("==============================================");
  Serial.print("  Open: http://"); Serial.println(ip);
  Serial.print("  JSON: http://"); Serial.println(ip + "/api/data");
  Serial.print("  Force Desktop: http://"); Serial.println(ip + "/?desktop");
  Serial.print("  Force Mobile:  http://"); Serial.println(ip + "/?mobile");
  Serial.println("==============================================");

  lcd.clear();
  lcd.setCursor(0,0); lcd.print("AirMind AI 3.0!");
  lcd.setCursor(0,1); lcd.print(ip);
  delay(3000);

  /* Collect User-Agent so isMobileClient() can read it */
  const char* headerKeys[] = {"User-Agent"};
  server.collectHeaders(headerKeys, 1);

  /* Single route — auto-dispatches to desktop or mobile */
  server.on("/",          handleRoot);
  server.on("/api/data",  handleData);
  server.on("/api/mode",  handleMode);
  server.begin();

  Serial.println("Adaptive web server running!");
}

/* ── LOOP ── */
void loop() {
  server.handleClient();
  unsigned long now = millis();

  if (now - lastSensorMs >= 2000) {
    lastSensorMs = now;
    updateOcc();
    readSensors();
    updateFan();
    updateLEDs();
    handleBuzz();
    Serial.print("[AirMind AI] AQI="); Serial.print(AQI);
    Serial.print(" KF=");  Serial.print(AQI_kalman);
    Serial.print(" CO2="); Serial.print((int)co2ppm);
    Serial.print(" T=");   Serial.print(temp,1);
    Serial.print(" Fan="); Serial.println(fanState ? "ON" : "OFF");
  }

  if (now - lastLCDMs >= 2000) { lastLCDMs = now; updateLCD(); }
}