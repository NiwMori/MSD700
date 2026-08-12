#include <SPI.h>
#include <esp_now.h>
#include <WiFi.h>
#include <HardwareSerial.h>
#include <HTTPClient.h>
#include <M5Unified.h>
#include <SD.h>
#include <string.h>
#include <stdlib.h>

#define UART_RX 2
#define UART_TX 1

#define UART_BAUD 9600
#define UPDATE_MS 1000

// --- Machine unique code ---
const char *MACHINE_CODE = "MSD700_001"; // ここを各マシンごとにユニークなコードに変更してください

// --- 接続設定 ---
// wifi
const char *WIFI_SSID = "KD-249_314EC3";
const char *WIFI_PASS = "1234567890";

// オンプレミスサーバーAPIエンドポイント
// const char *UPLOAD_URL = "http://192.168.0.102/msd700_monitor_upload.php";
const char *UPLOAD_URL = "https://msdapi.nakayamairon.com/MSD_API/saga_univ/index.php";
// const char *UPLOAD_URL = "https://msdapi.nakayamairon.com/saga_univ/index.php";

// デバッグ用の代替URL（必要に応じて使用）
// const char *UPLOAD_URL = "https://msdapi.nakayamairon.com/saga_univ/";

// デバッグフラグ
#define DEBUG_CSV_PARSING true

// 通信相手（M5Tough / Tough代替CoreS3）のMACアドレス
uint8_t targetAddress[] = {0x1C, 0xDB, 0xD4, 0xBA, 0x80, 0xF4};
const unsigned long TIMEOUT_MS = 5000;

// --- グローバル定数 ---
const unsigned long WRITE_INTERVAL = 60000;     // SDカードへの書き込み間隔(ms)
const unsigned long UPLOAD_INTERVAL = 30000;    // サーバーへのアップロード間隔(ms)
const int MAX_RECORDS_PER_FILE = 200;           // 1ファイルあたりの最大レコード数
const int MAX_LOG_FILES_ROTATION = 100;         // ログファイルのローテーション数
const int WIFI_CONNECT_MAX_RETRIES = 30;        // WiFi接続試行の最大回数
const int MIN_VALID_CSV_LINE_LENGTH = 10;       // 有効なCSV行の最小長
const int CSV_FIELD_COUNT = 13;                 // CSVのフィールド数
const int MAX_JOYSTICK_INPUT_LENGTH = 100;      // ジョイスティック入力文字列の最大長
const unsigned long SD_SCAN_INTERVAL_MS = 5000; // 未送信ファイルチェックの間隔(ms)
bool isConnected = false;
bool isTimeSynced = false; // ★追加：時刻同期が完了したかどうかのフラグ
File logFile;

// --- 2. データ構造定義 ---
struct SensorData
{
  int16_t battery_V;
  int16_t battery_I;
  int16_t battery_SOC;
  int16_t battery_temp_max;
  int16_t battery_temp_min;
  uint16_t soc_mAh;
  int16_t bnoX;
  int16_t bnoY;
  int16_t bnoZ;
  int16_t temp;
  int16_t humid;
  int16_t press;
} __attribute__((packed));
SensorData data;

struct JoystickRawData
{
  uint16_t btn;
  uint16_t rx;
  uint16_t ry;
  uint16_t r_knob;
  uint16_t lx;
  uint16_t ly;
  uint16_t l_knob;
} __attribute__((packed));
JoystickRawData joyData;

// PowerHub から受け取るデータ構造 (coreSE.ino から移植)
struct __attribute__((packed)) HubData
{
  uint8_t header;     // 0xAA (同期バイト)
  uint8_t level;      // バッテリー残量 0~100 %
  uint16_t voltage;   // バッテリー電圧 mV
  int16_t current;    // バッテリー電流 mA (+ = 充電, - = 放電)
  uint8_t charging;   // 1 = 充電中, 0 = 未充電
  uint32_t timestamp; // Unixタイムスタンプ (UTC エポック秒、JST 表示は +9h)
  uint8_t checksum;   // 簡易チェックサム
};

uint8_t calcChecksum(const HubData &d)
{
  const uint8_t *p = (const uint8_t *)&d;
  uint8_t sum = 0;
  for (size_t i = 0; i < sizeof(HubData) - 1; i++)
  {
    sum += p[i];
  }
  return sum;
}

// --- 3. 状態管理変数 ---
volatile bool dataUpdated = false;
unsigned long lastRecvTime = 0;
unsigned long lastSdWriteTime = 0; // 前回のSD書き込み時刻
unsigned long lastUploadTime = 0;  // 前回のサーバーアップロード時刻
bool uploadInProgress = false;     // アップロード中フラグ

char fileName[32] = "";
bool sdReady = false;
bool isLoggingActive = false;
uint32_t recordCount = 0;

static HubData powerHubData;
static bool powerHubDataValid = false;

// --- 4. 画面描写関数 ---
// --- 上部ステータスバー専用の描画関数 ---
void drawPowerHubStatus()
{
  M5.Lcd.startWrite();
  M5.Lcd.setTextSize(2);

  // --- 1. 時刻の取得と表示 (左端) ---
  M5.Lcd.setCursor(5, 2);
  M5.Lcd.setTextColor(WHITE, BLACK);

  // 同期完了フラグを見て、内部時計から時間を取得する
  if (isTimeSynced)
  {
    time_t now = time(NULL);
    now += 9 * 3600;
    struct tm *tm_info = gmtime(&now);

    M5.Lcd.printf("%02d/%02d %02d:%02d",
                  tm_info->tm_mon + 1,
                  tm_info->tm_mday,
                  tm_info->tm_hour,
                  tm_info->tm_min);
  }
  else
  {
    M5.Lcd.print("--/-- --:--");
  }

  // --- 2. PowerHubのステータス表示 ---
  if (powerHubDataValid)
  {
    M5.Lcd.fillCircle(310, 9, 6, GREEN); // 右端：Hub接続状態(緑丸)

    // 中央：充電状態 (X=140)
    M5.Lcd.setCursor(140, 2);
    if (powerHubData.charging)
    {
      M5.Lcd.setTextColor(GREEN, BLACK);
      M5.Lcd.print("charge");
    }
    else
    {
      M5.Lcd.fillRect(140, 0, 100, 18, BLACK); // 表示を消す
    }

    // 右詰め：バッテリー状況 (X=245)
    M5.Lcd.setCursor(245, 2);
    uint16_t levelColor = (powerHubData.level < 50) ? RED : GREEN;
    M5.Lcd.setTextColor(levelColor, BLACK);
    M5.Lcd.printf("%3d%%", powerHubData.level);
  }
  else
  {
    M5.Lcd.fillCircle(310, 9, 6, RED); // 右端：Hub未接続状態(赤丸)
    M5.Lcd.fillRect(140, 0, 160, 18, BLACK);
  }

  M5.Lcd.endWrite();
}

// --- 待機画面の描画関数 ---
void drawDisconnectedScreen()
{
  // トップバー（上から20px）を残して下半分だけを黒塗り
  M5.Lcd.fillRect(0, 20, 320, 220, BLACK);

  // トップバーと被らないよう、四角形のY座標を 23 に下げる
  M5.Lcd.drawRect(3, 23, 314, 214, RED);
  M5.Lcd.drawRect(6, 26, 308, 208, RED);

  M5.Lcd.setTextSize(3);
  M5.Lcd.setTextColor(RED, BLACK);
  // 少し下にずらす
  M5.Lcd.setCursor(52, 90);
  M5.Lcd.print("DISCONNECTED");

  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(DARKGREY, BLACK);
  M5.Lcd.setCursor(28, 140);
  M5.Lcd.print("Waiting for MSD700...");

  // 背景を塗った直後なので念のためトップバーも再描画
  drawPowerHubStatus();
}

// --- メインUIの描画関数 ---
void drawUI()
{
  M5.Lcd.startWrite();

  // (トップバーの描画は drawPowerHubStatus が担当するため省略)

  // トップバー下の区切り線 & 左右カラム縦線
  M5.Lcd.drawFastHLine(0, 22, 320, DARKGREY);
  M5.Lcd.drawFastVLine(160, 22, 142, DARKGREY);

  // --- 1. セクションヘッダー ---
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(4, 25);
  M5.Lcd.setTextColor(YELLOW, BLACK);
  M5.Lcd.print("  BATTERY   ");
  M5.Lcd.drawFastHLine(0, 42, 159, YELLOW);

  M5.Lcd.setCursor(164, 25);
  M5.Lcd.setTextColor(CYAN, BLACK);
  M5.Lcd.print("   SENSOR   ");
  M5.Lcd.drawFastHLine(161, 42, 159, CYAN);

  // --- 2. 左カラム：バッテリー情報 ---
  M5.Lcd.setTextColor(WHITE, BLACK);
  M5.Lcd.setCursor(4, 46);
  M5.Lcd.printf("Volt:%6.2fV", data.battery_V / 100.0);
  M5.Lcd.setCursor(4, 66);
  M5.Lcd.printf("Curr:%6.2fA", data.battery_I / 100.0);
  M5.Lcd.setTextColor((data.battery_SOC < 200) ? RED : WHITE, BLACK);
  M5.Lcd.setCursor(4, 86);
  M5.Lcd.printf("SOC :%5.1f%% ", data.battery_SOC / 10.0);

  M5.Lcd.setTextColor(WHITE, BLACK);
  M5.Lcd.setCursor(4, 106);
  M5.Lcd.printf("Cap :%5.1fAh", data.soc_mAh / 1000.0);
  M5.Lcd.setCursor(4, 126);
  M5.Lcd.printf("T.Hi:%5.1fC ", data.battery_temp_max / 10.0);
  M5.Lcd.setCursor(4, 146);
  M5.Lcd.printf("T.Lo:%5.1fC ", data.battery_temp_min / 10.0);

  // --- 3. 右カラム：センサー情報 ---
  M5.Lcd.setTextColor(WHITE, BLACK);
  M5.Lcd.setCursor(164, 46);
  M5.Lcd.printf("BnoX:%6.1f ", data.bnoX / 100.0);
  M5.Lcd.setCursor(164, 66);
  M5.Lcd.printf("BnoY:%6.1f ", data.bnoY / 100.0);
  M5.Lcd.setCursor(164, 86);
  M5.Lcd.printf("BnoZ:%6.1f ", data.bnoZ / 100.0);
  M5.Lcd.setCursor(164, 106);
  M5.Lcd.printf("Temp:%5.1fC ", data.temp / 100.0);
  M5.Lcd.setCursor(164, 126);
  M5.Lcd.printf("Humi:%5.1f%% ", data.humid / 100.0);
  M5.Lcd.setCursor(164, 146);
  M5.Lcd.printf("Pres:%4.0fhPa", data.press / 10.0);

  // --- 4. 下部ステータスエリア ---
  M5.Lcd.drawFastHLine(0, 164, 320, DARKGREY);

  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(4, 172);
  M5.Lcd.setTextColor(WHITE, BLACK);
  M5.Lcd.print("Conn: ");
  if (isConnected)
  {
    M5.Lcd.setTextColor(GREEN, BLACK);
    M5.Lcd.print("MSD700");
  }
  else
  {
    M5.Lcd.setTextColor(RED, BLACK);
    M5.Lcd.print("Searching...       ");
  }

  M5.Lcd.setCursor(4, 206);
  if (!sdReady)
  {
    M5.Lcd.setTextColor(RED, BLACK);
    M5.Lcd.print("Log: SD ERROR!          ");
  }
  else if (!isLoggingActive)
  {
    M5.Lcd.setTextColor(YELLOW, BLACK);
    M5.Lcd.print("Log: Waiting for data...");
  }
  else
  {
    M5.Lcd.setTextColor(GREEN, BLACK);
    // %-12s や末尾の空白で古い文字を消去
    M5.Lcd.printf("Log: %-12s [%3d]", fileName, recordCount);
  }

  M5.Lcd.endWrite();
}
// --- SDカードに「最後に使った番号」を保存・読み込みする関数 ---

// 最後に使った番号を読み出す
int getLastIdx()
{
  if (!SD.exists("/last_idx.txt"))
  {
    return 0; // ファイルがなければ0を返す（次は1番になる）
  }
  File f = SD.open("/last_idx.txt", FILE_READ);
  if (!f)
    return 0;

  String s = f.readString();
  f.close();
  return s.toInt();
}

// 最後に使った番号を書き込む
void saveLastIdx(int idx)
{
  File f = SD.open("/last_idx.txt", FILE_WRITE);
  if (f)
  {
    f.print(idx);
    f.close();
  }
}

void writeDataToSD()
{
  if (!sdReady)
    return;

  // 1. 180件に達したら今のファイルを閉じる
  if (isLoggingActive && recordCount >= MAX_RECORDS_PER_FILE)
  {
    logFile.close();
    isLoggingActive = false;
    recordCount = 0;
  }

  // 2. 新しいファイルを作るタイミング
  if (!isLoggingActive)
  {
    // 【current_noの管理】
    // SDカードに保存しておいた「前回値」を読み込んで +1 する
    int last_no = getLastIdx();
    int current_no = (last_no % MAX_LOG_FILES_ROTATION) + 1; // 1〜50をループ

    // ファイル名を作成
    sprintf(fileName, "/data_%02d.csv", current_no);
    char sentFileName[32];
    sprintf(sentFileName, "/data_%02d.sent", current_no);

    // 【50個先（＝次のスロット）の掃除】
    // これから使う番号の古いファイルを、CSVもSENTも両方消しておく
    if (SD.exists(fileName))
      SD.remove(fileName);
    if (SD.exists(sentFileName))
      SD.remove(sentFileName);

    // 新規作成
    logFile = SD.open(fileName, FILE_WRITE);
    if (logFile)
    {
      logFile.println("timestamp,V,I,SOC,mAh,T_Hi,T_Lo,X,Y,Z,Air,Hum,hPa");
      isLoggingActive = true;
      // 「今、何番を使ったか」をSDにメモして、次回の再起動に備える
      saveLastIdx(current_no);
      Serial.printf("Next Slot Prepared: %s\n", fileName);
    }
  }

  // 3. 書き込み（1分おき）
  // 3. 書き込み（1分おき）
  if (logFile && isLoggingActive)
  {
    time_t now = time(NULL); // SDカード保存時のUNIXタイム
    logFile.printf("%lu,%d,%d,%d,%u,%d,%d,%d,%d,%d,%d,%d,%d\n",
                   (unsigned long)now, data.battery_V, data.battery_I, data.battery_SOC, data.soc_mAh,
                   data.battery_temp_max, data.battery_temp_min, data.bnoX, data.bnoY,
                   data.bnoZ, data.temp, data.humid, data.press);
    logFile.flush();
    recordCount++;
  }
}

// SDカード内のファイル数を管理し、古いものを削除する関数
void limitFileCount(int maxFiles)
{
  int count = 0;
  int minIdx = 9999;
  char oldestFileName[32] = "";

  File root = SD.open("/");
  while (File file = root.openNextFile())
  {
    String name = file.name();
    // .csv と .sent の両方をカウント対象にする
    if (name.startsWith("/data_") && (name.endsWith(".csv") || name.endsWith(".sent")))
    {
      count++;
      int idx = name.substring(6, 8).toInt();
      if (idx < minIdx)
      {
        minIdx = idx;
        strcpy(oldestFileName, name.c_str());
      }
    }
    file.close();
  }
  root.close();

  if (count > maxFiles && strlen(oldestFileName) > 0)
  {
    if (SD.remove(oldestFileName))
    {
      Serial.printf("SD Cleaned: Deleted oldest file %s\n", oldestFileName);
    }
  }
}

// --- 1. WiFi接続関数 ---
bool connectWiFi()
{
  // WiFi接続中もESP-NOWの受信を可能にするため、deinitをコメントアウト
  // esp_now_deinit();
  // delay(100);

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  M5.Lcd.setTextColor(WHITE, BLUE);
  M5.Lcd.setCursor(10, 30);
  M5.Lcd.print("Connecting WiFi");

  int retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry < WIFI_CONNECT_MAX_RETRIES)
  { 
    delay(500);
    M5.Lcd.print(".");
    retry++;
  }

  if (WiFi.status() != WL_CONNECTED)
  {
    M5.Lcd.setCursor(10, 50);
    M5.Lcd.setTextColor(RED, BLUE);
    M5.Lcd.println("WiFi Connect Failed!");
    Serial.println("WiFi connection failed");
    return false;
  }
  
  M5.Lcd.setCursor(10, 50);
  M5.Lcd.setTextColor(GREEN, BLUE);
  M5.Lcd.print("WiFi Connected!");
  M5.Lcd.setCursor(10, 70);
  M5.Lcd.printf("IP: %s", WiFi.localIP().toString().c_str());
  Serial.printf("WiFi connected! IP: %s\n", WiFi.localIP().toString().c_str());
  delay(1000);

  return true;
}

// --- クイックWiFi接続関数（リアルタイム送信用）---
bool connectWiFiQuick()
{
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  
  int retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry < 10) // 5秒で諦める
  { 
    delay(500);
    retry++;
  }

  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("Quick WiFi connection failed");
    return false;
  }
  
  Serial.printf("Quick WiFi connected: %s\n", WiFi.localIP().toString().c_str());
  return true;
}

// --- 2. HTTP送信関数（センサーデータ直接送信）---
int sendSensorDataToServer(const SensorData& sensorData, time_t timestamp)
{
  HTTPClient https;
  
  // SSL証明書の検証をスキップ（HTTPS使用時の問題を回避）
  https.begin(UPLOAD_URL);
  // https.setInsecure(); // SSL証明書チェックをスキップ
  
  // 適切なHTTPヘッダーを設定
  https.addHeader("Content-Type", "application/x-www-form-urlencoded");
  https.addHeader("User-Agent", "ESP32-MSD700/1.0");
  https.addHeader("Accept", "application/json");
  https.addHeader("Connection", "close");
  https.setTimeout(15000); // 15秒タイムアウト

  // センサーデータから直接PHP APIパラメータを構築
  String postData = "time=" + String((unsigned long)timestamp) +
                   "&soc=" + String(sensorData.battery_SOC / 10.0, 1) +
                   "&volt=" + String(sensorData.battery_V / 100.0, 2) +
                   "&curr=" + String(sensorData.battery_I / 100.0, 2) +
                   "&t_bat=" + String(sensorData.battery_temp_max / 10.0, 1) +
                   "&t_air=" + String(sensorData.temp / 10.0, 1) +
                   "&hum=" + String(sensorData.humid / 10.0, 1) +
                   "&ax=" + String(sensorData.bnoX / 10.0, 1) +
                   "&ay=" + String(sensorData.bnoY / 10.0, 1) +
                   "&az=" + String(sensorData.bnoZ / 10.0, 1) +
                   "&machine_code=" + String(MACHINE_CODE);

  Serial.printf("\n=== HTTP POST REQUEST ===\n");
  Serial.printf("URL: %s\n", UPLOAD_URL);
  Serial.printf("Method: POST\n");
  Serial.printf("Content-Type: application/x-www-form-urlencoded\n");
  Serial.printf("Data Length: %d\n", postData.length());
  Serial.printf("POST Data: %s\n", postData.c_str());
  Serial.printf("=========================\n");
  
  int httpCode = https.POST(postData);
  
  // レスポンスを詳細に取得してログに出力
  Serial.printf("\n=== HTTP RESPONSE ===\n");
  Serial.printf("HTTP Status Code: %d\n", httpCode);
  
  if (httpCode > 0) {
    String response = https.getString();
    Serial.printf("Response Length: %d\n", response.length());
    Serial.printf("Response Body: %s\n", response.c_str());
    
    // HTTPヘッダー情報も表示
    if (httpCode == 405) {
      Serial.printf("\n*** ERROR 405: Method Not Allowed ***\n");
      Serial.printf("This means the server doesn't accept POST requests at this URL.\n");
      Serial.printf("Please check the endpoint URL and server configuration.\n");
    }
  } else {
    Serial.printf("HTTP Request failed, error code: %d\n", httpCode);
    Serial.printf("Error description: %s\n", https.errorToString(httpCode).c_str());
  }
  Serial.printf("==================\n\n");
  
  https.end();
  return httpCode;
}

// --- CSV送信関数（バックアップファイル用）---
int sendHttpData(double *v)
{
  HTTPClient https;
  https.begin(UPLOAD_URL);
  // https.setInsecure(); // SSL証明書チェックをスキップ
  https.addHeader("Content-Type", "application/x-www-form-urlencoded");
  https.addHeader("User-Agent", "ESP32-MSD700/1.0");
  https.addHeader("Connection", "close");
  https.setTimeout(15000);

  String postData = "time=" + String((unsigned long)v[0]) +
                   "&soc=" + String(v[3] / 10.0, 1) +
                   "&volt=" + String(v[1] / 100.0, 2) +
                   "&curr=" + String(v[2] / 100.0, 2) +
                   "&t_bat=" + String(v[5] / 10.0, 1) +
                   "&t_air=" + String(v[10] / 10.0, 1) +
                   "&hum=" + String(v[11] / 10.0, 1) +
                   "&ax=" + String(v[7] / 10.0, 1) +
                   "&ay=" + String(v[8] / 10.0, 1) +
                   "&az=" + String(v[9] / 10.0, 1) +
                   "&machine_code=" + String(MACHINE_CODE);

  Serial.printf("[CSV Upload] POST to: %s\n", UPLOAD_URL);
  Serial.printf("[CSV Upload] Data: %s\n", postData.c_str());
  
  int httpCode = https.POST(postData);
  
  if (httpCode > 0) {
    String response = https.getString();
    Serial.printf("[CSV Upload] Response %d: %s\n", httpCode, response.c_str());
  } else {
    Serial.printf("[CSV Upload] Failed: %d - %s\n", httpCode, https.errorToString(httpCode).c_str());
  }
  
  https.end();
  return httpCode;
}

// --- 3. SDカードスキャンと送信関数 ---
void scanAndSendSdFiles()
{
  File root = SD.open("/");
  root.rewindDirectory(); // ディレクトリの読み取り位置をリセット

  while (File dataFile = root.openNextFile())
  {
    if (dataUpdated)
    {
      dataFile.close();
      break; // M5Toughからの通信があればファイルループを抜ける
    }

    String fileNameStr = String(dataFile.name());

    // 判定条件: "data_" を含み、かつ ".csv" で終わる未送信ファイル
    if (fileNameStr.indexOf("data_") != -1 && fileNameStr.endsWith(".csv"))
    {
      M5.Lcd.fillRect(0, 80, 320, 100, BLUE);
      M5.Lcd.setTextColor(WHITE, BLUE);
      M5.Lcd.setCursor(10, 90);
      M5.Lcd.printf("Processing: %s", fileNameStr.c_str());

      // 1行目（ヘッダー）を読み飛ばす
      if (dataFile.available())
      {
        dataFile.readStringUntil('\n');
      }

      int rowCount = 0;
      int successCount = 0;
      int errorCount = 0;
      bool fileSuccess = true;

      // ファイルの中身を1行ずつ最後までループ
      while (dataFile.available())
      {
        if (dataUpdated)
        {
          fileSuccess = false;
          break; // M5Toughからの通信があれば行ループを抜ける
        }

        String line = dataFile.readStringUntil('\n');
        line.trim();
        if (line.length() < MIN_VALID_CSV_LINE_LENGTH)
          continue; // 空行や不正行は飛ばす

        // CSVパース（カンマで分割）
        double v[13] = {0};
        char buf[256];
        strncpy(buf, line.c_str(), 256);
        char *p = strtok(buf, ",");
        int c = 0;
        while (p != NULL && c < CSV_FIELD_COUNT)
        {
          v[c++] = atof(p);
          p = strtok(NULL, ",");
        }

        // 必須データ（soc, volt）の検証
        if (v[3] <= 0 || v[1] <= 0) {
          if (DEBUG_CSV_PARSING) {
            Serial.printf("Skipping invalid data row: soc=%.1f, volt=%.2f\n", v[3]/10.0, v[1]/100.0);
          }
          continue;
        }

        if (DEBUG_CSV_PARSING) {
          Serial.printf("Sending data row %d: time=%lu, soc=%.1f, volt=%.2f, curr=%.2f\n", 
                       rowCount + 1, (unsigned long)v[0], v[3]/10.0, v[1]/100.0, v[2]/100.0);
        }

        rowCount++;
        
        // 分離したHTTP関数で送信
        int httpCode = sendHttpData(v);

        if (httpCode == 200)
        {
          successCount++;
          M5.Lcd.setCursor(10, 110);
          M5.Lcd.setTextColor(GREEN, BLUE);
          M5.Lcd.printf("Success: %d/%d rows    ", successCount, rowCount);
        }
        else
        {
          errorCount++;
          M5.Lcd.setCursor(10, 130);
          M5.Lcd.setTextColor(RED, BLUE);
          M5.Lcd.printf("Errors: %d (Code: %d)    ", errorCount, httpCode);
          
          // 3回連続エラーが出たらそのファイルは中断
          if (errorCount >= 3) {
            Serial.println("Too many consecutive errors, aborting file");
            fileSuccess = false;
            break;
          }
        }
        
        delay(100); // サーバー負荷軽減のため少し待機
      }
      dataFile.close();

      // ファイル処理結果の表示
      if (fileSuccess && successCount > 0)
      {
        // 1. パスの整合性を整える（先頭に / を付ける）
        String oldPath = fileNameStr;
        if (!oldPath.startsWith("/"))
          oldPath = "/" + oldPath;

        String newPath = oldPath;
        newPath.replace(".csv", ".sent");

        // 2. 同名の .sent が既にある場合は削除しておく（上書き失敗防止）
        if (SD.exists(newPath))
        {
          SD.remove(newPath);
        }

        // 3. リネーム実行
        if (SD.rename(oldPath.c_str(), newPath.c_str()))
        {
          M5.Lcd.setCursor(10, 150);
          M5.Lcd.setTextColor(GREEN, BLUE);
          M5.Lcd.printf("✓ Uploaded %d rows successfully", successCount);
          Serial.printf("File successfully uploaded and renamed: %s -> %s\n", oldPath.c_str(), newPath.c_str());
        }
        else
        {
          M5.Lcd.setCursor(10, 150);
          M5.Lcd.setTextColor(RED, BLUE);
          M5.Lcd.printf("Rename failed: %s", oldPath.c_str());
        }
      }
      else if (errorCount > 0)
      {
        M5.Lcd.setCursor(10, 150);
        M5.Lcd.setTextColor(YELLOW, BLUE);
        M5.Lcd.printf("Partial upload: %d/%d rows", successCount, rowCount);
      }
      
      delay(1000); // 結果表示の時間
    }
    else
    {
      dataFile.close(); // 対象外のファイルは閉じる
    }
  }
  root.close();
}

// --- PowerHubデータ受信関数 ---
bool readPowerHubData()
{
  // 0xAA ヘッダーを探して同期
  while (Serial2.available() && Serial2.peek() != 0xAA)
  {
    Serial2.read(); // ヘッダー以外は捨てる
  }

  // データが揃っているか確認して読み込む
  if (Serial2.available() >= (int)sizeof(HubData))
  {
    HubData tmp;
    Serial2.readBytes((uint8_t *)&tmp, sizeof(HubData));

    if (tmp.header == 0xAA && tmp.checksum == calcChecksum(tmp))
    {
      powerHubData = tmp;
      powerHubDataValid = true;

      return true; // 正常に受信
    }
  }
  return false; // 受信失敗またはデータ不足
}

// --- データベース送信関数 (バックアップファイル用のみ) ---
void uploadToDatabase()
{
  M5.Lcd.fillScreen(BLUE);
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(10, 10);
  M5.Lcd.println("[HTTP ALL SYNC MODE]");

  // --- 充電状態の確認（数秒待機して最新情報を取得） ---
  M5.Lcd.setCursor(10, 50);
  M5.Lcd.println("Checking charger status...");

  unsigned long checkStartTime = millis();

  // 最大3秒間、充電中(給電中)のデータが来るのを待つ
  while (millis() - checkStartTime < 3000)
  {
    if (readPowerHubData())
    {
      if (powerHubData.charging)
      {
        break;
      }
    }
    M5.Lcd.print(".");
    delay(100);
  }

  // ---【診断用】---
  M5.Lcd.fillRect(0, 50, 320, 50, BLUE);
  M5.Lcd.setCursor(10, 50);
  M5.Lcd.printf("Check Done. Valid:%d Charging:%d", powerHubDataValid, powerHubData.charging);
  delay(3000);
  // -----------------

  // 最新の充電状態で判断し、充電中でなければ送信を中止する
  if (!powerHubDataValid || !powerHubData.charging)
  {
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setTextColor(YELLOW);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setCursor(10, 50);
    M5.Lcd.println("Upload Skipped:");
    M5.Lcd.setCursor(10, 80);
    M5.Lcd.println("Not Charging.");
    delay(3000);
    M5.Lcd.fillRect(0, 20, 320, 220, BLACK); // トップバー以外を黒くして戻る
    return;
  }

  // 1. WiFi接続
  if (!connectWiFi())
  {
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setCursor(10, 50);
    M5.Lcd.setTextColor(RED, BLACK);
    M5.Lcd.println("WiFi connection failed!");
    M5.Lcd.setCursor(10, 70);
    M5.Lcd.setTextColor(WHITE, BLACK);
    M5.Lcd.println("Check WiFi settings");
    delay(3000);
    WiFi.disconnect();                       // WiFiをオフに戻す
    M5.Lcd.fillRect(0, 20, 320, 220, BLACK); // トップバー以外を黒くして戻る
    return;
  }

  // 2. SDカードスキャンとHTTP送信
  M5.Lcd.fillRect(0, 80, 320, 160, BLUE);
  M5.Lcd.setCursor(10, 90);
  M5.Lcd.setTextColor(WHITE, BLUE);
  M5.Lcd.println("Starting data upload...");
  delay(1000);
  
  scanAndSendSdFiles();
  
  M5.Lcd.setCursor(10, 170);
  M5.Lcd.setTextColor(YELLOW, BLUE);
  M5.Lcd.println("Disconnecting WiFi...");
  WiFi.disconnect();

  if (dataUpdated)
  {
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setCursor(10, 50);
    M5.Lcd.setTextColor(YELLOW);
    M5.Lcd.println("Upload Aborted by MSD700!");
    delay(2000);
    M5.Lcd.fillRect(0, 20, 320, 220, BLACK); // トップバー以外を黒くして戻る
  }
  else
  {

    M5.Lcd.fillScreen(BLACK);
    drawPowerHubStatus(); // トップバーを描画

    M5.Lcd.setTextSize(3);
    M5.Lcd.setTextColor(GREEN, BLACK);
    M5.Lcd.setCursor(61, 110); // 画面の中央付近に配置
    M5.Lcd.print("CHARGING...");

    // 充電ケーブルが抜かれるか、M5Toughからデータが届くまで待機
    while (true)
    {
      M5.update();
      readPowerHubData();

      // 時計を進めるための1秒ごとの更新
      static unsigned long lastClockUpdate = 0;
      if (millis() - lastClockUpdate > 1000)
      {
        drawPowerHubStatus();
        lastClockUpdate = millis();
      }

      // 充電が終了した、またはM5Toughから新しいデータが届いた場合はループを抜ける
      if ((powerHubDataValid && !powerHubData.charging) || dataUpdated)
      {
        break;
      }
      delay(10);
    }

    // 充電が終わったら、上部（トップバー）以外をすべて真っ黒にする
    M5.Lcd.fillRect(0, 20, 320, 220, BLACK);
    drawPowerHubStatus();
  }
  // ESP.restart(); は削除し、そのまま return でメインループに戻る
  return;
}
// --- 7. 通信ハンドラ ---

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
void onDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len)
{
#else
void onDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len)
{
#endif
  if (len == sizeof(data))
  {
    memcpy(&data, incomingData, sizeof(data));
    dataUpdated = true;
    lastRecvTime = millis();
  }
}

// --- 初期設定 & メインループ ---

void setup()
{
  // タイムゾーン設定（日本時間）
  // setenv("TZ", "JST-9", 1);
  // tzset();
  auto cfg = M5.config();
  cfg.output_power = false;
  M5.begin(cfg);

  // Serial1: ジョイスティック用
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, 44, 43);

  // Serial2: PowerHub用
  Serial2.begin(UART_BAUD, SERIAL_8N1, UART_RX, UART_TX);

  if (SD.begin(GPIO_NUM_4, SPI, 40000000))
    sdReady = true;

  // 再起動直後の「真っ暗画面」を防ぐための設定
  lastRecvTime = millis() - (TIMEOUT_MS + 1000);
  isConnected = false;

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK)
    ESP.restart();
  esp_now_register_recv_cb(onDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, targetAddress, 6);
  esp_now_add_peer(&peerInfo);

  drawDisconnectedScreen(); // 起動時は必ず待機画面
}

void setRTCTime()
{
  // ESP32のシステム時刻をセット
  struct timeval tv;
  tv.tv_sec = powerHubData.timestamp;
  tv.tv_usec = 0;
  settimeofday(&tv, NULL);

  // M5Stack内部のRTC（ハードウェア時計）にもセット
  time_t t = powerHubData.timestamp;
  struct tm *tm_info = gmtime(&t);
  m5::rtc_datetime_t rtc_dt;
  rtc_dt.date.year = tm_info->tm_year + 1900;
  rtc_dt.date.month = tm_info->tm_mon + 1;
  rtc_dt.date.date = tm_info->tm_mday;
  rtc_dt.time.hours = tm_info->tm_hour;
  rtc_dt.time.minutes = tm_info->tm_min;
  rtc_dt.time.seconds = tm_info->tm_sec;
  M5.Rtc.setDateTime(rtc_dt);

  isTimeSynced = true; // 同期完了フラグを立てる（二度と同期しない）
  Serial.println("Time Synced with PowerHub!");
}

void loop()
{
  M5.update();
  if (!isTimeSynced && powerHubDataValid && powerHubData.timestamp >= 1704067200)
  {
    setRTCTime();
  }
  // 5秒以上データが届いていないか？
  bool isTimeout = (millis() - lastRecvTime > TIMEOUT_MS);

  if (isTimeout)
  {
    if (isConnected)
    {
      isConnected = false; // 切断状態へ移行
      if (isLoggingActive)
      {
        logFile.close();
        isLoggingActive = false;
      }
      // 通信が切れた瞬間に1回だけ待機画面を描画する
      drawDisconnectedScreen();
    }

    // --- 送信待ちファイルがあるか定期的にチェック ---
    static unsigned long lastSdCheck = 0;
    if (millis() - lastSdCheck > SD_SCAN_INTERVAL_MS)
    { // 5秒おきにSDをスキャン
      lastSdCheck = millis();

      bool hasCsv = false;
      File root = SD.open("/");
      root.rewindDirectory(); // 読み取り位置をリセット
      while (File f = root.openNextFile())
      {
        String n = String(f.name());
        // 判定を indexOf("data_") に変更（スラッシュの有無を問わない）
        if (n.indexOf("data_") != -1 && n.endsWith(".csv"))
        {
          hasCsv = true;
          f.close();
          break;
        }
        f.close();
      }
      root.close();

      if (hasCsv)
      {
        Serial.println(">>> Found pending CSV files. Starting Upload...");
        uploadToDatabase(); // 強制的に送信モードへ
        return;
      }
    }
  }
  else
  {
    if (!isConnected)
    {
      isConnected = true;
      M5.Lcd.fillRect(0, 20, 320, 220, BLACK); // 復帰時に画面を一度クリアする
    }
    // --- 通信成功時 ---
    if (dataUpdated)
    {
      drawUI();
      dataUpdated = false;

      // 時刻が同期済みなら処理を開始する
      if (isTimeSynced)
      {
        time_t currentTime = time(NULL);
        
        // 1. SDカードへの書き込み（バックアップ用）
        if (!isLoggingActive || (millis() - lastSdWriteTime >= WRITE_INTERVAL))
        {
          writeDataToSD();
          lastSdWriteTime = millis();
        }
        
        // 2. リアルタイムサーバーアップロード (充電中のみ実行 — 電池駆動時の WiFi 消費を回避)
        if (!uploadInProgress
            && (millis() - lastUploadTime >= UPLOAD_INTERVAL)
            && powerHubDataValid
            && powerHubData.charging)
        {
          uploadInProgress = true;

          // WiFi接続してセンサーデータを直接送信
          if (connectWiFiQuick())
          {
            Serial.printf("\n*** ATTEMPTING LIVE DATA UPLOAD ***\n");
            Serial.printf("WiFi Connected: %s\n", WiFi.localIP().toString().c_str());
            
            int httpCode = sendSensorDataToServer(data, currentTime);
            
            if (httpCode == 200)
            {
              Serial.println("✓ Live data uploaded successfully");
            }
            else if (httpCode == 405)
            {
              Serial.printf("✗ HTTP 405 Error - Server doesn't accept POST at this URL\n");
              Serial.printf("Check if the endpoint URL is correct: %s\n", UPLOAD_URL);
            }
            else
            {
              Serial.printf("✗ Live upload failed with HTTP code: %d\n", httpCode);
            }
            
            WiFi.disconnect();
            Serial.printf("*** UPLOAD ATTEMPT COMPLETED ***\n\n");
          }
          else
          {
            Serial.println("WiFi connection failed for live upload");
          }
          
          lastUploadTime = millis();
          uploadInProgress = false;
        }
      }
      else
      {
        Serial.println("Skip processing - time not synced");
      }
    }
  } // M5Toughへのジョイスティック送信（既存）

  // --- Serial1からのデータ受信 (Joystick) ---
  static String receivedString = "";
  while (Serial1.available())
  {
    char c = Serial1.read();
    if (c == '\n')
    {
      if (sscanf(receivedString.c_str(), "%hu,%hu,%hu,%hu,%hu,%hu,%hu",
                 &joyData.btn, &joyData.rx, &joyData.ry, &joyData.r_knob,
                 &joyData.lx, &joyData.ly, &joyData.l_knob) == 7)
      {
        esp_now_send(targetAddress, (uint8_t *)&joyData, sizeof(joyData));
      }
      receivedString = "";
    }
    else if (receivedString.length() < MAX_JOYSTICK_INPUT_LENGTH)
    {
      receivedString += c;
    }
  }

  // --- Serial2からのデータ受信 (PowerHub) ---
  readPowerHubData();
  static unsigned long lastClockUpdate = 0;
  if (millis() - lastClockUpdate > 1000)
  {
    drawPowerHubStatus();
    lastClockUpdate = millis();
  }
}
