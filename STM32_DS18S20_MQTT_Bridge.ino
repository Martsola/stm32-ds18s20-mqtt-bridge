#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <EEPROM.h>

#define ENABLE_WATCHDOG 1
#define CONFIG_RESET_PIN PB12  // Hold LOW 10s at power-on for factory reset
// Added button timing + debug flags
#ifndef BTN_SHORT_MIN_MS
#define BTN_SHORT_MIN_MS 50
#endif
#ifndef BTN_LONG_MS
#define BTN_LONG_MS 10000
#endif
#ifndef BTN_DEBUG
#define BTN_DEBUG 0
#endif

// Watchdog control
bool watchdogStarted = false;
const unsigned long WATCHDOG_START_DELAY = 15000; // start WDT 15s after boot

// Safe mode flag if network failed to initialize properly
bool safeMode = false;

// Non-blocking temperature cycle state
bool tempRequestPending = false;
unsigned long tempRequestTime = 0;
const unsigned long TEMP_CONVERSION_MS = 1200; // 12-bit DS18S20 conversion
unsigned long temperatureInterval = 30000; // 30s

// Forward declarations
void startTemperatureRequests();
void completeTemperatureCycle();
void checkFactoryResetButton();

// --- Watchdog (STM32 independent watchdog) ---
static inline void watchdogInit() {
#if ENABLE_WATCHDOG && defined(IWDG)
  if (watchdogStarted) return;
  // Use longest practical timeout (~26s): prescaler /256, reload near max
  IWDG->KR = 0x5555;     // Enable access
  IWDG->PR = 0x06;       // Prescaler 256
  IWDG->RLR = 4000;      // Reload (<=4095). 4001 / (40000/256) ≈ 25.6s
  IWDG->KR = 0xCCCC;     // Start
  IWDG->KR = 0xAAAA;     // First kick
  watchdogStarted = true;
  Serial.println("[WDT] Watchdog started (~25s timeout)");
#endif
}
static inline void watchdogKick() {
#if ENABLE_WATCHDOG && defined(IWDG)
  if (watchdogStarted) IWDG->KR = 0xAAAA;
#endif
}

// --- Added: CRC16 helper for configuration integrity ---
uint16_t crc16_update(uint16_t crc, uint8_t b) {
  crc ^= (uint16_t)b << 8;
  for (uint8_t i = 0; i < 8; i++) {
    if (crc & 0x8000) crc = (crc << 1) ^ 0x1021; else crc <<= 1;
  }
  return crc;
}

uint16_t computeConfigCRC(size_t lengthWithoutCRC) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < lengthWithoutCRC; i++) {
    crc = crc16_update(crc, EEPROM.read(i));
  }
  return crc;
}
// --- End CRC16 helper ---

// Structure to hold sensor information
struct SensorInfo {
  uint8_t address[8];
  int bus;
  int index;
  bool isConnected;
  char serialHex[17];
};

// Network configuration
byte mac[] = { 0x02, 0x34, 0x56, 0x78, 0x9A, 0xBC };
IPAddress ip;
IPAddress subnet(255, 255, 255, 0);  // Default subnet mask
IPAddress gateway;                    // Gateway IP
IPAddress dns1;                       // Primary DNS server
bool useDhcp = true;

// MQTT configuration
char mqtt_server[40] = "";  // Default MQTT server
char mqtt_username[32] = "";  // Default username
char mqtt_password[32] = "";  // Default password
int mqtt_port = 1883;

bool discoveryPublished = false;
unsigned long lastDiscoveryAttempt = 0;
const unsigned long DISCOVERY_RETRY_INTERVAL = 60000; // Retry every 60 seconds

// Web server
EthernetServer server(80);

// MQTT client
EthernetClient ethClient;
PubSubClient mqtt(ethClient);

// OneWire buses configuration
const int ONE_WIRE_BUS_COUNT = 4;
const int ONE_WIRE_PINS[ONE_WIRE_BUS_COUNT] = {PA0, PA1, PA2, PA3}; // 5V tolerant pins
OneWire* oneWireBuses[ONE_WIRE_BUS_COUNT];
DallasTemperature* sensors[ONE_WIRE_BUS_COUNT];

// Sensor tracking
SensorInfo connectedSensors[12];  // Max 12 sensors (3 per bus × 4 buses)
int activeSensorCount = 0;

// EEPROM configuration addresses
const int EEPROM_DHCP_ENABLED = 0;
const int EEPROM_IP_START = 1;
const int EEPROM_SUBNET_START = 5;
const int EEPROM_GATEWAY_START = 9;
const int EEPROM_DNS_START = 13;
const int EEPROM_MQTT_SERVER_START = 17;
const int EEPROM_MQTT_USERNAME_START = 57;
const int EEPROM_MQTT_PASSWORD_START = 89;
const byte EEPROM_VALID_FLAG = 0xAA;
const int EEPROM_VALID_FLAG_ADDR = 121;

// Pin definitions
const int W5500_CS_PIN = PA4;    // SPI CS for W5500
const int W5500_SCK_PIN = PA5;   // SPI SCK for W5500
const int W5500_MISO_PIN = PA6;  // SPI MISO for W5500
const int W5500_MOSI_PIN = PA7;  // SPI MOSI for W5500
const int W5500_RST_PIN = PB0;   // SPI Reset for W5500

// Helper function to convert address to hex string
void addressToHex(uint8_t* addr, char* hexStr) {
  for(int i = 0; i < 8; i++) {
    sprintf(hexStr + (i * 2), "%02X", addr[i]);
  }
  hexStr[16] = '\0';
}

// Function to scan and catalog all connected sensors
void scanSensors() {
  activeSensorCount = 0;
  for(int bus = 0; bus < ONE_WIRE_BUS_COUNT; bus++) {
    sensors[bus]->begin();
    delay(500);
    int deviceCount = 0;
    for(int retry = 0; retry < 3; retry++) {
      deviceCount = sensors[bus]->getDeviceCount();
      if(deviceCount > 0) break;
      if(retry < 2) delay(200);
    }
    for(int i = 0; i < deviceCount && activeSensorCount < 12; i++) {
      uint8_t addr[8];
      bool addressFound = false;
      for(int retry = 0; retry < 5; retry++) {
        if(sensors[bus]->getAddress(addr, i)) { addressFound = true; break; }
        delay(100);
      }
      if(addressFound) {
        // CRC8 validation (Dallas address last byte is CRC of first 7 bytes)
        if (OneWire::crc8(addr, 7) != addr[7]) {
          Serial.print("Bad CRC addr bus "); Serial.print(bus); Serial.print(" idx "); Serial.println(i);
          continue; // skip invalid
        }
        sensors[bus]->requestTemperaturesByAddress(addr);
        delay(1200);
        float testTemp = sensors[bus]->getTempC(addr);
        if(testTemp != DEVICE_DISCONNECTED_C && testTemp > -55 && testTemp < 125) {
          memcpy(connectedSensors[activeSensorCount].address, addr, 8);
          connectedSensors[activeSensorCount].bus = bus;
          connectedSensors[activeSensorCount].index = i;
          connectedSensors[activeSensorCount].isConnected = true;
          addressToHex(addr, connectedSensors[activeSensorCount].serialHex);
          activeSensorCount++;
        }
      } else {
        Serial.print("Failed get addr sensor "); Serial.print(i); Serial.print(" bus "); Serial.println(bus);
      }
    }
  }
  Serial.print("Total active sensors: "); Serial.println(activeSensorCount);
}

// EEPROM Configuration
#define EEPROM_START_ADDR 0
#define CONFIG_VERSION "V0"  // Updated version due to checksum & new features
#define CONFIG_START 32
// Reserve last two bytes of first 256 bytes for CRC (simple fixed region)
const int CONFIG_MAX_BYTES = 254; // bytes 254 & 255 hold CRC16
const int CONFIG_CRC_L = 254;
const int CONFIG_CRC_H = 255;

// Uptime / availability
unsigned long bootMillis = 0;
unsigned long lastAvailRefresh = 0;
const unsigned long AVAIL_REFRESH_INTERVAL = 300000; // 5 min

// MQTT backoff
unsigned long nextMqttAttempt = 0;
unsigned long mqttBackoff = 5000; // start 5s
const unsigned long MQTT_BACKOFF_MAX = 60000;

// Uptime publishing (minimal, optional)
#define ENABLE_UPTIME 1  // set to 0 to compile out uptime feature
#if ENABLE_UPTIME
static unsigned long lastUptimeSec = 0; // last published seconds value
#endif

// Function to resolve hostname using DNS
void setup() {
  Serial.begin(115200);
  delay(800);
  Serial.println("BOOT");
  bootMillis = millis();
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  pinMode(CONFIG_RESET_PIN, INPUT_PULLUP);
  // Fast factory reset: pin low immediately at boot triggers instant defaults
  if (digitalRead(CONFIG_RESET_PIN)==LOW) {
    delay(40); // brief debounce
    if (digitalRead(CONFIG_RESET_PIN)==LOW) {
      Serial.println("Boot reset");
      initializeDefaultConfig();
      if (saveConfiguration()) Serial.println("Defaults saved"); else Serial.println("Default save ERR");
      unsigned long wStart = millis();
      while (digitalRead(CONFIG_RESET_PIN)==LOW && millis()-wStart < 5000) { watchdogKick(); delay(10); }
      Serial.println("Reboot...");
      delay(200);
      rebootDevice();
    }
  }
  checkFactoryResetButton(); // EARLY: allows wiping before reading config
  if (!verifyConfigurationVersion()) {
    initializeDefaultConfig();
  } else {
    if (!loadConfiguration()) {
      initializeDefaultConfig();
    }
  }
  
  // Initialize pins for W5500
  pinMode(W5500_CS_PIN, OUTPUT);
  pinMode(W5500_RST_PIN, OUTPUT);
  pinMode(W5500_MISO_PIN, INPUT);
  pinMode(W5500_MOSI_PIN, OUTPUT);
  pinMode(W5500_SCK_PIN, OUTPUT);
  
  digitalWrite(W5500_CS_PIN, HIGH);
  
  // Reset W5500
  digitalWrite(W5500_RST_PIN, HIGH);
  delay(100);
  digitalWrite(W5500_RST_PIN, LOW);
  delay(100);
  digitalWrite(W5500_RST_PIN, HIGH);
  delay(1000);
  
  // Initialize SPI and Ethernet
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV4);
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  Ethernet.init(W5500_CS_PIN);
  
  // Setup network
  setupNetwork();
  
  // Initialize OneWire buses and sensors with long wire settings
  for(int i = 0; i < ONE_WIRE_BUS_COUNT; i++) {
    oneWireBuses[i] = new OneWire(ONE_WIRE_PINS[i]);
    sensors[i] = new DallasTemperature(oneWireBuses[i]);
    sensors[i]->begin();
    sensors[i]->setResolution(12);
    sensors[i]->setWaitForConversion(true);
    sensors[i]->setCheckForConversion(true);
  }
  
  // Allow extra time for long wire sensors to stabilize
  delay(2000);
  
  // Scan for sensors and get their serial numbers
  scanSensors();
  
  // Initialize MQTT
  mqtt.disconnect();  // Ensure clean state
  
  // Resolve MQTT server address if needed
  IPAddress serverIP;
  if (resolveMqttServer(serverIP)) {
    mqtt.setServer(serverIP, mqtt_port);
  } else {
    mqtt.setServer(mqtt_server, mqtt_port);
  }
  mqtt.setBufferSize(1024); // Increase MQTT buffer size
  
  // Start web server
  server.begin();
  Serial.print("Web interface available at http://");
  Serial.println(Ethernet.localIP());
  
  digitalWrite(LED_BUILTIN, LOW);  // Setup complete
  Serial.println("Setup OK");
}

void setupNetwork() {
  Serial.println("Net setup");
  Serial.print("DHCP ="); Serial.println(useDhcp ? "1" : "0");
  Serial.flush();
  unsigned long netStart = millis();
  bool done = false;
  uint8_t attempts = 0;
  while(!done && attempts < 3) {
    attempts++;
    if (!useDhcp) {
      uint32_t ip32 = ((uint32_t)ip[0]<<24)|((uint32_t)ip[1]<<16)|((uint32_t)ip[2]<<8)|ip[3];
      uint32_t gw32 = ((uint32_t)gateway[0]<<24)|((uint32_t)gateway[1]<<16)|((uint32_t)gateway[2]<<8)|gateway[3];
      uint32_t mask32 = ((uint32_t)subnet[0]<<24)|((uint32_t)subnet[1]<<16)|((uint32_t)subnet[2]<<8)|subnet[3];
      bool invalid = (ip[0]==0 || ip[0]==255 || ((ip32 & mask32)!=(gw32 & mask32)) || ip[3]==0 || ip[3]==255);
      if (invalid) {
        Serial.println("Bad static -> DHCP");
        useDhcp = true;
      }
    }
    if (useDhcp) {
      Serial.println("DHCP...");
      if (Ethernet.begin(mac)) {
        Serial.print("DHCP IP ="); Serial.println(Ethernet.localIP());
        ip = Ethernet.localIP(); subnet = Ethernet.subnetMask(); gateway = Ethernet.gatewayIP(); dns1 = Ethernet.dnsServerIP();
        done = true; break;
      } else {
        Serial.println("DHCP fail");
        if (attempts >= 2) { // switch to fallback static
          ip = IPAddress(192,168,1,200);
          subnet = IPAddress(255,255,255,0);
            gateway = IPAddress(192,168,1,1);
            dns1 = IPAddress(8,8,8,8);
            useDhcp = false; // now try static path
        }
      }
    }
    if (!useDhcp) {
      Serial.print("Static IP ="); Serial.println(ip);
      Ethernet.begin(mac, ip, dns1, gateway, subnet);
      Serial.print("Local ="); Serial.println(Ethernet.localIP());
      done = true;
    }
  }
  // Link check with timeout
  if (done) {
    unsigned long linkDeadline = millis() + 4000;
    while (Ethernet.linkStatus()==LinkOFF && millis()<linkDeadline) { delay(100); watchdogKick(); }
    if (Ethernet.linkStatus()==LinkOFF) {
      Serial.println("Link down");
      safeMode = true;
    }
  } else {
    safeMode = true;
  }
  Serial.print("Net ms:"); Serial.println(millis()-netStart);
}

void loop() {
  unsigned long _loopStart = millis();
  EthernetClient client = server.available();
  if(client) { Serial.println("\nNew web connection detected"); handleWebClient(client); delay(1); }
  static unsigned long lastTempTrigger = 0; static unsigned long lastCheck = 0; static unsigned long prevMillis = 0; unsigned long currentMillis = millis();
  // Millis rollover handling (about every 49.7 days)
  if (currentMillis < prevMillis) {
    Serial.println(F("[TIME] rollover"));
    lastTempTrigger = currentMillis; lastCheck = currentMillis; /* lastUptimePublish = currentMillis; */ lastAvailRefresh = currentMillis; lastDiscoveryAttempt = currentMillis; nextMqttAttempt = currentMillis; tempRequestTime = currentMillis;
  }
  prevMillis = currentMillis;
  if (currentMillis - lastCheck >= 10000) { Ethernet.maintain(); if(Ethernet.linkStatus()==LinkOFF) Serial.println("WARN: link down"); lastCheck = currentMillis; }
  // Single delayed rescan shortly after boot if first scan produced zero sensors
  if(!discoveryPublished && activeSensorCount==0){ static bool earlyRescanDone=false; if(!earlyRescanDone && (millis()-bootMillis)>5000){ Serial.println("Early rescan (initial empty)"); scanSensors(); earlyRescanDone=true; } }
  if(!mqtt.connected()) reconnectMqtt(); else {
    mqtt.loop();
    if (!discoveryPublished && (currentMillis - lastDiscoveryAttempt >= 10000)) {
      if(activeSensorCount==0){ Serial.println("Rescan sensors"); scanSensors(); }
      if(activeSensorCount>0){
        Serial.println("HA disc...");
        publishDiscoveryMessages();
      } else {
        Serial.println("No sensors, retry");
      }
      lastDiscoveryAttempt = currentMillis;
    }
  }
  if (mqtt.connected() && discoveryPublished) {
    if (!tempRequestPending && (currentMillis - lastTempTrigger >= temperatureInterval)) { startTemperatureRequests(); lastTempTrigger = currentMillis; }
    completeTemperatureCycle();
  }
  if (mqtt.connected()) {
    // Removed periodic uptime publish to save flash
    if (currentMillis - lastAvailRefresh >= AVAIL_REFRESH_INTERVAL) {
      char device_id[13]; snprintf(device_id, sizeof(device_id), "%02x%02x%02x%02x%02x%02x", mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
      char willTopic[64]; snprintf(willTopic, sizeof(willTopic), "homeassistant/bridge/%s/status", device_id);
      mqtt.publish(willTopic, "online", true); lastAvailRefresh = currentMillis;
    }
#if ENABLE_UPTIME
    if (discoveryPublished) {
      unsigned long sec = currentMillis / 1000UL;
      if (sec != lastUptimeSec && (sec % 60UL)==0) {
        char device_id[13]; snprintf(device_id, sizeof(device_id), "%02x%02x%02x%02x%02x%02x", mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
        char topic[72]; snprintf(topic, sizeof(topic), "homeassistant/bridge/%s/uptime", device_id);
        char up[12]; int p=11; up[p]=0; unsigned long v=sec; do { up[--p]='0'+(v%10); v/=10; } while(v && p);
        mqtt.publish(topic, &up[p], true);
        lastUptimeSec = sec;
      }
    }
#endif
  }
  // Link change detection: when link comes up after being down, retry network + schedule immediate MQTT attempt
  {
    static uint8_t lastLink = 255; uint8_t ls = Ethernet.linkStatus();
    if (lastLink == 255) lastLink = ls; else if (ls != lastLink) {
      if (ls == LinkON) { Serial.println("Link up, net retry"); setupNetwork(); nextMqttAttempt = 0; }
      else Serial.println("Link lost");
      lastLink = ls;
    }
  }
  if (!watchdogStarted && !safeMode && (millis() - bootMillis) > WATCHDOG_START_DELAY) watchdogInit();
  watchdogKick();
  // PB12 button: short press (>=BTN_SHORT_MIN_MS < BTN_LONG_MS) reboot, long press (>=BTN_LONG_MS) factory reset
  {
    static unsigned long holdStart = 0; static bool reportedHold = false;
    int lvl = digitalRead(CONFIG_RESET_PIN);
    if (lvl==LOW) {
      if (holdStart==0) {
        holdStart = millis();
        reportedHold=false;
#if BTN_DEBUG
        Serial.println("BTN down");
#endif
      } else if (!reportedHold) {
        unsigned long dt = millis()-holdStart;
        if (dt>500) {
          reportedHold=true;
#if BTN_DEBUG
          Serial.print("BTN holding "); Serial.print(dt); Serial.println("ms");
#endif
        }
      }
      if (holdStart && (millis() - holdStart) >= BTN_LONG_MS) {
        Serial.println("[RST] Factory reset"); initializeDefaultConfig(); saveConfiguration(); delay(120); rebootDevice();
      }
    } else if (holdStart) {
      unsigned long d = millis() - holdStart; if (d >= BTN_SHORT_MIN_MS && d < BTN_LONG_MS) { Serial.print("[RST] Reboot short press "); Serial.print(d); Serial.println("ms"); rebootDevice(); }
#if BTN_DEBUG
      else { Serial.print("BTN up dur="); Serial.print(d); Serial.println("ms (ignored)"); }
#endif
      holdStart = 0; reportedHold=false;
    }
  }
#if ENABLE_OVERRUN
  unsigned long loopDur = millis() - _loopStart;
  if (loopDur > OVR_THRESHOLD_MS) { static uint8_t ovc=0; if (ovc < 5) { Serial.print(F("[OVR]")); Serial.println(loopDur); ovc++; } }
#endif
}

// Reboot helper (centralize to save duplicate code)
static inline void rebootDevice(){ delay(50); NVIC_SystemReset(); }

void initializeDefaultConfig() {
  Serial.println("Defaults set");
  useDhcp = true;
  ip = IPAddress(192, 168, 1, 200); // fallback only
  subnet = IPAddress(255, 255, 255, 0);
  gateway = IPAddress(192, 168, 1, 1);
  dns1 = IPAddress(8, 8, 8, 8);
  mqtt_server[0] = '\0';
  mqtt_username[0] = '\0';
  mqtt_password[0] = '\0';
  mqtt_port = 1883;
  
  // Not writing to EEPROM - this will happen only when user saves via web UI
}

bool loadConfiguration() {
  Serial.println("Load cfg");
  int addr = sizeof(CONFIG_VERSION);
  EEPROM.get(addr, useDhcp);              addr += sizeof(useDhcp);
  EEPROM.get(addr, ip);                   addr += sizeof(ip);
  EEPROM.get(addr, subnet);               addr += sizeof(subnet);
  EEPROM.get(addr, gateway);              addr += sizeof(gateway);
  EEPROM.get(addr, dns1);                 addr += sizeof(dns1);
  char storedServer[sizeof(mqtt_server)];
  EEPROM.get(addr, storedServer);         addr += sizeof(mqtt_server);
  if (strlen(storedServer) > 0) strncpy(mqtt_server, storedServer, sizeof(mqtt_server)-1);
  char storedUsername[sizeof(mqtt_username)];
  EEPROM.get(addr, storedUsername);       addr += sizeof(mqtt_username);
  if (strlen(storedUsername) > 0) strncpy(mqtt_username, storedUsername, sizeof(mqtt_username)-1);
  char storedPassword[sizeof(mqtt_password)];
  EEPROM.get(addr, storedPassword);       addr += sizeof(mqtt_password);
  if (strlen(storedPassword) > 0) strncpy(mqtt_password, storedPassword, sizeof(mqtt_password)-1);
  EEPROM.get(addr, mqtt_port);            addr += sizeof(mqtt_port);
  // Compute CRC over bytes 0..253 (exclude CRC storage bytes 254,255)
  uint16_t stored = EEPROM.read(CONFIG_CRC_L) | (EEPROM.read(CONFIG_CRC_H) << 8);
  uint16_t calc = computeConfigCRC(CONFIG_CRC_L);
  if (stored != calc) {
    Serial.print("Config CRC mismatch st="); Serial.print(stored, HEX); Serial.print(" calc="); Serial.println(calc, HEX);
    return false;
  }
  if (!useDhcp) {
    if (ip[0]==0 || subnet[0]==0 || gateway[0]==0) {
      Serial.println("Invalid static IP, switching DHCP");
      useDhcp = true;
      return false;
    }
  }
  Serial.println("Cfg OK");
  return true;
}

bool saveConfiguration() {
  Serial.println("Save cfg");
  int addr = 0;
  EEPROM.put(addr, CONFIG_VERSION);   addr += sizeof(CONFIG_VERSION);
  EEPROM.put(addr, useDhcp);          addr += sizeof(useDhcp);
  EEPROM.put(addr, ip);               addr += sizeof(ip);
  EEPROM.put(addr, subnet);           addr += sizeof(subnet);
  EEPROM.put(addr, gateway);          addr += sizeof(gateway);
  EEPROM.put(addr, dns1);             addr += sizeof(dns1);
  EEPROM.put(addr, mqtt_server);      addr += sizeof(mqtt_server);
  EEPROM.put(addr, mqtt_username);    addr += sizeof(mqtt_username);
  EEPROM.put(addr, mqtt_password);    addr += sizeof(mqtt_password);
  EEPROM.put(addr, mqtt_port);        addr += sizeof(mqtt_port);
  // Fill any remaining bytes before CRC with 0 for deterministic CRC (optional)
  for (int i = addr; i < CONFIG_CRC_L; i++) EEPROM.write(i, 0);
  uint16_t crc = computeConfigCRC(CONFIG_CRC_L);
  EEPROM.write(CONFIG_CRC_L, crc & 0xFF);
  EEPROM.write(CONFIG_CRC_H, (crc >> 8) & 0xFF);
  uint16_t verify = EEPROM.read(CONFIG_CRC_L) | (EEPROM.read(CONFIG_CRC_H) << 8);
  if (verify != crc) { Serial.println("CRC verify failed"); return false; }
  Serial.println("Cfg+CRC saved");
  return true;
}

// Helper function to print IP addresses
void printIP(uint8_t* ip) {
  Serial.print(ip[0]); Serial.print(".");
  Serial.print(ip[1]); Serial.print(".");
  Serial.print(ip[2]); Serial.print(".");
  Serial.println(ip[3]);
}

void handleWebClient(EthernetClient& client) {
  Serial.println("New client connected");
  unsigned long timeout = millis() + 5000;  // 5 second timeout
  String currentLine = "";
  bool currentLineIsBlank = true;
  bool isPost = false;
  String postData = "";
  
  // Clear any pending data
  while(client.available() && millis() < timeout) {
    char c = client.read();
    
    // Start of request line
    if (currentLineIsBlank && c != '\r' && c != '\n') {
      currentLine = c;
      currentLineIsBlank = false;
      continue;
    }
    
    // Build the current line
    if (c != '\r' && c != '\n') {
      currentLine += c;
    }
    
    // End of line
    if (c == '\n') {
      Serial.print("Received: ");
      Serial.println(currentLine);
      
      // Check for POST request
      if (currentLine.startsWith("POST ")) {
        isPost = true;
      }
      
      // Empty line marks the end of headers
      if (currentLine.length() == 0) {
        // If this is a POST request, read the POST data
        if (isPost) {
          while(client.available() && millis() < timeout) {
            postData += (char)client.read();
          }
          if (postData.length() > 0) {
            Serial.print("POST data: ");
            Serial.println(postData);
            handlePostData(client, postData);
          }
        }
        // Send the response
        Serial.println("Sending response headers...");
        client.println("HTTP/1.1 200 OK");
        client.println("Content-Type: text/html; charset=utf-8");
        client.println("Connection: close");
        client.println();  // Empty line after headers
        
        Serial.println("Sending web page...");
        sendWebPage(client);
        break;
      }
      
      currentLine = "";
      currentLineIsBlank = true;
    }
  }
  
  // Give the web browser time to receive the data
  delay(10);
  client.stop();
  Serial.println("Client disconnected");
}

void sendWebPage(EthernetClient& client) {
  // Unified minimal CSS + layout (compact to save flash)
  client.println(
    "<!DOCTYPE html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>DS18S20 MQTT Bridge</title><style>"
    "body{font-family:Arial,Helvetica,sans-serif;margin:10px;background:#f4f6f9;color:#222;font-size:14px}"
    "h1{margin:0 0 8px;font-size:22px}h2{margin:4px 0 6px;font-size:16px;border-bottom:1px solid #d0d7df;padding-bottom:2px}h3{margin:6px 0 4px;font-size:14px;color:#333}"
    ".badge{display:inline-block;padding:3px 8px;border-radius:4px;font-size:12px;margin:2px 6px 2px 0;background:#888;color:#fff}.ok{background:#2e7d32}.bad{background:#c62828}"
    ".card{background:#fff;border:1px solid #d0d7df;border-radius:6px;padding:10px;margin:0 0 12px;max-width:760px;box-shadow:0 1px 2px rgba(0,0,0,.04);font-size:12px}"
    "table{border-collapse:collapse;width:100%;font-size:12px}th,td{border:1px solid #ccc;padding:4px 6px;text-align:left}th{background:#e9eef3;font-weight:600}"
    "form{margin:0;font-size:12px}label{display:block;font-size:12px;margin:6px 0 4px}input[type=text],input[type=password]{width:100%;box-sizing:border-box;padding:4px 6px;font-size:12px}"
    ".rads label{display:inline-block;margin-right:14px} .btn{margin-top:10px;padding:6px 14px;background:#1976d2;color:#fff;border:0;border-radius:4px;cursor:pointer;font-size:12px} .btn:hover{background:#125ea8} .reset{background:#d32f2f}"
    ".small{font-size:12px;color:#444}"
    ".sens{width:auto;table-layout:fixed} .sens th,.sens td{padding:4px 6px;font-size:12px} .sens td:nth-child(2){font-family:monospace} .sens th.bus,.sens td.bus{width:3ch;text-align:center} .sens th.temp,.sens td.temp{width:15ch}"
    "</style></head><body><h1>DS18S20 MQTT Bridge</h1>");

  bool m = mqtt.connected();
  // Status card (uptime removed)
  client.print("<div class='card'><h2>Status</h2><div>");
  client.print(m?"<span class='badge ok'>MQTT Connected</span>":"<span class='badge bad'>MQTT Disconnected</span>");
  client.print(discoveryPublished?"<span class='badge ok'>Discovery OK</span>":"<span class='badge bad'>Discovery Pending</span>");
  client.print("<span class='badge ");
  if (Ethernet.linkStatus()==LinkOFF) client.print("bad'>Link Down"); else client.print("ok'>Link Up");
  client.print("</span></div>");
  client.print("<div class='small'><b>IP:</b> "); client.print(Ethernet.localIP()); client.print(" &nbsp; <b>MQTT:</b> "); client.print(mqtt_server);
  { // uptime: X d HH h MM min SS sec
    unsigned long sec = millis()/1000UL;
    unsigned long days = sec / 86400UL;
    unsigned long hrs  = (sec / 3600UL) % 24UL;
    unsigned long mins = (sec / 60UL) % 60UL;
    unsigned long secs = sec % 60UL;
    char bufU[40]; int p=0;
    // days
    unsigned long d=days; char dtmp[8]; int dp=7; dtmp[dp]=0; do { dtmp[--dp]='0'+(d%10); d/=10; } while(d && dp);
    while(dtmp[dp]) bufU[p++]=dtmp[dp++];
    bufU[p++]=' '; bufU[p++]='d'; bufU[p++]=' ';
    bufU[p++]='0'+(hrs/10); bufU[p++]='0'+(hrs%10); bufU[p++]=' '; bufU[p++]='h'; bufU[p++]=' ';
    bufU[p++]='0'+(mins/10); bufU[p++]='0'+(mins%10); bufU[p++]=' '; bufU[p++]='m'; bufU[p++]='i'; bufU[p++]='n'; bufU[p++]=' ';
    bufU[p++]='0'+(secs/10); bufU[p++]='0'+(secs%10); bufU[p++]=' '; bufU[p++]='s'; bufU[p++]='e'; bufU[p++]='c';
    bufU[p]=0;
    client.print(" &nbsp; <b>Uptime:</b> "); client.print(bufU);
  }
  client.println("</div></div>");

  // Sensors card
  client.println("<div class='card'><h2>Sensors</h2>");
  if(activeSensorCount==0){
    client.println("<p class='small'>No sensors detected.</p>");
  } else {
    client.println("<table class='sens'><tr><th>#</th><th>Serial</th><th class='bus'>Bus</th><th class='temp'>Temperature (°C)</th></tr>");
    for(int i=0;i<activeSensorCount;i++){
      SensorInfo& sn = connectedSensors[i];
      sensors[sn.bus]->requestTemperaturesByAddress(sn.address); delay(80);
      float t = sensors[sn.bus]->getTempC(sn.address);
      client.print("<tr><td>"); client.print(i+1); client.print("</td><td>"); client.print(sn.serialHex); client.print("</td><td class='bus'>"); client.print(sn.bus+1); client.print("</td><td class='temp'>");
      if(t!=DEVICE_DISCONNECTED_C && t>-55 && t<125){ char ts[8]; dtostrf(t,4,1,ts); client.print(ts);} else client.print("-");
      client.println("</td></tr>");
    }
    client.println("</table>");
  }
  client.println("</div>");

  // Configuration card
  client.print("<div class='card'><h2>Configuration</h2><form method='POST'>");
  // Network
  client.print("<h3>Network</h3><div class='rads'><label><input type='radio' name='ipMode' value='dhcp'"); if(useDhcp) client.print(" checked"); client.print(">DHCP</label><label><input type='radio' name='ipMode' value='static'"); if(!useDhcp) client.print(" checked"); client.print(">Static</label></div>");
  client.print("<label>IP<input type='text' name='staticIp' value='"); client.print(Ethernet.localIP()); client.print("'></label>");
  client.print("<label>Subnet<input type='text' name='staticSubnet' value='"); client.print(Ethernet.subnetMask()); client.print("'></label>");
  client.print("<label>Gateway<input type='text' name='staticGateway' value='"); client.print(Ethernet.gatewayIP()); client.print("'></label>");
  IPAddress dnsShow = useDhcp ? Ethernet.dnsServerIP() : dns1; if(dnsShow[0]==0) dnsShow=IPAddress(8,8,8,8);
  client.print("<label>DNS<input type='text' name='staticDns' value='"); client.print(dnsShow); client.print("'></label>");
  // MQTT
  client.print("<h3>MQTT</h3>");
  client.print("<label>Server hostname or IP address<input type='text' name='mqttServer' value='"); client.print(mqtt_server); client.print("'></label>");
  client.print("<label>Username<input type='text' name='mqttUsername' value='"); client.print(mqtt_username); client.print("'></label>");
  client.print("<label>Password<input type='password' name='mqttPassword' value='"); client.print(mqtt_password); client.print("'></label>");
  client.print("<input type='submit' class='btn' value='Save'></form>");
  client.print("<form method='POST' style='margin-top:6px'><input type='hidden' name='reset' value='1'><input type='submit' class='btn reset' value='Reboot Device'></form>");
  client.println("</div></body></html>");
}

// Helper function to extract values from POST data
String getValue(String data, String key) {
  int start = data.indexOf(key);
  if (start == -1) return "";
  start += key.length();
  int end = data.indexOf("&", start);
  if (end == -1) end = data.length();
  String value = data.substring(start, end);
  
  // URL decode
  value.replace("+", " ");
  String decodedValue = "";
  for (int i = 0; i < value.length(); i++) {
    if (value[i] == '%' && i + 2 < value.length()) {
      char hex[3] = { value[i+1], value[i+2], 0 };
      decodedValue += (char)strtol(hex, NULL, 16);
      i += 2;
    } else {
      decodedValue += value[i];
    }
  }
  return decodedValue;
}

void handlePostData(EthernetClient& client, String data) {
  // Early check for reset request
  if (data.indexOf("reset=1") != -1) {
    const char* resp = "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n"
                       "<html><body><h2>Resetting...</h2><p>Device will reboot.</p></body></html>";
    client.print(resp);
    client.stop();
    delay(500);
    rebootDevice();
    return;
  }
  
  Serial.println("\nProcessing config update...");
  bool configChanged = false;
  
  // Network Configuration
  bool newDhcp = (data.indexOf("ipMode=dhcp") != -1);
  if (newDhcp != useDhcp) {
    useDhcp = newDhcp;
    configChanged = true;
  }
  
  if (!useDhcp) {
    // Parse static IP settings
    String newIp = getValue(data, "staticIp=");
    String newSubnet = getValue(data, "staticSubnet=");
    String newGateway = getValue(data, "staticGateway=");
    String newDns = getValue(data, "staticDns=");
    
    IPAddress tempIp, tempSubnet, tempGateway, tempDns;
    
    if (newIp.length() > 0 && tempIp.fromString(newIp)) {
      ip = tempIp;
      configChanged = true;
    }
    if (newSubnet.length() > 0 && tempSubnet.fromString(newSubnet)) {
      subnet = tempSubnet;
      configChanged = true;
    }
    if (newGateway.length() > 0 && tempGateway.fromString(newGateway)) {
      gateway = tempGateway;
      configChanged = true;
    }
    if (newDns.length() > 0 && tempDns.fromString(newDns)) {
      dns1 = tempDns;
      configChanged = true;
    }
  }
  
  // MQTT Configuration
  String newServer = getValue(data, "mqttServer=");
  String newUsername = getValue(data, "mqttUsername=");
  String newPassword = getValue(data, "mqttPassword=");
  
  if (newServer.length() > 0 && strcmp(mqtt_server, newServer.c_str()) != 0) {
    strncpy(mqtt_server, newServer.c_str(), sizeof(mqtt_server)-1);
    configChanged = true;
  }
  
  if (newUsername.length() > 0 && strcmp(mqtt_username, newUsername.c_str()) != 0) {
    strncpy(mqtt_username, newUsername.c_str(), sizeof(mqtt_username)-1);
    configChanged = true;
  }
  
  if (newPassword.length() > 0 && strcmp(mqtt_password, newPassword.c_str()) != 0) {
    strncpy(mqtt_password, newPassword.c_str(), sizeof(mqtt_password)-1);
    configChanged = true;
  }
  
  if (configChanged) {
    Serial.println("Config changed, saving to EEPROM...");
    if (saveConfiguration()) {
      const char* response = "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n"
                             "<html><body><h2>Configuration saved!</h2>"
                             "<p>Device will reboot. Page will reload automatically in about 15 seconds...</p>"
                             "<script>setTimeout(function(){window.location.href='/'},15000);</script>"
                             "</body></html>";
      client.print(response);
      client.stop();
      Serial.println("Config saved success. Restarting in 1s...");
      delay(1000);
      rebootDevice();
    } else {
      const char* response = "HTTP/1.1 500 Internal Server Error\r\nContent-Type: text/html\r\n\r\n"
                             "<html><body><h2>Error saving configuration!</h2>"
                             "<p>Please try again.</p></body></html>";
      client.print(response);
      client.stop();
      Serial.println("Error saving config!");
    }
    return;
  }
  
  // Send no changes response
  const char* response = "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n"
                         "<html><body><h2>No configuration changes detected</h2>"
                         "<p>Returning to main page...</p>"
                         "<script>setTimeout(function(){window.location.href='/';},15000);</script>"
                         "</body></html>";
  client.print(response);
  Serial.println("No config changes detected");
  
  client.stop();
}

void printConfiguration() {
  Serial.println("\nCurrent Config:");
  Serial.print("DHCP Enabled: "); 
  Serial.println(useDhcp ? "Yes" : "No");
  
  Serial.print("IP: "); 
  Serial.println(ip.toString());
  
  Serial.print("Subnet: "); 
  Serial.println(subnet.toString());
  
  Serial.print("Gw: "); 
  Serial.println(gateway.toString());
  
  Serial.print("DNS: "); 
  Serial.println(dns1.toString());
  
  Serial.print("MQTT Srv: "); 
  Serial.println(mqtt_server);
  
  Serial.print("MQTT User: "); 
  Serial.println(mqtt_username);
  
  Serial.println("MQTT Pass: [hidden]");
}

bool verifyConfiguration() {
  // Check valid flag first
  if (EEPROM.read(EEPROM_VALID_FLAG_ADDR) != EEPROM_VALID_FLAG) {
    Serial.println("ERR: EEPROM valid flag not set!");
    return false;
  }
  
  // Verify version
  char version[3];
  for(byte i = 0; i < 2; i++) {
    version[i] = EEPROM.read(i);  // Read version from start of EEPROM
  }
  version[2] = '\0';
  
  if (String(version) != String(CONFIG_VERSION)) {
    Serial.println("ERR: Config ver mismatch!");
    Serial.print("Expected: "); Serial.print(CONFIG_VERSION);
    Serial.print(" Got: "); Serial.println(version);
    return false;
  }
  
  // Verify MQTT server is not empty
  bool mqtt_server_empty = true;
  for(int i = 0; i < sizeof(mqtt_server); i++) {
    if (EEPROM.read(EEPROM_MQTT_SERVER_START + i) != 0) {
      mqtt_server_empty = false;
      break;
    }
  }
  
  if (mqtt_server_empty) {
    Serial.println("ERR: MQTT server config empty!");
    return false;
  }
  
  // If using static IP, verify IP configurations
  if (EEPROM.read(EEPROM_DHCP_ENABLED) == 0) {
    // Check if static IP is valid (not 0.0.0.0)
    bool valid_ip = false;
    for(int i = 0; i < 4; i++) {
      if (EEPROM.read(EEPROM_IP_START + i) != 0) {
        valid_ip = true;
        break;
      }
    }
    
    if (!valid_ip) {
      Serial.println("ERR: Invalid static IP config!");
      return false;
    }
  }
  
  return true;
}

bool verifyConfigurationVersion() {
  char storedVersion[3];
  for(byte i = 0; i < 2; i++) {
    storedVersion[i] = EEPROM.read(i);
  }
  storedVersion[2] = '\0';
  
  if (String(storedVersion) != String(CONFIG_VERSION)) {
    Serial.println("Conf ver mismatch");
    return false;
  }
  return true;
}

bool resolveMqttServer(IPAddress& serverIP) {
  // Only parse literal IPs here; hostnames will be handled by client
  return serverIP.fromString(mqtt_server);
}

void reconnectMqtt() {
  if (mqtt.connected()) return;
  if (Ethernet.linkStatus() == LinkOFF) return;
  if (millis() < nextMqttAttempt) return;
  char clientId[32];
  uint16_t r = (uint16_t)random(0xFFFF);
  snprintf(clientId, sizeof(clientId), "STM32TS-%04X", r);
  IPAddress serverIP; if (resolveMqttServer(serverIP)) mqtt.setServer(serverIP, mqtt_port); else mqtt.setServer(mqtt_server, mqtt_port);
  char device_id[13]; snprintf(device_id, sizeof(device_id), "%02x%02x%02x%02x%02x%02x", mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
  char willTopic[64]; snprintf(willTopic, sizeof(willTopic), "homeassistant/bridge/%s/status", device_id);
  Serial.print("MQTT try "); Serial.print(mqttBackoff/1000); Serial.println("s");
  if (mqtt.connect(clientId, mqtt_username, mqtt_password, willTopic, 1, true, "offline")) {
    Serial.println("MQTT OK");
    mqtt.publish(willTopic, "online", true);
    discoveryPublished = false; lastDiscoveryAttempt = 0; mqttBackoff = 5000;
#if ENABLE_UPTIME
    lastUptimeSec = 0; // force first minute publish
#endif
    if (activeSensorCount > 0) { Serial.println("Disc now"); publishDiscoveryMessages(); }
  } else {
    Serial.print("MQTT ERR "); Serial.println(mqtt.state());
    mqttBackoff = (mqttBackoff < MQTT_BACKOFF_MAX) ? mqttBackoff * 2 : MQTT_BACKOFF_MAX;
  }
  nextMqttAttempt = millis() + mqttBackoff;
}

void publishDiscoveryMessages() {
  if (discoveryPublished) return;
  char bridge_id[13];
  snprintf(bridge_id, sizeof(bridge_id), "%02x%02x%02x%02x%02x%02x", mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
  IPAddress lip = Ethernet.localIP();
  char configUrl[64]; snprintf(configUrl, sizeof(configUrl), "http://%d.%d.%d.%d", lip[0],lip[1],lip[2],lip[3]);
  char buf[1024];
  bool ok = true;
  // Bridge status binary sensor
  {
    char cfgTopic[96]; char stateTopic[96];
    snprintf(cfgTopic, sizeof(cfgTopic),  "homeassistant/binary_sensor/%s/status/config", bridge_id);
    snprintf(stateTopic, sizeof(stateTopic),"homeassistant/bridge/%s/status", bridge_id);
    snprintf(buf, sizeof(buf),
      "{\"name\":\"Bridge Status\",\"unique_id\":\"%s_status\",\"device_class\":\"connectivity\",\"state_topic\":\"%s\",\"payload_on\":\"online\",\"payload_off\":\"offline\",\"device\":{\"identifiers\":[\"%s\"],\"name\":\"STM32 DS18S20 Bridge\",\"manufacturer\":\"Custom\",\"model\":\"Custom\",\"configuration_url\":\"%s\"}}",
      bridge_id, stateTopic, bridge_id, configUrl);
    if (!mqtt.publish(cfgTopic, buf, true)) { Serial.println("Disc fail bs cfg"); ok = false; }
    if (!mqtt.publish(stateTopic, "online", true)) { Serial.println("Disc fail bs st"); ok = false; }
  }
  // Bridge URL diagnostic sensor
  {
    char cfgTopic[96]; char stateTopic[96];
    snprintf(cfgTopic, sizeof(cfgTopic),  "homeassistant/sensor/%s/url/config", bridge_id);
    snprintf(stateTopic, sizeof(stateTopic),"homeassistant/bridge/%s/url", bridge_id);
    snprintf(buf, sizeof(buf),
      "{\"name\":\"Bridge URL\",\"unique_id\":\"%s_url\",\"icon\":\"mdi:link-variant\",\"state_topic\":\"%s\",\"entity_category\":\"diagnostic\",\"device\":{\"identifiers\":[\"%s\"],\"name\":\"STM32 DS18S20 Bridge\",\"manufacturer\":\"Custom\",\"model\":\"Custom\",\"configuration_url\":\"%s\"}}",
      bridge_id, stateTopic, bridge_id, configUrl);
    if (!mqtt.publish(cfgTopic, buf, true)) { Serial.println("Disc fail url cfg"); ok = false; }
    if (!mqtt.publish(stateTopic, configUrl, true)) { Serial.println("Disc fail url st"); ok = false; }
  }
#if ENABLE_UPTIME
  // Bridge uptime sensor (simplified JSON)
  {
    char cfgTopic[96]; char stateTopic[96];
    snprintf(cfgTopic, sizeof(cfgTopic),  "homeassistant/sensor/%s/uptime/config", bridge_id);
    snprintf(stateTopic, sizeof(stateTopic),"homeassistant/bridge/%s/uptime", bridge_id);
    snprintf(buf, sizeof(buf),
      "{\"name\":\"Bridge Uptime\",\"unique_id\":\"%s_uptime\",\"unit_of_measurement\":\"s\",\"state_topic\":\"%s\",\"entity_category\":\"diagnostic\",\"device\":{\"identifiers\":[\"%s\"],\"name\":\"STM32 DS18S20 Bridge\",\"manufacturer\":\"Custom\",\"model\":\"Custom\",\"configuration_url\":\"%s\"}}",
      bridge_id, stateTopic, bridge_id, configUrl);
    if (!mqtt.publish(cfgTopic, buf, true)) { Serial.println("Disc fail up cfg"); ok = false; }
    // Initial uptime value (seconds)
    unsigned long sec = millis()/1000UL; lastUptimeSec = sec;
    char up[12]; int p=11; up[p]=0; unsigned long v=sec; do { up[--p]='0'+(v%10); v/=10; } while(v && p);
    mqtt.publish(stateTopic, &up[p], true);
  }
#endif
  // Per-sensor temperature entities (corrected config topic path)
  for (int i = 0; i < activeSensorCount; i++) {
    SensorInfo &si = connectedSensors[i];
    char sensor_node_id[32]; snprintf(sensor_node_id, sizeof(sensor_node_id), "ds18s20_%s", si.serialHex);
    char cfgTopic[128]; char stateTopic[128]; char attrTopic[128];
    snprintf(cfgTopic, sizeof(cfgTopic),  "homeassistant/sensor/%s/config", sensor_node_id);
    snprintf(stateTopic, sizeof(stateTopic),"homeassistant/sensor/%s/temperature/state", sensor_node_id);
    snprintf(attrTopic, sizeof(attrTopic), "homeassistant/sensor/%s/temperature/attrs", sensor_node_id);
    // Shared availability topic (bridge LWT) so sensors go unavailable (not removed) when device offline
    char bridgeAvailTopic[96]; snprintf(bridgeAvailTopic, sizeof(bridgeAvailTopic), "homeassistant/bridge/%s/status", bridge_id);
    snprintf(buf, sizeof(buf),
      "{\"name\":\"Temperature\",\"unique_id\":\"%s_temp\",\"unit_of_measurement\":\"°C\",\"device_class\":\"temperature\",\"state_class\":\"measurement\",\"state_topic\":\"%s\",\"availability_topic\":\"%s\",\"json_attributes_topic\":\"%s\",\"payload_on\":\"online\",\"payload_off\":\"offline\",\"device\":{\"identifiers\":[\"%s\"],\"name\":\"DS18S20 %s\",\"manufacturer\":\"Dallas\",\"model\":\"DS18S20\",\"via_device\":\"%s\"}}",
      sensor_node_id, stateTopic, bridgeAvailTopic, attrTopic, si.serialHex, si.serialHex, bridge_id);
    if (!mqtt.publish(cfgTopic, buf, true)) { Serial.print("Disc fail cfg "); Serial.println(i); ok = false; }
    char attrJson[20]; snprintf(attrJson, sizeof(attrJson), "{\"bus\":%d}", si.bus+1);
    mqtt.publish(attrTopic, attrJson, true);
    mqtt.loop();
  }
  if (ok) { discoveryPublished = true; Serial.println("Disc OK"); }
  else Serial.println("Disc retry");
}

void startTemperatureRequests() {
  if (activeSensorCount == 0) return;
  // Request conversions per bus (one request covers all devices on bus)
  for (int b=0; b<ONE_WIRE_BUS_COUNT; b++) {
    sensors[b]->setWaitForConversion(false);
    sensors[b]->requestTemperatures();
  }
  tempRequestTime = millis();
  tempRequestPending = true;
}

void completeTemperatureCycle() {
  if (!tempRequestPending) return;
  if (millis() - tempRequestTime < TEMP_CONVERSION_MS) return; // not ready
  char bridge_device_id[13];
  snprintf(bridge_device_id, sizeof(bridge_device_id), "%02x%02x%02x%02x%02x%02x", mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
  char bridgeStatusTopic[64];
  snprintf(bridgeStatusTopic, sizeof(bridgeStatusTopic), "homeassistant/bridge/%s/status", bridge_device_id);
  mqtt.publish(bridgeStatusTopic, "online", true);
  for (int i=0;i<activeSensorCount;i++) {
    SensorInfo &sensor = connectedSensors[i];
    float tempC = sensors[sensor.bus]->getTempC(sensor.address);
    if(tempC != DEVICE_DISCONNECTED_C && tempC > -55 && tempC < 125) {
      char sensor_node_id[32]; snprintf(sensor_node_id, sizeof(sensor_node_id), "ds18s20_%s", sensor.serialHex);
      char temp_state_topic[96]; snprintf(temp_state_topic, sizeof(temp_state_topic), "homeassistant/sensor/%s/temperature/state", sensor_node_id);
      char tempStr[8]; dtostrf(tempC, 4, 1, tempStr);
      mqtt.publish(temp_state_topic, tempStr, false);
    }
    // If sensor fails reading we skip publish; entity remains last value or becomes unavailable only when whole bridge offline.
  }
  tempRequestPending = false;
  Serial.println("Temp cycle");
}

void checkFactoryResetButton() {
  if (digitalRead(CONFIG_RESET_PIN) == LOW) {
    Serial.println("Hold PB12 10s");
    unsigned long start = millis();
    unsigned long lastMsg = 0;
    while (digitalRead(CONFIG_RESET_PIN) == LOW) {
      unsigned long elapsed = millis() - start;
      if (elapsed - lastMsg >= 1000) {
        lastMsg = elapsed;
        Serial.print("  Holding "); Serial.print(elapsed/1000); Serial.print("s / 10s\r\n");
      }
      if (elapsed >= 10000) {
        Serial.println("Factory reset");
        initializeDefaultConfig();
        if (saveConfiguration()) {
          Serial.println("Defaults saved");
        } else {
          Serial.println("Default save ERR");
        }
        delay(400);
        rebootDevice();
      }
      delay(50);
    }
    Serial.println("Reset abort");
  }
}
