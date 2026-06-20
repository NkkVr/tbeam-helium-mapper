/*
  WiFi + BLE configuration portal for the Helium Mapper.
  See config_portal.h for an overview and the rationale for keeping WiFi and
  BLE as separate, never-simultaneous modes.
*/

#include "config_portal.h"

// When the portal feature is disabled this whole translation unit compiles to
// nothing, so the default build pulls in no WiFi/BLE code and stays identical
// in size/behavior to the upstream base.
#ifdef ENABLE_CONFIG_PORTAL

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WiFi.h>
#include <axp20x.h>
#include <esp_system.h>  // esp_read_mac / ESP_MAC_WIFI_STA (works without WiFi init)
#ifdef USE_RADIOLIB
#include "lmic_compat.h"  // dr_t / DR_SF* without LMIC
#else
#include <lmic.h>
#endif

#include "configuration.h"
#include "gps.h"
#include "screen.h"
#include "ttn.h"

// -----------------------------------------------------------------------------
// Settings globals (defined in main.cpp) that the portal can read/modify
// -----------------------------------------------------------------------------
extern float min_dist_moved;
extern unsigned int stationary_tx_interval_s;
extern unsigned int rest_wait_s;
extern unsigned int rest_tx_interval_s;
extern dr_t lorawan_sf;
extern double deadzone_lat;
extern double deadzone_lon;
extern double deadzone_radius_m;
extern char sf_name[40];

extern AXP20X_Class axp;
extern bool axp192_found;

extern void mapper_save_prefs(void);

// -----------------------------------------------------------------------------
// Portal configuration
// -----------------------------------------------------------------------------
#define CONFIG_PORTAL_TIMEOUT_MS (10 * 60 * 1000UL)  // Auto-exit after 10 idle minutes
#define CONFIG_EXIT_HOLD_MS 2000                     // Hold the middle button this long to exit
#define CONFIG_PREFS_NS "portal"                     // NVS namespace for portal settings (AP password)

// BLE UUIDs (custom 128-bit).  One service, one characteristic per setting.
#define SVC_UUID "9e6a0001-b5a3-f393-e0a9-e50e24dcca9e"
#define CH_MIN_DIST "9e6a0002-b5a3-f393-e0a9-e50e24dcca9e"
#define CH_TX_INTERVAL "9e6a0003-b5a3-f393-e0a9-e50e24dcca9e"
#define CH_REST_WAIT "9e6a0004-b5a3-f393-e0a9-e50e24dcca9e"
#define CH_REST_TX "9e6a0005-b5a3-f393-e0a9-e50e24dcca9e"
#define CH_SF "9e6a0006-b5a3-f393-e0a9-e50e24dcca9e"
#define CH_DZ_LAT "9e6a0007-b5a3-f393-e0a9-e50e24dcca9e"
#define CH_DZ_LON "9e6a0008-b5a3-f393-e0a9-e50e24dcca9e"
#define CH_DZ_RADIUS "9e6a0009-b5a3-f393-e0a9-e50e24dcca9e"

static WebServer server(80);
static bool portal_should_exit = false;

// Device name used for both the WiFi SSID and the BLE advertised name.
// Read the MAC straight from efuse so this works regardless of whether WiFi
// has been started yet (WiFi.macAddress() needs WiFi init; esp_read_mac does not).
static String device_name() {
  uint8_t mac[6] = {0};
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  char suffix[5];
  snprintf(suffix, sizeof(suffix), "%02X%02X", mac[4], mac[5]);
  return String("HeliumMapper-") + suffix;
}

// Active LoRaWAN region.  With the RadioLib backend it is selected at runtime
// from GPS; with LMIC it is fixed at build time.
static const char *region_name() {
#ifdef USE_RADIOLIB
  return ttn_region_name();
#elif defined(CFG_eu868)
  return "EU868";
#elif defined(CFG_us915)
  return "US915";
#elif defined(CFG_au915)
  return "AU915";
#elif defined(CFG_eu433)
  return "EU433";
#elif defined(CFG_as923)
  return "AS923";
#else
  return "Unknown";
#endif
}

// Coarse LoRaWAN region expected at the current GPS location.  This is only an
// approximate continental mapping used to WARN about a region mismatch -- MCCI
// LMIC cannot actually switch region at runtime, so the firmware must be built
// for the matching region.  Returns "" when there is no valid fix.
static const char *gps_expected_region() {
  if (!tGPS.location.isValid())
    return "";
  double lat = tGPS.location.lat();
  double lon = tGPS.location.lng();
  if (lon >= -170 && lon < -30)
    return "US915";  // Americas
  if (lon >= 110 && lat < -10)
    return "AU915";  // Australia / New Zealand
  if (lon >= -30 && lon < 65)
    return "EU868";  // Europe / Africa / Middle East
  if (lon >= 65 && lon < 150)
    return "AS923";  // most of Asia
  return "";         // unsure
}

// Log a warning if the GPS location does not match the compiled region.
static void warn_region_mismatch() {
  const char *det = gps_expected_region();
  if (det[0] && strcmp(det, region_name()) != 0)
    Serial.printf("[Config] WARNING: GPS suggests %s but this firmware is built for %s\n", det, region_name());
}

// WiFi AP password: saved value from NVS, or a device-specific default that is
// stronger than a shared constant (so each unit ships with a unique password).
static String get_ap_password() {
  String pw;
  Preferences p;
  if (p.begin(CONFIG_PREFS_NS, true)) {
    pw = p.getString("appw", "");
    p.end();
  }
  if (pw.length() < 8) {
    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    char buf[20];
    snprintf(buf, sizeof(buf), "mapper-%02X%02X%02X", mac[3], mac[4], mac[5]);
    pw = buf;
  }
  return pw;
}

// Format n bytes as upper-case hex (no separators).
static String bytes_to_hex(const uint8_t *b, size_t n) {
  String s;
  char t[3];
  for (size_t i = 0; i < n; i++) {
    snprintf(t, sizeof(t), "%02X", b[i]);
    s += t;
  }
  return s;
}

// Parse a hex string (ignoring any non-hex characters such as spaces/commas/0x)
// into exactly n bytes.  Returns true only on an exact, complete match.
static bool parse_hex(const String &in, uint8_t *out, size_t n) {
  size_t cnt = 0;
  int hi = -1;
  for (size_t i = 0; i < in.length() && cnt < n; i++) {
    char c = in[i];
    int v;
    if (c >= '0' && c <= '9')
      v = c - '0';
    else if (c >= 'a' && c <= 'f')
      v = c - 'a' + 10;
    else if (c >= 'A' && c <= 'F')
      v = c - 'A' + 10;
    else
      continue;
    if (hi < 0)
      hi = v;
    else {
      out[cnt++] = (uint8_t)((hi << 4) | v);
      hi = -1;
    }
  }
  return (cnt == n && hi < 0);
}

// -----------------------------------------------------------------------------
// Spreading-Factor helpers (settings store dr_t; users think in SF7..SF10)
// -----------------------------------------------------------------------------
static int sf_to_num(dr_t sf) {
  if (sf == DR_SF8)
    return 8;
  if (sf == DR_SF9)
    return 9;
  if (sf == DR_SF10)
    return 10;
  return 7;  // DR_SF7 / default
}

static dr_t num_to_sf(int n) {
  switch (n) {
    case 8:
      return DR_SF8;
    case 9:
      return DR_SF9;
    case 10:
      return DR_SF10;
    default:
      return DR_SF7;
  }
}

// -----------------------------------------------------------------------------
// Generic get/apply for one setting, shared by the web UI and BLE
// -----------------------------------------------------------------------------
static String get_setting(const String &key) {
  if (key == "min_dist")
    return String(min_dist_moved, 0);
  if (key == "tx_interval")
    return String(stationary_tx_interval_s);
  if (key == "rest_wait")
    return String(rest_wait_s);
  if (key == "rest_tx")
    return String(rest_tx_interval_s);
  if (key == "sf")
    return String(sf_to_num(lorawan_sf));
  if (key == "dz_lat")
    return String(deadzone_lat, 6);
  if (key == "dz_lon")
    return String(deadzone_lon, 6);
  if (key == "dz_radius")
    return String(deadzone_radius_m, 0);
  return String("");
}

// Returns true if the key was recognized and applied.
static bool apply_setting(const String &key, const String &val) {
  bool changed = true;

  if (key == "min_dist") {
    float v = val.toFloat();
    if (v < 10)
      v = 10;
    min_dist_moved = v;
  } else if (key == "tx_interval") {
    long v = val.toInt();
    if (v < 10)
      v = 10;
    stationary_tx_interval_s = (unsigned int)v;
  } else if (key == "rest_wait") {
    long v = val.toInt();
    if (v < 0)
      v = 0;
    rest_wait_s = (unsigned int)v;
  } else if (key == "rest_tx") {
    long v = val.toInt();
    if (v < 10)
      v = 10;
    rest_tx_interval_s = (unsigned int)v;
  } else if (key == "sf") {
    lorawan_sf = num_to_sf(val.toInt());
    ttn_set_sf(lorawan_sf);
    ttn_get_sf_name(sf_name, sizeof(sf_name));
  } else if (key == "dz_lat") {
    deadzone_lat = val.toDouble();
  } else if (key == "dz_lon") {
    deadzone_lon = val.toDouble();
  } else if (key == "dz_radius") {
    double v = val.toDouble();
    if (v < 0)
      v = 0;
    deadzone_radius_m = v;
  } else {
    changed = false;
  }

  if (changed)
    Serial.printf("[Config] %s = %s\n", key.c_str(), val.c_str());

  return changed;
}

// -----------------------------------------------------------------------------
// Shared exit loop: poll until button-hold / timeout (and web, if requested)
// -----------------------------------------------------------------------------
static void portal_loop(bool service_web) {
  portal_should_exit = false;
  uint32_t start = millis();
  uint32_t press_start = 0;

  while (!portal_should_exit) {
    if (service_web)
      server.handleClient();

    if (!digitalRead(MIDDLE_BUTTON_PIN)) {  // active low
      if (!press_start)
        press_start = millis();
      else if (millis() - press_start > CONFIG_EXIT_HOLD_MS)
        portal_should_exit = true;
    } else {
      press_start = 0;
    }

    if (millis() - start > CONFIG_PORTAL_TIMEOUT_MS)
      portal_should_exit = true;

    delay(10);
  }
}

// -----------------------------------------------------------------------------
// Web UI (WiFi mode)
// -----------------------------------------------------------------------------
static String field(const char *key, const char *label, const char *hint) {
  String s = "<p><label>";
  s += label;
  s += "</label><br><small>";
  s += hint;
  s += "</small><br><input id='";
  s += key;
  s += "' name='";
  s += key;
  s += "' value='";
  s += get_setting(key);
  s += "'></p>";
  return s;
}

static void handle_root() {
  String h;
  h.reserve(3200);
  h += "<!DOCTYPE html><html><head><meta charset='utf-8'>";
  h += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  h += "<title>Helium Mapper Setup</title><style>";
  h += "body{font-family:sans-serif;margin:16px;max-width:480px}";
  h += "input{width:100%;padding:8px;font-size:16px;box-sizing:border-box}";
  h += "label{font-weight:bold}small{color:#666;font-weight:normal}";
  h += "button{padding:12px;font-size:16px;width:100%;margin-top:8px;border:0;border-radius:6px}";
  h += ".save{background:#2a7;color:#fff}.loc{background:#06c;color:#fff}.exit{background:#c00;color:#fff}";
  h += "</style></head><body><h2>Helium Mapper</h2>";
  h += "<p>Region: <b>";
  h += region_name();
  h += "</b> (set at build time) &middot; SF: <b>";
  h += sf_name;
  h += "</b></p>";
  // GPS-based region check: warn (cannot switch automatically) if the firmware's
  // compiled region does not match where the device currently is.
  const char *det = gps_expected_region();
  if (det[0]) {
    bool match = (strcmp(det, region_name()) == 0);
    h += "<p style='padding:8px;border-radius:6px;background:";
    h += match ? "#e7f7ec" : "#fdecea";
    h += "'>GPS location suggests <b>";
    h += det;
    h += "</b> &mdash; ";
    if (match)
      h += "matches this firmware.";
    else {
      h += "this firmware is <b>";
      h += region_name();
      h += "</b>. Flash the matching region build to map here.";
    }
    h += "</p>";
  }
#ifdef USE_RADIOLIB
  // Fallback region used to join when there is no usable GPS fix in time.
  {
    static const lora_region_t opts[] = {REGION_EU868, REGION_US915, REGION_AU915,
                                         REGION_AS923, REGION_IN865, REGION_KR920};
    lora_region_t cur = ttn_get_fallback_region();
    h += "<form method='post' action='/savefallback' style='margin:8px 0'>";
    h += "<label>Fallback region</label><br><small>Used to join if GPS gives no usable fix in time.</small><br>";
    h += "<select name='fb'>";
    for (auto o : opts) {
      h += "<option value='";
      h += String((int)o);
      h += "'";
      if (o == cur)
        h += " selected";
      h += ">";
      h += region_to_string(o);
      h += "</option>";
    }
    h += "</select><button type='submit' class='save'>Save Fallback Region</button></form>";
  }
#endif

  h += "<form method='post' action='/save'>";
  h += field("min_dist", "Min Distance (m)", "Meters moved before an uplink (min 10)");
  h += field("tx_interval", "Stationary TX (s)", "Heartbeat interval while still (min 10)");
  h += field("rest_wait", "Rest Wait (s)", "Idle seconds before slowing down");
  h += field("rest_tx", "Rest TX (s)", "Slow resting ping interval (min 10)");
  h += field("sf", "Spreading Factor", "7, 8, 9 or 10 (7 is best for mapping)");
  h += field("dz_lat", "Deadzone Lat", "Center latitude (no uplinks inside)");
  h += field("dz_lon", "Deadzone Lon", "Center longitude");
  h += field("dz_radius", "Deadzone Radius (m)", "0 disables the deadzone");
  h += "<button type='button' class='loc' onclick='useLoc()'>&#128241; Use my phone's location</button>";
  h += "<span id='locmsg'></span>";
  h += "<button type='submit' class='save'>Save</button></form>";
  h += "<form method='get' action='/exit'><button class='exit' type='submit'>Save &amp; Exit Config</button></form>";
  // Geolocation: fill the deadzone center from the phone's GPS.
  h += "<script>function useLoc(){var m=document.getElementById('locmsg');";
  h += "if(!navigator.geolocation){m.textContent=' (not supported)';return;}";
  h += "m.textContent=' locating...';";
  h += "navigator.geolocation.getCurrentPosition(function(p){";
  h += "document.getElementById('dz_lat').value=p.coords.latitude.toFixed(6);";
  h += "document.getElementById('dz_lon').value=p.coords.longitude.toFixed(6);";
  h += "if(parseFloat(document.getElementById('dz_radius').value)<=0)";
  h += "document.getElementById('dz_radius').value='";
  h += String((int)DEADZONE_RADIUS_M);
  h += "';m.textContent=' set! press Save';},";
  h += "function(e){m.textContent=' error: '+e.message;},{enableHighAccuracy:true,timeout:10000});}</script>";

  // --- LoRaWAN credentials (DevEUI / AppEUI / AppKey) ---
  uint8_t de[8], ae[8], ak[16];
  ttn_get_credentials(de, ae, ak);
  uint8_t de_msb[8], ae_msb[8];
  for (int i = 0; i < 8; i++) {
    de_msb[i] = de[7 - i];
    ae_msb[i] = ae[7 - i];
  }
  h += "<hr><h3>LoRaWAN Keys</h3>";
  h += "<p><small>Enter values exactly as shown in the Helium Console (MSB hex). "
       "Region is fixed to <b>";
  h += region_name();
  h += "</b> by this firmware build. Saving keys reboots the device to apply.</small></p>";
  h += "<form method='post' action='/savekeys'>";
  h += "<p><label>DevEUI (MSB)</label><br><input name='deveui' value='";
  h += bytes_to_hex(de_msb, 8);
  h += "'></p>";
  h += "<p><label>AppEUI (MSB)</label><br><input name='appeui' value='";
  h += bytes_to_hex(ae_msb, 8);
  h += "'></p>";
  h += "<p><label>AppKey (MSB)</label><br><small>Leave blank to keep the current key</small>"
       "<br><input name='appkey' value='' placeholder='32 hex chars'></p>";
  h += "<button type='submit' class='save'>Save Keys &amp; Reboot</button></form>";

  // --- WiFi AP password ---
  h += "<hr><h3>WiFi AP Password</h3>";
  h += "<p><small>Min 8 characters. Applies the next time you enter WiFi config.</small></p>";
  h += "<form method='post' action='/savepw'>";
  h += "<input name='appw' value='' placeholder='new AP password'>";
  h += "<button type='submit' class='save'>Change AP Password</button></form>";

  h += "</body></html>";
  server.send(200, "text/html", h);
}

static void handle_save() {
  for (int i = 0; i < server.args(); i++) apply_setting(server.argName(i), server.arg(i));
  mapper_save_prefs();
  server.sendHeader("Location", "/");
  server.send(303, "text/plain", "Saved");
}

#ifdef USE_RADIOLIB
static void handle_savefallback() {
  ttn_set_fallback_region((lora_region_t)server.arg("fb").toInt());
  server.sendHeader("Location", "/");
  server.send(303, "text/plain", "Saved");
}
#endif

static void handle_exit() {
  server.send(200, "text/html",
              "<!DOCTYPE html><html><body style='font-family:sans-serif'>"
              "<h2>Saved.</h2><p>Config mode is closing. You may disconnect.</p></body></html>");
  portal_should_exit = true;
}

static void handle_savekeys() {
  uint8_t de_msb[8], ae_msb[8], ak_msb[16];
  String d = server.arg("deveui"), a = server.arg("appeui"), k = server.arg("appkey");

  if (!parse_hex(d, de_msb, 8) || !parse_hex(a, ae_msb, 8)) {
    server.send(400, "text/html",
                "<h2>DevEUI and AppEUI must each be 8 bytes (16 hex chars).</h2><a href='/'>Back</a>");
    return;
  }

  // Helium Console shows EUIs MSB-first; LMIC stores them LSB-first.
  uint8_t deveui[8], appeui[8], appkey[16];
  for (int i = 0; i < 8; i++) {
    deveui[i] = de_msb[7 - i];
    appeui[i] = ae_msb[7 - i];
  }

  k.trim();
  if (k.length() == 0) {
    // Blank AppKey -> keep the current one.
    uint8_t cur_de[8], cur_ae[8];
    ttn_get_credentials(cur_de, cur_ae, appkey);
  } else if (parse_hex(k, ak_msb, 16)) {
    memcpy(appkey, ak_msb, 16);  // AppKey is used MSB-first
  } else {
    server.send(400, "text/html",
                "<h2>AppKey must be 16 bytes (32 hex chars), or blank to keep current.</h2><a href='/'>Back</a>");
    return;
  }

  ttn_set_credentials(deveui, appeui, appkey);
  server.send(200, "text/html",
              "<!DOCTYPE html><html><body style='font-family:sans-serif'>"
              "<h2>Keys saved.</h2><p>Rebooting to apply the new credentials...</p></body></html>");
  delay(1500);
  ESP.restart();
}

static void handle_savepw() {
  String pw = server.arg("appw");
  if (pw.length() < 8) {
    server.send(400, "text/html", "<h2>Password must be at least 8 characters.</h2><a href='/'>Back</a>");
    return;
  }
  Preferences p;
  if (p.begin(CONFIG_PREFS_NS, false)) {
    p.putString("appw", pw);
    p.end();
  }
  server.send(200, "text/html",
              "<!DOCTYPE html><html><body style='font-family:sans-serif'>"
              "<h2>AP password changed.</h2><p>It applies the next time you enter WiFi config.</p>"
              "<a href='/'>Back</a></body></html>");
}

// -----------------------------------------------------------------------------
// BLE (BLE mode)
// -----------------------------------------------------------------------------
class SettingCallback : public BLECharacteristicCallbacks {
  String key;

 public:
  explicit SettingCallback(const String &k) : key(k) {}
  void onWrite(BLECharacteristic *c) override {
    String v(c->getValue().c_str());
    apply_setting(key, v);
    mapper_save_prefs();
    // Reflect the normalized/clamped value back to the reader.
    String cur = get_setting(key);
    c->setValue((uint8_t *)cur.c_str(), cur.length());
  }
};

static void add_char(BLEService *svc, const char *uuid, const char *key, const char *desc) {
  BLECharacteristic *c =
      svc->createCharacteristic(uuid, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  String cur = get_setting(key);
  c->setValue((uint8_t *)cur.c_str(), cur.length());
  c->setCallbacks(new SettingCallback(key));

  // 0x2901 = Characteristic User Description, so generic BLE apps show a label.
  BLEDescriptor *d = new BLEDescriptor(BLEUUID((uint16_t)0x2901));
  d->setValue(desc);
  c->addDescriptor(d);
}

static void ble_setup(const String &name) {
  BLEDevice::init(name.c_str());
  BLEServer *srv = BLEDevice::createServer();
  BLEService *svc = srv->createService(BLEUUID(SVC_UUID), 40 /* numHandles */);

  add_char(svc, CH_MIN_DIST, "min_dist", "Min Distance m");
  add_char(svc, CH_TX_INTERVAL, "tx_interval", "Stationary TX s");
  add_char(svc, CH_REST_WAIT, "rest_wait", "Rest Wait s");
  add_char(svc, CH_REST_TX, "rest_tx", "Rest TX s");
  add_char(svc, CH_SF, "sf", "Spreading Factor 7-10");
  add_char(svc, CH_DZ_LAT, "dz_lat", "Deadzone Lat");
  add_char(svc, CH_DZ_LON, "dz_lon", "Deadzone Lon");
  add_char(svc, CH_DZ_RADIUS, "dz_radius", "Deadzone Radius m");

  svc->start();

  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(BLEUUID(SVC_UUID));
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();
}

// -----------------------------------------------------------------------------
// Power management while in config mode
//
// Mapping is paused during config, so power down the GPS (the largest current
// consumer) just like low_power_sleep() does.  This frees current budget for
// the WiFi/BLE radio (helps avoid brown-outs) and saves battery.
// -----------------------------------------------------------------------------
static void config_power_down(void) {
  if (axp192_found) {
    axp.setPowerOutPut(AXP192_LDO3, AXP202_OFF);  // GPS power off
    axp.setChgLEDMode(AXP20X_LED_OFF);
  }
}

static void config_power_restore(void) {
  if (axp192_found)
    axp.setPowerOutPut(AXP192_LDO3, AXP202_ON);  // GPS power back on
  delay(100);                                    // GPS needs a moment after power-on
  gps_setup(false);                              // Re-sync with the GPS after the power cycle
}

// -----------------------------------------------------------------------------
// Entry points -- WiFi and BLE are mutually exclusive (no coexistence)
// -----------------------------------------------------------------------------
void config_portal_run_wifi(void) {
  btStop();  // Make sure BLE/BT is off: never run AP + BLE together.
  config_power_down();
  warn_region_mismatch();

  String ssid = device_name();
  String pass = get_ap_password();
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid.c_str(), pass.c_str());
  WiFi.setTxPower(WIFI_POWER_11dBm);  // Phone is close by; lower TX power = less current
  IPAddress ip = WiFi.softAPIP();

  server.on("/", handle_root);
  server.on("/save", HTTP_POST, handle_save);
  server.on("/savekeys", HTTP_POST, handle_savekeys);
  server.on("/savepw", HTTP_POST, handle_savepw);
#ifdef USE_RADIOLIB
  server.on("/savefallback", HTTP_POST, handle_savefallback);
#endif
  server.on("/exit", handle_exit);
  server.begin();

  char buf[200];
  snprintf(buf, sizeof(buf), "\n== WiFi CONFIG ==\nSSID: %s\nPass: %s\nhttp://%s\nHold btn to exit\n", ssid.c_str(),
           pass.c_str(), ip.toString().c_str());
  screen_print(buf);
  screen_update();
  Serial.print(buf);

  portal_loop(true /* service web */);

  mapper_save_prefs();
  screen_print("\nExiting WiFi...\n");
  screen_update();

  server.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_MODE_NULL);
  config_power_restore();
  delay(200);
  Serial.println("[Config] WiFi portal closed, resuming mapper.");
}

void config_portal_run_ble(void) {
  WiFi.mode(WIFI_MODE_NULL);  // Make sure WiFi is off: never run AP + BLE together.
  config_power_down();
  warn_region_mismatch();

  String name = device_name();
  ble_setup(name);

  char buf[160];
  snprintf(buf, sizeof(buf), "\n== BLE CONFIG ==\nName: %s\nUse nRF Connect\nHold btn to exit\n", name.c_str());
  screen_print(buf);
  screen_update();
  Serial.print(buf);

  portal_loop(false /* no web */);

  mapper_save_prefs();
  screen_print("\nExiting BLE...\n");
  screen_update();

  BLEDevice::deinit(true);
  btStop();
  config_power_restore();
  delay(200);
  Serial.println("[Config] BLE portal closed, resuming mapper.");
}

#endif  // ENABLE_CONFIG_PORTAL
