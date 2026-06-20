/*
  RadioLib LoRaWAN backend (USE_RADIOLIB).

  Drop-in replacement for ttn.cpp that uses RadioLib instead of MCCI LMIC, so
  the LoRaWAN region can be selected at RUNTIME from the GPS location
  (lora_region.*).  Implements the same ttn.h interface used by main.cpp.

  The API calls here were checked against RadioLib 6.6.0 headers (beginOTAA,
  activateOTAA, sendReceive, get/setBufferNonces, get/setBufferSession, the
  negative success codes, and the per-region data-rate mapping).  It still needs
  a real device for end-to-end validation (timing, Helium sub-bands, antenna).
  See RADIOLIB_MIGRATION.md for the bring-up checklist.
*/

#include "ttn.h"

#ifdef USE_RADIOLIB

#include <Arduino.h>
#include <Preferences.h>
#include <RadioLib.h>
#include <esp_system.h>  // esp_efuse_mac_get_default

#include <vector>

#include "configuration.h"
#include "credentials.h"
#include "gps.h"
#include "lora_region.h"
#include "screen.h"

// -----------------------------------------------------------------------------
// Radio + node
// -----------------------------------------------------------------------------
// SX1276 on the T-Beam: Module(cs/nss, irq/dio0, reset, gpio/dio1)
static SX1276 radio = new Module(NSS_GPIO, DIO0_GPIO, RESET_GPIO, DIO1_GPIO);
static LoRaWANNode *node = nullptr;

static lora_region_t g_region = REGION_UNKNOWN;
static lora_region_t g_fallback_region = REGION_EU868;  // used if no/unknown GPS fix in time
static bool g_joined = false;
static bool g_join_pending = false;     // asked to join, waiting for a GPS fix
static uint32_t g_join_request_ms = 0;  // when ttn_join() was called
static dr_t g_sf = LORAWAN_SF;          // SF7..SF10 (our numbering)
static bool g_adr = LORAWAN_ADR;

// How long to wait for a GPS fix (to pick the region) before falling back.
#define JOIN_GPS_WAIT_MS (3 * 60 * 1000UL)

// NVS namespace for LoRa state (frame count, nonces, session, fallback region).
#define LORA_NVS "lora"

// Last downlink captured during sendReceive(), exposed via ttn_response().
static uint8_t g_dn[64];
static size_t g_dn_len = 0;
static uint8_t g_dn_port = 0;

// Uplink frame counter, mirrored to flash like the LMIC backend.
static RTC_DATA_ATTR uint32_t count = 0;

static std::vector<void (*)(uint8_t)> _cbs;
static void fire(uint8_t m) {
  for (auto c : _cbs) c(m);
}

// -----------------------------------------------------------------------------
// Credentials (runtime-configurable, same model as the LMIC backend)
// -----------------------------------------------------------------------------
static uint8_t ram_appeui[8];
static uint8_t ram_appkey[16];

// DevEUI generator from the MAC, used when DEVEUI is all-zero.
static void gen_lora_deveui(uint8_t *pdeveui) {
  uint8_t *p = pdeveui, dmac[6];
  esp_efuse_mac_get_default(dmac);
  *p++ = 0xFF;
  *p++ = 0xFE;
  for (int i = 0; i < 6; i++) *p++ = dmac[5 - i];
}

static void load_credentials_internal() {
  memcpy_P(ram_appeui, APPEUI, 8);
  memcpy_P(ram_appkey, APPKEY, 16);
  Preferences p;
  if (p.begin("creds", true)) {
    if (p.getBytesLength("appeui") == 8)
      p.getBytes("appeui", ram_appeui, 8);
    if (p.getBytesLength("appkey") == 16)
      p.getBytes("appkey", ram_appkey, 16);
    if (p.getBytesLength("deveui") == 8)
      p.getBytes("deveui", DEVEUI, 8);
    p.end();
  }
  // Generate a DevEUI from the MAC if it is still all-zero.
  bool zero = true;
  for (int i = 0; i < 8; i++)
    if (DEVEUI[i])
      zero = false;
  if (zero)
    gen_lora_deveui(DEVEUI);
}

#ifdef ENABLE_CONFIG_PORTAL
void ttn_load_credentials(void) {
  load_credentials_internal();
}
void ttn_set_credentials(const uint8_t *deveui, const uint8_t *appeui, const uint8_t *appkey) {
  memcpy(DEVEUI, deveui, 8);
  memcpy(ram_appeui, appeui, 8);
  memcpy(ram_appkey, appkey, 16);
  Preferences p;
  if (p.begin("creds", false)) {
    p.putBytes("deveui", DEVEUI, 8);
    p.putBytes("appeui", ram_appeui, 8);
    p.putBytes("appkey", ram_appkey, 16);
    p.end();
  }
}
void ttn_get_credentials(uint8_t *deveui, uint8_t *appeui, uint8_t *appkey) {
  memcpy(deveui, DEVEUI, 8);
  memcpy(appeui, ram_appeui, 8);
  memcpy(appkey, ram_appkey, 16);
}
#endif  // ENABLE_CONFIG_PORTAL

// -----------------------------------------------------------------------------
// Fallback region (used when there is no/an unknown GPS fix at join time)
// -----------------------------------------------------------------------------
static void load_fallback_region() {
  Preferences p;
  if (p.begin(LORA_NVS, true)) {
    uint8_t r = p.getUChar("fbregion", (uint8_t)REGION_EU868);
    if (r > REGION_KR920)
      r = REGION_EU868;
    g_fallback_region = (lora_region_t)r;
    p.end();
  }
}

lora_region_t ttn_get_fallback_region(void) {
  return g_fallback_region;
}

void ttn_set_fallback_region(lora_region_t r) {
  if (r == REGION_UNKNOWN || r > REGION_KR920)
    return;
  g_fallback_region = r;
  Preferences p;
  if (p.begin(LORA_NVS, false)) {
    p.putUChar("fbregion", (uint8_t)r);
    p.end();
  }
}

// Build a RadioLib uint64 EUI from our LSB-first byte array (Helium MSB value).
static uint64_t to_eui(const uint8_t *lsb) {
  uint64_t v = 0;
  for (int i = 7; i >= 0; i--) v = (v << 8) | lsb[i];
  return v;
}

// -----------------------------------------------------------------------------
// Region / band / data-rate helpers
// -----------------------------------------------------------------------------
// Map our region enum to a RadioLib band pointer.
static const LoRaWANBand_t *radiolib_band(lora_region_t r) {
  switch (r) {
    case REGION_US915:
      return &US915;
    case REGION_AU915:
      return &AU915;
    case REGION_AS923:
      return &AS923;
    case REGION_IN865:
      return &IN865;
    case REGION_KR920:
      return &KR920;
    case REGION_EU868:
    default:
      return &EU868;
  }
}

// Map our SF (7..10) to a LoRaWAN data-rate index for the active region
// (per the LoRaWAN regional parameters used by RadioLib).
static uint8_t sf_to_datarate(lora_region_t r, dr_t sf) {
  switch (r) {
    case REGION_US915:
    case REGION_AU915:
      // DR0=SF10 .. DR3=SF7
      return (uint8_t)(10 - sf);  // SF10->0, SF9->1, SF8->2, SF7->3
    default:
      // EU868/AS923/IN865/KR920: DR0=SF12 .. DR5=SF7
      return (uint8_t)(12 - sf);  // SF10->2, SF9->3, SF8->4, SF7->5
  }
}

// -----------------------------------------------------------------------------
// ttn.h interface
// -----------------------------------------------------------------------------
void ttn_register(void (*callback)(uint8_t message)) {
  _cbs.push_back(callback);
}

const char *ttn_region_name(void) {
  return (g_region == REGION_UNKNOWN) ? "auto" : region_to_string(g_region);
}

bool ttn_setup() {
  load_credentials_internal();
  load_fallback_region();

  Serial.println();
  Serial.println("[RadioLib] LoRaWAN backend (runtime region from GPS)");

  int16_t state = radio.begin();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("[RadioLib] radio.begin() failed: %d\n", state);
    return false;
  }
  return true;
}

// -----------------------------------------------------------------------------
// Session / nonces persistence (NVS namespace "lora")
//
// Nonces hold the DevNonce/JoinNonce counters that MUST survive reboots, or the
// network rejects join requests (DevNonce reuse).  Session holds the live
// session keys + frame counters so we can resume without re-joining.
// -----------------------------------------------------------------------------
static void save_nonces() {
  if (!node)
    return;
  uint8_t *b = node->getBufferNonces();
  Preferences p;
  if (p.begin(LORA_NVS, false)) {
    p.putBytes("nonces", b, RADIOLIB_LORAWAN_NONCES_BUF_SIZE);
    p.end();
  }
}

static void save_session() {
  if (!node)
    return;
  uint8_t *b = node->getBufferSession();
  Preferences p;
  if (p.begin(LORA_NVS, false)) {
    p.putBytes("session", b, RADIOLIB_LORAWAN_SESSION_BUF_SIZE);
    p.end();
  }
}

// Restore persisted nonces (+ session, if any) into the node before activation.
static void restore_nonces_session() {
  Preferences p;
  if (!p.begin(LORA_NVS, true))
    return;
  uint8_t nb[RADIOLIB_LORAWAN_NONCES_BUF_SIZE];
  if (p.getBytesLength("nonces") == sizeof(nb)) {
    p.getBytes("nonces", nb, sizeof(nb));
    node->setBufferNonces(nb);

    uint8_t sb[RADIOLIB_LORAWAN_SESSION_BUF_SIZE];
    if (p.getBytesLength("session") == sizeof(sb)) {
      p.getBytes("session", sb, sizeof(sb));
      node->setBufferSession(sb);  // lets activateOTAA() resume instead of re-joining
    }
  }
  p.end();
}

// Actually perform the OTAA join now that a region is known.
static void do_join(lora_region_t region) {
  g_region = region;
  uint8_t subBand = region_helium_subband(region);

  if (node) {
    delete node;
    node = nullptr;
  }
  node = new LoRaWANNode(&radio, radiolib_band(region), subBand);

  uint64_t joinEUI = to_eui(ram_appeui);
  uint64_t devEUI = to_eui(DEVEUI);

  char msg[40];
  snprintf(msg, sizeof(msg), "Join %s...\n", region_to_string(region));
  screen_print(msg);
  Serial.printf("[RadioLib] Joining %s (subBand %d)\n", region_to_string(region), subBand);

  // LoRaWAN 1.0.x (Helium): only the AppKey is used, so nwkKey is NULL.
  node->beginOTAA(joinEUI, devEUI, NULL, ram_appkey);
  restore_nonces_session();

  fire(EV_JOINING);

  // activateOTAA() resumes a restored session or performs a fresh OTAA join.
  int16_t state = node->activateOTAA(sf_to_datarate(region, g_sf));
  if (state == RADIOLIB_LORAWAN_NEW_SESSION || state == RADIOLIB_LORAWAN_SESSION_RESTORED) {
    g_joined = true;
    g_join_pending = false;
    save_nonces();   // DevNonce advanced on a fresh join -- persist it
    save_session();  // persist the live session
    Serial.printf("[RadioLib] Joined (%s)\n", state == RADIOLIB_LORAWAN_NEW_SESSION ? "new" : "restored");
    // main.cpp marks isJoined once it has seen both of these.
    fire(EV_JOINING);
    fire(EV_JOINED);
  } else {
    save_nonces();  // a failed join attempt still consumes a DevNonce -- persist it
    Serial.printf("[RadioLib] Join failed: %d (will retry)\n", state);
    fire(EV_JOIN_FAILED);
  }
}

void ttn_join(void) {
  // Defer the real join until we have a GPS fix so the region can be chosen
  // from the location.  ttn_loop() performs the join when a fix arrives, or
  // after JOIN_GPS_WAIT_MS falls back to the configured region.
  g_join_pending = true;
  g_joined = false;
  g_join_request_ms = millis();
  Serial.printf("[RadioLib] Join deferred for GPS region (fallback %s after %lus)\n",
                region_to_string(g_fallback_region), JOIN_GPS_WAIT_MS / 1000);
  screen_print("Wait GPS for region\n");
}

void ttn_loop(void) {
  if (!g_join_pending || g_joined)
    return;

  lora_region_t region = REGION_UNKNOWN;
  if (tGPS.location.isValid())
    region = region_from_location(tGPS.location.lat(), tGPS.location.lng());

  // No fix yet, or the fix is outside any known box: wait a while, then fall
  // back to the user-configured region so the mapper can still join.
  if (region == REGION_UNKNOWN && (millis() - g_join_request_ms) > JOIN_GPS_WAIT_MS) {
    Serial.printf("[RadioLib] No usable GPS region; using fallback %s\n", region_to_string(g_fallback_region));
    region = g_fallback_region;
  }

  if (region != REGION_UNKNOWN)
    do_join(region);
}

boolean ttn_send(uint8_t *data, uint8_t data_size, uint8_t port, bool confirmed) {
  if (!node || !g_joined)
    return false;

  node->setDatarate(sf_to_datarate(g_region, g_sf));  // keep a fixed SF (ADR off for mappers)
  fire(EV_TXSTART);

  g_dn_len = sizeof(g_dn);
  g_dn_port = port;
  int16_t state = node->sendReceive(data, data_size, port, g_dn, &g_dn_len, confirmed);

  count++;
  ttn_write_prefs();  // mirror our app-level frame count to flash
  save_session();     // persist RadioLib's frame counters so they survive reboot

  // The uplink itself succeeded unless we got a real error.  "No downlink" is
  // a normal, successful outcome (negative sentinel, not a failure).
  if (state < RADIOLIB_ERR_NONE && state != RADIOLIB_LORAWAN_NO_DOWNLINK) {
    Serial.printf("[RadioLib] sendReceive error: %d\n", state);
    g_dn_len = 0;
    return false;
  }

  if (state == RADIOLIB_LORAWAN_NO_DOWNLINK)
    g_dn_len = 0;
  if (g_dn_len > 0)
    fire(EV_RESPONSE);  // a downlink was received
  fire(EV_TXCOMPLETE);
  return true;
}

size_t ttn_response_len(void) {
  return g_dn_len;
}

void ttn_response(uint8_t *port, uint8_t *buffer, size_t len) {
  if (port)
    *port = g_dn_port;
  size_t n = (len < g_dn_len) ? len : g_dn_len;
  memcpy(buffer, g_dn, n);
}

void ttn_set_sf(dr_t sf) {
  g_sf = sf;
  if (node)
    node->setDatarate(sf_to_datarate(g_region, g_sf));
}

void ttn_get_sf_name(char *b, size_t len) {
  snprintf(b, len, "SF%d BW125", g_sf);
}

void ttn_adr(bool enabled) {
  // RadioLib 6.6 has no public ADR toggle; mappers run with a fixed SF (ADR off)
  // which is the default here.  Kept for ttn.h interface compatibility.
  g_adr = enabled;
}

uint32_t ttn_get_count() {
  return count;
}

void ttn_write_prefs(void) {
  static uint32_t last = UINT32_MAX;
  uint32_t now = millis();
  if (now >= last && (now - last) < 5 * 60 * 1000L)
    return;  // rate-limit flash writes to ~once per 5 min
  last = now;
  Preferences p;
  if (p.begin("lora", false)) {
    p.putUInt("count", count);
    p.end();
  }
}

void ttn_erase_prefs(void) {
  Preferences p;
  if (p.begin("lora", false)) {
    p.clear();
    p.end();
  }
}

#endif  // USE_RADIOLIB
