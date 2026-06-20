#pragma once

#include <Arduino.h>
#ifdef USE_RADIOLIB
#include "lmic_compat.h"  // dr_t / DR_SF* / EV_* without LMIC
#else
#include <lmic.h>
#endif

uint32_t ttn_get_count();
boolean ttn_send(uint8_t* data, uint8_t data_size, uint8_t port, bool confirmed);
void ttn_set_sf(dr_t sf);
void ttn_get_sf_name(char* b, size_t len);
size_t ttn_response_len(void);
void ttn_erase_prefs(void);
void ttn_loop(void);
void ttn_write_prefs(void);
void ttn_response(uint8_t* port, uint8_t* buffer, size_t len);
void ttn_adr(bool enabled);
void ttn_join(void);
bool ttn_setup(void);

#ifdef ENABLE_CONFIG_PORTAL
// Runtime LoRaWAN credentials (entered via the WiFi config portal).
// DevEUI/AppEUI are stored LSB-first (LMIC order); AppKey is MSB-first.
void ttn_load_credentials(void);
void ttn_set_credentials(const uint8_t* deveui, const uint8_t* appeui, const uint8_t* appkey);
void ttn_get_credentials(uint8_t* deveui, uint8_t* appeui, uint8_t* appkey);
#endif

#ifdef USE_RADIOLIB
#include "lora_region.h"
// Active region name (runtime-selected from GPS in the RadioLib backend).
const char* ttn_region_name(void);
// Fallback region used when there is no/an unknown GPS fix at join time.
lora_region_t ttn_get_fallback_region(void);
void ttn_set_fallback_region(lora_region_t r);
#endif
