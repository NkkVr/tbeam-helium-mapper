/*
  WiFi + BLE configuration portal for the Helium Mapper.

  Provides an on-demand configuration mode so all of the runtime Mapper
  settings can be adjusted from a phone or laptop without a USB cable.

  WiFi and BLE are offered as SEPARATE modes that are never active at the same
  time.  This is deliberate: the ESP32's software WiFi/BLE coexistence is only
  reliable in WiFi station mode -- running a WiFi Access Point and BLE together
  is unstable (AP mode cannot use the power-save that time-division relies on).
  Each mode therefore brings up exactly one radio protocol and shuts the other
  one down first.

  Both are entered from the on-screen menu and block the normal mapping loop
  while active.  The radios are fully torn down on exit to preserve battery.
*/
#pragma once

// WiFi Access Point + web UI only (BLE forced off).  Blocks until the user
// exits (long button hold, web "exit" button, or timeout).  Saves prefs.
void config_portal_run_wifi(void);

// BLE GATT service only (WiFi forced off).  Blocks until the user exits
// (long button hold or timeout).  Saves prefs.
void config_portal_run_ble(void);
