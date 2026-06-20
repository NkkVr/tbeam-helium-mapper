/*
  GPS-location -> LoRaWAN region selection.

  This module is intentionally free of any radio-stack dependency so it can be
  reasoned about (and unit-tested) on its own.  ttn_radiolib.cpp translates the
  lora_region_t value into the matching RadioLib band + sub-band at runtime.

  NOTE: the location->region mapping is an APPROXIMATION based on coarse
  geographic boxes.  LoRaWAN frequency plans are assigned per-country and there
  are exceptions; treat the result as a sensible default that the user can
  override.  It is "good enough" to pick the right plan automatically for a
  travelling mapper and to warn about an obvious mismatch.
*/
#pragma once

#include <Arduino.h>

enum lora_region_t {
  REGION_UNKNOWN = 0,
  REGION_EU868,
  REGION_US915,
  REGION_AU915,
  REGION_AS923,
  REGION_IN865,
  REGION_KR920,
};

// Pick a region from a GPS fix.  Returns REGION_UNKNOWN if the location does
// not fall into a known box.
lora_region_t region_from_location(double lat, double lon);

// Human-readable name, e.g. "EU868".
const char *region_to_string(lora_region_t r);

// Helium-recommended sub-band (1-indexed, RadioLib style) for the 64/72-channel
// plans; returns 0 for regions where a sub-band does not apply.
//   US915 -> 2  (channels 8-15)
//   AU915 -> 6  (Helium DualPlan, channels 40-47, since 2022-11-17)
uint8_t region_helium_subband(lora_region_t r);
