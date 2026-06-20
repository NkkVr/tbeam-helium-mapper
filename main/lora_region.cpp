/*
  GPS-location -> LoRaWAN region selection.  See lora_region.h.
*/

#include "lora_region.h"

// Helper: is (lat,lon) inside the inclusive box [lat0,lat1] x [lon0,lon1]?
static bool in_box(double lat, double lon, double lat0, double lat1, double lon0, double lon1) {
  return lat >= lat0 && lat <= lat1 && lon >= lon0 && lon <= lon1;
}

lora_region_t region_from_location(double lat, double lon) {
  // Specific countries/areas first (they would otherwise fall into a broader
  // continental box below).

  // South Korea -> KR920
  if (in_box(lat, lon, 33.0, 39.5, 124.0, 131.5))
    return REGION_KR920;

  // Japan -> AS923
  if (in_box(lat, lon, 24.0, 46.0, 129.0, 146.5))
    return REGION_AS923;

  // India / Sri Lanka -> IN865
  if (in_box(lat, lon, 5.0, 36.0, 68.0, 90.0))
    return REGION_IN865;

  // South-East Asia / China / Indonesia -> AS923
  if (in_box(lat, lon, -11.0, 41.0, 90.0, 145.0))
    return REGION_AS923;

  // Australia / New Zealand -> AU915
  if (in_box(lat, lon, -50.0, -10.0, 110.0, 180.0))
    return REGION_AU915;

  // South & Central America -> AU915 (Helium plan for most of the region)
  if (in_box(lat, lon, -56.0, 14.0, -82.0, -34.0))
    return REGION_AU915;

  // North America (US / Canada / Mexico) -> US915
  if (in_box(lat, lon, 7.0, 72.0, -170.0, -50.0))
    return REGION_US915;

  // Europe / Africa / Middle East / western Russia -> EU868
  if (in_box(lat, lon, -35.0, 72.0, -25.0, 63.0))
    return REGION_EU868;

  return REGION_UNKNOWN;
}

const char *region_to_string(lora_region_t r) {
  switch (r) {
    case REGION_EU868:
      return "EU868";
    case REGION_US915:
      return "US915";
    case REGION_AU915:
      return "AU915";
    case REGION_AS923:
      return "AS923";
    case REGION_IN865:
      return "IN865";
    case REGION_KR920:
      return "KR920";
    default:
      return "Unknown";
  }
}

uint8_t region_helium_subband(lora_region_t r) {
  switch (r) {
    case REGION_US915:
      return 2;  // channels 8-15
    case REGION_AU915:
      return 6;  // Helium DualPlan, channels 40-47
    default:
      return 0;  // not applicable
  }
}
