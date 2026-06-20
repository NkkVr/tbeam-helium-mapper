/*
  Minimal LMIC compatibility shim for the RadioLib backend (USE_RADIOLIB).

  main.cpp and config_portal.cpp reference a handful of LMIC types/constants
  (the data-rate type and the EV_* event ids used to drive the screen/LED).
  When LMIC is not compiled in, this header provides drop-in equivalents so
  those files build unchanged.  ttn_radiolib.cpp fires these same EV_* ids.

  Note: the custom EV_QUEUED/EV_PENDING/EV_ACK/EV_RESPONSE ids live in
  configuration.h (100..103); the values here stay below that range.
*/
#pragma once

#include <Arduino.h>

// Data-rate / spreading-factor handle.  We only ever carry SF7..SF10.
typedef uint8_t dr_t;

#define DR_SF7 7
#define DR_SF8 8
#define DR_SF9 9
#define DR_SF10 10

// LMIC event ids referenced by main.cpp's lora_msg_callback().
#define EV_JOINING 1
#define EV_JOINED 2
#define EV_JOIN_FAILED 3
#define EV_REJOIN_FAILED 4
#define EV_TXSTART 5
#define EV_TXCOMPLETE 6
#define EV_RXCOMPLETE 7
#define EV_RXSTART 8
#define EV_TXCANCELED 9
#define EV_JOIN_TXCOMPLETE 10
#define EV_RESET 11
#define EV_LINK_DEAD 12
