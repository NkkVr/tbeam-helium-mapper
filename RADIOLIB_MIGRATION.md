# RadioLib migration (runtime region selection)

## Why
MCCI LMIC compiles the LoRaWAN region into the binary and cannot switch it at
runtime.  To pick the region automatically from the GPS location, the LoRaWAN
layer is being migrated to **RadioLib**, whose `LoRaWANNode` takes the band as a
runtime parameter and supports the SX1276 on the T-Beam v1.1.

## Approach: parallel backend behind a flag
The migration is gated by the `USE_RADIOLIB` build flag so the proven LMIC build
stays the default and nothing existing breaks:

- `USE_RADIOLIB` **unset** (default): `ttn.cpp` (LMIC) is compiled, exactly as
  before. `pio run -e release`, `-e portal_eu`, etc. are unchanged.
- `USE_RADIOLIB` **set**: `ttn.cpp` compiles to nothing and `ttn_radiolib.cpp`
  provides the same `ttn.h` interface using RadioLib, choosing the region from
  GPS via `lora_region.*`. Built with `pio run -e portal_radiolib`.

`lmic_compat.h` provides the few LMIC types/constants (`dr_t`, `DR_SF7..10`,
`EV_*`) that `main.cpp` / `config_portal.cpp` reference, so those files compile
without LMIC headers when `USE_RADIOLIB` is set.

## Files
- `lora_region.{h,cpp}` — pure GPS-location → region/sub-band mapping (no radio
  dependency; reviewable/unit-testable on its own). **Done.**
- `lmic_compat.h` — minimal `dr_t` / `DR_SF*` / `EV_*` shim for the RadioLib
  build. **Done.**
- `ttn_radiolib.cpp` — RadioLib implementation of the `ttn.h` interface.
  **First pass — needs hardware bring-up.**
- `ttn.cpp` — guarded with `#ifndef USE_RADIOLIB`. **Done.**

## Region auto-selection flow
1. At boot the mapper waits for a valid GPS fix before the first join (a mapper
   has nothing useful to send without a location anyway).
2. `region_from_location(lat, lon)` returns `lora_region_t`.
3. `ttn_radiolib.cpp` maps that to a RadioLib `LoRaWANBand_t` + Helium sub-band
   (`region_helium_subband`) and constructs the `LoRaWANNode`.
4. The chosen region is shown on screen / in the web UI.

The location→region map is approximate (continental boxes with a few country
exceptions). It is a sensible default; the user can still flash a fixed-region
build if needed.

## Status

Verified against the RadioLib 6.6.0 headers (`src/protocols/LoRaWAN/LoRaWAN.h`,
`src/TypeDef.h`):

- [x] API signatures: `beginOTAA(joinEUI, devEUI, nwkKey, appKey)` (void;
      `nwkKey = NULL` for LoRaWAN 1.0.x / Helium), `activateOTAA(initialDr)`,
      `sendReceive(data, len, fPort, dataDown, &lenDown, confirmed)`.
- [x] Success codes are negative sentinels: `RADIOLIB_LORAWAN_NEW_SESSION`
      (-1118), `RADIOLIB_LORAWAN_SESSION_RESTORED` (-1117);
      `RADIOLIB_LORAWAN_NO_DOWNLINK` (-1116) is a successful uplink.
- [x] `setADR()` is not public in 6.6.0 — mappers run a fixed SF (ADR off).
- [x] DevEUI/AppEUI converted to RadioLib `uint64_t`; AppKey passed MSB-first.
- [x] Session + nonce persistence in NVS via `get/setBufferNonces` and
      `get/setBufferSession` (avoids DevNonce-reuse join failures).
- [x] SF→data-rate mapping per LoRaWAN regional parameters
      (US/AU: DR0=SF10..DR3=SF7; EU/AS/IN/KR: DR0=SF12..DR5=SF7).
- [x] Screen/LED event bridge fires `EV_JOINING/EV_JOINED/EV_TXSTART/`
      `EV_TXCOMPLETE/EV_RESPONSE`.

Still requires a device + serial monitor (cannot be verified in sandbox):

- [ ] End-to-end OTAA join + uplink on a real T-Beam + Helium gateway.
- [ ] Confirm Helium sub-bands in practice (US915 = 2, AU915 = 6).
- [ ] Validate `radio.begin()` defaults / timing for SX1276.
- [ ] Optional: a user-selectable fallback region when GPS gives no fix.

## Build
```
pio run -e portal_radiolib -t erase -t upload   # first flash (partition change)
pio device monitor -b 115200
```
