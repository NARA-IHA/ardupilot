#pragma once

/*
  NARA-IHA (TEKNOFEST Savasan IHA) additions, kept in one place so the
  savasan branch rebases cleanly onto new Plane releases:

  - companion flap request in GUIDED (MAV_CMD_USER_1)
  - overspeed airbrake: flap/crow from airspeed, independent of the companion
  - companion link watchdog (companion HEARTBEAT): warn, then RTL from GUIDED
  - pre-arm: companion must be seen and report MAV_STATE_ACTIVE (SVS_ARM_JTSN)
  - ground surface test (MAV_CMD_USER_2), disarmed only
  - SVS dataflash message
 */

#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

class Savasan {
public:
    Savasan();

    static const AP_Param::GroupInfo var_info[];

    // 10Hz: companion watchdog, ground test sequence, logging
    void update();

    // flap percent the additions ask for this loop (0..100); larger of
    // this, auto and manual flap is applied
    int8_t flap_percent();

    // MAV_CMD_USER_1: param1 = flap input 0..1, GUIDED only
    MAV_RESULT handle_flap_request(float value);

    // MAV_CMD_USER_2: param1 1 = start ground test, 0 = stop
    MAV_RESULT handle_ground_test(float start);

    void handle_heartbeat(const mavlink_message_t &msg);

    // clear the companion flap request (mode change, link loss)
    void clear_flap_request() { req_ms = 0; }

private:
    AP_Float ospd;          // SVS_OSPD: airbrake starts above this EAS (m/s), 0 = off
    AP_Float ospd_band;     // SVS_OSPD_BAND: full airbrake at SVS_OSPD + this
    AP_Int8  ospd_flap;     // SVS_OSPD_FLAP: flap input at full airbrake (%)
    AP_Int16 cc_compid;     // SVS_CC_COMPID: companion component id (heartbeat)
    AP_Float cc_tout;       // SVS_CC_TOUT: companion heartbeat timeout (s)
    AP_Float cc_rtl;        // SVS_CC_RTL: RTL from GUIDED this long after loss (s), 0 = never
    AP_Int8  arm_jtsn;      // SVS_ARM_JTSN: refuse arming without a ready companion
    AP_Int8  fork;          // SVS_FORK: marker, tells savasan.lua the firmware has these additions

    // companion flap request
    int8_t req_pct;
    uint32_t req_ms;

    // overspeed airbrake output (for logging)
    int8_t os_pct;

    // companion link
    uint32_t hb_ms;
    uint32_t lost_ms;
    bool rtl_done;
    bool ready;             // companion heartbeat system_status == MAV_STATE_ACTIVE

    // pre-arm (auxiliary authorisation)
    uint8_t auth_id;
    bool have_auth_id;

    // ground test
    uint8_t test_step;
    uint32_t test_step_ms;

    int8_t request_pct() const;
    int8_t test_pct() const;
    void update_companion(uint32_t now);
    void update_ground_test(uint32_t now);
    void update_prearm();
    void log();
};
