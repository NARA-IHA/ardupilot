#include "Plane.h"

#include <AP_Logger/AP_Logger.h>

// ground test: flap input and message per step, each held for TEST_STEP_MS
static const struct {
    int8_t pct;
    const char *text;
} test_steps[] = {
    {  0, "SVS test: notr" },
    { 50, "SVS test: FLAP - ic flaplar ASAGI" },
    {100, "SVS test: CROW - dis kanatciklar da YUKARI" },
    {  0, "SVS test: bitti, notr" },
};
static const uint32_t TEST_STEP_MS = 4000;

const AP_Param::GroupInfo Savasan::var_info[] = {
    // @Param: OSPD
    // @DisplayName: Overspeed airbrake start
    // @Description: Above this equivalent airspeed the flaps (and with DSPOILER crow the ailerons) are deployed automatically, in every mode except MANUAL, whatever the companion computer does. 0 disables.
    // @Units: m/s
    // @Range: 0 60
    // @User: Advanced
    AP_GROUPINFO("OSPD", 1, Savasan, ospd, 0),

    // @Param: OSPD_BAND
    // @DisplayName: Overspeed airbrake band
    // @Description: Airbrake rises linearly from SVS_OSPD to SVS_OSPD + this speed.
    // @Units: m/s
    // @Range: 0.5 20
    // @User: Advanced
    AP_GROUPINFO("OSPD_BAND", 2, Savasan, ospd_band, 4),

    // @Param: OSPD_FLAP
    // @DisplayName: Overspeed airbrake flap input
    // @Description: Flap input at full overspeed airbrake. With DSPOILER_OPTS 4 (progressive crow), 50 is full flap only and 100 adds the ailerons up. Mind the flap structural speed.
    // @Units: %
    // @Range: 0 100
    // @User: Advanced
    AP_GROUPINFO("OSPD_FLAP", 3, Savasan, ospd_flap, 100),

    // @Param: CC_COMPID
    // @DisplayName: Companion component id
    // @Description: MAVLink component id of the companion computer whose HEARTBEAT is watched (same system id as the autopilot). 0 disables the watchdog.
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("CC_COMPID", 4, Savasan, cc_compid, MAV_COMP_ID_ONBOARD_COMPUTER),

    // @Param: CC_TOUT
    // @DisplayName: Companion heartbeat timeout
    // @Description: The companion link is lost when no HEARTBEAT arrives for this long. Its flap request is dropped and a warning is sent.
    // @Units: s
    // @Range: 1 10
    // @User: Advanced
    AP_GROUPINFO("CC_TOUT", 5, Savasan, cc_tout, 3),

    // @Param: CC_RTL
    // @DisplayName: Companion loss RTL delay
    // @Description: If the companion link stays lost this long while in GUIDED, switch to RTL (GUIDED alone already loiters after GUIDED_TIMEOUT, at the safe fallback point the companion keeps sending). Once per loss; selecting GUIDED again is left to the pilot. 0 disables (default: RTL flies straight home and may cross no-fly zones the autopilot does not know).
    // @Units: s
    // @Range: 0 120
    // @User: Advanced
    AP_GROUPINFO("CC_RTL", 6, Savasan, cc_rtl, 0),

    // @Param: ARM_JTSN
    // @DisplayName: Require companion to arm
    // @Description: Refuse arming (auxiliary authorisation, part of ARMING_CHECK) unless the companion computer heartbeat is present and reports MAV_STATE_ACTIVE (boundary loaded, camera running).
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("ARM_JTSN", 7, Savasan, arm_jtsn, 1),

    // @Param: FORK
    // @DisplayName: NARA-IHA firmware marker
    // @Description: Present only in NARA-IHA/ardupilot firmware; the stock-firmware script savasan.lua (same SVS_ parameters) disables itself when it sees it. Not to be changed.
    // @ReadOnly: True
    // @User: Advanced
    AP_GROUPINFO("FORK", 8, Savasan, fork, 1),

    AP_GROUPEND
};

Savasan::Savasan()
{
    AP_Param::setup_object_defaults(this, var_info);
}

int8_t Savasan::request_pct() const
{
    if (req_ms == 0 || plane.control_mode != &plane.mode_guided ||
        AP_HAL::millis() - req_ms >= plane.g2.guided_timeout * 1000.0f) {
        return 0;
    }
    return req_pct;
}

int8_t Savasan::test_pct() const
{
    return test_step == 0 ? 0 : test_steps[test_step - 1].pct;
}

int8_t Savasan::flap_percent()
{
    os_pct = 0;
    float eas;
    if (is_positive(ospd) && plane.control_mode != &plane.mode_manual && plane.ahrs.airspeed_EAS(eas)) {
        const float frac = constrain_float((eas - ospd) / MAX(ospd_band.get(), 0.5f), 0, 1);
        os_pct = frac * constrain_int16(ospd_flap, 0, 100);
    }
    return MAX(MAX(request_pct(), os_pct), test_pct());
}

MAV_RESULT Savasan::handle_flap_request(float value)
{
    if (plane.control_mode != &plane.mode_guided || isnan(value)) {
        return MAV_RESULT_DENIED;
    }
    req_pct = constrain_float(value, 0, 1) * 100;
    req_ms = AP_HAL::millis();
    return MAV_RESULT_ACCEPTED;
}

MAV_RESULT Savasan::handle_ground_test(float start)
{
    if (!is_positive(start)) {
        if (test_step != 0) {
            test_step = 0;
            gcs().send_text(MAV_SEVERITY_INFO, "SVS test: durduruldu");
        }
        return MAV_RESULT_ACCEPTED;
    }
    if (plane.arming.is_armed()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "SVS test: once disarm");
        return MAV_RESULT_DENIED;
    }
    if (test_step != 0) {
        return MAV_RESULT_ACCEPTED;     // already running (repeated command)
    }
    test_step = 1;
    test_step_ms = AP_HAL::millis();
    gcs().send_text(MAV_SEVERITY_INFO, "%s", test_steps[0].text);
    return MAV_RESULT_ACCEPTED;
}

void Savasan::handle_heartbeat(const mavlink_message_t &msg)
{
    if (cc_compid <= 0 || msg.sysid != mavlink_system.sysid || msg.compid != cc_compid) {
        return;
    }
    if (lost_ms != 0) {
        gcs().send_text(MAV_SEVERITY_INFO, "SVS: Jetson geri geldi");
        lost_ms = 0;
    }
    rtl_done = false;
    ready = mavlink_msg_heartbeat_get_system_status(&msg) == MAV_STATE_ACTIVE;
    hb_ms = AP_HAL::millis();
}

void Savasan::update_companion(uint32_t now)
{
    if (cc_compid <= 0 || hb_ms == 0) {
        return;     // never heard from it: nothing to lose
    }
    if (lost_ms == 0) {
        if (now - hb_ms < cc_tout * 1000.0f) {
            return;
        }
        lost_ms = now;
        ready = false;
        clear_flap_request();
        gcs().send_text(MAV_SEVERITY_CRITICAL, "SVS: JETSON KOPTU");
    }
    if (!rtl_done && is_positive(cc_rtl) && now - lost_ms >= cc_rtl * 1000.0f &&
        plane.control_mode == &plane.mode_guided && plane.arming.is_armed()) {
        rtl_done = true;
        if (plane.set_mode(plane.mode_rtl, ModeReason::GCS_FAILSAFE)) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "SVS: Jetson yok, RTL");
        }
    }
}

void Savasan::update_ground_test(uint32_t now)
{
    if (test_step == 0) {
        return;
    }
    if (plane.arming.is_armed()) {
        test_step = 0;
        gcs().send_text(MAV_SEVERITY_WARNING, "SVS test: arm edildi, durdu");
        return;
    }
    if (now - test_step_ms < TEST_STEP_MS) {
        return;
    }
    test_step_ms = now;
    if (test_step >= ARRAY_SIZE(test_steps)) {
        test_step = 0;
        return;
    }
    gcs().send_text(MAV_SEVERITY_INFO, "%s", test_steps[test_step].text);
    test_step++;
}

void Savasan::update_prearm()
{
#if AP_ARMING_AUX_AUTH_ENABLED
    if (!have_auth_id) {
        have_auth_id = plane.arming.get_aux_auth_id(auth_id);
        if (!have_auth_id) {
            return;
        }
    }
    if (arm_jtsn <= 0) {
        plane.arming.set_aux_auth_passed(auth_id);
    } else if (hb_ms == 0 || lost_ms != 0) {
        plane.arming.set_aux_auth_failed(auth_id, "SVS: Jetson yok");
    } else if (!ready) {
        plane.arming.set_aux_auth_failed(auth_id, "SVS: Jetson hazir degil");
    } else {
        plane.arming.set_aux_auth_passed(auth_id);
    }
#endif
}

void Savasan::update()
{
    const uint32_t now = AP_HAL::millis();
    update_companion(now);
    update_ground_test(now);
    update_prearm();
    log();
}

void Savasan::log()
{
#if HAL_LOGGING_ENABLED
    // @LoggerMessage: SVS
    // @Description: NARA-IHA additions
    // @Field: TimeUS: Time since system startup
    // @Field: Req: companion flap request in effect (%)
    // @Field: Os: overspeed airbrake (%)
    // @Field: Tst: ground test step (0 = off)
    // @Field: CC: companion link (0 never seen, 1 ok, 2 lost)
    AP::logger().WriteStreaming("SVS", "TimeUS,Req,Os,Tst,CC", "Qbbbb",
                                AP_HAL::micros64(), request_pct(), os_pct, int8_t(test_step),
                                int8_t(hb_ms == 0 ? 0 : lost_ms == 0 ? 1 : 2));
#endif
}
