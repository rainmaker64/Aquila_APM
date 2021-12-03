//
// Created by khancyr on 11/02/2021.
//
#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Arming/AP_Arming.h>
#include <AP_Math/vector2.h>
#include <AP_Math/vector3.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
#include "AC_PrecLand_SM.h"

const AP_Param::GroupInfo AC_PrecLand_SM::var_info[] = {

        // @Param: ENABLED
        // @DisplayName: Precision Land SM enabled/disabled
        // @Description: Precision Land SM enabled/disabled
        // @Values: 0:Disabled, 1:Enabled
        // @User: Advanced
        AP_GROUPINFO_FLAGS("ENABLED", 0, AC_PrecLand_SM, _enabled, 0, AP_PARAM_FLAG_ENABLE),

        // @Param: SRCH_ALT
        // @DisplayName: Search for beacon starting alt
        // @Description: Search for beacon starting alt
        // @Units: cm
        // @Range: 1000 3000
        // @User: Advanced
        AP_GROUPINFO("SRCH_ALT", 1, AC_PrecLand_SM, _search_start_alt, 1500),

        // @Param: SRCH_S_ALT
        // @DisplayName: Search for beacon stoping alt
        // @Description: Search for beacon stoping alt
        // @Units: cm
        // @Range: 500 1000
        // @User: Advanced
        AP_GROUPINFO("SRCH_S_ALT", 2, AC_PrecLand_SM, _search_stop_alt, 1000),

        // @Param: DESC_S_ALT
        // @DisplayName: Descend and alignate stop alt
        // @Description: Descend and alignate stop alt. We should be alignate correctly at this point or we will retry. Further descend won't be stopped.
        // @Units: cm
        // @Range: 100 200
        // @User: Advanced
        AP_GROUPINFO("DESC_S_ALT", 3, AC_PrecLand_SM, _descend_stop_alt, 200),

        // @Param: RETRY
        // @DisplayName: Number of RETRY
        // @Description: Number of RETRY
        // @User: Advanced
        AP_GROUPINFO("RETRY", 4, AC_PrecLand_SM, _retry_max, 3),

        // @Param: EM_BHV
        // @DisplayName: BEHAVIOR in case of battery failsafe.
        // @Description: BEHAVIOR in case of battery failsafe.
        // @Bitmask: 0:No changes, 1:Disable SM, 2: Disable Retry
        // @User: Advanced
        AP_GROUPINFO("EM_BHV", 5, AC_PrecLand_SM, _emergency_behavior, 0),

        // @Param: D_SPEED
        // @DisplayName: Descend speed max.
        // @Description: Descend speed max.
        // @Units: cm/s
        // @Range: 30 200
        // @Increment: 10
        // @User: Advanced
        AP_GROUPINFO("D_SPEED", 6, AC_PrecLand_SM, _descend_speed_max, 75),

        // @Param: F_ALT
        // @DisplayName: Failure altitude to reach before do safe landing.
        // @Description: Failure altitude to reach before do safe landing.
        // @Units: cm
        // @Range: 200 500
        // @User: Advanced
        AP_GROUPINFO("F_ALT", 7, AC_PrecLand_SM, _failure_alt, 300),

        // @Param: F_POSN
        // @DisplayName: Failure position oriented on North in m.
        // @Description: Position that will be reach for safe landing in case of precland failure. This a position in m on North of the home point
        // @Units: m
        // @Range: 2 5
        // @User: Advanced
        AP_GROUPINFO("F_POSN", 8, AC_PrecLand_SM, _failure_posn, 2),

        // @Param: ACC_ERR
        // @DisplayName: Acceptable Error in cm
        // @Description: Acceptable Error
        // @Units: cm
        // @Range: 2 30
        // @User: Advanced
        AP_GROUPINFO("ACC_ERR", 9, AC_PrecLand_SM, _acceptable_error_cm, 15),

        // @Param: S_L_ALT
        // @DisplayName: Land Altitude in cm
        // @Description: Altitude at which Land starts
        // @Units: cm
        // @Range: 15 100
        // @User: Advanced
        AP_GROUPINFO("S_L_ALT", 10, AC_PrecLand_SM, _start_land_alt_cm, 20),

        AP_GROUPEND
};

AC_PrecLand_SM::AC_PrecLand_SM(AC_PosControl& pos_control, AC_Loiter& loiter_nav, AC_WPNav& wp_nav, AC_PrecLand& precland):
        psm_pos_control(pos_control),
        psm_loiter_nav(loiter_nav),
        psm_wp_nav(wp_nav),
        psm_precland(precland)
{
    // set parameters to defaults
    AP_Param::setup_object_defaults(this, var_info);
}


/*
    Reset State-Machine
*/
void AC_PrecLand_SM::reset()
{
    psm_state = PLD_STATE::PRECISION_LAND_STATE_NOT_ACTIVE;
    psm_is_final_landing = false;
    psm_is_emergency = false;
    psm_retry_count = 0;
    psm_in_failure = false;
    psm_wp_nav_is_active = false;
    psm_initialised = false;

    psm_wp_nav.reset_radius();
    psm_wp_nav.reset_rangefinder_use();
}

/*
    Initialize
*/
void AC_PrecLand_SM::init()
{
    if (!_enabled.get() || !psm_precland.enabled() || !AP::arming().is_armed()) 
    {
        return;
    }

    psm_initialised = true;
    reset();
    if (!AP::ahrs().get_position(init_location)) 
    {
        init_location = AP::ahrs().get_home();  // default to home in case of issue ...
    }
    //gcs().send_text(MAV_SEVERITY_INFO, "Initialise Precland");
    search_start();
}

/*
    Run state-machine
*/
void AC_PrecLand_SM::run()
{
    if (!_enabled.get() || !psm_initialised || !psm_precland.enabled()) 
    {
        return;
    }

    if (psm_is_emergency && _emergency_behavior.get() & EM_BHV_DISABLE_SM) 
    {
        return;
    }

    if (!AP::arming().is_armed()) 
    {
        if (psm_state == PLD_STATE::PRECISION_LAND_STATE_DESCEND_FINAL) 
        {
            gcs().send_text(MAV_SEVERITY_INFO, "Landed on target");
            psm_state = PLD_STATE::PRECISION_LAND_STATE_SUCCESS;
        } 
        else 
        {
            if (psm_state != PLD_STATE::PRECISION_LAND_STATE_NOT_ACTIVE && psm_state != PLD_STATE::PRECISION_LAND_STATE_SUCCESS) 
            {
                psm_state = PLD_STATE::PRECISION_LAND_STATE_NOT_ACTIVE;
                gcs().send_text(MAV_SEVERITY_INFO, "Unexpected Land");
            }
        }
    }

    switch (psm_state) 
    {
        case PLD_STATE::PRECISION_LAND_STATE_NOT_ACTIVE:
        case PLD_STATE::PRECISION_LAND_STATE_SUCCESS:
            break;
        case PLD_STATE::PRECISION_LAND_STATE_SEARCH:
            search_run();
            break;
        case PLD_STATE::PRECISION_LAND_STATE_DESCEND:
            descend_run();
            break;
        case PLD_STATE::PRECISION_LAND_STATE_DESCEND_FINAL:
            descend_final_run();
            break;
        case PLD_STATE::PRECISION_LAND_STATE_RETRY:
            retry_run();
            break;
        case PLD_STATE::PRECISION_LAND_STATE_FAILED:
            failed_run();
            break;
    }
}

/*
    Start to search
*/
void AC_PrecLand_SM::search_start()
{
    psm_state = PLD_STATE::PRECISION_LAND_STATE_SEARCH;

    psm_wp_nav.set_radius(WP_RADIUS);
    psm_wp_nav.use_rangefinder(false);
    psm_wp_nav.wp_and_spline_init();

    if (psm_precland.target_acquired()) 
    {
        Vector2f target_pos;
        if (psm_precland.get_target_position_cm(target_pos)) 
        {
            psm_wp_nav.set_wp_destination(Vector3f(target_pos.x, target_pos.y, static_cast<float>(_search_start_alt.get() - 500)), false);
            psm_wp_nav.get_wp_destination(search_target);
        }
    } 
    else 
    {
        search_target = init_location;
        search_target.set_alt_cm((_search_start_alt.get() - 500), Location::AltFrame::ABOVE_HOME);
        psm_wp_nav.set_wp_destination(search_target);
    }

    psm_first_pass = true;
    psm_doing_first_pass = false;
    psm_wp_nav_is_active = true;
    gcs().send_text(MAV_SEVERITY_INFO, "search_start");
}

/*
    Run to search
*/
void AC_PrecLand_SM::search_run()
{
    if (alt_above_ground_cm >= _search_start_alt.get() || psm_doing_first_pass) 
    {
        if (psm_wp_nav.reached_wp_destination()) 
        {
            psm_doing_first_pass = false;
        }
    }
    else
    {
        // if we are under the stop alt, we get up
        if (alt_above_ground_cm < _search_stop_alt.get()) 
        {
            // get up to stop alt This unsure we always do the same patern and garanty the same descend phase
            if (psm_first_pass) 
            {
                search_target.set_alt_cm((_search_start_alt.get() - 500), Location::AltFrame::ABOVE_HOME);
                psm_wp_nav.set_wp_destination(search_target);
                psm_doing_first_pass = true;
                psm_first_pass = false;
            } 
            else 
            {
                // inform user
                psm_state = PLD_STATE::PRECISION_LAND_STATE_FAILED;
                failed_start();
            }
        }
        else
        {
            psm_first_pass = false;
            psm_doing_first_pass = false;
            // if we lock the target, go on the target
            if (psm_precland.target_acquired()) 
            {
                if (alt_above_ground_cm > _search_stop_alt + 1000) 
                {
                    Vector2f target_pos;
                    if (psm_precland.get_target_position_cm(target_pos)) 
                    {
                        psm_wp_nav.set_wp_destination(Vector3f(target_pos.x, target_pos.y, static_cast<float>(_search_stop_alt.get())), false);
                    }
                } 
                else 
                {
                    descend_start();
                }
            } 
            else 
            {
                // search the target
                // circle to stop alt
            }
        }
    }
}

/*
    Descend to start
*/
void AC_PrecLand_SM::descend_start()
{
    gcs().send_text(MAV_SEVERITY_INFO, "descend_start");
    psm_state = PLD_STATE::PRECISION_LAND_STATE_DESCEND;

    psm_avg_pos_error = 0.0f;
    avg_filter.reset();
    psm_wp_nav.reset_radius();
    psm_wp_nav.reset_rangefinder_use();
    psm_loiter_nav.init_target();
    psm_pos_control.init_xy_controller();
    last_target_time = AP_HAL::millis();
    psm_wp_nav_is_active = false;

    max_descend_speed = MIN(-psm_pos_control.get_max_speed_down(), _descend_speed_max);
}

/*
    Calculate the slow-down rate
*/
void AC_PrecLand_SM::calculate_slowdown()
{
    // we get back using loiter_nav for tighter locking on target. We will just slowdown the descend according to the pos error
    // recenter and descend
    if (psm_precland.target_acquired()) 
    {
        last_target_time = AP_HAL::millis();
        psm_avg_pos_error = avg_filter.apply(psm_pos_control.get_horizontal_error());
    } 
    else 
    {
        if (AP_HAL::millis() - last_target_time > 500) 
        {  
            //timeout of 0.5sec for launching a retry
            // inform user
            gcs().send_text(MAV_SEVERITY_INFO, "Lost target for more than 0.5s");
            psm_state = PLD_STATE::PRECISION_LAND_STATE_FAILED;
            failed_start();
            return;
        }
    }

    float land_slowdown = 0.0f;
    if (psm_avg_pos_error > _acceptable_error_cm.get()) 
    {
        // slowdown max at 2 * _acceptable_error_cm
        land_slowdown = MAX(0.0f, psm_avg_pos_error * (max_descend_speed / (_acceptable_error_cm * 2.0f)));
    }

    precland_sm_climb_rate = MIN(-MIN_DESCENT_SPEED, -max_descend_speed + land_slowdown);
}

/*
    Descending
*/
void AC_PrecLand_SM::descend_run()
{
    // Calculate the slow-down rate
    calculate_slowdown();

    // if the altitude is within "descend_stop_alt", then start to descend
    if (alt_above_ground_cm <= _descend_stop_alt.get()) 
    {
        // If the error is acceptable, then go to the final stage.
        if (psm_avg_pos_error <= _acceptable_error_cm) 
        {
            descend_final_start();
        } 
        else 
        {
            // try to give 1s on stationnary to alignate better
            if (descend_timer == 0) 
            {
                descend_timer = AP_HAL::millis();
            }

            if (AP_HAL::millis() - descend_timer > 1000) 
            {
                // inform user
                gcs().send_text(MAV_SEVERITY_INFO, "Failed to lock target accuratly");
                psm_state = PLD_STATE::PRECISION_LAND_STATE_FAILED;
                failed_start();
            } 
            else 
            {
                precland_sm_climb_rate = 0.1f; // slightly positive to ensure a little thrust up to stop inertia !
            }
        }
    }
}

/*
    Start "Final Descending Stage"
*/
void AC_PrecLand_SM::descend_final_start()
{
    gcs().send_text(MAV_SEVERITY_INFO, "descend_final_start");

    psm_state = PLD_STATE::PRECISION_LAND_STATE_DESCEND_FINAL;
    max_descend_speed = MIN(-psm_pos_control.get_max_speed_down(), _descend_speed_max) * 0.5f;
}

/*
    Run "Final Descending Stage"
*/
void AC_PrecLand_SM::descend_final_run()
{
    // Check if it is final landing stage
    if (alt_above_ground_cm <= (_descend_stop_alt * 0.25f) || alt_above_ground_cm <= _start_land_alt_cm.get()) 
    {
        psm_is_final_landing = true;  // unsure smooth landing by the land controler
    }

    // Calculate the slow-down rate
    calculate_slowdown();
}

/*
    Start "Failure Stage"
*/
void AC_PrecLand_SM::failed_start()
{
    psm_is_final_landing = false;
    bool allow_retry = !(psm_is_emergency && _emergency_behavior & EM_BHV_DISABLE_RETRY);

    gcs().send_text(MAV_SEVERITY_INFO, "failed_start");

    if (allow_retry && psm_retry_count < _retry_max.get()) 
    {
        gcs().send_text(MAV_SEVERITY_INFO, "Retry %d", psm_retry_count);
        psm_retry_count++;
        retry_start();
        return;
    }
    else
    {
        psm_state = PLD_STATE::PRECISION_LAND_STATE_FAILED;

        search_target = init_location;
        search_target.set_alt_cm(_failure_alt.get(), Location::AltFrame::ABOVE_HOME);
        search_target.offset(_failure_posn.get(), 0);  // set safe land at _failure_posn of north of the beacon.

        psm_wp_nav.set_wp_destination(search_target);
        psm_wp_nav_is_active = true;

        gcs().send_text(MAV_SEVERITY_INFO, "Failed to retry going to safe position");
    }
}

/*
    Run "Failure Stage"
*/
void AC_PrecLand_SM::failed_run()
{
    // case retry count failure, land at X m from the home without precland
    if (psm_wp_nav.reached_wp_destination()) 
    {
        // else do normal land
        psm_in_failure = true;
        psm_wp_nav_is_active = false;
        psm_state = PLD_STATE::PRECISION_LAND_STATE_NOT_ACTIVE;
    }
}

/*
    Start "Retry"
*/
void AC_PrecLand_SM::retry_start()
{
    psm_state = PLD_STATE::PRECISION_LAND_STATE_RETRY;

    psm_wp_nav.use_rangefinder(false);
    psm_wp_nav.wp_and_spline_init();

    // if the target is acquired, then move the target point
    if (psm_precland.target_acquired()) 
    {
        Vector2f target_pos;
        if (psm_precland.get_target_position_cm(target_pos)) 
        {
            psm_wp_nav.set_wp_destination(Vector3f(target_pos.x, target_pos.y, static_cast<float>(_search_start_alt.get())), false);
            psm_wp_nav.get_wp_destination(search_target);
        }
    } 
    else    // if the target is not acquired, then move the initial point
    {
        search_target = init_location;
        search_target.set_alt_cm(static_cast<int32_t>(_search_start_alt), Location::AltFrame::ABOVE_HOME);
        psm_wp_nav.set_wp_destination(search_target);
    }

    psm_wp_nav_is_active = true;
    gcs().send_text(MAV_SEVERITY_INFO, "retry_start");
}

/*
    Run "Retry"
*/
void AC_PrecLand_SM::retry_run()
{
    if (psm_wp_nav.reached_wp_destination()) 
    {
        search_start();
    }
}

/*
    Write logs
*/
void AC_PrecLand_SM::write_log()
{
    struct log_Precland_SM pkt = {
            LOG_PACKET_HEADER_INIT(LOG_PRECLAND_SM_MSG),
            .time_us               = AP_HAL::micros64(),
            .state                 =  static_cast<uint8_t>(psm_state),
            .alt_above_ground      =  alt_above_ground_cm,
            .last_alt_above_ground =  last_alt_above_ground_cm,
            .climb_rate            =  precland_sm_climb_rate,
            .avg_pos_error         =  psm_avg_pos_error,
            .retry                 =  psm_retry_count,
    };

    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

