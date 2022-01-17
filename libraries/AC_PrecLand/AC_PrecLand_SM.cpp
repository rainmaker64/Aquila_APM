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
    this->psm_state = PLD_STATE::PRECISION_LAND_STATE_NOT_ACTIVE;
    this->psm_is_final_landing = false;
    this->psm_is_emergency = false;
    this->psm_retry_count = 0;
    this->psm_in_failure = false;
    this->psm_wp_nav_is_active = false;
    this->psm_initialised = false;

    this->psm_wp_nav.reset_radius();
    this->psm_wp_nav.reset_rangefinder_use();
}

/*
    Initialize
*/
void AC_PrecLand_SM::init()
{
    if (_enabled.get() == false || this->psm_precland.enabled() == false || AP::arming().is_armed() == false) 
    {
        return;
    }

    // Initialize the variables before start.
    this->reset();

    // If init_location is invalid, then set it to home position.
    if (AP::ahrs().get_position(this->init_location) == false) 
    {
        this->init_location = AP::ahrs().get_home();  // default to home in case of issue ...
    }

    // set PSM initialized
    this->psm_initialised = true;

    gcs().send_text(MAV_SEVERITY_INFO, "Initialise Precland");

    this->search_start();
}

/*
    Run state-machine
*/
void AC_PrecLand_SM::run()
{
    if (_enabled.get() == false || this->psm_initialised == false || this->psm_precland.enabled() == false) 
    {
        return;
    }

    if (this->psm_is_emergency == true && (_emergency_behavior.get() & EM_BHV_DISABLE_SM) != 0) 
    {
        return;
    }

    if (AP::arming().is_armed() == false) 
    {
        if (this->psm_state == PLD_STATE::PRECISION_LAND_STATE_DESCEND_FINAL) 
        {
            gcs().send_text(MAV_SEVERITY_INFO, "Landed on target");
            this->psm_state = PLD_STATE::PRECISION_LAND_STATE_SUCCESS;
        } 
        else 
        {
            if (this->psm_state != PLD_STATE::PRECISION_LAND_STATE_NOT_ACTIVE && this->psm_state != PLD_STATE::PRECISION_LAND_STATE_SUCCESS) 
            {
                this->psm_state = PLD_STATE::PRECISION_LAND_STATE_NOT_ACTIVE;
                gcs().send_text(MAV_SEVERITY_INFO, "Unexpected Land");
            }
        }
    }

    switch (this->psm_state) 
    {
        case PLD_STATE::PRECISION_LAND_STATE_NOT_ACTIVE:
        case PLD_STATE::PRECISION_LAND_STATE_SUCCESS:
            break;
        case PLD_STATE::PRECISION_LAND_STATE_SEARCH:
            this->search_run();
            break;
        case PLD_STATE::PRECISION_LAND_STATE_DESCEND:
            this->descend_run();
            break;
        case PLD_STATE::PRECISION_LAND_STATE_DESCEND_FINAL:
            this->descend_final_run();
            break;
        case PLD_STATE::PRECISION_LAND_STATE_RETRY:
            this->retry_run();
            break;
        case PLD_STATE::PRECISION_LAND_STATE_FAILED:
            this->failed_run();
            break;
    }
}

/*
    Start to search
*/
void AC_PrecLand_SM::search_start()
{
    this->psm_state = PLD_STATE::PRECISION_LAND_STATE_SEARCH;

    this->psm_wp_nav.set_radius(WP_RADIUS);
    this->psm_wp_nav.use_rangefinder(false);
    this->psm_wp_nav.wp_and_spline_init();

    if (this->psm_precland.target_acquired() == true) 
    {
        Vector2f target_pos;
        if (this->psm_precland.get_target_position_cm(target_pos) == true) 
        {
            this->psm_wp_nav.set_wp_destination(Vector3f(target_pos.x, target_pos.y, static_cast<float>(_search_start_alt.get() - 500)), false);
            this->psm_wp_nav.get_wp_destination(this->search_target);
        }
    } 
    else 
    {
        this->search_target = this->init_location;
        this->search_target.set_alt_cm((_search_start_alt.get() - 500), Location::AltFrame::ABOVE_HOME);
        this->psm_wp_nav.set_wp_destination(this->search_target);
    }

    this->psm_first_pass = true;
    this->psm_doing_first_pass = false;
    this->psm_wp_nav_is_active = true;

    gcs().send_text(MAV_SEVERITY_INFO, "search_start");
}

/*
    Run to search
*/
void AC_PrecLand_SM::search_run()
{
    if (this->alt_above_ground_cm >= _search_start_alt.get() || this->psm_doing_first_pass) 
    {
        if (this->psm_wp_nav.reached_wp_destination() == true) 
        {
            this->psm_doing_first_pass = false;
        }
    }
    else
    {
        // if we are under the stop alt, we get up
        if (this->alt_above_ground_cm < _search_stop_alt.get()) 
        {
            // get up to stop alt This unsure we always do the same patern and garanty the same descend phase
            if (this->psm_first_pass) 
            {
                this->search_target.set_alt_cm((_search_start_alt.get() - 500), Location::AltFrame::ABOVE_HOME);
                this->psm_wp_nav.set_wp_destination(search_target);
                this->psm_doing_first_pass = true;
                this->psm_first_pass = false;
            } 
            else 
            {
                // inform user
                this->psm_state = PLD_STATE::PRECISION_LAND_STATE_FAILED;
                this->failed_start();
            }
        }
        else
        {
            this->psm_first_pass = false;
            this->psm_doing_first_pass = false;

            // if we lock the target, go on the target
            if (this->psm_precland.target_acquired() == true) 
            {
                if (this->alt_above_ground_cm > _search_stop_alt.get() + 1000) 
                {
                    Vector2f target_pos;
                    if (this->psm_precland.get_target_position_cm(target_pos)) 
                    {
                        this->psm_wp_nav.set_wp_destination(Vector3f(target_pos.x, target_pos.y, static_cast<float>(_search_stop_alt.get())), false);
                    }
                } 
                else 
                {
                     this->descend_start();
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

    this->psm_state = PLD_STATE::PRECISION_LAND_STATE_DESCEND;

    avg_filter.reset();

    this->psm_avg_pos_error = 0.0f;

    this->psm_wp_nav.reset_radius();
    this->psm_wp_nav.reset_rangefinder_use();
    this->psm_loiter_nav.init_target();
    this->psm_pos_control.init_xy_controller();

    this->last_target_time = AP_HAL::millis();
    this->psm_wp_nav_is_active = false;

    this->max_descend_speed = MIN(-this->psm_pos_control.get_max_speed_down(), _descend_speed_max.get());
}

/*
    Calculate the slow-down rate
*/
void AC_PrecLand_SM::calculate_slowdown()
{
    // we get back using loiter_nav for tighter locking on target. We will just slowdown the descend according to the pos error
    // recenter and descend
    if (this->psm_precland.target_acquired() == true) 
    {
        this->last_target_time = AP_HAL::millis();
        this->psm_avg_pos_error = avg_filter.apply(this->psm_pos_control.get_horizontal_error());
    } 
    else 
    {
        if (AP_HAL::millis() - this->last_target_time > 500) 
        {  
            //timeout of 0.5sec for launching a retry
            // inform user
            gcs().send_text(MAV_SEVERITY_INFO, "Lost target for more than 0.5s");
            this->psm_state = PLD_STATE::PRECISION_LAND_STATE_FAILED;
            this->failed_start();

            return;
        }
    }

    float land_slowdown = 0.0f;
    if (this->psm_avg_pos_error > _acceptable_error_cm.get()) 
    {
        // slowdown max at 2 * _acceptable_error_cm
        land_slowdown = MAX(0.0f, this->psm_avg_pos_error * (this->max_descend_speed / (_acceptable_error_cm.get() * 2.0f)));
    }

    this->precland_sm_climb_rate = MIN(-MIN_DESCENT_SPEED, -this->max_descend_speed + land_slowdown);
}

/*
    Descending
*/
void AC_PrecLand_SM::descend_run()
{
    // Calculate the slow-down rate
    this->calculate_slowdown();

    // if the altitude is within "descend_stop_alt", then start to descend
    if (this->alt_above_ground_cm <= _descend_stop_alt.get()) 
    {
        // If the error is acceptable, then go to the final stage.
        if (this->psm_avg_pos_error <= _acceptable_error_cm.get()) 
        {
            this->descend_final_start();
        } 
        else 
        {
            // try to give 1s on stationnary to alignate better
            if (this->descend_timer == 0) 
            {
                this->descend_timer = AP_HAL::millis();
            }

            if (AP_HAL::millis() - this->descend_timer > 1000) 
            {
                // inform user
                gcs().send_text(MAV_SEVERITY_INFO, "Failed to lock target accuratly");
                this->psm_state = PLD_STATE::PRECISION_LAND_STATE_FAILED;

                this->failed_start();
            } 
            else 
            {
                this->precland_sm_climb_rate = 0.1f; // slightly positive to ensure a little thrust up to stop inertia !
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

    this->psm_state = PLD_STATE::PRECISION_LAND_STATE_DESCEND_FINAL;
    this->max_descend_speed = MIN(-this->psm_pos_control.get_max_speed_down(), _descend_speed_max.get()) * 0.5f;
}

/*
    Run "Final Descending Stage"
*/
void AC_PrecLand_SM::descend_final_run()
{
    // Check if it is final landing stage
    if (this->alt_above_ground_cm <= (_descend_stop_alt.get() * 0.25f) || this->alt_above_ground_cm <= _start_land_alt_cm.get()) 
    {
        this->psm_is_final_landing = true;  // unsure smooth landing by the land controler
    }

    // Calculate the slow-down rate
    this->calculate_slowdown();
}

/*
    Start "Failure Stage"
*/
void AC_PrecLand_SM::failed_start()
{
    bool allow_retry = ((psm_is_emergency == true && (_emergency_behavior & EM_BHV_DISABLE_RETRY) != 0) == false);

    gcs().send_text(MAV_SEVERITY_INFO, "failed_start");

    this->psm_is_final_landing = false;

    if (allow_retry == true && this->psm_retry_count < _retry_max.get()) 
    {
        gcs().send_text(MAV_SEVERITY_INFO, "Retry %d", psm_retry_count);

        this->psm_retry_count++;
        this->retry_start();
        return;
    }
    else
    {
        this->psm_state = PLD_STATE::PRECISION_LAND_STATE_FAILED;

        this->search_target = this->init_location;
        this->search_target.set_alt_cm(_failure_alt.get(), Location::AltFrame::ABOVE_HOME);
        this->search_target.offset(_failure_posn.get(), 0);  // set safe land at _failure_posn of north of the beacon.

        this->psm_wp_nav.set_wp_destination(this->search_target);
        this->psm_wp_nav_is_active = true;

        gcs().send_text(MAV_SEVERITY_INFO, "Failed to retry going to safe position");
    }
}

/*
    Run "Failure Stage"
*/
void AC_PrecLand_SM::failed_run()
{
    // case retry count failure, land at X m from the home without precland
    if (this->psm_wp_nav.reached_wp_destination() == true) 
    {
        // else do normal land
        this->psm_in_failure = true;
        this->psm_wp_nav_is_active = false;
        this->psm_state = PLD_STATE::PRECISION_LAND_STATE_NOT_ACTIVE;
    }
}

/*
    Start "Retry"
*/
void AC_PrecLand_SM::retry_start()
{
    this->psm_state = PLD_STATE::PRECISION_LAND_STATE_RETRY;

    this->psm_wp_nav.use_rangefinder(false);
    this->psm_wp_nav.wp_and_spline_init();

    // if the target is acquired, then move the target point
    if (this->psm_precland.target_acquired() == true) 
    {
        Vector2f target_pos;
        if (this->psm_precland.get_target_position_cm(target_pos)) 
        {
            this->psm_wp_nav.set_wp_destination(Vector3f(target_pos.x, target_pos.y, static_cast<float>(_search_start_alt.get())), false);
            this->psm_wp_nav.get_wp_destination(this->search_target);
        }
    } 
    else    // if the target is not acquired, then move the initial point
    {
        this->search_target = this->init_location;
        this->search_target.set_alt_cm(static_cast<int32_t>(_search_start_alt), Location::AltFrame::ABOVE_HOME);
        this->psm_wp_nav.set_wp_destination(this->search_target);
    }

    this->psm_wp_nav_is_active = true;
    gcs().send_text(MAV_SEVERITY_INFO, "retry_start");
}

/*
    Run "Retry"
*/
void AC_PrecLand_SM::retry_run()
{
    if (this->psm_wp_nav.reached_wp_destination() == true) 
    {
        this->search_start();
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
            .state                 =  static_cast<uint8_t>(this->psm_state),
            .alt_above_ground      =  this->alt_above_ground_cm,
            .last_alt_above_ground =  this->last_alt_above_ground_cm,
            .climb_rate            =  this->precland_sm_climb_rate,
            .avg_pos_error         =  this->psm_avg_pos_error,
            .retry                 =  this->psm_retry_count,
    };

    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

