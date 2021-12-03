//
// Created by khancyr on 11/02/2021.
//

#pragma once
#include <AP_Param/AP_Param.h>
#include <AC_WPNav/AC_WPNav.h>
#include <AC_AttitudeControl/AC_PosControl.h>
#include <AC_WPNav/AC_Loiter.h>
#include <AC_PrecLand/AC_PrecLand.h>
#include <Filter/AverageFilter.h>

class AC_PrecLand_SM {
public:
    AC_PrecLand_SM(AC_PosControl &pos_control, AC_Loiter &loiter_nav, AC_WPNav &wp_nav, AC_PrecLand &precland);
    void reset();
    void init();
    void run();
    bool is_enable() const { return _enabled; }
    bool in_failure() const { return psm_in_failure; }
    bool get_glitch_state() const { return is_glitching; }
    bool is_final_landing() const { return psm_is_final_landing; }
    void set_emergency_flag(bool flag) { psm_is_emergency = flag; }
    void set_glitch(bool isglitching) { is_glitching = isglitching; }
    int32_t get_last_alt_above_ground_cm() const { return last_alt_above_ground_cm; }
    bool is_active() const { return psm_initialised && psm_state != PLD_STATE::PRECISION_LAND_STATE_NOT_ACTIVE; }
    bool wp_nav_is_active() const { return psm_wp_nav_is_active; }
    float get_desired_cmb_rate() const { return precland_sm_climb_rate; }

    void set_alt_above_ground_cm(int32_t altaboveground_cm) 
    {
        if (!is_glitching) 
        {
            last_alt_above_ground_cm = alt_above_ground_cm;
        }
        alt_above_ground_cm = altaboveground_cm;
    }
    void write_log();
    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];

private:
    enum class PLD_STATE: uint8_t {
        PRECISION_LAND_STATE_NOT_ACTIVE = 0,
        PRECISION_LAND_STATE_SEARCH,
        PRECISION_LAND_STATE_DESCEND,
        PRECISION_LAND_STATE_DESCEND_FINAL,
        PRECISION_LAND_STATE_RETRY,
        PRECISION_LAND_STATE_SUCCESS,
        PRECISION_LAND_STATE_FAILED,
    };
    /* Parameters */
    AP_Int8 _enabled;
    AP_Int8 _retry_max;
    AP_Int16 _emergency_behavior;
    AP_Int16 _descend_speed_max;
    AP_Int16 _failure_alt;
    AP_Int32 _search_start_alt; // in cm
    AP_Int32 _search_stop_alt;
    AP_Float _failure_posn;
    AP_Float _acceptable_error_cm;
    AP_Int32 _descend_stop_alt; //cm
    AP_Int32 _start_land_alt_cm;

    /* Local Variables */
    static constexpr uint32_t WP_RADIUS = 50; // cm
    static constexpr uint8_t AVG_FILTER_SIZE = 100;
    static constexpr float MIN_DESCENT_SPEED = 10.0f;

    bool is_glitching;
    bool psm_initialised;
    bool psm_wp_nav_is_active;
    bool psm_is_emergency;
    bool psm_first_pass;
    bool psm_doing_first_pass;
    bool psm_in_failure;
    bool psm_is_final_landing;
    float psm_avg_pos_error; //cm
    float max_descend_speed;
    float precland_sm_climb_rate;
    uint8_t psm_retry_count;
    int32_t alt_above_ground_cm;
    int32_t last_alt_above_ground_cm;
    uint32_t last_target_time;
    uint32_t descend_timer;
    PLD_STATE psm_state;
    AverageFilter<float, float, AVG_FILTER_SIZE> avg_filter;
    Location search_target;
    Location init_location;

    AC_PosControl &psm_pos_control;
    AC_Loiter &psm_loiter_nav;
    AC_WPNav &psm_wp_nav;
    AC_PrecLand &psm_precland;

    // for EM_BHV parameter
    static constexpr uint8_t EM_BHV_NO_CHANGES = (1<<0);
    static constexpr uint8_t EM_BHV_DISABLE_SM = (1<<1);
    static constexpr uint8_t EM_BHV_DISABLE_RETRY = (1<<2);

    /*******************/
    void search_start();
    void search_run();
    void calculate_slowdown();
    void descend_start();
    void descend_run();

    void set_acceptable_error(float error);
    void descend_final_start();
    void descend_final_run();

    void failed_start();
    void failed_run();
    
    void retry_start();
    void retry_run();
};

