#pragma once

#include "AP_Compass_config.h"

#include <inttypes.h>

#include <AP_Common/AP_Common.h>
#include <AP_Declination/AP_Declination.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_MSP/msp.h>
#include <AP_ExternalAHRS/AP_ExternalAHRS.h>

#include "AP_Compass_Backend.h"
#include "Compass_PerMotor.h"
#include <AP_Common/TSIndex.h>

// motor compensation types (for use with motor_comp_enabled)
#define AP_COMPASS_MOT_COMP_DISABLED    0x00
#define AP_COMPASS_MOT_COMP_THROTTLE    0x01
#define AP_COMPASS_MOT_COMP_CURRENT     0x02
#define AP_COMPASS_MOT_COMP_PER_MOTOR   0x03

#ifndef MAG_BOARD_ORIENTATION
#define MAG_BOARD_ORIENTATION ROTATION_NONE
#endif

#ifndef COMPASS_MOT_ENABLED
#define COMPASS_MOT_ENABLED 1
#endif
#ifndef COMPASS_LEARN_ENABLED
#define COMPASS_LEARN_ENABLED AP_COMPASS_CALIBRATION_FIXED_YAW_ENABLED
#endif

// define default compass calibration fitness and consistency checks
#define AP_COMPASS_CALIBRATION_FITNESS_DEFAULT 16.0f
#define AP_COMPASS_MAX_XYZ_ANG_DIFF radians(90.0f)
#define AP_COMPASS_MAX_XY_ANG_DIFF radians(60.0f)
#define AP_COMPASS_MAX_XY_LENGTH_DIFF 200.0f

/**
   maximum number of compass instances available on this platform. If more
   than 1 then redundant sensors may be available
 */
#ifndef HAL_BUILD_AP_PERIPH
#ifndef HAL_COMPASS_MAX_SENSORS
#define HAL_COMPASS_MAX_SENSORS 3
#endif
#if HAL_COMPASS_MAX_SENSORS > 1
#define COMPASS_MAX_UNREG_DEV 5
#else
#define COMPASS_MAX_UNREG_DEV 0
#endif
#else
#ifndef HAL_COMPASS_MAX_SENSORS
#define HAL_COMPASS_MAX_SENSORS 1
#endif
#define COMPASS_MAX_UNREG_DEV 0
#endif

#define COMPASS_MAX_INSTANCES HAL_COMPASS_MAX_SENSORS
#define COMPASS_MAX_BACKEND   HAL_COMPASS_MAX_SENSORS

#define MAX_CONNECTED_MAGS (COMPASS_MAX_UNREG_DEV+COMPASS_MAX_INSTANCES)

#include "CompassCalibrator.h"

class CompassLearn;

class Compass
{
friend class AP_Compass_Backend;
public:
    Compass();

    /* Do not allow copies */
    CLASS_NO_COPY(Compass);

    // get singleton instance
    static Compass *get_singleton() {
        return _singleton;
    }

    friend class CompassLearn;

    /// Initialize the compass device.
    ///
    /// @returns    True if the compass was initialized OK, false if it was not
    ///             found or is not functioning.
    ///
    __INITFUNC__ void init();

    /// Read the compass and update the mag_ variables.
    ///
    bool read();

    // available returns true if the compass is both enabled and has
    // been initialised
    bool available() const { return _enabled && init_done; }

    /// Calculate the tilt-compensated heading_ variables.
    ///
    /// @param dcm_matrix			The current orientation rotation matrix
    ///
    /// @returns heading in radians
    ///
    float calculate_heading(const Matrix3f &dcm_matrix) const {
        return calculate_heading(dcm_matrix, _first_usable);
    }
    float calculate_heading(const Matrix3f &dcm_matrix, uint8_t i) const;

    /// Sets offset x/y/z values.
    ///
    /// @param  i                   compass instance
    /// @param  offsets             Offsets to the raw mag_ values in milligauss.
    ///
    void set_offsets(uint8_t i, const Vector3f &offsets);

    /// Sets and saves the compass offset x/y/z values.
    ///
    /// @param  i                   compass instance
    /// @param  offsets             Offsets to the raw mag_ values in milligauss.
    ///
    void set_and_save_offsets(uint8_t i, const Vector3f &offsets);
#if AP_COMPASS_DIAGONALS_ENABLED
    void set_and_save_diagonals(uint8_t i, const Vector3f &diagonals);
    void set_and_save_offdiagonals(uint8_t i, const Vector3f &diagonals);
#endif
    void set_and_save_scale_factor(uint8_t i, float scale_factor);
    void set_and_save_orientation(uint8_t i, Rotation orientation);

    /// Saves the current offset x/y/z values for one or all compasses
    ///
    /// @param  i                   compass instance
    ///
    /// This should be invoked periodically to save the offset values maintained by
    /// ::learn_offsets.
    ///
    void save_offsets(uint8_t i);
    void save_offsets(void);

    // return the number of compass instances
    uint8_t get_count(void) const { return _compass_count; }

    // return the number of enabled sensors
    uint8_t get_num_enabled(void) const;
    
    /// Return the current field as a Vector3f in milligauss
    const Vector3f &get_field(uint8_t i) const { return _get_state(Priority(i)).field; }
    const Vector3f &get_field(void) const { return get_field(_first_usable); }

    /// Return true if we have set a scale factor for a compass
    bool have_scale_factor(uint8_t i) const;

#if COMPASS_MOT_ENABLED
    // per-motor calibration access
    void per_motor_calibration_start(void) {
        _per_motor.calibration_start();
    }
    void per_motor_calibration_update(void) {
        _per_motor.calibration_update();
    }
    void per_motor_calibration_end(void) {
        _per_motor.calibration_end();
    }
#endif

#if COMPASS_CAL_ENABLED
    // compass calibrator interface
    void cal_update();

    // start_calibration_all will only return false if there are no
    // compasses to calibrate.
    bool start_calibration_all(bool retry=false, bool autosave=false, float delay_sec=0.0f, bool autoreboot = false);

    void cancel_calibration_all();

    bool compass_cal_requires_reboot() const { return _cal_requires_reboot; }
    bool is_calibrating() const;

#if HAL_MAVLINK_BINDINGS_ENABLED
    /*
      handle an incoming MAG_CAL command
    */
    MAV_RESULT handle_mag_cal_command(const mavlink_command_int_t &packet);

    bool send_mag_cal_progress(const class GCS_MAVLINK& link);
    bool send_mag_cal_report(const class GCS_MAVLINK& link);
#endif  // HAL_MAVLINK_BINDINGS_ENABLED
#endif  // COMPASS_CAL_ENABLED

    // indicate which bit in LOG_BITMASK indicates we should log compass readings
    void set_log_bit(uint32_t log_bit) { _log_bit = log_bit; }

    // check if the compasses are pointing in the same direction
    bool consistent() const;

    /// Return the health of a compass
    bool healthy(uint8_t i) const;
    bool healthy(void) const { return healthy(_first_usable); }
    uint8_t get_healthy_mask() const;

    /// Returns the current offset values
    ///
    /// @returns                    The current compass offsets in milligauss.
    ///
    const Vector3f &get_offsets(uint8_t i) const { return _get_state(Priority(i)).offset; }
    const Vector3f &get_offsets(void) const { return get_offsets(_first_usable); }

#if AP_COMPASS_DIAGONALS_ENABLED
    const Vector3f &get_diagonals(uint8_t i) const { return _get_state(Priority(i)).diagonals; }
    const Vector3f &get_diagonals(void) const { return get_diagonals(_first_usable); }

    const Vector3f &get_offdiagonals(uint8_t i) const { return _get_state(Priority(i)).offdiagonals; }
    const Vector3f &get_offdiagonals(void) const { return get_offdiagonals(_first_usable); }
#endif  // AP_COMPASS_DIAGONALS_ENABLED

    // learn offsets accessor
    bool learn_offsets_enabled() const { return _learn == LearnType::INFLIGHT; }

    /// return true if the compass should be used for yaw calculations
    bool use_for_yaw(uint8_t i) const;
    bool use_for_yaw(void) const;

    /// Sets the local magnetic field declination.
    ///
    /// @param  radians             Local field declination.
    /// @param save_to_eeprom       true to save to eeprom (false saves only to memory)
    ///
    void set_declination(float radians, bool save_to_eeprom = true);
    float get_declination() const;

    bool auto_declination_enabled() const { return _auto_declination != 0; }

    // set overall board orientation
    void set_board_orientation(enum Rotation orientation) {
        _board_orientation = orientation;
    }

    // get overall board orientation
    enum Rotation get_board_orientation(void) const {
        return _board_orientation;
    }

    /// Set the motor compensation type
    ///
    /// @param  comp_type           0 = disabled, 1 = enabled use throttle, 2 = enabled use current
    ///
    void motor_compensation_type(const uint8_t comp_type);

    /// get the motor compensation value.
    uint8_t get_motor_compensation_type() const {
        return _motor_comp_type;
    }

    /// Set the motor compensation factor x/y/z values.
    ///
    /// @param  i                   instance of compass
    /// @param  offsets             Offsets multiplied by the throttle value and added to the raw mag_ values.
    ///
    void set_motor_compensation(uint8_t i, const Vector3f &motor_comp_factor);

    /// get motor compensation factors as a vector
    const Vector3f& get_motor_compensation(uint8_t i) const { return _get_state(Priority(i)).motor_compensation; }
    const Vector3f& get_motor_compensation(void) const { return get_motor_compensation(_first_usable); }

    /// Saves the current motor compensation x/y/z values.
    ///
    /// This should be invoked periodically to save the offset values calculated by the motor compensation auto learning
    ///
    void save_motor_compensation();

    /// Returns the current motor compensation offset values
    ///
    /// @returns                    The current compass offsets in milligauss.
    ///
    const Vector3f &get_motor_offsets(uint8_t i) const { return _get_state(Priority(i)).motor_offset; }
    const Vector3f &get_motor_offsets(void) const { return get_motor_offsets(_first_usable); }

    /// Set the throttle as a percentage from 0.0 to 1.0
    /// @param thr_pct              throttle expressed as a percentage from 0 to 1.0
    void set_throttle(float thr_pct) {
        if (_motor_comp_type == AP_COMPASS_MOT_COMP_THROTTLE) {
            _thr = thr_pct;
        }
    }

#if COMPASS_MOT_ENABLED
    /// Set the battery voltage for per-motor compensation
    void set_voltage(float voltage) {
        _per_motor.set_voltage(voltage);
    }
#endif
    
    /// Returns True if the compasses have been configured (i.e. offsets saved)
    ///
    /// @returns                    True if compass has been configured
    ///
    bool configured(uint8_t i);
    bool configured(char *failure_msg, uint8_t failure_msg_len);

    // return last update time in microseconds
    uint32_t last_update_usec(void) const { return last_update_usec(_first_usable); }
    uint32_t last_update_usec(uint8_t i) const { return _get_state(Priority(i)).last_update_usec; }

    uint32_t last_update_ms(void) const { return last_update_ms(_first_usable); }
    uint32_t last_update_ms(uint8_t i) const { return _get_state(Priority(i)).last_update_ms; }

    static const struct AP_Param::GroupInfo var_info[];

    enum class LearnType {
        NONE          = 0,
        // INTERNAL   = 1,
        COPY_FROM_EKF = 2,
        INFLIGHT      = 3,
    };

    // return the chosen learning type
    LearnType get_learn_type(void) const {
        return (LearnType)_learn.get();
    }

    // set the learning type
    void set_learn_type(LearnType type, bool save) {
        if (save) {
            _learn.set_and_save(type);
        } else {
            _learn.set(type);
        }
    }
    
    // return maximum allowed compass offsets
    uint16_t get_offsets_max(void) const {
        return (uint16_t)_offset_max.get();
    }

    uint8_t get_filter_range() const { return uint8_t(_filter_range.get()); }

#if AP_COMPASS_CALIBRATION_FIXED_YAW_ENABLED
    /*
      fast compass calibration given vehicle position and yaw
     */
    bool mag_cal_fixed_yaw(float yaw_deg, uint8_t compass_mask,
                           float lat_deg, float lon_deg,
                           bool force_use=false);
#endif

#if AP_COMPASS_MSP_ENABLED
    void handle_msp(const MSP::msp_compass_data_message_t &pkt);
#endif

#if AP_COMPASS_EXTERNALAHRS_ENABLED
    void handle_external(const AP_ExternalAHRS::mag_data_message_t &pkt);
#endif

    // force save of current calibration as valid
    void force_save_calibration(void);

    // get the first compass marked for use by COMPASSx_USE
    uint8_t get_first_usable(void) const { return _first_usable; }

private:
    static Compass *_singleton;

    // Use Priority and StateIndex typesafe index types
    // to distinguish between two different type of indexing
    // We use StateIndex for access by Backend
    // and Priority for access by Frontend
    DECLARE_TYPESAFE_INDEX(Priority, uint8_t);
    DECLARE_TYPESAFE_INDEX(StateIndex, uint8_t);

    /// Register a new compas driver, allocating an instance number
    ///
    /// @param  dev_id                   Dev ID of compass to register against
    ///
    /// @return instance number saved against the dev id or first available empty instance number
    bool register_compass(int32_t dev_id, uint8_t& instance);

    // load backend drivers
    bool _add_backend(AP_Compass_Backend *backend);
    void _probe_external_i2c_compasses(void);
    void _detect_backends(void);
    void probe_i2c_spi_compasses(void);
#if AP_COMPASS_DRONECAN_ENABLED
    void probe_dronecan_compasses(void);
#endif

#if COMPASS_CAL_ENABLED
    // compass cal
    void _update_calibration_trampoline();
    bool _accept_calibration(uint8_t i);
    bool _accept_calibration_mask(uint8_t mask);
    void _cancel_calibration(uint8_t i);
    void _cancel_calibration_mask(uint8_t mask);
    uint8_t _get_cal_mask();
    bool _start_calibration(uint8_t i, bool retry=false, float delay_sec=0.0f);
    bool _start_calibration_mask(uint8_t mask, bool retry=false, bool autosave=false, float delay_sec=0.0f, bool autoreboot=false);
    bool _auto_reboot() const { return _compass_cal_autoreboot; }
#if HAL_MAVLINK_BINDINGS_ENABLED
    Priority next_cal_progress_idx[MAVLINK_COMM_NUM_BUFFERS];
    Priority next_cal_report_idx[MAVLINK_COMM_NUM_BUFFERS];
#endif
#endif  // COMPASS_CAL_ENABLED

    // see if we already have probed a i2c driver by bus number and address
    bool _have_i2c_driver(uint8_t bus_num, uint8_t address) const;

#if AP_COMPASS_CALIBRATION_FIXED_YAW_ENABLED
    /*
      get mag field with the effects of offsets, diagonals and
      off-diagonals removed
    */
    bool get_uncorrected_field(uint8_t instance, Vector3f &field) const;
#endif

#if COMPASS_CAL_ENABLED
    //keep track of which calibrators have been saved
    RestrictIDTypeArray<bool, COMPASS_MAX_INSTANCES, Priority> _cal_saved;
    bool _cal_autosave;

    //autoreboot after compass calibration
    bool _compass_cal_autoreboot;
    bool _cal_requires_reboot;
    bool _cal_has_run;
#endif  // COMPASS_CAL_ENABLED

    // enum of drivers for COMPASS_DISBLMSK
    enum DriverType {
#if AP_COMPASS_HMC5843_ENABLED
        DRIVER_HMC5843  =0,
#endif
#if AP_COMPASS_LSM303D_ENABLED
        DRIVER_LSM303D  =1,
#endif
#if AP_COMPASS_AK8963_ENABLED
        DRIVER_AK8963   =2,
#endif
#if AP_COMPASS_BMM150_ENABLED
        DRIVER_BMM150   =3,
#endif
#if AP_COMPASS_LSM9DS1_ENABLED
        DRIVER_LSM9DS1  =4,
#endif
#if AP_COMPASS_LIS3MDL_ENABLED
        DRIVER_LIS3MDL  =5,
#endif
#if AP_COMPASS_AK09916_ENABLED
        DRIVER_AK09916  =6,
#endif
#if AP_COMPASS_IST8310_ENABLED
        DRIVER_IST8310  =7,
#endif
#if AP_COMPASS_ICM20948_ENABLED
        DRIVER_ICM20948 =8,
#endif
#if AP_COMPASS_MMC3416_ENABLED
        DRIVER_MMC3416  =9,
#endif
#if AP_COMPASS_DRONECAN_ENABLED
        DRIVER_UAVCAN   =11,
#endif
#if AP_COMPASS_QMC5883L_ENABLED
        DRIVER_QMC5883L =12,
#endif
#if AP_COMPASS_SITL_ENABLED
        DRIVER_SITL     =13,
#endif
#if AP_COMPASS_MAG3110_ENABLED
        DRIVER_MAG3110  =14,
#endif
#if AP_COMPASS_IST8308_ENABLED
        DRIVER_IST8308  =15,
#endif
#if AP_COMPASS_RM3100_ENABLED
		DRIVER_RM3100   =16,
#endif
#if AP_COMPASS_MSP_ENABLED
        DRIVER_MSP      =17,
#endif
#if AP_COMPASS_EXTERNALAHRS_ENABLED
        DRIVER_EXTERNALAHRS   =18,
#endif
#if AP_COMPASS_MMC5XX3_ENABLED
        DRIVER_MMC5XX3  =19,
#endif
#if AP_COMPASS_QMC5883P_ENABLED
        DRIVER_QMC5883P =20,
#endif
#if AP_COMPASS_BMM350_ENABLED
        DRIVER_BMM350   =21,
#endif
#if AP_COMPASS_IIS2MDC_ENABLED
        DRIVER_IIS2MDC  =22,
#endif
#if AP_COMPASS_ORIENTUS_ENABLED
        DRIVER_ORIENTUS  =23,
#endif
};

    bool _driver_enabled(enum DriverType driver_type);
    
    // backend objects
    AP_Compass_Backend *_backends[COMPASS_MAX_BACKEND];
    uint8_t     _backend_count;

    // whether to enable the compass drivers at all
    AP_Int8     _enabled;

    // number of registered compasses.
    uint8_t     _compass_count;

    // number of unregistered compasses.
    uint8_t     _unreg_compass_count;

    // settable parameters
    AP_Enum<LearnType> _learn;

    // board orientation from AHRS
    enum Rotation _board_orientation = ROTATION_NONE;

    // declination in radians
    AP_Float    _declination;

    // enable automatic declination code
    AP_Int8     _auto_declination;

    // stores which bit is used to indicate we should log compass readings
    uint32_t _log_bit = -1;

    // motor compensation type
    // 0 = disabled, 1 = enabled for throttle, 2 = enabled for current
    AP_Int8     _motor_comp_type;

    // automatic compass orientation on calibration
    AP_Int8     _rotate_auto;

    // throttle expressed as a percentage from 0 ~ 1.0, used for motor compensation
    float       _thr;

    struct mag_state {
        AP_Int8     external;
        bool        healthy;
        bool        registered;
        Compass::Priority priority;
        AP_Int8     orientation;
        AP_Vector3f offset;
#if AP_COMPASS_DIAGONALS_ENABLED
        AP_Vector3f diagonals;
        AP_Vector3f offdiagonals;
#endif
        AP_Float    scale_factor;

        // device id detected at init.
        // saved to eeprom when offsets are saved allowing ram &
        // eeprom values to be compared as consistency check
        AP_Int32    dev_id;
        // Initialised when compass is detected
        int32_t detected_dev_id;
        // Initialised at boot from saved devid
        int32_t expected_dev_id;

        // factors multiplied by throttle and added to compass outputs
        AP_Vector3f motor_compensation;

        // latest compensation added to compass
        Vector3f    motor_offset;

        // corrected magnetic field strength
        Vector3f    field;

        // when we last got data
        uint32_t    last_update_ms;
        uint32_t    last_update_usec;

        // board specific orientation
        enum Rotation rotation;

        // accumulated samples, protected by _sem, used by AP_Compass_Backend
        Vector3f accum;
        uint32_t accum_count;
        // We only copy persistent params
        void copy_from(const mag_state& state);
    };

    //Create an Array of mag_state to be accessible by StateIndex only
    RestrictIDTypeArray<mag_state, COMPASS_MAX_INSTANCES+1, StateIndex> _state;

    //Convert Priority to StateIndex
    StateIndex _get_state_id(Priority priority) const;
    //Get State Struct by Priority
    const struct mag_state& _get_state(Priority priority) const { return _state[_get_state_id(priority)]; }
    //Convert StateIndex to Priority
    Priority _get_priority(StateIndex state_id) { return _state[state_id].priority; }
    //Method to detect compass beyond initialisation stage
    void _detect_runtime(void);
    // This method reorganises devid list to match
    // priority list, only call before detection at boot
#if COMPASS_MAX_INSTANCES > 1
    void _reorder_compass_params();
#endif
    // Update Priority List for Mags, by default, we just
    // load them as they come up the first time
    Priority _update_priority_list(int32_t dev_id);
    
    // method to check if the mag with the devid 
    // is a replacement mag
    bool is_replacement_mag(uint32_t dev_id);

    //remove the devid from unreg compass list
    void remove_unreg_dev_id(uint32_t devid);

    void _reset_compass_id();
    //Create Arrays to be accessible by Priority only
    RestrictIDTypeArray<AP_Int8, COMPASS_MAX_INSTANCES, Priority> _use_for_yaw;
#if COMPASS_MAX_INSTANCES > 1
    RestrictIDTypeArray<AP_Int32, COMPASS_MAX_INSTANCES, Priority> _priority_did_stored_list;
    RestrictIDTypeArray<int32_t, COMPASS_MAX_INSTANCES, Priority> _priority_did_list;
#endif

    AP_Int16 _offset_max;

    // bitmask of options
    enum class Option : uint16_t {
        CAL_REQUIRE_GPS = (1U<<0),
        ALLOW_DRONECAN_AUTO_REPLACEMENT = (1U<<1),
    };
    bool option_set(Option opt) const { return (_options.get() & uint16_t(opt)) != 0; }
    AP_Int16 _options;

#if COMPASS_CAL_ENABLED
    RestrictIDTypeArray<CompassCalibrator*, COMPASS_MAX_INSTANCES, Priority> _calibrator;
#endif

#if COMPASS_MOT_ENABLED
    // per-motor compass compensation
    Compass_PerMotor _per_motor{*this};
#endif
    
    AP_Float _calibration_threshold;

    // mask of driver types to not load. Bit positions match DEVTYPE_ in backend
    AP_Int32 _driver_type_mask;

#if COMPASS_MAX_UNREG_DEV
    // Put extra dev ids detected
    AP_Int32 extra_dev_id[COMPASS_MAX_UNREG_DEV];
    uint32_t _previously_unreg_mag[COMPASS_MAX_UNREG_DEV];
#endif

    AP_Int8 _filter_range;

    CompassLearn *learn;
    bool learn_allocated;

    /// Sets the initial location used to get declination
    ///
    /// @param  latitude             GPS Latitude.
    /// @param  longitude            GPS Longitude.
    ///
    void try_set_initial_location();
    bool _initial_location_set;

    bool _cal_thread_started;

#if AP_COMPASS_MSP_ENABLED
    uint8_t msp_instance_mask;
#endif
    bool init_done;

    bool suppress_devid_save;

    uint8_t _first_usable; // first compass usable based on COMPASSx_USE param
};

namespace AP {
    Compass &compass();
};
