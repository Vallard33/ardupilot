/******************************************************************************/
/*                                                                            */
/*                Advanced Navigation Packet Protocol Library                 */
/*                C Language Dynamic Orientus SDK, Version 7.3                */
/*                    Copyright 2024, Advanced Navigation                     */
/*                                                                            */
/******************************************************************************/
/*
 * Copyright (C) 2024 Advanced Navigation
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#ifndef INS_PACKETS_H_
#define INS_PACKETS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

#define MAXIMUM_PACKET_PERIODS 50

#define START_SYSTEM_PACKETS 0
#define START_STATE_PACKETS 20
#define START_CONFIGURATION_PACKETS 180

typedef enum
{
	packet_id_acknowledge,
	packet_id_request,
	packet_id_boot_mode,
	packet_id_device_information,
	packet_id_restore_factory_settings,
	packet_id_reset,
	packet_id_6_reserved,
	packet_id_file_transfer_request,
	packet_id_file_transfer_acknowledge,
	packet_id_file_transfer,
	packet_id_serial_port_passthrough,
	packet_id_ip_configuration,
	packet_id_12_reserved,
	packet_id_extended_device_information,
	packet_id_subcomponent_information,
	end_system_packets,

	packet_id_system_state = START_STATE_PACKETS,
	packet_id_unix_time,
	packet_id_formatted_time,
	packet_id_status,
	packet_id_position_standard_deviation,
	packet_id_velocity_standard_deviation,
	packet_id_euler_orientation_standard_deviation,
	packet_id_quaternion_orientation_standard_deviation,
	packet_id_raw_sensors,
	packet_id_raw_gnss,
	packet_id_satellites,
	packet_id_satellites_detailed,
	packet_id_geodetic_position,
	packet_id_ecef_position,
	packet_id_utm_position,
	packet_id_ned_velocity,
	packet_id_body_velocity,
	packet_id_acceleration,
	packet_id_body_acceleration,
	packet_id_euler_orientation,
	packet_id_quaternion_orientation,
	packet_id_dcm_orientation,
	packet_id_angular_velocity,
	packet_id_angular_acceleration,
	packet_id_external_position_velocity,
	packet_id_external_position,
	packet_id_external_velocity,
	packet_id_external_body_velocity,
	packet_id_external_heading,
	packet_id_running_time,
	packet_id_local_magnetics,
	packet_id_odometer_state,
	packet_id_external_time,
	packet_id_external_depth,
	packet_id_geoid_height,
	packet_id_rtcm_corrections,
	packet_id_56_reserved,
	packet_id_wind,
	packet_id_heave,
	packet_id_59_reserved,
	packet_id_raw_satellite_data,
	packet_id_raw_satellite_ephemeris,
	packet_id_62_reserved,
	packet_id_63_reserved,
	packet_id_64_reserved,
	packet_id_65_reserved,
	packet_id_gnss_summary,
	packet_id_external_odometer,
	packet_id_external_air_data,
	packet_id_gnss_receiver_information,
	packet_id_raw_dvl_data,
	packet_id_north_seeking_status,
	packet_id_gimbal_state,
	packet_id_automotive,
	packet_id_74_reserved,
	packet_id_external_magnetometers,
	packet_id_76_reserved,
	packet_id_77_reserved,
	packet_id_78_reserved,
	packet_id_79_reserved,
	packet_id_basestation,
	packet_id_81_reserved,
	packet_id_82_reserved,
	packet_id_zero_angular_velocity,
	packet_id_extended_satellites,
	packet_id_sensor_temperatures,
	packet_id_system_temperature,
	packet_id_87_reserved,
	packet_id_88_reserved,
	packet_id_vessel_motion,
	packet_id_automatic_magnetometer_calibration_status,
	end_state_packets,

	packet_id_packet_timer_period = START_CONFIGURATION_PACKETS,
	packet_id_packet_periods,
	packet_id_baud_rates,
	packet_id_183_reserved,
	packet_id_sensor_ranges,
	packet_id_installation_alignment,
	packet_id_filter_options,
	packet_id_187_reserved,
	packet_id_gpio_configuration,
	packet_id_magnetic_calibration_values,
	packet_id_magnetic_calibration_configuration,
	packet_id_magnetic_calibration_status,
	packet_id_odometer_configuration,
	packet_id_zero_alignment,
	packet_id_reference_offsets,
	packet_id_gpio_output_configuration,
	packet_id_dual_antenna_configuration,
	packet_id_gnss_configuration,
	packet_id_user_data,
	packet_id_gpio_input_configuration,
	packet_id_200_reserved,
	packet_id_201_reserved,
	packet_id_ip_dataports_configuration,
	packet_id_can_configuration,
	packet_id_device_name,
	end_configuration_packets
} packet_id_e;

/* start of system packets typedef structs */

typedef enum
{
	acknowledge_success,
	acknowledge_failure_crc,
	acknowledge_failure_length,
	acknowledge_failure_range,
	acknowledge_failure_flash,
	acknowledge_failure_not_ready,
	acknowledge_failure_unknown_packet,
	acknowledge_failure_mode
} acknowledge_result_e;

typedef struct
{
	uint8_t packet_id;
	uint16_t packet_crc;
	uint8_t acknowledge_result;
} acknowledge_packet_t;

typedef enum
{
	boot_mode_bootloader,
	boot_mode_main_program
} boot_mode_e;

typedef struct
{
	uint8_t boot_mode;
} boot_mode_packet_t;

typedef enum
{
	device_id_spatial = 1,
	device_id_orientus = 3,
	device_id_spatial_fog,
	device_id_spatial_dual,
	device_id_obdii_odometer = 10,
	device_id_orientus_v3,
	device_id_ilu,
	device_id_air_data_unit,
	device_id_spatial_fog_dual = 16,
	device_id_motus,
	device_id_gnss_compass = 19,
	device_id_certus = 26,
	device_id_aries,
	device_id_boreas_d90,
	device_id_boreas_90_fpga = 35,
	device_id_boreas_90_coil,
	device_id_boreas_70_coil = 40,
	device_id_boreas_d70,
	device_id_boreas_70_fpga,
	device_id_boreas_a90,
	device_id_boreas_a70,
	device_id_certus_mini_a = 49,
	device_id_certus_mini_n,
	device_id_certus_mini_d
} device_id_e;

typedef struct
{
	uint32_t software_version;
	uint32_t device_id;
	uint32_t hardware_revision;
	uint32_t serial_number[3];
} device_information_packet_t;

typedef enum
{
	file_transfer_response_completed_successfully,
	file_transfer_response_ready,
	file_transfer_response_index_mismatch,
	file_transfer_response_refused = 64,
	file_transfer_response_bad_metadata,
	file_transfer_response_timeout,
	file_transfer_response_retry_error,
	file_transfer_response_storage_error,
	file_transfer_response_data_invalid,
	file_transfer_response_packet_length_invalid,
	file_transfer_response_total_size_invalid,
	file_transfer_response_overflow_error,
	file_transfer_response_busy,
	file_transfer_response_cancelled,
	file_transfer_response_incorrect_device,
	file_transfer_response_file_not_found = 128,
	file_transfer_response_access_denied
} file_transfer_response_e;

typedef struct
{
	uint32_t unique_id;
	uint32_t data_index;
	uint8_t response_code;
} file_transfer_acknowledge_packet_t;

typedef enum
{
	data_encoding_binary,
	data_encoding_aes256
} data_encoding_e;

typedef enum
{
	file_transfer_metadata_none,
	file_transfer_metadata_extended_anpp,
	file_transfer_metadata_utf8_filename,
	file_transfer_metadata_an_firmware
} file_transfer_metadata_e;

typedef struct
{
	uint32_t unique_id;
	uint32_t data_index;
	uint32_t total_size;
	uint8_t data_encoding;
	uint8_t metadata_type;
	uint16_t metadata_length;
	uint8_t* metadata;
	uint8_t* packet_data;
} file_transfer_packet_t;

/* start of state packets typedef structs */

typedef enum
{
	gnss_fix_none,
	gnss_fix_2d,
	gnss_fix_3d,
	gnss_fix_sbas,
	gnss_fix_differential,
	gnss_fix_omnistar,
	gnss_fix_rtk_float,
	gnss_fix_rtk_fixed
} gnss_fix_type_e;

typedef struct
{
	union
	{
		uint16_t r;
		struct
		{
			uint16_t system_failure :1;
			uint16_t accelerometer_sensor_failure :1;
			uint16_t gyroscope_sensor_failure :1;
			uint16_t magnetometer_sensor_failure :1;
			uint16_t pressure_sensor_failure :1;
			uint16_t gnss_failure :1;
			uint16_t accelerometer_over_range :1;
			uint16_t gyroscope_over_range :1;
			uint16_t magnetometer_over_range :1;
			uint16_t pressure_over_range :1;
			uint16_t minimum_temperature_alarm :1;
			uint16_t maximum_temperature_alarm :1;
			uint16_t low_voltage_alarm :1;
			uint16_t high_voltage_alarm :1;
			uint16_t gnss_antenna_fault :1;
			uint16_t serial_port_overflow_alarm :1;
		} b;
	} system_status;
	union
	{
		uint16_t r;
		struct
		{
			uint16_t orientation_filter_initialised :1;
			uint16_t ins_filter_initialised :1;
			uint16_t heading_initialised :1;
			uint16_t utc_time_initialised :1;
			uint16_t gnss_fix_type :3;
			uint16_t event1_flag :1;
			uint16_t event2_flag :1;
			uint16_t internal_gnss_enabled :1;
			uint16_t magnetic_heading_enabled :1;
			uint16_t velocity_heading_enabled :1;
			uint16_t atmospheric_altitude_enabled :1;
			uint16_t external_position_active :1;
			uint16_t external_velocity_active :1;
			uint16_t external_heading_active :1;
		} b;
	} filter_status;
	uint32_t unix_time_seconds;
	uint32_t microseconds;
	double latitude;
	double longitude;
	double height;
	float velocity[3];
	float body_acceleration[3];
	float g_force;
	float orientation[3];
	float angular_velocity[3];
	float standard_deviation[3];
} system_state_packet_t;

typedef struct
{
	uint32_t unix_time_seconds;
	uint32_t microseconds;
} unix_time_packet_t;

typedef struct
{
	union
	{
		uint16_t r;
		struct
		{
			uint16_t system_failure :1;
			uint16_t accelerometer_sensor_failure :1;
			uint16_t gyroscope_sensor_failure :1;
			uint16_t magnetometer_sensor_failure :1;
			uint16_t pressure_sensor_failure :1;
			uint16_t gnss_failure :1;
			uint16_t accelerometer_over_range :1;
			uint16_t gyroscope_over_range :1;
			uint16_t magnetometer_over_range :1;
			uint16_t pressure_over_range :1;
			uint16_t minimum_temperature_alarm :1;
			uint16_t maximum_temperature_alarm :1;
			uint16_t low_voltage_alarm :1;
			uint16_t high_voltage_alarm :1;
			uint16_t gnss_antenna_disconnected :1;
			uint16_t serial_port_overflow_alarm :1;
		} b;
	} system_status;
	union
	{
		uint16_t r;
		struct
		{
			uint16_t orientation_filter_initialised :1;
			uint16_t ins_filter_initialised :1;
			uint16_t heading_initialised :1;
			uint16_t utc_time_initialised :1;
			uint16_t gnss_fix_type :3;
			uint16_t event1_flag :1;
			uint16_t event2_flag :1;
			uint16_t internal_gnss_enabled :1;
			uint16_t magnetic_heading_enabled :1;
			uint16_t velocity_heading_enabled :1;
			uint16_t atmospheric_altitude_enabled :1;
			uint16_t external_position_active :1;
			uint16_t external_velocity_active :1;
			uint16_t external_heading_active :1;
		} b;
	} filter_status;
} status_packet_t;

typedef struct
{
	float standard_deviation[3];
} euler_orientation_standard_deviation_packet_t;

typedef struct
{
	float standard_deviation[4];
} quaternion_orientation_standard_deviation_packet_t;

typedef struct
{
	float accelerometers[3];
	float gyroscopes[3];
	float magnetometers[3];
	float imu_temperature;
	float pressure;
	float pressure_temperature;
} raw_sensors_packet_t;

typedef struct
{
	float acceleration[3];
} acceleration_packet_t;

typedef struct
{
	float orientation[3];
} euler_orientation_packet_t;

typedef struct
{
	float orientation[4];
} quaternion_orientation_packet_t;

typedef struct
{
	float orientation[3][3];
} dcm_orientation_packet_t;

typedef struct
{
	float angular_velocity[3];
} angular_velocity_packet_t;

typedef struct
{
	float angular_acceleration[3];
} angular_acceleration_packet_t;

typedef struct
{
	double position[3];
	float velocity[3];
	float position_standard_deviation[3];
	float velocity_standard_deviation[3];
} external_position_velocity_packet_t;

typedef struct
{
	double position[3];
	float standard_deviation[3];
} external_position_packet_t;

typedef struct
{
	float velocity[3];
	float standard_deviation[3];
} external_velocity_packet_t;

typedef struct
{
	float heading;
	float standard_deviation;
} external_heading_packet_t;

typedef struct
{
	uint32_t seconds;
	uint32_t microseconds;
} running_time_packet_t;

typedef struct
{
	float magnetic_field[3];
} local_magnetics_packet_t;

/* start of configuration packets typedef structs */

typedef struct
{
	uint8_t permanent;
	uint8_t utc_synchronisation;
	uint16_t packet_timer_period;
} packet_timer_period_packet_t;

typedef struct
{
	uint8_t packet_id;
	uint32_t period;
} packet_period_t;

typedef struct
{
	uint8_t permanent;
	uint8_t clear_existing_packets;
	packet_period_t packet_periods[MAXIMUM_PACKET_PERIODS];
} packet_periods_packet_t;

typedef struct
{
	uint8_t permanent;
	uint32_t primary_baud_rate;
	uint32_t gpio_1_2_baud_rate;
	uint32_t auxiliary_baud_rate;
	uint32_t reserved;
} baud_rates_packet_t;

typedef enum
{
	accelerometer_range_2g,
	accelerometer_range_4g,
	accelerometer_range_16g
} accelerometer_range_e;

typedef enum
{
	gyroscope_range_250dps,
	gyroscope_range_500dps,
	gyroscope_range_2000dps
} gyroscope_range_e;

typedef enum
{
	magnetometer_range_2g,
	magnetometer_range_4g,
	magnetometer_range_8g
} magnetometer_range_e;

typedef struct
{
	uint8_t permanent;
	uint8_t accelerometers_range;
	uint8_t gyroscopes_range;
	uint8_t magnetometers_range;
} sensor_ranges_packet_t;

typedef struct
{
	uint8_t permanent;
	float alignment_dcm[3][3];
	float gnss_antenna_offset[3];
	float odometer_offset[3];
	float external_data_offset[3];
} installation_alignment_packet_t;

typedef enum
{
	vehicle_type_unlimited,
	vehicle_type_bicycle,
	vehicle_type_car,
	vehicle_type_hovercraft,
	vehicle_type_submarine,
	vehicle_type_3d_underwater,
	vehicle_type_fixed_wing_plane,
	vehicle_type_3d_aircraft,
	vehicle_type_human,
	vehicle_type_small_boat,
	vehicle_type_ship,
	vehicle_type_stationary,
	vehicle_type_stunt_plane,
	vehicle_type_race_car,
	vehicle_type_train
} vehicle_type_e;

typedef struct
{
	uint8_t permanent;
	uint8_t vehicle_type;
	uint8_t reserved2;
	uint8_t magnetometers_enabled;
	uint8_t reserved4;
	uint8_t reserved5;
	uint8_t reserved6;
	uint8_t reserved7;
	uint8_t reserved8;
	uint8_t reserved9;
	uint8_t reserved10;
} filter_options_packet_t;

typedef enum
{
	gpio_function_inactive,
	gpio_function_1pps_output,
	gpio_function_gnss_fix_output,
	gpio_function_odometer_input,
	gpio_function_stationary_input,
	gpio_function_pitot_tube_input,
	gpio_function_nmea_input,
	gpio_function_nmea_output,
	gpio_function_novatel_gnss_input,
	gpio_function_topcon_gnss_input,
	gpio_function_motec_output,
	gpio_function_anpp_input,
	gpio_function_anpp_output,
	gpio_function_disable_magnetometers,
	gpio_function_disable_gnss,
	gpio_function_disable_pressure,
	gpio_function_set_zero_alignment,
	gpio_function_packet_trigger_system_state,
	gpio_function_packet_trigger_raw_sensors,
	gpio_function_rtcm_corrections_input,
	gpio_function_trimble_gnss_input,
	gpio_function_ublox_gnss_input,
	gpio_function_hemisphere_gnss_input,
	gpio_function_teledyne_dvl_input,
	gpio_function_tritech_usbl_input,
	gpio_function_linkquest_dvl_input,
	gpio_function_pressure_depth_sensor,
	gpio_function_left_wheel_speed_sensor,
	gpio_function_right_wheel_speed_sensor,
	gpio_function_pps_input,
	gpio_function_wheel_speed_sensor,
	gpio_function_wheel_encoder_phase_a,
	gpio_function_wheel_encoder_phase_b,
	gpio_function_event1_input,
	gpio_function_event2_input,
	gpio_function_linkquest_usbl_input,
	gpio_function_ixblue_input,
	gpio_function_sonardyne_input,
	gpio_function_gnss_receiver_passthrough,
	gpio_function_tss1_output,
	gpio_function_simrad_1000_output,
	gpio_function_simrad_3000_output,
	gpio_function_serial_port_passthrough,
	gpio_function_gimbal_encoder_phase_a,
	gpio_function_gimbal_encoder_phase_b,
	gpio_function_odometer_direction_forward_low,
	gpio_function_odometer_direction_forward_high,
	gpio_function_virtual_odometer_output,
	gpio_function_novatel_mark_output,
	gpio_function_gpio2_8mhz_output,
	gpio_function_ng206_output,
	gpio_function_nortek_dvl_input,
	gpio_function_moving_base_corrections_output,
	gpio_function_reverse_alignment_forward_low,
	gpio_function_reverse_alignment_forward_high,
	gpio_function_zero_angular_velocity_input,
	gpio_function_mavlink_output,
	gpio_function_gnss_receiver2_passthrough,
	gpio_function_water_linked_dvl_input = 59,
	gpio_function_nortek_nucleus_dvl_input = 63,
	gpio_function_nortek_nucleus_dvl_output
} gpio_function_e;

typedef enum
{
	gpio_index_gpio1,
	gpio_index_gpio2,
	gpio_index_auxiliary_tx,
	gpio_index_auxiliary_rx
} gpio_index_e;


typedef struct
{
	uint8_t permanent;
	uint8_t gpio_function[4];
} gpio_configuration_packet_t;

typedef struct
{
	uint8_t permanent;
	float hard_iron[3];
	float soft_iron[3][3];
} magnetic_calibration_values_packet_t;

typedef enum
{
	magnetic_calibration_action_cancel,
	magnetic_calibration_action_stabilise,
	magnetic_calibration_action_start_2d,
	magnetic_calibration_action_start_3d
} magnetic_calibration_action_e;

typedef struct
{
	uint8_t magnetic_calibration_action;
} magnetic_calibration_configuration_packet_t;

typedef enum
{
	magnetic_calibration_status_not_completed,
	magnetic_calibration_status_completed_2d,
	magnetic_calibration_status_completed_3d,
	magnetic_calibration_status_completed_user,
	magnetic_calibration_status_stabilizing,
	magnetic_calibration_status_in_progress_2d,
	magnetic_calibration_status_in_progress_3d,
	magnetic_calibration_status_error_excessive_roll,
	magnetic_calibration_status_error_excessive_pitch,
	magnetic_calibration_status_error_overrange_event,
	magnetic_calibration_status_error_timeout,
	magnetic_calibration_status_error_system,
	magnetic_calibration_status_error_interference
} magnetic_calibration_status_e;

typedef struct
{
	uint8_t magnetic_calibration_status;
	uint8_t magnetic_calibration_progress_percentage;
	uint8_t local_magnetic_error_percentage;
} magnetic_calibration_status_packet_t;

typedef struct
{
	uint8_t permanent;
} zero_alignment_packet_t;


int decode_acknowledge_packet(acknowledge_packet_t* acknowledge_packet, an_packet_t* an_packet);
an_packet_t* encode_request_packet(uint8_t requested_packet_id);
int decode_boot_mode_packet(boot_mode_packet_t* boot_mode_packet, an_packet_t* an_packet);
an_packet_t* encode_boot_mode_packet(boot_mode_packet_t* boot_mode_packet);
int decode_device_information_packet(device_information_packet_t* device_information_packet, an_packet_t* an_packet);
an_packet_t* encode_restore_factory_settings_packet();
an_packet_t* encode_reset_packet();
int decode_file_transfer_acknowledge_packet(file_transfer_acknowledge_packet_t* file_transfer_acknowledge_packet, an_packet_t* an_packet);
an_packet_t* encode_file_transfer_packet(file_transfer_packet_t* file_transfer_packet, int data_size);
int decode_system_state_packet(system_state_packet_t* system_state_packet, an_packet_t* an_packet);
int decode_unix_time_packet(unix_time_packet_t* unix_time_packet, an_packet_t* an_packet);
int decode_status_packet(status_packet_t* status_packet, an_packet_t* an_packet);
int decode_euler_orientation_standard_deviation_packet(euler_orientation_standard_deviation_packet_t* euler_orientation_standard_deviation, an_packet_t* an_packet);
int decode_quaternion_orientation_standard_deviation_packet(quaternion_orientation_standard_deviation_packet_t* quaternion_orientation_standard_deviation_packet, an_packet_t* an_packet);
int decode_raw_sensors_packet(raw_sensors_packet_t* raw_sensors_packet, an_packet_t* an_packet);
int decode_acceleration_packet(acceleration_packet_t* acceleration, an_packet_t* an_packet);
int decode_euler_orientation_packet(euler_orientation_packet_t* euler_orientation_packet, an_packet_t* an_packet);
int decode_quaternion_orientation_packet(quaternion_orientation_packet_t* quaternion_orientation_packet, an_packet_t* an_packet);
int decode_dcm_orientation_packet(dcm_orientation_packet_t* dcm_orientation_packet, an_packet_t* an_packet);
int decode_angular_velocity_packet(angular_velocity_packet_t* angular_velocity_packet, an_packet_t* an_packet);
int decode_angular_acceleration_packet(angular_acceleration_packet_t* angular_acceleration_packet, an_packet_t* an_packet);
int decode_external_position_velocity_packet(external_position_velocity_packet_t* external_position_velocity_packet, an_packet_t* an_packet);
an_packet_t* encode_external_position_velocity_packet(external_position_velocity_packet_t* external_position_velocity_packet);
int decode_external_position_packet(external_position_packet_t* external_position_packet, an_packet_t* an_packet);
an_packet_t* encode_external_position_packet(external_position_packet_t* external_position_packet);
int decode_external_velocity_packet(external_velocity_packet_t* external_velocity_packet, an_packet_t* an_packet);
an_packet_t* encode_external_velocity_packet(external_velocity_packet_t* external_velocity_packet);
int decode_external_heading_packet(external_heading_packet_t* external_heading_packet, an_packet_t* an_packet);
an_packet_t* encode_external_heading_packet(external_heading_packet_t* external_heading_packet);
int decode_running_time_packet(running_time_packet_t* running_time_packet, an_packet_t* an_packet);
int decode_local_magnetics_packet(local_magnetics_packet_t* local_magnetics_packet, an_packet_t* an_packet);
int decode_packet_timer_period_packet(packet_timer_period_packet_t* packet_timer_period_packet, an_packet_t* an_packet);
an_packet_t* encode_packet_timer_period_packet(packet_timer_period_packet_t* packet_timer_period_packet);
int decode_packet_periods_packet(packet_periods_packet_t* packet_periods_packet, an_packet_t* an_packet);
an_packet_t* encode_packet_periods_packet(packet_periods_packet_t* packet_periods_packet);
int decode_baud_rates_packet(baud_rates_packet_t* baud_rates_packet, an_packet_t* an_packet);
an_packet_t* encode_baud_rates_packet(baud_rates_packet_t* baud_rates_packet);
int decode_sensor_ranges_packet(sensor_ranges_packet_t* sensor_ranges_packet, an_packet_t* an_packet);
an_packet_t* encode_sensor_ranges_packet(sensor_ranges_packet_t* sensor_ranges_packet);
int decode_installation_alignment_packet(installation_alignment_packet_t* installation_alignment_packet, an_packet_t* an_packet);
an_packet_t* encode_installation_alignment_packet(installation_alignment_packet_t* installation_alignment_packet);
int decode_filter_options_packet(filter_options_packet_t* filter_options_packet, an_packet_t* an_packet);
an_packet_t* encode_filter_options_packet(filter_options_packet_t* filter_options_packet);
int decode_gpio_configuration_packet(gpio_configuration_packet_t* gpio_configuration_packet, an_packet_t* an_packet);
an_packet_t* encode_gpio_configuration_packet(gpio_configuration_packet_t* gpio_configuration_packet);
int decode_magnetic_calibration_values_packet(magnetic_calibration_values_packet_t* magnetic_calibration_values_packet, an_packet_t* an_packet);
an_packet_t* encode_magnetic_calibration_values_packet(magnetic_calibration_values_packet_t* magnetic_calibration_values_packet);
an_packet_t* encode_magnetic_calibration_configuration_packet(magnetic_calibration_configuration_packet_t* magnetic_calibration_configuration_packet);
int decode_magnetic_calibration_status_packet(magnetic_calibration_status_packet_t* magnetic_calibration_status_packet, an_packet_t* an_packet);
an_packet_t* encode_zero_alignment_packet(zero_alignment_packet_t* zero_alignment_packet);

#ifdef __cplusplus
}
#endif

#endif
