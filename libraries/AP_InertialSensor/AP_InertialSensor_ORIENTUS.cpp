
#include <utility>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

#include <stdlib.h> //déjà inclus dans un header de Orientus_scripts
#include <stdio.h>
#include <stdint.h> //déjà inclus dans un header de Orientus_scripts
#define _USE_MATH_DEFINES
#include <math.h>
#include <time.h>
#include <string.h>

#include <unistd.h>

#include "AP_InertialSensor_ORIENTUS.h"

#define DEV_BACKEND_SAMPLE_PER 2
#define DEV_BACKEND_SAMPLE_RATE 1000 / DEV_BACKEND_SAMPLE_PER

static unsigned char request_all_configuration[] = {0xE2, 0x01, 0x10, 0x9A, 0x73, 0xB6, 0xB4, 0xB5, 0xB8, 0xB9, 0xBA, 0xBC, 0xBD, 0xC0, 0xC2, 0xC3, 0xC4, 0x03, 0xC6, 0x45, 0xC7};

extern const AP_HAL::HAL &hal;

#define int16_val(v, idx) ((int16_t)(((uint16_t)v[2 * idx] << 8) | v[2 * idx + 1]))


AP_InertialSensor_ORIENTUS::AP_InertialSensor_ORIENTUS(AP_InertialSensor &imu,
                                                       enum Rotation _rotation)
    : AP_InertialSensor_Backend(imu), rotation(_rotation)
{
}

AP_InertialSensor_Backend *
AP_InertialSensor_ORIENTUS::probe(AP_InertialSensor &imu,
                                  enum Rotation rotation)
{
    auto sensor = NEW_NOTHROW AP_InertialSensor_ORIENTUS(imu, rotation);
    
    if (!sensor)
    {
        return nullptr;
    }

    if (!sensor->init())
    {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

uint8_t AP_InertialSensor_ORIENTUS::bus_id = 0;

void AP_InertialSensor_ORIENTUS::start()
{
    if (!_imu.register_gyro(gyro_instance, DEV_BACKEND_SAMPLE_RATE, AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_SERIAL, bus_id, 1, DEVTYPE_INS_ORIENTUS)) ||
        !_imu.register_accel(accel_instance, DEV_BACKEND_SAMPLE_RATE, AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_SERIAL, bus_id, 2, DEVTYPE_INS_ORIENTUS)))
    {
        return;
    }
    bus_id++;
    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_InertialSensor_ORIENTUS::read_packet, void));
    compt =0;

    // setup sensor rotations from probe()
    set_gyro_orientation(gyro_instance, rotation);
    set_accel_orientation(accel_instance, rotation);
}

bool AP_InertialSensor_ORIENTUS::hardware_init()
{
    /* Request all the configuration and the device information from the unit */
    transmit(request_all_configuration, sizeof(request_all_configuration));

    an_decoder_initialise(&an_decoder);

    packet_periods_packet_t packet1 = {
        .permanent = 0,
        .clear_existing_packets = 1,
        .packet_periods = {}};
    an_packet_transmit(encode_packet_periods_packet(&packet1));

    sensor_ranges_packet_t packet2 = {
        .permanent = 0,
        .accelerometers_range = accelerometer_range_4g,
        .gyroscopes_range = gyroscope_range_250dps};
    an_packet_transmit(encode_sensor_ranges_packet(&packet2));

    return true;
}

bool AP_InertialSensor_ORIENTUS::init()
{
    const auto &serial_manager = AP::serialmanager();
    AP_HAL::UARTDriver *driv = serial_manager.find_serial(AP_SerialManager::SerialProtocol_ORIENTUS, 0);
    _driv = driv;
    if (!driv)
    {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "No driver found");
        return false;
    }
    else
    {
        return hardware_init();
    }
}

void AP_InertialSensor_ORIENTUS::read_packet(void)
{
    if (++compt == DEV_BACKEND_SAMPLE_PER) {
        compt = 0;
        int bytes_received;
        an_packet_transmit(encode_request_packet(packet_id_raw_sensors));
        if ((bytes_received = receive(an_decoder_pointer(&an_decoder), an_decoder_size(&an_decoder))) > 0)
        {
            an_decoder_increment(&an_decoder, bytes_received);
            while ((an_packet = an_packet_decode(&an_decoder)) != NULL)
            {
                if (an_packet->id == packet_id_raw_sensors) /* raw sensors packet */
                {
                    /* copy all the binary data into the typedef struct for the packet */
                    /* this allows easy access to all the different values             */
                    if (decode_raw_sensors_packet(&raw_sensors_packet, an_packet) == 0)
                    {
                        Vector3f accel(raw_sensors_packet.accelerometers[0], raw_sensors_packet.accelerometers[1], raw_sensors_packet.accelerometers[2]);
                        _rotate_and_correct_accel(accel_instance, accel);
                        _notify_new_accel_raw_sample(accel_instance, accel);

                        Vector3f gyro(raw_sensors_packet.gyroscopes[0], raw_sensors_packet.gyroscopes[1], raw_sensors_packet.gyroscopes[2]);
                        _rotate_and_correct_gyro(gyro_instance, gyro);
                        _notify_new_gyro_raw_sample(gyro_instance, gyro);

                        if (temperature_counter++ == 100)
                        {
                            temperature_counter = 0;
                            float temp_degc = raw_sensors_packet.imu_temperature;
                            _publish_temperature(accel_instance, temp_degc);
                        }
                    }
                    else
                    {
                        gcs().send_text(MAV_SEVERITY_CRITICAL, "Decription Issue");
                    }
                }
                /* Ensure that you free the an_packet when your done with it or you will leak memory */
                an_packet_free(&an_packet);
            }
        }
    }
}

bool AP_InertialSensor_ORIENTUS::update()
{
    update_accel(accel_instance);
    update_gyro(gyro_instance);
    return true;
}

int AP_InertialSensor_ORIENTUS::transmit(const unsigned char *data, int length)
{
    return _driv->write(data, length);
}

int AP_InertialSensor_ORIENTUS::receive(unsigned char *data, int length)
{
    return _driv->read(data, length);
}

int AP_InertialSensor_ORIENTUS::an_packet_transmit(an_packet_t *packet)
{
    an_packet_encode(packet);
    return transmit(an_packet_pointer(packet), an_packet_size(packet));
}

/*
 * This is an example of sending a configuration packet to Orientus.
 *
 * 1. First declare the structure for the packet, in this case filter_options_packet_t.
 * 2. Set all the fields of the packet structure
 * 3. Encode the packet structure into an an_packet_t using the appropriate helper function
 * 4. Send the packet
 * 5. Free the packet
 */
void AP_InertialSensor_ORIENTUS::set_filter_options()
{
    an_packet_t *packet;
    filter_options_packet_t filter_options_packet;

    /* initialise the structure by setting all the fields to zero */
    memset(&filter_options_packet, 0, sizeof(filter_options_packet_t));

    filter_options_packet.permanent = TRUE;
    filter_options_packet.vehicle_type = vehicle_type_3d_underwater;
    filter_options_packet.magnetometers_enabled = TRUE;

    packet = encode_filter_options_packet(&filter_options_packet);

    an_packet_transmit(packet);

    an_packet_free(&packet);
}