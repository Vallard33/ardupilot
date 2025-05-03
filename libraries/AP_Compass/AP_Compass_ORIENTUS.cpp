
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

#include "AP_Compass_ORIENTUS.h"

#define MAG_BACKEND_SAMPLE_RATE 100
#define KEY_COMPASS 2 // key for locking the port (1 for imu)

static unsigned char request_all_configuration[] = {0xE2, 0x01, 0x10, 0x9A, 0x73, 0xB6, 0xB4, 0xB5, 0xB8, 0xB9, 0xBA, 0xBC, 0xBD, 0xC0, 0xC2, 0xC3, 0xC4, 0x03, 0xC6, 0x45, 0xC7};

extern const AP_HAL::HAL &hal;

#define int16_val(v, idx) ((int16_t)(((uint16_t)v[2 * idx] << 8) | v[2 * idx + 1]))

#define BAUD_RATE 112500
#define DEV_BACKEND_SAMPLE_PER 10 // 100Hz

AP_Compass_ORIENTUS::AP_Compass_ORIENTUS(enum Rotation rotation)
    : _rotation(rotation)
{
}

uint8_t AP_Compass_ORIENTUS::bus_id = 0;

AP_Compass_Backend *AP_Compass_ORIENTUS::probe(enum Rotation rotation)
{
    gcs().send_text(MAV_SEVERITY_CRITICAL, "probe");
    AP_Compass_ORIENTUS *sensor = NEW_NOTHROW AP_Compass_ORIENTUS(rotation);
    if (!sensor || !sensor->init())
    {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "aie aie aie");
        delete sensor;
        return nullptr;
    }
    return sensor;
}

bool AP_Compass_ORIENTUS::hardware_init()
{
    gcs().send_text(MAV_SEVERITY_CRITICAL, "hardware init");
    an_decoder_initialise(&an_decoder);
    // if (_driv->lock_port(2,2)) {
    /* Request all the configuration and the device information from the unit */
    transmit(request_all_configuration, sizeof(request_all_configuration));

    packet_periods_packet_t packet1 = {
        .permanent = 0,
        .clear_existing_packets = 1,
        .packet_periods = {}};
    an_packet_transmit(encode_packet_periods_packet(&packet1));

    sensor_ranges_packet_t packet2 = {
        .permanent = 0,
        .accelerometers_range = accelerometer_range_4g,
        .gyroscopes_range = gyroscope_range_250dps,
        .magnetometers_range = magnetometer_range_2g};
    an_packet_transmit(encode_sensor_ranges_packet(&packet2));
    //_driv->lock_port(0,0);
    //}
    return true;
}

bool AP_Compass_ORIENTUS::init()
{
    const auto &serial_manager = AP::serialmanager();
    AP_HAL::UARTDriver *driv = serial_manager.find_serial(AP_SerialManager::SerialProtocol_ORIENTUS, 0);
    _driv = driv;
    if (!driv)
    {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Pas de driver");
        return false;
    }
    if (!register_compass(AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_SERIAL, bus_id, 0, DEVTYPE_ORIENTUS), _compass_instance))
    {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "oupsi");
        return false;
    }

    set_dev_id(_compass_instance, AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_SERIAL, bus_id, 1, DEVTYPE_ORIENTUS));
    set_rotation(_compass_instance, _rotation);
    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_Compass_ORIENTUS::_update, void));
    compt = 0;
    mag_over_range_counter = 0;
    return hardware_init();
}

/*
  read packet
 */
bool AP_Compass_ORIENTUS::read_packet(void)
{
    int bytes_received = 0;
    // if (_driv->lock_port(2, 2))
    //{
    // an_packet_transmit(encode_request_packet(packet_id_system_state));
    an_packet_transmit(encode_request_packet(packet_id_raw_sensors));
    bytes_received = receive(an_decoder_pointer(&an_decoder), an_decoder_size(&an_decoder));
    //    _driv->lock_port(0, 0);
    //}
    if (bytes_received > 0)
    { /* increment the decode buffer length by the number of bytes received */
        an_decoder_increment(&an_decoder, bytes_received);
        /* decode all the packets in the buffer */
        while ((an_packet = an_packet_decode(&an_decoder)) != NULL)
        {
            if (an_packet->id == packet_id_system_state) /* system state packet */
            {
                if (decode_system_state_packet(&system_state_packet, an_packet) == 0)
                {
                    if (system_state_packet.system_status.b.magnetometer_over_range)
                    {
                        if ((mag_over_range_counter++ % 400) == 0)
                        {
                            gcs().send_text(MAV_SEVERITY_CRITICAL, "Magnetometer Over Range");
                            return false;
                        }
                    }
                    else
                    {
                        mag_over_range_counter = 0;
                    }
                }
            }
            // else if (an_packet->id == packet_id_raw_sensors) /* raw sensors packet */
            if (an_packet->id == packet_id_raw_sensors) /* raw sensors packet */
            {               
                if (decode_raw_sensors_packet(&raw_sensors_packet, an_packet) == 0)
                {
                    _mag_x = raw_sensors_packet.magnetometers[0];
                    _mag_y = raw_sensors_packet.magnetometers[1];
                    _mag_z = raw_sensors_packet.magnetometers[2];
                }
                else
                {
                    gcs().send_text(MAV_SEVERITY_CRITICAL, "Probleme decryptage");
                    return false;
                }
            }
            /* Ensure that you free the an_packet when your done with it or you will leak memory */
            an_packet_free(&an_packet);
        }
    }
    return true;
}

void AP_Compass_ORIENTUS::_update()
{
    // if (++compt == DEV_BACKEND_SAMPLE_PER)
    //{
    //     compt = 0;
    if (!read_packet())
    {
        return;
    }
    Vector3f raw_field = Vector3f((float)_mag_x, (float)_mag_y, (float)_mag_z);
    accumulate_sample(raw_field, _compass_instance);
    //}
}

void AP_Compass_ORIENTUS::read()
{
    drain_accumulated_samples(_compass_instance);
}

int AP_Compass_ORIENTUS::transmit(const unsigned char *data, int length)
{
    //    return _driv->write_locked(data, length, 2);
    return _driv->write(data, length);
}

int AP_Compass_ORIENTUS::receive(unsigned char *data, int length)
{
    // return _driv->read_locked(data, length, 2);
    return _driv->read(data, length);
}
int AP_Compass_ORIENTUS::an_packet_transmit(an_packet_t *packet)
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
void AP_Compass_ORIENTUS::set_filter_options()
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