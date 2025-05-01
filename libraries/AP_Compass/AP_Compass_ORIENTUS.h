#pragma once

#include <AP_HAL/AP_HAL.h>

#include "../../AP_InertialSensor/ORIENTUS_scripts/an_packet_protocol.h"
#include "../../AP_InertialSensor/ORIENTUS_scripts/ins_packets.h"

#include "AP_Compass.h"
#include "AP_Compass_Backend.h"

class AP_Compass_ORIENTUS : public AP_Compass_Backend
{
public:
    static AP_Compass_Backend *probe(enum Rotation rotation);

    /**
     * Configure the sensors and start reading routine.
     */
    void read() override;

    static constexpr const char *name = "Orientus";

    virtual ~AP_Compass_ORIENTUS() { }

private:
    an_decoder_t an_decoder;
    an_packet_t *an_packet;

    system_state_packet_t system_state_packet;
    raw_sensors_packet_t raw_sensors_packet;

    uint8_t compt;

    AP_Compass_ORIENTUS(enum Rotation rotation);

    AP_HAL::OwnPtr<AP_HAL::UARTDriver> _driv;

    /*
     initialise hardware layer
     */
    bool hardware_init();

    bool read_packet();

    /*
      initialise driver
     */
    bool init();

    void _update();

    static uint8_t bus_id;
    uint8_t _compass_instance;

    float _mag_x;
    float _mag_y;
    float _mag_z;
    

    enum Rotation _rotation;

    uint16_t mag_over_range_counter;

    int transmit(const unsigned char *data, int length);
    int receive(unsigned char *data, int length);
    int an_packet_transmit(an_packet_t *an_packet);
    void set_filter_options();
    
};