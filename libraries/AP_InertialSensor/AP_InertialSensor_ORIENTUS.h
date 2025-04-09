#pragma once

#include <AP_HAL/AP_HAL.h>

#include "ORIENTUS_scripts/an_packet_protocol.h"
#include "ORIENTUS_scripts/ins_packets.h"

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

class AP_InertialSensor_ORIENTUS : public AP_InertialSensor_Backend {
    public:
        static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                                enum Rotation rotation);
    
        /**
         * Configure the sensors and start reading routine.
         */
        void start() override;
        bool update() override;
    
    private:

        an_decoder_t an_decoder;
	    an_packet_t *an_packet;

	    system_state_packet_t system_state_packet;
	    raw_sensors_packet_t raw_sensors_packet;

        AP_InertialSensor_ORIENTUS(AP_InertialSensor &imu,
                                   enum Rotation rotation);
    
        /*
         initialise hardware layer
         */
        bool hardware_init();
    
        /*
          initialise driver
         */
        bool init();
    
        /*
          read packets
         */
        void read_packet();
        
        static uint8_t bus_id;
        
        enum Rotation rotation;
        uint8_t temperature_counter;


        int transmit(const unsigned char *data, int length);
        int receive(unsigned char *data, int length);
        int an_packet_transmit(an_packet_t *an_packet);
        void set_filter_options();
    };
    