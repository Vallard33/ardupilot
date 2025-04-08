
#include <utility>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

#define RS232 0
#define NETWORK 1

#define CONNECTION_TYPE RS232

// #include <stdlib.h> //déjà inclus dans un header de Orientus_scripts
#include <stdio.h>
// #include <stdint.h> //déjà inclus dans un header de Orientus_scripts
#define _USE_MATH_DEFINES
#include <math.h>
#include <time.h>
#include <string.h>

#ifdef _WIN32
#include <windows.h>
#if CONNECTION_TYPE == NETWORK
#define _WIN32_WINNT 0x0501
#include <winsock2.h>
#include <ws2tcpip.h>
#endif
#else
#include <unistd.h>
// #if CONNECTION_TYPE == RS232
// #include "rs232/rs232.h"
//#elif CONNECTION_TYPE == NETWORK
//#include <sys/socket.h>
//#include <arpa/inet.h>
//#include <netdb.h>
//#endif
#endif

#include "AP_InertialSensor_ORIENTUS.h"
#include "ORIENTUS_scripts/rs232.h"

#define ACCEL_BACKEND_SAMPLE_RATE 2000
#define GYRO_BACKEND_SAMPLE_RATE 2000
#define DEV_BACKEND_SAMPLE_RATE 400

#define RADIANS_TO_DEGREES (180.0 / M_PI)

static unsigned char request_all_configuration[] = {0xE2, 0x01, 0x10, 0x9A, 0x73, 0xB6, 0xB4, 0xB5, 0xB8, 0xB9, 0xBA, 0xBC, 0xBD, 0xC0, 0xC2, 0xC3, 0xC4, 0x03, 0xC6, 0x45, 0xC7};
int comPortIndex = -1;
int socket_fd = -1;

extern const AP_HAL::HAL &hal;

#define int16_val(v, idx) ((int16_t)(((uint16_t)v[2 * idx] << 8) | v[2 * idx + 1]))

#define PORT_COM "/dev/ttyUSB0"
#define BAUD_RATE 112500

AP_InertialSensor_ORIENTUS::AP_InertialSensor_ORIENTUS(AP_InertialSensor &imu,
                                                       AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                                       enum Rotation _rotation)
    : AP_InertialSensor_Backend(imu), _dev(std::move(dev)), rotation(_rotation)
{
}

AP_InertialSensor_Backend *
AP_InertialSensor_ORIENTUS::probe(AP_InertialSensor &imu,
                                  enum Rotation rotation)
{
    AP_HAL::OwnPtr<AP_HAL::Device> dev;
    if (!dev)
    {
        return nullptr;
    }
    auto sensor = NEW_NOTHROW AP_InertialSensor_ORIENTUS(imu, std::move(dev), rotation);

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

void AP_InertialSensor_ORIENTUS::start()
{

    if (!_imu.register_accel(accel_instance, DEV_BACKEND_SAMPLE_RATE, _dev->get_bus_id_devtype(DEVTYPE_ACC_ORIENTUS)) ||
        !_imu.register_gyro(gyro_instance, DEV_BACKEND_SAMPLE_RATE, _dev->get_bus_id_devtype(DEVTYPE_GYR_ORIENTUS)))
    {
        return;
    }

    // setup sensor rotations from probe()
    set_gyro_orientation(gyro_instance, rotation);
    set_accel_orientation(accel_instance, rotation);

    // setup callbacks on le fait juste pour accel parce que read_packet change aussi le gyro
    _dev->register_periodic_callback(1000000UL / DEV_BACKEND_SAMPLE_RATE,
                                          FUNCTOR_BIND_MEMBER(&AP_InertialSensor_ORIENTUS::read_packet, void));
    
}

bool AP_InertialSensor_ORIENTUS::hardware_init()
{
    
    _dev->get_semaphore()->take_blocking();
    // Il faudrait personnaliser tout ça si on veut changer les ranges de l'accéléromètre et du gyroscope et pour d'autres paramètres, ...
    /* Find the serial port */
    comEnumerate();
    comPortIndex = comFindPort(PORT_COM);
    if (comPortIndex == -1)
    {
        DEV_PRINTF("Serial port not available\n");
        _dev->get_semaphore()->give();
        return false;
    }
    /* Open the serial port */
    //if (comOpen(comPortIndex, atoi(BAUD_RATE)) == 0)
    if (comOpen(comPortIndex, BAUD_RATE) == 0)  //ici atoi ne sert à rien car on a déjà un entier
    {
        DEV_PRINTF("Could not open serial port\n");
        _dev->get_semaphore()->give();
        return false;
    }

    /* Request all the configuration and the device information from the unit */
    transmit(request_all_configuration, sizeof(request_all_configuration));
    
    an_decoder_initialise(&an_decoder);
    
    packet_periods_packet_t packet = {
        .permanent = 0,
        .clear_existing_packets = 1,
        .packet_periods = {}};
    an_packet_transmit(encode_packet_periods_packet(&packet));
    
    _dev->get_semaphore()->give();
    return true;
}

bool AP_InertialSensor_ORIENTUS::init()
{   
    return hardware_init();
}

/*
  read packet
 */
void AP_InertialSensor_ORIENTUS::read_packet(void)
{
    int bytes_received;
    an_packet_transmit(encode_request_packet(0x1C));
    if ((bytes_received = receive(an_decoder_pointer(&an_decoder), an_decoder_size(&an_decoder))) > 0)
    { /* increment the decode buffer length by the number of bytes received */
        an_decoder_increment(&an_decoder, bytes_received);

        /* decode all the packets in the buffer */
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
            }
            /* Ensure that you free the an_packet when your done with it or you will leak memory */
            an_packet_free(&an_packet);
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
#if CONNECTION_TYPE == RS232
    return comWrite(comPortIndex, data, length);
#elif CONNECTION_TYPE == NETWORK
#if _WIN32
    return send(socket_fd, (char *)data, length, 0);
#else
    return write(socket_fd, data, length);
#endif
#endif
}

int AP_InertialSensor_ORIENTUS::receive(unsigned char *data, int length)
{
#if CONNECTION_TYPE == RS232
    return comRead(comPortIndex, data, length);
#elif CONNECTION_TYPE == NETWORK
#if _WIN32
    return recv(socket_fd, (char *)data, length, 0);
#else
    return read(socket_fd, data, length);
#endif
#endif
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