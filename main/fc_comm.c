#include <string.h>
#include "fc_comm.h"
#include "uart.h"
#include "typedefs.h"
#include "nv_storage.h"


static flight_t *flight_ptr;
static flight_t prev_flight;
static range_finder_t *range_ptr;
static nav_config_t *config_ptr;
static states_t *state_ptr;
static lsm6dsl_t *imu_ptr;
static hmc5883l_t *mag_ptr;
static pmw3901_t *flow_ptr;
static bmp390_t *baro_ptr;

static data_1_t data1;
static data_2_t data2;
static data_3_t data3;

static uint8_t validate_checksum(uint8_t *buff, uint8_t size);
static void create_packet(uint8_t *write_buff, uint8_t size, uint8_t header1, uint8_t header2, uint8_t footer);

void fc_comm_init(states_t *std, flight_t *flt, range_finder_t *rng, nav_config_t *cfg, lsm6dsl_t *lsm, hmc5883l_t *hmc, pmw3901_t *pmw, bmp390_t *bro)
{
    flight_ptr = flt;
    range_ptr = rng;
    config_ptr = cfg;
    state_ptr = std;
    imu_ptr = lsm;
    mag_ptr = hmc;
    flow_ptr = pmw;
    baro_ptr = bro;
}

void parse_fc_data(uart_data_t *uart_buff)
{
    static const uint8_t data1_size = sizeof(nav_config_t);
    static const uint8_t data2_size = sizeof(flight_t);
    static const uint8_t data3_size = sizeof(range_finder_t);
    static uint8_t isHeader1Found = 0;
    static uint8_t isHeader2Found = 0;
    static uint8_t isHeader3Found = 0;
    static uint8_t read_buffer[50];
    static uint8_t byte_counter = 0;
    static uint8_t prev_data_byte;

    for (uint8_t i = 0; i < uart_buff->lenght; i++)
    {
        

        if (isHeader1Found == 1)
        {
            read_buffer[byte_counter++] = uart_buff->data[i];

            if (byte_counter == data1_size + 2)
            {
                byte_counter = 0;
                isHeader1Found = 0;
                if (read_buffer[data1_size + 1] == ins_config_footer)
                {
                    if (validate_checksum(read_buffer, data1_size) == 1)
                    {
                        memcpy(config_ptr, read_buffer, data1_size);
                        save_config(config_ptr);
                    }
                }
            }
        }
        else if (isHeader2Found == 1)
        {
            read_buffer[byte_counter++] = uart_buff->data[i];

            if (byte_counter == data2_size + 2)
            {
                byte_counter = 0;
                isHeader2Found = 0;
                if (read_buffer[data2_size + 1] == flight_footer)
                {
                    if (validate_checksum(read_buffer, data2_size) == 1)
                    {
                        memcpy(flight_ptr, read_buffer, data2_size);
                        check_flight_status();
                        
                    }
                }
            }
        }
        else if (isHeader3Found == 1)
        {
            read_buffer[byte_counter++] = uart_buff->data[i];

            if (byte_counter == data3_size + 2)
            {
                byte_counter = 0;
                isHeader3Found = 0;
                if (read_buffer[data3_size + 1] == range_finder_footer)
                {
                    if (validate_checksum(read_buffer, data3_size) == 1)
                    {
                        memcpy(range_ptr, read_buffer, data3_size);
                        //printf("%d\n", range_ptr->range_cm);
                    }
                }
            }
        }
        else if (uart_buff->data[i] == ins_config_header2 && prev_data_byte == ins_config_header1)
        {
            isHeader1Found = 1;
        }
        else if (uart_buff->data[i] == flight_header2 && prev_data_byte == flight_header1)
        {
            isHeader2Found = 1;
        }
        else if (uart_buff->data[i] == range_finder_header2 && prev_data_byte == range_finder_header1)
        {
            isHeader3Found = 1;
        }

        prev_data_byte = uart_buff->data[i];
    }
}

void send_data1()
{
    // 12 + 2 header + 1 checksum + 1 footer = 16 bytes

    ///////         INT16 WILL OVERFLOW !!!!!!!!!!!!
    static uint8_t byteArray[sizeof(data_1_t) + 4];
    data1.pitch = constrain(state_ptr->pitch_deg * 360.0f, INT16_MIN, INT16_MAX);
    data1.roll = constrain(state_ptr->roll_deg * 360.0f, INT16_MIN, INT16_MAX);
    data1.heading = constrain(state_ptr->heading_deg * 180.0f, 0, UINT16_MAX);
    data1.pitch_dps = constrain(state_ptr->pitch_dps * 100.0f, INT16_MIN, INT16_MAX);
    data1.roll_dps = constrain(state_ptr->roll_dps * 100.0f, INT16_MIN, INT16_MAX);
    data1.yaw_dps = constrain(state_ptr->yaw_dps * 100.0f, INT16_MIN, INT16_MAX);

    memcpy(byteArray + 2, &data1, sizeof(data_1_t));
    create_packet(byteArray, sizeof(data_1_t), data1_header1, data1_header2, data1_footer);
    uart_write(UART_NUM_1, byteArray, sizeof(data_1_t) + 4);
}
void send_data2()
{
    // 20 + 2 header + 1 checksum + 1 footer = 24 bytes
    static uint8_t byteArray[sizeof(data_2_t) + 4];
    data2.barometer_temperature = baro_ptr->temp * 100.0f;
    data2.barometer_pressure = baro_ptr->press * 10.0f;
    data2.altitude = state_ptr->altitude_m * 100.0f;
    data2.baro_altitude = baro_ptr->altitude_m * 100.0f;
    data2.velocity_x_ms = state_ptr->vel_forward_ms * 1000.0f;
    data2.velocity_y_ms = state_ptr->vel_right_ms * 1000.0f;
    data2.velocity_z_ms = state_ptr->vel_up_ms * 1000.0f;
    data2.flow_x_velocity_ms = flow_ptr->velocity_x_ms * 1000.0f;
    data2.flow_y_velocity_ms = flow_ptr->velocity_y_ms * 1000.0f;

    //printf("x: %.2f     y: %.2f\n", flow_ptr->velocity_x_ms, flow_ptr->velocity_y_ms);
    data2.flow_quality = flow_ptr->quality;

    memcpy(byteArray + 2, &data2, sizeof(data_2_t));
    create_packet(byteArray, sizeof(data_2_t), data2_header1, data2_header2, data2_footer);
    uart_write(UART_NUM_1, byteArray, sizeof(data_2_t) + 4);
}
void send_data3()
{
    // 20 + 2 header + 1 checksum + 1 footer = 24 bytes
    static uint8_t byteArray[sizeof(data_3_t) + 4];
    data3.imu_temperature = imu_ptr->temp_mC;
    data3.acc_x_ned_ms2 = state_ptr->acc_forward_ms2 * 10.0;
    data3.acc_y_ned_ms2 = state_ptr->acc_right_ms2 * 10.0;
    data3.acc_z_ned_ms2 = state_ptr->acc_up_ms2 * 10.0;
    data3.acc_x_ms2 = imu_ptr->accel_ms2[X] * 100.0;
    data3.acc_y_ms2 = imu_ptr->accel_ms2[Y] * 100.0;
    data3.acc_z_ms2 = imu_ptr->accel_ms2[Z] * 100.0;

    data3.mag_x_gauss = mag_ptr->axis[X] * 10.0;
    data3.mag_y_gauss = mag_ptr->axis[Y] * 10.0;
    data3.mag_z_gauss = mag_ptr->axis[Z] * 10.0;

    memcpy(byteArray + 2, &data3, sizeof(data_3_t));
    create_packet(byteArray, sizeof(data_3_t), data3_header1, data3_header2, data3_footer);
    uart_write(UART_NUM_1, byteArray, sizeof(data_3_t) + 4);
}

static uint8_t validate_checksum(uint8_t *buff, uint8_t size)
{
    uint8_t checksum = 0;
    for (uint8_t idx = 0; idx < size; idx++)
    {
        checksum ^= buff[idx];
    }
    if (checksum == buff[size])
    {
        return 1;
    }
    return 0;
}

static void create_packet(uint8_t *write_buff, uint8_t size, uint8_t header1, uint8_t header2, uint8_t footer)
{
    uint8_t checksum = 0;
    write_buff[0] = header1;
    write_buff[1] = header2;
    for (uint8_t idx = 2; idx < size + 2; idx++)
    {
        checksum ^= write_buff[idx];
    }
    write_buff[size + 2] = checksum;
    write_buff[size + 3] = footer;
}


void check_flight_status()
{
  if (flight_ptr->arm_status != prev_flight.arm_status && flight_ptr->arm_status == 1)
  {
    baro_ptr->gnd_press = baro_ptr->press;
    baro_ptr->init_temp = baro_ptr->temp;
    state_ptr->altitude_m = 0;
    state_ptr->vel_up_ms = 0;
  }
  prev_flight = *flight_ptr;
}
