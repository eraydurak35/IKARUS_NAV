#include <string.h>
#include "fc_comm.h"
#include "typedefs.h"
#include "nv_storage.h"
#include "driver/spi_slave.h"
#include "esp_heap_caps.h"




spi_slave_transaction_t trans;

static uint8_t *spi_heap_mem_receive = NULL;
static uint8_t *spi_heap_mem_send = NULL;
static nav_data_t nav_data;
static flight_data_t flight_data;
static const uint8_t spi_trans_byte_size = sizeof(nav_data_t) + 4;

static states_t *state_ptr = NULL;
static bmp390_t *baro_ptr = NULL;
static pmw3901_t *flow_ptr = NULL;
static lsm6dsl_t *imu_ptr = NULL;
static hmc5883l_t *mag_ptr = NULL;
static flight_t *flight_ptr = NULL;
static nav_config_t *config_ptr = NULL; 
static range_finder_t *range_ptr = NULL;


static void checksum_generate(uint8_t *data, uint8_t dataLenght, uint8_t *cs1, uint8_t *cs2);
static uint8_t checksum_verify(uint8_t *data, uint8_t dataLenght);

void flight_comm_init(states_t *sts, bmp390_t *bmp, pmw3901_t *pmw, lsm6dsl_t *lsm, hmc5883l_t *hmc, flight_t *flt, nav_config_t *cfg, range_finder_t *rng)
{
    state_ptr = sts;
    baro_ptr = bmp;
    flow_ptr = pmw;
    imu_ptr = lsm;
    mag_ptr = hmc;
    flight_ptr = flt;
    config_ptr = cfg;
    range_ptr = rng;

    spi_slave_interface_config_t slave_config =
        {
            .mode = 0,
            .spics_io_num = PIN_NUM_CS,
            .queue_size = 7,
            .flags = 0,
        };

    spi_bus_config_t bus_cfg =
        {
            .miso_io_num = PIN_NUM_MISO,
            .mosi_io_num = PIN_NUM_MOSI,
            .sclk_io_num = PIN_NUM_CLK,
            .quadhd_io_num = -1,
            .quadwp_io_num = -1,
            .max_transfer_sz = 0
        };
    spi_slave_initialize(SPI2_HOST, &bus_cfg, &slave_config, SPI_DMA_CH_AUTO);

    spi_heap_mem_receive = heap_caps_malloc(spi_trans_byte_size, MALLOC_CAP_DMA);
    spi_heap_mem_send = heap_caps_malloc(spi_trans_byte_size, MALLOC_CAP_DMA);

    memset(spi_heap_mem_receive, 0, spi_trans_byte_size);
    memset(spi_heap_mem_send, 0, spi_trans_byte_size);
}

void slave_send_recv_flight_comm()
{
    // prepare send packet

    nav_data.pitch = state_ptr->pitch_deg;
    nav_data.roll = state_ptr->roll_deg;
    nav_data.heading = state_ptr->heading_deg;
    nav_data.pitch_dps = state_ptr->pitch_dps;
    nav_data.roll_dps = state_ptr->roll_dps;
    nav_data.yaw_dps = state_ptr->yaw_dps;

    nav_data.barometer_temperature = baro_ptr->temp * 100.0f;
    nav_data.barometer_pressure = baro_ptr->press * 10.0f;
    nav_data.altitude = state_ptr->altitude_m * 100.0f;
    nav_data.baro_altitude = baro_ptr->altitude_m * 100.0f;
    nav_data.velocity_x_ms = state_ptr->vel_forward_ms * 1000.0f;
    nav_data.velocity_y_ms = state_ptr->vel_right_ms * 1000.0f;
    nav_data.velocity_z_ms = state_ptr->vel_up_ms * 1000.0f;
    nav_data.flow_x_velocity_ms = flow_ptr->velocity_x_ms * 1000.0f;
    nav_data.flow_y_velocity_ms = flow_ptr->velocity_y_ms * 1000.0f;
    nav_data.flow_quality = flow_ptr->quality;

    nav_data.imu_temperature = imu_ptr->temp_mC;
    nav_data.acc_x_ned_ms2 = state_ptr->acc_forward_ms2 * 10.0f;
    nav_data.acc_y_ned_ms2 = state_ptr->acc_right_ms2 * 10.0f;
    nav_data.acc_z_ned_ms2 = state_ptr->acc_up_ms2 * 10.0f;
    nav_data.acc_x_ms2 = imu_ptr->accel_ms2[X] * 100.0f;
    nav_data.acc_y_ms2 = imu_ptr->accel_ms2[Y] * 100.0f;
    nav_data.acc_z_ms2 = imu_ptr->accel_ms2[Z] * 100.0f;

    nav_data.mag_x_gauss = mag_ptr->axis[X] * 10.0f;
    nav_data.mag_y_gauss = mag_ptr->axis[Y] * 10.0f;
    nav_data.mag_z_gauss = mag_ptr->axis[Z] * 10.0f;


    memcpy(spi_heap_mem_send + 1, &nav_data, sizeof(nav_data_t));
    spi_heap_mem_send[0] = HEADER;
    static uint8_t checksum_a, checksum_b;
    checksum_generate(spi_heap_mem_send + 1, spi_trans_byte_size - 4, &checksum_a, &checksum_b);
    spi_heap_mem_send[spi_trans_byte_size - 3] = checksum_a;
    spi_heap_mem_send[spi_trans_byte_size - 2] = checksum_b;
    spi_heap_mem_send[spi_trans_byte_size - 1] = FOOTER;

    // Send data must be bigger than 8 and divisible by 4
    trans.length = spi_trans_byte_size * 8;
    trans.tx_buffer = spi_heap_mem_send;
    trans.rx_buffer = spi_heap_mem_receive;
    /* spi_slave_transmit(SPI3_HOST, &trans, portMAX_DELAY); */
    spi_slave_transmit(SPI2_HOST, &trans, portMAX_DELAY);

    if (spi_heap_mem_receive[0] == HEADER && spi_heap_mem_receive[spi_trans_byte_size - 1] == FOOTER && checksum_verify(spi_heap_mem_receive, spi_trans_byte_size))
    {
        memcpy(&flight_data, spi_heap_mem_receive + 1, sizeof(flight_data_t));
        
        flight_ptr->arm_status = flight_data.arm_status;
        range_ptr->range_cm = flight_data.range_cm;

        if (flight_data.is_new_config)
        {
            config_ptr->ahrs_filter_beta = flight_data.ahrs_filter_beta;
            config_ptr->ahrs_filter_zeta = flight_data.ahrs_filter_zeta;
            config_ptr->alt_filter_beta = flight_data.alt_filter_beta;
            config_ptr->mag_declination_deg = flight_data.mag_declination_deg;
            config_ptr->notch_1_bandwidth = flight_data.notch_1_bandwidth;
            config_ptr->notch_1_freq = flight_data.notch_1_freq;
            config_ptr->notch_2_bandwidth = flight_data.notch_2_bandwidth;
            config_ptr->notch_2_freq = flight_data.notch_2_freq;
            config_ptr->velxy_filter_beta = flight_data.velxy_filter_beta;
            config_ptr->velz_filter_beta = flight_data.velz_filter_beta;
            config_ptr->velz_filter_zeta = flight_data.velz_filter_zeta;

            save_config(config_ptr);
        }
    }
}

static void checksum_generate(uint8_t *data, uint8_t dataLenght, uint8_t *cs1, uint8_t *cs2)
{
    uint8_t checksum1 = 0, checksum2 = 0;

    for (uint8_t i = 0; i < dataLenght - 2; i++)
    {
        checksum1 = checksum1 + data[i];
        checksum2 = checksum2 + checksum1;
    }

    *cs1 = checksum1;
    *cs2 = checksum2;
}

static uint8_t checksum_verify(uint8_t *data, uint8_t dataLenght)
{
    uint8_t c1, c2;
    checksum_generate(data + 1, dataLenght - 4, &c1, &c2);
    if (c1 == data[dataLenght - 3] && c2 == data[dataLenght - 2])
        return 1;
    else
        return 0;
}






































/* #include "uart.h"

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
    // 24 + 2 header + 1 checksum + 1 footer = 28 bytes
    static uint8_t byteArray[sizeof(data_1_t) + 4];
    data1.pitch = state_ptr->pitch_deg;
    data1.roll = state_ptr->roll_deg;
    data1.heading = state_ptr->heading_deg;
    data1.pitch_dps = state_ptr->pitch_dps;
    data1.roll_dps = state_ptr->roll_dps;
    data1.yaw_dps = state_ptr->yaw_dps;

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
    data3.acc_x_ned_ms2 = state_ptr->acc_forward_ms2 * 10.0f;
    data3.acc_y_ned_ms2 = state_ptr->acc_right_ms2 * 10.0f;
    data3.acc_z_ned_ms2 = state_ptr->acc_up_ms2 * 10.0f;
    data3.acc_x_ms2 = imu_ptr->accel_ms2[X] * 100.0f;
    data3.acc_y_ms2 = imu_ptr->accel_ms2[Y] * 100.0f;
    data3.acc_z_ms2 = imu_ptr->accel_ms2[Z] * 100.0f;

    data3.mag_x_gauss = mag_ptr->axis[X] * 10.0f;
    data3.mag_y_gauss = mag_ptr->axis[Y] * 10.0f;
    data3.mag_z_gauss = mag_ptr->axis[Z] * 10.0f;

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
    state_ptr->vel_forward_ms = 0;
    state_ptr->vel_right_ms = 0;
  }
  prev_flight = *flight_ptr;
}
 */