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

    nav_data.acc_x_ms2 = imu_ptr->accel_ms2[X] * 400.0f;
    nav_data.acc_y_ms2 = imu_ptr->accel_ms2[Y] * 400.0f;
    nav_data.acc_z_ms2 = imu_ptr->accel_ms2[Z] * 400.0f;

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
    
    spi_slave_transmit(SPI2_HOST, &trans, portMAX_DELAY);

    if (spi_heap_mem_receive[0] == HEADER && spi_heap_mem_receive[spi_trans_byte_size - 1] == FOOTER && checksum_verify(spi_heap_mem_receive, spi_trans_byte_size))
    {
        memcpy(&flight_data, spi_heap_mem_receive + 1, sizeof(flight_data_t));
        
        if (flight_ptr->arm_status == 0 && flight_data.arm_status == 1)
        {
            baro_ptr->gnd_press = baro_ptr->press;
            baro_ptr->init_temp = baro_ptr->temp;
            state_ptr->vel_forward_ms = 0;
            state_ptr->vel_right_ms = 0;
            state_ptr->vel_up_ms = 0;
        }
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
