/*
 *    8888888 888    d8P         d8888 8888888b.  888     888  .d8888b.                                  
 *      888   888   d8P         d88888 888   Y88b 888     888 d88P  Y88b                                 
 *      888   888  d8P         d88P888 888    888 888     888 Y88b.                                      
 *      888   888d88K         d88P 888 888   d88P 888     888  "Y888b.        88888b.   8888b.  888  888 
 *      888   8888888b       d88P  888 8888888P"  888     888     "Y88b.      888 "88b     "88b 888  888 
 *      888   888  Y88b     d88P   888 888 T88b   888     888       "888      888  888 .d888888 Y88  88P 
 *      888   888   Y88b   d8888888888 888  T88b  Y88b. .d88P Y88b  d88P      888  888 888  888  Y8bd8P  
 *    8888888 888    Y88b d88P     888 888   T88b  "Y88888P"   "Y8888P"       888  888 "Y888888   Y88P   
 */
// ||############################||
// ||      ESP IDF LIBRARIES     ||
// ||############################||
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_system.h"
#include "driver/i2c.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include <stdio.h>
// ||############################||
// ||      CUSTOM LIBRARIES      ||
// ||############################||
#include "state_estimator.h"
#include "noise_analize.h"
#include "nv_storage.h"
#include "typedefs.h"
#include "qmc5883l.h"
#include "lsm6dsl.h"
#include "pmw3901.h"
#include "filters.h"
#include "fc_comm.h"
#include "bmp390.h"
#include "gpio.h"
#include "uart.h"
#include "i2c.h"
// ||############################||
// ||      GLOBAL WARIABLES      ||
// ||############################||
static TaskHandle_t task1_handler;
static TaskHandle_t task2_handler;
static TaskHandle_t task3_handler;
static TaskHandle_t task4_handler;
static TaskHandle_t task5_handler;
static lsm6dsl_t imu;
static lsm6dsl_t unfilt_imu;
static lsm6dsl_t lpf_imu;
static biquad_lpf_t lpf[6];
static notch_filter_t notch[18];
static fft_t fft[6];
static states_t state;
static nav_config_t config;
static magnetometer_t mag;
static magnetometer_t uncalib_mag;
static bmp390_t baro;
static pmw3901_t flow;
static flight_t flight;
static range_finder_t range_finder = {-1};
static float mag_calib_data[12] = {0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
static float acc_runtime_bias[2] = {0.0f, 0.0f};

void IRAM_ATTR timer1_callback(void *arg)
{
    xTaskNotifyFromISR(task1_handler, 1, eIncrement, false);
}

void task_1(void *pvParameters)
{
    static uint32_t notification = 0;
    static uint8_t counter1 = 0;
    static uint8_t counter2 = 0;
    static uint8_t counter3 = 0;
    imu.gyro_bias_dps[X] = 2.1f;
    imu.gyro_bias_dps[Y] = -3.05f;
    imu.gyro_bias_dps[Z] = -1.1f;
    biquad_lpf_array_init(lpf);
    notch_filter_init(notch);
    noise_analize_init(fft);

    i2c_master_init(I2C_NUM_0, GPIO_NUM_21, GPIO_NUM_22, 1000000, GPIO_PULLUP_DISABLE);
    lsm6dsl_setup(acc_runtime_bias);

    for (uint8_t i = 0; i < 200; i++)
    {
        lsmldsl_read(&imu);
        apply_biquad_lpf_to_imu(&imu, lpf);
        apply_notch_filter_to_imu(&imu, notch);
        vTaskDelay(3 / portTICK_PERIOD_MS);
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    estimator_init(&config, &state, &imu, &mag, &baro, &flow, &range_finder, &flight);

    while (1) // 1000 Hz
    {
        if (xTaskNotifyWait(0, ULONG_MAX, &notification, 1 / portTICK_PERIOD_MS) == pdTRUE)
        {
            lsmldsl_read(&imu);
            unfilt_imu = imu;
            apply_biquad_lpf_to_imu(&imu, lpf);
            lpf_imu = imu;
            apply_notch_filter_to_imu(&imu, notch);
            ahrs_predict();
            ahrs_correct();
            get_earth_frame_accel();
            predict_velocityXY();
            estimate_altitude_velocity();

            counter1++;
            if (counter1 >= 2) // 500 Hz
            {
                counter1 = 0;
                sample_imu_to_analize(&lpf_imu, fft);
            }
            else // 500 Hz
            {
                counter2++;
                if (counter2 >= 5) // 100 Hz
                {
                    counter2 = 0;
                    get_flow_velocity();
                    correct_velocityXY();
                }
                else // 400 Hz
                {
                    counter3++;
                    if (counter3 >= 16) // 25 Hz
                    {
                        counter3 = 0;
                        reconfig_all_notch_filters(notch, fft);
                    }
                }
            }
        }
    }
}

void task_2(void *pvParameters)
{
    while (1)
    {
        analize_imu_noise(fft);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void task_3(void *pvParameters)
{
    i2c_master_init(I2C_NUM_1, GPIO_NUM_33, GPIO_NUM_32, 400000, GPIO_PULLUP_DISABLE);
    qmc5883l_setup(mag_calib_data);
    bmp390_setup_i2c();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    baro_get_ground_pressure(&baro);

    while (1)
    {
        qmc5883l_read(&mag, &uncalib_mag, flight.throttle);
        bmp390_read_i2c(&baro);
        get_baro_altitude(&baro);
        vTaskDelay(19 / portTICK_PERIOD_MS);
    }
}

void task_4(void *pvParameters)
{
    uart_begin(UART_NUM_2, 19200, 17, 16, UART_PARITY_DISABLE);
    static uart_data_t uart2_buff;

    while (1)
    {
        uart_read(UART_NUM_2, &uart2_buff, 5);
        parse_pmw3901_data(&flow, &uart2_buff);
    }
}


void task_5(void *pvParameters)
{
    flight_comm_init(&state, &baro, &flow, &unfilt_imu, &uncalib_mag, &flight, &config, &range_finder, mag_calib_data, acc_runtime_bias);

    while (1)
    {
        slave_send_recv_flight_comm();
    }
}
void app_main(void)
{
    nvs_flash_init();
    read_config(&config);
    read_mag_cal(mag_calib_data);
    read_acc_cal(acc_runtime_bias);

    xTaskCreatePinnedToCore(&task_3, "task3", 1024 * 2, NULL, 1, &task3_handler, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(&task_4, "task4", 1024 * 8, NULL, 1, &task4_handler, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(&task_1, "task1", 1024 * 4, NULL, 1, &task1_handler, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(&task_2, "task2", 1024 * 8, NULL, 1, &task2_handler, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(&task_5, "task5", 1024 * 4, NULL, 1, &task5_handler, tskNO_AFFINITY);

    esp_timer_handle_t timer1;
    const esp_timer_create_args_t timer1_args =
    {
        .callback = &timer1_callback,
        .arg = NULL,
        .name = "timer1"
    };
    esp_timer_create(&timer1_args, &timer1);
    esp_timer_start_periodic(timer1, 1000);
}
