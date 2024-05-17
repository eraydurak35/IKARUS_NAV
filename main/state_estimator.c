#include "state_estimator.h"
#include "math.h"
#include "quaternion.h"
#include "typedefs.h"

// An Extended Complementary Filter (ECF) for Full-Body MARG Orientation Estimation
// DOI:10.1109/TMECH.2020.2992296

static quat_t q = {1.0f, 0.0f, 0.0f, 0.0f};
static quat_t q_dot = {0.0f, 0.0f, 0.0f, 0.0f};
static vector_t gyr_vec = {0.0f, 0.0f, 0.0f};
static vector_t acc_vec = {0.0f, 0.0f, 1.0f};
static vector_t mag_vec = {1.0f, 0.0f, 0.0f};
static vector_t err = {0.0f, 0.0f, 0.0f};
static vector_t local_vr_a = {0.0f, 0.0f, 0.0f};
static vector_t local_vr_m = {0.0f, 0.0f, 0.0f};

static states_t *state_ptr = NULL;
static lsm6dsl_t *imu_ptr = NULL;
static magnetometer_t *mag_ptr = NULL;
static bmp390_t *baro_ptr = NULL;
static pmw3901_t *flow_ptr = NULL;
static range_finder_t *range_ptr = NULL;
static nav_config_t *config_ptr = NULL;
static flight_t *flight_ptr = NULL;

static float acc_up_bias = 0.0f;
static float rotation_matrix[3][3];
static float observed_body_accel_correction[3];
static float observed_earth_accel_forward_ms2;
static float observed_earth_accel_right_ms2;
static float observed_earth_accel_up_ms2;

static void get_attitude_heading();
static void get_heading_free_rotation_matrix();
static void calculate_earth_observed_acceleration();
static uint8_t is_movement_detected();
static float predict_z_centrifugal_accel_ms2();
static void limit_symmetric(float *value, float limit);
static uint8_t movement_state = 0;

void estimator_init(nav_config_t *cfg, states_t *sta, lsm6dsl_t *lsm, magnetometer_t *hmc, bmp390_t *baro, pmw3901_t *flw, range_finder_t *rng, flight_t *flt)
{
    config_ptr = cfg;
    state_ptr = sta;
    imu_ptr = lsm;
    mag_ptr = hmc;
    baro_ptr = baro;
    flow_ptr = flw;
    range_ptr = rng;
    flight_ptr = flt;

    acc_vec.x = imu_ptr->accel_ms2[X];
    acc_vec.y = imu_ptr->accel_ms2[Y];
    acc_vec.z = imu_ptr->accel_ms2[Z];

    mag_vec.x = mag_ptr->axis[X];
    mag_vec.y = mag_ptr->axis[Y];
    mag_vec.z = mag_ptr->axis[Z];

    // initialize quaternion attitude from accelerometer and magnetometer
    get_quat_from_vector_measurements(&acc_vec, &mag_vec, &q);
}

void ahrs_predict()
{   
    movement_state = is_movement_detected();

    gyr_vec.x = imu_ptr->gyro_dps[Y] * DEG_TO_RAD;
    gyr_vec.y = imu_ptr->gyro_dps[X] * DEG_TO_RAD;
    gyr_vec.z = -imu_ptr->gyro_dps[Z] * DEG_TO_RAD;

    // if not armed and no movement use higher gains
    if (flight_ptr->arm_status == 0 && movement_state == 0)
    {
        gyr_vec.x -= (0.4f * err.x);
        gyr_vec.y -= (0.4f * err.y);
        gyr_vec.z -= (0.4f * err.z);
    }
    else
    {
        gyr_vec.x -= (config_ptr->ahrs_filter_beta * err.x);
        gyr_vec.y -= (config_ptr->ahrs_filter_beta * err.y);
        gyr_vec.z -= (config_ptr->ahrs_filter_beta * err.z);
    }

    // calculate quaternion deriavative
    get_quat_deriv(&q, &gyr_vec, &q_dot);

    // integrate rate of change of quaternion to yield quaternion attitude
    q.w += q_dot.w / 1000.0f;
    q.x += q_dot.x / 1000.0f;
    q.y += q_dot.y / 1000.0f;
    q.z += q_dot.z / 1000.0f;

    norm_quat(&q);
}

static void get_attitude_heading()
{
    static quat_t q_conj;
    static vector_t euler;

    q_conj = quat_conj(&q);
    quat_to_euler(&q_conj, &euler);

    state_ptr->pitch_deg = euler.x;
    state_ptr->roll_deg = euler.y;
    state_ptr->heading_deg = euler.z - config_ptr->mag_declination_deg;

    if (state_ptr->heading_deg < 0) state_ptr->heading_deg += 360.0f;
    else if (state_ptr->heading_deg > 360.0f) state_ptr->heading_deg -= 360.0f;

    state_ptr->pitch_dps = imu_ptr->gyro_dps[X];
    state_ptr->roll_dps = imu_ptr->gyro_dps[Y];
    state_ptr->yaw_dps = -imu_ptr->gyro_dps[Z];
}

void ahrs_correct()
{
    static vector_t err_acc = {0.0f, 0.0f, 0.0f};
    static vector_t err_mag = {0.0f, 0.0f, 0.0f};
    static vector_t temp = {0.0f, 0.0f, 0.0f};
    static uint8_t prev_movement_state = 0;

    calculate_earth_observed_acceleration();

    acc_vec.x = imu_ptr->accel_ms2[Y] - observed_body_accel_correction[Y];
    acc_vec.y = imu_ptr->accel_ms2[X] - observed_body_accel_correction[X];
    acc_vec.z = -(imu_ptr->accel_ms2[Z] - observed_body_accel_correction[Z]);

    mag_vec.x = mag_ptr->axis[Y];
    mag_vec.y = -mag_ptr->axis[X];
    mag_vec.z = -mag_ptr->axis[Z];

    norm_vector(&acc_vec);
    norm_vector(&mag_vec);

    local_vr_a.x = 2.0f * ((q.x * q.z) - (q.w * q.y));
    local_vr_a.y = 2.0f * ((q.w * q.x) + (q.y * q.z));
    local_vr_a.z = 2.0f * ((q.w * q.w) + (q.z * q.z)) - 1.0f;

    err_acc = cross_product(&acc_vec, &local_vr_a);

    local_vr_m.x = 2.0f * ((q.x * q.y) + (q.w * q.z));
    local_vr_m.y = 2.0f * ((q.w * q.w) + (q.y * q.y)) - 1.0f;
    local_vr_m.z = 2.0f * ((q.y * q.z) - (q.w * q.x));

    temp = cross_product(&acc_vec, &mag_vec);
    err_mag = cross_product(&temp, &local_vr_m);

    err.x = err_acc.x + err_mag.x;
    err.y = err_acc.y + err_mag.y;
    err.z = err_acc.z + err_mag.z;

    // if not armed, transitioning from movement to no movement
    // set heading to mag heading
    if (flight_ptr->arm_status == 0 && (movement_state == 0 && prev_movement_state == 1))
    {
        mag_vec.x = mag_ptr->axis[X];
        mag_vec.y = mag_ptr->axis[Y];
        mag_vec.z = mag_ptr->axis[Z];
        set_heading_quat(state_ptr->pitch_deg, state_ptr->roll_deg, &mag_vec, &q);
    }
    prev_movement_state = movement_state;

    // ground can distort magnetic heading. we need to reset magnetic heading when we are away from ground some distance
    if (flight_ptr->is_in_flight_mag_allign_done == 0 && flight_ptr->arm_status == 1 && state_ptr->altitude_m > IN_FLT_MAG_ALLN_ALT)
    {
        mag_vec.x = mag_ptr->axis[X];
        mag_vec.y = mag_ptr->axis[Y];
        mag_vec.z = mag_ptr->axis[Z];
        set_heading_quat(state_ptr->pitch_deg, state_ptr->roll_deg, &mag_vec, &q);
        flight_ptr->is_in_flight_mag_allign_done = 1;
    }

    // if not armed and no movement use higher gains
    if (flight_ptr->arm_status == 0 && movement_state == 0)
    {
        imu_ptr->gyro_bias_dps[Y] += err.x * 0.001f;
        imu_ptr->gyro_bias_dps[X] += err.y * 0.001f;
        imu_ptr->gyro_bias_dps[Z] -= err.z * 0.001f;
    }
    else if (flight_ptr->arm_status == 1)
    {
        imu_ptr->gyro_bias_dps[Y] += err.x * config_ptr->ahrs_filter_zeta;
        imu_ptr->gyro_bias_dps[X] += err.y * config_ptr->ahrs_filter_zeta;
        imu_ptr->gyro_bias_dps[Z] -= err.z * config_ptr->ahrs_filter_zeta;
    }

    get_attitude_heading();
}

void get_earth_frame_accel()
{
    get_heading_free_rotation_matrix();

    state_ptr->acc_forward_ms2 = imu_ptr->accel_ms2[Y] * rotation_matrix[0][0] + imu_ptr->accel_ms2[X] * rotation_matrix[1][0] + imu_ptr->accel_ms2[Z] * rotation_matrix[2][0];
    state_ptr->acc_right_ms2 = imu_ptr->accel_ms2[Y] * rotation_matrix[0][1] + imu_ptr->accel_ms2[X] * rotation_matrix[1][1] + imu_ptr->accel_ms2[Z] * rotation_matrix[2][1];
    state_ptr->acc_up_ms2 = (imu_ptr->accel_ms2[Y] * rotation_matrix[0][2] + imu_ptr->accel_ms2[X] * rotation_matrix[1][2] + imu_ptr->accel_ms2[Z] * rotation_matrix[2][2]) - 9.806f;
}

void correct_velocityXY()
{
    float diff_x = flow_ptr->velocity_x_ms - state_ptr->vel_forward_ms;
    float diff_y = flow_ptr->velocity_y_ms - state_ptr->vel_right_ms;

    limit_symmetric(&diff_x, 1.0f);
    limit_symmetric(&diff_y, 1.0f);

    state_ptr->vel_forward_ms += diff_x * (1.0f - config_ptr->velxy_filter_beta);
    state_ptr->vel_right_ms += diff_y * (1.0f - config_ptr->velxy_filter_beta);
}

void predict_velocityXY() // 1000Hz
{
    static const float dt = 0.001f;
    static float flow_accel_forward_ms2 = 0;
    static float flow_accel_right_ms2 = 0;
    static float prev_vel_forward_ms = 0;
    static float prev_vel_right_ms = 0;

    state_ptr->vel_forward_ms += state_ptr->acc_forward_ms2 * dt;
    state_ptr->vel_right_ms += state_ptr->acc_right_ms2 * dt;

    state_ptr->vel_forward_ms -= state_ptr->vel_right_ms * sinf(-state_ptr->yaw_dps * DEG_TO_RAD * dt);
    state_ptr->vel_right_ms += state_ptr->vel_forward_ms * sinf(-state_ptr->yaw_dps * DEG_TO_RAD * dt);

    flow_accel_forward_ms2 = (state_ptr->vel_forward_ms - prev_vel_forward_ms) / dt;
    flow_accel_right_ms2 = (state_ptr->vel_right_ms - prev_vel_right_ms) / dt;

    observed_earth_accel_forward_ms2 += (flow_accel_forward_ms2 - observed_earth_accel_forward_ms2) * 0.15f;
    observed_earth_accel_right_ms2 += (flow_accel_right_ms2 - observed_earth_accel_right_ms2) * 0.15f;

    prev_vel_forward_ms = state_ptr->vel_forward_ms;
    prev_vel_right_ms = state_ptr->vel_right_ms;
}

static void calculate_earth_observed_acceleration()
{
    get_heading_free_rotation_matrix();

    observed_body_accel_correction[Y] = observed_earth_accel_forward_ms2 * rotation_matrix[0][0] + observed_earth_accel_right_ms2 * rotation_matrix[0][1] + observed_earth_accel_up_ms2 * rotation_matrix[0][2];
    observed_body_accel_correction[X] = observed_earth_accel_forward_ms2 * rotation_matrix[1][0] + observed_earth_accel_right_ms2 * rotation_matrix[1][1] + observed_earth_accel_up_ms2 * rotation_matrix[1][2];
    observed_body_accel_correction[Z] = observed_earth_accel_forward_ms2 * rotation_matrix[2][0] + observed_earth_accel_right_ms2 * rotation_matrix[2][1] + observed_earth_accel_up_ms2 * rotation_matrix[2][2];
}

void get_flow_velocity()
{
    static float filt_gyr_degs_pitch;
    static float filt_gyr_degs_roll;
    // Additional filtering required to match the phases of gyro and opt flow sensor
    // 11.547 is calibration gain (degs / gain) has to be equal flow sensor output
    filt_gyr_degs_roll += ((state_ptr->roll_dps / 11.5474487f) - filt_gyr_degs_roll) * 0.24f;
    filt_gyr_degs_pitch += ((state_ptr->pitch_dps / 11.3375732f) - filt_gyr_degs_pitch) * 0.24f;

    // Flow sensor outputs cpi value. Height info needed to calculate velocity in m/s
    // This compansates the sensor output for roll and pitch and filters
    flow_ptr->filt_x_cpi += ((flow_ptr->raw_x_cpi + filt_gyr_degs_roll) - flow_ptr->filt_x_cpi) * 0.2f;
    flow_ptr->filt_y_cpi += ((flow_ptr->raw_y_cpi + filt_gyr_degs_pitch) - flow_ptr->filt_y_cpi) * 0.2f;

    if ((range_ptr->range_cm / 100.0f) < RANGE_BARO_TRANS_END_ALT)
    {
        flow_ptr->velocity_y_ms = flow_ptr->filt_x_cpi * ((range_ptr->range_cm / 100.0f) / 5.0f);
        flow_ptr->velocity_x_ms = flow_ptr->filt_y_cpi * ((range_ptr->range_cm / 100.0f) / -5.0f);
    }
    else
    {
        flow_ptr->velocity_y_ms = flow_ptr->filt_x_cpi * (state_ptr->altitude_m / 5.0f);
        flow_ptr->velocity_x_ms = flow_ptr->filt_y_cpi * (state_ptr->altitude_m / -5.0f);
    }

    // 4cm away from center of rotation correction via angular speed to linear speed calculation
    // y axis is not needed
    flow_ptr->velocity_x_ms -= (state_ptr->yaw_dps * DEG_TO_RAD) * 0.045f;
}

void estimate_altitude_velocity() // 1000Hz
{
    static uint8_t init = 1;
    static float baro_alt_offset_m;
    static float baro_altitude_m;
    static float range_altitude_m = -1;
    static float prev_baro_altitude_m;
    float rangefinder_transition_gain;
    float velocity_accel_ms;
    static float accel_altitude_m;
    float baro_velocity_ms;
    static const float dt = 0.001f;
    static float upsampled_baro_altitude_m;
    static float upsampled_range_altitude_m;
    static float filt_range = 0.0f;
    static float range_m = 0;

    range_m = range_ptr->range_cm / 100.0f;
    filt_range += (range_m - filt_range) * 0.02f;
    float diff = range_m - filt_range;
    if (diff > 0.3 || diff < -0.3) filt_range += diff * 0.3f;
    upsampled_range_altitude_m += (filt_range - upsampled_range_altitude_m) * 0.1f; // updates 100 Hz

    //printf("%.2f,%.2f\n", (range_ptr->range_cm / 100.0f), upsampled_range_altitude_m);

    upsampled_baro_altitude_m += (baro_ptr->altitude_m - upsampled_baro_altitude_m) * 0.05f; // updates 50 Hz
    baro_altitude_m = upsampled_baro_altitude_m;

    if (upsampled_range_altitude_m > RANGE_BARO_TRANS_END_ALT) range_altitude_m = -1.0;
    else range_altitude_m = upsampled_range_altitude_m;

    //////////   DISABLE RANGE FINDER FOR TEST   //////////////
    //range_altitude_m = -1;
    //////////   DISABLE RANGE FINDER FOR TEST   //////////////

    if (range_altitude_m >= 0 && range_altitude_m < RANGE_BARO_TRANS_START_ALT)
    {
        baro_alt_offset_m = baro_altitude_m - range_altitude_m;
        baro_altitude_m = range_altitude_m;
    }
    else
    {
        baro_altitude_m -= baro_alt_offset_m;
        if (range_altitude_m > 0)
        {
            rangefinder_transition_gain = (RANGE_BARO_TRANS_END_ALT - range_altitude_m) * (RANGE_BARO_TRANS_END_ALT / (RANGE_BARO_TRANS_END_ALT - RANGE_BARO_TRANS_START_ALT));
            baro_altitude_m = range_altitude_m * rangefinder_transition_gain + baro_altitude_m * (1.0f - rangefinder_transition_gain);
        }
    }


    velocity_accel_ms = ((state_ptr->acc_up_ms2 + predict_z_centrifugal_accel_ms2()) - acc_up_bias) * dt;
    accel_altitude_m += (velocity_accel_ms * 0.5f) * dt + state_ptr->vel_up_ms * dt;
    accel_altitude_m = accel_altitude_m * config_ptr->alt_filter_beta + baro_altitude_m * (1.0f - config_ptr->alt_filter_beta);
    state_ptr->vel_up_ms += velocity_accel_ms;

    if (range_altitude_m >= 0 && range_altitude_m < RANGE_BARO_TRANS_START_ALT) state_ptr->altitude_m = baro_altitude_m;
    else state_ptr->altitude_m = accel_altitude_m;

    if (init == 1)
    {
        prev_baro_altitude_m = baro_altitude_m;
        init = 0;
    }

    //printf("%.2f,%.2f\n", state_ptr->altitude_m, baro_altitude_m);

    baro_velocity_ms = (baro_altitude_m - prev_baro_altitude_m) / dt;
    prev_baro_altitude_m = baro_altitude_m;

    float velDiff;
    velDiff = baro_velocity_ms - state_ptr->vel_up_ms;

    limit_symmetric(&velDiff, 2.0f);

    state_ptr->vel_up_ms += velDiff * config_ptr->velz_filter_beta * dt;
    acc_up_bias -= velDiff * config_ptr->velz_filter_zeta * dt * dt;

    limit_symmetric(&acc_up_bias, ACC_UP_BIAS_PREDICT_LIM);

    //printf("%.2f,%.2f\n", baro_velocity_ms, state_ptr->vel_up_ms);

    static float prev_velocity_up = 0;
    static float accel_up_ms2 = 0;

    accel_up_ms2 = (state_ptr->vel_up_ms - prev_velocity_up) / dt;
    prev_velocity_up = state_ptr->vel_up_ms;
    observed_earth_accel_up_ms2 += (accel_up_ms2 - observed_earth_accel_up_ms2) * 0.15f;
}


static void get_heading_free_rotation_matrix()
{
    static float pitch_radians = 0.0f;
    static float roll_radians = 0.0f;
    static float cosx = 0.0f;
    static float sinx = 0.0f;
    static float cosy = 0.0f;
    static float siny = 0.0;

    pitch_radians = state_ptr->pitch_deg * DEG_TO_RAD;
    roll_radians = state_ptr->roll_deg * DEG_TO_RAD;

    cosx = cosf(roll_radians);
    sinx = sinf(roll_radians);
    cosy = cosf(pitch_radians);
    siny = sinf(pitch_radians);

    rotation_matrix[0][0] = cosy;
    rotation_matrix[0][1] = 0.0f;
    rotation_matrix[0][2] = siny;
    rotation_matrix[1][0] = sinx * siny;
    rotation_matrix[1][1] = cosx;
    rotation_matrix[1][2] = -sinx * cosy;
    rotation_matrix[2][0] = -(cosx * siny);
    rotation_matrix[2][1] = sinx;
    rotation_matrix[2][2] = cosy * cosx;
}

static uint8_t is_movement_detected()
{
    static float gyro_vector = 0;
    static float filt_vector = 0;
    gyro_vector = imu_ptr->gyro_dps[X] * imu_ptr->gyro_dps[X] + imu_ptr->gyro_dps[Y] * imu_ptr->gyro_dps[Y] + imu_ptr->gyro_dps[Z] * imu_ptr->gyro_dps[Z];
    if (gyro_vector > 2000.0f) gyro_vector = 2000.0f;
    filt_vector += (gyro_vector - filt_vector) * 0.001f;
    return (filt_vector > GYRO_MOVEMENT_DETECT_THRESHOLD) ? 1 : 0;;
}

static float predict_z_centrifugal_accel_ms2()
{
    // sudden pitch roll change creates centrifugal accel
    // this couses z velocity pedictions to drift to negative values
    // 0.6 m/s2 centrifugal accel is mesasured at 80 deg/s 
    return (sqrtf(state_ptr->roll_dps * state_ptr->roll_dps + state_ptr->pitch_dps * state_ptr->pitch_dps) * 0.6f) / 80.0f;
}

static void limit_symmetric(float *value, float limit)
{
    if (*value > limit) *value = limit;
    else if (*value < -limit) *value = -limit;
}
