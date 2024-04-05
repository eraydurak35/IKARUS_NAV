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
static hmc5883l_t *mag_ptr = NULL;
static bmp390_t *baro_ptr = NULL;
static pmw3901_t *flow_ptr = NULL;
static range_finder_t *range_finder = NULL;
static nav_config_t *config_ptr = NULL;

static float acc_forward_bias = 0.0f;
static float acc_right_bias = 0.0f;
static float acc_up_bias = 0.0f;

static float rotation_matrix[3][3];

static float flow_accel_correct[3];

static void get_attitude_heading();
static void get_rotation_matrix();

void estimator_init(nav_config_t *cfg, states_t *sta, lsm6dsl_t *lsm, hmc5883l_t *hmc, bmp390_t *baro, pmw3901_t *flw, range_finder_t *rng)
{
    config_ptr = cfg;
    state_ptr = sta;
    imu_ptr = lsm;
    mag_ptr = hmc;
    baro_ptr = baro;
    flow_ptr = flw;
    range_finder = rng;

    /*
    quat_t q_acc, q_mag;
    acc_vec.x = imu_ptr->accel_ms2[Y];
    acc_vec.y = imu_ptr->accel_ms2[X];
    acc_vec.z = imu_ptr->accel_ms2[Z];

    mag_vec.x = mag_ptr->axis[Y];
    mag_vec.y = mag_ptr->axis[X];
    mag_vec.z = -mag_ptr->axis[Z];

    norm_vector(&acc_vec);
    norm_vector(&mag_vec);

    get_attitude_from_accel(&acc_vec, &q_acc);
    get_heading_from_mag(&mag_vec, &q_mag);
    q = get_quat_product(&q_acc, &q_mag);
    */

    acc_vec.x = imu_ptr->accel_ms2[X];
    acc_vec.y = imu_ptr->accel_ms2[Y];
    acc_vec.z = imu_ptr->accel_ms2[Z];

    mag_vec.x = mag_ptr->axis[X];
    mag_vec.y = mag_ptr->axis[Y];
    mag_vec.z = mag_ptr->axis[Z];

    get_quat_from_vector_measurements(&acc_vec, &mag_vec, &q);
}

void ahrs_predict()
{
    gyr_vec.x = imu_ptr->gyro_dps[Y] * DEG_TO_RAD;
    gyr_vec.y = imu_ptr->gyro_dps[X] * DEG_TO_RAD;
    gyr_vec.z = -imu_ptr->gyro_dps[Z] * DEG_TO_RAD;

    gyr_vec.x -= (config_ptr->ahrs_filter_beta * err.x);
    gyr_vec.y -= (config_ptr->ahrs_filter_beta * err.y);
    gyr_vec.z -= (config_ptr->ahrs_filter_beta * err.z);

    get_quat_deriv(&q, &gyr_vec, &q_dot);

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
    if (state_ptr->heading_deg > 360.0f) state_ptr->heading_deg -= 360.0f;

    state_ptr->pitch_dps = imu_ptr->gyro_dps[X];
    state_ptr->roll_dps = imu_ptr->gyro_dps[Y];
    state_ptr->yaw_dps = -imu_ptr->gyro_dps[Z];
}

void ahrs_correct()
{
    static vector_t err_acc = {0.0f, 0.0f, 0.0f};
    static vector_t err_mag = {0.0f, 0.0f, 0.0f};
    static vector_t temp = {0.0f, 0.0f, 0.0f};

/*     static int32_t c = 0;


    c++;
    if (c < 5000)
    {
        acc_vec.x = imu_ptr->accel_ms2[Y];
        acc_vec.y = imu_ptr->accel_ms2[X];
        acc_vec.z = -imu_ptr->accel_ms2[Z];
    }
    else if (c == 5000)
    {
        printf("100\n");
        acc_vec.x = imu_ptr->accel_ms2[Y];
        acc_vec.y = imu_ptr->accel_ms2[X];
        acc_vec.z = -imu_ptr->accel_ms2[Z];
    }
    else if (c < 17000)
    {
        acc_vec.x = imu_ptr->accel_ms2[Y] + 0.635f;
        acc_vec.y = imu_ptr->accel_ms2[X];
        acc_vec.z = -imu_ptr->accel_ms2[Z];
    }
    else if (c == 17000)
    {
        printf("-100\n");
        acc_vec.x = imu_ptr->accel_ms2[Y];
        acc_vec.y = imu_ptr->accel_ms2[X];
        acc_vec.z = -imu_ptr->accel_ms2[Z];
    }
    else if (c > 17000)
    {
        acc_vec.x = imu_ptr->accel_ms2[Y];
        acc_vec.y = imu_ptr->accel_ms2[X];
        acc_vec.z = -imu_ptr->accel_ms2[Z]; 
    }
 */

    //printf("%.2f,",imu_ptr->accel_ms2[Z]);
/*     acc_vec.x = imu_ptr->accel_ms2[Y];
    acc_vec.y = imu_ptr->accel_ms2[X];
    acc_vec.z = -(imu_ptr->accel_ms2[Z]); */
    acc_vec.x = imu_ptr->accel_ms2[Y] - flow_accel_correct[1];
    acc_vec.y = imu_ptr->accel_ms2[X] - flow_accel_correct[0];
    acc_vec.z = -(imu_ptr->accel_ms2[Z] - flow_accel_correct[2]);
    //printf("%.2f\n",-acc_vec.z);
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

    if (err.x > 0.4f) err.x = 0.4f;
    else if (err.x < -0.4f) err.x = -0.4f;

    if (err.y > 0.4f) err.y = 0.4f;
    else if (err.y < -0.4f) err.y = -0.4f;

    if (err.z > 0.4f) err.z = 0.4f;
    else if (err.z < -0.4f) err.z = -0.4f;

    imu_ptr->gyro_bias_dps[Y] += err.x * config_ptr->ahrs_filter_zeta;
    imu_ptr->gyro_bias_dps[X] += err.y * config_ptr->ahrs_filter_zeta;
    imu_ptr->gyro_bias_dps[Z] -= err.z * config_ptr->ahrs_filter_zeta;

    

    get_attitude_heading();

    //printf("%.2f\n", state_ptr->pitch_deg);
}

void get_earth_frame_accel()
{
/*     static float rot_matrix[3][3] = {0};
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

    rot_matrix[0][0] = cosy;
    rot_matrix[0][1] = 0.0f;
    rot_matrix[0][2] = siny;
    rot_matrix[1][0] = sinx * siny;
    rot_matrix[1][1] = cosx;
    rot_matrix[1][2] = -sinx * cosy;
    rot_matrix[2][0] = -(cosx * siny);
    rot_matrix[2][1] = sinx;
    rot_matrix[2][2] = cosy * cosx; */

    get_rotation_matrix();

    state_ptr->acc_forward_ms2 = imu_ptr->accel_ms2[Y] * rotation_matrix[0][0] + imu_ptr->accel_ms2[X] * rotation_matrix[1][0] + imu_ptr->accel_ms2[Z] * rotation_matrix[2][0];
    state_ptr->acc_right_ms2 = imu_ptr->accel_ms2[Y] * rotation_matrix[0][1] + imu_ptr->accel_ms2[X] * rotation_matrix[1][1] + imu_ptr->accel_ms2[Z] * rotation_matrix[2][1];
    state_ptr->acc_up_ms2 = (imu_ptr->accel_ms2[Y] * rotation_matrix[0][2] + imu_ptr->accel_ms2[X] * rotation_matrix[1][2] + imu_ptr->accel_ms2[Z] * rotation_matrix[2][2]) - 9.806f;
}

void correct_velocityXY()
{
    float diff_x = flow_ptr->velocity_x_ms - state_ptr->vel_forward_ms;
    float diff_y = flow_ptr->velocity_y_ms - state_ptr->vel_right_ms;

    if (diff_x < -1.5f)
        diff_x = -1.5f;
    else if (diff_x > 1.5f)
        diff_x = 1.5f;

    if (diff_y < -1.5f)
        diff_y = -1.5f;
    else if (diff_y > 1.5f)
        diff_y = 1.5f;

    acc_forward_bias -= diff_x * 0.001f;
    acc_right_bias -= diff_y * 0.001f;

    state_ptr->vel_forward_ms += diff_x * (1.0f - config_ptr->velxy_filter_beta);
    state_ptr->vel_right_ms += diff_y * (1.0f - config_ptr->velxy_filter_beta);
}
void predict_velocityXY() // 500Hz
{
    static const float dt = 0.002f;
    static float flow_accel_forward_ms = 0;
    static float flow_accel_right_ms = 0;
    static float filt_flow_accel_forward_ms = 0;
    static float filt_flow_accel_right_ms = 0;
    static float prev_vel_forward_ms = 0;
    static float prev_vel_right_ms = 0;

    state_ptr->vel_forward_ms += (state_ptr->acc_forward_ms2 - acc_forward_bias) * dt;
    state_ptr->vel_right_ms += (state_ptr->acc_right_ms2 - acc_right_bias) * dt;

    state_ptr->vel_forward_ms -= state_ptr->vel_right_ms * sinf(-state_ptr->yaw_dps * DEG_TO_RAD * 0.002f);
    state_ptr->vel_right_ms += state_ptr->vel_forward_ms * sinf(-state_ptr->yaw_dps * DEG_TO_RAD * 0.002f);


    ///////////////////////////
    flow_accel_forward_ms = (state_ptr->vel_forward_ms - prev_vel_forward_ms) / dt;
    flow_accel_right_ms = (state_ptr->vel_right_ms - prev_vel_right_ms) / dt;

    filt_flow_accel_forward_ms += (flow_accel_forward_ms - filt_flow_accel_forward_ms) * 0.15f;
    filt_flow_accel_right_ms += (flow_accel_right_ms - filt_flow_accel_right_ms) * 0.15f;

    prev_vel_forward_ms = state_ptr->vel_forward_ms;
    prev_vel_right_ms = state_ptr->vel_right_ms;

    //printf("%.2f,%.2f\n", flow_accel_right_ms, filt_flow_accel_right_ms);

    get_rotation_matrix();

    flow_accel_correct[1] = filt_flow_accel_forward_ms * rotation_matrix[0][0] + filt_flow_accel_right_ms * rotation_matrix[0][1] + 0 * rotation_matrix[0][2];
    flow_accel_correct[0] = filt_flow_accel_forward_ms * rotation_matrix[1][0] + filt_flow_accel_right_ms * rotation_matrix[1][1] + 0 * rotation_matrix[1][2];
    flow_accel_correct[2] = filt_flow_accel_forward_ms * rotation_matrix[2][0] + filt_flow_accel_right_ms * rotation_matrix[2][1] + 0 * rotation_matrix[2][2];
}

void get_flow_velocity()
{

    static float filt_gyr_degs_pitch;
    static float filt_gyr_degs_roll;
    // Additional filtering required to match the phases of gyro and opt flow sensor
    // 10 is calibration gain (degs / gain) has to be equal flow sensor output
    filt_gyr_degs_roll += ((state_ptr->roll_dps / 11.5474487f) - filt_gyr_degs_roll) * 0.24f;
    filt_gyr_degs_pitch += ((state_ptr->pitch_dps / 11.3375732f) - filt_gyr_degs_pitch) * 0.24f;

    // Flow sensor outputs cpi value. Height info needed to calculate velocity in m/s
    // This compansates the sensor output for roll and pitch and filters
    flow_ptr->filt_x_cpi += ((flow_ptr->raw_x_cpi + filt_gyr_degs_roll) - flow_ptr->filt_x_cpi) * 0.2f;
    flow_ptr->filt_y_cpi += ((flow_ptr->raw_y_cpi + filt_gyr_degs_pitch) - flow_ptr->filt_y_cpi) * 0.2f;

    if (range_finder->range_cm > 400 && state_ptr->altitude_m < 4.0f)
    {
        flow_ptr->velocity_y_ms = flow_ptr->filt_x_cpi * (4.0f / 5.0f);
        flow_ptr->velocity_x_ms = flow_ptr->filt_y_cpi * (4.0f / -5.0f);
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

void calculate_altitude_velocity()
{
    static uint8_t init = 1;
    static float baroAlt_offset;
    static float BaroAlt;
    static float RangeAlt = -1;
    static float lastBaroAlt;
    float rangefinder_transition;
    float vel_acc;
    static float vel;
    static float accAlt;
    float baroVel;
    static const float dt = 0.002f;

    //BaroAlt = baro_ptr->altitude_m;
    BaroAlt += (baro_ptr->altitude_m - BaroAlt) * 0.1f;

    if (range_finder->range_cm > 400)
        RangeAlt = -1.0; // 400

    else
        RangeAlt += ((range_finder->range_cm / 100.0f) - RangeAlt) * 0.1f; //RangeAlt = range_finder->range_cm / 100.0f;
    
    //printf("%.2f\n", RangeAlt);
    //RangeAlt = -1;

    if (RangeAlt >= 0 && RangeAlt < 3.0f) // 3
    {
        baroAlt_offset = BaroAlt - RangeAlt;
        BaroAlt = RangeAlt;
    }
    else
    {
        BaroAlt -= baroAlt_offset;
        if (RangeAlt > 0)
        {
            rangefinder_transition = (4.0f - RangeAlt); // 4
            BaroAlt = RangeAlt * rangefinder_transition + BaroAlt * (1.0f - rangefinder_transition);
        }
    }

    vel_acc = (state_ptr->acc_up_ms2 - acc_up_bias) * dt;
    accAlt += (vel_acc * 0.5f) * dt + vel * dt;
    accAlt = accAlt * config_ptr->alt_filter_beta + BaroAlt * (1.0f - config_ptr->alt_filter_beta);
    vel += vel_acc;

    if (RangeAlt >= 0 && RangeAlt < 3.0f) // 3
    {
        state_ptr->altitude_m = BaroAlt;
    }
    else
    {
        state_ptr->altitude_m = accAlt;
    }

    if (init == 1)
    {
        lastBaroAlt = BaroAlt;
        init = 0;
    }
    baroVel = (BaroAlt - lastBaroAlt) / dt;
    lastBaroAlt = BaroAlt;

    /* printf("%.2f\n", baroVel); */

    float velDiff;
    velDiff = baroVel - vel;
/*     if (velDiff > 5.0f)
        velDiff = 5.0f;
    else if (velDiff < -5.0f)
        velDiff = -5.0f; */
    vel += velDiff * config_ptr->velz_filter_beta * dt;
    acc_up_bias -= velDiff * config_ptr->velz_filter_zeta * dt * dt;

    state_ptr->vel_up_ms = vel;
}


static void get_rotation_matrix()
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