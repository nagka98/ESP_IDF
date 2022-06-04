/* Blink Example
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
//#include <stdio.h>

#include "AD5933.h"
#include "AD7680.h"
#include "LSM6.h"
#include "LIS2.h"
#include "wireless.h"
#include "common.h"
#include "ArduiKalman.h"
#include "Fusion/Fusion.h"

#include "Matrix3.h"


#define convert_g_to_ms2 9.80665f
#define frequency_hz     50
#define interval_ms      (1000/(frequency_hz+1))

////////////////////////////////////////////////////
//////// VARIABLE DEFINITIONS
////////////////////////////////////////////////////

static const char *TAG = "main";
static unsigned long last_interval_ms = 0;

//IMU fusion
#define SAMPLE_PERIOD (0.001f) 
FusionAhrs ahrs;

static float kalman_predict(KalmanFilter kalman_obj, float value);
static int R_elim[10] = {0,1,1,0,0,1,1,0,0,0};
static int R_treshold[10] = {0,180000,185000,0,0,185000,180000,0,0,0};
// static int R_min[10] = {0,190000,190000,190000,0,180000,180000,0,0,0};
// static int R_max[10] = {0,200000,230000,200000,0,190000,200000,0,0,0};

// Impedance measurement
#ifdef AD5933_ADDR
    AD5933 impc(I2C_PORT, AD5933_ADDR);

    long inv_gain[ARR_SIZE];
    int phase[ARR_SIZE];

    int Z_values[NUM_CHANNELS];

    int16_t tmp_re[ARR_SIZE];
    int16_t tmp_im[ARR_SIZE];
#endif

// Resistance measurement
int rm_range;

#ifdef SPI_BUS
    uint16_t tmp_r;
    int R_values[NUM_CHANNELS];

    spi_device_handle_t spidev;
    AD7680 radc(&spidev);
#endif

// Accelerometer and gyroscope
#ifdef LSM6_ADDR
    LSM6 gyxl(I2C_PORT, LSM6_ADDR);
    Matrix3 rot;
    lsm6_fifo_data xl;
    lsm6_fifo_data gy;

    const float deg2rad = 3.14159265 / 180;
    // Measurement range, 1000 dps, measurements
    const float gyro_base = deg2rad * GYRO_SENS_DPS / 32767;
    uint64_t gyro_last_time;

    int gyro_counter = 0;
#endif

// Magnetometer
#ifdef LIS2_ADDR
    LIS2 magn(I2C_PORT, LIS2_ADDR);
    lis2_data mag;
#endif

//Kalman init Params
int stateNum = 1;
int measureNum = 1;

float xc_r[NUM_CHANNELS][1];        // correct state vector 
float xc_z[NUM_CHANNELS][1];        // correct state vector 
float xc_acc[3][1];        // correct state vector 
float xc_gyr[3][1];        // correct state vector 
float xc_mag[3][1];        // correct state vector 
float xp_r[NUM_CHANNELS][1];        // correct state vector 
float xp_z[NUM_CHANNELS][1];        // correct state vector 
float xp_acc[3][1];        // predict state vector 
float xp_gyr[3][1];        // predict state vector
float xp_mag[3][1];        // predict state vector
float A_r[NUM_CHANNELS][1][1];      // prediction error covariance 
float A_z[NUM_CHANNELS][1][1];      // prediction error covariance 
float A_acc[3][1][1];      // prediction error covariance
float A_gyr[3][1][1];      // prediction error covariance
float A_mag[3][1][1];      // prediction error covariance
float Q_r[NUM_CHANNELS][1][1];      // process noise covariance 
float Q_z[NUM_CHANNELS][1][1];      // process noise covariance 
float Q_acc[3][1][1];      // prediction error covariance
float Q_gyr[3][1][1];      // prediction error covariance
float Q_mag[3][1][1];      // prediction error covariance
float R_r[NUM_CHANNELS][1][1];      // measurement error covariance
float R_z[NUM_CHANNELS][1][1];      // measurement error covariance
float R_acc[3][1][1];      // prediction error covariance
float R_gyr[3][1][1];      // prediction error covariance
float R_mag[3][1][1];      // prediction error covariance
float H_r[NUM_CHANNELS][1][1];      // Measurement model
float H_z[NUM_CHANNELS][1][1];      // Measurement model
float H_acc[3][1][1];      // prediction error covariance
float H_gyr[3][1][1];      // prediction error covariance
float H_mag[3][1][1];      // prediction error covariance
float P_r[NUM_CHANNELS][1][1];      // Post-prediction, pre-update
float P_z[NUM_CHANNELS][1][1];      // Post-prediction, pre-update
float P_acc[3][1][1];      // prediction error covariance
float P_gyr[3][1][1];      // prediction error covariance
float P_mag[3][1][1];      // prediction error covariance

KalmanFilter m_kf_r[NUM_CHANNELS];
KalmanFilter m_kf_z[NUM_CHANNELS];
KalmanFilter m_kf_acc[3];
KalmanFilter m_kf_gyr[3];
KalmanFilter m_kf_mag[3];

////////////////////////////////////////////////////
//////// GESTURE DETECTION SETUP
////////////////////////////////////////////////////

void mux_config(gpio_num_t pin) {
    if(pin == GPIO_NUM_NC) return;
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
    gpio_set_level(pin, 0);
}

// Selects the channel of the R and Z subject multiplexers.
// 0-9 -> subjects, 10-15 -> calibration resistors
void select_channel (int channel) {
    gpio_set_level(SUBJ_A0_GPIO, channel & 0b0001);
    gpio_set_level(SUBJ_A1_GPIO, channel & 0b0010);
    gpio_set_level(SUBJ_A2_GPIO, channel & 0b0100);
    gpio_set_level(SUBJ_A3_GPIO, channel & 0b1000);
}

// Selects the resistance measurement range
//   0 -> 100k - 1M   Ohm
//   1 -> 10k  - 100k Ohm
//   2 -> 1k   - 10k  Ohm
//   3 -> 100  - 1k   Ohm
void set_rm_range (int range) {
    rm_range = range;
    gpio_set_level(RANGE_A0_GPIO, range & 0b01);
    gpio_set_level(RANGE_A1_GPIO, range & 0b10);
}

// Selects the impedance amplifier feedback resistor
// The resistor values are board revision dependent
void set_zm_fb (int fb) {
    #ifdef HW_IS_REV_1_3
        if(fb == 0) {
            ESP_LOGW(TAG, "Feedback resistor 1 is unavailable for board revision 1.3 and will be effectively 0 Ohm.");
        }
    #endif
    gpio_set_level(FB_A0_GPIO, fb & 0b01);
    gpio_set_level(FB_A1_GPIO, fb & 0b10);
}

void set_enabled (bool enabled) {
    gpio_set_level(MEASURE_EN_GPIO, enabled);
}


void gesture_setup() {
    ////// CONFIGURE INTERFACES
    // Configure I2C
    #ifdef I2C_PORT
        i2c_config_t i2ccfg = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = SDA_GPIO,
            .scl_io_num = SCL_GPIO,
            .sda_pullup_en = GPIO_PULLUP_DISABLE,
            .scl_pullup_en = GPIO_PULLUP_DISABLE
        };
        i2ccfg.master.clk_speed = I2C_FREQ;
        i2c_param_config(I2C_PORT, &i2ccfg);
        i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
    #endif

    // Configure SPI
    #ifdef SPI_BUS
        spi_bus_config_t spicfg = {
            .mosi_io_num = MOSI_GPIO,
            .miso_io_num = MISO_GPIO,
            .sclk_io_num = SCK_GPIO,
            .quadwp_io_num = GPIO_NUM_NC,
            .quadhd_io_num = GPIO_NUM_NC,
            .max_transfer_sz = 32,
        };
        spi_bus_initialize(SPI_BUS, &spicfg, SPI_DMA_CH_AUTO);
        spi_device_interface_config_t devcfg = {
            .mode = 3, // CPOL = 1, CPHA = 1
            .clock_speed_hz = SPI_FREQ,
            .spics_io_num = CS0_GPIO,
            .queue_size = 2,
        };
        spi_bus_add_device(SPI_BUS, &devcfg, &spidev);
    #endif

    // Configure MUX GPIOs
    mux_config(MEASURE_EN_GPIO);
    mux_config(SUBJ_A0_GPIO);
    mux_config(SUBJ_A1_GPIO);
    mux_config(SUBJ_A2_GPIO);
    mux_config(SUBJ_A3_GPIO);
    mux_config(RANGE_A0_GPIO);
    mux_config(RANGE_A1_GPIO);
    mux_config(FB_A0_GPIO);
    mux_config(FB_A1_GPIO);
    
    ////// CONFIGURE SENSORS
    //Perform initial configuration of AD5933. Fail if any one of these fail.
    #ifdef AD5933_ADDR
        if (!(impc.reset() &&
            impc.setInternalClock(true) &&
            impc.setStartFrequency(START_FREQ) &&
            impc.setIncrementFrequency(FREQ_INCR) &&
            impc.setNumberIncrements(NUM_INCR) &&
            impc.setRange(VRANGE_3) &&
            impc.setPGAGain(PGA_GAIN_X5) &&
            impc.setSettlingCycles(SETTLING_CYCLES)
        )) {
            ESP_LOGE(TAG, "FAILED initializing AD5933!");
            vTaskSuspend(NULL);
        }
    #endif

    //Configure LSM6DSO
    #ifdef LSM6_ADDR
        gyxl.test();
        gyxl.enable();
    #endif

    //Configure LIS2MDL
    #ifdef LIS2_ADDR
        magn.test();
        magn.enable();
    #endif

    // Setup Wireless connection
    wireless_init();
}




////////////////////////////////////////////////////
//////// GESTURE DETECTION LOOP
////////////////////////////////////////////////////

int loop_counter = 0;

// Result string buffer
char results[512];

#ifdef AD5933_ADDR
int calculate_impedance() {
    int ref_impedance = 0;
    long magnitude;
    int re;
    int im;
    // for now, average magnitudes over the measurements of each channel
    // update if necessary
    for (int i = 0; i < ARR_SIZE; i++) {
        re = tmp_re[i];
        im = tmp_im[i];
        magnitude = fast_int_sqrt(re * re + im * im);
        if(magnitude != 0)
            ref_impedance += inv_gain[i] * ARR_SIZE / magnitude;
    }
    return ref_impedance;
}
#endif

#ifdef SPI_BUS
void rm_measure_and_check() {
    // Power on and allow the value to settle after a range or channel change
    radc.powerOn();

    #ifndef ADC_FIXED_RANGE
        // Shorter reading to determine if the range is correct
        radc.read_averaging(&tmp_r, ADC_AVERAGING / 5 + 1);

        // Calculate the optimal range for the reading
        int new_range = rm_range;
        if(tmp_r < 13000) {
            new_range++;
            if(tmp_r < 1750) new_range++;
            if(new_range > 3) new_range = 3;
        } else if(tmp_r > 52500) {
            new_range--;
            if(tmp_r > 63800) new_range--;
            if(new_range < 0) new_range = 0;
        }
        if(new_range != rm_range) {
            // If the range should be changed, update range and check again.
            set_rm_range(new_range);
            rm_measure_and_check();
            return;
        }
    #endif
    // If the range is correct already, average multiple voltage readings.
    radc.read_averaging(&tmp_r, ADC_AVERAGING);
}

int calculate_resistance() {
    // Value V = 65535 * R / (R + R_REF)
    // R = V R_REF / (65535 - V)
    uint64_t r_ref;
    switch(rm_range) {
        case 0:  r_ref = 330000; break;
        case 1:  r_ref = 33000;  break;
        case 2:  r_ref = 3300;   break;
        default: r_ref = 330;    break;
    }
    uint16_t div = 65535 - tmp_r;
    if(div < 100) return 0;
    int result = r_ref * tmp_r / div;
    return result;
}
#endif

// Starts the next resistance and impedance measurement
void rz_start(int channel) {
    #ifdef AD5933_ADDR
        impc.startFrequencySweep();
    #endif
    // resistance measurement
}

// Finishes the current measurement and sets the pointers
// for the results to be stored
void rz_finish(int channel) {
    #ifdef SPI_BUS
        rm_measure_and_check();
        radc.powerOff();
    #endif

    #ifdef AD5933_ADDR
        // Wait until the frequency sweep is done
        impc.setDataStore(tmp_re, tmp_im);
        while(!impc.advanceFrequencySweep(FAST_UNSAFE)) ;
    #endif
}

// Queries the results of the previous measurements and
// writes them to the storage
void rz_collect_result(int channel) {
    if(channel < 0) return;

    // Collect magnetometer data
    #ifdef LIS2_ADDR
        magn.read_data(&mag);
    #endif

    // Collect accelerometer / gyroscope data
    #ifdef LSM6_ADDR
        int fifo_num = (gyxl.fifo_size() / 2) * 2;    
        if(fifo_num > 30) fifo_num = 10;
        else if(fifo_num > 4) fifo_num = 4;
        float gyro_factor = 0;
        if(fifo_num) {
            uint64_t current_time = esp_timer_get_time();
            gyro_factor = gyro_base * (current_time - gyro_last_time) * 2 / fifo_num / 1.e6f;
            gyro_last_time = current_time;
        }
        lsm6_fifo_data data;
        float rx;
        float ry;
        float rz;
        Matrix3 rtmp;
        for(int i = 0; i < fifo_num; i++) {
            gyxl.read_fifo_entry(&data);
            // for now, ignore acceleration and only handle gyroscope
            switch(data.tag.tag_sensor) {
                case LSM6DSO_GYRO_NC_TAG: 
                    gy = data;
                    rx = data.x * gyro_factor;
                    ry = data.y * gyro_factor;
                    rz = data.z * gyro_factor;
                    rtmp.set_rot_3d(rx, ry, rz);
                    rot.mul_left(&rtmp);
                    gyro_counter ++;
                    break;
                case LSM6DSO_XL_NC_TAG:
                    xl = data;
                    break;
                default:
                    ESP_LOGI(TAG, "LSM6 FIFO Packet: [%d], %d, %d, %d", data.tag.tag_sensor, data.x, data.y, data.z);
            }
        }
    #endif

    #ifdef AD5933_ADDR
        // Process impedance results
        Z_values[channel] = calculate_impedance();
    #endif

    #ifdef SPI_BUS
        // Process resistance results
        R_values[channel] = calculate_resistance();
    #endif
}

void rz_poweron() {
    set_enabled(true);
}

// Sets the resistance and impedance circuits into standby
void rz_poweroff() {
    set_enabled(false);
    #ifdef AD5933_ADDR
        impc.powerOff();
    #endif
}

void create_result_string(int64_t timestamp) {
    int delta = esp_timer_get_time() - timestamp;
    char *ptr = results;
    //ptr += sprintf(ptr, "[%7d+%2d.%dms]", (int)(timestamp / 1000), delta / 1000, (delta / 100) % 10);
    int i;
    #ifdef SPI_BUS
// Resistance data printing
       // ptr += sprintf(ptr, "\n");
        for(i = 0; i < NUM_CHANNELS; i++) {
            int R = R_values[i];
            if(R_values[i] <= 0 || R_values[i] > 1000000) {
                ptr += sprintf(ptr, "0,");//remove comments 
            } else {
            //      //ptr += sprintf(ptr, "%d,", R);//remove comments
                  ptr += sprintf(ptr, "%d,",(int)round(kalman_predict(m_kf_r[i],(float)(R))));//remove comments         
            }
            // if(i != 9)
            // {
            //      ptr += sprintf(ptr, ",");
            // }  
        }
    #endif
    #ifdef AD5933_ADDR
//Impedance data printing
        //ptr += sprintf(ptr, "\n");
        for(i = 0; i < NUM_CHANNELS; i++) {
            int Z = Z_values[i];
            if(Z <= 0 || Z > 1000000) {// set this to 500000
                 //ptr += sprintf(ptr, "0,");//remove comments
            } else {
                 //ptr += sprintf(ptr, "%d,", Z);//remove comments
                 //ptr += sprintf(ptr, "%d,",(int)round(kalman_predict(m_kf_r[i],(float)(Z))));//remove comments
            }
        }
    #endif
    #ifdef LSM6_ADDR
       //ptr += sprintf(ptr, "%f,%f,%f,", xl.x*(convert_g_to_ms2), xl.y*(convert_g_to_ms2), xl.z*(convert_g_to_ms2));
       //ptr += sprintf(ptr, "%f,%f,%f,", gy.x*(convert_g_to_ms2), gy.y*(convert_g_to_ms2), gy.z*(convert_g_to_ms2));
       ptr += sprintf(ptr, "%d,%d,%d", (int)kalman_predict(m_kf_acc[0],xl.x*(convert_g_to_ms2)), (int)kalman_predict(m_kf_acc[1],xl.y*(convert_g_to_ms2)), (int)kalman_predict(m_kf_acc[2],xl.z*(convert_g_to_ms2)));
       //ptr += sprintf(ptr, "%d,%d,%d,", (int)kalman_predict(m_kf_gyr[0],gy.x*(convert_g_to_ms2)), (int)kalman_predict(m_kf_gyr[1],gy.y*(convert_g_to_ms2)), (int)kalman_predict(m_kf_gyr[2],gy.z*(convert_g_to_ms2)));
        const FusionVector gyroscope = {(float)gy.x,(float)gy.y,(float)gy.z};
        const FusionVector accelerometer = {(float)xl.x,(float)xl.y,(float)xl.z};
        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);
        const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
        //ptr += sprintf(ptr, "%f,%f,%f", euler.angle.roll, euler.angle.pitch, euler.angle.yaw);

    #endif
    #ifdef LIS2_ADDR
       //ptr += sprintf(ptr, "%f,%f,%f", mag.x*(convert_g_to_ms2), mag.y*(convert_g_to_ms2), mag.z*(convert_g_to_ms2));
       //ptr += sprintf(ptr, "%d,%d,%d", (int)kalman_predict(m_kf_mag[0],mag.x*(convert_g_to_ms2)), (int)kalman_predict(m_kf_mag[1],mag.y*(convert_g_to_ms2)), (int)kalman_predict(m_kf_mag[2],mag.z*(convert_g_to_ms2)));
    #endif
}

void print_info() {
    if(!PRINT_INTERVAL) return;
    if(PRINT_INTERVAL > 1 && loop_counter % PRINT_INTERVAL) return;
    if(esp_timer_get_time() > (last_interval_ms + interval_ms) )
    {
        last_interval_ms = esp_timer_get_time();
        printf("%s\n", results);//add /n
    }
}

void gesture_loop ()  {
    int64_t start = esp_timer_get_time();

    // Perform measurements
    rz_poweron();
    for(int channel = 0; channel < NUM_CHANNELS; channel++) {
        select_channel(channel);
        rz_start(channel);
        rz_collect_result(channel - 1);
        rz_finish(channel);
    }
    rz_poweroff();
    rz_collect_result(NUM_CHANNELS - 1);

    create_result_string(start);
    wireless_send(results);
    print_info();
    loop_counter++;
}




////////////////////////////////////////////////////
//////// CALIBRATION AND ENTRY POINT
////////////////////////////////////////////////////

void gesture_calibrate() {
    // Calibrate AD5933
    rz_poweron();
    
    select_channel(13); // 49.9 k
    set_zm_fb(1);
    #ifdef AD5933_ADDR
        bool success = impc.calibrate(inv_gain, phase, 49900, ARR_SIZE);
        if(!success) {
            ESP_LOGE(TAG, "FAILED calibrating AD5933!");
            vTaskSuspend(NULL);
        }
    #endif

    #ifdef SPI_BUS
        // TODO: through calibration, calculate a correction factor for
        //       calculating the resistance from the value
        #ifdef ADC_FIXED_RANGE
            set_rm_range(ADC_FIXED_RANGE);
        #else
            set_rm_range(1);
        #endif
        radc.powerOn();
        radc.read_averaging(&tmp_r, ADC_AVERAGING * 2);
        int r_calc = calculate_resistance();
        ESP_LOGI(TAG, "ADC Data for 10kOhm is %u (calculated R = %d).", tmp_r, r_calc);

        select_channel(15);
        radc.powerOn();
        radc.read_averaging(&tmp_r, ADC_AVERAGING * 2);
        r_calc = calculate_resistance();
        ESP_LOGI(TAG, "ADC Data for 100kOhm is %u (calculated R = %d).", tmp_r, r_calc);

        radc.powerOff();
    #endif

    rz_poweroff();
}

/* KALMAN setup */

static void kalman_setup(void)
{
    for(int i=0; i<NUM_CHANNELS; i++)
    {
    m_kf_r[i].init(stateNum, measureNum, &A_r[i][0][0], &P_r[i][0][0], &Q_r[i][0][0], &H_r[i][0][0], &R_r[i][0][0], &xp_r[i][0], &xc_r[i][0]);
    m_kf_r[i].zeros();
    m_kf_z[i].init(stateNum, measureNum, &A_z[i][0][0], &P_z[i][0][0], &Q_z[i][0][0], &H_z[i][0][0], &R_z[i][0][0], &xp_z[i][0], &xc_z[i][0]);
    m_kf_z[i].zeros();
    A_r[i][0][0] = 1.0f;
    A_z[i][0][0] = 1.0f;
    H_r[i][0][0] = 1.0f;
    H_z[i][0][0] = 1.0f;
    Q_r[i][0][0] = 0.3f;
    Q_z[i][0][0] = 0.3f;
    R_r[i][0][0] = 15.0f;
    R_z[i][0][0] = 20.0f;
    P_r[i][0][0] = 1.0f;
    P_z[i][0][0] = 1.0f;
    }
    for(int i=0; i<3; i++)
    {
    m_kf_acc[i].init(stateNum, measureNum, &A_acc[i][0][0], &P_acc[i][0][0], &Q_acc[i][0][0], &H_acc[i][0][0], &R_acc[i][0][0], &xp_acc[i][0], &xc_acc[i][0]);
    m_kf_acc[i].zeros();
    m_kf_gyr[i].init(stateNum, measureNum, &A_gyr[i][0][0], &P_gyr[i][0][0], &Q_gyr[i][0][0], &H_gyr[i][0][0], &R_gyr[i][0][0], &xp_gyr[i][0], &xc_gyr[i][0]);
    m_kf_gyr[i].zeros();
    m_kf_mag[i].init(stateNum, measureNum, &A_mag[i][0][0], &P_mag[i][0][0], &Q_mag[i][0][0], &H_mag[i][0][0], &R_mag[i][0][0], &xp_mag[i][0], &xc_mag[i][0]);
    m_kf_mag[i].zeros();
    A_acc[i][0][0] = 1.0f;
    A_mag[i][0][0] = 1.0f;
    A_gyr[i][0][0] = 1.0f;
    H_acc[i][0][0] = 1.0f;
    H_mag[i][0][0] = 1.0f;
    H_gyr[i][0][0] = 1.0f;
    Q_acc[i][0][0] = 0.3f;
    Q_mag[i][0][0] = 0.3f;
    Q_gyr[i][0][0] = 0.3f;
    R_acc[i][0][0] = 10.0f;
    R_mag[i][0][0] = 10.0f;
    R_gyr[i][0][0] = 10.0f;
    P_acc[i][0][0] = 1.0f;
    P_mag[i][0][0] = 1.0f;
    P_gyr[i][0][0] = 1.0f;
    }
}

/* KALMAN predict function */

static float kalman_predict(KalmanFilter kalman_obj, float value)
{
      float *predict = kalman_obj.predict();
      float measurement[measureNum];
      measurement[0] = value;
      float *correct = kalman_obj.correct(measurement);
      float estimated_value = correct[0];
      return estimated_value;
}

/* MAIN function */
extern "C" void app_main(void)
{
    FusionAhrsInitialise(&ahrs);
    gesture_setup();
    gesture_calibrate();
    kalman_setup();
    TickType_t tick = xTaskGetTickCount();
    const int intervalTicks = LOOP_INTERVAL_MS / portTICK_PERIOD_MS;
    printf("Started\n");
    while (true)
    {
        gesture_loop();
        vTaskDelayUntil(&tick, intervalTicks);
    }
}

