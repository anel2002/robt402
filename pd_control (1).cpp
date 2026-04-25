#include <pigpio.h>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <chrono>
#include <thread>
#include <atomic>
#include <cstdint>
#include <algorithm>
#include <csignal>

using namespace std;

// ============================================================
// GPIO PIN MAP
// ============================================================

// Left motor
static constexpr int M1_PWM   = 12; // hardware PWM
static constexpr int M1_IN1   = 24;
static constexpr int M1_IN2   = 23;
static constexpr int M1_ENC_A = 17;
static constexpr int M1_ENC_B = 27;

// Right motor
static constexpr int M2_PWM   = 13; // hardware PWM
static constexpr int M2_IN1   = 6;
static constexpr int M2_IN2   = 5;
static constexpr int M2_ENC_A = 16;
static constexpr int M2_ENC_B = 26;

static constexpr int GPIO_EN  = 22; // driver enable

// ============================================================
// IMU: Pololu MinIMU-9 v5 -> LSM6DS33 accel/gyro
// ============================================================

static constexpr int I2C_BUS = 1;
static constexpr int LSM6DS33_ADDR = 0x6B;

// Registers
static constexpr int WHO_AM_I   = 0x0F;
static constexpr int CTRL2_G    = 0x11;
static constexpr int CTRL1_XL   = 0x10;
static constexpr int OUTX_L_G   = 0x22;
static constexpr int OUTX_L_XL  = 0x28;

// ============================================================
// SYSTEM CONSTANTS
// ============================================================

static constexpr double LOOP_HZ = 200.0;
static constexpr double DT = 1.0 / LOOP_HZ;

static constexpr double GEAR_RATIO = 20.4;
static constexpr double CPR = 48.0;
static constexpr double TOTAL_CPR = GEAR_RATIO * CPR; // counts per output shaft rev

static constexpr double RAD_TO_DEG = 180.0 / M_PI;
static constexpr double DEG_TO_RAD = M_PI / 180.0;

// If encoder callback counts all edges from both A and B, this may need x4 scaling.
// Start with this value and verify experimentally.
// If measured speed/angle is 4x too large, replace TOTAL_COUNTS_PER_REV with TOTAL_CPR * 4.0
static constexpr double TOTAL_COUNTS_PER_REV = TOTAL_CPR;

// Wheel radius in meters: measure your actual wheel radius carefully
static constexpr double WHEEL_RADIUS = 0.0325; // example 65 mm diameter wheel

// ============================================================
// CONTROL GAINS
// IMPORTANT: these are STARTING values, not guaranteed final.
// Tune experimentally.
// ============================================================

static double Kp = 6.0;   // angle term
static double Kd = 0.1;   // gyro damping
static double Kv = 0.3;   // wheel velocity damping

// Small trim if one motor is stronger/weaker
static double LEFT_GAIN  = 1.00;
static double RIGHT_GAIN = 1.00;

// Desired upright pitch offset (rad)
// Use this to compensate mechanical bias / center-of-mass bias.
static double THETA_OFFSET = 0.0;

// Safety thresholds
static constexpr double FALL_ANGLE_DEG = 25.0;
static constexpr double ARM_ANGLE_DEG  = 10.0;

// Motor settings
static constexpr int PWM_FREQ = 20000; // 20 kHz
static constexpr double UMAX = 1.0;
static constexpr double MOTOR_DEADBAND = 0.18; // minimum usable duty once active
static constexpr double COMMAND_DEADBAND = 0.03;

// Complementary filter
static constexpr double COMP_ALPHA = 0.985;

// ============================================================
// GLOBALS
// ============================================================

static atomic<long> m1_encoder_counts{0};
static atomic<long> m2_encoder_counts{0};
static volatile bool g_running = true;

// ============================================================
// UTILS
// ============================================================

template<typename T>
T clamp_val(T x, T lo, T hi) {
    return std::max(lo, std::min(hi, x));
}

void stop_motors() {
    gpioWrite(M1_IN1, 0);
    gpioWrite(M1_IN2, 0);
    gpioWrite(M2_IN1, 0);
    gpioWrite(M2_IN2, 0);

    gpioHardwarePWM(M1_PWM, PWM_FREQ, 0);
    gpioHardwarePWM(M2_PWM, PWM_FREQ, 0);
}

void signal_handler(int) {
    g_running = false;
}

// ============================================================
// ENCODER ISR
// ============================================================

void m1_encoder_isr(int gpio, int level, uint32_t tick) {
    if (level == PI_TIMEOUT) return;
    int a = gpioRead(M1_ENC_A);
    int b = gpioRead(M1_ENC_B);

    if (gpio == M1_ENC_A) {
        (a != b) ? m1_encoder_counts++ : m1_encoder_counts--;
    } else {
        (a == b) ? m1_encoder_counts++ : m1_encoder_counts--;
    }
}

void m2_encoder_isr(int gpio, int level, uint32_t tick) {
    if (level == PI_TIMEOUT) return;
    int a = gpioRead(M2_ENC_A);
    int b = gpioRead(M2_ENC_B);

    if (gpio == M2_ENC_A) {
        (a != b) ? m2_encoder_counts++ : m2_encoder_counts--;
    } else {
        (a == b) ? m2_encoder_counts++ : m2_encoder_counts--;
    }
}

// ============================================================
// IMU HELPERS
// ============================================================

int16_t make_int16(uint8_t lo, uint8_t hi) {
    return static_cast<int16_t>((hi << 8) | lo);
}

bool read_accel_gyro_raw(int i2c, int16_t &ax, int16_t &ay, int16_t &az,
                         int16_t &gx, int16_t &gy, int16_t &gz) {
    char gbuf[6];
    char abuf[6];

    if (i2cReadI2CBlockData(i2c, OUTX_L_G, gbuf, 6) != 6) return false;
    if (i2cReadI2CBlockData(i2c, OUTX_L_XL, abuf, 6) != 6) return false;

    gx = make_int16((uint8_t)gbuf[0], (uint8_t)gbuf[1]);
    gy = make_int16((uint8_t)gbuf[2], (uint8_t)gbuf[3]);
    gz = make_int16((uint8_t)gbuf[4], (uint8_t)gbuf[5]);

    ax = make_int16((uint8_t)abuf[0], (uint8_t)abuf[1]);
    ay = make_int16((uint8_t)abuf[2], (uint8_t)abuf[3]);
    az = make_int16((uint8_t)abuf[4], (uint8_t)abuf[5]);

    return true;
}

// For LSM6DS33 configured as:
// gyro: ±245 dps  -> 8.75 mdps/LSB
// accel: ±2 g     -> 0.061 mg/LSB
double gyro_lsb_to_rad_s(int16_t raw) {
    double dps = raw * 0.00875;
    return dps * DEG_TO_RAD;
}

double accel_lsb_to_g(int16_t raw) {
    return raw * 0.000061;
}

// Pitch from accel.
// Assuming:
// X = forward/back
// Y = left/right
// Z = up/down
// Then pitch about Y can be estimated from X and Z.
double accel_pitch_rad(double ax_g, double az_g) {
    return atan2(ax_g, az_g);
}

// ============================================================
// MOTOR DRIVE
// ============================================================

void drive_one_motor(int in1, int in2, int pwm_pin, double u, double gain = 1.0) {
    u *= gain;
    u = clamp_val(u, -UMAX, UMAX);

    if (fabs(u) < COMMAND_DEADBAND) {
        gpioWrite(in1, 0);
        gpioWrite(in2, 0);
        gpioHardwarePWM(pwm_pin, PWM_FREQ, 0);
        return;
    }

    bool forward = (u > 0.0);
    double mag = fabs(u);

    // map to overcome stiction/dead zone
    double duty = MOTOR_DEADBAND + (1.0 - MOTOR_DEADBAND) * mag;
    duty = clamp_val(duty, 0.0, 1.0);

    if (forward) {
        gpioWrite(in1, 1);
        gpioWrite(in2, 0);
    } else {
        gpioWrite(in1, 0);
        gpioWrite(in2, 1);
    }

    gpioHardwarePWM(pwm_pin, PWM_FREQ, static_cast<unsigned>(duty * 1e6));
}

void drive_motors(double left_u, double right_u) {
    drive_one_motor(M1_IN1, M1_IN2, M1_PWM, -left_u, LEFT_GAIN);
    drive_one_motor(M2_IN1, M2_IN2, M2_PWM, -right_u, RIGHT_GAIN);
}

// ============================================================
// MAIN
// ============================================================

int main() {
    signal(SIGINT, signal_handler);

    if (gpioInitialise() < 0) {
        cerr << "pigpio initialization failed. Run with sudo." << endl;
        return 1;
    }

    // ---------------- GPIO setup ----------------
    gpioSetMode(M1_IN1, PI_OUTPUT);
    gpioSetMode(M1_IN2, PI_OUTPUT);
    gpioSetMode(M1_PWM, PI_OUTPUT);

    gpioSetMode(M2_IN1, PI_OUTPUT);
    gpioSetMode(M2_IN2, PI_OUTPUT);
    gpioSetMode(M2_PWM, PI_OUTPUT);

    gpioSetMode(GPIO_EN, PI_OUTPUT);

    gpioSetMode(M1_ENC_A, PI_INPUT);
    gpioSetMode(M1_ENC_B, PI_INPUT);
    gpioSetMode(M2_ENC_A, PI_INPUT);
    gpioSetMode(M2_ENC_B, PI_INPUT);

    gpioSetPullUpDown(M1_ENC_A, PI_PUD_UP);
    gpioSetPullUpDown(M1_ENC_B, PI_PUD_UP);
    gpioSetPullUpDown(M2_ENC_A, PI_PUD_UP);
    gpioSetPullUpDown(M2_ENC_B, PI_PUD_UP);

    gpioSetAlertFunc(M1_ENC_A, m1_encoder_isr);
    gpioSetAlertFunc(M1_ENC_B, m1_encoder_isr);
    gpioSetAlertFunc(M2_ENC_A, m2_encoder_isr);
    gpioSetAlertFunc(M2_ENC_B, m2_encoder_isr);

    gpioWrite(GPIO_EN, 1);
    stop_motors();

    // ---------------- I2C setup ----------------
    int i2c = i2cOpen(I2C_BUS, LSM6DS33_ADDR, 0);
    if (i2c < 0) {
        cerr << "Failed to open I2C for LSM6DS33." << endl;
        gpioTerminate();
        return 1;
    }

    int who = i2cReadByteData(i2c, WHO_AM_I);
    cout << "WHO_AM_I = 0x" << hex << who << dec << endl;

    // Gyro: ODR 208 Hz, 245 dps
    // CTRL2_G = 0x5C
    if (i2cWriteByteData(i2c, CTRL2_G, 0x5C) < 0) {
        cerr << "Failed to configure gyro." << endl;
    }

    // Accel: ODR 208 Hz, ±2 g, 100 Hz filter default acceptable
    // CTRL1_XL = 0x50
    if (i2cWriteByteData(i2c, CTRL1_XL, 0x50) < 0) {
        cerr << "Failed to configure accel." << endl;
    }

    this_thread::sleep_for(chrono::milliseconds(200));

    // ---------------- IMU calibration ----------------
    cout << "Keep robot perfectly still for IMU calibration..." << endl;

    const int CAL_SAMPLES = 600;
    double ax_bias = 0.0, ay_bias = 0.0, az_bias = 0.0;
    double gx_bias = 0.0, gy_bias = 0.0, gz_bias = 0.0;

    int valid = 0;
    for (int i = 0; i < CAL_SAMPLES; ++i) {
        int16_t axr, ayr, azr, gxr, gyr, gzr;
        if (read_accel_gyro_raw(i2c, axr, ayr, azr, gxr, gyr, gzr)) {
            ax_bias += accel_lsb_to_g(axr);
            ay_bias += accel_lsb_to_g(ayr);
            az_bias += accel_lsb_to_g(azr);

            gx_bias += gyro_lsb_to_rad_s(gxr);
            gy_bias += gyro_lsb_to_rad_s(gyr);
            gz_bias += gyro_lsb_to_rad_s(gzr);
            valid++;
        }
        this_thread::sleep_for(chrono::milliseconds(3));
    }

    if (valid == 0) {
        cerr << "IMU calibration failed: no valid samples." << endl;
        stop_motors();
        i2cClose(i2c);
        gpioTerminate();
        return 1;
    }

    ax_bias /= valid;
    ay_bias /= valid;
    az_bias /= valid;
    gx_bias /= valid;
    gy_bias /= valid;
    gz_bias /= valid;

    // For accel, keep gravity on Z, so remove only offsets conceptually:
    // easiest practical approach:
    // subtract gyro biases; for accel, use angle zero from initial stationary pose.
    int16_t axr, ayr, azr, gxr, gyr, gzr;
    if (!read_accel_gyro_raw(i2c, axr, ayr, azr, gxr, gyr, gzr)) {
        cerr << "Initial IMU read failed." << endl;
        stop_motors();
        i2cClose(i2c);
        gpioTerminate();
        return 1;
    }

    double ax_g = accel_lsb_to_g(axr);
    double az_g = accel_lsb_to_g(azr);
    double theta_acc0 = accel_pitch_rad(ax_g, az_g);

    // Use the initial pose as zero reference
    THETA_OFFSET = theta_acc0;
    double theta_est = 0.0;

    // ---------------- encoder state ----------------
    long prev_m1_counts = m1_encoder_counts.load();
    long prev_m2_counts = m2_encoder_counts.load();

    double wheel_vel_filt = 0.0;
    constexpr double VEL_LP_ALPHA = 0.25;

    bool armed = false;

    cout << fixed << setprecision(3);
    cout << "Starting control loop at " << LOOP_HZ << " Hz" << endl;
    cout << "Tune Kp/Kd/Kv carefully on the real robot." << endl;

    // =========================================================
    // CONTROL LOOP
    // =========================================================
    while (g_running) {
        auto t0 = chrono::steady_clock::now();

        // ----- IMU read -----
        int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
        bool ok = read_accel_gyro_raw(i2c, ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw);

        if (!ok) {
            stop_motors();
            cerr << "\nIMU read error, motors stopped." << endl;
            this_thread::sleep_for(chrono::milliseconds(10));
            continue;
        }

        double ax = accel_lsb_to_g(ax_raw);
        double az = accel_lsb_to_g(az_raw);
        double gy = gyro_lsb_to_rad_s(gy_raw) - gy_bias; // pitch rate about Y

        // accel angle relative to initial zero
        double theta_acc = accel_pitch_rad(ax, az) - THETA_OFFSET;

        // wrap accel angle to [-pi, pi]
        if (theta_acc > M_PI) theta_acc -= 2.0 * M_PI;
        if (theta_acc < -M_PI) theta_acc += 2.0 * M_PI;

        // complementary filter
        theta_est = COMP_ALPHA * (theta_est + gy * DT) + (1.0 - COMP_ALPHA) * theta_acc;

        // ----- encoder velocity -----
        long c1 = m1_encoder_counts.load();
        long c2 = m2_encoder_counts.load();

        long dc1 = c1 - prev_m1_counts;
        long dc2 = c2 - prev_m2_counts;

        prev_m1_counts = c1;
        prev_m2_counts = c2;

        double rev1 = static_cast<double>(dc1) / TOTAL_COUNTS_PER_REV;
        double rev2 = static_cast<double>(dc2) / TOTAL_COUNTS_PER_REV;

        double omega1 = rev1 * 2.0 * M_PI / DT; // rad/s wheel shaft
        double omega2 = rev2 * 2.0 * M_PI / DT;

        double v_wheel = WHEEL_RADIUS * 0.5 * (omega1 + omega2); // m/s average linear velocity
        wheel_vel_filt = VEL_LP_ALPHA * v_wheel + (1.0 - VEL_LP_ALPHA) * wheel_vel_filt;

        // ----- safety / arming -----
        double theta_deg = theta_est * RAD_TO_DEG;

        if (!armed) {
            stop_motors();
            if (fabs(theta_deg) < ARM_ANGLE_DEG) {
                armed = true;
            }

            cout << "\rWaiting for near-upright position... theta=" << setw(7) << theta_deg << " deg" << flush;
            this_thread::sleep_until(t0 + chrono::microseconds((int)(DT * 1e6)));
            continue;
        }

        if (fabs(theta_deg) > FALL_ANGLE_DEG) {
            stop_motors();
            armed = false;
            cout << "\nFall detected. Re-arm by holding robot near upright." << endl;
            this_thread::sleep_until(t0 + chrono::microseconds((int)(DT * 1e6)));
            continue;
        }

        // ----- CONTROL LAW -----
        // Manual says: u = Kp*theta + Kd*theta_dot + Kv*v
        // Sign may need inversion depending on IMU/motor direction.
        // This implementation assumes positive feedback terms here,
        // then applies an overall minus sign to stabilize.
        double u = -(Kp * theta_est + Kd * gy + Kv * wheel_vel_filt);

        u = clamp_val(u, -UMAX, UMAX);

        // same command to both motors for straight balancing
        double left_cmd  = u;
        double right_cmd = u;

        drive_motors(left_cmd, right_cmd);

        // ----- debug print -----
        static int print_div = 0;
        if (++print_div >= 20) { // about 10 Hz console output
            print_div = 0;
            cout << "\rtheta="
                 << setw(7) << theta_deg
                 << " deg | gyroY="
                 << setw(8) << gy * RAD_TO_DEG
                 << " dps | v="
                 << setw(7) << wheel_vel_filt
                 << " m/s | u="
                 << setw(6) << u
                 << " | enc=(" << c1 << "," << c2 << ")     "
                 << flush;
        }

        // ----- fixed timing -----
        this_thread::sleep_until(t0 + chrono::microseconds((int)(DT * 1e6)));
    }

    cout << "\nStopping..." << endl;
    stop_motors();
    gpioWrite(GPIO_EN, 0);
    i2cClose(i2c);
    gpioTerminate();
    return 0;
}
