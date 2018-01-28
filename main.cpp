
//#include <QCoreApplication>//QT

#include <stdint.h>
#include <iostream>
#include <stdio.h> /* Standard input/output definitions */
#include <string.h> /* String function definitions */
#include <unistd.h> /* UNIX standard function definitions */
#include <fcntl.h> /* File control definitions */
#include <errno.h> /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */

#include <softPwm.h>
#include <math.h>
#include <sys/time.h>
#include <signal.h>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <pthread.h>
#include <wiringPi.h>

#include "lsm6.h"
#include "i2c_bus.h"
#include "Pid.h"
#include "Kalman.h"

#define SLEEP_PERIOD 1000 //us;
#define SERIAL_TIME 100 //ms
#define SAMPLE_TIME 1 //ms
int pwnLimit = 100;

#define KF_VAR_ACCEL 0.0075 // Variance of pressure acceleration noise input.
#define KF_VAR_MEASUREMENT 0.05
#define RESTRICT_PITCH

//physcal pins  -------------need to change
#define PWMR1  31
#define PWMR2  33
#define PWML1  38
#define PWML2  40
#define PWMR  32
#define PWML  37
#define Emagnet 0       
#define LED 0

//encoder define
#define SPD_INT_L 12   //interrupt R Phys:12
#define SPD_PUL_L 16   //Phys:16
#define SPD_INT_R 18   //interrupt L Phys:18
#define SPD_PUL_R 22   //Phys:22


bool m_IsRunning = false;
bool m_IsMainThreadRunning = false;
bool m_IsSerialThreadRunning = false;
bool StopFlag = true;


char buf[500];

int fd; //rfcomm0
int Speed_L,Speed_R;
int pwm,pwm_l,pwm_r;
int Speed_Need = 0;
int Turn_Need = 0;
int Speed_Diff = 0;
int Speed_Diff_ALL = 0;
int Position_AVG = 0;
int Position_Add = 0;

uint32_t timer;
int16_t ax, ay, az;
int16_t gx, gy, gz;

double RAD_TO_DEG = 57.2958;
double timediff = 0.0;
double Correction = 0.0;
double Setpoint = 0.0;
double aggKp = 40.0;
double aggKi = 5.0;
double aggKd = 0.4;
double aggVs = 15.0; //Velocity wheel
double aggKm = 1.0; //Velocity wheel
double angle_error = 0.0;
double Angle_MPU = 0.0;
double Gyro_MPU = 0.0;
double Temperature = 0.0;

double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter


double Input, Output;
//Specify the links and initial tuning parameters
PID balancePID(&Input, &Output, &Setpoint, aggKp, aggKi, aggKd, DIRECT);

lsm6::handle lsm6Data;
Kalman kalmanX;
Kalman kalmanY;

//speed control values
long lastSpeedError = 0;
long speedAdjust = 0;
long dSpeedError;
long SKp ,SKi ,SKd;

double DataAvg[3];

lsm6::comm_config lsm6Config;

void ResetValues()
{
    Setpoint = 0.0;
    Speed_Diff = 0;
    Speed_Diff_ALL = 0;
    Input = 0.0;
    Angle_MPU = 0.0;
    Gyro_MPU = 0.0;
    Temperature = 0.0;
    Speed_Need = 0;
    Turn_Need = 0;
    Speed_L = 0;
    Speed_R = 0;
    Position_Add = 0;
    Position_AVG = 0;
    pwm = 0;
    pwm_l = 0;
    pwm_r = 0;
    //motor speed difference (Left-Right) correction
    lastSpeedError = 0;
    speedAdjust = 0;
    dSpeedError = 0;
    SKp = 1L;
    SKi = 0.5L;
    SKd = 0.3L;
}


void initGyro()
{
    std::string i2c_bus_name = "/dev/i2c-1";

    i2c_bus bus(i2c_bus_name.c_str());


    

    auto addrs = { lsm6::SA0_LOW_ADDR, lsm6::SA0_HIGH_ADDR };
    for (uint8_t addr : addrs)
    {
      int result = bus.try_write_byte_and_read_byte(addr, lsm6::WHO_AM_I);
      if (result == lsm6::LSM6DS33)
      {
        lsm6Config.use_sensor = true;
        lsm6Config.device = (lsm6::device_type)result;
        lsm6Config.i2c_bus_name = i2c_bus_name;
        lsm6Config.i2c_address = (lsm6::i2c_addr)addr;
        break;
      }
    }


 

    lsm6Data.open(lsm6Config);
    lsm6Data.enable();

    printf("initGyro OK\n");


}

void initConnection()
{

}

//get now time
unsigned int micros()
{
    struct timeval timer;
    gettimeofday(&timer, NULL);
    unsigned int time_in_micros = 1000000 * timer.tv_sec + timer.tv_usec;
    return time_in_micros;
}

void calculateGyro()
{
    //currentAngle = 0.9934 * (previousAngle + gyroAngle) + 0.0066 * (accAngle)

    timediff = (micros() - timer)/1000;
    double dt = (double)(micros() - timer) / 1000000; // Calculate delta time

    timer = micros();

    lsm6Data.read_acc();
    lsm6Data.read_gyro();

    ax = lsm6Data.a[0]; ay = lsm6Data.a[1]; az = lsm6Data.a[2];
    gx = lsm6Data.g[0]; gy = lsm6Data.g[1]; gz = lsm6Data.g[2];

    
    // display accel/gyro x/y/z values
    //printf("accel/gyro: %6hd %6hd %6hd   %6hd %6hd %6hd\n",ax,ay,az,gx,gy,gz);

    accX = (int16_t)(ax);
    accY = (int16_t)(ay);
    accZ = (int16_t)(az);

    gyroX = (int16_t)(gx);
    gyroY = (int16_t)(gy);
    gyroZ = (int16_t)(gz);

    //printf("%d %d %d %d %d %d\n",lsm6Data.a[0],lsm6Data.a[1],lsm6Data.a[2],lsm6Data.g[0],lsm6Data.g[1],lsm6Data.g[2]);
   // printf("%.0f %.0f %.0f %.0f %.0f %.0f\n",accX,accY,accZ,gyroX,gyroY,gyroZ);

    // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
    double roll  = atan2(accY, accZ) * RAD_TO_DEG;
    double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
    double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

    double gyroXrate = gyroX / 131.0; // Convert to deg/s
    double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
        kalmanX.setAngle(roll);
        compAngleX = roll;
        kalAngleX = roll;
        gyroXangle = roll;
    } else
        kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

    if (abs(kalAngleX) > 90)
        gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
        kalmanY.setAngle(pitch);
        compAngleY = pitch;
        kalAngleY = pitch;
        gyroYangle = pitch;
    } else
        kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

    if (abs(kalAngleY) > 90)
        gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

    if (gyroXangle < -180 || gyroXangle > 180)
        gyroXangle = kalAngleX;
    if (gyroYangle < -180 || gyroYangle > 180)
        gyroYangle = kalAngleY;

    DataAvg[2] = DataAvg[1];
    DataAvg[1] = DataAvg[0];
    DataAvg[0] = kalAngleX;

    Angle_MPU = (DataAvg[0]+DataAvg[1]+DataAvg[2])/3;

    Gyro_MPU = gyroXrate;
    Temperature = (double)1 / 340.0 + 36.53;
    // printf("Angle_MPU: %.2f  Time_Diff: %.1f\n",Angle_MPU,timediff);

}

void  ExitHandler(int sig)
{
    signal(sig, SIG_IGN);
    m_IsMainThreadRunning = false;
    m_IsSerialThreadRunning = false;
    usleep(1000 * 100);

    printf("\b\bExiting...\n");

    softPwmWrite(PWML, 0);
    softPwmWrite(PWMR, 0);
    exit(0);
}


template<class T>
const T& constrain(const T& x, const T& a, const T& b) {
    if(x < a) {
        return a;
    }
    else if(b < x) {
        return b;
    }
    else
        return x;
}

void correctSpeedDiff() 
{
    dSpeedError = Speed_Diff - lastSpeedError;

    speedAdjust = constrain(int((SKp * Speed_Diff) + (SKi * Speed_Diff_ALL) + (SKd * dSpeedError)), -pwnLimit, pwnLimit);
    lastSpeedError = Speed_Diff;

}


void PWM_Calculate()
{
    Speed_Diff = Speed_R + Speed_L;
    Speed_Diff_ALL += Speed_Diff;

    Setpoint = Correction +  (Speed_Need / 10);
    Input = Angle_MPU;
    angle_error = abs(Setpoint - Input); //distance away from setpoint
    correctSpeedDiff() ;

    float ftmp = 0;
    ftmp = (Speed_L + Speed_R) * 0.5;
    if( ftmp > 0)
        Position_AVG = ftmp +0.5;
    else
        Position_AVG = ftmp -0.5;

    Position_Add += Position_AVG;  //position
    Position_Add = constrain(Position_Add, -pwnLimit, pwnLimit);

    if (angle_error < 10)
    {   //we're close to setpoint, use conservative tuning parameters
        balancePID.SetTunings(aggKp/3, aggKi/3, aggKd/3);
    }
    else
    {   //we're far from setpoint, use aggressive tuning parameters
        balancePID.SetTunings(aggKp,   aggKi,  aggKd);
    }

    balancePID.Compute();

    pwm = -(int)(Output - (Gyro_MPU * aggKd / 5) - (Position_Add / 5));
    //pwm = -(int)(Output);

    pwm_r =int(pwm - aggVs * speedAdjust + Turn_Need);
    pwm_l =int(pwm + aggVs * speedAdjust - Turn_Need);

    //printf("Angle: %.02f  pwm_r: %3d  pwm_l: %3d\n",Angle_MPU,pwm_r,pwm_l);

    Speed_L = 0;
    Speed_R = 0;
}

void Robot_Control()
{
    if (pwm_r>0)
    {
        digitalWrite(PWMR1, HIGH);
        digitalWrite(PWMR2, LOW);
    }

    if (pwm_l>0)
    {
        digitalWrite(PWML1, LOW);
        digitalWrite(PWML2, HIGH);
    }

    if (pwm_r<0)
    {
        digitalWrite(PWMR1, LOW);
        digitalWrite(PWMR2, HIGH);
        pwm_r =- pwm_r;  //cchange to positive
    }

    if (pwm_l<0)
    {
        digitalWrite(PWML1, HIGH);
        digitalWrite(PWML2, LOW);
        pwm_l = -pwm_l;
    }

    if( Angle_MPU > 45 || Angle_MPU < -45 || !m_IsRunning)
    {
        pwm_l = 0;
        pwm_r = 0;
    }

    softPwmWrite(PWML, pwm_l);
    softPwmWrite(PWMR, pwm_r);
}

PI_THREAD (mainThread)
{
    while (1)
    {
        if(!m_IsMainThreadRunning)
        {
            softPwmWrite(PWML, 0);
            softPwmWrite(PWMR, 0);
            continue;
        }

        calculateGyro();
        PWM_Calculate();
        //PWM_Calculate_Pos();
        Robot_Control();

        printf("%.0f, %.0f, %d, %d, %d, %d\n",
        Angle_MPU,
        Gyro_MPU,
        Speed_Need,
        Turn_Need,
        Speed_L,
        Speed_R);
        printf("%d, %d, %d, %.0f, %.0f, %.0f\n",
        pwm,
        pwm_l,
        pwm_r,
        SKp,
        SKi,
        SKd);

        delay(100);
        ::usleep(SLEEP_PERIOD * SAMPLE_TIME);
    }

    return 0;
}

void encodeL (void)
{
    if (digitalRead(SPD_PUL_L))
        Speed_L += 1;
    else
        Speed_L -= 1;

    //printf("Speed_L: %d\n",Speed_L);
}

void encodeR (void)
{
    if (digitalRead(SPD_PUL_R))
        Speed_R += 1;
    else
        Speed_R -= 1;

    //printf("Speed_R: %d\n",Speed_R);
}

void init()
{
    bool islsm6_Found = false;

    m_IsMainThreadRunning = false;
    m_IsSerialThreadRunning = false;

    printf("\nInitializing I2C devices.\n");
    initGyro();

    
    if (wiringPiSetupPhys () < 0)
    {
        fprintf (stderr, "Unable to setup wiringPiSetupGpio: %s\n\n", strerror (errno)) ;
    }
    else
    {
        //set up the GPIO
        pinMode(PWML1, OUTPUT);
        pinMode(PWML2, OUTPUT);
        pinMode(PWMR1, OUTPUT);
        pinMode(PWMR2, OUTPUT);
        pinMode(LED, OUTPUT);
        pinMode(Emagnet, OUTPUT);
        printf("Set pinModes ok.\n");

        //can control the motor's speed
        softPwmCreate(PWML,0,pwnLimit);
        softPwmCreate(PWMR,0,pwnLimit);

        // ISR簡單來說就是中斷會跳去執行的函式
        // 當digital 回傳時會進入encodeL函式
        if (wiringPiISR (SPD_INT_L, INT_EDGE_FALLING, &encodeL) < 0)
        {
            fprintf (stderr, "Unable to setup ISR for left channel: %s\n", strerror (errno));
            return;
        }
        else
        {
            printf("Setup encodeL for left channel successful.\n");
        }

        if (wiringPiISR (SPD_INT_R, INT_EDGE_FALLING, &encodeR) < 0)
        {
            fprintf (stderr, "Unable to setup ISR for right channel: %s\n", strerror (errno));
            return;
        }
        else
        {
            printf("Setup encodeR for right channel successful.\n");
        }
        printf("wiringPiSetupPhys ok.\n\n");

        //初始化傳輸方式
        initConnection();

        //make log file
        //initConf();
        
        //set pid
        balancePID.SetMode(AUTOMATIC);
        balancePID.SetSampleTime(SAMPLE_TIME);
        balancePID.SetOutputLimits(-pwnLimit, pwnLimit);
        printf("setup PID finish\n");

        DataAvg[0]=0; DataAvg[1]=0; DataAvg[2]=0;

        m_IsMainThreadRunning = true;
        int x = piThreadCreate (&mainThread) ;
        if(x == 0)
            printf("mainThread open\n");
        else
            printf("mainThread fall\n");
    }

    timer = micros();
}


int main(int argc, char *argv[])
{
    //QCoreApplication a(argc, argv); //主控台Qt
    signal(SIGINT, ExitHandler);

    if (getuid())
    {
        printf("%s", "You must be root for starting rfcomm.\n");
        //exit(0);
    }

    ResetValues();
    init();
    for (;;)
    {
        delay (5000) ;
    }
    return 0;
    //return a.exec();
}