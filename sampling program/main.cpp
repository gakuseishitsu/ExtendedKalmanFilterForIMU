#include <vector>
#include <string>

#include "mbed.h"
#include "MPU6050.h"
#include "SDFileSystem.h"

#define SD_MOSI     p5
#define SD_MISO     p6
#define SD_SCK      p7
#define SD_CS       p8
#define MPU6000_SDA  p28
#define MPU6000_SCL  p27

SDFileSystem sd(SD_MOSI, SD_MISO, SD_SCK, SD_CS, "sd");
MPU6050 imu(MPU6000_SDA,MPU6000_SCL);
DigitalOut led1 (LED1);
DigitalOut led2 (LED2);
Serial pc(USBTX,USBRX);
Ticker imuTicker;
Ticker timeTicker;
Ticker sdTicker;

const double TIME_INCREMENT = 0.001; // 0.001sec
const double IMU_RATE = 0.01; // 100Hz
const double SD_RATE = 5.0; // 5.0sec
const int SERIAL_BAUD = 115200;
const double SERIAL_RATE = 0.5; //sec
const std::string logFile = "/sd/mydir/DATALOG.txt";

double t = 0.0; // {0.0}って書き方はC++11なのね...
std::vector<double> imuVal(6);
FILE *fp;

void InitUsart(Serial& u, int baud);
void InitImu(MPU6050& mpu6050);
void myLedBlink(DigitalOut& l);
template<typename T>
void updateImuVal(std::vector<T>& vec, MPU6050& mpu6000);
void sampleImu(void);
void timeCount(void);
void updateSD(void);

int main()
{       
    //! Init periph
    InitUsart(pc, SERIAL_BAUD);
    InitImu(imu);
    fp = fopen(logFile.c_str(), "a");
    if(fp == NULL) {
        error("Could not open file for write\r\n");
    }

    //! start main timer
    timeTicker.attach(&timeCount,TIME_INCREMENT);
    //timeTicker.attach([]{t += TIME_INCREMENT;}, TIME_INCREMENT); // mbedではラムダ式が使えないの...  

    //! set tasks
    imuTicker.attach(&sampleImu, IMU_RATE);
    sdTicker.attach(&updateSD,SD_RATE);

    //! main loop
    wait(SERIAL_RATE);
    while(1) {
        //pc.printf("%f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\r\n",t,imuVal[0],imuVal[1],imuVal[2],imuVal[3],imuVal[4],imuVal[5]);
        wait(SERIAL_RATE);
        myLedBlink(led2);
    }

    //return 0; // unreachable
}

void InitUsart(Serial& u, int baud)
{
    u.baud(baud);
}

void InitImu(MPU6050& mpu6050)
{
    mpu6050.setAcceleroRange(MPU6050_ACCELERO_RANGE_16G);
}

void myLedBlink(DigitalOut& l)
{
    l = !l;
}

template<typename T>
void updateImuVal(std::vector<T>& vec, MPU6050& mpu6050)
{
    float acc[3];
    mpu6050.getAccelero(acc);
    float gyr[3];
    mpu6050.getGyro(gyr);
    
    for (int i=0;i<3;i++)
        vec[i] = static_cast<T>(acc[i]);
    for (int i=0;i<3;i++)
        vec[i+3] = static_cast<T>(gyr[i]);
}

void timeCount(void)
{
    t += TIME_INCREMENT;
}

void sampleImu(void)
{
    updateImuVal(imuVal, imu);
    fprintf(fp, "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n",t,imuVal[0],imuVal[1],imuVal[2],imuVal[3],imuVal[4],imuVal[5]);
}

void updateSD(void)
{
    fclose(fp);
    fp = fopen(logFile.c_str(), "a");
    if(fp == NULL) {
        error("Could not open file for write\r\n");
    }
    myLedBlink(led1);
}