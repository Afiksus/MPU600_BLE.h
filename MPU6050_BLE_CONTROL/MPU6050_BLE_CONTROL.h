#ifndef MPU6050_BLE_CONTROL
#define MPU6050_BLE_CONTROL

//#if ARDUINO >= 100
#include <Arduino.h>
//#else

#include<Wire.h>

typedef unsigned char byte ;

#define MPU_ADDR 0x68
#define MPU_ADDR_ALT 0X69
#define MPU6050_ADDR_FINGER 0x69
#define MPU6050_ADDR_ARM 0X68
// набор изменяемых регистров с их адресами
#define MPU6050_SMPLRT_DIV   0x19 // 25 Sample rate частота дискретизации
#define MPU6050_CONFIG       0x1a // 26 DLPF and FSYNC фильтр нижних частот и внешняя синхронизация кадров
#define MPU6050_GYRO_CONFIG  0x1b // 27 full scale range gyro - настройка чувствительности гироскопа
#define MPU6050_ACCEL_CONFIG 0x1c // 28 full scale range accel - настройка чувствительности акселерометра
#define MPU6050_WHO_AM_I     0x75 // 117 идентификация устройства - вывод шестибитного адреса
#define MPU6050_PWR_MGMT_1   0x6b // 107 power management 1 - power mode - режим работы внутреннего генератора
#define MPU6050_PWR_MGMT_2   0x6c // 108 power management 2 - settings in Low Power Mode

#define RAD_DEG 57.2957795131
#define DEFAULT_KALMAN  0.98      // Коэффициент фильтрации полученного угла
#define error -1
// Чувствительность аксселерометра соответственно 2g, 4g, 8g, 16g
#define ACCEL_2G        16384.0   
#define ACCEL_4G        8192.0
#define ACCEL_8G        4096.0
#define ACCEL_16G       2048.0
// Чувствительность гироскопа соответственно 250deg, 500deg, 1000deg, 2000deg
#define GYRO_250DEG     131.0
#define GYRO_500DEG     65.5
#define GYRO_1000DEG    32.8
#define GYRO_2000DEG    16.4

#define DEFAULT_MIN_MEASURE -2
#define DEFAULT_MAX_MEASURE 2

#define DEFAULT_ACCURACY 0.15
#define MIDDLE_MEASUREMENTS 0


class MPU_6050 {

    private:

    TwoWire *Wire;

    unsigned long preInterval;
    float interval; // промежутки времени между измерениями
    float accuracy, minMeasure, maxMeasure;
    float AccX, AccY, AccZ;     // ускорение по осям акселерометра
    float angleAccX, angleAccY, angleAccZ; // данные dугла по акселерометру
    float angleGyroX, angleGyroY, angleGyroZ; // угол по данным гироскопа
    float angleX, angleY, angleZ; // сам угол уже фильтрованный
    float GyroX, GyroY, GyroZ; // скорость поворота в углах
    float Kalman;
    uint16_t SensAccel, SensGyro;
    int16_t rawAccX, rawAccY, rawAccZ, rawGyroX, rawGyroY, rawGyroZ; // сырые данные с модуля
    byte ADDR;
    bool Kenable;

    

    public:
        
        //MPU_6050();
        // конструктор для иницилизации акселерометра на шину I2C
        MPU_6050(TwoWire &w, byte ADDR);

        // конструктор для иницилизации и задания коэффициентов для фильтра Калмана 
	    MPU_6050(TwoWire &w, byte ADDR, bool Kenable, float Kalman);

        // функция отправки данных data на регистр reg прибора ADDR 
        void i2cWrite(byte reg, byte data);

        // настройка акселерометра дефолтными параметрами
        void DefaultSettings();

        // настройка акселерометра пользовательскими параметрами
        void UserSettings(byte SMPLRT, byte CONFIG, byte GYRO_CONFIG, byte ACCEL_CONFIG, byte PWR_MGMT_1, byte PWR_MGMT_2);

        // Записать во внутренние переменные данные с регистров 59-72 mpu6050
        void GetAllData(byte reg);

        // Передача определённого количества байт по последовательному порту
        void SendData(const byte& DATA);

        // Детектирование движения - движение мышью
        void DetectMove();

        // конвертация движения в смещение компьютерной мыши
        void ConvertMouse();

        // Считывание байта с определённого регистра reg по адресу ADDR
        byte i2cRead(byte reg);

        // Задание необходимого уровня чувствительности акселя
        byte SensitivityAccel(byte DATA);

        // Задание необходимого уровня чувствительности гироскопа
        byte SensitivityGyro(byte DATA);

        float GetAccX() { return AccX; }
        float GetAccY() { return AccY; }
        float GetAccZ() { return AccZ; }
        
        float GetAngleAccX() { return angleAccX; }
        float GetAngleAccY() { return angleAccY; }
        float GetAngleAccZ() { return angleAccZ; }

        float GetAngleGyroX() { return angleGyroX; }
        float GetAngleGyroY() { return angleGyroY; }
        float GetAngleGyroZ() { return angleGyroZ; }

        float GetGyroX() { return GyroX; }
        float GetGyroY() { return GyroY; }
        float GetGyroZ() { return GyroZ; }

        float GetFilterAngleX() { return Kenable == true ? angleX : error; }
        float GetFilterAngleY() { return Kenable == true ? angleY : error; }
        float GetFilterAngleZ() { return Kenable == true ? angleZ : error; }

        bool isKalmanEnable() { return Kenable; }

        void SetKalmanEnable(bool Kenable) { this->Kenable = Kenable; }
        void SetKalmanKoef(float Kalman) { Kalman > 1 ? this->Kalman = DEFAULT_KALMAN : this->Kalman = Kalman; } 
        
        int16_t GetRawAccX() { return rawAccX; }
        int16_t GetRawAccY() { return rawAccY; }
        int16_t GetRawAccZ() { return rawAccZ; }

        int16_t GetRawGyroX() { return rawGyroX; }
        int16_t GetRawGyroY() { return rawGyroY; }
        int16_t GetRawGyroZ() { return rawGyroZ; }

        void setAccuracy(float accuracy) { this->accuracy = accuracy; }
        void SetMaxMeasure() { this->maxMeasure = 2*(ACCEL_16G / SensAccel); }
        void SetMinMeasure() { this->minMeasure = -2*(ACCEL_16G / SensAccel); }


};

#endif

