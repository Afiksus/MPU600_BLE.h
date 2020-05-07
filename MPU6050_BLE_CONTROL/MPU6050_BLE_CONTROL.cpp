#include "MPU6050_BLE_CONTROL.h"
#include "Arduino.h"

#include<Wire.h>



//MPU_6050::MPU_6050()
//{

//}

MPU_6050::MPU_6050(TwoWire &w, byte ADDR)
{
    Wire = &w;
    this->ADDR = ADDR;
    Kenable = true;
	Kalman = DEFAULT_KALMAN;
    Serial.begin(9600);
    
}

MPU_6050::MPU_6050(TwoWire &w, byte ADDR, bool Kenable, float Kalman)
{
    Wire = &w;
    this->ADDR = ADDR;
    this->Kenable = Kenable;
	this->Kalman = Kalman;
    Serial.begin(9600);
}

void MPU_6050::i2cWrite(byte reg, byte data)
{
    Wire->beginTransmission(ADDR);
    Wire->write(reg);
    Wire->write(data);
    Wire->endTransmission(true);
}

void MPU_6050::DefaultSettings()
{
    i2cWrite(MPU6050_SMPLRT_DIV, 0x01);
    i2cWrite(MPU6050_CONFIG, 0x03);
    // i2cWrite(MPU6050_GYRO_CONFIG, 0x08, ADDR);
    // i2cWrite(MPU6050_ACCEL_CONFIG, 0x18, ADDR);
    SensitivityGyro(0x08);
    SensitivityAccel(0x18);
    i2cWrite(MPU6050_PWR_MGMT_1, 0x09);
    i2cWrite(MPU6050_PWR_MGMT_2, 0x00);
    setAccuracy(DEFAULT_ACCURACY);
    SetMinMeasure();
    SetMaxMeasure();
}

void MPU_6050::UserSettings(byte SMPLRT, byte CONFIG, byte GYRO_CONFIG, byte ACCEL_CONFIG, byte PWR_MGMT_1, byte PWR_MGMT_2)
{
    i2cWrite(MPU6050_SMPLRT_DIV, SMPLRT);
    i2cWrite(MPU6050_CONFIG, CONFIG);
    SensitivityGyro(GYRO_CONFIG);
    SensitivityAccel(ACCEL_CONFIG);
    i2cWrite(MPU6050_PWR_MGMT_1, PWR_MGMT_1);
    i2cWrite(MPU6050_PWR_MGMT_2, PWR_MGMT_2);
    setAccuracy(DEFAULT_ACCURACY);
    SetMinMeasure();
    SetMaxMeasure();
}

byte MPU_6050::SensitivityAccel(byte DATA)
{
    i2cWrite(MPU6050_ACCEL_CONFIG, DATA);
        // ignore the bits 5-7
    switch (DATA) {
            case 0:
                SensAccel = ACCEL_2G;
                break;
            case 8:
                SensAccel = ACCEL_4G;
                break;
            case 16:
                SensAccel = ACCEL_8G;
                break;
            case 24:
                SensAccel = ACCEL_16G;
                break;
            default:
                return error;
            }

    return i2cRead(MPU6050_ACCEL_CONFIG);
}

byte MPU_6050::SensitivityGyro(byte DATA)
{
    i2cWrite(MPU6050_GYRO_CONFIG, DATA);
        // ignore the bits 5-7
    switch (DATA) {
            case 0:
                SensGyro = GYRO_250DEG;
                break;
            case 8:
                SensGyro = GYRO_500DEG;
                break;
            case 16:
                SensGyro = GYRO_1000DEG;
                break;
            case 24:
                SensGyro = GYRO_2000DEG;
                break;
            default:
                return error;
            }

    return i2cRead(MPU6050_GYRO_CONFIG);
}

void MPU_6050::GetAllData(byte reg)
{
      // начало связи с модулем по адресу ADDR     
Wire->beginTransmission(ADDR);   
        // связь с необходимым регистром
Wire->write(reg);
        // конец передачи данных
Wire->endTransmission(true);
        // принять с модуля 14 байтов данных
Wire->requestFrom((int)ADDR, 14, (int)true);
   
   // принятие восьмибитных данных с каждого регистра - смещение старших битов на 8 разрядов влево
    rawAccX =  Wire->read() << 8 | Wire->read();
	rawAccY =  Wire->read() << 8 | Wire->read();
	rawAccZ =  Wire->read() << 8 | Wire->read();
	           Wire->read() << 8 | Wire->read();
	rawGyroX = Wire->read() << 8 | Wire->read();
	rawGyroY = Wire->read() << 8 | Wire->read();
	rawGyroZ = Wire->read() << 8 | Wire->read();
 
    // превращение сырых данных акселерометра в ускорение при SensAccel чувствительности прибора
    AccX = ((float)rawAccX) / SensAccel;        
    AccY = ((float)rawAccY) / SensAccel;
    AccZ = ((float)rawAccZ) / SensAccel;

    // угол поворота за показаниями акселерометра
    angleAccX = atan(AccX / sqrt(AccY*AccY+AccZ*AccZ))*(float)RAD_DEG;
    angleAccY = atan(AccY / sqrt(AccX*AccX+AccZ*AccZ))*(float)RAD_DEG;
    angleAccZ = atan(AccZ / sqrt(AccX*AccX+AccY*AccY))*(float)RAD_DEG;

    // скорость поворота по гироскопу при SensGyro чувствительности прибора
    GyroX = ((float)rawGyroX) / SensGyro;
	GyroY = ((float)rawGyroY) / SensGyro;
	GyroZ = ((float)rawGyroZ) / SensGyro;

    // промежуток времени, между которыми делались два последовательных измерения
    interval = (millis() - preInterval) * 0.001;

    // подсчёт угла за показаниями гироскопа
    angleGyroX += GyroX * interval;
	angleGyroY += GyroY * interval;
	angleGyroZ += GyroZ * interval;

    // Kalman Filter - фильтр Калмана для фильтрации данных
    if (Kalman) 
    {
    angleX = Kalman * angleGyroX + (1 - Kalman) * angleAccX;
    angleY = Kalman * angleGyroY + (1 - Kalman) * angleAccY;
    angleZ = Kalman * angleGyroZ + (1 - Kalman) * angleAccZ;
    }
    // отсчитываем новый интервал времени
    preInterval = millis();

}

byte MPU_6050::i2cRead(byte reg)
{
    Wire->beginTransmission(ADDR);   
        // связь с необходимым регистром
        Wire->write(reg);
        // конец передачи данных
        Wire->endTransmission(true);
        // принять с модуля 1 байт данных
        Wire->requestFrom((int)ADDR, 1, (int)true);

            return Wire->read();
}

void MPU_6050::DetectMove()
{
   
    
   
   
    // точность измерений, в котором считается уровень движений - ноль
    if (AccX >= (MIDDLE_MEASUREMENTS - accuracy) && AccX <= (MIDDLE_MEASUREMENTS + accuracy)){
        AccX = MIDDLE_MEASUREMENTS;
    }
    if (AccY >= (MIDDLE_MEASUREMENTS - accuracy) && AccY <= (MIDDLE_MEASUREMENTS + accuracy)){
        AccY = MIDDLE_MEASUREMENTS;
    }

    // // вычисление был ли клик мыши
    // if (AccZ_vcc >= (max_transmitt - accuracy) && mode == true){
    //     click = 1;
    // } else {
    //     click = 0;
    // }


    // здесь функция подсчёта перемещения мыши по осям

    ConvertMouse();
}

void MPU_6050::ConvertMouse()
{
    //unsigned long timer;
    uint8_t Ax = AccX * 100;
    uint8_t Ay = AccY * 100;
    uint8_t Az = AccZ * 100;
    
      Ax = (uint8_t)map(Ax, minMeasure * 100, maxMeasure * 100, 0, 255);
      Ay = (uint8_t)map(Ay, minMeasure * 100, maxMeasure * 100, 0, 255);
      Az = (uint8_t)map(Az, minMeasure * 100, maxMeasure * 100, 0, 255);
    
}

void MPU_6050::SendData(const byte& DATA)
{
    Serial.write(&DATA, sizeof(DATA));
}