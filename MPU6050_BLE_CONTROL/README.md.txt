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

        float GetAccX();
        float GetAccY() ;
        float GetAccZ() ;
        
        float GetAngleAccX(); 
        float GetAngleAccY();
        float GetAngleAccZ();

        float GetAngleGyroX(); 
        float GetAngleGyroY(); 
        float GetAngleGyroZ(); 

        float GetGyroX(); 
        float GetGyroY(); 
        float GetGyroZ(); 

        float GetFilterAngleX(); 
        float GetFilterAngleY();
        float GetFilterAngleZ();

        bool isKalmanEnable();

        void SetKalmanEnable(bool Kenable);
        void SetKalmanKoef(float Kalman); 
        
        int16_t GetRawAccX();
        int16_t GetRawAccY();
        int16_t GetRawAccZ();

        int16_t GetRawGyroX();
        int16_t GetRawGyroY();
        int16_t GetRawGyroZ();

        void setAccuracy(float accuracy);
        void SetMaxMeasure();
        void SetMinMeasure();
