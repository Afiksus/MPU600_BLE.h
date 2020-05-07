// ����������� ��� ������������ ������������� �� ���� I2C
        MPU_6050(TwoWire &w, byte ADDR);

        // ����������� ��� ������������ � ������� ������������� ��� ������� ������� 
	    MPU_6050(TwoWire &w, byte ADDR, bool Kenable, float Kalman);

        // ������� �������� ������ data �� ������� reg ������� ADDR 
        void i2cWrite(byte reg, byte data);

        // ��������� ������������� ���������� �����������
        void DefaultSettings();

        // ��������� ������������� ����������������� �����������
        void UserSettings(byte SMPLRT, byte CONFIG, byte GYRO_CONFIG, byte ACCEL_CONFIG, byte PWR_MGMT_1, byte PWR_MGMT_2);

        // �������� �� ���������� ���������� ������ � ��������� 59-72 mpu6050
        void GetAllData(byte reg);

        // �������� ������������ ���������� ���� �� ����������������� �����
        void SendData(const byte& DATA);

        // �������������� �������� - �������� �����
        void DetectMove();

        // ����������� �������� � �������� ������������ ����
        void ConvertMouse();

        // ���������� ����� � ������������ �������� reg �� ������ ADDR
        byte i2cRead(byte reg);

        // ������� ������������ ������ ���������������� ������
        byte SensitivityAccel(byte DATA);

        // ������� ������������ ������ ���������������� ���������
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
