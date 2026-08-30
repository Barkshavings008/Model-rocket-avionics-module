#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <Adafruit_BMP280.h>
#include <SPI.h>
#include <SdFat.h> // for SD card

#define SERVO_PIN 0
#define MPU6050_ADDRESS 0x68
#define BMP280_I2C_ADDRESS 0x77
#define MPU6050_POWER_REGISTER 0x6B // The adress that is responsible for powering the MPU6050 on and off
#define MPU6050_ACCEL_REGISTER 0x3B // The address that is responsible for the IMU readings
#define LED_PIN 10
#define CLOSED 0
#define OPEN 90
#define PROCEED 1
#define STOP 0
#define TIME_TO_APEX 4000
#define TERMINATION_ANGLE 45
#define SETUP_TIME 5000
#define AIR_PRESSURE_SEA_LEVEL 1025.00
#define SDA_PIN 8
#define SCL_PIN 9
#define MISO_PIN 5
#define MOSI_PIN 6
#define SCK_PIN 4
#define CS_PIN 7

enum State {
    ASCENT,
    DESCENT,
};

struct IMU_data {
    float ax;
    float ay;
    float az;
    float temp;
    float gx;
    float gy;
    float gz;
    float pitch;
    float roll;
};

struct BMP_data {
    float pressure;
    float temperature;
    float altitude;
};

// using enum intiliases the varibale itself as an enum variable
enum State stage = ASCENT; // Initialize the stage to ASCENT

struct IMU_data imu_data;
struct BMP_data bmp_data;

Servo servo;
Adafruit_BMP280 bmp; // Create the Adafruit BMP280 object
SdFat sd;
FsFile dataFile;

////////////////////////
// Function prototypes//
////////////////////////
void wake_up_mpu6050(void);
void wake_up_bmp280(void);
void wake_up_sd_card(void);
struct IMU_data read_mpu6050(void);
struct BMP_data read_bmp280(void);
void setup_nosecone (void);
int open_condition(struct IMU_data imu_data, int start_time, int pitch_offset, int roll_offset);
int average_pitch(struct IMU_data imu_data);
int average_roll(struct IMU_data imu_data);
int average_altitude(struct BMP_data bmp_data);
void print_all_info(struct IMU_data imu_data, struct BMP_data bmp_data, int pitch_offset,
    int roll_offset, int altitude_offset);
void log_data(float pitch, float roll, float altitude);
////////////////////////

int pitch_offset = 0;
int roll_offset = 0;
int altitude_offset = 0;
unsigned long start_time = 0;

void setup() {
    Serial.begin(9600);
    Wire.begin(SDA_PIN, SCL_PIN); // Initialize I2C with specified SDA and SCL pins
    
    wake_up_mpu6050();
    wake_up_bmp280();
    wake_up_sd_card();
    
    servo.attach(SERVO_PIN); 

    pinMode(LED_PIN, OUTPUT);
    setup_nosecone();
    digitalWrite(LED_PIN, LOW); // Ensure LED is off after setup

    delay(1000);
    // to stabilise rocket for readings

    // stops main loop from kicking in until rocket leaves pad

    /*while (imu_data.ay < 2) {
        imu_data = read_mpu6050();
        delay(100);
    }*/

    pitch_offset = average_pitch(imu_data);
    roll_offset = average_roll(imu_data);
    altitude_offset = average_altitude(bmp_data);

    start_time = millis();


    // might need to change direction names based off installation direction of IMU
}

void loop() {
    imu_data = read_mpu6050();
    bmp_data = read_bmp280();
    
    print_all_info(imu_data, bmp_data, pitch_offset, roll_offset, altitude_offset);

    if (open_condition(imu_data, start_time, pitch_offset, roll_offset) == PROCEED && stage == ASCENT) {
        Serial.println("Warning: High tilt detected!");
        servo.write(OPEN);
        digitalWrite(LED_PIN, HIGH);
        stage = DESCENT; // Update the stage to DESCENT after opening the nosecone
    } else if (stage == DESCENT) {
        if (digitalRead(LED_PIN) == LOW) {
            digitalWrite(LED_PIN, HIGH);
        }
        else { // blinks LED at interval according to the delay
            digitalWrite(LED_PIN, LOW); 
        }
    } else {
        servo.write(CLOSED);
        digitalWrite(LED_PIN,LOW);
    }

    log_data(imu_data.pitch - pitch_offset, imu_data.roll - roll_offset,
        bmp_data.altitude - altitude_offset);

    delay(500); // delay will need to be shortened in the real flight
}


/////////////////////////
// Function definitions//
/////////////////////////

void wake_up_mpu6050(void) {
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(MPU6050_POWER_REGISTER); // Writes to the power management register (to wake up the MPU-6050)
    Wire.write(0); // This is that 'wake up' bit
    Wire.endTransmission(); // 'Hangs up' the call
}

struct IMU_data read_mpu6050(void) {
    struct IMU_data imu_data;
    
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(MPU6050_ACCEL_REGISTER); // Starting register for Accel Readings
    Wire.endTransmission(false); 

    // ESP32 requires you explicitly state the no. of bytes to read, and if to send stop bit after reading (the "true")
    Wire.requestFrom((uint8_t)MPU6050_ADDRESS, (uint8_t)14, (uint8_t)true);

    int16_t raw_ax = (Wire.read() << 8) | Wire.read();
    int16_t raw_ay = (Wire.read() << 8) | Wire.read();
    int16_t raw_az = (Wire.read() << 8) | Wire.read();
    int16_t raw_temp = (Wire.read() << 8) | Wire.read(); // Discard temperature
    int16_t raw_gx = (Wire.read() << 8) | Wire.read();
    int16_t raw_gy = (Wire.read() << 8) | Wire.read();
    int16_t raw_gz = (Wire.read() << 8) | Wire.read();

    // 2. Convert to g's and store directly in struct
    imu_data.ax = raw_ax / 16384.0;
    imu_data.ay = raw_ay / 16384.0;
    imu_data.az = raw_az / 16384.0;
    imu_data.temp = raw_temp / 340.0 + 36.53; // Convert to Celsius
    imu_data.gx = raw_gx / 131.0;
    imu_data.gy = raw_gy / 131.0;
    imu_data.gz = raw_gz / 131.0;

    // 3. Pitch & Roll calculations using the struct values
    imu_data.pitch = atan2(imu_data.ax, sqrt(imu_data.ay * imu_data.ay + imu_data.az * imu_data.az)) * 180.0 / M_PI;
    imu_data.roll  = atan2(imu_data.ay, sqrt(imu_data.ax * imu_data.ax + imu_data.az * imu_data.az)) * 180.0 / M_PI;

    return imu_data;
}

void setup_nosecone () {
    servo.write(OPEN);

    unsigned long blink_millis = millis();
    unsigned long currentMillis = millis();

    while (millis() - currentMillis < SETUP_TIME) {
        if (millis() - blink_millis < 300) {
            digitalWrite(LED_PIN, HIGH); // turns on LED for 0.3 seconds
        } else if (millis() - blink_millis < 1000) {
            digitalWrite(LED_PIN, LOW); // turns off LED for 0.7 seconds
        } else {
            blink_millis = millis(); // resets the blink timer every 1 second
        }
        delay(10); // small delay to avoid overwhelming the loop
        // YOU MUST DO THIS WITH ESP 32 SYSTEMS WHICH ARE FAST
    }

    servo.write(CLOSED); 
}

int open_condition(struct IMU_data imu_data, int start_time, int pitch_offset, int roll_offset) {
    if ((imu_data.pitch - pitch_offset > TERMINATION_ANGLE || imu_data.pitch - pitch_offset < -TERMINATION_ANGLE ||
         imu_data.roll - roll_offset > TERMINATION_ANGLE || imu_data.roll - roll_offset < -TERMINATION_ANGLE) &&
        millis() - start_time > TIME_TO_APEX) {
        return PROCEED; // Open the nosecone
    } else {
        return STOP; // Keep the nosecone closed
    }
}

int average_pitch(struct IMU_data imu_data) {
    float average_pitch = 0;
    for (int i = 0; i < 100; i++) {
        imu_data = read_mpu6050();
        average_pitch += imu_data.pitch;
    }
    return average_pitch / 100;
}

int average_roll(struct IMU_data imu_data) {
    float average_roll = 0;
    for (int i = 0; i < 100; i++) {
        imu_data = read_mpu6050();
        average_roll += imu_data.roll;
    }
    return average_roll / 100;
}

int average_altitude(struct BMP_data bmp_data) {
    float average_altitude = 0;
    for (int i = 0; i < 100; i++) {
        bmp_data = read_bmp280();
        average_altitude += bmp_data.altitude;
    }
    return average_altitude / 100;
}

/////////////////////////
/////// BAROMETER ///////
/////////////////////////

void wake_up_bmp280(void) {
    // Initialize BMP280 using the Adafruit Library
    if (!bmp.begin(BMP280_I2C_ADDRESS)) {
        Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    }
    
    // Adafruit BMP280 Default Settings
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

}

struct BMP_data read_bmp280(void) {
    struct BMP_data bmp_data;

    // Use Adafruit Library to fetch values directly
    bmp_data.temperature = bmp.readTemperature(); // Returns standard Celsius
    bmp_data.pressure = bmp.readPressure();       // Returns Pascals (Pa)
    
    // 1013.25 is standard sea level pressure in hPa. 
    // You can adjust this to your local sea level pressure for higher accuracy.
    bmp_data.altitude = bmp.readAltitude(AIR_PRESSURE_SEA_LEVEL); 

    return bmp_data; 
}

void print_all_info(struct IMU_data imu_data, struct BMP_data bmp_data, int pitch_offset,
    int roll_offset, int altitude_offset) {
    Serial.print("Pitch: ");
    Serial.print(imu_data.pitch - pitch_offset);
    Serial.print(" Roll: ");
    Serial.println(imu_data.roll - roll_offset);

    Serial.println("-----------------------------");

    Serial.print("Pressure: ");
    Serial.print(bmp_data.pressure);
    Serial.print(" Temperature: ");
    Serial.println(bmp_data.temperature);

    Serial.println("-----------------------------");

    Serial.print("Altitude: ");
    Serial.println(bmp_data.altitude - altitude_offset);

    Serial.println("-----------------------------");
}

//////////////////////////////
/////// SD CARD READER ///////
/////////////////////////////

void wake_up_sd_card(void) {
    pinMode(CS_PIN, OUTPUT);
    digitalWrite(CS_PIN, HIGH);          // deselect until we're ready
    SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);  // configure the bus

    if (!sd.begin(CS_PIN, SD_SCK_MHZ(16))) { // Initialize the SD card at 16 MHz
        Serial.println("SD card failed!");

        // Blink the LED to indicate SD card failure
        unsigned long error_millis = millis();
        while (millis() - error_millis < 5000) {
            digitalWrite(LED_PIN, HIGH);
            delay(200);
            digitalWrite(LED_PIN, LOW);
            delay(800);
            digitalWrite(LED_PIN, HIGH);
            delay(800);
            digitalWrite(LED_PIN, LOW);
            delay(200);
        }
        return;
    }

    Serial.println("SD card connected!");

    // dataFile = sd.open("flight_data.txt", FILE_WRITE);
    dataFile = sd.open("flight_data.txt", O_RDWR | O_CREAT | O_AT_END);
    // Open the file for reading and writing, create it if it doesn't exist, and append to the end

    if (!dataFile) {
        Serial.println("Failed to open flight_data.txt");
    }
}

void log_data(float pitch, float roll, float altitude) {
    if (dataFile) {
        dataFile.print("Pitch: ");
        dataFile.print(pitch);
        dataFile.print(", Roll: ");
        dataFile.print(roll);
        dataFile.print(", Altitude: ");
        dataFile.println(altitude);
        dataFile.flush(); // Ensure data is written to the SD card
    } else {
        Serial.println("Data file not open!");
    }
}