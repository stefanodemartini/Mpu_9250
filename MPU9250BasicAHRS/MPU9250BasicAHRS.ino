#include <Wire.h>
//Magnetometer Registers
#define AK8963_ADDRESS   0x0C
#define AK8963_WHO_AM_I  0x00 // should return 0x48
#define AK8963_INFO      0x01
#define AK8963_ST1       0x02  // data ready status bit 0
#define AK8963_XOUT_L	 0x03  // data
#define AK8963_XOUT_H	 0x04
#define AK8963_YOUT_L	 0x05
#define AK8963_YOUT_H	 0x06
#define AK8963_ZOUT_L	 0x07
#define AK8963_ZOUT_H	 0x08
#define AK8963_ST2       0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL      0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASTC      0x0C  // Self test control
#define AK8963_I2CDIS    0x0F  // I2C disable
#define AK8963_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY      0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ      0x12  // Fuse ROM z-axis sensitivity adjustment value

#define SELF_TEST_X_GYRO 0x00
#define SELF_TEST_Y_GYRO 0x01
#define SELF_TEST_Z_GYRO 0x02

/*#define X_FINE_GAIN      0x03 // [7:0] fine gain
  #define Y_FINE_GAIN      0x04
  #define Z_FINE_GAIN      0x05
  #define XA_OFFSET_H      0x06 // User-defined trim values for accelerometer
  #define XA_OFFSET_L_TC   0x07
  #define YA_OFFSET_H      0x08
  #define YA_OFFSET_L_TC   0x09
  #define ZA_OFFSET_H      0x0A
  #define ZA_OFFSET_L_TC   0x0B */

#define SELF_TEST_X_ACCEL 0x0D
#define SELF_TEST_Y_ACCEL 0x0E
#define SELF_TEST_Z_ACCEL 0x0F

#define SELF_TEST_A      0x10

#define XG_OFFSET_H      0x13  // User-defined trim values for gyroscope
#define XG_OFFSET_L      0x14
#define YG_OFFSET_H      0x15
#define YG_OFFSET_L      0x16
#define ZG_OFFSET_H      0x17
#define ZG_OFFSET_L      0x18
#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D
#define LP_ACCEL_ODR     0x1E
#define WOM_THR          0x1F

#define MOT_DUR          0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define ZMOT_THR         0x21  // Zero-motion detection threshold bits [7:0]
#define ZRMOT_DUR        0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

#define FIFO_EN          0x23
#define I2C_MST_CTRL     0x24
#define I2C_SLV0_ADDR    0x25
#define I2C_SLV0_REG     0x26
#define I2C_SLV0_CTRL    0x27
#define I2C_SLV1_ADDR    0x28
#define I2C_SLV1_REG     0x29
#define I2C_SLV1_CTRL    0x2A
#define I2C_SLV2_ADDR    0x2B
#define I2C_SLV2_REG     0x2C
#define I2C_SLV2_CTRL    0x2D
#define I2C_SLV3_ADDR    0x2E
#define I2C_SLV3_REG     0x2F
#define I2C_SLV3_CTRL    0x30
#define I2C_SLV4_ADDR    0x31
#define I2C_SLV4_REG     0x32
#define I2C_SLV4_DO      0x33
#define I2C_SLV4_CTRL    0x34
#define I2C_SLV4_DI      0x35
#define I2C_MST_STATUS   0x36
#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define DMP_INT_STATUS   0x39  // Check DMP interrupt
#define INT_STATUS       0x3A
#define ACCEL_XOUT_H     0x3B
#define ACCEL_XOUT_L     0x3C
#define ACCEL_YOUT_H     0x3D
#define ACCEL_YOUT_L     0x3E
#define ACCEL_ZOUT_H     0x3F
#define ACCEL_ZOUT_L     0x40
#define TEMP_OUT_H       0x41
#define TEMP_OUT_L       0x42
#define GYRO_XOUT_H      0x43
#define GYRO_XOUT_L      0x44
#define GYRO_YOUT_H      0x45
#define GYRO_YOUT_L      0x46
#define GYRO_ZOUT_H      0x47
#define GYRO_ZOUT_L      0x48
#define EXT_SENS_DATA_00 0x49
#define EXT_SENS_DATA_01 0x4A
#define EXT_SENS_DATA_02 0x4B
#define EXT_SENS_DATA_03 0x4C
#define EXT_SENS_DATA_04 0x4D
#define EXT_SENS_DATA_05 0x4E
#define EXT_SENS_DATA_06 0x4F
#define EXT_SENS_DATA_07 0x50
#define EXT_SENS_DATA_08 0x51
#define EXT_SENS_DATA_09 0x52
#define EXT_SENS_DATA_10 0x53
#define EXT_SENS_DATA_11 0x54
#define EXT_SENS_DATA_12 0x55
#define EXT_SENS_DATA_13 0x56
#define EXT_SENS_DATA_14 0x57
#define EXT_SENS_DATA_15 0x58
#define EXT_SENS_DATA_16 0x59
#define EXT_SENS_DATA_17 0x5A
#define EXT_SENS_DATA_18 0x5B
#define EXT_SENS_DATA_19 0x5C
#define EXT_SENS_DATA_20 0x5D
#define EXT_SENS_DATA_21 0x5E
#define EXT_SENS_DATA_22 0x5F
#define EXT_SENS_DATA_23 0x60
#define MOT_DETECT_STATUS 0x61
#define I2C_SLV0_DO      0x63
#define I2C_SLV1_DO      0x64
#define I2C_SLV2_DO      0x65
#define I2C_SLV3_DO      0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL  0x69
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C
#define DMP_BANK         0x6D  // Activates a specific bank in the DMP
#define DMP_RW_PNT       0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG          0x6F  // Register in DMP from which to read or to which to write
#define DMP_REG_1        0x70
#define DMP_REG_2        0x71
#define FIFO_COUNTH      0x72
#define FIFO_COUNTL      0x73
#define FIFO_R_W         0x74
#define WHO_AM_I_MPU9250 0x75 // Should return 0x71
#define XA_OFFSET_H      0x77
#define XA_OFFSET_L      0x78
#define YA_OFFSET_H      0x7A
#define YA_OFFSET_L      0x7B
#define ZA_OFFSET_H      0x7D
#define ZA_OFFSET_L      0x7E

// Using the MSENSR-9250 breakout board, ADO is set to 0
// Seven-bit device address is 110100 for ADO = 0 and 110101 for ADO = 1
#define ADO 0
#if ADO
#define MPU9250_ADDRESS 0x69  // Device address when ADO = 1
#else
#define MPU9250_ADDRESS 0x68  // Device address when ADO = 0
#define AK8963_ADDRESS 0x0C   //  Address of magnetometer
#endif

#define AHRS true         // set to false for basic data read
#define SerialDebug true   // set to true to get Serial output for debugging

// Set initial input parameters
enum Ascale {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};

enum Gscale {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};

enum Mscale {
  MFS_14BITS = 0, // 0.6 mG per LSB
  MFS_16BITS      // 0.15 mG per LSB
};

// Specify sensor full scale
uint8_t Gscale = GFS_250DPS;
uint8_t Ascale = AFS_16G;
uint8_t Mscale = MFS_16BITS; // Choose either 14-bit or 16-bit magnetometer resolution
uint8_t Mmode = 0x06;        // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read


float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors

// Pin definitions
int intPin = 14;  // These can be changed, 2 and 3 are the Arduinos ext int pins
int myLed = 13; // Set up pin 13 led for toggling

int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output

float magCalibration[3] = {0, 0, 0};  // Factory mag calibration and mag bias
float magBias[3], magScale[3];
float gyroBias[3] = {0, 0, 0};
float accelBias[3] = {0, 0, 0};      // Bias corrections for gyro and accelerometer
int16_t tempCount;      // temperature raw count output
float   temperature;    // Stores the real internal chip temperature in degrees Celsius
float   SelfTest[6];    // holds results of gyro and accelerometer self test

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
float GyroMeasError = PI * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = PI * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
// There is a tradeoff in the beta parameter between accuracy and response speed.

// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense;
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy.
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.

float beta = sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

uint32_t delt_t = 0; // used to control display output rate
uint32_t count = 0;
uint32_t sumCount = 0; // used to control display output rate

float pitch, yaw, roll;
float a12, a22, a31, a32, a33;
float deltat = 0.0f;
float sum = 0.0f;                           // integration interval for both filter schemes
uint32_t lastUpdate = 0;
uint32_t firstUpdate = 0;                   // used to calculate integration interval
uint32_t Now = 0;                           // used to calculate integration interval

float ax, ay, az, gx, gy, gz, mx, my, mz;   // variables to hold latest sensor data values
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};      // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};         // vector to hold integral error for Mahony method


void setup()
{
  Wire.begin();
  Serial.begin(115200);

  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(intPin, INPUT);
  digitalWrite(intPin, LOW);
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH);

  // Read the WHO_AM_I register, this is a good test of communication
  byte c = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
  Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x71, HEX);

  if (c == 0x71) // WHO_AM_I should always be 0x68
  {
    Serial.println("MPU9250 is online...");
    MPU9250SelfTest(SelfTest); // Start by performing self test and reporting values

    if (SerialDebug) {
      Serial.print("x-axis self test: acceleration trim within : "); Serial.print(SelfTest[0], 1); Serial.println("% of factory value");
      Serial.print("y-axis self test: acceleration trim within : "); Serial.print(SelfTest[1], 1); Serial.println("% of factory value");
      Serial.print("z-axis self test: acceleration trim within : "); Serial.print(SelfTest[2], 1); Serial.println("% of factory value");
      Serial.print("x-axis self test: gyration trim within : "); Serial.print(SelfTest[3], 1); Serial.println("% of factory value");
      Serial.print("y-axis self test: gyration trim within : "); Serial.print(SelfTest[4], 1); Serial.println("% of factory value");
      Serial.print("z-axis self test: gyration trim within : "); Serial.print(SelfTest[5], 1); Serial.println("% of factory value");
    }

    calibrateMPU9250(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers

    initMPU9250();
    Serial.println("MPU9250 initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature

    // Read the WHO_AM_I register of the magnetometer, this is a good test of communication
    byte d = readByte(AK8963_ADDRESS, AK8963_WHO_AM_I);  // Read WHO_AM_I register for AK8963
    Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX); Serial.print(" I should be "); Serial.println(0x48, HEX);

    // Get magnetometer calibration from AK8963 ROM

    initAK8963(magCalibration);

    Serial.println("AK8963 initialized for active data mode...."); // Initialize device for active mode read of magnetometer

    calibrateAK8963(magBias, magScale);

    if (SerialDebug) {
      Serial.println("Calibration values: ");
      Serial.print("X-Axis sensitivity adjustment value "); Serial.println(magCalibration[0], 2);
      Serial.print("Y-Axis sensitivity adjustment value "); Serial.println(magCalibration[1], 2);
      Serial.print("Z-Axis sensitivity adjustment value "); Serial.println(magCalibration[2], 2);
    }
  }
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);
    while (1) ; // Loop forever if communication doesn't happen
  }
}

void loop()
{
  // If intPin goes high, all data registers have new data
  if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {  // On interrupt, check if data ready interrupt

    readAccelData(accelCount);  // Read the x/y/z adc values
    getAres();

    // Now we'll calculate the acceleration value into actual g's
    ax = (float)accelCount[0] * aRes ;//- accelBias[0];  // get actual g value, this depends on scale being set
    ay = (float)accelCount[1] * aRes ;//- accelBias[1];
    az = (float)accelCount[2] * aRes ;//- accelBias[2];

    readGyroData(gyroCount);  // Read the x/y/z adc values
    getGres();

    // Calculate the gyro value into actual degrees per second
    gx = (float)gyroCount[0] * gRes; // get actual gyro value, this depends on scale being set
    gy = (float)gyroCount[1] * gRes;
    gz = (float)gyroCount[2] * gRes;

    readMagData(magCount);  // Read the x/y/z adc values

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental corrections

    mx = (float)magCount[0] * mRes * magCalibration[0] - magBias[0]; // get actual magnetometer value, this depends on scale being set
    my = (float)magCount[1] * mRes * magCalibration[1] - magBias[1];
    mz = (float)magCount[2] * mRes * magCalibration[2] - magBias[2];
  }

  Now = micros();
  //deltat è la differenza fra il valore di now appena valorizzato sopra e il valore di lastupdate valorizzato di seguito nel loop precendente
  deltat = ((Now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update.

  lastUpdate = Now;
  sum += deltat; // sum for averaging filter update rate. Aggiungo deltat a sum
  sumCount++; //aggiungo 1 a sunCout ogni volta che passo di qui

  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of the magnetometer;
  // the magnetometer z-axis (+ down) is opposite to z-axis (+ up) of accelerometer and gyro!
  // We have to make some allowance for this orientationmismatch in feeding the output to the quaternion filter.
  // For the MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward along the x-axis just like in the LSM9DS0 sensor. This rotation can be modified to allow any convenient orientation convention.
  // This is ok by aircraft orientation standards!
  // Pass gyro rate as rad/s

  MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz);
  //  MahonyQuaternionUpdate(ax, ay, az, gx * PI / 180.0f, gy * PI / 180.0f, gz * PI / 180.0f, my, mx, mz);
  //   MahonyQuaternionUpdate(ax, ay, az, gx * PI / 180.0f, gy * PI / 180.0f, gz * PI / 180.0f, my, mx, -mz);
 // MadgwickQuaternionUpdate(-ax, ay, az, gx, -gy, -gz, my, -mx, mz);



  if (!AHRS) {
    delt_t = millis() - count;
    if (SerialDebug) {
      // Print acceleration values in milligs!
      Serial.print("X-acceleration: "); Serial.print(1000 * ax); Serial.print(" mg "); Serial.print("\t");
      Serial.print("Y-acceleration: "); Serial.print(1000 * ay); Serial.print(" mg "); Serial.print("\t");
      Serial.print("Z-acceleration: "); Serial.print(1000 * az); Serial.print(" mg "); Serial.print("\t");

      // Print gyro values in degree/sec
      Serial.print("X-gyro rate: "); Serial.print(gx, 3); Serial.print(" deg/sec ");Serial.print("\t");
      Serial.print("Y-gyro rate: "); Serial.print(gy, 3); Serial.print(" deg/sec ");Serial.print("\t");
      Serial.print("Z-gyro rate: "); Serial.print(gz, 3); Serial.print(" deg/sec");Serial.print("\t");

      // Print mag values in milli Gauss
      Serial.print("X-mag field: "); Serial.print(mx); Serial.print(" mG ");Serial.print("\t");
      Serial.print("Y-mag field: "); Serial.print(my); Serial.print(" mG ");Serial.print("\t");
      Serial.print("Z-mag field: "); Serial.print(mz); Serial.println(" mG");

      tempCount = readTempData();  // Read the adc values
      temperature = ((float) tempCount) / 333.87 + 21.0; // Temperature in degrees Centigrade
      // Print temperature in degrees Centigrade
      Serial.print("Temperature is ");  Serial.print(temperature, 1);  Serial.println(" degrees C"); // Print T values to tenths of s degree C
    }

    count = millis();
    digitalWrite(myLed, !digitalRead(myLed));  // toggle led

  }
  else {

    //count = valorizzato alla fine del loop con millis()
    delt_t = millis() - count;
    //delt_t = differenza di tempo che è intercorsa fra un refresh della visualizzazione dei dati e l'altro

    if (SerialDebug) {
      Serial.print (deltat); Serial.print("\t");
      Serial.print("ax="); Serial.print((int)1000 * ax); Serial.print("mg"); Serial.print("\t");
      Serial.print("ay="); Serial.print((int)1000 * ay); Serial.print("mg"); Serial.print("\t");
      Serial.print("az="); Serial.print((int)1000 * az); Serial.print("mg"); Serial.print("\t");

      Serial.print("gx="); Serial.print( gx, 2); Serial.print("deg/s"); Serial.print("\t");
      Serial.print("gy="); Serial.print( gy, 2); Serial.print("deg/s"); Serial.print("\t");
      Serial.print("gz="); Serial.print( gz, 2); Serial.print("deg/s"); Serial.print("\t");

      //      Serial.print("mx="); Serial.print( mx, 2 ); Serial.print("mG"); Serial.print("\t");
      //      Serial.print("my="); Serial.print( my, 2 ); Serial.print("mG"); Serial.print("\t");
      //      Serial.print("mz="); Serial.print( mz, 2 ); Serial.print("mG"); Serial.print("\t");

      //      Serial.print("q0 = "); Serial.print(q[0]);
      //      Serial.print(" qx = "); Serial.print(q[1]);
      //      Serial.print(" qy = "); Serial.print(q[2]);
      //      Serial.print(" qz = "); Serial.println(q[3]);
    }

    // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
    // In this coordinate system, the positive z-axis is down toward Earth.
    // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
    // Tait-Bryan angles as well as Euler angles are non-commutative; that is, to get the correct orientation the rotations must be applied in the correct order which for this configuration is yaw, pitch, and then roll.
    // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.

    a12 =   2.0f * (q[1] * q[2] + q[0] * q[3]);
    a22 =   q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
    a31 =   2.0f * (q[0] * q[1] + q[2] * q[3]);
    a32 =   2.0f * (q[1] * q[3] - q[0] * q[2]);
    a33 =   q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];


    // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
    yaw   = atan2(a12, a22);
    yaw   *= 180.0f / PI;
    yaw   += 2.9; // Declination at Chiavari, Italy
    if (yaw < 0) yaw   += 360.0f; // Ensure yaw stays between 0 and 360

    // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
    pitch = -asin(a32);
    pitch *= 180.0f / PI;

    // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
    roll  = atan2(a31, a33);
    roll  *= 180.0f / PI;

    if (SerialDebug) {
      Serial.print("Yaw="); Serial.print(yaw, 2); Serial.print("\t");
      Serial.print("Pitch="); Serial.print(pitch, 2); Serial.print("\t");
      Serial.print("Roll="); Serial.print(roll, 2); Serial.println("\t");
     }

    count = millis();
    sumCount = 0;
    sum = 0;
  }
}
