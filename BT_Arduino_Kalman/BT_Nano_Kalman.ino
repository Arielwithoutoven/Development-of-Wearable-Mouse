#include <Wire.h>
#include <SoftwareSerial.h>
#include <Kalman.h>


Kalman kalmanX;
Kalman kalmanY;

double gyroX, gyroY, gyroZ;  // The angular acceleration on each axis (in the unit of the gyro).
int16_t tempRaw;

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

int vx, vy;
int buttonLeft = 2;  // D2引脚
int buttonRight = 3; // D3引脚
int buttonMouse = 4; // D4引脚
int stateLeft;  // 1 for press, 0 for release
int stateRight; // 1 for press, 0 for release
int stateMouse; // 1 for press, 0 for release
int mouseOn;    // 1 for on, 1 for off

SoftwareSerial BT_SEND(10, 11);

int get_key(int BUTTON);  // 短按返回1， 长按返回2，否则返回0

void setup() {
    mouseOn = true;
    BT_SEND.begin(9600);
    Serial.begin(115200);
    pinMode(buttonLeft, INPUT);
    pinMode(buttonRight, INPUT);
    pinMode(buttonMouse, INPUT);
    
    Wire.begin(8);

#if ARDUINO >= 157
    Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#else
    TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif

    i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
    i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
    i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
    i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
    
    while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
    while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

    while (i2cRead(0x75, i2cData, 1));
    if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
        Serial.print(F("Error reading sensor"));
        while (1);
    }
    delay(100); // Wait for sensor to stabilize
}

void loop() {
   /* Update all the values */
    while (i2cRead(0x3B, i2cData, 14));
    gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
    gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
    gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);

    double gyroXrate = gyroX / 131.0; // Convert to deg/s
    double gyroYrate = gyroY / 131.0; // Convert to deg/s
    double gyroZrate = gyroZ / 131.0; // Convert to deg/s

    stateLeft = get_key(buttonLeft);
    stateRight = get_key(buttonRight);
    stateMouse = get_key(buttonMouse);
    if (stateMouse == 1)
        mouseOn |= 1;

    int vx = gyroZrate / -5;
    int vy = gyroYrate / 6;
    
    BT_SEND.write(vx);
    BT_SEND.write(vy);
    BT_SEND.write(stateLeft*100 + stateRight*10 + mouseOn);
    
    BT_SEND.write(-1);
    delay(2);

#if 1
    Serial.print(mouseOn ? "Mouse is on" : "Mouse is off"); Serial.print("\t");
    Serial.print(gyroY / 131.0); Serial.print("\t");
    Serial.print(gyroZ / 131.0); Serial.print("\t");

    Serial.print("\t");
#endif

    Serial.print("\r\n");
    delay(2);

}


int get_key(int BUTTON)
{
    int tt = 0;
    if (digitalRead(BUTTON) == LOW) {
        delay(50);
        while (digitalRead(BUTTON) == LOW) tt++;
    }
    if(tt != 0 && tt < 30)   return 1;
    if(tt > 150)  return 2;
    return 0;
}
