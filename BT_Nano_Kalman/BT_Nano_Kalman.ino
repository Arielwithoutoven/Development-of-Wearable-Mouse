/*
 * 开发板：Arduino Nano, 处理数据，仅向蓝牙发送位移信息。鼠标状态。
 * 
 * 2021/10/3 --------- 优化鼠标状态检定（支持长按和短按）
 * 
 * 2021/10/5 --------- 优化双击事件
 */


#include <Wire.h>
#include <SoftwareSerial.h>
#include <Kalman.h>

#define buttonLeft 2  // D2引脚
#define buttonRight 3 // D3引脚
#define buttonMouse 4 // D4引脚

Kalman kalmanX;
Kalman kalmanY;

double gyroX, gyroY, gyroZ;  // The angular acceleration on each axis (in the unit of the gyro).
uint8_t i2cData[14]; // Buffer for I2C data
static double statimer = 0;
int vx, vy;
int Start = -1;
int End = -2;
int stateLeft = 0;  // 1 for press, 0 for release, 2 for long release
int stateRight = 0; // 1 for press, 0 for release
int stateMouse = 0; // 1 for press, 0 for release
int mouseOn;    // 1 for on, 0 for off

SoftwareSerial BT_SEND(10, 11);

int get_key(int BUTTON, int state = 0);  // 短按返回1，双击返回2，否则返回0
int get_leftkey(int state = 0);  // 按下返回1，单击返回2，双击返回3，长按返回4，否则返回0

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
    gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
    gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);

    double gyroYrate = gyroY / 131.0; // Convert to deg/s
    double gyroZrate = gyroZ / 131.0; // Convert to deg/s

    if (mouseOn) {
        stateLeft = get_leftkey(stateLeft);
        stateRight = get_key(buttonRight);
    }
    stateMouse = get_key(buttonMouse);
    if (stateMouse == 1)
        mouseOn ^= 1;

    int vx = (int)(gyroZrate / -1.5);
    int vy = (int)(gyroYrate / 2.5);

    BT_SEND.write(-2);
    delay(2);
    BT_SEND.write(vx);
    delay(2);
    BT_SEND.write(vy);
    delay(2);
    BT_SEND.write(stateLeft);
    delay(2);
    BT_SEND.write(stateRight);
    delay(2);
    BT_SEND.write(mouseOn);
    delay(2);
    BT_SEND.write(-1);
    delay(2);

#if 1
    Serial.print(mouseOn ? "Mouse is on" : "Mouse is off"); Serial.print("\t");
    Serial.print(gyroY / 131.0); Serial.print("\t");
    Serial.print(gyroZ / 131.0); Serial.print("\t");
    Serial.print(stateLeft); Serial.print("\t");
    Serial.print(stateRight); Serial.print("\t");
    Serial.print(stateMouse); Serial.print("\t");
    Serial.print(mouseOn);Serial.print("\t");

    Serial.print("\t");
#endif

    Serial.print("\r\n");
    delay(2);

}

int get_key(int BUTTON, int state = 0) {
    int tt = 0;
    if (digitalRead(BUTTON) == LOW) {
        if (state == 2) return 2;
        delay(50);
        while (digitalRead(BUTTON) == LOW) {
            tt++;
            delay(5);
            if (tt > 50) return 2;
        }
    }
    if (tt != 0 && tt < 30) return 1;
    return 0;
}

int get_leftkey(int state = 0)
{
    
    int tt = 0;
    if (digitalRead(buttonLeft) == LOW) {
        if (state == 4) return 4;
        delay(50);
        while (digitalRead(buttonLeft) == LOW) {
            tt++;
            delay(5);
            if (tt > 50) return 4;
        }
    }

    if (tt == 0) {
        double temtimer = micros();
//        Serial.print(temtimer); Serial.print("\t");
        double dtime = (double)(micros() - statimer) / 1000000;
        Serial.print("距上次时间... "); Serial.print(dtime); Serial.print("\t");
        if (state == 1) {
            if (dtime > 0.20) return 2;
            else return 1;
        }
        return 0;
    }
    if (tt != 0 && tt < 30) {
        double dtime = (double)(micros() - statimer) / 1000000;
        Serial.print("间隔时间. . "); Serial.print(dtime); Serial.print("\t");
        statimer = micros();
        Serial.print(statimer); Serial.print("\t");
        if (dtime < 0.20) return 3;
        else return 1;
    }
    
}
