/*
 * 蓝牙从机-Leonardo, 接收原始数据
 * 
 * 2021/7/2 ---------- 仅接收两个数据，控制鼠标的移动
 */

#include <Mouse.h>
#include <SoftwareSerial.h>

SoftwareSerial BT_RCV(10, 11);

int vx, vy;
int stateLeft, stateRight;
int mouseOn;
int mouseOn2;
int endint;

void dealMouse(int mouse, int state);


void setup() {
    Serial.begin(115200); 
    BT_RCV.begin(9600);
    Mouse.begin();
    mouseOn = 1;
}


void loop() {
    vx = 0;
    vy = 0;
    if (BT_RCV.available()) {
        vx = BT_RCV.read();
        Serial.print(vx);
        Serial.print('\t');
        delay(2);

        vy = BT_RCV.read();
        Serial.print(vy);
        Serial.print('\t');
        delay(2);

        stateLeft = BT_RCV.read();
        Serial.print(stateLeft);
        Serial.print('\t');
        delay(2);

        stateRight = BT_RCV.read();
        Serial.print(stateRight);
        Serial.print('\t');
        delay(2);

        mouseOn2 = BT_RCV.read();
        mouseOn |= mouseOn2;
        Serial.print(mouseOn);
        Serial.print('\t');
        delay(2);
        
        endint = BT_RCV.read();
        Serial.print(endint);
        Serial.print('\t');
        delay(2);
        
        if (endint != 255) {
            while (BT_RCV.available()) {
                endint = BT_RCV.read();
                if (endint == 255)
                    break;
            }
        }
    }
    
    if (mouseOn){
        Mouse.move(vx, vy);
        dealMouse(1, stateLeft);
        dealMouse(2, stateRight);
    }
    
    Serial.print("\r\n");
}


void dealMouse(int mouse, int state) {
    switch (state) {
        case -2: Mouse.click(mouse); break;
        case -3: Mouse.press(mouse); break;
        case 0: if (Mouse.isPressed(mouse)) Mouse.release(mouse);
        default: break;
    }
}
