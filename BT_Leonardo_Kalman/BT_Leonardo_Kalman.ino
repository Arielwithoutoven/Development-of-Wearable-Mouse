/*
 * 开发板：Arduino Leonardo,
 * 2021/7/2 ---------- 仅接收两个数据，控制鼠标的移动
 * 
 * 2021/10/3 --------- 优化数据处理函数
 * 
 * 2021/10/5 --------- 特殊处理双击事件
 */

#include <Mouse.h>
#include <SoftwareSerial.h>
#define dataNum 5

SoftwareSerial BT_RCV(10, 11);
int Data[dataNum + 2];  // {Start, vx, vy, stateLeft, stateRight, mouseOn, End}

int vx, vy;
int stateLeft, stateRight, mouseOn;
int mouseState;

void dealMouse(int mouse, int state);
void dealLeftMouse(int state);

void setup() {
    Serial.begin(9600); 
    BT_RCV.begin(9600);
    Mouse.begin();
    mouseState = 1;
}


void loop() {
    while (BT_RCV.available()) {
        int x = 1;
        
        Data[0] = BT_RCV.read();
        while (Data[0] != 254) {
            Data[0] = BT_RCV.read();
        } Serial.print(Data[0]); Serial.print('\t');
        
        while (x <= dataNum) {
            Data[x] = BT_RCV.read();
            if (Data[x] == -1)
                continue;
            Serial.print(Data[x]); Serial.print('\t');
            x++;
        }
        
        Data[dataNum + 1] = BT_RCV.read();
        while (Data[dataNum + 1] != 255) {
            Data[dataNum + 1] = BT_RCV.read();
        } Serial.print(Data[dataNum + 1]); Serial.print('\t');
        
        if (BT_RCV.overflow()) {
        Serial.println("SoftwareSerial overflow!");
        }


        if (Data[dataNum] == 1){
            Mouse.move(Data[1], Data[2]);
            dealLeftMouse(Data[3]);
            dealMouse(2, Data[4]);
        }
        Serial.print("\r\n");
    }
    
}


void dealMouse(int mouse, int state) {
    switch (state) {
        case 1: Mouse.click(mouse); break;
        case 2: if (!Mouse.isPressed(mouse)) Mouse.press(mouse); break;
        case 0: if (Mouse.isPressed(mouse)) Mouse.release(mouse);
        default: break;
    }
}

void dealLeftMouse(int state) {
    switch (state) {
        case 2: Mouse.click(); break;
        case 3: {
            Mouse.click();
            delay(2);
            Mouse.click(); break;
        }
        case 4: if (!Mouse.isPressed()) Mouse.press(); break;
        case 0: if (Mouse.isPressed()) Mouse.release();
        default: break;
    }
}
