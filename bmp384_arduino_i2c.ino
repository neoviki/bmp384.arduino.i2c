#include <Wire.h>
#include "SparkFunBMP384.h"
#define PIN_WIRE_SDA         (20u)
#define PIN_WIRE_SCL         (21u)

BMP384 pressureSensor;


//BMP384_I2C_ADDRESS_DEFAULT; // 0x77

//IC1
//BMP384_I2C_ADDRESS_SECONDARY; // 0x76

void setup()
{
    Serial.begin(115200);
    Serial.println("BMP384 Example1 begin!");


    pinMode(21, OUTPUT);  
    
    for (int i = 0; i < 8; i++) {    
      digitalWrite(21, HIGH);    
      delayMicroseconds(3);    
      digitalWrite(21, LOW);    
      delayMicroseconds(3);  
    } 
    
    pinMode(21, INPUT);
}

int init_bmp1()
{
    int i;
    Wire.end();
    delay(300);
    Wire.begin();

    for(i=0;i<5;i++){
        if(pressureSensor.beginI2C(BMP384_I2C_ADDRESS_DEFAULT) != BMP3_OK){
            Serial.println("Error: BMP384(1) not connected, check wiring and I2C address!");
        }else{
            Serial.println("BMP384(1) connected!");
            return 1;
        }
        delay(1000);
    }

    return 0;
}

int init_bmp2()
{
    int i;
    Wire.end();
    delay(300);
    Wire.begin();

    for(i=0;i<5;i++){
        if(pressureSensor.beginI2C(BMP384_I2C_ADDRESS_SECONDARY) != BMP3_OK){
            Serial.println("Error: BMP384(2) not connected, check wiring and I2C address!");
        }else{
            Serial.println("BMP384(2) connected!");
            return 1;
        }
        delay(1000);
    }

    return 0;
}

void loop()
{

    bmp3_data data;
    static int flag = 0;
    int ret = 0;

    if(flag)
    {   
      ret = init_bmp2();
      flag=0;
    }else{
      ret = init_bmp1();
      flag=1;      
    }

    if(ret == 0) return;

    delay(1000);

    int8_t err = pressureSensor.getSensorData(&data);

    if(err == BMP3_OK)
    {
        Serial.print("Temperature (C): ");
        Serial.print(data.temperature);
        Serial.print("\t\t");
        Serial.print("Pressure (Pa): ");
        Serial.println(data.pressure);
    }
    else
    {
        Serial.print("Error getting data from sensor! Error code: ");
        Serial.println(err);
    }

    delay(1000);
}
