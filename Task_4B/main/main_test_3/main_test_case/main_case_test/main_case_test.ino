#include <Arduino.h>

#define IR_SENSOR_TOP_RIGHT 21
#define IR_SENSOR_TOP_LEFT 23
#define IR_SENSOR_BOTTOM_RIGHT 19
#define IR_SENSOR_BOTTOM_LEFT 34
#define IR_CENTRE 22
#define SPEED_OFFSET 0
#define MOTOR_SPEED 255

int rmf = 5;
int rmb = 15;
int lmf = 16;
int lmb = 4;
int enr = 27;
int enl = 26;

void setup()
{
    Serial.begin(115200);
    pinMode(rmf, OUTPUT);
    pinMode(rmb, OUTPUT);
    pinMode(lmf, OUTPUT);
    pinMode(lmb, OUTPUT);
    pinMode(enl, OUTPUT);
    pinMode(enr, OUTPUT);

    pinMode(IR_SENSOR_TOP_RIGHT, INPUT);
    pinMode(IR_SENSOR_TOP_LEFT, INPUT);
    pinMode(IR_SENSOR_BOTTOM_RIGHT, INPUT);
    pinMode(IR_SENSOR_BOTTOM_LEFT, INPUT);
    pinMode(IR_CENTRE, INPUT);
    rotateMotor(0, 0);
}

void loop()
{
    delay(10); //sampling time
    int toprightIRSensorValue = digitalRead(IR_SENSOR_TOP_RIGHT);
    int topleftIRSensorValue = digitalRead(IR_SENSOR_TOP_LEFT);
    int bottomrightIRSensorValue = digitalRead(IR_SENSOR_BOTTOM_RIGHT);
    int bottomleftIRSensorValue = digitalRead(IR_SENSOR_BOTTOM_LEFT);
    int centre_ir_value = digitalRead(IR_CENTRE);

    int conditionValue = 0;
    conditionValue |= (toprightIRSensorValue == HIGH && topleftIRSensorValue == LOW) ? 0b010000 : 0;
    conditionValue |= (bottomrightIRSensorValue == LOW && bottomleftIRSensorValue == HIGH) ? 0b100001 : 0;
    conditionValue |= (bottomrightIRSensorValue == HIGH && bottomleftIRSensorValue == LOW) ? 0b001100 : 0;
    conditionValue |= (toprightIRSensorValue == LOW && topleftIRSensorValue == HIGH) ? 0b000000 : 0;
    conditionValue |= (toprightIRSensorValue == LOW && topleftIRSensorValue == LOW) ? 0b000000 : 0;
    conditionValue |= (bottomrightIRSensorValue == LOW && bottomleftIRSensorValue == LOW) ? 0b000000 : 0;

    Serial.print("Condition Value: ");
    Serial.println(conditionValue, BIN);

    switch (conditionValue)
    {
    case 0b010000:
        Serial.println("check2");
        rotateMotor(-MOTOR_SPEED, MOTOR_SPEED);
        // delay(500);//sampling time
        break;

    case 0b100001:
        Serial.println("check1");
        rotateMotor(MOTOR_SPEED, -MOTOR_SPEED);
        // delay(500);//sampling time
        break;

    case 0b001100:
        rotateMotor(MOTOR_SPEED, MOTOR_SPEED - SPEED_OFFSET);
        // delay(500);//sampling time
        break;

    case 0b000000:
        rotateMotor(0, 0);
        break;

    default:
        // Handle unexpected condition
        break;
    }
}

void rotateMotor(int right, int left)
{
    Serial.print("Right: ");
    Serial.println(right);
    Serial.print("left: ");
    Serial.println(left);
    if (right > 0)
    {
        analogWrite(enr, right);
        digitalWrite(rmf, HIGH);
        digitalWrite(rmb, 0);
    }
    else
    {
        analogWrite(enr, abs(right));
        digitalWrite(rmf, LOW);
        digitalWrite(rmb, HIGH);
    }

    if (left > 0)
    {
        analogWrite(enl, left);
        digitalWrite(lmf, HIGH);
        digitalWrite(lmb, LOW);
    }
    else
    {
        analogWrite(enl, abs(left));
        digitalWrite(lmf, LOW);
        digitalWrite(lmb, HIGH);
    }
}
