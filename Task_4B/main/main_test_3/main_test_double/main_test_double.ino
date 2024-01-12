#define IR_SENSOR_TOP_RIGHT 21
#define IR_SENSOR_TOP_LEFT 23
#define IR_SENSOR_BOTTOM_RIGHT 34
#define IR_SENSOR_BOTTOM_LEFT 19
#define IR_CENTRE 22
#define MOTOR_SPEED 255

int rmf = 5;
int rmb = 15;
int lmf = 16;
int lmb = 4;
int enr = 27;
int enl = 26;
int topRightIRSensorValue, topLeftIRSensorValue, bottomRightIRSensorValue, bottomLeftIRSensorValue, centerIRSensorValue;

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
    rotateMotor(0,0);   
}


void loop()
{
    delay(10);
    topRightIRSensorValue = !digitalRead(IR_SENSOR_TOP_RIGHT);
    topLeftIRSensorValue = !digitalRead(IR_SENSOR_TOP_LEFT);
    bottomRightIRSensorValue = !digitalRead(IR_SENSOR_BOTTOM_RIGHT);
    bottomLeftIRSensorValue = !digitalRead(IR_SENSOR_BOTTOM_LEFT);
    centerIRSensorValue = !digitalRead(IR_CENTRE);
    delay(10);

    if(topLeftIRSensorValue == HIGH && topRightIRSensorValue == HIGH  && bottomLeftIRSensorValue == LOW && bottomLeftIRSensorValue == LOW )
            rotateMotor(MOTOR_SPEED,MOTOR_SPEED);
    if(topLeftIRSensorValue == HIGH && topRightIRSensorValue == HIGH  && bottomLeftIRSensorValue == HIGH && bottomLeftIRSensorValue == HIGH )
            rotateMotor(MOTOR_SPEED,MOTOR_SPEED);
    else if(topLeftIRSensorValue == HIGH && topRightIRSensorValue == LOW && bottomLeftIRSensorValue == HIGH && bottomLeftIRSensorValue == HIGH )
            rotateMotor(MOTOR_SPEED,-MOTOR_SPEED);
    else if(topLeftIRSensorValue == LOW && topRightIRSensorValue == HIGH && bottomLeftIRSensorValue == HIGH && bottomLeftIRSensorValue == HIGH )
            rotateMotor(-MOTOR_SPEED,MOTOR_SPEED);
    else 
            rotateMotor(MOTOR_SPEED,MOTOR_SPEED);
    
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