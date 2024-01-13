#define IR_SENSOR_RIGHT 21
#define IR_SENSOR_LEFT 23
#define MOTOR_SPEED 255
#define RIGHT_SPEED_OFFSET 3
#define LEFT_SPEED_OFFSET 1

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

    pinMode(IR_SENSOR_RIGHT, INPUT);
    pinMode(IR_SENSOR_LEFT, INPUT);
    rotateMotor(0,0);   
}


void loop()
{

  int rightIRSensorValue = digitalRead(IR_SENSOR_RIGHT);
  int leftIRSensorValue = digitalRead(IR_SENSOR_LEFT);
  delay(10);

  //If none of the sensors detects black line, then go straight
  if (rightIRSensorValue == LOW && leftIRSensorValue == LOW)
  {
    Serial.println("check1");
    rotateMotor(MOTOR_SPEED-LEFT_SPEED_OFFSET, MOTOR_SPEED-RIGHT_SPEED_OFFSET);
  }
  //If right sensor detects black line, then turn right
  else if (rightIRSensorValue == HIGH && leftIRSensorValue == LOW )
  {
      Serial.println("check2");
      rotateMotor(-MOTOR_SPEED, MOTOR_SPEED); 
  }
  //If left sensor detects black line, then turn left  
  else if (rightIRSensorValue == LOW && leftIRSensorValue == HIGH )
  {
      Serial.println("check1");
      rotateMotor(MOTOR_SPEED, -MOTOR_SPEED); 
  } 
  //If both the sensors detect black line, then stop 
  else 
  {
    rotateMotor(0, 0);
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





