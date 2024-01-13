#define BOTTOM_RIGHT_IR_SENSOR 19
#define BOTTOM_LEFT_IR_SENSOR 34
#define SPEED_OFFSET 3
//MOTOR SPEED
int MOTOR_SPEED = 255;

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

    pinMode(BOTTOM_RIGHT_IR_SENSOR, INPUT);
    pinMode(BOTTOM_LEFT_IR_SENSOR, INPUT);
    rotateMotor(0,0);   
}


void loop()
{

  delay(10);//sampling  time 
  int bottomrightIRSensorValue = digitalRead(BOTTOM_RIGHT_IR_SENSOR);
  int bottomleftIRSensorValue = digitalRead(BOTTOM_LEFT_IR_SENSOR);

  //If none of the sensors detects black line, then go straight
  if (bottomrightIRSensorValue == LOW && bottomleftIRSensorValue == LOW)
  {
    Serial.println("check1");
    rotateMotor(MOTOR_SPEED, MOTOR_SPEED-SPEED_OFFSET);
  }
  //If right sensor detects black line, then turn right
  else if (bottomrightIRSensorValue == HIGH && bottomleftIRSensorValue == LOW )
  {
      Serial.println("check2");
      rotateMotor(MOTOR_SPEED, -MOTOR_SPEED); 
  }
  //If left sensor detects black line, then turn left  
  else if (bottomrightIRSensorValue == LOW && bottomleftIRSensorValue == HIGH )
  {
      Serial.println("check1");
      rotateMotor(-MOTOR_SPEED, MOTOR_SPEED); 
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