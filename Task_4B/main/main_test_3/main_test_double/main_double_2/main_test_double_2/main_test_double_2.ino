#define IR_SENSOR_TOP_RIGHT 21
#define IR_SENSOR_TOP_LEFT 23
#define IR_SENSOR_BOTTOM_RIGHT 19
#define IR_SENSOR_BOTTOM_LEFT 34
#define IR_CENTRE 22
#define SPEED_OFFSET 0
#define MOTOR_SPEED  255
//MOTOR SPEED

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
    rotateMotor(0,0);   
}


void loop()
{

  delay(10);//sampling  time 
  int toprightIRSensorValue = digitalRead(IR_SENSOR_TOP_RIGHT);
  int topleftIRSensorValue = digitalRead(IR_SENSOR_TOP_LEFT);
  int bottomrightIRSensorValue = digitalRead(IR_SENSOR_BOTTOM_RIGHT);
  int bottomleftIRSensorValue = digitalRead(IR_SENSOR_BOTTOM_LEFT);
  int centre_ir_value = digitalRead(IR_CENTRE);

    //If none of the sensors detects black line, then go straight
  // if (bottomrightIRSensorValue == LOW && bottomleftIRSensorValue == LOW)
  // {
  //   Serial.println("check1");
  //   rotateMotor(MOTOR_SPEED, MOTOR_SPEED-SPEED_OFFSET);
  // }
  //If right sensor detects black line, then turn right
  // else if (bottomrightIRSensorValue == HIGH && bottomleftIRSensorValue == LOW )
  // {
  //     Serial.println("check2");
  //     rotateMotor(MOTOR_SPEED, -MOTOR_SPEED); 
  // }
  // //If left sensor detects black line, then turn left  
  // else if (bottomrightIRSensorValue == LOW && bottomleftIRSensorValue == HIGH )
  // {
  //     Serial.println("check1");
  //     rotateMotor(-MOTOR_SPEED, MOTOR_SPEED); 
  // } 
  //If none of the sensors detects black line, then go straight

  //If right sensor detects black line, then turn right
  if ((toprightIRSensorValue == HIGH && topleftIRSensorValue )== LOW ||(bottomrightIRSensorValue == LOW && bottomleftIRSensorValue == HIGH) )
  {
      Serial.println("check2");
      rotateMotor(-MOTOR_SPEED, MOTOR_SPEED); 
      delay(500);//sampling  time 
  }
  //If left sensor detects black line, then turn left  
  else if ((bottomrightIRSensorValue == HIGH && bottomleftIRSensorValue == LOW) ||(toprightIRSensorValue == LOW && topleftIRSensorValue == HIGH) )
  {
      Serial.println("check1");
      rotateMotor(MOTOR_SPEED,- MOTOR_SPEED);
      delay(500);//sampling  time  
  } 
  //If both the sensors detect black line, then stop 
  else if ((toprightIRSensorValue == LOW && topleftIRSensorValue == LOW) && (bottomrightIRSensorValue == LOW && bottomleftIRSensorValue == LOW))
  {
    rotateMotor(MOTOR_SPEED, MOTOR_SPEED-SPEED_OFFSET);
    delay(500);//sampling  time
  } 
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