#define IR_SENSOR_TOP_RIGHT 21
#define IR_SENSOR_TOP_LEFT 23
#define IR_SENSOR_BOTTOM_RIGHT 34
#define IR_SENSOR_BOTTOM_LEFT 19
#define IR_CENTRE 22
#define MOTOR_SPEED 255
#define right_delay 1500
#define left_delay 1500


int rmf = 5;
int rmb = 15;
int lmf = 16;
int lmb = 4;
int enr = 27;
int enl = 26;
int count =0;
int topRightIRSensorValue, topLeftIRSensorValue;
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
     topRightIRSensorValue = digitalRead(IR_SENSOR_TOP_RIGHT);
     topLeftIRSensorValue = digitalRead(IR_SENSOR_TOP_LEFT);

    if(!topRightIRSensorValue && !topLeftIRSensorValue){
    stop();
    delay(2000);
    if(count <= 11){
        count += 1;
        switch (count){
            case 1:
            case 2:
            case 7:
            case 9:
                forward_single_l();
                break;

            case 3:
            case 5:
            case 6:
            case 8:
                turnRight();
                break;

            case 4:
            case 10:
                turnLeft();
                break;

            case 11:
                stop();
                break;

            default: 
                break;
        }
    }
    else{
        count = 0;
    }
    } 
}
void forward(){
  rotateMotor(MOTOR_SPEED,MOTOR_SPEED-3);
}
void forward_single_l(){
//   int rightIRSensorValue = digitalRead(IR_SENSOR_RIGHT);
//   int leftIRSensorValue = digitalRead(IR_SENSOR_LEFT);
  delay(10);

  //If none of the sensors detects black line, then go straight
  if (topRightIRSensorValue == LOW && topLeftIRSensorValue == LOW)
  {
    Serial.println("check1");
    rotateMotor(MOTOR_SPEED, MOTOR_SPEED);
  }
  //If right sensor detects black line, then turn right
  else if (topRightIRSensorValue == HIGH && topLeftIRSensorValue == LOW )
  {
      Serial.println("check2");
      rotateMotor(-MOTOR_SPEED, MOTOR_SPEED); 
  }
  //If left sensor detects black line, then turn left  
  else if (topLeftIRSensorValue == LOW && topLeftIRSensorValue == HIGH )
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
void stop()
{
  analogWrite(enr, 0);
  analogWrite(enl, 0);
  digitalWrite(rmf, LOW);
  digitalWrite(rmb, LOW);
  digitalWrite(lmf, LOW);
  digitalWrite(lmb, LOW);
}
void turnLeft() {
    analogWrite(enl, 255);
    analogWrite(enr, 250);
    digitalWrite(lmf, LOW);
    digitalWrite(lmb, LOW);
    digitalWrite(rmf, HIGH);
    digitalWrite(rmb, LOW );
    delay(left_delay);
    stop();
}

void turnRight() {    
    analogWrite(enr, 255);
    analogWrite(enl, 250);
    digitalWrite(rmf, LOW);
    digitalWrite(rmb, LOW);
    digitalWrite(lmf, HIGH);
    digitalWrite(lmb, LOW);
    delay(right_delay);
    stop();
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