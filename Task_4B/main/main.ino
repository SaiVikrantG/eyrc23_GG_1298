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
/*
 RR: rotateMotor(-MOTOR_SPEED,MOTOR_SPEED);
 RL: rotateMotor(MOTOR_SPEED,-MOTOR_SPEED);
 STOP: rotateMOtor(0,0);
*/

void loop()
{

  delay(10);//sampling  time 
  int toprightIRSensorValue =! digitalRead(IR_SENSOR_TOP_RIGHT);
  int topleftIRSensorValue = !digitalRead(IR_SENSOR_TOP_LEFT);
  int bottomrightIRSensorValue =! digitalRead(IR_SENSOR_BOTTOM_RIGHT);
  int bottomleftIRSensorValue = !digitalRead(IR_SENSOR_BOTTOM_LEFT);
  int centre_ir_value = !digitalRead(IR_CENTRE);
  int combinedValue = (bottomleftIRSensorValue << 3) | (topleftIRSensorValue << 2) | (toprightIRSensorValue << 1) | bottomrightIRSensorValue;
  Serial.println(combinedValue);
  switch(combinedValue){
    case 5:
    case 7:
    case 13:
            //rotate right
            rotateMotor(-MOTOR_SPEED,MOTOR_SPEED);
            break;
    case 10:
    case 11:
    case 14:
            rotateMotor(MOTOR_SPEED,-MOTOR_SPEED);
            break;
    case 6:
    case 15:
            rotateMotor(MOTOR_SPEED, MOTOR_SPEED);
            break;
    // case 0:
    // case 1:
    // case 2:
    // case 3:
    // case 4:
    // case 8:
    // case 9:
    // case 12:
    default:
            rotateMotor(0, 0);
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