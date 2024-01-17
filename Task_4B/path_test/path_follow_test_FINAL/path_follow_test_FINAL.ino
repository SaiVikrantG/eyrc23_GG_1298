#define LED_PIN 32
#define BUZZER_PIN 33
#define IR_SENSOR_TOP_RIGHT 21
#define IR_SENSOR_TOP_LEFT 23
#define IR_SENSOR_BOTTOM_RIGHT 19
#define IR_SENSOR_BOTTOM_LEFT 34
#define IR_CENTRE 22
#define SPEED_OFFSET 0
#define MOTOR_SPEED  255
//MOTOR delay
#define delay_right 500
#define delay_left 500
#define delay_forward 400
#define delay_correction 100

int rmf = 5;
int rmb = 15;
int lmf = 16;
int lmb = 4;
int enr = 27;
int enl = 26; 
//counter variable for blob
int count =0;

void setup()
{ 
    Serial.begin(115200);
    pinMode(rmf, OUTPUT);
    pinMode(rmb, OUTPUT);
    pinMode(lmf, OUTPUT);
    pinMode(lmb, OUTPUT);
    pinMode(enl, OUTPUT);
    pinMode(enr, OUTPUT);
    pinMode(LED_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);

    pinMode(IR_SENSOR_TOP_RIGHT, INPUT);
    pinMode(IR_SENSOR_TOP_LEFT, INPUT);
    pinMode(IR_SENSOR_BOTTOM_RIGHT, INPUT);
    pinMode(IR_SENSOR_BOTTOM_LEFT, INPUT);
    pinMode(IR_CENTRE, INPUT);
    digitalWrite(LED_PIN,HIGH);
    digitalWrite(BUZZER_PIN,HIGH);
    //make the LED and BUZZER ON FOR 1 SEC
    digitalWrite(LED_PIN,LOW);
    digitalWrite(BUZZER_PIN,LOW);
    delay(1000);
    digitalWrite(LED_PIN,HIGH);
    digitalWrite(BUZZER_PIN,HIGH);
    rotateMotor(0,0);   
}
/*
 RR: rotateMotor(-MOTOR_SPEED,MOTOR_SPEED);
 RL: rotateMotor(MOTOR_SPEED,-MOTOR_SPEED);
 STOP: rotateMOtor(0,0);
*/

void loop()
{
  // Serial.print("Count: ");
  // Serial.println(count);

  delay(10);//sampling  time 
  int toprightIRSensorValue =! digitalRead(IR_SENSOR_TOP_RIGHT);
  int topleftIRSensorValue = !digitalRead(IR_SENSOR_TOP_LEFT);
  int bottomrightIRSensorValue =! digitalRead(IR_SENSOR_BOTTOM_RIGHT);
  int bottomleftIRSensorValue = !digitalRead(IR_SENSOR_BOTTOM_LEFT);
  int centre_ir_value = !digitalRead(IR_CENTRE);
  int combinedValue = (bottomleftIRSensorValue << 3) | (topleftIRSensorValue << 2) | (toprightIRSensorValue << 1) | bottomrightIRSensorValue;
  // Serial.println(combinedValue);
//for line following
  switch(combinedValue){
    //to predict where to rotate the bot right
    case 5:
    case 7:
    case 13:
            rotateMotor(-MOTOR_SPEED,MOTOR_SPEED);
            break;
             //to predict where to rotate the bot left
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
            //blob check
            // if(toprightIRSensorValue==High && topleftIRSensorValue==High){
              Serial.println("check");
              // delay(2000);
              if(count <= 11){
                  count += 1;
                  Serial.print("Count: ");
                  Serial.println(count);
                  switch (count){
                      case 1:
                      case 2:
                      case 9:
                          // digitalWrite(LED_PIN,LOW);
                          digitalWrite(BUZZER_PIN,LOW);
                          delay(1000);
                          // digitalWrite(LED_PIN,HIGH);
                          digitalWrite(BUZZER_PIN,HIGH);
                          rotateMotor(MOTOR_SPEED,MOTOR_SPEED);
                          delay(delay_forward);
                          break;
                      case 7:
                          // digitalWrite(LED_PIN,LOW);
                          digitalWrite(BUZZER_PIN,LOW);
                          delay(1000);
                          // digitalWrite(LED_PIN,HIGH);
                          digitalWrite(BUZZER_PIN,HIGH);

                          rotateMotor(-MOTOR_SPEED,MOTOR_SPEED);
                          delay(50);
                          rotateMotor(MOTOR_SPEED,MOTOR_SPEED);
                          delay(200);
                          break;

                      case 3:
                      case 5:
                      case 6:
                      case 8:
                          //Turn Right
                          // digitalWrite(LED_PIN,LOW);
                          digitalWrite(BUZZER_PIN,LOW);
                          delay(1000);
                          // digitalWrite(LED_PIN,HIGH);
                          digitalWrite(BUZZER_PIN,HIGH);

                          rotateMotor(MOTOR_SPEED,0);
                          delay(delay_correction); 
                          rotateMotor(MOTOR_SPEED,MOTOR_SPEED);
                          delay(delay_forward);
                          rotateMotor(MOTOR_SPEED,-MOTOR_SPEED);
                          delay(delay_right);
                          Serial.println("Right Done");
                          
                          break;

                      case 4:
                          // digitalWrite(LED_PIN,LOW);
                          digitalWrite(BUZZER_PIN,LOW);
                          delay(1000);
                          // digitalWrite(LED_PIN,HIGH);
                          digitalWrite(BUZZER_PIN,HIGH);

                          rotateMotor(MOTOR_SPEED,0);
                          delay(delay_correction); 
                          rotateMotor(MOTOR_SPEED,MOTOR_SPEED);
                          delay(delay_forward);
                          rotateMotor(-MOTOR_SPEED,MOTOR_SPEED);
                          delay(delay_left);
                          rotateMotor(MOTOR_SPEED, MOTOR_SPEED-4);
                          delay(700);
                          break;

                      case 10:
                          //Turn Left
                          // digitalWrite(LED_PIN,LOW);
                          digitalWrite(BUZZER_PIN,LOW);
                          delay(1000);
                          // digitalWrite(LED_PIN,HIGH);
                          digitalWrite(BUZZER_PIN,HIGH);

                          rotateMotor(MOTOR_SPEED,0);
                          delay(delay_correction); 
                          rotateMotor(MOTOR_SPEED,MOTOR_SPEED);
                          delay(delay_forward);
                          rotateMotor(-MOTOR_SPEED,MOTOR_SPEED);
                          delay(delay_left);
                          break;
                          Serial.println("left done");

                      case 11:
                          rotateMotor(0, 0);
                          digitalWrite(LED_PIN,LOW);
                          digitalWrite(BUZZER_PIN,LOW);
                          delay(5000);
                          digitalWrite(LED_PIN,HIGH);
                          digitalWrite(BUZZER_PIN,HIGH);
                          break;

                      default: 
                          break;
                  }
              } 
                    break;
      
    

  }


}


void rotateMotor(int right, int left)
    {
      // Serial.print("Right: ");
      // Serial.println(right);
      // Serial.print("left: ");
      // Serial.println(left);
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