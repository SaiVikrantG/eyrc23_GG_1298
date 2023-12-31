#define right_delay = 1000;
#define left_delay = 1000;
#define straight_delay = 1000;
int count = 0;

void turn_right();
void turn_left();
void straight();
void stop();

void setup() {
  Serial.begin(9600);
  
}

void loop() {
  switch(count){
    case 1: 
      straight(); 
      break;
    
    case 2: 
      straight(); 
      break;            

    case 3: turn_right(); break;

    case 4: turn_left(); break;

    case 5: turn_right(); break;

    case 6: turn_right(); break;

    case 7: straight(); break;

    case 8: turn_right(); break;

    case 9: straight(); break;

    case 10: turn_left(); break;

    case 11: stop(); break;
  }
  count++;
  delay(1000);
}

void turn_left() {
  Serial.println("right");
    analogWrite(enl, 255);
    digitalWrite(lmf, HIGH);
    digitalWrite(lmb, LOW);
    delay(left_delay);
    analogWrite(enl, 0);
    digitalWrite(lmf, LOW);
    digitalWrite(lmb, LOW);

}

void turn_right() {
    Serial.println("left");
    analogWrite(enr, 255);
    digitalWrite(rmf, HIGH);
    digitalWrite(rmb, LOW);
    delay(right_delay);
    analogWrite(enr, 0);
    digitalWrite(rmf, LOW);
    digitalWrite(rmb, LOW);
}

void straight() {
    Serial.println("straight");
    analogWrite(enr, 255);
    digitalWrite(rmf, HIGH);
    digitalWrite(rmb, LOW);
    digitalWrite(lmf, HIGH);
    digitalWrite(lmb, LOW);
    delay(straight_delay);
    analogWrite(enr, 0);
    digitalWrite(rmf, LOW);
    digitalWrite(rmb, LOW);
    digitalWrite(lmf, LOW);
    digitalWrite(lmb, LOW);
}

void stop(){
  Serial.println("stop");
  analogWrite(enr, 0);
  analogWrite(enl, 0);
  digitalWrite(rmf, LOW);
  digitalWrite(rmb, LOW);
  digitalWrite(lmf, LOW);
  digitalWrite(lmb, LOW);
  
}