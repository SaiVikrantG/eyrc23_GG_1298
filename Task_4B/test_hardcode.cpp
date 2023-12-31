#define read_offset 1
#define attach_offset 21

#define right_delay 0
#define left_delay 0

// blob count
int count = 0;

//ir pins
int front_left = 23;
int front_right = 21;
int back_left = 32;
int back_mid = 22;
int back_right = 19;

//motor driver pins
int rmf = 5;
int rmb = 15;
int lmf = 16;
int lmb = 4;
int enr = 27;
int enl = 26;

// speeds
int rspeed;
int lspeed;
const int base_speed = 255;

/// for pid
int j=0;
int pos;
long sensor_average;
int sensor_sum;

int button = 3; // to be pressed to find set point

float p;
float i;
float d;
float lp;
float error;
float correction;
float sp=0.866;

float kp = 0.0;
float ki = 0.0;
float kd = 0.0;

void setup()
{
  Serial.begin(115200);
  // sensors
  pinMode(front_left, INPUT);
  pinMode(front_right, INPUT);
  pinMode(back_left, INPUT);
  pinMode(back_mid, INPUT);
  pinMode(back_right, INPUT);

  // motors
  pinMode(rmf, OUTPUT);
  pinMode(rmb, OUTPUT);
  pinMode(lmf, OUTPUT);
  pinMode(lmb, OUTPUT);

  // pid correction
  while(1){
    pid_calc();
    calc_turn();
  }

}

void loop()
{
  // if (startSignal)
  // {
  //   pid_calc();
  //   calc_turn();
  // }

  // if (man_signal){
  //   manual_move(command);
  // }

  // {
  //   stop();
  // }
  int ir_0 = digitalRead(front_left);
  int ir_1 = digitalRead(front_right);
  if(!ir_0 && !ir_1){
    count++;
    handleControl(count);
    delay(1000);
  }
  else{
    moveForward();
  }
}

void pid_calc()
{
  sensor_average = 0;
  sensor_sum = 0;
  // int j = 0;

  // for (int i = -1; i <= 1; i++)
  // {
  //   sensor[i + read_offset] = !digitalRead(i + attach_offset);
  //   sensor_average = sensor[i + read_offset] * i * 15;
  //   sensor_sum += sensor[i + read_offset];
  // }
  // sensor_average += (-2) * (!digitalRead(32)) + !digitalRead(1) * (2);
  
  for (int i = 1; i <= 3; i++)
  {
    sensor[i] = !digitalRead(i+attach_offset-1);
    sensor_average = sensor[i] * i * 15;
    sensor_sum += sensor[i];
  }
  sensor_average += (4) * (!digitalRead(32)) + !digitalRead(1) * (5);
  sensor_sum += !digitalRead(32) + !digitalRead(19);
  if (sensor_sum)
    pos = int(sensor_average / sensor_sum);
  else
    stop();
  error = pos - sp;
  p = error;
  j += p;
  d = p - lp;
  Serial.print("ki: ");
  Serial.println(ki);
  Serial.print("kp: ");
  Serial.println(kp);
  Serial.print("kd: ");
  Serial.println(kd);
  lp = p;
  correction = int(kp * p + ki * j + kd * d);
}

void calc_turn()
{
  rspeed = base_speed + correction;
  lspeed = base_speed - correction;

  if (rspeed > 255)
    rspeed = 255;

  if (lspeed > 255)
    lspeed = 255;

  if (rspeed < -255)
    rspeed = -255;

  if (lspeed < -255)
    lspeed = -255;

  motor_drive(rspeed, lspeed);
}

void motor_drive(int right, int left)
{
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

void handleControl(int count) {
  switch (count) {
    case 1:
    case 2:
    case 7:
    case 9:
      moveForward();
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
      moveForward();
      break;
  }
}

void turnLeft() {
  Serial.println("left");
    analogWrite(enl, 255);
    analogWrite(enr, 255);
    digitalWrite(lmf, LOW);
    digitalWrite(lmb, LOW);
    digitalWrite(rmf, HIGH);
    digitalWrite(rmb, LOW );
    delay(left_delay);
    // analogWrite(enl, 0);
    // digitalWrite(lmf, LOW);
    // digitalWrite(lmb, LOW);
}

void turnRight() {
    Serial.println("right");
    analogWrite(enr, 255);
    analogWrite(enl, 255);
    digitalWrite(rmf, LOW);
    digitalWrite(rmb, LOW);
    digitalWrite(lmf, HIGH);
    digitalWrite(lmb, LOW);
    delay(right_delay);
    // analogWrite(enr, 0);
    // digitalWrite(rmf, LOW);
    // digitalWrite(rmb, LOW);
}

void moveForward() {    
    Serial.println("straight");
    analogWrite(enr, 255);
    analogWrite(enl ,255);
    digitalWrite(rmf, HIGH);
    digitalWrite(rmb, LOW);
    digitalWrite(lmf, HIGH);
    digitalWrite(lmb, LOW);
    // delay(straight_delay);
    // analogWrite(enr, 0);
    // digitalWrite(rmf, LOW);
    // digitalWrite(rmb, LOW);
    // digitalWrite(lmf, LOW);
    // digitalWrite(lmb, LOW);
}

void stop()
{
    Serial.println("stop");
    analogWrite(enr, 0);
    analogWrite(enl, 0);
    digitalWrite(rmf, LOW);
    digitalWrite(rmb, LOW);
    digitalWrite(lmf, LOW);
    digitalWrite(lmb, LOW);
}
