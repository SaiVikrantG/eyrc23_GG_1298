#include <WiFi.h>
#include <ArduinoJson.h>

#define read_offset 1
#define attach_offset 22

long sensor[] = {0, 1, 2}; //leftmost - 0, rightmost - 4
// WiFi credentials
const char* ssid = "Nope";                    //Enter your wifi hotspot ssid
const char* password =  "pi314420";               //Enter your wifi hotspot password
const uint16_t port = 8002;
const char * host = "192.168.0.101"; 
StaticJsonDocument<200> jsonDocument;
DeserializationError j_error;
int P=0,I=0,D=0;

char incomingPacket[80];
WiFiClient client;
String msg = "0";

int rmf = 5;
int rmb = 15;
int lmf = 16; 
int lmb = 4;
int enr = 26;
int enl =27;
//speeds
int rspeed;
int lspeed;
const int base_speed = 255;

int pos;
long sensor_average;
int sensor_sum;

int button = 3; //to be pressed to find set point

float p;
float i;
float d;
float lp;
float error;
float correction;
float sp;

float Kp = 5; // dummy
float Ki = 15; //dummy
float Kd = 40; //(Kp-1)*10

void get_PID_consts();
void pid_calc();
void calc_turn();
void motor_drive(int , int );

void setup()
{
  //sensors
  pinMode(21, INPUT);
  pinMode(22, INPUT);
  pinMode(23, INPUT);
  pinMode(32, INPUT);
  pinMode(1, INPUT);

  //motors
  pinMode(rmf, OUTPUT);
  pinMode(rmb, OUTPUT);
  pinMode(lmf, OUTPUT);
  pinMode(lmb, OUTPUT);

  Serial.begin(115200);
   //Connecting to wifi
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("...");
  }
 
  Serial.print("WiFi connected with IP: ");
  Serial.println(WiFi.localIP());
  
  //finding set point

  // while(button)
  // {
  //   sensor_average = 0;
  //   sensor_sum = 0;
  
  //   for (int i = -1; i <= 1; i++)
  //   {
  //     sensor[i+read_offset] = digitalRead(i+attach_offset);
  //     sensor_average += sensor[i+read_offset] * i * 1000;   //weighted mean   
  //     sensor_sum += int(sensor[i+read_offset]);
  //   }
  //   sensor_sum += digitalRead(32)*(-2)+digitalRead(1)*(2);
  
  //   pos = int(sensor_average / sensor_sum);
  
  //   Serial.print(sensor_average);
  //   Serial.print(' ');
  //   Serial.print(sensor_sum);
  //   Serial.print(' ');
  //   Serial.print(pos);
  //   Serial.println();
  //   delay(2000);
  // }

  sp = 0;//need to change pos to fit our needs
}

void loop()
{ 
  get_PID_consts();
  pid_calc();
  calc_turn();
}
void get_PID_consts()
{
  msg = client.readStringUntil('\n');         //Read the message through the socket until new line char(\n)
  Serial.println(msg);
  j_error = deserializeJson(jsonDocument,msg);
  if(j_error)
  {
    Serial.print("Failed to parse JSON: ");
    Serial.println(j_error.c_str());
  }
  else 
  {
    P = jsonDocument["P"];
    I = jsonDocument["I"];
    D = jsonDocument["D"];
    float Kp = P;
    float Ki = I;
    float Kd = D;

  }

}
void pid_calc()
{
  Serial.println("correction");
  sensor_average = 0;
  sensor_sum = 0;
  i = 0;

  for(int i = -2; i <= 2; i++)
  {
    // sensor[i]=analogRead(i);
    // sensor_average = sensor[i]*i*1000; //weighted mean
    // sensor_sum += sensor[i];
    sensor[i+read_offset]=!digitalRead(i+attach_offset);
    sensor_average = sensor[i+read_offset]*i*15; //weighted mean **may need to change me1000*
    sensor_sum += sensor[i+read_offset];
  }
  sensor_sum += !digitalRead(32)*(-2)+!digitalRead(1)*(2);
  pos = int(sensor_average / sensor_sum);

  error = pos-sp;
  Serial.print("Ki");
  Serial.println(Ki);
  Serial.print("Error:");
  Serial.println(error);
  
  Serial.print("pos");
  Serial.println(pos);

  p = error;
  i += p;
  d = p - lp;

  lp = p;
  correction = int(Kp*p + Ki*i + Kd*d);
}

void calc_turn()
{
  rspeed = base_speed + correction;
  lspeed = base_speed - correction;

  //restricting speeds of motors between 255 and -255
  
  if (rspeed > 255) 
    rspeed = 255;
    
  if (lspeed > 255) 
    lspeed = 255;
    
  if (rspeed < -255) 
    rspeed = -255;
    
  if (lspeed < -255) 
    lspeed = -255;
  Serial.println("calc_turn");
 motor_drive(rspeed,lspeed);  
}

void motor_drive(int right, int left){
  Serial.print("Right Motor");
  Serial.println(right);
  Serial.print("Left Motor");
  Serial.println(left);
  if(right>0)
  { 
    analogWrite(enr,right);
    digitalWrite(rmf, HIGH);   
    digitalWrite(rmb, 0);
  }
  else 
  {
    analogWrite(enr,abs(right));
    digitalWrite(rmf, LOW); 
    digitalWrite(rmb, HIGH);
  }
  
 
  if(left>0)
  {
    analogWrite(enl,255);
    digitalWrite(lmf, HIGH);
    digitalWrite(lmb, LOW);
  }
  else 
  {
    analogWrite(enl,abs(left));
    digitalWrite(lmf, LOW);
    digitalWrite(lmb, HIGH);
  }
  
}