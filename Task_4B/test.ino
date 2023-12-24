#include <WiFi.h>
#include <ArduinoJson.h>

#define read_offset 2
#define attach_offset 34
long sensor[] = {0, 1, 2, 3, 4}; //leftmost - 0, rightmost - 4
// WiFi credentials
const char* ssid = "TPLINK-52";                    //Enter your wifi hotspot ssid
const char* password =  "aditya20";               //Enter your wifi hotspot password
const uint16_t port = 8002;
const char * host = "192.168.0.106"; 
StaticJsonDocument<200> jsonDocument;
DeserializationError j_error;
int P=0,I=0,D=0;

char incomingPacket[80];
WiFiClient client;
String msg = "0";

int rmf = 9;
int rmb = 6;
int lmf = 10; 
int lmb = 11;

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
float Ki = 0; //dummy
float Kd = 40; //(Kp-1)*10

void get_PID_consts();
void pid_calc();
void calc_turn();
void motor_drive(int , int );

void setup()
{
  //sensors
  pinMode(1, INPUT);
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(4, INPUT);
  pinMode(5, INPUT);

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

  while(button)
  {
    sensor_average = 0;
    sensor_sum = 0;
  
    for (int i = -2; i <= 2; i++)
    {
      sensor[i+read_offset] = digitalRead(i+attach_offset);
      sensor_average += sensor[i+read_offset] * i * 1000;   //weighted mean   
      sensor_sum += int(sensor[i+read_offset]);
    }
  
    pos = int(sensor_average / sensor_sum);
  
    Serial.print(sensor_average);
    Serial.print(' ');
    Serial.print(sensor_sum);
    Serial.print(' ');
    Serial.print(pos);
    Serial.println();
    delay(2000);
  }

  sp = pos;//need to change pos to fit our needs
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
  sensor_average = 0;
  sensor_sum = 0;
  i = 0;

  for(int i = -2; i <= 2; i++)
  {
    // sensor[i]=analogRead(i);
    // sensor_average = sensor[i]*i*1000; //weighted mean
    // sensor_sum += sensor[i];
    sensor[i+read_offset]=digitalRead(i+attach_offset);
    sensor_average = sensor[i+read_offset]*i*1000; //weighted mean **may need to change 1000*
    sensor_sum += sensor[i+read_offset];
  }

  pos = int(sensor_average / sensor_sum);

  error = pos-sp;

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
 
 motor_drive(rspeed,lspeed);  
}

void motor_drive(int right, int left){
  
  if(right>0)
  {
    analogWrite(rmf, right);   
    analogWrite(rmb, 0);
  }
  else 
  {
    analogWrite(rmf, 0); 
    analogWrite(rmb, abs(right));
  }
  
 
  if(left>0)
  {
    analogWrite(lmf, left);
    analogWrite(lmb, 0);
  }
  else 
  {
    analogWrite(lmf, 0);
    analogWrite(lmb, abs(left));
  }
  
}