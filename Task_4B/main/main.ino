#include <AsyncTCP.h>
#include <ESPAsyncWebSrv.h>
#include <WiFi.h>

#define read_offset 1
#define attach_offset 22

long sensor[] = {0, 1, 2}; //leftmost - 0, rightmost - 4
// WiFi credentials
const char* ssid = "Nope";                    //Enter your wifi hotspot ssid
const char* password =  "pi314420";               //Enter your wifi hotspot password
const uint16_t port = 8002;
const char * host = "192.168.0.101"; 
AsyncWebServer server(80);



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

float kp = 5; // dummy
float ki = 15; //dummy
float kd = 40; //(Kp-1)*10
float maxki = 2.0;
float maxKp = 2.0;
float maxKd = 2.0;
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
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            {
              String html = "<html><body>";
              html += "<h1>PID Tuning</h1>";
              html += "ki: <input type='range' id='ki' min='0' max='" + String(maxki) + "' step='0.1' value='" + String(ki) + "'><span id='ki_value'>" + String(ki) + "</span><br>";
              html += "Kp: <input type='range' id='kp' min='0' max='" + String(maxKp) + "' step='0.1' value='" + String(kp) + "'><span id='kp_value'>" + String(kp) + "</span><br>";
              html += "Kd: <input type='range' id='kd' min='0' max='" + String(maxKd) + "' step='0.1' value='" + String(kd) + "'><span id='kd_value'>" + String(kd) + "</span><br>";
              html += "Max ki: <input type='number' id='maxki' value='" + String(maxki) + "'><br>";
              html += "Max Kp: <input type='number' id='maxKp' value='" + String(maxKp) + "'><br>";
              html += "Max Kd: <input type='number' id='maxKd' value='" + String(maxKd) + "'><br>";
              html += "<script>";
              html += "function updateConstants() {";
              html += "  var ki = document.getElementById('ki').value;";
              html += "  var kp = document.getElementById('kp').value;";
              html += "  var kd = document.getElementById('kd').value;";
              html += "  fetch('/update', { method: 'POST', body: 'ki=' + ki + '&kp=' + kp + '&kd=' + kd })";
              html += "}";
              html += "function updateDisplay() {";
              html += "  document.getElementById('ki_value').innerText = document.getElementById('ki').value;";
              html += "  document.getElementById('kp_value').innerText = document.getElementById('kp').value;";
              html += "  document.getElementById('kd_value').innerText = document.getElementById('kd').value;";
              html += "}";
              html += "function updateMaxLimits() {";
              html += "  maxki = parseFloat(document.getElementById('maxki').value);";
              html += "  maxKp = parseFloat(document.getElementById('maxKp').value);";
              html += "  maxKd = parseFloat(document.getElementById('maxKd').value);";
              html += "  document.getElementById('ki').max = maxki;";
              html += "  document.getElementById('kp').max = maxKp;";
              html += "  document.getElementById('kd').max = maxKd;";
              html += "  updateDisplay();";
              html += "}";
              html += "document.getElementById('ki').addEventListener('input', function() { updateConstants(); updateDisplay(); });";
              html += "document.getElementById('kp').addEventListener('input', function() { updateConstants(); updateDisplay(); });";
              html += "document.getElementById('kd').addEventListener('input', function() { updateConstants(); updateDisplay(); });";
              html += "document.getElementById('maxki').addEventListener('input', updateMaxLimits);";
              html += "document.getElementById('maxKp').addEventListener('input', updateMaxLimits);";
              html += "document.getElementById('maxKd').addEventListener('input', updateMaxLimits);";
              html += "</script>";
              html += "</body></html>";
              request->send(200, "text/html", html);
            });

  // Define route for updating PID constants
  server.on("/update", HTTP_POST, [](AsyncWebServerRequest *request)
            {
              ki = request->arg("ki").toFloat();
              kp = request->arg("kp").toFloat();
              kd = request->arg("kd").toFloat();
              request->send(200, "text/plain", "Constants updated successfully");
            });

  // Start server
  server.begin();
 
  sp = 0;//need to change pos to fit our needs
}

void loop()
{ 
  
  pid_calc();
  calc_turn();
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
  Serial.print("ki");
  Serial.println(ki);
  Serial.print("Error:");
  Serial.println(error);
  
  Serial.print("pos");
  Serial.println(pos);
  delay(500);
  p = error;
  i += p;
  d = p - lp;

  lp = p;
  correction = int(kp*p + ki*i + kd*d);
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