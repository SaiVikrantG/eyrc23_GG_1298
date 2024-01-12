#include <AsyncTCP.h>
#include <ESPAsyncWebSrv.h>
#include <WiFi.h>

#define read_offset 1
#define attach_offset 21

#define right_delay 1000
#define left_delay 1000

long sensor[] = {0, 1, 2}; // leftmost - 0, rightmost - 4
// WiFi credentials
const char *ssid = "TP-Link_BBF8";        // Enter your wifi hotspot ssid
const char *password = "51121921";        // Enter your wifi hotspot password
AsyncWebServer server(80);

//ir pins
int front_left = 23;
int front_right = 21;
int back_left = 32;
int back_mid = 22;
int back_right = 19;

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

int count = 0;

//int button = 3; // to be pressed to find set point

float p;
float i;
float d;
float lp;
float error;
float correction;
float sp=12.99;//*****************************************************
//float sp=30;//*****************************************************

float kp = 0.4;
float ki = 0.0;
float kd = 3.8;
int startSignal = 0;


void get_PID_consts();
void pid_calc();
void calc_turn();
void motor_drive(int, int);
void stop();
void updateWebValues();

void setup()
{
  // sensors
  pinMode(21, INPUT);
  pinMode(22, INPUT);
  pinMode(23, INPUT);
  pinMode(32, INPUT);
  pinMode(19, INPUT);

  // motors
  pinMode(rmf, OUTPUT);
  pinMode(rmb, OUTPUT);
  pinMode(lmf, OUTPUT);
  pinMode(lmb, OUTPUT);

  Serial.begin(115200);
  // Connecting to wifi
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.println("...");
  }

  Serial.print("WiFi connected with IP: ");
  Serial.println(WiFi.localIP());

server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    String html = "<html><body>";
    html += "<h1>PID Tuning</h1>";
    html += "ki: <input type='range' id='ki' min='0' max='10' step='0.1' value='" + String(ki) + "'><span id='ki_value'>" + String(ki) + "</span><br>";
    html += "kp: <input type='range' id='kp' min='0' max='10' step='0.1' value='" + String(kp) + "'><span id='kp_value'>" + String(kp) + "</span><br>";
    html += "kd: <input type='range' id='kd' min='0' max='10' step='0.1' value='" + String(kd) + "'><span id='kd_value'>" + String(kd) + "</span><br>";
    html += "Start: <button onclick='startStop()'>Start</button><br>";
    html += "<script>";
    html += "function updateConstants() {";
    html += "  var kiValue = document.getElementById('ki').value;";
    html += "  var kpValue = document.getElementById('kp').value;";
    html += "  var kdValue = document.getElementById('kd').value;";
    html += "  fetch('/update?ki=' + kiValue + '&kp=' + kpValue + '&kd=' + kdValue, { method: 'POST' })";
    html += "}";
    html += "function updateDisplay(sliderId, valueId) {";
    html += "  document.getElementById(valueId).innerText = document.getElementById(sliderId).value;";
    html += "}";
    html += "function startStop() {";
    html += "  fetch('/start', { method: 'POST' })";
    html += "}";
    html += "document.getElementById('ki').addEventListener('input', function() { updateConstants(); updateDisplay('ki', 'ki_value'); });";
    html += "document.getElementById('kp').addEventListener('input', function() { updateConstants(); updateDisplay('kp', 'kp_value'); });";
    html += "document.getElementById('kd').addEventListener('input', function() { updateConstants(); updateDisplay('kd', 'kd_value'); });";
    html += "</script>";
    html += "</body></html>";
    request->send(200, "text/html", html);
  });

  server.on("/update", HTTP_POST, [](AsyncWebServerRequest *request) {
     ki = request->arg("ki").toFloat();
     kp = request->arg("kp").toFloat();
     kd = request->arg("kd").toFloat();
    request->send(200, "text/plain", "Constants and Start signal updated successfully");
  });

  server.on("/start", HTTP_POST, [](AsyncWebServerRequest *request) {
    startSignal = !startSignal;  // Toggle start signal
    request->send(200, "text/plain", "Start signal toggled");
  });

  // Start server
  server.begin();

  // sp = 0; // need to change pos to fit our needs
}

void loop()
{
  if (startSignal)
  {
    pid_calc();
    calc_turn();
  }
  else
  {
    stop();
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

void check(){
  //blob encounter
  int ir_0 = digitalRead(front_left);
  int ir_1 = digitalRead(front_right);

  if(ir_0 && ir_1){
    stop();
    delay(2000);
    if(count <= 11){
        count += 1;
        switch (count){
            case 1:
            case 2:
            case 7:
            case 9:
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

void pid_calc()
{
  check();
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
  sensor_average += (4) * (!digitalRead(32)) + !digitalRead(19) * (5);
  sensor_sum += !digitalRead(32) + !digitalRead(19);
  if (sensor_sum)
    pos = int(sensor_average / sensor_sum);
  else
    stop();
  
  error = pos - sp;
  p = error;
  j += p;
  d = p - lp;
  

  // Serial.print("ki: ");
  // Serial.println(ki);
  // Serial.print("kp: ");
  // Serial.println(kp);
  // Serial.print("kd: ");
  // Serial.println(kd);
  lp = p;
  correction = int(kp * p + ki * j + kd * d);
  Serial.print("correction: ");
  Serial.println(correction);
  delay(100);
  check();
}

void calc_turn()
{
  check();
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