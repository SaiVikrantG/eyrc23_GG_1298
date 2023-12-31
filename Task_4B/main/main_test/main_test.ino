#include <AsyncTCP.h>
#include <ESPAsyncWebSrv.h>
#include <WiFi.h>

#define read_offset 1
#define attach_offset 21

long sensor[] = {0, 1, 2}; // leftmost - 0, rightmost - 4
// WiFi credentials
const char *ssid = "TP-Link_BBF8";        // Enter your wifi hotspot ssid
const char *password = "51121921";        // Enter your wifi hotspot password
AsyncWebServer server(80);

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
float sp=2;

float kp = 5;     // dummy
float ki = 15;    // dummy
float kd = 40;    //(Kp-1)*10
float maxki = 2.0;
float maxKp = 2.0;
float maxKd = 2.0;

bool updateEnabled = true; // Flag to control real-time updates
bool start = 1;        // Flag to control whether to start or stop the updates

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

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            {
              String html = "<html><body>";
              html += "<h1>PID Tuning</h1>";
              html += "Ki: <input type='range' id='ki' min='0' max='" + String(maxki) + "' step='0.1' value='" + String(ki) + "'><span id='ki_value'>" + String(ki) + "</span><br>";
              html += "Kp: <input type='range' id='kp' min='0' max='" + String(maxKp) + "' step='0.1' value='" + String(kp) + "'><span id='kp_value'>" + String(kp) + "</span><br>";
              html += "Kd: <input type='range' id='kd' min='0' max='" + String(maxKd) + "' step='0.1' value='" + String(kd) + "'><span id='kd_value'>" + String(kd) + "</span><br>";
              html += "Max Ki: <input type='number' id='maxki' value='" + String(maxki) + "'><br>";
              html += "Max Kp: <input type='number' id='maxKp' value='" + String(maxKp) + "'><br>";
              html += "Max Kd: <input type='number' id='maxKd' value='" + String(maxKd) + "'><br>";
              html += "<button id='startButton' onclick='toggleStart()'>Start/Stop</button>";
              html += "<span id='startStatus'>" + String(start ? "Started" : "Stopped") + "</span><br>";
              html += "<button id='stopButton' onclick='toggleUpdate()'>Toggle Update</button>";
              html += "<span id='updateStatus'>" + String(updateEnabled ? "Enabled" : "Disabled") + "</span><br>";
              html += "<div id='valuesContainer'></div>";
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
              html += "function toggleStart() {";
              html += "  fetch('/toggleStart', { method: 'POST' })";
              html += "}";
              html += "function toggleUpdate() {";
              html += "  updateEnabled = !updateEnabled;";
              html += "  document.getElementById('updateStatus').innerText = updateEnabled ? 'Enabled' : 'Disabled';";
              html += "}";
              html += "function updateValues() {";
              html += "  fetch('/getValues')";
              html += "    .then(response => response.text())";
              html += "    .then(data => {";
              html += "      const valuesContainer = document.getElementById('valuesContainer');";
              html += "      const values = data.split(',');";
              html += "      valuesContainer.innerHTML = '<p>Right: ' + values[0] + '</p><p>Left: ' + values[1] + '</p>';";
              html += "    });";
              html += "}";
              html += "setInterval(updateValues, 1000);"; // Update values every second
              html += "document.getElementById('ki').addEventListener('input', function() { if(updateEnabled) { updateConstants(); updateDisplay(); } });";
              html += "document.getElementById('kp').addEventListener('input', function() { if(updateEnabled) { updateConstants(); updateDisplay(); } });";
              html += "document.getElementById('kd').addEventListener('input', function() { if(updateEnabled) { updateConstants(); updateDisplay(); } });";
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
              maxki = request->arg("maxki").toFloat();
              maxKp = request->arg("maxKp").toFloat();
              maxKd = request->arg("maxKd").toFloat();
              
              request->send(200, "text/plain", "Constants updated successfully");
            });

  // Define route for toggling start
  // server.on("/toggleStart", HTTP_POST, [](AsyncWebServerRequest *request)
  //           {
  //             start = !start;
  //             Serial.println(start);
  //             request->send(200, "text/plain", "Start toggled successfully");
  //           });

  // Define route for getting motor values
  // server.on("/getValues", HTTP_GET, [](AsyncWebServerRequest *request)
  //           {
  //             String values = String(rspeed) + "," + String(lspeed);
  //             request->send(200, "text/plain", values);
  //           });

  // Start server
  server.begin();

  // sp = 0; // need to change pos to fit our needs
}

void loop()
{
  if (start)
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