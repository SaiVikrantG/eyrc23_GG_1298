  #include <AsyncTCP.h>
  #include <ESPAsyncWebSrv.h>
  #include <WiFi.h>

  #define read_offset 1
  #define attach_offset 21
  #define weighted_mean 10

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
  float sp=0;//*****************************************************
  // float sp=30;//*****************************************************

  float kp = 0.0;
  float ki = 0.0;
  float kd = 0.0;
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
    pinMode(34, INPUT);
    pinMode(19, INPUT);

    // motors
    pinMode(rmf, OUTPUT);
    pinMode(rmb, OUTPUT);
    pinMode(lmf, OUTPUT);
    pinMode(lmb, OUTPUT);
    pinMode(enl, OUTPUT);
    pinMode(enr, OUTPUT);


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

  void pid_calc()
  {
    int j =0;
    sensor_average = 0;
    sensor_sum = 0;

    // for (int i = -1; i <= 1; i++)
    // {
    // sensor[i+read_offset] = !digitalRead(i+attach_offset);
    // sensor_average += sensor[i+read_offset] * i * weighted_mean;   //weighted mean   
    // sensor_sum += int(sensor[i+read_offset]);
    // }
    sensor_average = (!digitalRead(19)*(-1)+!digitalRead(21)*(-2)+!digitalRead(22)*(0)+!digitalRead(23)*(2)+!digitalRead(34)*(1))*weighted_mean;
    sensor_sum = (!digitalRead(19)+!digitalRead(21)+!digitalRead(22)+!digitalRead(23)+!digitalRead(34));
    if(sensor_sum){
        pos = int(sensor_average / sensor_sum);
        Serial.print("Sensor Average: ");
        Serial.println(sensor_average);
        Serial.print("Sensor Sum: ");
        Serial.println(sensor_sum);
        Serial.print("Position: ");
        Serial.println(pos);
        error = pos - sp;
        p = error;
        j += p;
        d = p - lp;
        lp = p;
        correction = int(kp * p + ki * j + kd * d);
        Serial.print("error: ");
        Serial.println(error);
        Serial.print("correction: ");
        Serial.println(correction);
    }
    else 
        stop();
    delay(500);
  }
  
  void calc_turn()
  {
    rspeed = base_speed - correction;
    lspeed = base_speed + correction;

    if (rspeed > 255)
      rspeed = 255;

    if (lspeed > 255)
      lspeed = 255;

    if (rspeed < -255)
      rspeed = -255;

    if (lspeed < -255)
      lspeed = -255;

    motor_drive(rspeed, lspeed);
    Serial.print("right: ");
    Serial.println(rspeed);
    Serial.print("left: ");
    Serial.println(lspeed);
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