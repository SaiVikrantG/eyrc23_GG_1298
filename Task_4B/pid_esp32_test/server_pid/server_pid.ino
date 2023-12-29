#include <AsyncTCP.h>
#include <ESPAsyncWebSrv.h>
#include <WiFi.h>

const char *ssid = "Nope";
const char *password = "pi314420";

AsyncWebServer server(80);

// Initial PID constants
float ki = 1.0;
float kp = 0.5;
float kd = 0.2;

// Initial maximum limits for sliders
float maxKi = 2.0;
float maxKp = 2.0;
float maxKd = 2.0;

void setup()
{
  Serial.begin(115200);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  Serial.println(WiFi.localIP());

  // Serve the HTML and JavaScript for real-time updates
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            {
              String html = "<html><body>";
              html += "<h1>PID Tuning</h1>";
              html += "Ki: <input type='range' id='ki' min='0' max='" + String(maxKi) + "' step='0.1' value='" + String(ki) + "'><span id='ki_value'>" + String(ki) + "</span><br>";
              html += "Kp: <input type='range' id='kp' min='0' max='" + String(maxKp) + "' step='0.1' value='" + String(kp) + "'><span id='kp_value'>" + String(kp) + "</span><br>";
              html += "Kd: <input type='range' id='kd' min='0' max='" + String(maxKd) + "' step='0.1' value='" + String(kd) + "'><span id='kd_value'>" + String(kd) + "</span><br>";
              html += "Max Ki: <input type='number' id='maxKi' value='" + String(maxKi) + "'><br>";
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
              html += "  maxKi = parseFloat(document.getElementById('maxKi').value);";
              html += "  maxKp = parseFloat(document.getElementById('maxKp').value);";
              html += "  maxKd = parseFloat(document.getElementById('maxKd').value);";
              html += "  document.getElementById('ki').max = maxKi;";
              html += "  document.getElementById('kp').max = maxKp;";
              html += "  document.getElementById('kd').max = maxKd;";
              html += "  updateDisplay();";
              html += "}";
              html += "document.getElementById('ki').addEventListener('input', function() { updateConstants(); updateDisplay(); });";
              html += "document.getElementById('kp').addEventListener('input', function() { updateConstants(); updateDisplay(); });";
              html += "document.getElementById('kd').addEventListener('input', function() { updateConstants(); updateDisplay(); });";
              html += "document.getElementById('maxKi').addEventListener('input', updateMaxLimits);";
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
              // Serial.print("Updated Constants - Ki: ");
              // Serial.print(ki);
              // Serial.print(", Kp: ");
              // Serial.print(kp);
              // Serial.print(", Kd: ");
              // Serial.println(kd);
              request->send(200, "text/plain", "Constants updated successfully");
            });

  // Start server
  server.begin();
}

void loop() 
{
    Serial.print(ki);
    Serial.print(", Kp: ");
    Serial.print(kp);
    Serial.print(", Kd: ");
    Serial.println(kd);
}
