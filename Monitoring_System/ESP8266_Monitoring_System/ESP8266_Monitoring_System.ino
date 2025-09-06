#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>

// MQTT connection state definitions
#define MQTT_CONNECTION_TIMEOUT -4
#define MQTT_CONNECTION_LOST -3
#define MQTT_CONNECT_FAILED -2
#define MQTT_DISCONNECTED -1
#define MQTT_CONNECTED 0
#define MQTT_CONNECT_BAD_PROTOCOL 1
#define MQTT_CONNECT_BAD_CLIENT_ID 2
#define MQTT_CONNECT_UNAVAILABLE 3
#define MQTT_CONNECT_BAD_CREDENTIALS 4
#define MQTT_CONNECT_UNAUTHORIZED 5

// MQTT settings
const char* mqtt_server = "mqtt.example.com";
const int mqtt_port = 8883;
const char* mqtt_client_id = "STM32_System";
const char* mqtt_username = "Your_Username";
const char* mqtt_password = "Your_Password";
// Certificate fingerprint for MQTT server
const char* mqtt_fingerprint = "AA:BB:CC:DD:EE:FF:00:11:22:33:44:55:66:77:88:99:AA:BB:CC:DD";

// MQTT topics
const char* topic_sys = "stm32/system";
const char* topic_adc = "stm32/sensor/adc";
const char* topic_error = "stm32/system/error";
const char* topic_command = "stm32/system/command";

const char* ssid = "Your_WiFi_SSID";
const char* password = "Your_WiFi_Password";

// Static IP configuration
IPAddress local_IP(192, 168, 1, 100);  // Desired IP address
IPAddress gateway(192, 168, 1, 1);     // Network gateway
IPAddress subnet(255, 255, 255, 0);    // Subnet mask

ESP8266WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);

WiFiClientSecure espClient;
PubSubClient mqttClient(espClient);

// Variables for system information
String systemStatus = "Receiving data...";
String taskCount = "0";
String heapUsage = "0/0";
String heapStatus = "good"; // Can be "good", "warning", or "danger"

// Separate variables for each error type
String errorSemaphore = "";    // Semaphore take failed error (0xE1)
String errorStackOverflow = ""; // Stack overflow error (0xFF)  
String errorUART = "";         // UART error (0xE0)

// Data structures for received information
String adcValue = "---";
String errorLog = "No errors";
unsigned long lastUpdate = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  // Configure static IP
  if (!WiFi.config(local_IP, gateway, subnet)) {
    Serial.println("Failed to configure static IP");
  }

  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }

  // MQTT security settings
  espClient.setFingerprint(mqtt_fingerprint);

  // Initialize web server
  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html", createWebPage());
  });
  
  server.begin();
  
  // Initialize WebSocket
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  // Configure MQTT
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setKeepAlive(60); // Set KeepAlive to 60 seconds
}

void loop() {
  server.handleClient();
  webSocket.loop();

  // Manage MQTT connection
  if (!mqttClient.connected()) {
    mqttReconnect();
  }
  mqttClient.loop(); // This function must be called regularly

  parseUARTData(); // Process serial data
}

void mqttReconnect() {
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (mqttClient.connect(mqtt_client_id, mqtt_username, mqtt_password)) {
      Serial.println("connected & secured (TLS)");
      mqttClient.subscribe(topic_command);
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" trying again in 5 seconds");

      // Add SSL error detection
      int sslError = espClient.getLastSSLError();
      if(sslError != 0) {
        Serial.print("SSL Error: ");
        Serial.println(sslError);
      }

      delay(5000);
    }
  }
}

// Callback function for receiving commands from Broker
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  if (String(topic) == topic_command) {
    Serial.print("CMD:");
    Serial.print(message);
    Serial.println(";");
  }
}

void parseUARTData() {
  static String rawData = "";
  
  // Read only one character per call
  if (Serial.available()) {
    char c = Serial.read();
    
    if (c == '\n' || c == '\r') {
      if (rawData.length() > 0) {
        processDataLine(rawData);
        
        // Send to MQTT
        if (rawData.indexOf("[SYS]") >= 0) {
          String payload = rawData.substring(rawData.indexOf("]") + 1);
          mqttClient.publish(topic_sys, payload.c_str(), 1);
        }
        else if (rawData.indexOf("[ADC]") >= 0) {
          String payload = rawData.substring(rawData.indexOf("Value: ") + 7);
          payload.trim();
          mqttClient.publish(topic_adc, payload.c_str(), 0);
        }
        else if (rawData.indexOf("[ERR]") >= 0) {
          String payload = rawData.substring(rawData.indexOf("]") + 1);
          mqttClient.publish(topic_error, payload.c_str(), 2);
        }
        
        rawData = "";
      }
    } 
    else if (c >= 32 && c <= 126) {
      rawData += c;
    }
  }
}

void processDataLine(String &line) {
  line.trim();
  
  Serial.println(line);

  if (line.indexOf("[SYS]") != -1) { 
    // Parse SYS message into separate parts
    systemStatus = "Healthy";
    
    // Extract task count
    int tasksStart = line.indexOf("Tasks:");
    if (tasksStart != -1) {
      int tasksEnd = line.indexOf('|', tasksStart);
      if (tasksEnd != -1) {
        taskCount = line.substring(tasksStart + 6, tasksEnd);
        taskCount.trim();
      }
    }
    
    // Extract heap information
    int heapStart = line.indexOf("Heap:");
    if (heapStart != -1) {
      int heapEnd = line.indexOf('b', heapStart);
      if (heapEnd != -1) {
        heapUsage = line.substring(heapStart + 5, heapEnd);
        heapUsage.trim();
        
        // Determine heap status based on usage percentage
        int slashPos = heapUsage.indexOf('/');
        if (slashPos != -1) {
          int used = heapUsage.substring(0, slashPos).toInt();
          int total = heapUsage.substring(slashPos + 1).toInt();
          float percent = (used * 100.0) / total;
          
          if (percent > 80) {
            heapStatus = "danger";
          } else if (percent > 60) {
            heapStatus = "warning";
          } else {
            heapStatus = "good";
          }
        }
      }
    }
  } 
  else if (line.indexOf("[ADC]") != -1) {
    adcValue = extractADCValue(line);
  } 
  else if (line.indexOf("[ERR]") != -1) {
    // Extract error content
    String errorMessage = extractContent(line);
    
    // Detect error type and store in appropriate variable
    if (line.indexOf("0xE1") != -1) {
      errorSemaphore = errorMessage;
    } 
    else if (line.indexOf("0xFF") != -1) {
      errorStackOverflow = errorMessage;
    }
    else if (line.indexOf("0xE0") != -1) {
      errorUART = errorMessage;
    }
    
    // Send alert
    webSocket.broadcastTXT("{\"type\":\"alert\",\"message\":\"New error detected!\"}");
  }
  
  if (millis() - lastUpdate > 200) {
    sendDataToClients();
    lastUpdate = millis();
  }
}

String extractContent(const String &line) {
  // For errors, return content after error code
  if (line.indexOf("[ERR]") != -1) {
    int codePos = line.indexOf("0x");
    if (codePos != -1) {
      int contentPos = line.indexOf(' ', codePos);
      if (contentPos != -1) {
        return line.substring(contentPos + 1);
      }
    }
    return line.substring(line.indexOf("[ERR]") + 5);
  }
  
  // For other cases
  int startPos = line.indexOf(']', line.indexOf(']')) + 6;
  return (startPos > 0 && startPos < line.length()) ? 
         line.substring(startPos) : line;
}

String extractADCValue(const String &line) {
  int startPos = line.indexOf("Value: ");
  if (startPos != -1) {
    return line.substring(startPos + 7);
  }
  return "N/A";
}

void sendDataToClients() {
  DynamicJsonDocument doc(512);
  doc["systemStatus"] = systemStatus;
  doc["taskCount"] = taskCount;
  doc["heapUsage"] = heapUsage;
  doc["heapStatus"] = heapStatus;
  doc["adc"] = adcValue;
  
  // Combine all errors into one text
  String allErrors = "";
  if (!errorSemaphore.isEmpty()) {
    allErrors += "Semaphore: " + errorSemaphore + "\n";
  }
  if (!errorStackOverflow.isEmpty()) {
    allErrors += "Stack: " + errorStackOverflow + "\n";
  }
  if (!errorUART.isEmpty()) {
    allErrors += "UART: " + errorUART + "\n";
  }
  
  if (allErrors.isEmpty()) {
    allErrors = "No errors";
  }
  
  doc["error"] = allErrors;
  
  String jsonStr;
  serializeJson(doc, jsonStr);
  
  webSocket.broadcastTXT(jsonStr);
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  if (type == WStype_TEXT) {
    String message = (char*)payload;
    
    if (message == "refresh") {
      sendDataToClients();
    }
    else if (message == "clear-errors") {
      // Clear all errors
      errorSemaphore = "";
      errorStackOverflow = "";
      errorUART = "";
      sendDataToClients();
    }
  }
}

String createWebPage() {
  String page = F("<!DOCTYPE html><html lang='en'><head>");
  page += F("<meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1.0'>");
  page += F("<title>STM32F103 Monitoring System</title>");
  page += F("<style>");
  page += F("* {box-sizing: border-box; margin: 0; padding: 0; font-family: Tahoma, Arial, sans-serif;}");
  page += F("body {background: linear-gradient(135deg, #1a2a6c, #2c3e50, #1a2a6c); color: #fff; padding: 20px; min-height: 100vh;}");
  page += F(".container {max-width: 1000px; margin: 0 auto; background: rgba(0, 0, 30, 0.7); backdrop-filter: blur(10px); border-radius: 15px; padding: 25px; box-shadow: 0 8px 32px rgba(0, 0, 0, 0.4);}");
  page += F("header {text-align: center; padding: 15px 0 25px 0; border-bottom: 2px solid rgba(100, 150, 255, 0.3); margin-bottom: 20px;}");
  page += F("h1 {font-size: 1.8rem; margin-bottom: 10px; color: #4fc3f7;}");
  
  page += F(".system-details {display: flex; flex-direction: column; gap: 12px;}");
  page += F(".system-item {display: flex; justify-content: space-between; align-items: center; padding: 8px 0; border-bottom: 1px solid rgba(255, 255, 255, 0.1);}");
  page += F(".system-item .label {font-weight: bold; color: #4fc3f7; flex: 1;}");
  page += F(".system-item .value {font-family: 'Courier New', monospace; flex: 1; text-align: left;}");
  page += F(".status-indicator {width: 12px; height: 12px; border-radius: 50%; margin-right: 10px;}");
  page += F(".health-good {color: #81c784;}");
  page += F(".health-warning {color: #ffc107;}");
  page += F(".health-danger {color: #f44336;}");
  page += F(".status-good {background-color: #81c784;}");
  page += F(".status-warning {background-color: #ffc107;}");
  page += F(".status-danger {background-color: #f44336;}");
  
  page += F(".dashboard {display: grid; grid-template-columns: 1fr 1fr; gap: 20px; margin-top: 20px;}");
  page += F(".panel {background: rgba(0, 15, 30, 0.85); border-radius: 12px; padding: 20px; box-shadow: 0 4px 15px rgba(0, 0, 0, 0.2); border: 1px solid rgba(100, 150, 255, 0.2);}");
  page += F(".full-width {grid-column: span 2;}");
  page += F(".panel h2 {display: flex; align-items: center; margin-bottom: 15px; padding-bottom: 12px; border-bottom: 1px solid rgba(100, 150, 255, 0.3); color: #4fc3f7;}");
  page += F(".panel h2 i {margin-left: 10px; font-size: 1.4rem;}");
  page += F(".data-display {min-height: 120px; overflow-y: auto; font-family: 'Courier New', monospace; white-space: pre-wrap; font-size: 1.05rem; line-height: 1.7; padding: 10px; background: rgba(0, 10, 20, 0.6); border-radius: 8px; direction: ltr; text-align: left;}");
  page += F("#adc-value {font-size: 2.5rem; text-align: center; font-weight: bold; color: #81c784; padding: 20px 0; text-shadow: 0 0 10px rgba(129, 199, 132, 0.5);}");
  page += F(".status-bar {display: flex; justify-content: space-between; background: rgba(0, 30, 60, 0.8); padding: 15px; border-radius: 10px; margin: 25px 0; font-size: 0.95rem;}");
  page += F(".status-item {display: flex; flex-direction: column; align-items: center;}");
  page += F(".status-value {font-weight: bold; font-size: 1.1rem; color: #81c784; margin-top: 5px;}");
  page += F(".buttons {display: flex; gap: 15px; margin-top: 25px;}");
  page += F("button {flex: 1; padding: 14px; border: none; border-radius: 8px; background: linear-gradient(to right, #2196F3, #21CBF3); color: white; font-weight: bold; cursor: pointer; transition: all 0.3s; font-size: 1.05rem;}");
  page += F("button:hover {transform: translateY(-3px); box-shadow: 0 6px 12px rgba(0, 0, 0, 0.3);}");
  page += F("#clear-btn {background: linear-gradient(to right, #f44336, #ff9800);}");
  page += F("#refresh-btn {background: linear-gradient(to right, #4CAF50, #8BC34A);}");
  page += F("footer {text-align: center; margin-top: 25px; padding-top: 20px; border-top: 1px solid rgba(255, 255, 255, 0.1); font-size: 0.9rem; color: #90a4ae;}");
  page += F(".alert-badge {position: fixed; top: 20px; right: 20px; background: #f44336; color: white; padding: 15px 25px; border-radius: 8px; box-shadow: 0 4px 15px rgba(244, 67, 54, 0.4); z-index: 1000; display: none; animation: pulse 2s infinite;}");
  page += F("@keyframes pulse {0% { opacity: 1; } 50% { opacity: 0.6; } 100% { opacity: 1; }}");
  page += F("@media (max-width: 768px) {.dashboard {grid-template-columns: 1fr;} .full-width {grid-column: span 1;} .system-item {flex-direction: column; align-items: flex-start;}}");
  page += F("</style>");
  page += F("<link href='https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.0.0/css/all.min.css' rel='stylesheet'>");
  page += F("</head><body>");
  page += F("<div class='alert-badge' id='alertBadge'><i class='fas fa-exclamation-triangle'></i><span id='alertText'>New Error!</span></div>");
  page += F("<div class='container'>");
  page += F("<header><h1><i class='fas fa-microchip'></i> STM32F103 Monitoring System</h1><p>Real-time system data display</p></header>");
  page += F("<div class='status-bar'>");
  page += F("<div class='status-item'><span>Connection Status:</span><span id='conn-status' class='status-value'>Connected</span></div>");
  page += F("<div class='status-item'><span>Device IP:</span><span id='ip-address' class='status-value'>");
  page += WiFi.localIP().toString();
  page += F("</span></div>");
  page += F("<div class='status-item'><span>WiFi Signal:</span><span id='wifi-rssi' class='status-value'>");
  page += String(WiFi.RSSI());
  page += F(" dBm</span></div>");
  page += F("</div>");
  page += F("<div class='dashboard'>");
  
  // System status panel with three separate fields
  page += F("<div class='panel'><h2><i class='fas fa-heartbeat'></i> System Status</h2>");
  page += F("<div class='system-details'>");
  page += F("<div class='system-item'><span class='label'>System Status:</span><span id='system-status' class='value health-good'>");
  page += systemStatus;
  page += F("</span></div>");
  page += F("<div class='system-item'><span class='label'>Active Tasks:</span><span id='task-count' class='value'>");
  page += taskCount;
  page += F("</span></div>");
  page += F("<div class='system-item'><span class='label'>Heap Usage:</span><span id='heap-usage' class='value'>");
  page += heapUsage;
  page += F("</span><span id='heap-status' class='status-indicator status-good'></span></div>");
  page += F("</div></div>");
  
  page += F("<div class='panel'><h2><i class='fas fa-bolt'></i> ADC Value</h2><div id='adc-value'>");
  page += adcValue;
  page += F("</div></div>");
  page += F("<div class='panel full-width'><h2><i class='fas fa-exclamation-circle'></i> Error Reports</h2><div class='data-display' id='error-data'>");
  page += errorLog;
  page += F("</div></div>");
  page += F("</div>");
  page += F("<div class='buttons'>");
  page += F("<button id='clear-btn'><i class='fas fa-trash-alt'></i> Clear All Data</button>");
  page += F("<button id='refresh-btn'><i class='fas fa-sync-alt'></i> Force Update</button>");
  page += F("</div>");
  page += F("<footer><p>Industrial Monitoring System | Developed with ESP8266 and STM32F103</p><p>Software Version: 1.2.0 | Last Update: 2024/08/09</p></footer>");
  page += F("</div>");
  page += F("<script>");
  page += F("const systemStatusElement = document.getElementById('system-status');");
  page += F("const taskCountElement = document.getElementById('task-count');");
  page += F("const heapUsageElement = document.getElementById('heap-usage');");
  page += F("const heapStatusElement = document.getElementById('heap-status');");
  page += F("const adcValueElement = document.getElementById('adc-value');");
  page += F("const errorDataElement = document.getElementById('error-data');");
  page += F("const clearBtn = document.getElementById('clear-btn');");
  page += F("const refreshBtn = document.getElementById('refresh-btn');");
  page += F("const alertBadge = document.getElementById('alertBadge');");
  page += F("const alertText = document.getElementById('alertText');");
  page += F("const connStatus = document.getElementById('conn-status');");
  page += F("let socket; let reconnectAttempts = 0;");
  page += F("function initWebSocket() {");
  page += F("const host = window.location.hostname;");
  page += F("socket = new WebSocket('ws://' + host + ':81/');");
  page += F("socket.onopen = () => {");
  page += F("connStatus.textContent = 'Connected';");
  page += F("connStatus.style.color = '#81C784';");
  page += F("reconnectAttempts = 0;};");
  page += F("socket.onmessage = (event) => {");
  page += F("try {");
  page += F("const data = JSON.parse(event.data);");
  page += F("if (data.systemStatus) {");
  page += F("systemStatusElement.textContent = data.systemStatus;");
  page += F("systemStatusElement.className = 'value health-good';}");
  page += F("if (data.taskCount) {");
  page += F("taskCountElement.textContent = data.taskCount;}");
  page += F("if (data.heapUsage) {");
  page += F("heapUsageElement.textContent = data.heapUsage;");
  page += F("heapStatusElement.classList.remove('status-good', 'status-warning', 'status-danger');");
  page += F("if (data.heapStatus === 'good') {");
  page += F("heapStatusElement.classList.add('status-good');");
  page += F("} else if (data.heapStatus === 'warning') {");
  page += F("heapStatusElement.classList.add('status-warning');");
  page += F("} else if (data.heapStatus === 'danger') {");
  page += F("heapStatusElement.classList.add('status-danger');}}");
  page += F("if (data.adc) {");
  page += F("adcValueElement.textContent = data.adc;");
  page += F("const value = parseInt(data.adc);");
  page += F("if (value < 100) adcValueElement.style.color = '#F44336';");
  page += F("else if (value < 500) adcValueElement.style.color = '#FFC107';");
  page += F("else adcValueElement.style.color = '#81C784';}");
  page += F("if (data.error) {");
  page += F("errorDataElement.textContent = data.error;");
  page += F("errorDataElement.scrollTop = errorDataElement.scrollHeight;}");
  page += F("if (data.type === 'alert') {");
  page += F("alertText.textContent = data.message || 'New error detected!';");
  page += F("alertBadge.style.display = 'block';");
  page += F("setTimeout(() => {alertBadge.style.display = 'none';}, 5000);}} catch(e) {");
  page += F("console.error('Error processing data:', e);}};");
  page += F("socket.onerror = (error) => {");
  page += F("console.error('WebSocket error:', error);");
  page += F("connStatus.textContent = 'Error';");
  page += F("connStatus.style.color = '#F44336';};");
  page += F("socket.onclose = () => {");
  page += F("connStatus.textContent = 'Disconnected';");
  page += F("connStatus.style.color = '#FF9800';");
  page += F("setTimeout(initWebSocket, Math.min(5000, 1000 * (reconnectAttempts + 1)));");
  page += F("reconnectAttempts++;};}");
  page += F("clearBtn.addEventListener('click', () => {");
  page += F("systemStatusElement.textContent = 'Healthy';");
  page += F("systemStatusElement.className = 'value health-good';");
  page += F("taskCountElement.textContent = '0';");
  page += F("heapUsageElement.textContent = '0/0';");
  page += F("heapStatusElement.classList.remove('status-warning', 'status-danger');");
  page += F("heapStatusElement.classList.add('status-good');");
  page += F("adcValueElement.textContent = '---';");
  page += F("adcValueElement.style.color = '#81C784';");
  page += F("errorDataElement.textContent = '';");
  page += F("if(socket.readyState === WebSocket.OPEN) {");
  page += F("socket.send('clear-errors');}});");
  page += F("refreshBtn.addEventListener('click', () => {");
  page += F("if(socket.readyState === WebSocket.OPEN) {");
  page += F("socket.send('refresh');}});");
  page += F("window.addEventListener('load', () => {");
  page += F("initWebSocket();");
  page += F("setInterval(() => {");
  page += F("document.getElementById('wifi-rssi').textContent = ");
  page += F("navigator.connection ? ");
  page += F("`${navigator.connection.rtt}ms | ${navigator.connection.downlink}Mb/s` : ");
  page += F("'---';}, 10000);});");
  page += F("</script></body></html>");

  return page;
}