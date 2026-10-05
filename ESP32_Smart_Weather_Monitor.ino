#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>

// =====================================================
// WIFI HOTSPOT
// =====================================================

const char* AP_SSID = "ESP32-Sensor-Hub";
const char* AP_PASSWORD = "12345678";

WebServer server(80);

// =====================================================
// ANALOG SENSOR PINS
// =====================================================

#define MQ135_PIN       35
#define LDR1_PIN        32
#define LDR2_PIN        33
#define FLAME_AO_PIN    34
#define SOUND_AO_PIN    39
#define RAIN_AO_PIN     36

// =====================================================
// DIGITAL SENSOR PINS
// =====================================================

#define FLAME_DO_PIN    27
#define SOUND_DO_PIN    14
#define RAIN_DO_PIN     13

// =====================================================
// I2C PINS
// =====================================================

// AHT20
#define AHT_SDA         21
#define AHT_SCL         22

// MPU6050
#define MPU_SDA         25
#define MPU_SCL         26

// =====================================================
// SENSOR ADDRESSES
// =====================================================

#define AHT20_ADDRESS   0x38
#define MPU6050_ADDRESS 0x68

// =====================================================
// SECOND I2C BUS FOR MPU6050
// =====================================================

TwoWire I2C_MPU = TwoWire(1);

// =====================================================
// SENSOR STATUS
// =====================================================

bool ahtOK = false;
bool mpuOK = false;

// =====================================================
// SENSOR VARIABLES
// =====================================================

// Analog sensors
int mq135Value = 0;
int ldr1Value = 0;
int ldr2Value = 0;
int flameAnalog = 0;
int soundAnalog = 0;
int rainAnalog = 0;

// Digital sensors
int flameDigital = 0;
int soundDigital = 0;
int rainDigital = 0;

// AHT20
float ahtTemperature = 0;
float ahtHumidity = 0;

// MPU6050
float accelX = 0;
float accelY = 0;
float accelZ = 0;

float gyroX = 0;
float gyroY = 0;
float gyroZ = 0;

float mpuTemperature = 0;

// =====================================================
// MPU6050 REGISTERS
// =====================================================

#define MPU_PWR_MGMT_1     0x6B
#define MPU_SMPLRT_DIV     0x19
#define MPU_CONFIG         0x1A
#define MPU_GYRO_CONFIG    0x1B
#define MPU_ACCEL_CONFIG   0x1C
#define MPU_ACCEL_XOUT_H   0x3B
#define MPU_WHO_AM_I       0x75

// =====================================================
// MPU6050 WRITE
// =====================================================

void mpuWriteByte(uint8_t reg, uint8_t value) {
    I2C_MPU.beginTransmission(MPU6050_ADDRESS);
    I2C_MPU.write(reg);
    I2C_MPU.write(value);
    I2C_MPU.endTransmission();
}

// =====================================================
// MPU6050 READ
// =====================================================

bool mpuReadBytes(uint8_t reg, uint8_t* buffer, uint8_t length) {
    I2C_MPU.beginTransmission(MPU6050_ADDRESS);
    I2C_MPU.write(reg);

    if (I2C_MPU.endTransmission(false) != 0) {
        return false;
    }

    uint8_t received = I2C_MPU.requestFrom(MPU6050_ADDRESS, length);

    if (received != length) {
        return false;
    }

    for (uint8_t i = 0; i < length; i++) {
        buffer[i] = I2C_MPU.read();
    }

    return true;
}

// =====================================================
// INITIALIZE MPU6050
// =====================================================

void initializeMPU6050() {
    Serial.println();
    Serial.println("Checking MPU6050...");

    I2C_MPU.beginTransmission(MPU6050_ADDRESS);
    uint8_t error = I2C_MPU.endTransmission();

    if (error != 0) {
        Serial.println("MPU6050 NOT detected.");
        mpuOK = false;
        return;
    }

    Serial.println("MPU6050 device found at 0x68.");

    uint8_t whoAmI = 0;

    if (mpuReadBytes(MPU_WHO_AM_I, &whoAmI, 1)) {
        Serial.print("WHO_AM_I = 0x");
        Serial.println(whoAmI, HEX);
    }

    mpuWriteByte(MPU_PWR_MGMT_1, 0x00);
    delay(100);

    mpuWriteByte(MPU_SMPLRT_DIV, 0x07);
    mpuWriteByte(MPU_CONFIG, 0x03);
    mpuWriteByte(MPU_GYRO_CONFIG, 0x00);
    mpuWriteByte(MPU_ACCEL_CONFIG, 0x00);

    delay(100);

    mpuOK = true;

    Serial.println("MPU6050 initialized successfully.");
}

// =====================================================
// READ MPU6050
// =====================================================

void readMPU6050() {
    if (!mpuOK) {
        return;
    }

    uint8_t data[14];

    if (!mpuReadBytes(MPU_ACCEL_XOUT_H, data, 14)) {
        return;
    }

    // Raw accelerometer
    int16_t rawAccelX = ((int16_t)data[0] << 8) | data[1];
    int16_t rawAccelY = ((int16_t)data[2] << 8) | data[3];
    int16_t rawAccelZ = ((int16_t)data[4] << 8) | data[5];

    // Raw temperature
    int16_t rawTemp = ((int16_t)data[6] << 8) | data[7];

    // Raw gyroscope
    int16_t rawGyroX = ((int16_t)data[8] << 8) | data[9];
    int16_t rawGyroY = ((int16_t)data[10] << 8) | data[11];
    int16_t rawGyroZ = ((int16_t)data[12] << 8) | data[13];

    // ===================================================
    // ACCELEROMETER
    // ±2g = 16384 LSB/g
    // ===================================================

    accelX = ((float)rawAccelX / 16384.0) * 9.80665;
    accelY = ((float)rawAccelY / 16384.0) * 9.80665;
    accelZ = ((float)rawAccelZ / 16384.0) * 9.80665;

    // ===================================================
    // GYROSCOPE
    // ±250 degrees/sec = 131 LSB
    // Convert to rad/sec
    // ===================================================

    gyroX = ((float)rawGyroX / 131.0) * DEG_TO_RAD;
    gyroY = ((float)rawGyroY / 131.0) * DEG_TO_RAD;
    gyroZ = ((float)rawGyroZ / 131.0) * DEG_TO_RAD;

    // ===================================================
    // MPU TEMPERATURE
    // ===================================================

    mpuTemperature = ((float)rawTemp / 340.0) + 36.53;
}

// =====================================================
// AHT20 COMMAND
// =====================================================

bool ahtWriteCommand(uint8_t command, uint8_t param1, uint8_t param2) {
    Wire.beginTransmission(AHT20_ADDRESS);
    Wire.write(command);
    Wire.write(param1);
    Wire.write(param2);

    return (Wire.endTransmission() == 0);
}

// =====================================================
// INITIALIZE AHT20
// =====================================================

void initializeAHT20() {
    Serial.println();
    Serial.println("Checking AHT20...");

    Wire.beginTransmission(AHT20_ADDRESS);

    if (Wire.endTransmission() != 0) {
        Serial.println("AHT20 NOT detected.");
        ahtOK = false;
        return;
    }

    delay(40);

    // Initialization
    ahtWriteCommand(0xBE, 0x08, 0x00);
    delay(10);

    ahtOK = true;

    Serial.println("AHT20 detected.");
}

// =====================================================
// READ AHT20
// =====================================================

bool readAHT20() {
    if (!ahtOK) {
        return false;
    }

    // Trigger measurement
    if (!ahtWriteCommand(0xAC, 0x33, 0x00)) {
        return false;
    }

    delay(80);

    // Request 7 bytes
    Wire.requestFrom(AHT20_ADDRESS, (uint8_t)7);

    if (Wire.available() != 7) {
        return false;
    }

    uint8_t data[7];

    for (int i = 0; i < 7; i++) {
        data[i] = Wire.read();
    }

    // Sensor busy
    if (data[0] & 0x80) {
        return false;
    }

    // Humidity raw
    uint32_t humidityRaw =
        ((uint32_t)data[1] << 12) |
        ((uint32_t)data[2] << 4) |
        ((uint32_t)data[3] >> 4);

    // Temperature raw
    uint32_t temperatureRaw =
        ((uint32_t)(data[3] & 0x0F) << 16) |
        ((uint32_t)data[4] << 8) |
        data[5];

    // Humidity %
    ahtHumidity =
        ((float)humidityRaw * 100.0) / 1048576.0;

    // Temperature °C
    ahtTemperature =
        ((float)temperatureRaw * 200.0) / 1048576.0 - 50.0;

    return true;
}

// =====================================================
// READ ALL SENSORS
// =====================================================

void readSensors() {
    // ===================================================
    // ANALOG
    // ===================================================

    mq135Value = analogRead(MQ135_PIN);
    ldr1Value = analogRead(LDR1_PIN);
    ldr2Value = analogRead(LDR2_PIN);
    flameAnalog = analogRead(FLAME_AO_PIN);
    soundAnalog = analogRead(SOUND_AO_PIN);
    rainAnalog = analogRead(RAIN_AO_PIN);

    // ===================================================
    // DIGITAL
    // ===================================================

    flameDigital = digitalRead(FLAME_DO_PIN);
    soundDigital = digitalRead(SOUND_DO_PIN);
    rainDigital = digitalRead(RAIN_DO_PIN);

    // ===================================================
    // AHT20
    // ===================================================

    readAHT20();

    // ===================================================
    // MPU6050
    // ===================================================

    readMPU6050();
}

// =====================================================
// JSON DATA
// =====================================================

void handleData() {
    readSensors();

    String json = "{";

    // MQ135
    json += "\"mq135\":";
    json += String(mq135Value);
    json += ",";

    // LDR
    json += "\"ldr1\":";
    json += String(ldr1Value);
    json += ",";

    json += "\"ldr2\":";
    json += String(ldr2Value);
    json += ",";

    // Flame
    json += "\"flameAnalog\":";
    json += String(flameAnalog);
    json += ",";

    json += "\"flameDigital\":";
    json += String(flameDigital);
    json += ",";

    // Sound
    json += "\"soundAnalog\":";
    json += String(soundAnalog);
    json += ",";

    json += "\"soundDigital\":";
    json += String(soundDigital);
    json += ",";

    // Rain
    json += "\"rainAnalog\":";
    json += String(rainAnalog);
    json += ",";

    json += "\"rainDigital\":";
    json += String(rainDigital);
    json += ",";

    // AHT20
    json += "\"ahtOK\":";
    json += ahtOK ? "true" : "false";
    json += ",";

    json += "\"ahtTemp\":";
    json += String(ahtTemperature, 2);
    json += ",";

    json += "\"humidity\":";
    json += String(ahtHumidity, 2);
    json += ",";

    // MPU6050
    json += "\"mpuOK\":";
    json += mpuOK ? "true" : "false";
    json += ",";

    json += "\"accelX\":";
    json += String(accelX, 2);
    json += ",";

    json += "\"accelY\":";
    json += String(accelY, 2);
    json += ",";

    json += "\"accelZ\":";
    json += String(accelZ, 2);
    json += ",";

    json += "\"gyroX\":";
    json += String(gyroX, 2);
    json += ",";

    json += "\"gyroY\":";
    json += String(gyroY, 2);
    json += ",";

    json += "\"gyroZ\":";
    json += String(gyroZ, 2);
    json += ",";

    json += "\"mpuTemp\":";
    json += String(mpuTemperature, 2);

    json += "}";

    server.send(200, "application/json", json);
}

// =====================================================
// WEBPAGE
// =====================================================

void handleRoot() {
    String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>ESP32 Smart Weather Monitor</title>

<style>
* {
    box-sizing: border-box;
}

body {
    margin: 0;
    padding: 20px;
    font-family: Arial, sans-serif;
    background: #101820;
    color: white;
}

h1 {
    text-align: center;
    margin-bottom: 5px;
}

.status {
    text-align: center;
    color: #00ff88;
    margin-bottom: 25px;
    line-height: 1.7;
}

.grid {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(230px, 1fr));
    gap: 15px;
    max-width: 1200px;
    margin: auto;
}

.card {
    background: #1c2730;
    border-radius: 12px;
    padding: 20px;
    box-shadow: 0 3px 10px rgba(0,0,0,0.3);
}

.card h2 {
    margin-top: 0;
    font-size: 20px;
}

.value {
    font-size: 26px;
    font-weight: bold;
    margin: 8px 0;
}

.small {
    font-size: 18px;
}

.label {
    color: #aaaaaa;
    font-size: 14px;
    margin-top: 5px;
}

.detected {
    color: #00ff88;
}

.notdetected {
    color: #ff5555;
}

hr {
    border: 0;
    border-top: 1px solid #444;
    margin: 15px 0;
}
</style>
</head>

<body>

<h1>ESP32 Smart Weather Monitor</h1>

<div class="status">
    Wi-Fi: ESP32-Sensor-Hub
    <br>
    IP: 192.168.4.1
    <br>
    Live Update: Every 1 Second
</div>

<div class="grid">

<!-- MQ135 -->
<div class="card">
    <h2>MQ-135</h2>
    <div class="label">Raw ADC Value</div>
    <div class="value" id="mq135">--</div>
</div>

<!-- LDR -->
<div class="card">
    <h2>LDR Sensors</h2>

    <div class="label">LDR 1</div>
    <div class="value" id="ldr1">--</div>

    <div class="label">LDR 2</div>
    <div class="value" id="ldr2">--</div>
</div>

<!-- FLAME -->
<div class="card">
    <h2>Flame Sensor</h2>

    <div class="label">Analog</div>
    <div class="value" id="flameAnalog">--</div>

    <div class="label">Digital</div>
    <div class="value" id="flameDigital">--</div>
</div>

<!-- SOUND -->
<div class="card">
    <h2>Sound Sensor</h2>

    <div class="label">Analog</div>
    <div class="value" id="soundAnalog">--</div>

    <div class="label">Digital</div>
    <div class="value" id="soundDigital">--</div>
</div>

<!-- RAIN -->
<div class="card">
    <h2>Rain Sensor</h2>

    <div class="label">Analog</div>
    <div class="value" id="rainAnalog">--</div>

    <div class="label">Digital</div>
    <div class="value" id="rainDigital">--</div>
</div>

<!-- AHT20 -->
<div class="card">
    <h2>AHT20</h2>

    <div class="detected" id="ahtStatus">
        Checking...
    </div>

    <div class="label">Temperature</div>
    <div class="value">
        <span id="ahtTemp">--</span> °C
    </div>

    <div class="label">Humidity</div>
    <div class="value">
        <span id="humidity">--</span> %
    </div>
</div>

<!-- MPU6050 -->
<div class="card">
    <h2>MPU6050</h2>

    <div class="detected" id="mpuStatus">
        Checking...
    </div>

    <hr>

    <div class="label">Acceleration X</div>
    <div class="value small" id="accelX">--</div>

    <div class="label">Acceleration Y</div>
    <div class="value small" id="accelY">--</div>

    <div class="label">Acceleration Z</div>
    <div class="value small" id="accelZ">--</div>

    <br>

    <div class="label">Gyroscope X</div>
    <div class="value small" id="gyroX">--</div>

    <div class="label">Gyroscope Y</div>
    <div class="value small" id="gyroY">--</div>

    <div class="label">Gyroscope Z</div>
    <div class="value small" id="gyroZ">--</div>

    <br>

    <div class="label">Temperature</div>
    <div class="value small">
        <span id="mpuTemp">--</span> °C
    </div>
</div>

</div>

<script>
function updateData() {
    fetch('/data')
        .then(response => response.json())
        .then(data => {

            // MQ135
            document.getElementById("mq135").innerText =
                data.mq135;

            // LDR
            document.getElementById("ldr1").innerText =
                data.ldr1;

            document.getElementById("ldr2").innerText =
                data.ldr2;

            // FLAME
            document.getElementById("flameAnalog").innerText =
                data.flameAnalog;

            document.getElementById("flameDigital").innerText =
                data.flameDigital;

            // SOUND
            document.getElementById("soundAnalog").innerText =
                data.soundAnalog;

            document.getElementById("soundDigital").innerText =
                data.soundDigital;

            // RAIN
            document.getElementById("rainAnalog").innerText =
                data.rainAnalog;

            document.getElementById("rainDigital").innerText =
                data.rainDigital;

            // AHT20
            document.getElementById("ahtTemp").innerText =
                data.ahtTemp;

            document.getElementById("humidity").innerText =
                data.humidity;

            if (data.ahtOK) {
                document.getElementById("ahtStatus").innerText =
                    "Detected";
            } else {
                document.getElementById("ahtStatus").innerText =
                    "Not Detected";
            }

            // MPU6050
            if (data.mpuOK) {
                document.getElementById("mpuStatus").innerText =
                    "Detected";
            } else {
                document.getElementById("mpuStatus").innerText =
                    "Not Detected";
            }

            document.getElementById("accelX").innerText =
                data.accelX + " m/s²";

            document.getElementById("accelY").innerText =
                data.accelY + " m/s²";

            document.getElementById("accelZ").innerText =
                data.accelZ + " m/s²";

            document.getElementById("gyroX").innerText =
                data.gyroX + " rad/s";

            document.getElementById("gyroY").innerText =
                data.gyroY + " rad/s";

            document.getElementById("gyroZ").innerText =
                data.gyroZ + " rad/s";

            document.getElementById("mpuTemp").innerText =
                data.mpuTemp;
        })
        .catch(error => {
            console.log("Connection error:", error);
        });
}

// Update every 1 second
setInterval(updateData, 1000);

// First update
updateData();
</script>

</body>
</html>
)rawliteral";

    server.send(200, "text/html", html);
}

// =====================================================
// SETUP
// =====================================================

void setup() {
    Serial.begin(115200);

    delay(1500);

    Serial.println();
    Serial.println("================================");
    Serial.println("ESP32 SMART WEATHER MONITOR");
    Serial.println("================================");

    // DIGITAL SENSOR PINS
    pinMode(FLAME_DO_PIN, INPUT);
    pinMode(SOUND_DO_PIN, INPUT);
    pinMode(RAIN_DO_PIN, INPUT);

    // ADC
    analogReadResolution(12);

    // AHT20
    // SDA = GPIO21
    // SCL = GPIO22
    Wire.begin(AHT_SDA, AHT_SCL, 100000);

    // MPU6050
    // SDA = GPIO25
    // SCL = GPIO26
    I2C_MPU.begin(MPU_SDA, MPU_SCL, 100000);

    // INITIALIZE AHT20
    initializeAHT20();

    // INITIALIZE MPU6050
    initializeMPU6050();

    // START WIFI HOTSPOT
    Serial.println();
    Serial.println("Starting WiFi Access Point...");

    WiFi.mode(WIFI_AP);

    WiFi.softAP(AP_SSID, AP_PASSWORD);

    IPAddress IP = WiFi.softAPIP();

    Serial.println();
    Serial.println("================================");
    Serial.println("WIFI HOTSPOT STARTED");
    Serial.println("================================");

    Serial.print("SSID: ");
    Serial.println(AP_SSID);

    Serial.print("Password: ");
    Serial.println(AP_PASSWORD);

    Serial.print("IP Address: ");
    Serial.println(IP);

    Serial.println("================================");

    // WEB SERVER
    server.on("/", handleRoot);
    server.on("/data", handleData);

    server.begin();

    Serial.println("Web server started.");
    Serial.println("Open: http://192.168.4.1");
    Serial.println();
}

// =====================================================
// LOOP
// =====================================================

void loop() {
    server.handleClient();
}
