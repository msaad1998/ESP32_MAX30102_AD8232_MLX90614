#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <FirebaseESP32.h>
#include <MAX30105.h>

#define FIREBASE_HOST "https://healthmonitoringsystem-557b3-default-rtdb.firebaseio.com/"  //Paste your Realtime database url here 
#define FIREBASE_AUTH "AIzaSyBgX-mmOUoSzmuaDoCv0Z1T3nuAxKLQZfQ" ///// <---- Paste your API key here
#define WIFI_SSID "ASIM"
#define WIFI_PASSWORD "retrograde"        
#define FIREBASE_ROOT "values"

const int AD8232_LO_PLUS = 34; // LO+ pin of AD8232 connected to pin 34 of ESP32
const int AD8232_LO_MINUS = 35; // LO- pin of AD8232 connected to pin 35 of ESP32
const int AD8232_OUTPUT = 32; // OUTPUT pin of AD8232 connected to pin 32 of ESP32

Adafruit_MLX90614 mlx = Adafruit_MLX90614();

FirebaseData firebaseData;
FirebaseJson json;

MAX30105 particleSensor;

const int BUFFER_SIZE = 250;
int ecgBuffer[BUFFER_SIZE];
int bufferIndex = 0;

void setup()
{
  Serial.begin(115200);
  mlx.begin();
  
  pinMode(AD8232_LO_PLUS, INPUT);
  pinMode(AD8232_LO_MINUS, INPUT);
  pinMode(AD8232_OUTPUT, INPUT);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Firebase.reconnectWiFi(true);

  Firebase.setReadTimeout(firebaseData, 1000 * 60);
  Firebase.setwriteSizeLimit(firebaseData, "tiny");

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 was not found. Please check wiring/power.");
    while (1);
  }

  particleSensor.setup(0x1F, 4, 2, 4096); // Configure sensor with settings for SpO2 calculation
  particleSensor.setPulseAmplitudeRed(0x0A); // Set LED pulse amplitude for red LED
  particleSensor.setPulseAmplitudeIR(0x0A); // Set LED pulse amplitude for IR LED

  Serial.println("------------------------------------");
  Serial.println("Connected...");
  
}

float calculateECG(int pin) {
  // Take 250 samples of ECG signal
  const int SAMPLES = 250;
  int ecgBuffer[SAMPLES];
  for (int i = 0; i < SAMPLES; i++) {
    ecgBuffer[i] = analogRead(pin);
    delayMicroseconds(2); // 500 samples per second
  }
  // Calculate ECG signal as average of absolute differences between each sample and the average of all samples
  float ecgSignal = 0;
  float ecgAverage = 0;
  for (int i = 0; i < SAMPLES; i++) {
    ecgAverage += ecgBuffer[i];
  }
  ecgAverage /= SAMPLES;
  for (int i = 0; i < SAMPLES; i++) {
    ecgSignal += abs(ecgBuffer[i] - ecgAverage);
  }
  ecgSignal /= SAMPLES;
  return ecgSignal;
}

float calculateHeartRate(float spo2) {
  float hr = 0.0;

  if (spo2 > 90 && spo2 <= 100) {
    hr = -30.245 * spo2 + 3030.45;
  } else if (spo2 >= 80 && spo2 <= 90) {
    hr = -2.5 * spo2 + 252.5;
  } else {
    hr = -1.667 * spo2 + 166.67;
  }

  return hr;
}


void loop()
{
  
  
  uint32_t ir_led, red_led;
  float ratio, spo2;

  particleSensor.check(); // Check for new data available

  ir_led = particleSensor.getIR(); // Get IR LED intensity
  red_led = particleSensor.getRed(); // Get red LED intensity

  ratio = red_led / (float) ir_led; // Calculate ratio of red to IR LED intensity
  spo2 = 110 - 25 * ratio; // Calculate SpO2 using Beer-Lambert Law
  

  float output_temp = mlx.readObjectTempC();
  
  float output_ecg = calculateECG(AD8232_OUTPUT);
  
  float hr = calculateHeartRate(spo2);
  
  // Create a new FirebaseJson object and add the key-value pairs for ECG and temperature
  FirebaseJson json_data;
  json_data.add("ECG", output_ecg);
  json_data.add("Temperature", output_temp);
  json_data.add("SpO2", spo2);
  json_data.add("HeartRate", hr);
  // Send data to Firebase
  Firebase.pushJSON(firebaseData, FIREBASE_ROOT, json_data);
 delayMicroseconds(100);  // Pauses execution for 100 microseconds
}