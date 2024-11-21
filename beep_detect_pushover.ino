#include "arduino_secrets.h"
#include <PDM.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <NTPClient.h>

unsigned long currentTime; // Time in microseconds since the program started

// ---------- Wifi setting ------------------------------------------
WiFiClient wifiClient;
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_OPTIONAL_PASS;    // your network password (use for WPA, or use as key for WEP)
int status = WL_IDLE_STATUS;     // the Wifi radio's status

// ---------- NPT and time update setting ---------------------------
WiFiUDP ntpUDP;
// NTPClient timeClient(ntpUDP);
NTPClient timeClient(ntpUDP, "pool.ntp.org", -28800, 60000);

const int nptUpdateFrequency = 60*60*1000; // 1 hour
int clockHour = 0;

// ---------- Audio FFT setting -------------------------------------
const int samples = 256; // Number of samples for the FFT - 64, 128, 256, or 512
const int samplingFrequency = 14000; // Frequency range plotted is 2.5kHz (sampling frequency/4)
// 256 samples at 14kHz corresponds to 18ms

// Audio sample buffer
short sampleBuffer[64];
short dumpBuffer[64];

// Audio sample buffers used for analysis and display
short streamBuffer[samples]; // Sample stream buffer (N samples)
int approxBuffer[samples];   // ApproxFFT sample buffer

volatile int samplesRead = 0; // Samples read from the microphone (expected to be 64)
volatile int sampleCount = 0; // Samples put into streamBuffer

void onPDMdata(void);  // Callback to get the audio samples

int frequencyOfInterest = 2050; // Extract amplitude at a specific frequency (2050Hz)
int amplitudeAtFrequency = 0;
int indexFreqSample = frequencyOfInterest * samples / samplingFrequency;

// ---------- Variables for max peak volume calc --------------------
unsigned long startTime = 0;
const unsigned long extractionInterval = 5000000; // 5 seconds in microseconds
int maxAmplitudeDuringInterval = 0;

// ---------- Variables for alarm duration calc ---------------------
int amplitudeThreshold = 5000;
int limitTime = 1000000; // Silence time to determine the end of the sound (in microseconds)

int high_total_time_output;
int total_time_output;
int volume_at_freq;

// ---------- Pushover Notify setting -----------------------------------
const char* pushoverAPI = "api.pushover.net";
// const char* tokenLine = SECRET_OPTIONAL_LINETOKEN;
const char* userKey = SECRET_PUSHOVER_USER_KEY;         // Your Pushover User Key
const char* apiToken = SECRET_PUSHOVER_API_KEY;       // Your Pushover API Token
const char* messageTitle = "Bread is ready!";


void setup() {
  // Initialize serial and wait for port to open:
  Serial.begin(9600);
  // This delay gives the chance to wait for a Serial Monitor without blocking if none is found
  delay(1500);

  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    while (true);
  }
  
  Serial.print("Attempting to connect to WPA SSID: ");
  Serial.println(ssid);
  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED) {}

  // GMT-8 (US Pacific Standard time) -> -28800
  // NTP update every 10 min
  timeClient.begin();
  // timeClient.setTimeOffset(-28800);
  // timeClient.setUpdateInterval(600000);
  timeClient.update();


  // Initialize the PDM library
  PDM.onReceive(onPDMdata);
  PDM.setGain(20);
  // Setup the I2S audio input for the sample rate with 32-bits per sample
  if (!PDM.begin(1, samplingFrequency)) {
    Serial.println("Failed to start PDM!");
    while (1);
  }
}



void loop() {  
  // ---------- FFT on audio data from PDM --------------------------
  // Only 64 PDM audio samples are available at a time so accumulate 256 for the FFT
  if (samplesRead) {
    // Fill the buffers with the samples
    for (int i = 0; i < samplesRead; i++) {
      streamBuffer[i + sampleCount] = sampleBuffer[i];
      approxBuffer[i + sampleCount] = sampleBuffer[i] / 2;
    }
    sampleCount += samplesRead;
    samplesRead = 0;
  }

  // If we have all new samples then run the FFT.
  if (sampleCount >= samples) {
    // Do the FFT number crunching
    // approxBuffer contains samples, but will be updated with amplitudes
    Approx_FFT(approxBuffer, samples, samplingFrequency);
    amplitudeAtFrequency = approxBuffer[indexFreqSample];
    // Serial.print("Amplitude at 2050Hz: ");
    // Serial.println(amplitudeAtFrequency);

    sampleCount = 0; // Reset audio sample count

    // ---------- Extract the max amplitude -------------------------
    if (amplitudeAtFrequency > maxAmplitudeDuringInterval) {
      maxAmplitudeDuringInterval = amplitudeAtFrequency;
    }

    // Check if 5 seconds have elapsed
    if (micros() - startTime >= extractionInterval) {
      // Perform the extraction logic here
      Serial.print("Maximum amplitude during the last 5 seconds: ");
      Serial.println(maxAmplitudeDuringInterval);

      volume_at_freq = maxAmplitudeDuringInterval;

      // Reset variables for the next interval
      maxAmplitudeDuringInterval = 0;
      startTime = micros(); // Reset the timer
    }

    // ---------- Extract the duration of the signal ----------------
    processContSignal();
  }
}

void onPDMdata() {
  // This fetches a maximum of 64 samples
  int bytesAvailable = PDM.available();
  PDM.read(sampleBuffer, bytesAvailable);
  samplesRead = bytesAvailable / 2;
}


// ---------- Process continuous signal duration --------------------
int totalTime = 0; // Total time from the start to the end of a single sound
int highTotalTime = 0; // Total time the sound signal > amplitudeThreshold
int lowTime; // Time the sound sensor signal was LOW
int firstRisePoint; // Time of the moment when the signal went HIGH (recorded only once)
int risePoint; // Time of the moment when the signal went HIGH
int fallPoint; // Time of the moment when the signal went LOW
bool isPrevSignalHigh = false;
int countSignalHigh = 0; // Number of times the signal went HIGH

void processContSignal() {
  currentTime = micros();

  if (amplitudeAtFrequency > amplitudeThreshold) {
    if (!isPrevSignalHigh) { // Moment when the signal went HIGH (rising edge)
      isPrevSignalHigh = true;
      risePoint = currentTime;
      countSignalHigh++;
      firstRisePoint = (countSignalHigh == 1) ? currentTime : firstRisePoint;
      
      Serial.println("Rising edge");
    }
  } else {
    if (isPrevSignalHigh) { // Moment when the signal went LOW (falling edge)
      isPrevSignalHigh = false;
      fallPoint = currentTime;
      highTotalTime += currentTime - risePoint; 
      Serial.print("Falling edge: current highTotalTime of ");
      Serial.print(highTotalTime/1000);
      Serial.println(" ms");
    } else {
      lowTime = currentTime - fallPoint;

      // Serial.println(lowTime);
      // Send alert after a break of sound signal
      if (countSignalHigh > 0 && lowTime > limitTime) {
        totalTime = currentTime - firstRisePoint - lowTime;
        Serial.println("End of alert:");
        Serial.print("current highTotalTime of ");
        Serial.print(highTotalTime/1000);
        Serial.println(" ms");
        Serial.print("current totalTime of ");
        Serial.print(totalTime/1000);
        Serial.println(" ms");

        if (totalTime > 300000) {
          Serial.print(timeClient.getFormattedTime());
          Serial.print(" - ");
          Serial.print(totalTime / 1000);
          Serial.print("[");
          Serial.print(highTotalTime / 1000);
          Serial.println("]");

          total_time_output = totalTime / 1000;
          high_total_time_output = highTotalTime / 1000;

          clockHour = timeClient.getHours();

          if (totalTime > 7000000 && totalTime < 800000 && highTotalTime > 2000000 && highTotalTime > 3000000) {
            sendNotification();
          }
        }

        // if (total_time > 3000000 && total_time < 4000000) {
        //   if (high_total_time > 350000 && high_total_time < 600000) {
        //     Serial.println(" HIT!");
        //     sendNotification();
        //   }
        // }

        Serial.println("");
        totalTime = 0;
        highTotalTime = 0;
        countSignalHigh = 0;
      }
    }
  }
}


// ---------- Notify on Line ----------------------------------------
bool sendNotification() {
  Serial.println("Starting notification process...");

  // Attempt to connect to the server with SSL
  if (!wifiClient.connectSSL("api.pushover.net", 443)) {
    Serial.println("Failed to connect to Pushover API server!");
    return false;
  }
  
  Serial.println("Connected to Pushover API server.");

  // Construct the HTTP POST payload
  String postData = "token=" + String(apiToken) +
                    "&user=" + String(userKey) +
                    "&title=" + String(messageTitle) +
                    "&message=" + String(timeClient.getFormattedTime());
  
  // Construct and send the HTTP POST request
  wifiClient.println("POST /1/messages.json HTTP/1.1");
  wifiClient.println("Host: api.pushover.net");
  wifiClient.println("Connection: close");
  wifiClient.println("Content-Type: application/x-www-form-urlencoded");
  wifiClient.print("Content-Length: ");
  wifiClient.println(postData.length());
  wifiClient.println();
  wifiClient.print(postData);

  Serial.println("POST request sent. Waiting for response...");

  // Wait for the response from the server
  unsigned long timeout = millis();
  while (wifiClient.connected() && !wifiClient.available()) {
    if (millis() - timeout > 5000) { // 5-second timeout
      Serial.println("Timeout waiting for server response!");
      wifiClient.stop();
      return false;
    }
  }

  // Read the response from the server
  String response = "";
  while (wifiClient.available()) {
    response += wifiClient.readStringUntil('\n');
  }

  // Debugging: Print the response
  Serial.println("Response received:");
  Serial.println(response);

  // Close the connection
  wifiClient.stop();

  // Check for success in the response
  if (response.indexOf("\"status\":1") > -1) {
    Serial.println("Notification sent successfully!");
    return true;
  } else {
    Serial.println("Failed to send notification. Check the API response.");
    return false;
  }
}


