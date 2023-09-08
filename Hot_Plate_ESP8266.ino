 /*
 This ESP8266 program connects to wifi,
 displays the temp from a thermistor and 
 waits for a temperature to be selected, 
 then turns on a heater to the selected temp
 */

#include "Arduino.h"
#include "math.h"
#include "PID_v1.h"
#include "ESP8266WiFi.h"

const String ssid = "xx919xx", password = "grantorino";

const int heater = 12; //pin connected to the heater trigger board pin: D6
const int therm = A0; //set the analog pin to read from      board pin: A0

const float R1 = 9990; //resistance of R1
const float TEMP_NOM = 294.15; //temperature at which B value is measured
const float THERM_NOM = 10000; //thermistor resistance at temperature TEMP_NOM
const float B = 4000; //B value of thermistor

float resistance;
float temp = 0;

#define SAMPLES 50
#define SAMPLE_DELAY 50

double Setpoint = 180, Input, Output;
double kp = 1.8, ki = 1.6, kd = 2.0;
PID myPID(&Input, &Output, &Setpoint, kp, ki, kd, DIRECT);

void setup() {
  Serial.begin(9600);
  // connectWiFi();

  pinMode(therm, INPUT);
  pinMode(heater, OUTPUT);

  myPID.SetSampleTime(SAMPLE_DELAY);
  myPID.SetMode(AUTOMATIC);

}
int serialDelay = 100, iDelay = 0;
void loop() {

 for (int i = 0; i < SAMPLES; i++) { // Collect an average of the temperature.
    temp += readTemp(); //add this sample to the total
  }
  temp /= SAMPLES; //average the samples

  Input = temp; // Calculate the PID
  myPID.Compute();

  if (Output > 255) { // Maintain output below/equal to 255 and above/equal to 0.
    Output = 255;
  } else if (Output < 0) {
    Output = 0;
  }
  analogWrite(heater, Output); // Set heater output.

  iDelay++;
  if (iDelay == serialDelay){
    Serial.print("Temperature:");
    Serial.print(Input);
    Serial.print(",");
    Serial.print("DutyCycle:");
    Serial.println(Output);
    iDelay = 0;
  }
  
}

double readTemp(void) {
  double tempB = 0;
  double val = analogRead(therm); //read the value of the thermistor
  resistance = R1 * (1023.0 / val - 1.0); //convert to resistance
  tempB = 1/(log(resistance/THERM_NOM)/B + 1/TEMP_NOM) - 273.15; //convert to kelvin
  tempB = tempB*9/5 + 32; //convert to fahrenheit
  return (tempB);
}

void connectWiFi(){ // Must have ssid and password global variables set, then call the function
  int counter = 0; // for the connectWiFi function
  WiFi.disconnect(true);
  delay(100);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoConnect(true);
  WiFi.begin(ssid, password);
  delay(100);
  while (WiFi.status() != WL_CONNECTED) {
    counter++;
    if(counter == 1){
      Serial.print("Connecting");
    }
    if (counter <= 126) {
      Serial.print(".");
    } else {
      counter = 0;
    }
    delay(10);
  }
  Serial.println("WiFi Connected");
  Serial.print("IP Address: ");
  String ipAddress = WiFi.localIP().toString();
  Serial.println(ipAddress);
  delay(2000);
}