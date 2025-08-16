// REQUIRED YOU MUST DOWNLOAD ONEWIRE AND DALLASTEMPERATURE LIBRARIES
// from the Arduino Library Manager or GitHub
#include <OneWire.h>
#include <DallasTemperature.h>
// #include <MLX90614ESF.h>
#include <Adafruit_MLX90614.h>
#include <PID_v1.h>

// Data wire is plugged into digital pin 2 on the Arduino
#define ONE_WIRE_BUS 2

// Setup a oneWire instance to communicate with any OneWire device
OneWire oneWire(ONE_WIRE_BUS);

// Pass oneWire reference to DallasTemperature library
DallasTemperature sensors(&oneWire);

Adafruit_MLX90614 mlx = Adafruit_MLX90614();

// Define PID variables
double Setpoint, Input, Output;
double Kp = 2, Ki = 5, Kd = 30; // Tune these values
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Define pin for controlling the heater/cooler
const int outputPin = A0; // PWM pin

void setupPID(double targetTemp)
{
  Setpoint = targetTemp;
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255); // Adjust limits based on your setup
  myPID.SetSampleTime(1000);     // Adjust sample time in milliseconds
 // myPID.begin();
}

void runPID(double currentTemp)
{
  Input = currentTemp;
  myPID.Compute();
  analogWrite(outputPin, Output); // Control the heater/cooler
}

double getThermistorTemp()
{
  return sensors.getTempCByIndex(0);
}

double getIRTemp()
{

  return mlx.readObjectTempC();
}

void setup(void)
{
  Serial.begin(9600);
  sensors.begin();
  setupPID(10);
  if (!mlx.begin())
  {
    Serial.println("Error connecting to MLX sensor. Check wiring.");
    while (1);
  }
}

void loop(void)
{
  // Send the command to get temperatures

  sensors.requestTemperatures();

  if (Serial.available() > 0)
  {
    char receivedChar = Serial.read(); // Read the character (but you don't need to use it)
    while (1)
    {
      // Send the command to get temperatures

      sensors.requestTemperatures();

      double currentTemp = getIRTemp();
      runPID(currentTemp); // Run PID control
      Serial.print("Temperature: ");
      Serial.print(getIRTemp());
      Serial.print("°C  |  ");

      Serial.print("Temperature: ");
      Serial.print(getIRTemp() + 273.15);
      Serial.print("°K  |  ");

      // print the temperature in Fahrenheit
      Serial.print((getIRTemp() * 9.0) / 5.0 + 32.0);
      Serial.println("°F");
    }
  }

  /*UNCOMMENT CODE BELOW WHEN USING THERMISTOR*/
  // print the temperature in Celsius
  /*Serial.print("Temperature: ");
  Serial.print( getThermistorTemp());
  Serial.print("°C  |  ");

  Serial.print("Temperature: ");
  Serial.print( getThermistorTemp() + 273.15);
  Serial.print("°K  |  ");

  //print the temperature in Fahrenheit
  Serial.print(( getThermistorTemp() * 9.0) / 5.0 + 32.0);
  Serial.println("°F");*/

  delay(500);
}