/*
   REVISION HISTORY
   Version 1.0 - Graham Smith
   Version 1.1 - Changed V_VAR1 to V_CUSTOM
   Version 1.2 - Correct battery voltage reading to use 12 bit ADC

   DESCRIPTION
   Uses the BME280 as a temperature, humidity and pressure sensor.
   Uses a connected RFM69HCW (915 MHz) radio to communicate to the
   MySensors gateway node which in turn is connected via WiFi
   to Home Assistant acting a controller.
   The baterry voltage is sent to HA to allow detection and warning of
   low battery voltage condition.
   Only significant changes in each of the four measurements
   are sent as messages to the gateway.  This saves power.
   This sensor is designed to be low power so that it will
   run on batteries for at least a year.
   The Moteino M0 board does an excellent job of reducing sleep
   currents. Several improvements have been made compared to
   competitors.

   CURRENT MEASUREMENTS
   Measured voltage drop across a 10 ohm resistor connected
   in series to 3 AA cells.
   30 mA when connected to the gateway
   14.8 mA with radio sleeping
   < 20 uA with Processor and Radio sleeping
   BME280 measurement time 8 mS
   Total connect time per loop 1756 mS if all 4 sensor values change.
   When all change 29.6 mA is measured a 1 M antenna separation, 30.6 mA
   a about 20 M antenna separation.
   Total 14 mS CPU active time when no signficant change in sensor value.
   and no radio transmissions.
   Calculated average current caculation:
   Worst case with all sensors sending = (.02 * 300 + 30 * 1.8 )/ 301.8 = 0.199 mA
   Typical with one sensor change = (.02 * 300 + 30 * 0.45 )/ 300.45 = 0.065 mA
   Life time of 2000 mA battery is 419 days worst case, 3.5 years typical.
   A Li-Poly battery will probably self-discharge first!
*/

#define DEBUG  // Controls print statements. Comment out for production
//#define MY_DEBUG_VERBOSE_RFM69

// Configure RFM69 Radio
#define MY_RADIO_RFM69
#define MY_RFM69_NEW_DRIVER
#define MY_IS_RFM69HW
#define MY_RFM69_FREQUENCY RFM69_915MHZ
#define MY_ENCRYPTION_SIMPLE_PASSWD "some16characters"

// Configure hardware pins
#define MY_RF69_IRQ_PIN 9
#define MY_RF69_IRQ_NUM 9
#define MY_RFM69_CS_PIN A2
#define BATTERY_PIN A5
#define LED_PIN 13

// Configure MySensors Node and its children
#define MY_NODE_ID 2 // Manually assigned node ID for this device.  AUTO not supported if using MQTT
#define BARO_CHILD 1
#define TEMP_CHILD 2
#define HUM_CHILD 3
#define VOLT_CHILD 4

// All defines must be before any library inclusions

#include <SPI.h>
#include <MySensors.h>
#include <Wire.h>
#include <SparkFunBME280.h>
#include "ArduinoLowPower.h"

// Define variables to use later
bool initialValueSent = false;
volatile bool awakeFlag = false; // Start awake
unsigned long sleepTime = 300000; // Update every 5 minutes
float lastTemperature = 0;
float lastPressure = 0;
float lastHumidity = 0;
float lastBattery = 0;

// Only use one variable type per child
// Customize HA to display icon and units
MyMessage tempMsg(TEMP_CHILD, V_CUSTOM);
MyMessage humMsg(HUM_CHILD, V_CUSTOM);
MyMessage pressureMsg(BARO_CHILD, V_CUSTOM);
MyMessage voltageMsg(VOLT_CHILD, V_CUSTOM);

BME280 mySensor; //Global sensor object

void setup()
{
  /*
    Used for RFM69 CS (NSS)
    I don't think that the MySensors.h anticipates using analog
    input as an output.
    The radio does not work without the following pinMode statement.
  */
  pinMode(A2, OUTPUT);  //Used for RFM69 CS (NSS)
  //Setup ADC. Arduino default is 10 bits
  analogReadResolution(12);
  LowPower.attachInterruptWakeup(RTC_ALARM_WAKEUP, awake, CHANGE);
#ifdef DEBUG
  SerialUSB.begin(9600);
  SerialUSB.println("In setup function.");
#endif
  Wire.begin();
  Wire.setClock(400000); //Increase to fast I2C speed!

  mySensor.beginI2C();
  // Use same settings as Micropython version
  mySensor.settings.filter = 3;
  mySensor.settings.tempOverSample = 8;
  mySensor.settings.pressOverSample = 4;
  mySensor.settings.humidOverSample = 2;

  mySensor.setMode(MODE_SLEEP); //Sleep for now
}

void presentation()
{
#ifdef DEBUG
  SerialUSB.println("Sending Presentation Info");
#endif
  sendSketchInfo("MySensors_BME280", "1.0");   //Max 25 characters.  Optional to do.
  // Use one child per measurement for consistent HA entity naming.
  present(TEMP_CHILD, S_CUSTOM);
  present(HUM_CHILD, S_CUSTOM);
  present(BARO_CHILD, S_CUSTOM);
  present(VOLT_CHILD, S_CUSTOM);
}
void loop()
{
  mySensor.setMode(MODE_FORCED); //Wake up sensor and take reading
  //#ifdef DEBUG
  long startTime = millis();
  //#endif
  while (mySensor.isMeasuring() == false) ; //Wait for sensor to start measurment
  while (mySensor.isMeasuring() == true) ; //Hang out while sensor completes the reading
#ifdef DEBUG
  long endTime = millis();

  //Sensor is now back asleep but we get get the data

  SerialUSB.print(" Measure time(ms): ");
  SerialUSB.println(endTime - startTime);
#endif
  float temperature = mySensor.readTempC();
  float humidity = mySensor.readFloatHumidity();
  float pressure = mySensor.readFloatPressure() / 100.0;
  float battery = analogRead(BATTERY_PIN);
  //Reads approx 620 per volt, 4095/3.3/2
  battery = battery / 625.0; //My calibration
#ifdef DEBUG
  SerialUSB.print(" Temp: ");
  SerialUSB.print(temperature);
  SerialUSB.print(" Humidity: ");
  SerialUSB.print(humidity);
  SerialUSB.print(" Pressure: ");
  SerialUSB.print(pressure);
  SerialUSB.print(" Battery Voltage ");
  SerialUSB.println(battery);
#endif
  // To Save power only send message on changed values
  if (abs(temperature - lastTemperature) > 0.1)
  {
    send(tempMsg.set(temperature, 1));
    lastTemperature = temperature;
  }
  if (abs(humidity - lastHumidity) > 1.0)
  {
    send(humMsg.set(humidity, 0));
    lastHumidity = humidity;
  }

  if (abs(pressure - lastPressure) > 1.0)
  {
    send(pressureMsg.set(pressure, 1));
    lastPressure = pressure;
  }

  if (abs(battery - lastBattery) > 0.1)
  {
    send(voltageMsg.set(battery, 2));
    lastBattery = battery;
  }

  // sendBatteryLevel(byte(battery)); Not using since percentage is not as meaningful
#ifdef DEBUG
  endTime = millis();
  SerialUSB.print(" Total time(ms): ");
  SerialUSB.println(endTime - startTime);
#endif

  //Read current mode of radio
  byte rfmOpMode = readRegister(0x01);

  // Sleeping Radio
  writeRegister(0x01, 0x00);
  // Arduino Sleep
  
#ifdef DEBUG
  LowPower.deepSleep(10000);
#else
  LowPower.deepSleep(sleepTime);
#endif
  wait(10);
  // Restoring Radio
  writeRegister(0x01, rfmOpMode);
  
#ifdef DEBUG  
  if (awakeFlag) // Flash LED twice when waking up from sleep.
  {
    if (transportCheckUplink())
    {
      blink();
    }

    awakeFlag = false;

  }
#endif
  //SerialUSB.println(sleep(sleepTime)); //Not working from MySensors.h
}

void awake()
{
  awakeFlag = true;
  // This function will be called once on device wakeup
  // You can do some little operations here (like changing variables which will be used in the loop)
  // Remember to avoid calling delay() and long running functions since this functions executes in interrupt context
}

//Read from RFM69HW register
byte readRegister(byte dataToSend)
{
  byte result;   // result to return
  // take the chip select low to select the device:
  digitalWrite(MY_RFM69_CS_PIN, LOW);
  // send the device the register you want to read:
  SPI.transfer(dataToSend);
  // send a value of 0 to read the first byte returned:
  result = SPI.transfer(0x00);
  // take the chip select high to de-select:
  digitalWrite(MY_RFM69_CS_PIN, HIGH);
  // return the result:
  return (result);
}


//Sends a write command to RFM69HW
void writeRegister(byte thisRegister, byte thisValue)
{
  // now combine the register address and the write command into one byte:
  byte dataToSend = thisRegister | 0x80;
  // take the chip select low to select the device:
  digitalWrite(MY_RFM69_CS_PIN, LOW);
  SPI.transfer(dataToSend); //Send register location
  SPI.transfer(thisValue);  //Send value to record into register
  // take the chip select high to de-select:
  digitalWrite(MY_RFM69_CS_PIN, HIGH);
}

void blink()
{
  digitalWrite(LED_PIN, HIGH);
  wait(100);
  digitalWrite(LED_PIN, LOW);
  wait(1000);
  digitalWrite(LED_PIN, HIGH);
  wait(100);
  digitalWrite(LED_PIN, LOW);
}
