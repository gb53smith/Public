/**
 *
 * REVISION HISTORY
 * Version 1.0 - Graham Smith
 * 
 * DESCRIPTION
 * Uses the BME280 as a temperature, humidity and pressure sensor.
 * Uses a connected RFM69HCW (915 MHz) radio to communicate to the
 * MySensors gateway node which in turn is connected via WiFi 
 * to Home Assistant acting a controller.
 * This sensor is design to be low power so that it will
 * run on batteries for at least a year.
 *
 * Current measurements
 * Measured voltage drop across a 10 ohm resistor connected to 3 AA cells.
 * 30 mA connected to the gateway
 * 14.5 mA with radio sleeping
 * < 20 uA Processor and Radio sleeping
 * BME280 measurement time 8 mS
 * Total connect time per loop 1500 mS
 * Calculated average current of 0.174 mA with 20 uA sleep current
 * Life time of 2000 mA battery is 479 days
 * 
 
*/

//#define DEBUG  // Controls print statements. Comment out for production
//#define MY_DEBUG_VERBOSE_RFM69
// Configure RFM69 Radio
#define MY_RADIO_RFM69
#define MY_RFM69_NEW_DRIVER
#define MY_IS_RFM69HW
#define MY_RFM69_FREQUENCY RFM69_915MHZ
#define MY_ENCRYPTION_SIMPLE_PASSWD "some16characters" 
#define MY_RF69_IRQ_PIN 9
#define MY_RF69_IRQ_NUM 9
#define MY_RFM69_CS_PIN A2
#define BATTERY_PIN A5

// Configure MySensors Node and its children
#define MY_NODE_ID 2 // Manually assigned node ID for this device.  AUTO not supported if using MQTT
#define BARO_CHILD 1
#define TEMP_CHILD 2
#define HUM_CHILD 3
// All defines must be before any library inclusions

#include <SPI.h>
#include <MySensors.h>  
#include <Wire.h>
#include <SparkFunBME280.h>
#include "ArduinoLowPower.h"

// Define variables to use later
bool initialValueSent = false;
unsigned long sleepTime = 300000; // Update every 5 minutes

MyMessage tempMsg(TEMP_CHILD, V_TEMP);
MyMessage humMsg(HUM_CHILD, V_HUM);
MyMessage pressureMsg(BARO_CHILD, V_PRESSURE);

BME280 mySensor; //Global sensor object

void setup()
{
  /*
  Used for RFM69 CS (NSS)
  I don't think that the MySensors.h anticipates using analog
  input as an output.
  The radio does not work without the floowing pinMode statement.
  */
  pinMode(A2, OUTPUT);  
  #ifdef DEBUG
  SerialUSB.begin(9600);
  SerialUSB.println("Example showing alternate I2C addresses");
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

void presentation()  {
  #ifdef DEBUG
  SerialUSB.println("Sending Presentation Info");
  #endif
  sendSketchInfo("MySensors_BME280", "1.0");   //Max 25 characters.  Optional to do.
  present(TEMP_CHILD, S_TEMP);
  present(HUM_CHILD, S_HUM);
  present(BARO_CHILD, S_BARO);
}
void loop()
{
  mySensor.setMode(MODE_FORCED); //Wake up sensor and take reading
  //#ifdef DEBUG
  long startTime = millis();
  //#endif
  while(mySensor.isMeasuring() == false) ; //Wait for sensor to start measurment
  while(mySensor.isMeasuring() == true) ; //Hang out while sensor completes the reading
  #ifdef DEBUG  
  long endTime = millis();

  //Sensor is now back asleep but we get get the data

  SerialUSB.print(" Measure time(ms): ");
  SerialUSB.println(endTime - startTime);
  #endif
  float temperature = mySensor.readTempC();
  float humidity = mySensor.readFloatHumidity();
  float pressure = mySensor.readFloatPressure() / 100.0;
  #ifdef DEBUG
  SerialUSB.print(" Temp: ");
  SerialUSB.print(temperature);
  SerialUSB.print(" Humidity: ");
  SerialUSB.print(humidity);
  SerialUSB.print(" Pressure: ");
  SerialUSB.print(pressure);
  #endif 
  send(tempMsg.set(temperature, 1));
  send(humMsg.set(humidity, 0));
  send(pressureMsg.set(pressure, 1));
  float battery = analogRead(BATTERY_PIN);
  //Reads 147 per volt
  // 5.00 max  =  0.136  (100/(5*147)
  battery = battery * 0.136;
  if (battery > 100.0)
  {
    battery = 100.0;
  }
  //uint8_t batteryVoltage = 93;
  #ifdef DEBUG
  SerialUSB.print(" Battery Percent: ");
  SerialUSB.println(byte(battery));
  #endif
  sendBatteryLevel(byte(battery));  
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
    wait(sleepTime); //Use this to debug radio sleep only.
  #else 
    LowPower.deepSleep(sleepTime);
    wait(10);
  #endif
  // Restoring Radio
  writeRegister(0x01, rfmOpMode);
  //SerialUSB.println(sleep(sleepTime)); //Not working from MySensors.h
}

void dummy() {
  // This function will be called once on device wakeup
  // You can do some little operations here (like changing variables which will be used in the loop)
  // Remember to avoid calling delay() and long running functions since this functions executes in interrupt context
}

//Read from RFM69HW register
byte readRegister(byte dataToSend) {
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
void writeRegister(byte thisRegister, byte thisValue) {
  // now combine the register address and the write command into one byte:
  byte dataToSend = thisRegister | 0x80;
  // take the chip select low to select the device:
  digitalWrite(MY_RFM69_CS_PIN, LOW);
  SPI.transfer(dataToSend); //Send register location
  SPI.transfer(thisValue);  //Send value to record into register
  // take the chip select high to de-select:
  digitalWrite(MY_RFM69_CS_PIN, HIGH);
}
