/* #include "esphome.h"
#include "DHTNew.h"


class MyDHT : public PollingComponent, public Sensor {
 public:
  Sensor *temperature_sensor = new Sensor();
  Sensor *humidity_sensor = new Sensor();
  // constructor.  Update interval set here to 10s
  MyDHT() : PollingComponent(10000) {}

  float get_setup_priority() const override { return esphome::setup_priority::DATA; }
  // Pin and model type set here
  DHT dht_dht{2, DHT_MODEL_DHT22};
  void setup() override {
    // This will be called by App.setup()
   
  }
  void update() override {
    // This will be called every "update_interval" milliseconds.
    float temperature = dht_dht.readTemperature();
    ESP_LOGD("custom", "The value of my temperature is: %f", temperature);
    temperature_sensor->publish_state(temperature);
    float humidity = dht_dht.readHumidity();
    ESP_LOGD("custom", "The value of my humidity is: %f", humidity);
    humidity_sensor->publish_state(humidity);
  }
};
 */

#include "esphome.h"
// DHTModel_t
typedef enum {
    DHT_MODEL_DHT11 = 11,
    DHT_MODEL_DHT22 = 22,
    DHT_MODEL_DHT21 = 21,
    DHT_MODEL_AM2302 = DHT_MODEL_DHT22, // Packaged DHT22
    DHT_MODEL_AM2301 = DHT_MODEL_DHT21, // Packaged DHT21
    DHT_MODEL_RHT03 = DHT_MODEL_DHT22  // Equivalent to DHT22
} DHTModel_t;

// DHTError_t
typedef enum {
    DHT_ERROR_NONE = 0,
    DHT_ERROR_TIMEOUT_START,
    DHT_ERROR_TIMEOUT_DATA,
    DHT_ERROR_CHECKSUM
} DHTError_t;

class MyDHT : public PollingComponent, public Sensor {
 public:
  Sensor *temperature_sensor = new Sensor();
  Sensor *humidity_sensor = new Sensor();
  // constructor.  Update interval set here to 10s
  MyDHT() : PollingComponent(10000) {}

  float get_setup_priority() const override { return esphome::setup_priority::DATA; }

// Returns true if a reading attempt was made (successful or not)
// Copied from diaoul/DHTNew @ 1.0.0 on PlatformIO.  
// All Esphome components should identify and credit the source with a version number too!
bool read(bool force) {
    // don't read more than every getMinimumSamplingPeriod() milliseconds
    int getMinimumSamplingPeriod = 2000;
    
    unsigned long currentTime = millis();
    if (!force && ((currentTime - _lastReadTime) < getMinimumSamplingPeriod)) {
        return false;
    }

    // reset lastReadTime, temperature and humidity
    _lastReadTime = currentTime;
    _temperature = NAN;
    _humidity = NAN;

    // send start signal
    pinMode(_pin, OUTPUT);
    digitalWrite(_pin, LOW);
    if (_model == DHT_MODEL_DHT11)
        delay(18); // [18-20]ms
    else
        // Increased from 800 us to use the datasheet recommended value.  Probably not necessary but an extra 200 us won't hurt.  
        delayMicroseconds(1000); // [0.8-20]ms 
    // init data
    uint8_t data[5] = {0};

    // begin of time critical code
    noInterrupts();

    // start reading the data line
    pinMode(_pin, INPUT_PULLUP);
    delayMicroseconds(20);  // [20-200]us

    // 80us low time + 80us high time + 10us error margin
    if (!pulseIn(_pin, HIGH, 170)) {
        _error = DHT_ERROR_TIMEOUT_START;
        interrupts();
        return true;
    }

    // read the 40 bits (5 bytes) of data
    for (uint8_t i = 0; i < sizeof(data) * 8; i++) {
        // 50us low time + [26-70]us high time + 10us error margin
        uint8_t pulse = pulseIn(_pin, HIGH, 150);

        if (!pulse) {
            _error = DHT_ERROR_TIMEOUT_DATA;
            interrupts();
            return true;
        }

        data[i/8] <<= 1;
        if (pulse > 50)  // this is a 1
            data[i/8] |= 1;
    }
    // end of time critical code
    interrupts();

    // verify checksum
    if (data[4] != ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) {
        _error = DHT_ERROR_CHECKSUM;
        return true;
    }

    // we made it
    _error = DHT_ERROR_NONE;

    // convert data to actual temperature and humidity
    switch (_model) {
    case DHT_MODEL_DHT11:
        _temperature = data[2];
        _humidity = data[0];
        break;
    case DHT_MODEL_DHT22:
    case DHT_MODEL_DHT21:
        _temperature = data[2] & 0x7F;
        _temperature *= 256;
        _temperature += data[3];
        _temperature *= 0.1;
        if (data[2] & 0x80) {
            _temperature *= -1;
        }
        _humidity = data[0];
        _humidity *= 256;
        _humidity += data[1];
        _humidity *= 0.1;
        break;
    }

    return true;
}

  void setup() override {
    // This will be called by App.setup()
   
  }
  void update() override {
    // This will be called every "update_interval" milliseconds.
    this->read(false);
    ESP_LOGD("custom", "The value of my temperature is: %f", _temperature);
    temperature_sensor->publish_state(_temperature);
    ESP_LOGD("custom", "The value of my humidity is: %f", _humidity);
    humidity_sensor->publish_state(_humidity);
    ESP_LOGD("custom", "The error code is: %d", _error);
  }
  
    protected:
    uint8_t _pin = 2;  // Hard code GPIO pin
    float _temperature;
    float _humidity;

    private:
    DHTModel_t _model = DHT_MODEL_DHT22; // Hard code DHT22
    DHTError_t _error;
    unsigned long _lastReadTime;
};