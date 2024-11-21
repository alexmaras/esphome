#include "jhm1200.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace jhm1200 {

static const char *const TAG = "jhm1200";

static const uint8_t JHM1200_START_MEASUREMENT = 0xAC;

static const uint32_t JHM1200_PRESSURE_MAX = 0xffffff;
static const float JHM1200_TEMPERATURE_MINIMUM = -20.0; // configurable
static const float JHM1200_TEMPERATURE_MAXIMUM = 60.0; // configurable

void JHM1200Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up JHM1200...");
}

float JHM1200Component::get_setup_priority() const { return setup_priority::DATA; }

void JHM1200Component::dump_config() {
  ESP_LOGCONFIG(TAG, "JHM1200:");
  LOG_I2C_DEVICE(this);

  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with JHM1200 failed!");
    return;
  }
  LOG_UPDATE_INTERVAL(this);

  LOG_SENSOR("  ", "Pressure", this->pressure_sensor_);
  LOG_SENSOR("  ", "Temperature", this->temperature_sensor_);
}


void JHM1200Component::update() {
  ESP_LOGD(TAG, "Asking sensor to start measurement");
  int res = this->write(&JHM1200_START_MEASUREMENT, 1);
  if (res != i2c::ERROR_OK) {
    ESP_LOGW(TAG, "Error while asking for measurement to start: %d", res);
    this->status_set_warning();
    return;
  }

  ESP_LOGD(TAG, "Waiting for sensor to complete measurement");
  auto f = std::bind(&JHM1200Component::wait_for_response_, this, 1);
  this->set_timeout("read_values", 100, f);
  ESP_LOGD(TAG, "end of function");
}

void JHM1200Component::wait_for_response_(uint8_t attempt_number) {
    ESP_LOGD(TAG, "Asking for measurement result");
    uint8_t i2c_response[6];
    i2c::ErrorCode res = this->read(i2c_response, 6);
    if (res != i2c::ERROR_OK) {
      ESP_LOGW(TAG, "Error while reading measurement result: %d", res);
      this->status_set_warning();
    }

    uint8_t status_byte = i2c_response[0];
    ESP_LOGD(TAG, "Status byte: %02X", status_byte);
    if ((status_byte & 0x20) == 0x20) {
      if (attempt_number < 3) {
        ESP_LOGD(TAG, "Measurement result not yet ready, waiting again");
        auto f = std::bind(&JHM1200Component::wait_for_response_, this, attempt_number + 1);
        this->set_timeout("read_values", 100, f);
      } else {
        ESP_LOGD(TAG, "Measurement result not yet within %d attempts, giving up", attempt_number);
        this->status_set_warning();
      }
      return;
    }

    ESP_LOGD(TAG, "Bytes: %02X %02X %02X %02X %02X %02X", i2c_response[0], i2c_response[1], i2c_response[2], i2c_response[3], i2c_response[4], i2c_response[5]);

    uint32_t pressure_raw = encode_uint24(i2c_response[1], i2c_response[2], i2c_response[3]);
    uint16_t temperature_raw = encode_uint16(i2c_response[4], i2c_response[5]);

    ESP_LOGD(TAG, "Raw numbers, pressure: %d, temp: %d", pressure_raw, temperature_raw);

    // digital value has a 10% to 90% range - i.e. the minimum value read should be ~ 16777216 * 0.1f
    float pressure_digital_minimum = 16777216 * 0.1f;
    float pressure_digital_maximum = 16777216 * 0.9f;
    float total_pressure_points = pressure_digital_maximum - pressure_digital_minimum;
    float max_bar = 10; // configurable
    float max_hpa = max_bar * 1000;
    float pressure_hpa = max_hpa / total_pressure_points * ((float)pressure_raw - pressure_digital_minimum);

    // convert temperature to a scale between min temp and max temp
    float total_temp_points = JHM1200_TEMPERATURE_MAXIMUM - JHM1200_TEMPERATURE_MINIMUM; // configurable
    
    float temperature_c = ((float)temperature_raw / 65536) * total_temp_points + JHM1200_TEMPERATURE_MINIMUM;

    ESP_LOGD(TAG, "Got pressure=%.2fhPa", pressure_hpa);
    ESP_LOGD(TAG, "Got temperature=%.2f°C", temperature_c);

    this->pressure_sensor_->publish_state(pressure_hpa);
    this->temperature_sensor_->publish_state(temperature_c);

    this->status_clear_warning();
}

}  // namespace jhm1200
}  // namespace esphome
