#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H

#include <stdint.h> // Include for uint32_t

typedef struct {
  int16_t temperature;
  int16_t humidity;
  int16_t smoke;
  uint32_t timestamp; // Timestamp for data reading
} SensorData;

#endif // SENSOR_DATA_H