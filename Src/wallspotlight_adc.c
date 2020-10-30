// #include <math.h>
// #include "app.h"
#include "main.h"

extern ADC_HandleTypeDef hadc;


// Internal voltage reference, address of parameter VREFINT_CAL:
// VrefInt ADC raw data acquired at temperature 30 DegC (tolerance: +-5 DegC),
// Vref+ = 3.0 V (tolerance: +-10 mV).
#define VREFINT_CAL_ADDR           ((uint16_t*) ((uint32_t)0x1FF80078U))
// Analog voltage reference (Vref+) value with which temperature sensor
// has been calibrated in production (tolerance: +-10 mV) (unit: mV).
#define VREFINT_CAL_VREF           ((uint32_t) 3000U)

// ADC has 3 channels enabled:
// - IN1 (PA1): batt
// - IN3 (PA3): light
// - VREF
#define NUM_ADC_CHANNELS           3
// How many ADC measurements per ADC channel
#define NUM_ADC_CONVERSIONS        10
#define ADC_VALUES_SIZE            (NUM_ADC_CONVERSIONS * NUM_ADC_CHANNELS)

static int32_t  adc_values[ADC_VALUES_SIZE];
static int32_t *adc_values_end = &adc_values[ADC_VALUES_SIZE];

// Function returns average for array disregarding min and max value
// It maybe considered as p80 for array of 10 items
static int32_t get_avg_p80(int32_t *start, int32_t *end, uint32_t step)
{
  int32_t sum = 0;
  int32_t cnt = 0;
  int32_t min = INT32_MAX;
  int32_t max = INT32_MIN;

  while(start < end) {
    if (*start > max) {
      max = *start;
    }
    if (*start < min) {
      min = *start;
    }
    sum += *start;
    start += step;
    cnt++;
  }

  if (cnt >= 3) {
    return (sum - min - max) / (cnt - 2);
  }

  return sum / cnt;
}

// Function reads ADC for 2 lines: battery and light sensor.
// Returns:
// - current battery voltage in millivolts
// - light sensor value (0-4096)
//
// Since STM32 connected through LDO - it is useless just to use VREF -
// voltage will always be <= ~3.3V
// Instead, PB1 connected to battery directly via 50/50 resistor divider
void read_light_and_battery_voltage(uint32_t* mvolts, uint32_t* light)
{
  int32_t vcc = 0;
  // To make it power efficient - resistor divider connected to open
  // drain pin, so, enable it
  HAL_GPIO_WritePin(PWR_BATT_GPIO_Port, PWR_BATT_Pin, GPIO_PIN_RESET);
  // The same idea for light sensor
  HAL_GPIO_WritePin(PWR_LIGHT_GPIO_Port, PWR_LIGHT_Pin, GPIO_PIN_SET);

  // Start ADC
  HAL_ADC_Start_DMA(&hadc, (uint32_t*)adc_values, ADC_VALUES_SIZE);
  // Wait for several measurements
  HAL_Delay(100);

  // Stop ADC / disable resistor divider power
  HAL_ADC_Stop_DMA(&hadc);
  HAL_GPIO_WritePin(PWR_BATT_GPIO_Port, PWR_BATT_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(PWR_LIGHT_GPIO_Port, PWR_LIGHT_Pin, GPIO_PIN_RESET);

  // avg raw values
  // 0: light
  // 1: batt
  // 2: vref
  int32_t adc_batt = get_avg_p80(&adc_values[0], adc_values_end, 3);
  int32_t adc_light = get_avg_p80(&adc_values[1], adc_values_end, 3);
  int32_t adc_vref = get_avg_p80(&adc_values[2], adc_values_end, 3);

  int32_t vref = (VREFINT_CAL_VREF * (*VREFINT_CAL_ADDR) / adc_vref);
  // printf("vref %d\r\n", vref);
  vcc = (float)vref / 4096.0 * (float)adc_batt * 2.0;
  if (mvolts) {
    *mvolts = vcc;
  }
  if (light) {
    *light = adc_light;
  }
}
