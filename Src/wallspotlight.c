#include <stdlib.h>
#include <stdio.h>

#include "main.h"
#include "wallspotlight_adc.h"

// When to turn on all LEDs (6) - meaning it is not too dark
// and more light needed to make it beautiful :)
#define LIGHT_THRESHOLD_OFF           350
#define LIGHT_THRESHOLD_6LED_ON       310
// Second threshold: when it is too dark - so to turn on only 2 LEDs
// to not make people blind :)
#define LIGHT_THRESHOLD_4LED_ON       100

// Radar Motion Detector:
// seconds between state change
#define MOTION_STATE_THRESHOLD        90

// Battery voltage ranges, in mVolts
#define BATTERY_MIN                   3400
#define BATTERY_MAX                   4210
#define BATTERY_ONE_PERCENT           ((BATTERY_MAX - BATTERY_MIN) / 100.0)
#define BATTERY_FIVE_PERCENTS         (BATTERY_ONE_PERCENT * 5 + BATTERY_MIN)
#define BATTERY_TEN_PERCENTS          (BATTERY_ONE_PERCENT * 10 + BATTERY_MIN)


// Radar based motion detection state machine
#define MOTION_STATES(s)  \
  s(MOTION_OFF),          \
  s(MOTION_DETECTING),    \
  s(MOTION_DETECTED),     \
  s(MOTION_CLEARING),     \

#define DEFINE_ENUM(s)  s
enum {
  MOTION_STATES(DEFINE_ENUM)
};
#undef DEFINE_ENUM

#define DEFINE_STATE_NAMES(s) #s
const char *motion_status_names[] = {
  MOTION_STATES(DEFINE_STATE_NAMES)
};
#undef DEFINE_STATE_NAMES

#define MOTION_PRESENT(v) ( ((v == MOTION_DETECTED) || (v == MOTION_CLEARING)) ? 1 : 0)


// Forward declaration of helpers / utils
static uint32_t battery_voltage_to_percent(uint32_t mvolts);
static void     read_switch_settings(uint32_t *radar, uint32_t *light);
static void     play_charging_animation();
static void     play_low_battery_animation();
static void     turn_radar_on();
static void     turn_radar_off();
static void     turn_spotlight_on(uint32_t leds);
static void     turn_spotlight_off();

// ISR variables
volatile uint32_t button_pressed        = 0;
volatile uint32_t charger_connected     = 0;
volatile uint32_t charging_completed    = 0;
volatile uint32_t motion_detected_isr   = 0;
volatile uint32_t wakeup_triggered      = 0;
volatile uint32_t skip_first_motion_isr = 0;

// These defined in main.c, bring them here
extern RTC_HandleTypeDef hrtc;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern void SystemClock_Config(void);

// static void play_charging_animation(struct open_iot* iot);
// static void play_low_battery_animation();


void wallspotlight_app()
{
  printf("Wall Spotlight v1.1\r\n");
  HAL_Delay(1000);

  // Read initial charger state
  charger_connected = !HAL_GPIO_ReadPin(BATT_CHRG_GPIO_Port, BATT_CHRG_Pin);
  charging_completed = !HAL_GPIO_ReadPin(BATT_STDBY_GPIO_Port, BATT_STDBY_Pin);

  // Battery handling
  uint32_t battery_depleted  = 0;
  uint32_t battery_low = 0;

  // Use WakeUp timer to enable / disable motion radar
  // Once motion detected turn off radar and wait N seconds
  // After 300 secs consider motion as cleared.
  uint32_t motion_state = MOTION_OFF;

  // charger_connected = 1;
  while(1) {
    // Read on board mini switch settings
    uint32_t sw_radar_on;
    uint32_t sw_light_on;
    read_switch_settings(&sw_radar_on, &sw_light_on);
    // Read battery voltage / light sensor value
    // (they both utilizing ADC so do it alltogether)
    uint32_t mvolts;
    uint32_t light;
    read_light_and_battery_voltage(&mvolts, &light);

    // If battery reaches minumum at least once - block
    // all functionality until charger connected
    if (mvolts < BATTERY_MIN) {
      battery_depleted = 1;
    }
    if (mvolts < BATTERY_TEN_PERCENTS) {
      battery_low = 1;
    }

    // When device is not operational (battery, charging) -
    // turn off everything
    if (battery_depleted || charger_connected) {
      motion_state = MOTION_OFF;
      turn_radar_off();
      turn_spotlight_off();
    }

    // Battery depleted. Save power and just sleep until connected for charge.
    if (battery_depleted && !charger_connected) {
      printf("Battery depleted, sleep until charger connected\r\n");
      goto sleep;
    }

    // If charger connected:
    // - Show current progress
    // - Turn off spotlight
    // - block until charger disconnected
    if (charger_connected || charging_completed) {
      printf("Charger connected!\r\n");
      play_charging_animation();
      // Assuming that battery got some charge
      battery_depleted = 0;
      battery_low = 0;
    }

    printf("Battery %ld%%, light %ld\r\n", battery_voltage_to_percent(mvolts), light);
    printf("%s wakeup timer %ld, motion_isr %ld\r\n",
        motion_status_names[motion_state], wakeup_triggered, motion_detected_isr);

    // When radar is not enabled - turn off power / clear state
    // (in order to not to duplicate this in all functions)
    if (!sw_radar_on) {
      turn_radar_off();
      motion_state = MOTION_OFF;
    }

    // RCWL-0516 Doppler Radar has one bug(?) - it always
    // generates interrupt when power on regardless of actual motion
    if (motion_detected_isr && skip_first_motion_isr) {
      motion_detected_isr = 0;
      skip_first_motion_isr = 0;
    }

    // Actual spotlight routine //

    // - All off: just sleep for minute then re-check settings
    if (!sw_radar_on && !sw_light_on) {
      printf("Spotlight disabled by onboard switches.\r\n");
      turn_spotlight_off();
      HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 60, RTC_WAKEUPCLOCK_CK_SPRE_16BITS);
      goto sleep;
    }

    // - Only Light Sensor on: turn spotlight on when it is dark.
    if (!sw_radar_on && sw_light_on) {
      if (light >= LIGHT_THRESHOLD_OFF) {
        // Turn off spotlight if it is too bright
        turn_spotlight_off();
      } else if (light < LIGHT_THRESHOLD_4LED_ON) {
        // Otherwise turn it on by light level (2 or 4 LEDs)
        if (battery_low) {
          play_low_battery_animation();
        }
        turn_spotlight_on(light);
      }
      // Re-read light sensor 10 seconds, turn on spotlight when treshold reached
      HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 10, RTC_WAKEUPCLOCK_CK_SPRE_16BITS);
    }

    // - Radar with / without light sensor
    if (sw_radar_on) {
      uint32_t enabled = 1;
      if (sw_light_on) {
        // When Radar + Light Sensor enabled - start motion detect
        // only if it is dark enough
        enabled = light < LIGHT_THRESHOLD_4LED_ON;
      }
      if (!enabled) {
        if (motion_state != MOTION_OFF) {
          printf("Spotlight no longer enabled, too bright.\r\n");
          motion_state = MOTION_OFF;
          turn_spotlight_off();
          turn_radar_off();
        }
        // Spotlight no longer enabled, re-check it in 1 minute
        HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 60, RTC_WAKEUPCLOCK_CK_SPRE_16BITS);
        goto sleep;
      }
      // Enabled (too dark, or, light sensor disabled therefore always enabled)
      // Radar state machine: detecting -> detected -> clearing -> detecting || detected
      switch (motion_state) {

      case MOTION_OFF:
        printf("MOTION_OFF -> MOTION_DETECTING\r\n");
        motion_state = MOTION_DETECTING;
        turn_radar_on();
        HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
        break;

      case MOTION_DETECTING:
        // We can get here by either:
        // - wakeup timer
        // - radar detected motion
        if (motion_detected_isr) {
          printf("MOTION_DETECTING -> MOTION_DETECTED\r\n");
          motion_state = MOTION_DETECTED;
          turn_radar_off();
          // Re-check for motion soon
          HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, MOTION_STATE_THRESHOLD, RTC_WAKEUPCLOCK_CK_SPRE_16BITS);
        }
        break;

      case MOTION_DETECTED:
        if (wakeup_triggered) {
          printf("MOTION_DETECTED -> MOTION_CLEARING\r\n");
          motion_state = MOTION_CLEARING;
          turn_radar_on();
          // Wait for motion at most threshold seconds
          HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, MOTION_STATE_THRESHOLD, RTC_WAKEUPCLOCK_CK_SPRE_16BITS);
        }
        break;

      case MOTION_CLEARING:
        if (motion_detected_isr) {
          // motion still present
          printf("MOTION_CLEARING -> MOTION_DETECTED\r\n");
          motion_state = MOTION_DETECTED;
          turn_radar_off();
          // Re-check for motion soon
          HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, MOTION_STATE_THRESHOLD, RTC_WAKEUPCLOCK_CK_SPRE_16BITS);
        } else if (wakeup_triggered) {
          // no motion, disable wakeup until next motion interrupt
          printf("MOTION_CLEARING -> MOTION_DETECTING\r\n");
          motion_state = MOTION_DETECTING;
          HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
        }
        break;
      }  // of switch()

      if (MOTION_PRESENT(motion_state)) {
        if (battery_low) {
          play_low_battery_animation();
        }
        turn_spotlight_on(light);
      } else {
        turn_spotlight_off();
      }
    }

sleep:
    button_pressed = 0;
    wakeup_triggered = 0;
    motion_detected_isr = 0;
    // re-verify, maybe charger just connected
    if (charger_connected) {
      continue;
    }
    // Enter STOP mode ("deep" sleep)
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
    SystemClock_Config();
  }  // of while
}


// ISRs //

void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
  if (pin == BTN_Pin) {
    button_pressed = 1;
  }
  if (pin == BATT_CHRG_Pin) {
    charger_connected = !HAL_GPIO_ReadPin(BATT_CHRG_GPIO_Port, BATT_CHRG_Pin);
  }
  if (pin == BATT_STDBY_Pin) {
    charging_completed = !HAL_GPIO_ReadPin(BATT_STDBY_GPIO_Port, BATT_STDBY_Pin);
  }

  if (pin == RADAR_Pin) {
    motion_detected_isr = HAL_GPIO_ReadPin(RADAR_GPIO_Port, RADAR_Pin);
  }

  __HAL_GPIO_EXTI_CLEAR_FLAG(pin);
  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
}

void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
{
  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
  wakeup_triggered = 1;
}


// Utils //

static void turn_radar_on()
{
  // DEBUGLN("radar on");
  HAL_GPIO_WritePin(PWR_RADAR_GPIO_Port, PWR_RADAR_Pin, GPIO_PIN_RESET);
  skip_first_motion_isr = 1;
}

static void turn_radar_off()
{
  // DEBUGLN("radar off");
  HAL_GPIO_WritePin(PWR_RADAR_GPIO_Port, PWR_RADAR_Pin, GPIO_PIN_SET);
}

static void turn_spotlight_on(uint32_t light)
{
  if (light < LIGHT_THRESHOLD_4LED_ON) {
    // night mode: 4 LEDs
    HAL_GPIO_WritePin(LED_W1_GPIO_Port, LED_W1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_W2_GPIO_Port, LED_W2_Pin, GPIO_PIN_SET);
  } else {
    // twilight mode: 6 LEDs
    HAL_GPIO_WritePin(LED_W1_GPIO_Port, LED_W1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_W2_GPIO_Port, LED_W2_Pin, GPIO_PIN_SET);
  }
}

static void turn_spotlight_off()
{
  HAL_GPIO_WritePin(LED_W1_GPIO_Port, LED_W1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_W2_GPIO_Port, LED_W2_Pin, GPIO_PIN_RESET);
}

// Read on board mini switch settings (Radar / Light - on/off)
static void read_switch_settings(uint32_t *radar, uint32_t *light)
{
    // Configure PINs to input with pullup
    GPIO_InitTypeDef mode = {0};
    mode.Pin  = SW_RADAR_Pin;
    mode.Mode = GPIO_MODE_INPUT;
    mode.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(SW_RADAR_GPIO_Port, &mode);
    mode.Pin = SW_LIGHT_Pin;
    HAL_GPIO_Init(SW_LIGHT_GPIO_Port, &mode);

    // Read current levels (LOW - means ON)
    *radar = !HAL_GPIO_ReadPin(SW_RADAR_GPIO_Port, SW_RADAR_Pin);
    *light = !HAL_GPIO_ReadPin(SW_LIGHT_GPIO_Port, SW_LIGHT_Pin);

    // Configure PINs to to analog - save battery!
    mode.Pin  = SW_RADAR_Pin;
    mode.Mode = GPIO_MODE_ANALOG;
    mode.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(SW_RADAR_GPIO_Port, &mode);
    mode.Pin = SW_LIGHT_Pin;
    HAL_GPIO_Init(SW_LIGHT_GPIO_Port, &mode);
}

// Animation //

static void play_low_battery_animation()
{
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  // Play it 3 times
  for (int i = 0; i < 3; i++) {
    for (int i = 50; i < 255; i++) {
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, i);
      HAL_Delay(5);
    }
    for (int i = 255; i >= 0; i--) {
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, i);
      HAL_Delay(2);
    }
  }
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
}

static void turn_rgb_on_by_percent(int p)
{
  int r = 60;
  int g = 35;

  if (p <= 40) {
    g = p <= 5 ? 0 : p - 5;
  } else {
    r = 100 - p;
  }
  if (p > 95) {
    r = 0;
    g = 150;
  }

  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, r);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, g);
}

static void play_charging_animation()
{
  uint32_t last_percent = 0;

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  while (charger_connected || charging_completed) {
    uint32_t mvolts;
    read_light_and_battery_voltage(&mvolts, NULL);
    uint32_t percents = battery_voltage_to_percent(mvolts);
    // "de-bouncing"
    if (last_percent > percents) {
      percents = last_percent;
    }
    last_percent = percents;
    printf("Charging progress: %ld%% (%ld mV)\r\n", percents, mvolts);
    turn_rgb_on_by_percent(percents);
    HAL_Delay(1000);
  }

  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
}

static uint32_t battery_voltage_to_percent(uint32_t mvolts)
{
  if (mvolts < BATTERY_MIN) {
    return 0;
  }
  if (mvolts > BATTERY_MAX) {
    return 100;
  }
  return (mvolts - BATTERY_MIN) / BATTERY_ONE_PERCENT;
}
