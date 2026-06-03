#pragma once

#include <esp_log.h>
#include <driver/gpio.h>
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include <math.h>

class PowerManager
{
private:
    gpio_num_t charging_pin_  = GPIO_NUM_NC;
    gpio_num_t bat_adc_pin_   = GPIO_NUM_NC;
    gpio_num_t bat_power_pin_ = GPIO_NUM_NC;

    adc_oneshot_unit_handle_t adc_handle_ = nullptr;
    adc_cali_handle_t adc_cali_handle_    = nullptr;
    adc_channel_t adc_channel_            = ADC_CHANNEL_0;
    bool do_calibration                   = false;

    bool adc_calibration_init(adc_unit_t unit,
                              adc_channel_t channel,
                              adc_atten_t atten,
                              adc_cali_handle_t *out_handle)
    {
        adc_cali_handle_t handle = nullptr;
        esp_err_t ret = ESP_FAIL;
        bool calibrated = false;

        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id  = unit,
            .chan     = channel,
            .atten    = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };

        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
            ESP_LOGI("power_manager", "ADC calibration success");
        } else if (ret == ESP_ERR_NOT_SUPPORTED) {
            ESP_LOGW("power_manager", "ADC calibration not supported, using raw ADC");
        } else {
            ESP_LOGE("power_manager", "ADC calibration init failed: %s", esp_err_to_name(ret));
        }

        *out_handle = handle;
        return calibrated;
    }

public:
    PowerManager(gpio_num_t charging_pin, gpio_num_t bat_adc_pin, gpio_num_t bat_power_pin)
        : charging_pin_(charging_pin),
          bat_adc_pin_(bat_adc_pin),
          bat_power_pin_(bat_power_pin)
    {
        // Charging detect pin
        if (charging_pin_ != GPIO_NUM_NC) {
            gpio_config_t io_conf = {};
            io_conf.intr_type    = GPIO_INTR_DISABLE;
            io_conf.mode         = GPIO_MODE_INPUT;
            io_conf.pin_bit_mask = 1ULL << charging_pin_;
            io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
            io_conf.pull_up_en   = GPIO_PULLUP_ENABLE;
            ESP_ERROR_CHECK(gpio_config(&io_conf));
        }

        // Battery enable pin (used on boards where BAT_EN is directly on ESP32 GPIO)
        if (bat_power_pin_ != GPIO_NUM_NC) {
            gpio_config_t io_conf = {};
            io_conf.intr_type    = GPIO_INTR_DISABLE;
            io_conf.mode         = GPIO_MODE_OUTPUT;
            io_conf.pin_bit_mask = 1ULL << bat_power_pin_;
            io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
            io_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
            ESP_ERROR_CHECK(gpio_config(&io_conf));
        }

        // Battery ADC pin
        if (bat_adc_pin_ != GPIO_NUM_NC) {
            adc_oneshot_unit_init_cfg_t init_config = {};
            init_config.ulp_mode = ADC_ULP_MODE_DISABLE;

            if (bat_adc_pin_ >= GPIO_NUM_1 && bat_adc_pin_ <= GPIO_NUM_10) {
                init_config.unit_id = ADC_UNIT_1;
                adc_channel_ = (adc_channel_t)((int)bat_adc_pin_ - 1);
            } else if (bat_adc_pin_ >= GPIO_NUM_11 && bat_adc_pin_ <= GPIO_NUM_20) {
                init_config.unit_id = ADC_UNIT_2;
                adc_channel_ = (adc_channel_t)((int)bat_adc_pin_ - 11);
            } else {
                ESP_LOGW("power_manager", "Battery ADC GPIO not supported: %d", (int)bat_adc_pin_);
                return;
            }

            ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle_));

            adc_oneshot_chan_cfg_t config = {};
            config.bitwidth = ADC_BITWIDTH_DEFAULT;
            config.atten    = ADC_ATTEN_DB_12;
            ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle_, adc_channel_, &config));

            do_calibration = adc_calibration_init(
                init_config.unit_id,
                adc_channel_,
                config.atten,
                &adc_cali_handle_
            );
        }
    }

    ~PowerManager()
    {
        if (adc_handle_) {
            ESP_ERROR_CHECK(adc_oneshot_del_unit(adc_handle_));
        }

        // Optional cleanup if needed in your IDF version:
        // if (adc_cali_handle_) {
        //     ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(adc_cali_handle_));
        // }
    }

    int GetBatteryLevel(void)
    {
        int adc_raw = 0;
        int voltage_int = 0;
        float voltage_float = 0.0f;

        static float last_voltage_float = 0.0f;
        static int last_battery_level = 100;

        const float voltage_float_threshold = 0.1f;

        if (adc_handle_ != nullptr) {
            ESP_ERROR_CHECK(adc_oneshot_read(adc_handle_, adc_channel_, &adc_raw));

            if (do_calibration && adc_cali_handle_ != nullptr) {
                ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_cali_handle_, adc_raw, &voltage_int));
                voltage_float = (voltage_int / 1000.0f) * 3.0f;
            } else {
                // Fallback rough estimation if calibration is unavailable
                voltage_float = ((float)adc_raw / 4095.0f) * 3.3f * 3.0f;
            }

            if (fabsf(voltage_float - last_voltage_float) >= voltage_float_threshold) {
                last_voltage_float = voltage_float;

                if (voltage_float < 3.52f) {
                    last_battery_level = 1;
                } else if (voltage_float < 3.64f) {
                    last_battery_level = 20;
                } else if (voltage_float < 3.76f) {
                    last_battery_level = 40;
                } else if (voltage_float < 3.88f) {
                    last_battery_level = 60;
                } else if (voltage_float < 4.0f) {
                    last_battery_level = 80;
                } else {
                    last_battery_level = 100;
                }
            }

            return last_battery_level;
        }

        // On ESP32-S3-CAM BAT_ADC is not on ESP32 GPIO, so this usually returns 100
        return 100;
    }

    bool IsCharging(void)
    {
        if (charging_pin_ != GPIO_NUM_NC) {
            return gpio_get_level(charging_pin_) == 0;
        }
        return false;
    }

    bool IsDischarging(void)
    {
        if (charging_pin_ != GPIO_NUM_NC) {
            return gpio_get_level(charging_pin_) == 1;
        }
        return true;
    }

    bool IsChargingDone(void)
    {
        return GetBatteryLevel() == 100;
    }

    void PowerOff(void)
    {
        if (bat_power_pin_ != GPIO_NUM_NC) {
            gpio_set_level(bat_power_pin_, 0);
        }
        // ESP32-S3-CAM actual shutdown is done in board file by:
        // esp_io_expander_set_level(io_expander_, IO_EXPANDER_PIN_NUM_6, 0);
    }

    void PowerON(void)
    {
        if (bat_power_pin_ != GPIO_NUM_NC) {
            gpio_set_level(bat_power_pin_, 1);
        }
    }
};