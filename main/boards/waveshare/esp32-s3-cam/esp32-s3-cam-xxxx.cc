#include "wifi_board.h"
#include "codecs/box_audio_codec.h"
#include "display/lcd_display.h"
#include "system_reset.h"
#include "application.h"
#include "button.h"
#include "config.h"

#include <esp_log.h>
#include "i2c_device.h"
#include <driver/i2c_master.h>
#include <driver/ledc.h>
#include <wifi_station.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_timer.h>
#include "esp32_camera.h"
#include "esp_camera.h"
#include "esp_wifi.h"

#include "esp_lcd_panel_vendor.h"
#include "custom_io_expander_ch32v003.h"
#include "esp_lcd_st7796.h"
#include "esp_ota_ops.h"

#include "power_manager.h"

#define TAG "waveshare_s3_cam_xxxx"

#define LCD_OPCODE_WRITE_CMD   (0x02ULL)
#define LCD_OPCODE_READ_CMD    (0x0BULL)
#define LCD_OPCODE_WRITE_COLOR (0x32ULL)

void switch_to_main(void)
{
    const esp_partition_t *factory_part =
        esp_partition_find_first(ESP_PARTITION_TYPE_APP,
                                 ESP_PARTITION_SUBTYPE_APP_FACTORY,
                                 NULL);
    if (!factory_part) {
        ESP_LOGE("APP_SWITCH", "factory partition not found");
        return;
    }

    esp_err_t err = esp_ota_set_boot_partition(factory_part);
    if (err == ESP_OK) {
        ESP_LOGI("APP_SWITCH", "set factory as boot partition, restart");
        esp_restart();
    } else {
        ESP_LOGE("APP_SWITCH", "set boot partition failed: %s", esp_err_to_name(err));
    }
}

class CustomBacklight : public Backlight {
public:
    explicit CustomBacklight(esp_io_expander_handle_t io_handle)
        : Backlight(), io_handle_(io_handle) {}

protected:
    esp_io_expander_handle_t io_handle_;

    void SetBrightnessImpl(uint8_t brightness) override {
        if (brightness > 100) brightness = 100;
        int flipped_brightness = 100 - brightness;
        if (io_handle_ != nullptr) {
            custom_io_expander_set_pwm(io_handle_, flipped_brightness * 255 / 100);
        }
    }
};

class CustomBoard : public WifiBoard {
private:
    Button boot_button_;
    Button pwr_button_;

    i2c_master_bus_handle_t i2c_bus_ = nullptr;
    LcdDisplay* display_ = nullptr;
    Esp32Camera* camera_ = nullptr;
    esp_io_expander_handle_t io_expander_ = nullptr;
    CustomBacklight* backlight_ = nullptr;
    PowerManager* power_manager_ = nullptr;

    bool TryInitIoExpander(uint8_t addr) {
        ESP_LOGI(TAG, "Trying CH32 expander at 0x%02X", addr);
        esp_err_t err = custom_io_expander_new_i2c_ch32v003(i2c_bus_, addr, &io_expander_);
        if (err != ESP_OK || io_expander_ == nullptr) {
            ESP_LOGW(TAG, "CH32 expander init failed at 0x%02X: %s", addr, esp_err_to_name(err));
            io_expander_ = nullptr;
            return false;
        }
        ESP_LOGI(TAG, "CH32 expander found at 0x%02X", addr);
        return true;
    }

    void InitializeI2c() {
        i2c_master_bus_config_t i2c_bus_cfg = {};
        i2c_bus_cfg.i2c_port = (i2c_port_t)1;
        i2c_bus_cfg.sda_io_num = I2C_SDA_IO;
        i2c_bus_cfg.scl_io_num = I2C_SCL_IO;
        i2c_bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
        i2c_bus_cfg.glitch_ignore_cnt = 7;
        i2c_bus_cfg.flags.enable_internal_pullup = 1;

        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_));
    }

    void InitializeI2cAndLatchPower() {
        InitializeI2c();

        bool ok = TryInitIoExpander(BSP_IO_EXPANDER_I2C_ADDRESS);
        if (!ok && BSP_IO_EXPANDER_I2C_ADDRESS_FALLBACK != BSP_IO_EXPANDER_I2C_ADDRESS) {
            ok = TryInitIoExpander(BSP_IO_EXPANDER_I2C_ADDRESS_FALLBACK);
        }
        ESP_ERROR_CHECK(ok ? ESP_OK : ESP_FAIL);

        uint32_t output_mask =
            EXIO_LCD_RST |
            EXIO_CAM_PWDN |
            EXIO_PA_CTRL |
            EXIO_BAT_EN |
            EXIO_PWR_LED;

        ESP_ERROR_CHECK(esp_io_expander_set_dir(io_expander_, output_mask, IO_EXPANDER_OUTPUT));
        ESP_ERROR_CHECK(esp_io_expander_set_dir(io_expander_, EXIO_CHG_DET, IO_EXPANDER_INPUT));

        ESP_ERROR_CHECK(esp_io_expander_set_level(io_expander_, EXIO_BAT_EN, 1));
        ESP_ERROR_CHECK(esp_io_expander_set_level(io_expander_, EXIO_CAM_PWDN, 0));
        ESP_ERROR_CHECK(esp_io_expander_set_level(io_expander_, EXIO_PA_CTRL, 1));
        ESP_ERROR_CHECK(esp_io_expander_set_level(io_expander_, EXIO_PWR_LED, 1));

        ESP_ERROR_CHECK(esp_io_expander_set_level(io_expander_, EXIO_LCD_RST, 1));
        vTaskDelay(pdMS_TO_TICKS(5));
        ESP_ERROR_CHECK(esp_io_expander_set_level(io_expander_, EXIO_LCD_RST, 0));
        vTaskDelay(pdMS_TO_TICKS(20));
        ESP_ERROR_CHECK(esp_io_expander_set_level(io_expander_, EXIO_LCD_RST, 1));
        vTaskDelay(pdMS_TO_TICKS(20));

        ESP_LOGI(TAG, "Power latch active on EXIO_BAT_EN");
    }

    void InitializePowerManager() {
        power_manager_ = new PowerManager(
            BATTERY_CHARGING_PIN,
            BATTERY_ADC_PIN,
            BATTERY_EN_PIN
        );
        power_manager_->PowerON();
    }

    void DisableWifiPowerSave() {
        esp_err_t err = esp_wifi_set_ps(WIFI_PS_NONE);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "WiFi power save disabled: WIFI_PS_NONE");
        } else {
            ESP_LOGW(TAG, "Failed to disable WiFi power save: %s", esp_err_to_name(err));
        }
    }

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = DISPLAY_MOSI_PIN;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = DISPLAY_SCLK_PIN;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeSt7789Display() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;

        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = DISPLAY_CS_PIN;
        io_config.dc_gpio_num = DISPLAY_DC_PIN;
        io_config.spi_mode = 0;
        io_config.pclk_hz = DISPLAY_SPI_SCLK_HZ;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI2_HOST, &io_config, &panel_io));

        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = GPIO_NUM_NC;
        panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
        panel_config.bits_per_pixel = 16;
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel));

        ESP_ERROR_CHECK(esp_lcd_panel_reset(panel));
        ESP_ERROR_CHECK(esp_lcd_panel_init(panel));
        ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY));
        ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y));
        ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel, DISPLAY_INVERT_COLOR));
        ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel, true));

        display_ = new SpiLcdDisplay(
            panel_io, panel,
            DISPLAY_WIDTH, DISPLAY_HEIGHT,
            DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y,
            DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY
        );

        backlight_ = new CustomBacklight(io_expander_);
        backlight_->RestoreBrightness();
    }

    void InitializeSt7796Display() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;

        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = DISPLAY_CS_PIN;
        io_config.dc_gpio_num = DISPLAY_DC_PIN;
        io_config.spi_mode = 0;
        io_config.pclk_hz = DISPLAY_SPI_SCLK_HZ;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI2_HOST, &io_config, &panel_io));

        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = GPIO_NUM_NC;
        panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR;
        panel_config.bits_per_pixel = 16;
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7796(panel_io, &panel_config, &panel));

        ESP_ERROR_CHECK(esp_lcd_panel_reset(panel));
        ESP_ERROR_CHECK(esp_lcd_panel_init(panel));
        ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY));
        ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y));
        ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel, DISPLAY_INVERT_COLOR));
        ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel, true));

        display_ = new SpiLcdDisplay(
            panel_io, panel,
            DISPLAY_WIDTH, DISPLAY_HEIGHT,
            DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y,
            DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY
        );

        backlight_ = new CustomBacklight(io_expander_);
        backlight_->RestoreBrightness();
    }

    void ShutdownBoard() {
        ESP_LOGI(TAG, "Shutdown requested");

        if (backlight_ != nullptr) {
            backlight_->SetBrightness(0);
        }

        if (io_expander_ != nullptr) {
            esp_io_expander_set_level(io_expander_, EXIO_PWR_LED, 0);
            esp_io_expander_set_level(io_expander_, EXIO_CAM_PWDN, 1);
            vTaskDelay(pdMS_TO_TICKS(20));
            esp_io_expander_set_level(io_expander_, EXIO_BAT_EN, 0);
        }

        vTaskDelay(pdMS_TO_TICKS(200));
        esp_restart();
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });

        boot_button_.OnLongPress([this]() {
            switch_to_main();
        });

        pwr_button_.OnClick([this]() {
        });

        pwr_button_.OnLongPress([this]() {
            if (power_manager_ != nullptr) {
                power_manager_->PowerOff();
            }
            ShutdownBoard();
        });
    }

    void ApplyOv3660Tuning() {
        sensor_t *s = esp_camera_sensor_get();
        if (s == nullptr) {
            ESP_LOGE(TAG, "esp_camera_sensor_get() failed");
            return;
        }

        s->set_vflip(s, 0);
        s->set_hmirror(s, 0);

        s->set_brightness(s, 1);
        s->set_contrast(s, 1);
        s->set_saturation(s, -1);

        s->set_whitebal(s, 1);
        s->set_awb_gain(s, 1);
        s->set_exposure_ctrl(s, 1);
        s->set_gain_ctrl(s, 1);
        s->set_aec2(s, 1);
        s->set_ae_level(s, 0);

        s->set_bpc(s, 1);
        s->set_wpc(s, 1);
        s->set_raw_gma(s, 1);
        s->set_lenc(s, 1);
        s->set_dcw(s, 1);

        s->set_quality(s, 10);

        ESP_LOGI(TAG, "OV3660 tuning applied");
    }

    void WarmUpCameraFrames() {
        for (int i = 0; i < 2; ++i) {
            camera_fb_t *fb = esp_camera_fb_get();
            if (fb != nullptr) {
                esp_camera_fb_return(fb);
            }
            vTaskDelay(pdMS_TO_TICKS(30));
        }
    }

    void InitializeCamera() {
        camera_config_t camera_config = {
            .pin_pwdn = CAMERA_PIN_PWDN,
            .pin_reset = CAMERA_PIN_RESET,
            .pin_xclk = CAMERA_PIN_XCLK,
            .pin_sccb_sda = -1,
            .pin_sccb_scl = -1,
            .pin_d7 = CAMERA_PIN_D7,
            .pin_d6 = CAMERA_PIN_D6,
            .pin_d5 = CAMERA_PIN_D5,
            .pin_d4 = CAMERA_PIN_D4,
            .pin_d3 = CAMERA_PIN_D3,
            .pin_d2 = CAMERA_PIN_D2,
            .pin_d1 = CAMERA_PIN_D1,
            .pin_d0 = CAMERA_PIN_D0,
            .pin_vsync = CAMERA_PIN_VSYNC,
            .pin_href = CAMERA_PIN_HREF,
            .pin_pclk = CAMERA_PIN_PCLK,

            .xclk_freq_hz = XCLK_FREQ_HZ,
            .ledc_timer = LEDC_TIMER_0,
            .ledc_channel = LEDC_CHANNEL_0,

            // Keep Xiaozhi path stable: do not raise RGB565 capture size here
            .pixel_format = PIXFORMAT_RGB565,
            .frame_size = FRAMESIZE_QVGA,

            // Only used when native JPEG path is selected elsewhere
            .jpeg_quality = 10,

            // Lower memory pressure and reduce stale-frame problems
            .fb_count = 1,
            .fb_location = CAMERA_FB_IN_PSRAM,
            .grab_mode = CAMERA_GRAB_LATEST,
            .sccb_i2c_port = (i2c_port_t)1,
        };

        camera_ = new Esp32Camera(camera_config);
        if (camera_ != nullptr) {
            camera_->SetVFlip(false);
        }

        vTaskDelay(pdMS_TO_TICKS(150));
        ApplyOv3660Tuning();
        WarmUpCameraFrames();
    }

public:
    CustomBoard()
        : boot_button_(BOOT_BUTTON_GPIO),
          pwr_button_(PWR_BUTTON_GPIO)
    {
        InitializeI2cAndLatchPower();
        InitializePowerManager();
        InitializeSpi();

    #ifdef CONFIG_BSP_LCD_SIZE_2INCH
        InitializeSt7789Display();
    #elif CONFIG_BSP_LCD_SIZE_2_8INCH
        InitializeSt7789Display();
    #elif CONFIG_BSP_LCD_SIZE_1_83INCH
        InitializeSt7789Display();
    #elif CONFIG_BSP_LCD_SIZE_3_5INCH
        InitializeSt7796Display();
    #endif

        InitializeCamera();
        InitializeButtons();
        DisableWifiPowerSave();

    #if CONFIG_USE_DEVICE_AEC
        auto& app = Application::GetInstance();
        app.SetAecMode(kAecOnDeviceSide);
    #endif
    }

    AudioCodec* GetAudioCodec() override {
        static BoxAudioCodec audio_codec(
            i2c_bus_, AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_MCLK, AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS,
            AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN,
            AUDIO_CODEC_PA_PIN, AUDIO_CODEC_ES8311_ADDR,
            AUDIO_CODEC_ES7210_ADDR, AUDIO_INPUT_REFERENCE
        );
        return &audio_codec;
    }

    Display* GetDisplay() override {
        return display_;
    }

    Camera* GetCamera() override {
        return camera_;
    }
};

DECLARE_BOARD(CustomBoard);