#include "oled.h"
#include "esp_log.h"
#include "ssd1306.h"
#include <string.h> 

static const char *TAG = "OLED";
static SSD1306_t dev;

void oled_init(void) {
    // Initialize I2C
    i2c_master_init(&dev, OLED_SDA_PIN, OLED_SCL_PIN, OLED_RST_PIN);
    
    // Initialize OLED
    ssd1306_init(&dev, OLED_WIDTH, OLED_HEIGHT);
    ssd1306_clear_screen(&dev, false);
    ssd1306_contrast(&dev, 0x7F);
    ESP_LOGI(TAG, "OLED initialized");
}

void oled_clear(void) {
    ssd1306_clear_screen(&dev, false);
    ssd1306_show_buffer(&dev);
}

void oled_display_text(const char *text, uint8_t page) {
    if(page > 7) return;
    ssd1306_clear_line(&dev, page, false);
    ssd1306_display_text(&dev, page, (char *)text, strlen(text), false);
    ssd1306_show_buffer(&dev);
}

void oled_display_bitmap(const uint8_t *bitmap) {
    ssd1306_set_buffer(&dev, (uint8_t *)bitmap);
    ssd1306_show_buffer(&dev);
}
