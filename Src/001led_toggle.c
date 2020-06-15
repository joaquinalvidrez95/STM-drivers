/**
 * @file 001led_toggle.c
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2020-05-19
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#include <string.h>
#include <stdio.h>
#include "stm32f446xx.h"
#include "stm32f446xx_gpio_driver.h"
#include "stm32f446xx_spi_driver.h"
#include "arduino.h"
#include "utils.h"

#define MAIN_001_LED_TOGGLE 0
#define MAIN_002_LED_BUTTON 1
#define MAIN_005_BUTTTON_INTERRUPT 2
#define MAIN_006_SPI_SEND 3
#define MAIN_007_SPI_TX_ONLY_ARDUINO 4
#define MAIN_008_SPI_ARDUINO 5

#define MAIN MAIN_008_SPI_ARDUINO

extern void initialise_monitor_handles();


#if MAIN == MAIN_001_LED_TOGGLE

int main()
{
    Gpio_handle_t led;
    led.reg = GPIOA;
    led.pin_config.number = GPIO_PIN_5;
    led.pin_config.mode = GPIO_MODE_OUT;
    led.pin_config.speed = GPIO_SPEED_FAST;
    led.pin_config.out_type = GPIO_OUT_TYPE_PUSH_PULL;
    led.pin_config.pull_mode = GPIO_PULL_MODE_NONE;

    Gpio_peripheral_clock_control(led.reg, En_status_enable);
    Gpio_init(&led);

    while (1)
    {
        Gpio_toggle_pin(&led);
        Utils_delay();
    }

    return 0;
}

#elif MAIN == MAIN_002_LED_BUTTON
int main()
{
    Gpio_handle_t led;
    led.reg = GPIOA;
    led.pin_config.number = Gpio_pin_5;
    led.pin_config.mode = GPIO_MODE_OUT;
    led.pin_config.speed = GPIO_SPEED_FAST;
    led.pin_config.out_type = GPIO_OUT_TYPE_PUSH_PULL;
    led.pin_config.pull_mode = GPIO_PULL_MODE_NONE;

    Gpio_peripheral_clock_control(led.reg, true);
    Gpio_init(&led);

    Gpio_handle_t button;
    button.reg = GPIOC;
    button.pin_config.number = GPIO_PIN_13;
    button.pin_config.mode = GPIO_MODE_IN;
    button.pin_config.speed = GPIO_SPEED_FAST;
    button.pin_config.pull_mode = GPIO_PULL_MODE_NONE;

    Gpio_peripheral_clock_control(button.reg, true);
    Gpio_init(&button);

    while (1)
    {
        if (GPIO_BUTTON_STATE_LOW == Gpio_read_from_input_pin(&button))
        {
            Utils_delay();
            Gpio_toggle_pin(&led);
        }
    }

    return 0;
}
#elif MAIN == MAIN_005_BUTTTON_INTERRUPT
static Gpio_handle_t button = {0u};
static Gpio_handle_t led = {0u};
int main()
{
    led.reg = GPIOA;
    led.pin_config.number = Gpio_pin_5;
    led.pin_config.mode = GPIO_MODE_OUT;
    led.pin_config.speed = GPIO_SPEED_FAST;
    led.pin_config.out_type = GPIO_OUT_TYPE_PUSH_PULL;
    led.pin_config.pull_mode = GPIO_PULL_MODE_NONE;

    Gpio_peripheral_clock_control(led.reg, true);
    Gpio_init(&led);

    button.reg = GPIOC;
    button.pin_config.number = GPIO_PIN_13;
    button.pin_config.mode = GPIO_MODE_INTERRUPT_FT;
    button.pin_config.speed = GPIO_SPEED_FAST;
    button.pin_config.pull_mode = GPIO_PULL_MODE_NONE;

    Gpio_peripheral_clock_control(button.reg, true);
    Gpio_init(&button);

    /* Configures IRQ */
    Gpio_config_irq_priority(IRQ_NUMBER_EXTI15_10, NVIC_IRQ_PRIORITY_15);
    Gpio_config_irq(IRQ_NUMBER_EXTI15_10, true);

    while (1)
        ;

    return 0;
}

void EXTI15_10_IRQHandler()
{
    Utils_delay();
    Gpio_irq_handling(button.pin_config.number);
    Gpio_toggle_pin(&led);
}
#elif MAIN == MAIN_006_SPI_SEND
/**
 * SPICLK - PA9 - D8
 * SPIMOSI - PB15 - H26
 * NSS - PB4 - D5
 */

typedef struct
{
    Gpio_handle_t clock;
    Gpio_handle_t mosi;
    Gpio_handle_t nss;
} Spi_pins_t;

static void init_gpio(Spi_pins_t *pins)
{
    pins->clock.reg = GPIOA;
    pins->clock.pin_config.number = GPIO_PIN_9;
    pins->clock.pin_config.mode = GPIO_MODE_ALT_FN;
    pins->clock.pin_config.alt_fun_mode = GPIO_ALTERNATE_FUNCTION_5;
    pins->clock.pin_config.out_type = GPIO_OUT_TYPE_PUSH_PULL;
    pins->clock.pin_config.pull_mode = GPIO_PULL_MODE_NONE;
    Gpio_init(&pins->clock);

    pins->mosi.reg = GPIOB;
    pins->mosi.pin_config.number = GPIO_PIN_15;
    pins->mosi.pin_config.mode = GPIO_MODE_ALT_FN;
    pins->mosi.pin_config.alt_fun_mode = GPIO_ALTERNATE_FUNCTION_5;
    pins->mosi.pin_config.out_type = GPIO_OUT_TYPE_PUSH_PULL;
    pins->mosi.pin_config.pull_mode = GPIO_PULL_MODE_NONE;
    Gpio_init(&pins->mosi);

    // pins->nss.reg = GPIOB;
    // pins->nss.pin_config.number = GPIO_PIN_4;
    // pins->nss.pin_config.mode = GPIO_MODE_ALT_FN;
    // pins->nss.pin_config.alt_fun_mode = GPIO_ALTERNATE_FUNCTION_7;
    // pins->nss.pin_config.out_type = GPIO_OUT_TYPE_PUSH_PULL;
    // pins->nss.pin_config.pull_mode = GPIO_PULL_MODE_NONE;
    // Gpio_init(pins->nss.reg);
}

void init_spi(Spi_handle_t *handle)
{
    handle->reg = SPI1;
    handle->config.device_mode = SPI_DEVICE_MODE_MASTER;
    handle->config.bus_config = SPI_BUS_CONFIG_FULL_DUPLEX;
    handle->config.sclk_speed = SPI_CLOCK_SPEED_DIV_2;
    handle->config.DFF = SPI_DFF_8_BITS;
    handle->config.CPOL = SPI_CPOL_LOW;
    handle->config.CPHA = SPI_CPHA_LOW;
    handle->config.SSM = SPI_SSM_ENABLED;

    Spi_init(handle);
}

int main()
{
    Spi_pins_t gpio_spi = {0u};
    init_gpio(&gpio_spi);

    Spi_handle_t spi = {0u};
    init_spi(&spi);
    Spi_enable_ssi(spi.reg, true);
    Spi_enable_peripheral(spi.reg, true);

    char user_data[] = "hello world";
    spi.tx.data = (uint8_t *)user_data;
    spi.tx.size = strlen(user_data);
    Spi_send(&spi);

    while (1)
    {
    }

    return 0;
}
#elif MAIN == MAIN_007_SPI_TX_ONLY_ARDUINO
/**
 * SPICLK - PA9 - D8
 * SPIMOSI - PB15 - H26
 * NSS - PB4 - D5
 */

typedef struct
{
    Gpio_handle_t clock;
    Gpio_handle_t mosi;
    Gpio_handle_t nss;
} Spi_pins_t;

static void init_gpio(Spi_pins_t *pins)
{
    pins->clock.reg = GPIOA;
    pins->clock.pin_config.number = GPIO_PIN_9;
    pins->clock.pin_config.mode = GPIO_MODE_ALT_FN;
    pins->clock.pin_config.alt_fun_mode = GPIO_ALTERNATE_FUNCTION_5;
    pins->clock.pin_config.out_type = GPIO_OUT_TYPE_PUSH_PULL;
    pins->clock.pin_config.pull_mode = GPIO_PULL_MODE_UP;
    Gpio_init(&pins->clock);

    pins->mosi.reg = GPIOB;
    pins->mosi.pin_config.number = GPIO_PIN_15;
    pins->mosi.pin_config.mode = GPIO_MODE_ALT_FN;
    pins->mosi.pin_config.alt_fun_mode = GPIO_ALTERNATE_FUNCTION_5;
    pins->mosi.pin_config.out_type = GPIO_OUT_TYPE_PUSH_PULL;
    pins->mosi.pin_config.pull_mode = GPIO_PULL_MODE_UP;
    Gpio_init(&pins->mosi);

    pins->nss.reg = GPIOB;
    pins->nss.pin_config.number = GPIO_PIN_4;
    pins->nss.pin_config.mode = GPIO_MODE_ALT_FN;
    pins->nss.pin_config.alt_fun_mode = GPIO_ALTERNATE_FUNCTION_7;
    pins->nss.pin_config.out_type = GPIO_OUT_TYPE_PUSH_PULL;
    pins->nss.pin_config.pull_mode = GPIO_PULL_MODE_UP;
    Gpio_init(&pins->nss);
}

void init_spi(Spi_handle_t *handle)
{
    handle->reg = SPI2;
    handle->config.device_mode = SPI_DEVICE_MODE_MASTER;
    handle->config.bus_config = SPI_BUS_CONFIG_FULL_DUPLEX;
    handle->config.sclk_speed = SPI_CLOCK_SPEED_DIV_8;
    handle->config.DFF = SPI_DFF_8_BITS;
    handle->config.CPOL = SPI_CPOL_LOW;
    handle->config.CPHA = SPI_CPHA_LOW;
    handle->config.SSM = SPI_SSM_DISABLED;

    Spi_init(handle);
}

static void init_button(Gpio_handle_t *button)
{
    button->reg = GPIOC;
    button->pin_config.number = GPIO_PIN_13;
    button->pin_config.mode = GPIO_MODE_IN;
    button->pin_config.speed = GPIO_SPEED_FAST;
    button->pin_config.pull_mode = GPIO_PULL_MODE_NONE;

    Gpio_init(button);
}

int main()
{
    char user_data[] = "hello world";
    Spi_pins_t gpio_spi = {0u};
    init_gpio(&gpio_spi);

    Spi_handle_t spi = {0u};
    init_spi(&spi);
    Spi_enable_ssoe(spi.reg, true);

    Gpio_handle_t button;
    init_button(&button);

    while (1)
    {
        while (GPIO_BUTTON_STATE_HIGH == Gpio_read_from_input_pin(&button))
        {
        }
        Utils_delay();

        Spi_enable_peripheral(spi.reg, true);
        uint8_t length = strlen(user_data);
        spi.tx.data = &length;
        spi.tx.size = sizeof(length);
        Spi_send(&spi);

        spi.tx.data = (uint8_t *)user_data;
        spi.tx.size = length;
        Spi_send(&spi);
        while (spi.reg->SR.BSY)
        {
        }

        Spi_enable_peripheral(spi.reg, false);
    }

    return 0;
}
#elif MAIN == MAIN_008_SPI_ARDUINO

typedef struct
{
    Gpio_handle_t clock;
    Gpio_handle_t mosi;
    Gpio_handle_t miso;
    Gpio_handle_t nss;
} Spi_pins_t;

#define LED_PIN ARDUINO_DIGITAL_PIN_9

/**
 * SPICLK - PA9 - D8
 * SPIMOSI - PB15 - H26
 * SPIMISO - PC2 - Left header 35
 * NSS - PB4 - D5
 */
static void init_gpio(Spi_pins_t *pins)
{
    pins->clock.reg = GPIOA;
    pins->clock.pin_config.number = GPIO_PIN_9;
    pins->clock.pin_config.mode = GPIO_MODE_ALT_FN;
    pins->clock.pin_config.alt_fun_mode = GPIO_ALTERNATE_FUNCTION_5;
    pins->clock.pin_config.out_type = GPIO_OUT_TYPE_PUSH_PULL;
    pins->clock.pin_config.pull_mode = GPIO_PULL_MODE_UP;
    Gpio_init(&pins->clock);

    pins->mosi.reg = GPIOB;
    pins->mosi.pin_config.number = GPIO_PIN_15;
    pins->mosi.pin_config.mode = GPIO_MODE_ALT_FN;
    pins->mosi.pin_config.alt_fun_mode = GPIO_ALTERNATE_FUNCTION_5;
    pins->mosi.pin_config.out_type = GPIO_OUT_TYPE_PUSH_PULL;
    pins->mosi.pin_config.pull_mode = GPIO_PULL_MODE_UP;
    Gpio_init(&pins->mosi);

    pins->miso.reg = GPIOC;
    pins->miso.pin_config.number = GPIO_PIN_2;
    pins->miso.pin_config.mode = GPIO_MODE_ALT_FN;
    pins->miso.pin_config.alt_fun_mode = GPIO_ALTERNATE_FUNCTION_5;
    pins->miso.pin_config.out_type = GPIO_OUT_TYPE_PUSH_PULL;
    pins->miso.pin_config.pull_mode = GPIO_PULL_MODE_UP;
    Gpio_init(&pins->miso);

    pins->nss.reg = GPIOB;
    pins->nss.pin_config.number = GPIO_PIN_4;
    pins->nss.pin_config.mode = GPIO_MODE_ALT_FN;
    pins->nss.pin_config.alt_fun_mode = GPIO_ALTERNATE_FUNCTION_7;
    pins->nss.pin_config.out_type = GPIO_OUT_TYPE_PUSH_PULL;
    pins->nss.pin_config.pull_mode = GPIO_PULL_MODE_UP;
    Gpio_init(&pins->nss);
}

void init_spi(Spi_handle_t *handle)
{
    handle->reg = SPI2;
    handle->config.device_mode = SPI_DEVICE_MODE_MASTER;
    handle->config.bus_config = SPI_BUS_CONFIG_FULL_DUPLEX;
    handle->config.sclk_speed = SPI_CLOCK_SPEED_DIV_8;
    handle->config.DFF = SPI_DFF_8_BITS;
    handle->config.CPOL = SPI_CPOL_LOW;
    handle->config.CPHA = SPI_CPHA_LOW;
    handle->config.SSM = SPI_SSM_DISABLED;

    Spi_init(handle);
}

static void init_button(Gpio_handle_t *button)
{
    button->reg = GPIOC;
    button->pin_config.number = GPIO_PIN_13;
    button->pin_config.mode = GPIO_MODE_IN;
    button->pin_config.speed = GPIO_SPEED_FAST;
    button->pin_config.pull_mode = GPIO_PULL_MODE_NONE;

    Gpio_init(button);
}

int main()
{
    initialise_monitor_handles();

    printf("hello man");
    Spi_pins_t gpio_spi = {0u};
    init_gpio(&gpio_spi);

    Spi_handle_t spi = {0u};
    init_spi(&spi);
    Spi_enable_ssoe(spi.reg, true);

    Gpio_handle_t button;
    init_button(&button);

    while (true)
    {
        while (GPIO_BUTTON_STATE_HIGH == Gpio_read_from_input_pin(&button))
        {
        }
        Utils_delay();

        Spi_enable_peripheral(spi.reg, true);

        Arduino_write_led(&spi, ARDUINO_DIGITAL_STATUS_ON, LED_PIN);

        while (GPIO_BUTTON_STATE_HIGH == Gpio_read_from_input_pin(&button))
        {
        }
        Utils_delay();

        uint8_t value = Arduino_read_analog(&spi, ARDUINO_ANALOG_PIN_0);

        while (GPIO_BUTTON_STATE_HIGH == Gpio_read_from_input_pin(&button))
        {
        }
        Utils_delay();
        value = Arduino_read_digital(&spi, LED_PIN);

        while (GPIO_BUTTON_STATE_HIGH == Gpio_read_from_input_pin(&button))
        {
        }
        Utils_delay();
        Arduino_print(&spi, "hello Arduino");

        while (GPIO_BUTTON_STATE_HIGH == Gpio_read_from_input_pin(&button))
        {
        }
        Utils_delay();
        char arduino_id[ARDUINO_ID_SIZE + 1u] = {'\0'};
        Arduino_read_id(&spi, arduino_id, sizeof(arduino_id));

        while (spi.reg->SR.BSY)
        {
        }

        Spi_enable_peripheral(spi.reg, false);
    }

    return 0;
}

#endif
