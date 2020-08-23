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
#include "stm32f446xx_i2c_driver.h"
#include "arduino.h"
#include "utils.h"
#include "nucleo.h"

#define MAIN_001_LED_TOGGLE 0
#define MAIN_002_LED_BUTTON 1
#define MAIN_005_BUTTTON_INTERRUPT 2
#define MAIN_006_SPI_SEND 3
#define MAIN_007_SPI_TX_ONLY_ARDUINO 4
#define MAIN_008_SPI_ARDUINO 5
#define MAIN_009_SPI_ARDUINO_IT 6
#define MAIN_010_I2C_MASTER_TX 7

#define MAIN MAIN_010_I2C_MASTER_TX

extern void initialise_monitor_handles();

#if MAIN == MAIN_001_LED_TOGGLE

int main()
{
    gpio_handle_t led;
    led.p_reg = GPIOA;
    led.cfg.number = GPIO_PIN_5;
    led.cfg.mode = GPIO_MODE_OUT;
    led.cfg.speed = GPIO_SPEED_FAST;
    led.cfg.out_type = GPIO_OUT_TYPE_PUSH_PULL;
    led.cfg.pull_mode = GPIO_PULL_MODE_NONE;

    gpio_enable_peripheral_clock(led.p_reg, En_status_enable);
    gpio_init(&led);

    while (1)
    {
        gpio_toggle_pin(&led);
        utils_delay();
    }

    return 0;
}

#elif MAIN == MAIN_002_LED_BUTTON
int main()
{
    gpio_handle_t led;
    led.p_reg = GPIOA;
    led.cfg.number = GPIO_PIN_5;
    led.cfg.mode = GPIO_MODE_OUT;
    led.cfg.speed = GPIO_SPEED_FAST;
    led.cfg.out_type = GPIO_OUT_TYPE_PUSH_PULL;
    led.cfg.pull_mode = GPIO_PULL_MODE_NONE;

    gpio_enable_peripheral_clock(led.p_reg, true);
    gpio_init(&led);

    gpio_handle_t button;
    button.p_reg = GPIOC;
    button.cfg.number = GPIO_PIN_13;
    button.cfg.mode = GPIO_MODE_IN;
    button.cfg.speed = GPIO_SPEED_FAST;
    button.cfg.pull_mode = GPIO_PULL_MODE_NONE;

    gpio_enable_peripheral_clock(button.p_reg, true);
    gpio_init(&button);

    while (1)
    {
        if (GPIO_BUTTON_STATE_LOW == gpio_read_pin(&button))
        {
            utils_delay();
            gpio_toggle_pin(&led);
        }
    }

    return 0;
}
#elif MAIN == MAIN_005_BUTTTON_INTERRUPT
static gpio_handle_t button = {0u};
static gpio_handle_t led = {0u};
int main()
{
    led.p_reg = GPIOA;
    led.cfg.number = Gpio_pin_5;
    led.cfg.mode = GPIO_MODE_OUT;
    led.cfg.speed = GPIO_SPEED_FAST;
    led.cfg.out_type = GPIO_OUT_TYPE_PUSH_PULL;
    led.cfg.pull_mode = GPIO_PULL_MODE_NONE;

    gpio_enable_peripheral_clock(led.p_reg, true);
    gpio_init(&led);

    button.p_reg = GPIOC;
    button.cfg.number = GPIO_PIN_13;
    button.cfg.mode = GPIO_MODE_INTERRUPT_FT;
    button.cfg.speed = GPIO_SPEED_FAST;
    button.cfg.pull_mode = GPIO_PULL_MODE_NONE;

    gpio_enable_peripheral_clock(button.p_reg, true);
    gpio_init(&button);

    /* Configures IRQ */
    gpio_config_irq_priority(IRQ_NUMBER_EXTI15_10, NVIC_IRQ_PRIORITY_15);
    gpio_config_irq(IRQ_NUMBER_EXTI15_10, true);

    while (1)
        ;

    return 0;
}

void EXTI15_10_IRQHandler()
{
    utils_delay();
    gpio_irq_handling(button.cfg.number);
    gpio_toggle_pin(&led);
}
#elif MAIN == MAIN_006_SPI_SEND
/**
 * SPICLK - PA9 - D8
 * SPIMOSI - PB15 - H26
 * NSS - PB4 - D5
 */

typedef struct
{
    gpio_handle_t clock;
    gpio_handle_t mosi;
    gpio_handle_t nss;
} Spi_pins_t;

static void init_gpio(Spi_pins_t *pins)
{
    pins->clock.p_reg = GPIOA;
    pins->clock.cfg.number = GPIO_PIN_9;
    pins->clock.cfg.mode = GPIO_MODE_ALT_FN;
    pins->clock.cfg.alt_fun_mode = GPIO_ALTERNATE_FUNCTION_5;
    pins->clock.cfg.out_type = GPIO_OUT_TYPE_PUSH_PULL;
    pins->clock.cfg.pull_mode = GPIO_PULL_MODE_NONE;
    gpio_init(&pins->clock);

    pins->mosi.p_reg = GPIOB;
    pins->mosi.cfg.number = GPIO_PIN_15;
    pins->mosi.cfg.mode = GPIO_MODE_ALT_FN;
    pins->mosi.cfg.alt_fun_mode = GPIO_ALTERNATE_FUNCTION_5;
    pins->mosi.cfg.out_type = GPIO_OUT_TYPE_PUSH_PULL;
    pins->mosi.cfg.pull_mode = GPIO_PULL_MODE_NONE;
    gpio_init(&pins->mosi);

    // pins->nss.p_reg = GPIOB;
    // pins->nss.cfg.number = GPIO_PIN_4;
    // pins->nss.cfg.mode = GPIO_MODE_ALT_FN;
    // pins->nss.cfg.alt_fun_mode = GPIO_ALTERNATE_FUNCTION_7;
    // pins->nss.cfg.out_type = GPIO_OUT_TYPE_PUSH_PULL;
    // pins->nss.cfg.pull_mode = GPIO_PULL_MODE_NONE;
    // gpio_init(pins->nss.p_reg);
}

void init_spi(Spi_handle_t *handle)
{
    handle->p_reg = SPI1;
    handle->cfg.device_mode = SPI_DEVICE_MODE_MASTER;
    handle->cfg.bus_config = SPI_BUS_CONFIG_FULL_DUPLEX;
    handle->cfg.sclk_speed = SPI_CLOCK_SPEED_DIV_2;
    handle->cfg.DFF = SPI_DFF_8_BITS;
    handle->cfg.CPOL = SPI_CPOL_LOW;
    handle->cfg.CPHA = SPI_CPHA_LOW;
    handle->cfg.SSM = SPI_SSM_ENABLED;

    Spi_init(handle);
}

int main()
{
    Spi_pins_t gpio_spi = {0u};
    init_gpio(&gpio_spi);

    Spi_handle_t spi = {0u};
    init_spi(&spi);
    Spi_enable_ssi(spi.p_reg, true);
    Spi_enable_peripheral(spi.p_reg, true);

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
    gpio_handle_t clock;
    gpio_handle_t mosi;
    gpio_handle_t nss;
} Spi_pins_t;

static void init_gpio(Spi_pins_t *pins)
{
    pins->clock.p_reg = GPIOA;
    pins->clock.cfg.number = GPIO_PIN_9;
    pins->clock.cfg.mode = GPIO_MODE_ALT_FN;
    pins->clock.cfg.alt_fun_mode = GPIO_ALTERNATE_FUNCTION_5;
    pins->clock.cfg.out_type = GPIO_OUT_TYPE_PUSH_PULL;
    pins->clock.cfg.pull_mode = GPIO_PULL_MODE_UP;
    gpio_init(&pins->clock);

    pins->mosi.p_reg = GPIOB;
    pins->mosi.cfg.number = GPIO_PIN_15;
    pins->mosi.cfg.mode = GPIO_MODE_ALT_FN;
    pins->mosi.cfg.alt_fun_mode = GPIO_ALTERNATE_FUNCTION_5;
    pins->mosi.cfg.out_type = GPIO_OUT_TYPE_PUSH_PULL;
    pins->mosi.cfg.pull_mode = GPIO_PULL_MODE_UP;
    gpio_init(&pins->mosi);

    pins->nss.p_reg = GPIOB;
    pins->nss.cfg.number = GPIO_PIN_4;
    pins->nss.cfg.mode = GPIO_MODE_ALT_FN;
    pins->nss.cfg.alt_fun_mode = GPIO_ALTERNATE_FUNCTION_7;
    pins->nss.cfg.out_type = GPIO_OUT_TYPE_PUSH_PULL;
    pins->nss.cfg.pull_mode = GPIO_PULL_MODE_UP;
    gpio_init(&pins->nss);
}

void init_spi(Spi_handle_t *handle)
{
    handle->p_reg = SPI2;
    handle->cfg.device_mode = SPI_DEVICE_MODE_MASTER;
    handle->cfg.bus_config = SPI_BUS_CONFIG_FULL_DUPLEX;
    handle->cfg.sclk_speed = SPI_CLOCK_SPEED_DIV_8;
    handle->cfg.DFF = SPI_DFF_8_BITS;
    handle->cfg.CPOL = SPI_CPOL_LOW;
    handle->cfg.CPHA = SPI_CPHA_LOW;
    handle->cfg.SSM = SPI_SSM_DISABLED;

    Spi_init(handle);
}

static void init_button(gpio_handle_t *button)
{
    button->p_reg = GPIOC;
    button->cfg.number = GPIO_PIN_13;
    button->cfg.mode = GPIO_MODE_IN;
    button->cfg.speed = GPIO_SPEED_FAST;
    button->cfg.pull_mode = GPIO_PULL_MODE_NONE;

    gpio_init(button);
}

int main()
{
    char user_data[] = "hello world";
    Spi_pins_t gpio_spi = {0u};
    init_gpio(&gpio_spi);

    Spi_handle_t spi = {0u};
    init_spi(&spi);
    Spi_enable_ssoe(spi.p_reg, true);

    gpio_handle_t button;
    init_button(&button);

    while (1)
    {
        while (GPIO_BUTTON_STATE_HIGH == gpio_read_pin(&button))
        {
        }
        utils_delay();

        Spi_enable_peripheral(spi.p_reg, true);
        uint8_t length = strlen(user_data);
        spi.tx.data = &length;
        spi.tx.size = sizeof(length);
        Spi_send(&spi);

        spi.tx.data = (uint8_t *)user_data;
        spi.tx.size = length;
        Spi_send(&spi);
        while (spi.p_reg->SR.BSY)
        {
        }

        Spi_enable_peripheral(spi.p_reg, false);
    }

    return 0;
}
#elif MAIN == MAIN_008_SPI_ARDUINO

typedef struct
{
    gpio_handle_t clock;
    gpio_handle_t mosi;
    gpio_handle_t miso;
    gpio_handle_t nss;
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
    pins->clock.p_reg = GPIOA;
    pins->clock.cfg.number = GPIO_PIN_9;
    pins->clock.cfg.mode = GPIO_MODE_ALT_FN;
    pins->clock.cfg.alt_fun_mode = GPIO_ALTERNATE_FUNCTION_5;
    pins->clock.cfg.out_type = GPIO_OUT_TYPE_PUSH_PULL;
    pins->clock.cfg.pull_mode = GPIO_PULL_MODE_UP;
    gpio_init(&pins->clock);

    pins->mosi.p_reg = GPIOB;
    pins->mosi.cfg.number = GPIO_PIN_15;
    pins->mosi.cfg.mode = GPIO_MODE_ALT_FN;
    pins->mosi.cfg.alt_fun_mode = GPIO_ALTERNATE_FUNCTION_5;
    pins->mosi.cfg.out_type = GPIO_OUT_TYPE_PUSH_PULL;
    pins->mosi.cfg.pull_mode = GPIO_PULL_MODE_UP;
    gpio_init(&pins->mosi);

    pins->miso.p_reg = GPIOC;
    pins->miso.cfg.number = GPIO_PIN_2;
    pins->miso.cfg.mode = GPIO_MODE_ALT_FN;
    pins->miso.cfg.alt_fun_mode = GPIO_ALTERNATE_FUNCTION_5;
    pins->miso.cfg.out_type = GPIO_OUT_TYPE_PUSH_PULL;
    pins->miso.cfg.pull_mode = GPIO_PULL_MODE_UP;
    gpio_init(&pins->miso);

    pins->nss.p_reg = GPIOB;
    pins->nss.cfg.number = GPIO_PIN_4;
    pins->nss.cfg.mode = GPIO_MODE_ALT_FN;
    pins->nss.cfg.alt_fun_mode = GPIO_ALTERNATE_FUNCTION_7;
    pins->nss.cfg.out_type = GPIO_OUT_TYPE_PUSH_PULL;
    pins->nss.cfg.pull_mode = GPIO_PULL_MODE_UP;
    gpio_init(&pins->nss);
}

void init_spi(Spi_handle_t *handle)
{
    handle->p_reg = SPI2;
    handle->cfg.device_mode = SPI_DEVICE_MODE_MASTER;
    handle->cfg.bus_config = SPI_BUS_CONFIG_FULL_DUPLEX;
    handle->cfg.sclk_speed = SPI_CLOCK_SPEED_DIV_8;
    handle->cfg.DFF = SPI_DFF_8_BITS;
    handle->cfg.CPOL = SPI_CPOL_LOW;
    handle->cfg.CPHA = SPI_CPHA_LOW;
    handle->cfg.SSM = SPI_SSM_DISABLED;

    Spi_init(handle);
}

static void init_button(gpio_handle_t *button)
{
    button->p_reg = GPIOC;
    button->cfg.number = GPIO_PIN_13;
    button->cfg.mode = GPIO_MODE_IN;
    button->cfg.speed = GPIO_SPEED_FAST;
    button->cfg.pull_mode = GPIO_PULL_MODE_NONE;

    gpio_init(button);
}

int main()
{
    initialise_monitor_handles();

    printf("hello man");
    Spi_pins_t gpio_spi = {0u};
    init_gpio(&gpio_spi);

    Spi_handle_t spi = {0u};
    init_spi(&spi);
    Spi_enable_ssoe(spi.p_reg, true);

    gpio_handle_t button;
    init_button(&button);

    while (true)
    {
        while (GPIO_BUTTON_STATE_HIGH == gpio_read_pin(&button))
        {
        }
        utils_delay();

        Spi_enable_peripheral(spi.p_reg, true);

        Arduino_write_led(&spi, ARDUINO_DIGITAL_STATUS_ON, LED_PIN);

        while (GPIO_BUTTON_STATE_HIGH == gpio_read_pin(&button))
        {
        }
        utils_delay();

        uint8_t value = Arduino_read_analog(&spi, ARDUINO_ANALOG_PIN_0);

        while (GPIO_BUTTON_STATE_HIGH == gpio_read_pin(&button))
        {
        }
        utils_delay();
        value = Arduino_read_digital(&spi, LED_PIN);

        while (GPIO_BUTTON_STATE_HIGH == gpio_read_pin(&button))
        {
        }
        utils_delay();
        Arduino_print(&spi, "hello Arduino");

        while (GPIO_BUTTON_STATE_HIGH == gpio_read_pin(&button))
        {
        }
        utils_delay();
        char arduino_id[ARDUINO_ID_SIZE + 1u] = {'\0'};
        Arduino_read_id(&spi, arduino_id, sizeof(arduino_id));

        while (spi.p_reg->SR.BSY)
        {
        }

        Spi_enable_peripheral(spi.p_reg, false);
    }

    return 0;
}

#elif MAIN == MAIN_010_I2C_MASTER_TX

static void init(const i2c_handle_t *p_handle);
static void init_pins(void);

#define ARDUINO_ADDRESS (0x68u)

int main()
{
    const i2c_handle_t i2c_handle = {
        .p_reg = I2C1,
        .cfg = {
            .ack_control = I2C_ACK_CONTROL_ENABLE,
            .device_address = 0x61u,
            .fm_duty_cycle = I2C_DUTY_2,
            .scl_speed = I2C_SCL_SPEED_STANDARD_MODE,
        },
    };

    init(&i2c_handle);
    uint8_t message[] = "hola man\n";

    for (;;)
    {
        while (!nucleo_is_button_pressed())
        {
        }
        utils_delay();

        i2c_send_as_master(&i2c_handle,
                           &(i2c_message_t){
                               .buffer = &message[0],
                               .size = sizeof(message),
                               .slave_address = ARDUINO_ADDRESS,
                           });
    }
}

static void init(const i2c_handle_t *p_handle)
{
    nucleo_init();
    init_pins();
    i2c_init(p_handle);
    i2c_enable_peripheral(p_handle->p_reg, true);
}

static void init_pins(void)
{
    const gpio_handle_t scl_pin = {
        .p_reg = GPIOB,
        .cfg = {
            .mode = GPIO_MODE_ALT_FN,
            .out_type = GPIO_OUT_TYPE_OPEN_DRAIN,
            .pull_mode = GPIO_PULL_MODE_UP,
            .alt_fun_mode = GPIO_ALTERNATE_FUNCTION_4,
            .number = GPIO_PIN_8,
            .speed = GPIO_SPEED_FAST,
        },
    };

    const gpio_handle_t sda_pin = {
        .p_reg = GPIOB,
        .cfg = {
            .mode = GPIO_MODE_ALT_FN,
            .out_type = GPIO_OUT_TYPE_OPEN_DRAIN,
            .pull_mode = GPIO_PULL_MODE_UP,
            .alt_fun_mode = GPIO_ALTERNATE_FUNCTION_4,
            .number = GPIO_PIN_9,
            .speed = GPIO_SPEED_FAST,
        },
    };

    /* SCL */
    gpio_init(&scl_pin);

    /* SDA */
    gpio_init(&sda_pin);
}

#endif
