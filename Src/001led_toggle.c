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

#define MAIN_001_LED_TOGGLE 0
#define MAIN_002_LED_BUTTON 1
#define MAIN_005_BUTTTON_INTERRUPT 2
#define MAIN_006_SPI_SEND 3
#define MAIN_007_SPI_TX_ONLY_ARDUINO 4
#define MAIN_008_SPI_ARDUINO 5

#define MAIN MAIN_008_SPI_ARDUINO

extern void initialise_monitor_handles();

void delay()
{
    for (uint32_t i = 0u; i < 500000u; i++)
    {
    }
}
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
        delay();
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
            delay();
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
    Gpio_config_irq_priority(irq_number_exti15_10, nvic_irq_priority_15);
    Gpio_config_irq(irq_number_exti15_10, true);

    while (1)
        ;

    return 0;
}

void EXTI15_10_IRQHandler()
{
    delay();
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
        delay();

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

typedef enum
{
    COMMAND_LED_CTRL = 0x50u,
    COMMAND_SENSOR_READ = 0x51u,
    COMMAND_LED_READ = 0x52u,
    COMMAND_PRINT = 0x53u,
    COMMAND_ID_READ = 0x54,
} Command_e;

typedef enum
{
    LED_STATUS_OFF = 0u,
    LED_STATUS_ON = 1u
} Led_status_e;

typedef enum
{
    ARDUINO_ANALOG_PIN_0 = 0u,
    ARDUINO_ANALOG_PIN_1 = 1u,
    ARDUINO_ANALOG_PIN_2 = 2u,
    ARDUINO_ANALOG_PIN_3 = 3u,
    ARDUINO_ANALOG_PIN_4 = 4u,
} Arduino_analog_pin_e;

typedef enum
{
    ARDUINO_DIGITAL_PIN_0 = 0u,
    ARDUINO_DIGITAL_PIN_1,
    ARDUINO_DIGITAL_PIN_2,
    ARDUINO_DIGITAL_PIN_3,
    ARDUINO_DIGITAL_PIN_4,
    ARDUINO_DIGITAL_PIN_5,
    ARDUINO_DIGITAL_PIN_6,
    ARDUINO_DIGITAL_PIN_7,
    ARDUINO_DIGITAL_PIN_8,
    ARDUINO_DIGITAL_PIN_9,
    ARDUINO_DIGITAL_PIN_10,
    ARDUINO_DIGITAL_PIN_11,
    ARDUINO_DIGITAL_PIN_12,
    ARDUINO_DIGITAL_PIN_13,
} Arduino_digital_pin_e;

#define LED_PIN ARDUINO_DIGITAL_PIN_9
#define ARDUINO_ID_SIZE 10u

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

static bool is_response_correct(uint8_t ack_byte)
{
    return ack_byte == 0xF5u;
}

static void read_dummy(Spi_handle_t *spi)
{
    /* Does dummy read to clear off the RXNE */
    uint8_t dummy_read = 0u;
    spi->rx.data = &dummy_read;
    spi->rx.size = sizeof(dummy_read);
    Spi_receive(spi);
}

static void send_dummy(Spi_handle_t *spi)
{
    /* Sends some dummy bits to fetch the response from the slave */
    uint8_t dummy_byte = 0xFFu;
    spi->tx.data = &dummy_byte;
    spi->tx.size = sizeof(dummy_byte);
    Spi_send(spi);
}

static bool verify_response(Spi_handle_t *spi)
{

    uint8_t ack_byte = 0u;
    spi->rx.data = &ack_byte;
    spi->rx.size = sizeof(ack_byte);
    Spi_receive(spi);

    return is_response_correct(ack_byte);
}
static bool send_command(Spi_handle_t *spi, Command_e command, uint8_t arguments[], size_t arguments_length)
{
    /* Command 1 */
    spi->tx.data = (uint8_t *)&command;
    spi->tx.size = sizeof(uint8_t);
    Spi_send(spi);

    read_dummy(spi);
    send_dummy(spi);

    const bool result = verify_response(spi);

    if (result && (arguments != NULL))
    {
        spi->tx.data = &arguments[0];
        spi->tx.size = arguments_length;
        Spi_send(spi);
    }

    return result;
}

static Led_status_e read_digital(Spi_handle_t *spi, Arduino_digital_pin_e pin)
{
    Led_status_e read = 0u;
    if (send_command(spi, COMMAND_LED_READ, (uint8_t *)&pin, sizeof(uint8_t)))
    {
        read_dummy(spi);
        delay();
        send_dummy(spi);

        spi->rx.data = (uint8_t *)&read;
        spi->rx.size = sizeof(uint8_t);
        Spi_receive(spi);
    }

    return read;
}

static void write_led(Spi_handle_t *spi, Led_status_e status, Arduino_digital_pin_e pin)
{
    uint8_t arguments[] = {(uint8_t)pin, (uint8_t)status};
    send_command(spi, COMMAND_LED_CTRL, arguments, sizeof(arguments));
}

static uint8_t read_analog(Spi_handle_t *spi, Arduino_analog_pin_e pin)
{
    uint8_t analog_read = 0u;

    if (send_command(spi, COMMAND_SENSOR_READ, (uint8_t *)&pin, sizeof(uint8_t)))
    {
        read_dummy(spi);
        delay();
        send_dummy(spi);

        spi->rx.data = &analog_read;
        spi->rx.size = sizeof(analog_read);
        Spi_receive(spi);
    }

    return analog_read;
}

static void arduino_print(Spi_handle_t *spi, const char message[])
{
    char new_message[UINT8_MAX + 1u] = {'\0'};
    size_t s = strlen(message);
    strcpy(new_message, (const char *)&s);
    strcat(new_message, message);
    send_command(spi, COMMAND_PRINT, (uint8_t *)new_message, strlen(new_message));
}

static void arduino_read_id(Spi_handle_t *spi, char id[], size_t length)
{
    if (send_command(spi, COMMAND_ID_READ, NULL, 0u))
    {
        spi->rx.size = sizeof(uint8_t);
        for (size_t i = 0u; i < length; i++)
        {
            send_dummy(spi);
            spi->rx.data = (uint8_t *)&id[i];
            Spi_receive(spi);
        }
    }
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
        delay();

        Spi_enable_peripheral(spi.reg, true);

        write_led(&spi, LED_STATUS_ON, LED_PIN);

        while (GPIO_BUTTON_STATE_HIGH == Gpio_read_from_input_pin(&button))
        {
        }
        delay();

        uint8_t value = read_analog(&spi, ARDUINO_ANALOG_PIN_0);

        while (GPIO_BUTTON_STATE_HIGH == Gpio_read_from_input_pin(&button))
        {
        }
        delay();
        value = read_digital(&spi, LED_PIN);

        while (GPIO_BUTTON_STATE_HIGH == Gpio_read_from_input_pin(&button))
        {
        }
        delay();
        arduino_print(&spi, "hello Arduino");

        while (GPIO_BUTTON_STATE_HIGH == Gpio_read_from_input_pin(&button))
        {
        }
        delay();
        char arduino_id[ARDUINO_ID_SIZE + 1u] = {'\0'};
        arduino_read_id(&spi, arduino_id, sizeof(arduino_id));

        while (spi.reg->SR.BSY)
        {
        }

        Spi_enable_peripheral(spi.reg, false);
    }

    return 0;
}

#endif
