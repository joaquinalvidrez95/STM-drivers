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
#define MAIN_011_I2C_MASTER_RX (8)
#define MAIN_012_I2C_MASTER_RX_IT (9)
#define MAIN_013_I2C_SLAVE_TX (10)

#define MAIN MAIN_013_I2C_SLAVE_TX

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

    gpio_enable_peripheral_clock(led.p_reg, true);
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
    gpio_config_irq_priority(IRQ_NUMBER_EXTI15_10, NVIC_IRQ_PRIO_15);
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

void init_spi(spi_handle_t *handle)
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

    spi_handle_t spi = {0u};
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

void init_spi(spi_handle_t *handle)
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

    spi_handle_t spi = {0u};
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

void init_spi(spi_handle_t *handle)
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

    spi_handle_t spi = {0u};
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

        arduino_write_led(&spi, ARDUINO_DIGITAL_STATUS_ON, LED_PIN);

        while (GPIO_BUTTON_STATE_HIGH == gpio_read_pin(&button))
        {
        }
        utils_delay();

        uint8_t value = arduino_read_analog(&spi, ARDUINO_ANALOG_PIN_0);

        while (GPIO_BUTTON_STATE_HIGH == gpio_read_pin(&button))
        {
        }
        utils_delay();
        value = arduino_read_digital(&spi, LED_PIN);

        while (GPIO_BUTTON_STATE_HIGH == gpio_read_pin(&button))
        {
        }
        utils_delay();
        arduino_print(&spi, "hello Arduino");

        while (GPIO_BUTTON_STATE_HIGH == gpio_read_pin(&button))
        {
        }
        utils_delay();
        char arduino_id[ARDUINO_ID_SIZE + 1u] = {'\0'};
        arduino_read_id(&spi, arduino_id, sizeof(arduino_id));

        while (spi.p_reg->SR.BSY)
        {
        }

        Spi_enable_peripheral(spi.p_reg, false);
    }

    return 0;
}

#elif MAIN == MAIN_010_I2C_MASTER_TX

static void init(void);

#define NUM_MESSAGES (3u)

int main()
{
    init();
    /* TODO: Try to make const */
    char messages[NUM_MESSAGES][10] = {"hola man\n", "middle", "bye"};
    uint8_t msg_idx = 0u;

    for (;;)
    {
        wait_till_button_pressed();

        i2c_transmit_as_master(NUCLEO_I2C_BUS,
                               &(i2c_msg_t){
                                   .buffer = (uint8_t *)&messages[msg_idx][0],
                                   .size = strlen(messages[msg_idx]),
                                   .slave_address = ARDUINO_I2C_ADDRESS,
                                   .repeated_start = I2C_ACK_CONTROL_DISABLED,
                               },
                               UTILS_MECHANISM_POLLING);
        msg_idx++;
        msg_idx %= NUM_MESSAGES;
    }
}

static void init(void)
{
    nucleo_init_button();
    nucleo_init_i2c(NULL);
}

#elif MAIN == MAIN_011_I2C_MASTER_RX

static void init(void);

#define BUFFER_SIZE (32u)

int main()
{
    init();

    uint8_t buffer[BUFFER_SIZE] = {0u};

    for (;;)
    {
        wait_till_button_pressed();

        const uint8_t msg_length = arduino_i2c_get_length(NUCLEO_I2C_BUS, UTILS_MECHANISM_POLLING);

        i2c_transmit_as_master(NUCLEO_I2C_BUS,
                               &(i2c_msg_t){
                                   .buffer = &(uint8_t){ARDUINO_I2C_COMMAND_READ_MSG},
                                   .size = sizeof(uint8_t),
                                   .slave_address = ARDUINO_I2C_ADDRESS,
                                   .repeated_start = I2C_ACK_CONTROL_DISABLED,
                               },
                               UTILS_MECHANISM_POLLING);

        i2c_receive_as_master(NUCLEO_I2C_BUS,
                              &(i2c_msg_t){
                                  .size = msg_length,
                                  .buffer = &buffer[0],
                                  .slave_address = ARDUINO_I2C_ADDRESS,
                                  .repeated_start = I2C_ACK_CONTROL_DISABLED,
                              },
                              UTILS_MECHANISM_POLLING);

        printf("Arduino's message: %s", &buffer[0]);
    }
}

static void init(void)
{
    initialise_monitor_handles();
    nucleo_init_button();
    nucleo_init_i2c(NULL);
}

#elif MAIN == MAIN_012_I2C_MASTER_RX_IT

static void init(void);
static void interrupt_callback(i2c_interrupt_t source);

#define BUFFER_SIZE (32u)

int main()
{
    init();

    uint8_t buffer[BUFFER_SIZE] = {0u};

    for (;;)
    {
        wait_till_button_pressed();

        const uint8_t msg_length = arduino_i2c_get_length(NUCLEO_I2C_BUS, UTILS_MECHANISM_INTERRUPT);

        i2c_transmit_as_master(NUCLEO_I2C_BUS,
                               &(i2c_msg_t){
                                   .buffer = &(uint8_t){ARDUINO_I2C_COMMAND_READ_MSG},
                                   .size = sizeof(uint8_t),
                                   .slave_address = ARDUINO_I2C_ADDRESS,
                                   .repeated_start = I2C_ACK_CONTROL_ENABLED,
                               },
                               UTILS_MECHANISM_INTERRUPT);

        while (!i2c_is_interrupt_rx_tx_done(NUCLEO_I2C_BUS))
        {
        }

        i2c_receive_as_master(NUCLEO_I2C_BUS,
                              &(i2c_msg_t){
                                  .size = msg_length,
                                  .buffer = &buffer[0],
                                  .slave_address = ARDUINO_I2C_ADDRESS,
                                  .repeated_start = I2C_ACK_CONTROL_DISABLED,
                              },
                              UTILS_MECHANISM_INTERRUPT);

        while (!i2c_is_interrupt_rx_tx_done(NUCLEO_I2C_BUS))
        {
        }
        printf("Arduino's message: %s", &buffer[0]);
    }
}

static void init(void)
{
    initialise_monitor_handles();
    nucleo_init_button();
    i2c_set_irq_enabled(NUCLEO_I2C_BUS, I2C_IRQ_EV, true);
    i2c_set_irq_enabled(NUCLEO_I2C_BUS, I2C_IRQ_ERR, true);
    nucleo_init_i2c(interrupt_callback);
}

void I2C1_EV_IRQHandler(void)
{
    i2c_handle_ev_irq(NUCLEO_I2C_BUS);
}

void I2C1_ER_IRQHandler(void)
{
    i2c_handle_err_irq(NUCLEO_I2C_BUS);
}

static void interrupt_callback(i2c_interrupt_t source)
{
    switch (source)
    {
    case I2C_INTERRUPT_EV_TX_DONE:
        printf("Tx is done\n");
        break;
    case I2C_INTERRUPT_EV_RX_DONE:
        printf("Rx is done\n");
        break;
    case I2C_INTERRUPT_ERR_AF:
        printf("Error: ACK failure\n");
        for (;;)
        {
        }
        break;

    default:
        break;
    }
}

#elif MAIN == MAIN_013_I2C_SLAVE_TX

static void init(void);
static void interrupt_callback(i2c_interrupt_t source);

static const volatile char *tx_buffer = "Hello man...";

int main()
{
    init();
    for (;;)
    {
    }
}

static void init(void)
{
    initialise_monitor_handles();
    nucleo_init_button();
    i2c_set_irq_enabled(NUCLEO_I2C_BUS, I2C_IRQ_EV, true);
    i2c_set_irq_enabled(NUCLEO_I2C_BUS, I2C_IRQ_ERR, true);
    nucleo_init_i2c(interrupt_callback);
    i2c_set_interrupts_enabled(NUCLEO_I2C_BUS, true);
}

void I2C1_EV_IRQHandler(void)
{
    i2c_handle_ev_irq(NUCLEO_I2C_BUS);
}

void I2C1_ER_IRQHandler(void)
{
    i2c_handle_err_irq(NUCLEO_I2C_BUS);
}

static void interrupt_callback(i2c_interrupt_t source)
{
    static volatile uint8_t command = 0u;
    static volatile uint8_t tx_buf_idx = 0u;

    switch (source)
    {
    case I2C_INTERRUPT_EV_SLAVE_TXE:
        switch (command)
        {
        case ARDUINO_I2C_COMMAND_READ_LENGTH:
            i2c_transmit_as_slave(NUCLEO_I2C_BUS, strlen(tx_buffer));
            break;

        case ARDUINO_I2C_COMMAND_READ_MSG:
            i2c_transmit_as_slave(NUCLEO_I2C_BUS, tx_buffer[tx_buf_idx]);
            tx_buf_idx++;
            break;

        default:
            break;
        }

        break;

    case I2C_INTERRUPT_EV_SLAVE_RXNE:
        command = i2c_receive_as_slave(NUCLEO_I2C_BUS);
        break;

    case I2C_INTERRUPT_ERR_AF:
        command = 0u;
        tx_buf_idx = 0u;
        break;

    case I2C_INTERRUPT_EV_STOP:
        break;

    default:
        break;
    }
}

#endif
