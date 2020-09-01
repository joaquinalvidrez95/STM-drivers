#include "stm32f446xx_i2c_driver.h"

#include <stdint.h>
#include <stdbool.h>
#include "stm32f446xx_rcc_driver.h"
#include "stm32f446xx_nvic_driver.h"
#include "utils.h"
#include <stddef.h>

#define CCR_FAST_STANDARD_MODE (1u << 15u)
#define MASK_CCR (0xFFFu)
#define BIT_POSITION_CCR_DUTY (14u)

#define SR1_SB (1u << 0u)
#define SR1_ADDR (1u << 1u)
#define SR1_BTF (1u << 2u)
#define SR1_ADD10 (1u << 3u)
#define SR1_STOPF (1u << 4u)
#define SR1_RXNE (1u << 6u)
#define SR1_TXE (1u << 7u)
#define SR1_BERR (1u << 8u)
#define SR1_ARLO (1u << 9u)
#define SR1_AF (1u << 10u)
#define SR1_OVR (1u << 11u)
#define SR1_TIMEOUT (1u << 14u)

#define CR1_PE (1u)
#define CR1_NOSTRETCH (1u << 7u)
#define CR1_START (1u << 8u)
#define CR1_STOP (1u << 9u)
#define CR1_ACK (1u << 10u)
#define CR1_POS (1u << 11u)
#define CR1_PEC (1u << 12u)
#define CR1_SWRST (1u << 15u)

#define CR2_FREQ (1u << 0u)
#define CR2_ITERREN (1u << 8u)
#define CR2_ITEVTEN (1u << 9u)
#define CR2_ITBUFEN (1u << 10u)

#define OAR1_ADD0 (1u << 0u)
#define OAR1_ADD71 (1u << 1u)
#define OAR1_ADD98 (1u << 8u)
#define OAR1_ADDMODE (1u << 15u)
#define BIT_POSITION_OAR1_ADD (1u)

#define CCR_CCR (1u << 0u)
#define CCR_DUTY (1u << 14u)

#define SR2_MSL (1u << 0u)
#define SR2_BUSY (1u << 1u)
#define SR2_TRA (1u << 2u)
#define SR2_GENCALL (1u << 4u)
#define SR2_DUALF (1u << 7u)

typedef enum
{
    OPERATION_READ,
    OPERATION_WRITE,
} operation_t;

typedef struct
{
    volatile i2c_msg_t *p_msg;
    volatile i2c_state_t state;
    volatile uint8_t msg_idx;
} irq_manager_t;

typedef struct
{
    irq_manager_t irq_mgr;
    const volatile i2c_cfg_t *p_cfg;
    volatile i2c_reg_t *const p_reg;
} i2c_handle_t;

static inline void reset_reg(uint32_t bit);
static inline uint16_t calculate_ccr(const i2c_cfg_t *p_cfg, uint32_t pclk1);
static inline void generate_start_condition(i2c_bus_t bus);
static inline void generate_blocking_start_condition(i2c_bus_t bus);
static inline void generate_stop_condition(i2c_bus_t bus);
static inline bool is_start_condition_generated(i2c_bus_t bus);
static inline bool is_address_phase_done(i2c_bus_t bus);
static inline void execute_address_phase(i2c_bus_t bus, uint8_t slave_address, operation_t operation);
static inline void execute_blocking_address_phase(i2c_bus_t bus, uint8_t slave_address, operation_t operation);
static inline void clear_addr(i2c_bus_t bus);
static inline bool is_tx_data_reg_empty(i2c_bus_t bus);
static inline bool is_rx_data_reg_not_empty(i2c_bus_t bus);
static inline void wait_until_tx_data_reg_empty(i2c_bus_t bus);
static inline bool is_byte_transfer_finished(i2c_bus_t bus);
static inline uint16_t calculate_rise_time(const i2c_cfg_t *p_cfg, uint32_t pclk1);
static inline void wait_until_rx_data_reg_not_empty(i2c_bus_t bus);
static inline void enable_buffer_interrupt(i2c_bus_t bus, bool b_enabled);
static inline void enable_event_interrupt(i2c_bus_t bus, bool b_enabled);
static inline void enable_err_interrupt(i2c_bus_t bus, bool b_enabled);
static inline void enable_interrupts(i2c_bus_t bus, bool b_enabled);
static inline void handle_err_interrupt(i2c_bus_t bus, i2c_interrupt_t it, uint16_t mask);
static inline bool is_err_interrupt_enabled(i2c_bus_t bus);
static inline bool has_bus_error(i2c_bus_t bus);
static inline void handle_start_condition_interrupt(i2c_bus_t bus);
static inline void handle_addr_interrupt(i2c_bus_t bus);
static inline void handle_byte_transfer_finished_interrupt(i2c_bus_t bus);
static inline void handle_txe_interrupt(i2c_bus_t bus);
static inline void handle_rxne_interrupt(i2c_bus_t bus);
static inline void handle_master_rxne_interrupt(i2c_bus_t bus);
static inline bool is_msg_done(i2c_bus_t bus);
static inline void close_operation_data(i2c_bus_t bus, operation_t operation);
static inline void clear_stopf(i2c_bus_t bus);
static inline bool is_master(i2c_bus_t bus);
static inline bool is_transmitter(i2c_bus_t bus);
static void setup_operation_with_interrupts(i2c_bus_t bus, i2c_msg_t *p_msg, operation_t operation);
static void transmit_as_master_with_polling(i2c_bus_t bus, const i2c_msg_t *p_msg);
static void receive_as_master_with_polling(i2c_bus_t bus, i2c_msg_t *p_msg);

static i2c_handle_t g_handles[I2C_BUS_TOTAL] = {
    [I2C_BUS_1] = {.p_reg = I2C1},
    [I2C_BUS_2] = {.p_reg = I2C2},
    [I2C_BUS_3] = {.p_reg = I2C3},
};

void i2c_init(const i2c_cfg_t *p_cfg)
{
    g_handles[p_cfg->bus].p_cfg = p_cfg;
    rcc_set_i2c_peripheral_clock_enabled(p_cfg->bus, true);

    /* Configures ACK control bit */
    i2c_set_ack(p_cfg->bus, p_cfg->ack_control);

    const uint32_t pclk1 = rcc_get_pclk1();

    /* Configures FREQ */
    g_handles[p_cfg->bus].p_reg->CR2 = (uint16_t)(pclk1 / 1000000u) & 0b111111u;

    /* Configures device address */
    g_handles[p_cfg->bus].p_reg->OAR1 |= ((uint16_t)p_cfg->device_address << BIT_POSITION_OAR1_ADD) | (1u << 14u);

    /* Clock Control Register (CCR) */
    g_handles[p_cfg->bus].p_reg->CCR = calculate_ccr(p_cfg, pclk1);
    g_handles[p_cfg->bus].p_reg->TRISE = calculate_rise_time(p_cfg, pclk1);
}

void i2c_transmit_as_master(i2c_bus_t bus, i2c_msg_t *p_msg, utils_mechanism_t mechanism)
{
    switch (mechanism)
    {
    case UTILS_MECHANISM_POLLING:
        transmit_as_master_with_polling(bus, p_msg);
        break;

    case UTILS_MECHANISM_INTERRUPT:
        setup_operation_with_interrupts(bus, p_msg, OPERATION_WRITE);
        break;

    default:
        break;
    }
}

void i2c_receive_as_master(i2c_bus_t bus, i2c_msg_t *p_msg, utils_mechanism_t mechanism)
{
    switch (mechanism)
    {
    case UTILS_MECHANISM_POLLING:
        receive_as_master_with_polling(bus, p_msg);
        break;

    case UTILS_MECHANISM_INTERRUPT:
        setup_operation_with_interrupts(bus, p_msg, OPERATION_READ);
        break;

    default:
        break;
    }
}

void i2c_set_irq_enabled(i2c_bus_t bus, i2c_irq_t irq, bool b_enabled)
{
    const nvic_irq_num_t irqs[I2C_BUS_TOTAL][I2C_IRQ_TOTAL] = {
        [I2C_BUS_1] = {[I2C_IRQ_EV] = NVIC_IRQ_NUM_I2C1_EV, [I2C_IRQ_ERR] = NVIC_IRQ_NUM_I2C1_ER},
        [I2C_BUS_2] = {[I2C_IRQ_EV] = NVIC_IRQ_NUM_I2C2_EV, [I2C_IRQ_ERR] = NVIC_IRQ_NUM_I2C2_ER},
        [I2C_BUS_3] = {[I2C_IRQ_EV] = NVIC_IRQ_NUM_I2C3_EV, [I2C_IRQ_ERR] = NVIC_IRQ_NUM_I2C3_ER},
    };
    nvic_set_irq_enabled(irqs[bus][irq], b_enabled);
}

void i2c_set_peripheral_enabled(i2c_bus_t bus, bool b_enabled)
{
    /* TODO: Use utils */
    if (b_enabled)
    {
        g_handles[bus].p_reg->CR1 |= (uint16_t)CR1_PE;
    }
    else
    {
        g_handles[bus].p_reg->CR1 &= ~((uint16_t)CR1_PE);
    }
}

void i2c_deinit(i2c_bus_t bus)
{
    const uint32_t bits[I2C_BUS_TOTAL] = {21u, 22u, 23u};
    reset_reg(1u << bits[bus]);
}

void i2c_set_ack(i2c_bus_t bus, i2c_ack_control_t ack)
{
    /* TODO: Use utils */
    if (I2C_ACK_CONTROL_ENABLED == ack)
    {
        g_handles[bus].p_reg->CR1 |= CR1_ACK;
    }
    else
    {
        g_handles[bus].p_reg->CR1 &= ~CR1_ACK;
    }
}

void i2c_handle_ev_irq(i2c_bus_t bus)
{
    if (utils_is_bit_set_u16(g_handles[bus].p_reg->CR2, CR2_ITEVTEN))
    {
        const bool is_buf_interrupt_enabled = utils_is_bit_set_u16(g_handles[bus].p_reg->CR2, CR2_ITBUFEN);

        if (is_start_condition_generated(bus))
        {
            handle_start_condition_interrupt(bus);
        }
        if (is_address_phase_done(bus))
        {
            handle_addr_interrupt(bus);
        }
        if (is_byte_transfer_finished(bus))
        {
            handle_byte_transfer_finished_interrupt(bus);
        }
        if (utils_is_bit_set_u16(g_handles[bus].p_reg->SR1, SR1_STOPF))
        {
            clear_stopf(bus);
            g_handles[bus].p_cfg->interrupt_cb(I2C_INTERRUPT_EV_STOP);
        }
        if (is_tx_data_reg_empty(bus) && is_buf_interrupt_enabled)
        {
            handle_txe_interrupt(bus);
        }
        if (is_rx_data_reg_not_empty(bus) && is_buf_interrupt_enabled)
        {
            handle_rxne_interrupt(bus);
        }
    }
}

void i2c_handle_err_irq(i2c_bus_t bus)
{
    if (is_err_interrupt_enabled(bus))
    {
        /***********************Check for Bus error************************************/
        if (has_bus_error(bus))
        {
            handle_err_interrupt(bus, I2C_INTERRUPT_ERR_BERR, SR1_BERR);
        }

        /***********************Check for arbitration lost error************************************/
        if (utils_is_bit_set_u16(g_handles[bus].p_reg->SR1, SR1_ARLO))
        {
            handle_err_interrupt(bus, I2C_INTERRUPT_ERR_ARLO, SR1_ARLO);
        }

        /***********************Check for ACK failure  error************************************/

        if (utils_is_bit_set_u16(g_handles[bus].p_reg->SR1, SR1_AF))
        {
            handle_err_interrupt(bus, I2C_INTERRUPT_ERR_AF, SR1_AF);
        }

        /***********************Check for Overrun/underrun error************************************/
        if (utils_is_bit_set_u16(g_handles[bus].p_reg->SR1, SR1_OVR))
        {
            handle_err_interrupt(bus, I2C_INTERRUPT_ERR_OVR, SR1_OVR);
        }

        /***********************Check for Time out error************************************/
        if (utils_is_bit_set_u16(g_handles[bus].p_reg->SR1, SR1_TIMEOUT))
        {
            handle_err_interrupt(bus, I2C_INTERRUPT_ERR_TIMEOUT, SR1_TIMEOUT);
        }
    }
}

bool i2c_is_interrupt_rx_tx_done(i2c_bus_t bus)
{
    return g_handles[bus].irq_mgr.state == I2C_STATE_READY;
}

static inline void wait_until_rx_data_reg_not_empty(i2c_bus_t bus)
{
    while (!is_rx_data_reg_not_empty(bus))
    {
    }
}

static inline void enable_buffer_interrupt(i2c_bus_t bus, bool b_enabled)
{
    utils_set_bit_u16(&g_handles[bus].p_reg->CR2, CR2_ITBUFEN, b_enabled);
}

static inline void enable_event_interrupt(i2c_bus_t bus, bool b_enabled)
{
    utils_set_bit_u16(&g_handles[bus].p_reg->CR2, CR2_ITEVTEN, b_enabled);
}

static inline void enable_err_interrupt(i2c_bus_t bus, bool b_enabled)
{
    utils_set_bit_u16(&g_handles[bus].p_reg->CR2, CR2_ITERREN, b_enabled);
}

static inline void enable_interrupts(i2c_bus_t bus, bool b_enabled)
{
    utils_set_bit_u16(&g_handles[bus].p_reg->CR2, CR2_ITBUFEN | CR2_ITEVTEN | CR2_ITERREN, b_enabled);
}

static inline void handle_err_interrupt(i2c_bus_t bus, i2c_interrupt_t it, uint16_t mask)
{
    /* Clears interrupt flag */
    g_handles[bus].p_reg->SR1 &= ~mask;

    close_operation_data(bus, OPERATION_WRITE);
    generate_stop_condition(bus);

    if (NULL != g_handles[bus].p_cfg->interrupt_cb)
    {
        g_handles[bus].p_cfg->interrupt_cb(it);
    }
}

static inline bool is_err_interrupt_enabled(i2c_bus_t bus)
{
    return utils_is_bit_set_u16(g_handles[bus].p_reg->CR2, CR2_ITERREN);
}

static inline bool has_bus_error(i2c_bus_t bus)
{
    return utils_is_bit_set_u16(g_handles[bus].p_reg->SR1, SR1_BERR);
}

static inline void handle_start_condition_interrupt(i2c_bus_t bus)
{
    switch (g_handles[bus].irq_mgr.state)
    {
    case I2C_STATE_BUSY_IN_RX:
        execute_address_phase(bus, g_handles[bus].irq_mgr.p_msg->slave_address, OPERATION_READ);
        break;

    case I2C_STATE_BUSY_IN_TX:
        execute_address_phase(bus, g_handles[bus].irq_mgr.p_msg->slave_address, OPERATION_WRITE);
        break;

    default:
        break;
    }
}

static inline void reset_reg(uint32_t bit)
{
    RCC->APB_RSTR[0] |= bit;
    RCC->APB_RSTR[0] &= ~bit;
}

static inline uint16_t calculate_ccr(const i2c_cfg_t *p_cfg, uint32_t pclk1)
{
    uint16_t ccr = 0u;

    if ((uint32_t)I2C_SCL_SPEED_STANDARD_MODE >= (uint32_t)p_cfg->scl_speed)
    {
        ccr = (uint16_t)(((pclk1 / 2u) / (uint32_t)p_cfg->scl_speed) & MASK_CCR);
    }
    else
    {
        const uint32_t divisor = p_cfg->fm_duty_cycle == I2C_DUTY_2 ? 3u : 25u;
        ccr = (uint16_t)((pclk1 / divisor / p_cfg->scl_speed) & MASK_CCR);
        ccr |= CCR_FAST_STANDARD_MODE;
        ccr |= (uint16_t)p_cfg->fm_duty_cycle << BIT_POSITION_CCR_DUTY;
    }

    return ccr;
}

static inline void generate_start_condition(i2c_bus_t bus)
{
    utils_set_bit_u16(&g_handles[bus].p_reg->CR1, CR1_START, true);
}

static inline void generate_blocking_start_condition(i2c_bus_t bus)
{
    generate_start_condition(bus);

    while (!is_start_condition_generated(bus))
    {
    }
}

static inline void generate_stop_condition(i2c_bus_t bus)
{
    /* TODO: Use utils */
    g_handles[bus].p_reg->CR1 |= CR1_STOP;
}

static inline bool is_start_condition_generated(i2c_bus_t bus)
{
    return utils_is_bit_set_u16(g_handles[bus].p_reg->SR1, SR1_SB);
}

static inline void execute_address_phase(i2c_bus_t bus, uint8_t slave_address, operation_t operation)
{
    switch (operation)
    {
    case OPERATION_READ:
        g_handles[bus].p_reg->DR = (uint16_t)((slave_address << 1u) | 1u);
        break;

    case OPERATION_WRITE:
        g_handles[bus].p_reg->DR = (uint16_t)((slave_address << 1u) & (uint8_t)(~1u));
        break;

    default:
        break;
    }
}

static inline void execute_blocking_address_phase(i2c_bus_t bus, uint8_t slave_address, operation_t operation)
{
    execute_address_phase(bus, slave_address, operation);
    while (!is_address_phase_done(bus))
    {
    }
}

static inline bool is_address_phase_done(i2c_bus_t bus)
{
    return utils_is_bit_set_u16(g_handles[bus].p_reg->SR1, SR1_ADDR);
}

static inline void clear_addr(i2c_bus_t bus)
{
    uint16_t dummy_read = g_handles[bus].p_reg->SR1;
    dummy_read = g_handles[bus].p_reg->SR2;
    (void)dummy_read;
}

static inline bool is_tx_data_reg_empty(i2c_bus_t bus)
{
    return utils_is_bit_set_u16(g_handles[bus].p_reg->SR1, SR1_TXE);
}

static inline bool is_rx_data_reg_not_empty(i2c_bus_t bus)
{
    return utils_is_bit_set_u16(g_handles[bus].p_reg->SR1, SR1_RXNE);
}

static inline void wait_until_tx_data_reg_empty(i2c_bus_t bus)
{
    while (!is_tx_data_reg_empty(bus))
    {
    }
}

static inline bool is_byte_transfer_finished(i2c_bus_t bus)
{
    return utils_is_bit_set_u16(g_handles[bus].p_reg->SR1, SR1_BTF);
}

static inline uint16_t calculate_rise_time(const i2c_cfg_t *p_cfg, uint32_t pclk1)
{
    uint16_t rise_time_reg = 0u;
    if ((uint32_t)I2C_SCL_SPEED_STANDARD_MODE >= (uint32_t)p_cfg->scl_speed)
    {
        rise_time_reg = (uint16_t)(pclk1 / 1000000u) + 1u;
    }
    else
    {
        rise_time_reg = (uint16_t)(pclk1 * 3u / 10000000u) + 1u;
    }
    return rise_time_reg & 0x3Fu;
}

/* TODO: Refactor logic */
static inline void handle_addr_interrupt(i2c_bus_t bus)
{
    if (is_master(bus))
    {
        if (I2C_STATE_BUSY_IN_RX == g_handles[bus].irq_mgr.state)
        {
            if (1u == g_handles[bus].irq_mgr.p_msg->size)
            {
                i2c_set_ack(bus, I2C_ACK_CONTROL_DISABLED);
                clear_addr(bus);
            }
        }
        else
        {
            clear_addr(bus);
        }
    }
    else
    {
        clear_addr(bus);
    }
}

static inline void handle_byte_transfer_finished_interrupt(i2c_bus_t bus)
{
    if (((I2C_STATE_BUSY_IN_TX == g_handles[bus].irq_mgr.state) &&
         is_tx_data_reg_empty(bus)) &&
        is_msg_done(bus))
    {
        if (I2C_REPEATED_START_DISABLED == g_handles[bus].irq_mgr.p_msg->repeated_start)
        {
            generate_stop_condition(bus);
        }
        close_operation_data(bus, OPERATION_WRITE);
        g_handles[bus].p_cfg->interrupt_cb(I2C_INTERRUPT_EV_TX_DONE);
    }
}

static inline bool is_msg_done(i2c_bus_t bus)
{
    return g_handles[bus].irq_mgr.msg_idx >= g_handles[bus].irq_mgr.p_msg->size;
}

static inline void close_operation_data(i2c_bus_t bus, operation_t operation)
{

    enable_buffer_interrupt(bus, false);
    enable_event_interrupt(bus, false);
    g_handles[bus].irq_mgr.state = I2C_STATE_READY;
    g_handles[bus].irq_mgr.msg_idx = 0u;
    g_handles[bus].irq_mgr.p_msg = NULL;

    if ((OPERATION_READ == operation) && (I2C_ACK_CONTROL_ENABLED == g_handles[bus].p_cfg->ack_control))
    {
        i2c_set_ack(bus, I2C_ACK_CONTROL_ENABLED);
    }
}

static inline void clear_stopf(i2c_bus_t bus)
{
    g_handles[bus].p_reg->CR1 |= 0u;
}

static inline void handle_txe_interrupt(i2c_bus_t bus)
{
    if (is_master(bus))
    {
        if ((I2C_STATE_BUSY_IN_TX == g_handles[bus].irq_mgr.state) && (!is_msg_done(bus)))
        {
            g_handles[bus].p_reg->DR = g_handles[bus].irq_mgr.p_msg->buffer[g_handles[bus].irq_mgr.msg_idx];
            g_handles[bus].irq_mgr.msg_idx++;
        }
    }
    else if (is_transmitter(bus))
    {
        g_handles[bus].p_cfg->interrupt_cb(I2C_INTERRUPT_EV_DATA_REQ);
    }
    else
    {
    }
}

static inline void handle_rxne_interrupt(i2c_bus_t bus)
{
    if (is_master(bus))
    {
        if ((I2C_STATE_BUSY_IN_RX == g_handles[bus].irq_mgr.state) && (!is_msg_done(bus)))
        {
            handle_master_rxne_interrupt(bus);
        }
    }
    else if (!is_transmitter(bus))
    {
        g_handles[bus].p_cfg->interrupt_cb(I2C_INTERRUPT_EV_DATA_RCV);
    }
    else
    {
    }
}

/* TODO: Reuse code with normal reception */
static inline void handle_master_rxne_interrupt(i2c_bus_t bus)
{
    if (0u < g_handles[bus].irq_mgr.p_msg->size)
    {
        if ((1u < g_handles[bus].irq_mgr.p_msg->size) && ((g_handles[bus].irq_mgr.p_msg->size - 2u) == g_handles[bus].irq_mgr.msg_idx))
        {
            i2c_set_ack(bus, I2C_ACK_CONTROL_DISABLED);
        }
        g_handles[bus].irq_mgr.p_msg->buffer[g_handles->irq_mgr.msg_idx] = g_handles[bus].p_reg->DR;
        g_handles[bus].irq_mgr.msg_idx++;
    }

    if (is_msg_done(bus))
    {
        if (I2C_REPEATED_START_DISABLED == g_handles[bus].irq_mgr.p_msg->repeated_start)
        {
            generate_stop_condition(bus);
        }

        close_operation_data(bus, OPERATION_READ);

        g_handles[bus].p_cfg->interrupt_cb(I2C_INTERRUPT_EV_RX_DONE);
    }
}

static inline bool is_master(i2c_bus_t bus)
{
    return utils_is_bit_set_u16(g_handles[bus].p_reg->SR2, SR2_MSL);
}

static inline bool is_transmitter(i2c_bus_t bus)
{
    return utils_is_bit_set_u16(g_handles[bus].p_reg->SR2, SR2_TRA);
}

static void transmit_as_master_with_polling(i2c_bus_t bus, const i2c_msg_t *p_msg)
{
    generate_blocking_start_condition(bus);

    execute_blocking_address_phase(bus, p_msg->slave_address, OPERATION_WRITE);
    clear_addr(bus);

    for (uint8_t buf_idx = 0u; buf_idx < p_msg->size; buf_idx++)
    {
        wait_until_tx_data_reg_empty(bus);
        g_handles[bus].p_reg->DR = p_msg->buffer[buf_idx];
    }

    wait_until_tx_data_reg_empty(bus);

    while (!is_byte_transfer_finished(bus))
    {
    }
    if (I2C_REPEATED_START_DISABLED == p_msg->repeated_start)
    {
        generate_stop_condition(bus);
    }
}

static void receive_as_master_with_polling(i2c_bus_t bus, i2c_msg_t *p_msg)
{
    generate_blocking_start_condition(bus);

    execute_blocking_address_phase(bus, p_msg->slave_address, OPERATION_READ);

    if (1u == p_msg->size)
    {
        i2c_set_ack(bus, I2C_ACK_CONTROL_DISABLED);
        clear_addr(bus);
        wait_until_rx_data_reg_not_empty(bus);
        if (I2C_REPEATED_START_DISABLED == p_msg->repeated_start)
        {
            generate_stop_condition(bus);
        }
        p_msg->buffer[0] = g_handles[bus].p_reg->DR;
    }
    else if (1u < p_msg->size)
    {
        /* TODO: Refactor nested ifs */
        clear_addr(bus);
        for (uint8_t buf_idx = 0u; buf_idx < p_msg->size; buf_idx++)
        {
            wait_until_rx_data_reg_not_empty(bus);
            if ((p_msg->size - 2u) == buf_idx)
            {
                i2c_set_ack(bus, I2C_ACK_CONTROL_DISABLED);
                if (I2C_REPEATED_START_DISABLED == p_msg->repeated_start)
                {
                    generate_stop_condition(bus);
                }
            }
            p_msg->buffer[buf_idx] = g_handles[bus].p_reg->DR;
        }
    }
    else
    {
    }

    if (I2C_ACK_CONTROL_ENABLED == g_handles[bus].p_cfg->ack_control)
    {
        i2c_set_ack(bus, I2C_ACK_CONTROL_ENABLED);
    }
}

static void setup_operation_with_interrupts(i2c_bus_t bus, i2c_msg_t *p_msg, operation_t operation)
{
    if (I2C_STATE_READY == g_handles[bus].irq_mgr.state)
    {
        g_handles[bus].irq_mgr.p_msg = p_msg;
        g_handles[bus].irq_mgr.state = (OPERATION_WRITE == operation) ? I2C_STATE_BUSY_IN_TX : I2C_STATE_BUSY_IN_RX;
        generate_start_condition(bus);
        enable_interrupts(bus, true);
    }
}
