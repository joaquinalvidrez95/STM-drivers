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

#define BIT_POSITION_OAR1_ADD (1u)

#define CR2_FREQ (1u << 0u)
#define CR2_ITERREN (1u << 8u)
#define CR2_ITEVTEN (1u << 9u)
#define CR2_ITBUFEN (1u << 10u)

typedef enum
{
    I2C_OAR1_ADD0 = 0u,
    I2C_OAR1_ADD71 = 1u << 1u,
    I2C_OAR1_ADD98 = 1u << 8u,
    I2C_OAR1_ADDMODE = 1u << 15u,
} I2c_oar1_e;

typedef enum
{
    I2C_SR2_MSL = 1u << 0u,
    I2C_SR2_BUSY = 1u << 1u,
    I2C_SR2_TRA = 1u << 2u,
    I2C_SR2_GENCALL = 1u << 4u,
    I2C_SR2_DUALF = 1u << 7u,
} I2c_sr2_e;

typedef enum
{
    I2C_CCR_CCR = 1u << 0u,
    I2C_CCR_DUTY = 1u << 14u,
} I2c_ccr_e;

typedef enum
{
    ADDRESS_PHASE_READ,
    ADDRESS_PHASE_WRITE,
} address_phase_t;

typedef struct
{
    i2c_msg_t *p_msg;
    i2c_state_t state;
} irq_manager_t;

typedef struct
{
    irq_manager_t irq_mgr;
    const i2c_cfg_t *p_cfg;
} i2c_handle_t;

static inline void reset_reg(uint32_t bit);
static inline uint16_t calculate_ccr(const i2c_cfg_t *p_cfg, uint32_t pclk1);
static inline void generate_start_condition(volatile i2c_reg_t *p_reg);
static inline void generate_blocking_start_condition(volatile i2c_reg_t *p_reg);
static inline void generate_stop_condition(volatile i2c_reg_t *p_reg);
static inline bool is_start_condition_generated(const volatile i2c_reg_t *p_reg);
static inline bool is_address_phase_done(const volatile i2c_reg_t *p_reg);
static inline void execute_address_phase(volatile i2c_reg_t *p_reg, uint8_t slave_address, address_phase_t operation);
static inline void execute_blocking_address_phase(volatile i2c_reg_t *p_reg, uint8_t slave_address, address_phase_t operation);
static inline void clear_addr(const volatile i2c_reg_t *p_reg);
static inline bool is_tx_data_reg_empty(const volatile i2c_reg_t *p_reg);
static inline void wait_until_tx_data_reg_empty(const volatile i2c_reg_t *p_reg);
static inline bool is_byte_transfer_finished(const volatile i2c_reg_t *p_reg);
static inline uint16_t calculate_rise_time(const i2c_cfg_t *p_cfg, uint32_t pclk1);
static inline void wait_until_rx_data_reg_not_empty(const volatile i2c_reg_t *p_reg);
static inline void enable_buffer_interrupt(volatile i2c_reg_t *p_reg);
static inline void enable_event_interrupt(volatile i2c_reg_t *p_reg);
static inline void enable_error_interrupt(volatile i2c_reg_t *p_reg);
static inline void enable_interrupts(volatile i2c_reg_t *p_reg);
static void handle_err_irq(volatile i2c_handle_t *p_handle);
static inline void handle_err_bit(volatile i2c_handle_t *p_handle, i2c_interrupt_t irq, uint16_t mask);
static void handle_ev_irq(volatile i2c_handle_t *p_handle);
static inline bool is_err_interrupt_enabled(const volatile i2c_reg_t *p_reg);
static inline bool has_bus_error(const volatile i2c_reg_t *p_reg);
static inline void handle_start_condition_irq(volatile i2c_handle_t *p_handle);

static volatile i2c_reg_t *const gp_registers[I2C_BUS_TOTAL] = {
    [I2C_BUS_1] = I2C1,
    [I2C_BUS_2] = I2C2,
    [I2C_BUS_3] = I2C3,
};

static volatile i2c_handle_t g_handles[I2C_BUS_TOTAL] = {0u};

void i2c_init(const i2c_cfg_t *p_cfg)
{
    g_handles[p_cfg->bus].p_cfg = p_cfg;
    rcc_set_i2c_peripheral_clock_enabled(p_cfg->bus, true);

    /* Configures ACK control bit */
    i2c_set_ack(p_cfg->bus, p_cfg->ack_control);

    const uint32_t pclk1 = rcc_get_pclk1();

    /* Configures FREQ */
    gp_registers[p_cfg->bus]->CR2 = (uint16_t)(pclk1 / 1000000u) & 0b111111u;

    /* Configures device address */
    gp_registers[p_cfg->bus]->OAR1 |= ((uint16_t)p_cfg->device_address << BIT_POSITION_OAR1_ADD) | (1u << 14u);

    /* Clock Control Register (CCR) */
    gp_registers[p_cfg->bus]->CCR = calculate_ccr(p_cfg, pclk1);
    gp_registers[p_cfg->bus]->TRISE = calculate_rise_time(p_cfg, pclk1);
}

void i2c_transmit_as_master(i2c_bus_t bus, const i2c_msg_t *p_msg)
{
    generate_blocking_start_condition(gp_registers[bus]);

    execute_blocking_address_phase(gp_registers[bus], p_msg->slave_address, ADDRESS_PHASE_WRITE);
    clear_addr(gp_registers[bus]);

    for (uint8_t buf_idx = 0u; buf_idx < p_msg->size; buf_idx++)
    {
        wait_until_tx_data_reg_empty(gp_registers[bus]);
        gp_registers[bus]->DR = p_msg->buffer[buf_idx];
    }

    wait_until_tx_data_reg_empty(gp_registers[bus]);

    while (!is_byte_transfer_finished(gp_registers[bus]))
    {
    }
    if (!p_msg->repeated_start)
    {
        generate_stop_condition(gp_registers[bus]);
    }
}

void i2c_receive_as_master(i2c_bus_t bus, i2c_msg_t *p_msg)
{
    generate_blocking_start_condition(gp_registers[bus]);

    execute_blocking_address_phase(gp_registers[bus], p_msg->slave_address, ADDRESS_PHASE_READ);

    if (1u == p_msg->size)
    {
        i2c_set_ack(bus, I2C_ACK_CONTROL_DISABLED);
        clear_addr(gp_registers[bus]);
        wait_until_rx_data_reg_not_empty(gp_registers[bus]);
        if (!p_msg->repeated_start)
        {
            generate_stop_condition(gp_registers[bus]);
        }
        p_msg->buffer[0] = gp_registers[bus]->DR;
    }
    else if (1u < p_msg->size)
    {
        /* TODO: Refactor nested ifs */
        clear_addr(gp_registers[bus]);
        for (uint8_t buf_idx = 0u; buf_idx < p_msg->size; buf_idx++)
        {
            wait_until_rx_data_reg_not_empty(gp_registers[bus]);
            if ((p_msg->size - 2u) == buf_idx)
            {
                i2c_set_ack(bus, I2C_ACK_CONTROL_DISABLED);
                if (!p_msg->repeated_start)
                {
                    generate_stop_condition(gp_registers[bus]);
                }
            }
            p_msg->buffer[buf_idx] = gp_registers[bus]->DR;
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

void i2c_transmit_as_master_with_isr(i2c_bus_t bus, i2c_msg_t *p_msg)
{
    if (I2C_STATE_READY == g_handles[bus].irq_mgr.state)
    {
        g_handles[bus].irq_mgr.p_msg = p_msg;
        g_handles[bus].irq_mgr.state = I2C_STATE_BUSY_IN_TX;
        generate_start_condition(gp_registers[bus]);
        enable_buffer_interrupt(gp_registers[bus]);
    }
}

/* TODO: Reuse code Tx/Rx */
void i2c_receive_as_master_with_isr(i2c_bus_t bus, i2c_msg_t *p_msg)
{
    if (I2C_STATE_READY == g_handles[bus].irq_mgr.state)
    {
        g_handles[bus].irq_mgr.p_msg = p_msg;
        g_handles[bus].irq_mgr.state = I2C_STATE_BUSY_IN_RX;
        generate_start_condition(gp_registers[bus]);
        enable_buffer_interrupt(gp_registers[bus]);
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

void i2c_enable_peripheral(i2c_bus_t bus, bool b_enabled)
{
    if (b_enabled)
    {
        gp_registers[bus]->CR1 |= (uint16_t)CR1_PE;
    }
    else
    {
        gp_registers[bus]->CR1 &= ~((uint16_t)CR1_PE);
    }
}

void i2c_deinit(volatile i2c_reg_t *p_reg)
{
    if (p_reg == I2C1)
    {
        reset_reg(1u << 21u);
    }
    else if (p_reg == I2C2)
    {
        reset_reg(1u << 22u);
    }
    else if (p_reg == I2C3)
    {
        reset_reg(1u << 23u);
    }
    else
    {
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

static inline void generate_start_condition(volatile i2c_reg_t *p_reg)
{
    p_reg->CR1 |= CR1_START;
}

static inline void generate_blocking_start_condition(volatile i2c_reg_t *p_reg)
{
    generate_start_condition(p_reg);

    while (!is_start_condition_generated(p_reg))
    {
    }
}

static inline void generate_stop_condition(volatile i2c_reg_t *p_reg)
{
    p_reg->CR1 |= CR1_STOP;
}

static inline bool is_start_condition_generated(const volatile i2c_reg_t *p_reg)
{
    return utils_is_bit_set_u16(p_reg->SR1, SR1_SB);
}

static inline void execute_address_phase(volatile i2c_reg_t *p_reg, uint8_t slave_address, address_phase_t operation)
{
    switch (operation)
    {
    case ADDRESS_PHASE_READ:
        p_reg->DR = (uint16_t)((slave_address << 1u) | 1u);
        break;

    case ADDRESS_PHASE_WRITE:
        p_reg->DR = (uint16_t)((slave_address << 1u) & (uint8_t)(~1u));
        break;

    default:
        break;
    }
}

static inline void execute_blocking_address_phase(volatile i2c_reg_t *p_reg, uint8_t slave_address, address_phase_t operation)
{
    execute_address_phase(p_reg, slave_address, operation);
    while (!is_address_phase_done(p_reg))
    {
    }
}

static inline bool is_address_phase_done(const volatile i2c_reg_t *p_reg)
{
    return utils_is_bit_set_u16(p_reg->SR1, SR1_ADDR);
}

static inline void clear_addr(const volatile i2c_reg_t *p_reg)
{
    uint16_t dummy_read = p_reg->SR1;
    dummy_read = p_reg->SR2;
    (void)dummy_read;
}

static inline bool is_tx_data_reg_empty(const volatile i2c_reg_t *p_reg)
{
    return utils_is_bit_set_u16(p_reg->SR1, SR1_TXE);
}

static inline void wait_until_tx_data_reg_empty(const volatile i2c_reg_t *p_reg)
{
    while (!is_tx_data_reg_empty(p_reg))
    {
    }
}

static inline bool is_byte_transfer_finished(const volatile i2c_reg_t *p_reg)
{
    return utils_is_bit_set_u16(p_reg->SR1, SR1_BTF);
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

void i2c_set_ack(i2c_bus_t bus, i2c_ack_control_t ack)
{
    if (I2C_ACK_CONTROL_ENABLED == ack)
    {
        gp_registers[bus]->CR1 |= CR1_ACK;
    }
    else
    {
        gp_registers[bus]->CR1 &= ~CR1_ACK;
    }
}

static inline void wait_until_rx_data_reg_not_empty(const volatile i2c_reg_t *p_reg)
{
    while (!utils_is_bit_set_u16(p_reg->SR1, SR1_RXNE))
    {
    }
}

static inline void enable_buffer_interrupt(volatile i2c_reg_t *p_reg)
{
    p_reg->CR2 |= CR2_ITBUFEN;
}

static inline void enable_event_interrupt(volatile i2c_reg_t *p_reg)
{
    p_reg->CR2 |= CR2_ITEVTEN;
}

static inline void enable_error_interrupt(volatile i2c_reg_t *p_reg)
{
    p_reg->CR2 |= CR2_ITERREN;
}

static inline void enable_interrupts(volatile i2c_reg_t *p_reg)
{
    enable_buffer_interrupt(p_reg);
    enable_event_interrupt(p_reg);
    enable_error_interrupt(p_reg);
}

static void handle_ev_irq(volatile i2c_handle_t *p_handle)
{
    if (utils_is_bit_set_u16(gp_registers[p_handle->p_cfg->bus]->SR1, CR2_ITEVTEN))
    {
        if (is_start_condition_generated(gp_registers[p_handle->p_cfg->bus]))
        {
            handle_start_condition_irq(p_handle);
        }
    }
}

static inline void handle_err_bit(volatile i2c_handle_t *p_handle, i2c_interrupt_t irq, uint16_t mask)
{
    /* Clears interrupt flag */
    gp_registers[p_handle->p_cfg->bus]->SR1 &= ~mask;

    if (NULL != p_handle->p_cfg->irq_cb)
    {
        p_handle->p_cfg->irq_cb(I2C_INTERRUPT_ERR_BERR);
    }
}

/* TODO: Create handling function for every bit */
static void handle_err_irq(volatile i2c_handle_t *p_handle)
{
    if (is_err_interrupt_enabled(gp_registers[p_handle->p_cfg->bus]))
    {
        /***********************Check for Bus error************************************/
        if (has_bus_error(gp_registers[p_handle->p_cfg->bus]))
        {
            handle_err_bit(p_handle, I2C_INTERRUPT_ERR_BERR, SR1_BERR);
        }

        /***********************Check for arbitration lost error************************************/
        if (utils_is_bit_set_u16(gp_registers[p_handle->p_cfg->bus]->SR1, SR1_ARLO))
        {
            handle_err_bit(p_handle, I2C_INTERRUPT_ERR_ARLO, SR1_ARLO);
        }

        /***********************Check for ACK failure  error************************************/

        if (utils_is_bit_set_u16(gp_registers[p_handle->p_cfg->bus]->SR1, SR1_AF))
        {
            handle_err_bit(p_handle, I2C_INTERRUPT_ERR_AF, SR1_AF);
        }

        /***********************Check for Overrun/underrun error************************************/
        if (utils_is_bit_set_u16(gp_registers[p_handle->p_cfg->bus]->SR1, SR1_OVR))
        {
            handle_err_bit(p_handle, I2C_INTERRUPT_ERR_OVR, SR1_OVR);
        }

        /***********************Check for Time out error************************************/
        if (utils_is_bit_set_u16(gp_registers[p_handle->p_cfg->bus]->SR1, SR1_TIMEOUT))
        {
            handle_err_bit(p_handle, I2C_INTERRUPT_ERR_TIMEOUT, SR1_TIMEOUT);
        }
    }
}

static inline bool is_err_interrupt_enabled(const volatile i2c_reg_t *p_reg)
{
    return utils_is_bit_set_u16(p_reg->CR2, CR2_ITERREN);
}

static inline bool has_bus_error(const volatile i2c_reg_t *p_reg)
{
    return utils_is_bit_set_u16(p_reg->SR1, SR1_BERR);
}

void I2C1_ER_IRQHandler(void)
{
    handle_err_irq(&g_handles[I2C_BUS_1]);
}

void I2C1_EV_IRQHandler(void)
{
    handle_ev_irq(&g_handles[I2C_BUS_1]);
}

static inline void handle_start_condition_irq(volatile i2c_handle_t *p_handle)
{
    switch (p_handle->irq_mgr.state)
    {
    case I2C_STATE_BUSY_IN_RX:
        // execute_address_phase(p_handle->p_reg, ) break;

    case I2C_STATE_BUSY_IN_TX:
        /* code */
        break;

    default:
        break;
    }
}