#include "stm32f446xx_i2c_driver.h"

#include <stdint.h>
#include <stdbool.h>
#include "stm32f446xx_rcc_driver.h"

#define CCR_FAST_STANDARD_MODE (1u << 15u)
#define MASK_CCR (0xFFFu)
#define BIT_POSITION_CCR_DUTY (14u)

#define CR1_START (1u << 8u)
#define CR1_STOP (1u << 9u)

#define SR1_SB (1u << 0u)

#define CR1_PE (1u)
#define CR1_NOSTRETCH (1u << 7u)
#define CR1_ACK (1u << 10u)
#define CR1_POS (1u << 11u)
#define CR1_PEC (1u << 12u)
#define CR1_SWRST (1u << 15u)

typedef enum
{
    I2C_CR2_FREQ = 0u,
    I2C_CR2_ITERREN = 1u << 8u,
    I2C_CR2_ITEVTEN = 1u << 9u,
    I2C_CR2_ITBUFEN = 1u << 10u,
} I2c_cr2_e;

typedef enum
{
    I2C_OAR1_ADD0 = 0u,
    I2C_OAR1_ADD71 = 1u << 1u,
    I2C_OAR1_ADD98 = 1u << 8u,
    I2C_OAR1_ADDMODE = 1u << 15u,
} I2c_oar1_e;

#define SR1_ADDR (1u << 1u)
#define SR1_BTF (1u << 2u)
#define SR1_ADD10 (1u << 3u)
#define SR1_STOPF (1u << 4u)
#define SR1_RXNE (1u << 6u)
#define SR1_TXE (1u << 7u)
#define SR1_BERR (1u << 8u)
#define SR1_ARLO (1u << 9u)
#define SR1_AF (u << 10u)
#define SR1_OVR (u << 11u)
#define SR1_TIMEOUT (u << 14u)

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

static inline void reset_register(uint32_t bit);
static inline uint16_t calculate_ccr(const i2c_cfg_t *p_cfg, uint32_t pclk1);
static inline void generate_start_condition(volatile i2c_reg_t *p_reg);
static inline void generate_stop_condition(volatile i2c_reg_t *p_reg);
static inline bool is_start_condition_generated(const volatile i2c_reg_t *p_reg);
static inline bool is_address_phase_done(const volatile i2c_reg_t *p_reg);
static inline void execute_address_phase(volatile i2c_reg_t *p_reg, uint8_t slave_address, address_phase_t operation);
static inline void clear_addr(const volatile i2c_reg_t *p_reg);
static inline bool is_data_register_empty(const volatile i2c_reg_t *p_reg);
static inline bool is_byte_transfer_finished(const volatile i2c_reg_t *p_reg);
static inline uint16_t calculate_rise_time(const i2c_cfg_t *p_cfg, uint32_t pclk1);

void i2c_init(const i2c_handle_t *p_handle)
{
    i2c_enable_peripheral_clock(p_handle->p_reg, true);
    /* Configures ACK control bit */
    p_handle->p_reg->CR1 |= (uint8_t)p_handle->cfg.ack_control << 10u;

    const uint32_t pclk1 = rcc_get_pclk1();

    /* Configures FREQ */
    p_handle->p_reg->CR2 = (uint16_t)(pclk1 / 1000000u) & 0b111111u;

    /* Configures device address */
    p_handle->p_reg->OAR1 |= 1u << 14u;
    p_handle->p_reg->OAR1 |= (uint8_t)p_handle->cfg.device_address << 1u;

    /* Clock Control Register (CCR) */
    p_handle->p_reg->CCR = calculate_ccr(&p_handle->cfg, pclk1);
    p_handle->p_reg->TRISE = calculate_rise_time(&p_handle->cfg, pclk1);
}

void i2c_send_as_master(const i2c_handle_t *p_handle, const i2c_message_t *p_message)
{
    generate_start_condition(p_handle->p_reg);

    while (!is_start_condition_generated(p_handle->p_reg))
    {
    }

    execute_address_phase(p_handle->p_reg, p_message->slave_address, ADDRESS_PHASE_WRITE);

    while (!is_address_phase_done(p_handle->p_reg))
    {
    }

    clear_addr(p_handle->p_reg);

    for (uint8_t data_idx = 0u; data_idx < p_message->size; data_idx++)
    {
        while (!is_data_register_empty(p_handle->p_reg))
        {
            p_handle->p_reg->DR = p_message->buffer[data_idx];
        }
    }

    while (!is_data_register_empty(p_handle->p_reg))
    {
    }

    while (!is_byte_transfer_finished(p_handle->p_reg))
    {
    }
}

void i2c_enable_peripheral_clock(volatile i2c_reg_t *p_reg, bool enable)
{
    if (enable == true)
    {
        if (p_reg == I2C1)
        {
            I2C1_PCLK_EN();
        }
        else if (p_reg == I2C2)
        {
            I2C2_PCLK_EN();
        }
        else if (p_reg == I2C3)
        {
            I2C3_PCLK_EN();
        }
    }
    else
    {
        if (p_reg == I2C1)
        {
            I2C1_PCLK_DI();
        }
        else if (p_reg == I2C2)
        {
            I2C2_PCLK_DI();
        }
        else if (p_reg == I2C3)
        {
            I2C3_PCLK_DI();
        }
    }
}

void i2c_enable_peripheral(volatile i2c_reg_t *p_reg, bool enable)
{
    if (enable)
    {
        p_reg->CR1 |= (uint16_t)CR1_PE;
    }
    else
    {
        p_reg->CR1 &= ~((uint16_t)CR1_PE);
    }
}

void i2c_deinit(volatile i2c_reg_t *p_reg)
{
    if (p_reg == I2C1)
    {
        reset_register(1u << 21u);
    }
    else if (p_reg == I2C2)
    {
        reset_register(1u << 22u);
    }
    else if (p_reg == I2C3)
    {
        reset_register(1u << 23u);
    }
    else
    {
    }
}

static inline void reset_register(uint32_t bit)
{
    RCC->APB_RSTR[0] |= bit;
    RCC->APB_RSTR[0] &= ~bit;
}

static inline uint16_t calculate_ccr(const i2c_cfg_t *p_cfg, uint32_t pclk1)
{
    uint16_t ccr = 0u;

    if ((uint32_t)I2C_SCL_SPEED_STANDARD_MODE >= (uint32_t)p_cfg->scl_speed)
    {
        ccr = (uint16_t)((pclk1 / 2u / (uint32_t)p_cfg->scl_speed) & MASK_CCR);
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

static inline void generate_stop_condition(volatile i2c_reg_t *p_reg)
{
    p_reg->CR1 |= CR1_STOP;
}

static inline bool is_start_condition_generated(const volatile i2c_reg_t *p_reg)
{
    return (p_reg->SR1 & SR1_SB) == SR1_SB;
}

static inline void execute_address_phase(volatile i2c_reg_t *p_reg, uint8_t slave_address, address_phase_t operation)
{
    switch (operation)
    {
    case ADDRESS_PHASE_READ:
        p_reg->DR = (slave_address << 1u) | 1u;
        break;

    case ADDRESS_PHASE_WRITE:
        p_reg->DR = (slave_address << 1u) & (uint8_t)(~1u);
        break;

    default:
        break;
    }
}

static inline bool is_address_phase_done(const volatile i2c_reg_t *p_reg)
{
    return (p_reg->SR1 & SR1_ADDR) == SR1_ADDR;
}

static inline void clear_addr(const volatile i2c_reg_t *p_reg)
{
    uint16_t dummy_read = p_reg->SR1;
    dummy_read = p_reg->SR2;
    (void)dummy_read;
}

static inline bool is_data_register_empty(const volatile i2c_reg_t *p_reg)
{
    return (p_reg->SR1 & SR1_TXE) == SR1_TXE;
}

static inline bool is_byte_transfer_finished(const volatile i2c_reg_t *p_reg)
{
    return (p_reg->SR1 & SR1_BTF) == SR1_BTF;
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