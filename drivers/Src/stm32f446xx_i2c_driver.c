#include "stm32f446xx_i2c_driver.h"

#include <stdint.h>
#include "stm32f446xx_rcc_driver.h"

#define FAST_MODE (1u << 15u)
#define MASK_CCR (0xFFFu)
#define BIT_POSITION_CCR_DUTY (14u)

static inline void reset_register(uint32_t bit);
static inline uint32_t calculate_ccr(const i2c_config_t *p_config, uint32_t pclk1);

void i2c_init(i2c_handle_t *p_handle)
{
    /* Configures ACK control bit */
    p_handle->p_reg->CR1 |= (uint8_t)p_handle->config.ack_control << 10u;

    const uint32_t pclk1 = rcc_get_pclk1();

    /* Configures FREQ */
    p_handle->p_reg->CR2 = (pclk1 / 1000000u) & 0b111111u;

    /* Configures device address */
    p_handle->p_reg->OAR1 |= 1u << 14u;
    p_handle->p_reg->OAR1 |= (uint8_t)p_handle->config.device_address << 1u;

    /* Clock Control Register (CCR) */
    p_handle->p_reg->CCR = calculate_ccr(&p_handle->config, pclk1);

    
}
void i2c_enable_peripheral_clock(i2c_reg_t *p_reg, bool enable)
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

void i2c_enable_peripheral(i2c_reg_t *p_reg, bool enable)
{
    if (enable)
    {
        p_reg->CR1 |= (uint32_t)I2C_CR1_PE;
    }
    else
    {
        p_reg->CR1 &= ~((uint32_t)I2C_CR1_PE);
    }
}

void i2c_deinit(i2c_reg_t *p_reg)
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

static inline uint32_t calculate_ccr(const i2c_config_t *p_config, uint32_t pclk1)
{
    uint32_t ccr = 0u;

    if ((uint32_t)I2C_SCL_SPEED_STANDARD_MODE >= (uint32_t)p_config->scl_speed)
    {
        ccr = (pclk1 / 2u / (uint32_t)p_config->scl_speed) & MASK_CCR;
    }
    else
    {
        const uint32_t divisor = p_config->fm_duty_cycle == I2C_DUTY_2 ? 3u : 25u;
        ccr = (pclk1 / divisor / p_config->scl_speed) & MASK_CCR;
        ccr |= FAST_MODE;
        ccr |= (uint32_t)p_config->fm_duty_cycle << BIT_POSITION_CCR_DUTY;
    }

    return ccr;
}