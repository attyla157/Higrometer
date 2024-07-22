#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_uart.h"
#include "main.h"


#include <string.h>

extern TIM_HandleTypeDef htim11;

extern UART_HandleTypeDef huart2;
__weak int __io_putchar(int ch)
{
  if (ch == '\n') {
    __io_putchar('\r');
  }

  HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);

  return 1;
}

__weak void delay_us(uint32_t us)
{
  __HAL_TIM_SET_COUNTER(&htim11, 0);
  while (__HAL_TIM_GET_COUNTER(&htim11) < us) {}
}

__weak HAL_StatusTypeDef wire_reset(void)
{
  int rc;

  HAL_GPIO_WritePin(DS_GPIO_Port, DS_Pin, GPIO_PIN_RESET);
  delay_us(480);
  HAL_GPIO_WritePin(DS_GPIO_Port, DS_Pin, GPIO_PIN_SET);
  delay_us(70);
  rc = HAL_GPIO_ReadPin(DS_GPIO_Port, DS_Pin);
  delay_us(410);

  if (rc == 0)
    return HAL_OK;
  else
    return HAL_ERROR;
}
__weak void write_bit(int value)
{
  if (value) {
    HAL_GPIO_WritePin(DS_GPIO_Port, DS_Pin, GPIO_PIN_RESET);
    delay_us(6);
    HAL_GPIO_WritePin(DS_GPIO_Port, DS_Pin, GPIO_PIN_SET);
    delay_us(64);
  } else {
    HAL_GPIO_WritePin(DS_GPIO_Port, DS_Pin, GPIO_PIN_RESET);
    delay_us(60);
    HAL_GPIO_WritePin(DS_GPIO_Port, DS_Pin, GPIO_PIN_SET);
    delay_us(10);
  }
}
__weak int read_bit(void)
{
  int rc;
  HAL_GPIO_WritePin(DS_GPIO_Port, DS_Pin, GPIO_PIN_RESET);
  delay_us(6);
  HAL_GPIO_WritePin(DS_GPIO_Port, DS_Pin, GPIO_PIN_SET);
  delay_us(9);
  rc = HAL_GPIO_ReadPin(DS_GPIO_Port, DS_Pin);
  delay_us(55);
  return rc;
}
__weak void wire_write(uint8_t byte)
{
  int i;
  for (i = 0; i < 8; i++) {
    write_bit(byte & 0x01);
    byte >>= 1;
  }
}

__weak uint8_t wire_read(void)
{
  uint8_t value = 0;
  int i;
  for (i = 0; i < 8; i++) {
    value >>= 1;
    if (read_bit())
      value |= 0x80;
  }
  return value;
}
__weak uint8_t byte_crc(uint8_t crc, uint8_t byte)
{
  int i;
  for (i = 0; i < 8; i++) {
    uint8_t b = crc ^ byte;
    crc >>= 1;
    if (b & 0x01)
      crc ^= 0x8c;
    byte >>= 1;
  }
  return crc;
}

__weak uint8_t wire_crc(const uint8_t* data, int len)
{
  int i;
    uint8_t crc = 0;

    for (i = 0; i < len; i++)
      crc = byte_crc(crc, data[i]);

    return crc;
}

__weak float conevrt_temp(uint8_t* adr)
{
	uint16_t temp;
	memcpy(&temp, adr, sizeof(temp));
	return temp / 16.0f;
}
__weak float get_tempOneWire()
{
	 HAL_StatusTypeDef rc = wire_reset();

		   // wire_write(0x33);
		    wire_write(0xcc); //pominięcie adresownaia
		    wire_write(0x44); // start pomiaru

		    HAL_Delay(1000);

		    wire_reset();
		    wire_write(0xcc); //pominięcie adresownaia
		     wire_write(0xbe);// odczyt danycch z rom
		    int i;
		    uint8_t scratchpad[9];
		    for (i = 0; i < 9; i++)
		      scratchpad[i] = wire_read();

		    uint8_t crc = wire_crc(scratchpad, 8);
		    float Temp = conevrt_temp(scratchpad);
		    return Temp;
}
__weak float convert_volts(uint16_t measure)
{
	float volts =  (float)measure/4095 * 3.3;
	return volts;
}
