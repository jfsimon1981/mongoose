// Copyright (c) 2022 Cesanta Software Limited
// All rights reserved
// https://www.st.com/resource/en/reference_manual/dm00031020-stm32f405-415-stm32f407-417-stm32f427-437-and-stm32f429-439-advanced-arm-based-32-bit-mcus-stmicroelectronics.pdf

#pragma once

#include <stm32f429xx.h>

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define BIT(x) (1UL << (x))
#define SETBITS(R, CLEARMASK, SETMASK) (R) = ((R) & ~(CLEARMASK)) | (SETMASK)
#define PIN(bank, num) ((((bank) - 'A') << 8) | (num))
#define PINNO(pin) (pin & 255)
#define PINBANK(pin) (pin >> 8)

// 6.3.3: APB1 clock <= 45MHz; APB2 clock <= 90MHz
// 3.5.1, Table 11: configure flash latency (WS) in accordance to clock freq
// 33.4: The AHB clock must be at least 25 MHz when Ethernet is used
enum { APB1_PRE = 5 /* AHB clock / 4 */, APB2_PRE = 4 /* AHB clock / 2 */ };
enum { PLL_HSI = 16, PLL_M = 8, PLL_N = 180, PLL_P = 2 };  // Run at 180 Mhz
#define FLASH_LATENCY 5
#define SYS_FREQUENCY ((PLL_HSI * PLL_N / PLL_M / PLL_P) * 1000000)
#define APB2_FREQUENCY (SYS_FREQUENCY / (BIT(APB2_PRE - 3)))
#define APB1_FREQUENCY (SYS_FREQUENCY / (BIT(APB1_PRE - 3)))

static inline void spin(volatile uint32_t count) {
  while (count--) (void) 0;
}

enum { GPIO_MODE_INPUT, GPIO_MODE_OUTPUT, GPIO_MODE_AF, GPIO_MODE_ANALOG };
enum { GPIO_OTYPE_PUSH_PULL, GPIO_OTYPE_OPEN_DRAIN };
enum { GPIO_SPEED_LOW, GPIO_SPEED_MEDIUM, GPIO_SPEED_HIGH, GPIO_SPEED_INSANE };
enum { GPIO_PULL_NONE, GPIO_PULL_UP, GPIO_PULL_DOWN };
#define GPIO(N) ((GPIO_TypeDef *) (0x40020000 + 0x400 * (N)))

static GPIO_TypeDef *gpio_bank(uint16_t pin) { return GPIO(PINBANK(pin)); }
static inline void gpio_toggle(uint16_t pin) {
  GPIO_TypeDef *gpio = gpio_bank(pin);
  uint32_t mask = BIT(PINNO(pin));
  gpio->BSRR = mask << (gpio->ODR & mask ? 16 : 0);
}
static inline int gpio_read(uint16_t pin) {
  return gpio_bank(pin)->IDR & BIT(PINNO(pin)) ? 1 : 0;
}
static inline void gpio_write(uint16_t pin, bool val) {
  GPIO_TypeDef *gpio = gpio_bank(pin);
  gpio->BSRR = BIT(PINNO(pin)) << (val ? 0 : 16);
}
static inline void gpio_init(uint16_t pin, uint8_t mode, uint8_t type,
                             uint8_t speed, uint8_t pull, uint8_t af) {
  GPIO_TypeDef *gpio = gpio_bank(pin);
  uint8_t n = (uint8_t) (PINNO(pin));
  RCC->AHB1ENR |= BIT(PINBANK(pin));  // Enable GPIO clock
  SETBITS(gpio->OTYPER, 1UL << n, ((uint32_t) type) << n);
  SETBITS(gpio->OSPEEDR, 3UL << (n * 2), ((uint32_t) speed) << (n * 2));
  SETBITS(gpio->PUPDR, 3UL << (n * 2), ((uint32_t) pull) << (n * 2));
  SETBITS(gpio->AFR[n >> 3], 15UL << ((n & 7) * 4),
          ((uint32_t) af) << ((n & 7) * 4));
  SETBITS(gpio->MODER, 3UL << (n * 2), ((uint32_t) mode) << (n * 2));
}
static inline void gpio_input(uint16_t pin) {
  gpio_init(pin, GPIO_MODE_INPUT, GPIO_OTYPE_PUSH_PULL, GPIO_SPEED_HIGH,
            GPIO_PULL_NONE, 0);
}
static inline void gpio_output(uint16_t pin) {
  gpio_init(pin, GPIO_MODE_OUTPUT, GPIO_OTYPE_PUSH_PULL, GPIO_SPEED_HIGH,
            GPIO_PULL_NONE, 0);
}

#if 0
struct syscfg {
  volatile uint32_t MEMRMP, PMC, EXTICR[4], RESERVED[2], CMPCR;
};
#define SYSCFG ((struct syscfg *) 0x40013800)

struct exti {
  volatile uint32_t IMR, EMR, RTSR, FTSR, SWIER, PR;
};
#define EXTI ((struct exti *) 0x40013c00)
#endif

static inline void irq_exti_attach(uint16_t pin) {
  uint8_t bank = (uint8_t) (PINBANK(pin)), n = (uint8_t) (PINNO(pin));
  RCC->APB2ENR |= BIT(14);  // Enable SYSCFG
  SYSCFG->EXTICR[n / 4] &= ~(15UL << ((n % 4) * 4));
  SYSCFG->EXTICR[n / 4] |= (uint32_t) (bank << ((n % 4) * 4));
  EXTI->IMR |= BIT(n);
  EXTI->RTSR |= BIT(n);
  EXTI->FTSR |= BIT(n);
  int irqvec = n < 5 ? 6 + n : n < 10 ? 23 : 40;  // IRQ vector index, 10.1.2
  NVIC_SetPriority(irqvec, 3);
  NVIC_EnableIRQ(irqvec);
}

#if 0
USART_TypeDef {
  volatile uint32_t SR, DR, BRR, CR1, CR2, CR3, GTPR;
};
#define UART1 ((USART_TypeDef *) 0x40011000)
#define UART2 ((USART_TypeDef *) 0x40004400)
#define UART3 ((USART_TypeDef *) 0x40004800)
#endif

#define UART1 USART1
#define UART2 USART2
#define UART3 USART3

#ifndef UART_DEBUG
#define UART_DEBUG UART3
#endif

static inline void uart_init(USART_TypeDef *uart, unsigned long baud) {
  // https://www.st.com/resource/en/datasheet/stm32f429zi.pdf
  uint8_t af = 7;           // Alternate function
  uint16_t rx = 0, tx = 0;  // pins
  uint32_t freq = 0;        // Bus frequency. UART1 is on APB2, rest on APB1

  if (uart == UART1) freq = APB2_FREQUENCY, RCC->APB2ENR |= BIT(4);
  if (uart == UART2) freq = APB1_FREQUENCY, RCC->APB1ENR |= BIT(17);
  if (uart == UART3) freq = APB1_FREQUENCY, RCC->APB1ENR |= BIT(18);

  if (uart == UART1) tx = PIN('A', 9), rx = PIN('A', 10);
  if (uart == UART2) tx = PIN('A', 2), rx = PIN('A', 3);
  if (uart == UART3) tx = PIN('D', 8), rx = PIN('D', 9);

  gpio_init(tx, GPIO_MODE_AF, GPIO_OTYPE_PUSH_PULL, GPIO_SPEED_HIGH, 0, af);
  gpio_init(rx, GPIO_MODE_AF, GPIO_OTYPE_PUSH_PULL, GPIO_SPEED_HIGH, 0, af);
  uart->CR1 = 0;                           // Disable this UART
  uart->BRR = freq / baud;                 // Set baud rate
  uart->CR1 |= BIT(13) | BIT(2) | BIT(3);  // Set UE, RE, TE
}
static inline void uart_write_byte(USART_TypeDef *uart, uint8_t byte) {
  uart->DR = byte;
  while ((uart->SR & BIT(7)) == 0) spin(1);
}
static inline void uart_write_buf(USART_TypeDef *uart, char *buf, size_t len) {
  while (len-- > 0) uart_write_byte(uart, *(uint8_t *) buf++);
}
static inline int uart_read_ready(USART_TypeDef *uart) {
  return uart->SR & BIT(5);  // If RXNE bit is set, data is ready
}
static inline uint8_t uart_read_byte(USART_TypeDef *uart) {
  return (uint8_t) (uart->DR & 255);
}

static inline void clock_init(void) {                 // Set clock frequency
  SCB->CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2));  // Enable FPU
  asm("DSB");
  asm("ISB");
  FLASH->ACR |= FLASH_LATENCY | BIT(8) | BIT(9);    // Flash latency
  RCC->PLLCFGR &= ~((BIT(17) - 1));                 // Clear PLL multipliers
  RCC->PLLCFGR |= (((PLL_P - 2) / 2) & 3) << 16;    // Set PLL_P
  RCC->PLLCFGR |= PLL_M | (PLL_N << 6);             // Set PLL_M and PLL_N
  RCC->CR |= BIT(24);                               // Enable PLL
  while ((RCC->CR & BIT(25)) == 0) spin(1);         // Wait until done
  RCC->CFGR = (APB1_PRE << 10) | (APB2_PRE << 13);  // Set prescalers
  RCC->CFGR |= 2;                                   // Set clock source to PLL
  while ((RCC->CFGR & 12) == 0) spin(1);            // Wait until done
}
