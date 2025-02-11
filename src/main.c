/* STM32L476 Discovery Kit - Bare-Metal Code */
/* GPS (SAM-M8Q) & LoRa (RYLR890) Communication */

#include "stm32l476xx.h"

/* UART Baud Rates */
#define UART1_BAUDRATE 9600    // GPS SAM-M8Q
#define UART2_BAUDRATE 115200  // LoRa RYLR890

/* Function Prototypes */
void clock_config(void);
void uart1_init(void);  // GPS UART1
void uart2_init(void);  // LoRa UART2
void ports_init(void);
void uart_send_char(USART_TypeDef *USARTx, char ch);
char uart_receive_char(USART_TypeDef *USARTx);
void uart_send_string(USART_TypeDef *USARTx, const char *str);
void lora_send_command(const char *cmd);
void gps_read_data(void);

/* Main Program */
int main(void) {
    clock_config();
    ports_init();
    uart1_init();  // Initialize GPS UART
    uart2_init();  // Initialize LoRa UART

    while (1) {
        gps_read_data();  // Read GPS Data
        lora_send_command("AT+SEND=1,5,Hello"); // Send LoRa message
        for (volatile int i = 0; i < 1000000; i++); // Delay
    }
}

/* System Clock Configuration */
void clock_config(void) {
    RCC->CR |= RCC_CR_HSION;  // Enable HSI16 clock
    while (!(RCC->CR & RCC_CR_HSIRDY)); // Wait until HSI is stable
    RCC->CFGR |= RCC_CFGR_SW_HSI; // Select HSI as system clock
    while (!(RCC->CFGR & RCC_CFGR_SWS_HSI)); // Wait for switch
}

/* GPIO Initialization */
void ports_init(void) {
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN; // Enable GPIOA Clock
    GPIOA->MODER &= ~(GPIO_MODER_MODE2 | GPIO_MODER_MODE3); // Set PA2, PA3 to alternate function
    GPIOA->AFR[0] |= (7 << GPIO_AFRL_AFSEL2_Pos) | (7 << GPIO_AFRL_AFSEL3_Pos); // AF7 (USART2)
}

/* UART1 Initialization (GPS) */
void uart1_init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN; // Enable USART1 Clock
    USART1->BRR = SystemCoreClock / UART1_BAUDRATE;
    USART1->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

/* UART2 Initialization (LoRa) */
void uart2_init(void) {
    RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN; // Enable USART2 Clock
    USART2->BRR = SystemCoreClock / UART2_BAUDRATE;
    USART2->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

/* Send a character via UART */
void uart_send_char(USART_TypeDef *USARTx, char ch) {
    while (!(USARTx->ISR & USART_ISR_TXE));
    USARTx->TDR = ch;
}

/* Receive a character via UART */
char uart_receive_char(USART_TypeDef *USARTx) {
    while (!(USARTx->ISR & USART_ISR_RXNE));
    return USARTx->RDR;
}

/* Send a string via UART */
void uart_send_string(USART_TypeDef *USARTx, const char *str) {
    while (*str) {
        uart_send_char(USARTx, *str++);
    }
}

/* Send AT Command to LoRa */
void lora_send_command(const char *cmd) {
    uart_send_string(USART2, cmd);
    uart_send_char(USART2, '\r'); // Send carriage return
}

/* Read Data from GPS */
void gps_read_data(void) {
    char c;
    while ((c = uart_receive_char(USART1)) != '\n') {
        uart_send_char(USART2, c); // Forward GPS data to LoRa (debugging)
    }
}
