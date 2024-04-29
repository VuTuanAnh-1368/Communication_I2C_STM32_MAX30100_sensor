#include "stm32f10x.h"

#define MAX30100_ADDR 0xAE // 7-bit I2C address of MAX30100, left shifted for the write, or +1 for read
#define REPORTING_PERIOD_MS 1000
#define I2C_TIMEOUT 10000
volatile uint32_t msTicks; 

void init_clock(void);
void init_GPIO(void);
void init_I2C1(void);
void USART1_Config(void);
void USART1_SendString(char *str);
uint32_t millis(void);
void SysTick_Handler(void);
void I2C_Start(void);
void I2C_Stop(void);
void I2C_Write(uint8_t data);
uint8_t I2C_Read_Ack(void);
uint8_t I2C_Read_Nack(void);
void writeRegister(uint8_t reg, uint8_t value);
uint8_t readRegister(uint8_t reg);


int main(void) {
    init_clock();
    init_GPIO();
    init_I2C1();
    USART1_Config();

    msTicks = 0; // Initialize millisecond tick count
    SysTick_Config(SystemCoreClock / 1000); // Configure SysTick for 1ms interrupt

    USART1_SendString("Initializing pulse oximeter..\r\n");

    // Attempt to write to a configuration register of MAX30100
    writeRegister(0x06, 0x03);  // Example: SpO2 configuration

    uint32_t lastReport = 0;

    while (1) {
        if (millis() - lastReport >= REPORTING_PERIOD_MS) {
            uint8_t heartRate = readRegister(0x02); // Register address for heart rate
            uint8_t spo2 = readRegister(0x03);      // Register address for SpO2

            char msg[80];
					 	sprintf(msg, "Heart rate: %d bpm / SpO2: %d%%\n", heartRate, spo2);
						USART1_SendString(msg);
            lastReport = millis();
        }
    }
}

void SysTick_Handler(void) {
    msTicks++; // Increment millisecond counter
}

uint32_t millis(void) {
    return msTicks;
}

void init_clock(void) {
    // Configure system clocks - Example for 72 MHz from 8 MHz crystal
    RCC->CR |= RCC_CR_HSEON; // Turn on the external crystal
    while(!(RCC->CR & RCC_CR_HSERDY)); // Wait for the external crystal to stabilize
    
    RCC->CFGR |= RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL9; // Set PLL source to HSE and multiply by 9
    RCC->CR |= RCC_CR_PLLON; // Enable the PLL
    while(!(RCC->CR & RCC_CR_PLLRDY)); // Wait for PLL to be ready
    
    RCC->CFGR |= RCC_CFGR_SW_PLL; // Switch the system clock to PLL output
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL); // Wait for PLL to be used as the system clock
}

void init_GPIO(void) {
    // Enable GPIO clock for GPIOB (for I2C1)
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    // Configure I2C1 pins (PB6, PB7)
    GPIOB->CRL |= GPIO_CRL_MODE6_1 | GPIO_CRL_MODE7_1; // Output mode, max speed 2 MHz
    GPIOB->CRL |= GPIO_CRL_CNF6_1 | GPIO_CRL_CNF7_1; // Alternate function open-drain
    GPIOB->ODR |= GPIO_ODR_ODR6 | GPIO_ODR_ODR7; // Set high
}

void init_I2C1(void) {
    // Enable I2C1 clock
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    // Reset I2C1
    I2C1->CR1 |= I2C_CR1_SWRST;
    I2C1->CR1 &= ~I2C_CR1_SWRST;
    // Set I2C1 clock control
    I2C1->CR2 = 36; // APB1 Clock Frequency 36 MHz
    I2C1->CCR = 180; // Standard mode, 100 kHz clock
    I2C1->TRISE = 37; // Maximum rise time
    I2C1->CR1 |= I2C_CR1_PE; // Enable I2C1
}

void USART1_Config(void) {
	 RCC->APB2ENR |= 1 << 14; 
   // GPIOA PA9 for USART1 TX as alternate function push-pull
	 // Enable GPIOA clock, bit 2 is IOPAEN
   RCC->APB2ENR |= 1 << 2;  
   GPIOA->CRH &= ~((0x0F) << (4 * (9 - 8))); 
   GPIOA->CRH |= (0x0B << (4 * (9 - 8)));  
   // Set baud rate 115200
   USART1->BRR = 72000000 /115200; 
   USART1->CR1 |= (1 << 13) | (1 << 3) | (1 << 2);
}

void USART1_SendString(char *str) {
    while (*str) {
        while (!(USART1->SR & USART_SR_TXE)); 
        USART1->DR = (uint8_t)(*str++); 
    }
}
void I2C_Start(void) {
    // Generate a START condition
    I2C1->CR1 |= I2C_CR1_START; // Send I2C1 START condition
    while (!(I2C1->SR1 & I2C_SR1_SB)); // Wait for START condition to be transmitted
}

void I2C_Stop(void) {
    // Generate a STOP condition
    I2C1->CR1 |= I2C_CR1_STOP; // Send I2C1 STOP condition
}

void I2C_Write(uint8_t data) {
    // Send a byte of data
    while (!(I2C1->SR1 & I2C_SR1_TXE)); // Wait until TXE (Transmit data register empty) is set
    I2C1->DR = data; // Send data byte
    while (!(I2C1->SR1 & I2C_SR1_BTF)); // Wait until BTF (Byte Transfer Finished) is set
}

uint8_t I2C_Read_Ack(void) {
    // Read a byte and acknowledge
    I2C1->CR1 |= I2C_CR1_ACK; // Enable acknowledgment
    while (!(I2C1->SR1 & I2C_SR1_RXNE)); // Wait until RXNE (Receive data register not empty) is set
    return I2C1->DR; // Return the data in the DR register
}

uint8_t I2C_Read_Nack(void) {
    // Read a byte and not acknowledge
    I2C1->CR1 &= ~I2C_CR1_ACK; // Disable acknowledgment
    I2C1->CR1 |= I2C_CR1_STOP; // Send I2C1 STOP condition after the current byte transfer
    while (!(I2C1->SR1 & I2C_SR1_RXNE)); // Wait until RXNE (Receive data register not empty) is set
    return I2C1->DR; // Return the data in the DR register
}
void writeRegister(uint8_t reg, uint8_t value) {
    I2C_Start();
    I2C_Write(MAX30100_ADDR << 1); // Device address and write operation
    I2C_Write(reg); // Register address
    I2C_Write(value); // Write value to register
    I2C_Stop();
}

uint8_t readRegister(uint8_t reg) {
    uint8_t value;
    I2C_Start();
    I2C_Write(MAX30100_ADDR << 1); // Device address and write operation
    I2C_Write(reg); // Register address
    I2C_Start();
    I2C_Write((MAX30100_ADDR << 1) | 1); // Device address and read operation
    value = I2C_Read_Nack(); // Read the register with NACK
    return value;
}
// Counter for 1ms SysTick interrupt

