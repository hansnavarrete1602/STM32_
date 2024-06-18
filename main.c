#include "stm32f767xx.h"

#include <stdio.h>

#include <string.h>

#include <stdlib.h>

//#include "iostream"

using namespace std;



#define FREQ 16000000

#define BAUD 9600

#define BufferSize 32



char *pEnd;



unsigned char buffer1[BufferSize];

unsigned char buffer2[BufferSize];

unsigned char buffer3[BufferSize];

char buffer[BufferSize];

char v[BufferSize];

char v2[BufferSize];

int i = 0, j = 0, x = 0, n1, n2, n3;

int numbers[]={};

//char a = 'A';



void systick_delay(int delay){

              SysTick -> LOAD = 16000;

              SysTick -> VAL = 0;

              SysTick ->CTRL = (1 << 0) | (1 << 2);

              for(int i=delay; i > 0; i--){

                            while(!(SysTick->CTRL & (1 << 16)));

              }

              SysTick->CTRL=0;

}



static uint16_t uart_bd(uint32_t PeriphClk, uint32_t BaudRate){

              return ((PeriphClk  +  (BaudRate/2U))/BaudRate);

}


/*
void leds_init(){

              //L1 -> RB0 | L2 -> RB7 | L3 -> RB14

              RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

              GPIOB -> MODER |= 0x10004001; //RB0,RB7,RB14

              GPIOB -> OTYPER |= 0x0; //push-pull

              GPIOB -> OSPEEDR |= 0x3000C003; //very-high

              GPIOB -> ODR |= 0x0;

}
*/


void uart_init(USART_TypeDef * USARTx, int baud){

              if(USARTx == USART1){

                            #define AF7 0x77

                            //USART1 -> TX -> PA9/AF7 | USART1 -> RX -> PA10/AF7

                            RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

                            GPIOA -> MODER |= 0x280000;

                            GPIOA -> OTYPER |= 0x0; //push-pull

                            GPIOA -> OSPEEDR |= 0x3C0000; //very-high

                            GPIOA -> PUPDR |= 0x140000; //pull-up

                            GPIOA -> AFR[1] |= 0x770;;

                            RCC -> APB2ENR |= RCC_APB2ENR_USART1EN;

                            USART1 -> BRR |= uart_bd(FREQ, baud);

                            USART1 -> CR1 |= USART_CR1_TE|USART_CR1_RE;

                            USART1 -> CR1 |= USART_CR1_UE;

              }

              if(USARTx == USART2){

                            #define AF7 0x77

                            //USART2 -> TX -> PD5/AF7 | USART2 -> RX -> PD6/AF7

                            RCC -> AHB1ENR |= RCC_AHB1ENR_GPIODEN;

                            GPIOD -> MODER |= 0x2800;

                            GPIOD -> OTYPER |= 0x0; //push-pull

                            GPIOD -> OSPEEDR |= 0x3C00; //very-high

                            GPIOD -> PUPDR |= 0x1400; //pull-up

                            GPIOD -> AFR[0] |= 0x7700000;;

                            RCC -> APB1ENR |= RCC_APB1ENR_USART2EN;

                            USART2 -> BRR |= uart_bd(FREQ, baud); // baud rate 9600 @16MHz 0x683

                            USART2 -> CR1 |= USART_CR1_TE|USART_CR1_RE;//USART_CR1_RXNEIE|USART_CR1_TE|USART_CR1_RE;

                            USART2 -> CR1 |= USART_CR1_UE; //enable UART

              }

              if(USARTx == USART3){

                            #define AF7 0x77

                            //USART3 -> TX -> PC10/AF7 | USART3 -> RX -> PC11/AF7

                            RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

                            GPIOC -> MODER |= 0xA00000;

                            GPIOC -> OTYPER |= 0x0; //push-pull

                            GPIOC -> OSPEEDR |= 0xF00000; //very-high

                            GPIOC -> PUPDR |= 0x500000; //pull-up

                            GPIOC -> AFR[1] |= 0x7700;

                            RCC -> APB1ENR |= RCC_APB1ENR_USART3EN;

                            USART3 -> BRR |= uart_bd(FREQ, baud);

                            USART3 -> CR1 |= USART_CR1_TE|USART_CR1_RE;

                            USART3 -> CR1 |= USART_CR1_UE;

              }

}



int8_t uart_data_available(USART_TypeDef * USARTx){

              if((USARTx->ISR & 0x20)){return 1;}

              else return 0;

}



unsigned char uart_read(USART_TypeDef * USARTx){

              return ((uint8_t)(USARTx->RDR));

}



void uart_type(USART_TypeDef * USARTx, int x){

              USARTx->TDR =(x);

              while((USARTx->ISR & 0x80) == 0){;}

}



void uart_write(USART_TypeDef * USARTx, char *string){

              while(*string!='\0'){

                            uart_type(USARTx, *string);

                            string++;

              }

}



void uart_write_2(USART_TypeDef * USARTx, uint8_t *buffer, uint32_t nBytes) {

              int i;

              for (i = 0; i <= nBytes; i++) {

                            while (!(USARTx->ISR & USART_ISR_TXE));       // wait until TXE (TX empty) bit is set

                            // Writing USART_DR automatically clears the TXE flag

                            USARTx->TDR = buffer[i] & 0xFF;

              }

              while (!(USARTx->ISR & USART_ISR_TC));                         // wait until TC bit is set

              USARTx->ISR &= ~USART_ISR_TC;

}



void enable_uart(USART_TypeDef * USARTx){

              USARTx -> CR1 |= USART_CR1_UE; //enable UART

}



void disable_uart(USART_TypeDef * USARTx){

              USARTx -> CR1 &= ~USART_CR1_UE; //disable UART

}



void read_pms(USART_TypeDef * USARTx, USART_TypeDef * USARTy, int n){

              enable_uart(USARTx);

              if(uart_data_available(USARTx)==1){

                            v[i] = uart_read(USARTx);

                            i=i+1;

                            if(i>=32){

                                           n1 = sprintf((char *)buffer1, "%d -> pm1.0: %d | pm2.5: %d | pm10: %d", n ,(int(v[11])+0)*256+(int(v[12])+0),(int(v[13])+0)*256+(int(v[14])+0),(int(v[15])+0)*256+(int(v[16])+0));

                                           uart_write_2(USARTy, buffer1, n1);

                                           uart_type(USART2, '\n');

                                           i=0;

                            }

                            disable_uart(USARTx);

                            return;

              }

}



int main(void){

              leds_init();

              uart_init(USART1, BAUD);

              uart_init(USART2, BAUD);

              uart_init(USART3, BAUD);

              disable_uart(USART1);

              disable_uart(USART3);

              while(1){

                            read_pms(USART1, USART2, 1);

                            read_pms(USART3, USART2, 2);

                            /*

                            if(uart_data_available(USART3)==1){

                                           v2[i] = uart_read(USART3);

                                           i=i+1;

                                           if(i>=32){

                                                         n2 = sprintf((char *)buffer2, "OUT -> pm1.0: %d | pm2.5: %d | pm10: %d\n\n",(int(v2[11])+0)*256+(int(v2[12])+0),(int(v2[13])+0)*256+(int(v2[14])+0),(int(v2[15])+0)*256+(int(v2[16])+0));

                                                         uart_write_2(USART2, buffer2, n2);

                                                         i=0;

                                           }

                                           disable_uart(USART3);

                                           enable_uart(USART1);

                            }

                            */

              }

}