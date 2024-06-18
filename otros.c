#include<stdio.h>
#include<stm32f7xx.h>

int pulse;
int dato=0;

extern "C"{
    void UART4_IRQHandler(void){
        if(UART4->ISR & 0x20){
            dato=UART4->RDR;
        }
    }
}
void settings (void){

    //Configuracion de poten, swit, pulsa.
    RCC->AHB1ENR|=RCC_AHB1ENR_GPIOCEN;
    GPIOC->MODER |= 0X00A00000;					//PIN C10_Tx Y C11_Rx
    GPIOC->OTYPER |= 0X0000;
    GPIOC->OSPEEDR |= 0X80000A2;
    GPIOC->PUPDR |= 0X80000A2;
    GPIOC->AFR[1]=0x00008800;

    //configuracion USART
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    GPIOD->MODER |= 0x2800;
    GPIOD->OTYPER |= 0x0; // push-pull
    GPIOD->OSPEEDR |= 0x3C00; // very-high
    GPIOD->PUPDR |= 0x1400; // pull-up
    GPIOD->AFR[0] |= 0x7700000;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    USART2->BRR |= 0x683;
    USART2->CR1 |= USART_CR1_TE | USART_CR1_RE;
    USART2->CR1 |= USART_CR1_UE; // enable UART

    //regitros del led para pwm
    RCC->AHB1ENR|=RCC_AHB1ENR_GPIOBEN;
    GPIOB->MODER |=0xA000;							//PINS B6 SERVO Y B7 LED
    GPIOB->OTYPER |=0X0000;
    GPIOB->OSPEEDR |=0XA000;
    GPIOB->PUPDR |=0X00000000;
    GPIOB->AFR[0] |=0X22000000;
}

void PWM(void){
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    TIM4->PSC=99;
    TIM4->ARR=3231;
    TIM4->CR1 |= 0X0001;								//Activación del contador
    TIM4->CCMR1 |= 0X6060;							// Modo de PWM1
    TIM4->CCER |=0X0011;								//Habilitar el canal
    TIM4->EGR |=0x0001;									//Reinicializa el contador y genera una actualización de los registros
}
void Servo(int angulo){
    if(angulo==30){
        pulse=(120+(angulo*(415-120)/180));
    }
    if(angulo==135){
        pulse=(120+(angulo*(415-120)/180));
    }

    TIM4->CCR1=pulse;
}

int main(void){
    settings();
    PWM();
    while(1){
        if(dato=='V'){
            Servo(30);
        }
    }
}