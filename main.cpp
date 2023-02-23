
#include <inttypes.h>
#include "libCH32V30x.h"
#include "uart.h"

#if 1 // ======================== Variables and defines ========================
// Forever
static const UartParams_t CmdUartParams(115200, CMD_UART_PARAMS);
CmdUart_t Uart{CmdUartParams};

#endif


int main() {
	Clk::UpdateFreqValues();
	Uart.Init();
	Printf("\r%S %S\r", APP_NAME, XSTRINGIFY(BUILD_TIME));
	Clk::PrintFreqs();

	Gpio::SetupOut(GPIOA, 8, Gpio::omPushPull);
	Gpio::SetLo(GPIOA, 8);

	uint32_t i=0;
	while(true) {
		for(volatile uint32_t i=0; i<100000; i++);
		Gpio::Toggle(GPIOA, 8);
		Printf("%u ", i++);
	}
}
/*
    RCC->APB2PCENR |= RCC_TIM1EN;
    TIM1->PSC = 8000;
    TIM1->ATRLR = 200;
    TIM1->CH1CVR = 100;

    TIM1->CTLR1 = TIM_ARPE;
    TIM1->CTLR2 = 0;

    TIM1->DMAINTENR = TIM_UIE;

    TIM1->CHCTLR1 = 0b110U << 4;
    TIM1->CCER = 1U << 0;
    TIM1->BDTR = 0xC000;

    NVIC_SetPriority(TIM1_UP_IRQn, 0x40);
//    NVIC_EnableIRQ(TIM1_UP_IRQn);

    uint32_t n = (uint32_t)TIM1_UP_IRQn;
    NVIC->IPRR[n >> 5U] = 1U << (n & 0x1F);
    NVIC->IENR[n >> 5U] = 1U << (n & 0x1F);


    Printf("ISR 0x%X 0x%X; ", PFIC->ISR[0], PFIC->ISR[1]);
    Printf("IPR 0x%X 0x%X; ", PFIC->IPR[0], PFIC->IPR[1]);
    Printf("IAC 0x%X 0x%X\r", PFIC->IACTR[0], PFIC->IACTR[1]);

    Printf("*F: 0x%X; ie: 0x%X\r", TIM1->INTFR, TIM1->DMAINTENR);

    TIM1->CTLR1 |= TIM_CEN;

    Gpio::SetupAlterFunc(GPIOA, 8, Gpio::omPushPull, Gpio::ps50MHz);

    while(1) {
//        Printf("F: 0x%X; ie: 0x%X\r", TIM1->INTFR, TIM1->DMAINTENR);
        for(volatile uint32_t i=0; i<1000000; i++);
//        Pin::Toggle(GPIOA, 8);
//        Printf("  F: 0x%X; ie: 0x%X\r", TIM1->INTFR, TIM1->DMAINTENR);
        Printf("  ISR 0x%X 0x%X; ", PFIC->ISR[0], PFIC->ISR[1]);
        Printf("IPR 0x%X 0x%X; ", PFIC->IPR[0], PFIC->IPR[1]);
        Printf("IAC 0x%X 0x%X\r", PFIC->IACTR[0], PFIC->IACTR[1]);
//        NVIC_ClearPendingIRQ(TIM1_UP_IRQn);
    }
}

extern "C" {

__attribute__((interrupt()))
void TIM1_UP_IRQHandler() {
//    Printf("1ISR 0x%X 0x%X; ", PFIC->ISR[0], PFIC->ISR[1]);
//    Printf("IPR 0x%X 0x%X; ", PFIC->IPR[0], PFIC->IPR[1]);
//    Printf("1IAC 0x%X 0x%X\r", PFIC->IACTR[0], PFIC->IACTR[1]);
//    Printf("1F: 0x%X; ie: 0x%X\r", TIM1->INTFR, TIM1->DMAINTENR);
    TIM1->INTFR = 0;
    Printf("###\r");
//    NVIC_ClearPendingIRQ(TIM1_UP_IRQn);
//    Printf("2F: 0x%X; ie: 0x%X\r", TIM1->INTFR, TIM1->DMAINTENR);
//    Printf("2ISR 0x%X 0x%X; ", PFIC->ISR[0], PFIC->ISR[1]);
//    Printf("IPR 0x%X 0x%X; ", PFIC->IPR[0], PFIC->IPR[1]);
//    Printf("2IAC 0x%X 0x%X\r", PFIC->IACTR[0], PFIC->IACTR[1]);
}

}
*/
