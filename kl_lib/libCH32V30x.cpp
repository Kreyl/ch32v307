/*
 * libCH32V.cpp
 *
 *  Created on: 19.02.23
 *      Author: layst
 */

#include <libCH32V30x.h>


#if 1 // ============================== DMA ====================================
struct DmaIrqHandler_t {
    ftVoidPVoidW32 Handler = nullptr;
    void *Param = nullptr;
    uint32_t Prio = IRQ_PRIO_LOW;
};

static DmaIrqHandler_t DmaIrqHandler[DMA_CHNL_CNT];

static const IRQn DmaIrqNum[DMA_CHNL_CNT] = {
        DMA1_Channel1_IRQn, DMA1_Channel2_IRQn, DMA1_Channel3_IRQn, DMA1_Channel4_IRQn,
        DMA1_Channel5_IRQn, DMA1_Channel6_IRQn, DMA1_Channel7_IRQn,
		DMA2_Channel1_IRQn, DMA2_Channel2_IRQn, DMA2_Channel3_IRQn, DMA2_Channel4_IRQn,
		DMA2_Channel5_IRQn, DMA2_Channel6_IRQn, DMA2_Channel7_IRQn, DMA2_Channel8_IRQn,
		DMA2_Channel9_IRQn, DMA2_Channel10_IRQn, DMA2_Channel11_IRQn,
};

static DMA_Channel_TypeDef *DmaChnlTbl[DMA_CHNL_CNT] = {
		DMA1_Channel1, DMA1_Channel2, DMA1_Channel3, DMA1_Channel4,
		DMA1_Channel5, DMA1_Channel6, DMA1_Channel7,
		DMA2_Channel1, DMA2_Channel2, DMA2_Channel3, DMA2_Channel4,
		DMA2_Channel5, DMA2_Channel6, DMA2_Channel7, DMA2_Channel8,
		DMA2_Channel9, DMA2_Channel10, DMA2_Channel11
};

DMA_t::DMA_t(uint32_t AChnl,
        ftVoidPVoidW32 PIrqFunc, void *PIrqParam, uint32_t AIrqPrio) {
	ChnlN = AChnl;
    PChnl = DmaChnlTbl[AChnl];
    DmaIrqHandler[AChnl].Handler = PIrqFunc;
    DmaIrqHandler[AChnl].Param = PIrqParam;
    DmaIrqHandler[AChnl].Prio = AIrqPrio;
}

void DMA_t::Init() const {
    RCC->AHBPCENR |= (1UL << 0) | (1UL << 1); // En DMA1 & DMA2 clocking
    if(DmaIrqHandler[ChnlN].Handler != nullptr) {
        NVIC_SetPriority(DmaIrqNum[ChnlN], DmaIrqHandler[ChnlN].Prio);
        NVIC_EnableIRQ(DmaIrqNum[ChnlN]);
    } // if irq
    PChnl->CFGR = 0; // Reset value
}

void DMA_t::Init(volatile void* PeriphAddr, void* MemAddr, uint32_t AMode, uint16_t Cnt) const {
    Init();
    SetPeriphAddr(PeriphAddr);
    SetMemoryAddr(MemAddr);
    SetMode(AMode);
    SetTransferDataCnt(Cnt);
}

void DMA_t::ClearIrq() const { // DMA1: chnls [0;6]; DMA2: chnls [7;13]; Dma2 ext: chnls [14;17]
	if(ChnlN <= 6) DMA1->INTFCR = 0b1111UL << ChnlN; // [0;6]
	else if(ChnlN <= 13) DMA2->INTFCR = 0b1111UL << (ChnlN - 7); // [7;13]
	else DMA2_EXTEN->INTFCR = 0b1111UL << (ChnlN - 14); // [14;17]
}

void DMA_t::DisableAndClearIRQ() const {
	PChnl->CFGR &= ~0b1111UL; // TEIE | HTIE | TCIE | EN
	ClearIrq();
}

// ==== IRQs ====
#define DMA_IRQ_HANDLER(DmaN, ChnlN) \
    __attribute__((interrupt("WCH-Interrupt-fast"))) \
    void DMA##DmaN##_Channel##ChnlN##_IRQHandler() { \
        uint32_t flags = (DMA##DmaN->INTFR >> (ChnlN - 1) * 4) & 0b1111UL; \
        DMA##DmaN->INTFCR = 1UL << (ChnlN - 1) * 4; /* Clear all irq flags */ \
        ftVoidPVoidW32 func = DmaIrqHandler[DMA_CHNL(DmaN, ChnlN)].Handler; \
        if(func) func(DmaIrqHandler[DMA_CHNL(DmaN, ChnlN)].Param, flags); \
    }

#define DMA2EXT_IRQ_HANDLER(ChnlN) \
    __attribute__((interrupt("WCH-Interrupt-fast"))) \
    void DMA2_Channel##ChnlN##_IRQHandler() { \
        uint32_t flags = (DMA2_EXTEN->INTFR >> (ChnlN - 8) * 4) & 0b1111UL; \
        DMA2_EXTEN->INTFCR = 1UL << (ChnlN - 8) * 4; /* Clear all irq flags */ \
        ftVoidPVoidW32 func = DmaIrqHandler[DMA_CHNL(2, ChnlN)].Handler; \
        if(func) func(DmaIrqHandler[DMA_CHNL(2, ChnlN)].Param, flags); \
    }

extern "C" {
#if DMA1_CH1_IRQ_EN
DMA_IRQ_HANDLER(1, 1);
#endif
#if DMA1_CH2_IRQ_EN
DMA_IRQ_HANDLER(1, 2);
#endif
#if DMA1_CH3_IRQ_EN
DMA_IRQ_HANDLER(1, 3);
#endif
#if DMA1_CH4_IRQ_EN
DMA_IRQ_HANDLER(1, 4);
#endif
#if DMA1_CH5_IRQ_EN
DMA_IRQ_HANDLER(1, 5);
#endif
#if DMA1_CH6_IRQ_EN
DMA_IRQ_HANDLER(1, 6);
#endif
#if DMA1_CH7_IRQ_EN
DMA_IRQ_HANDLER(1, 7);
#endif

#if DMA2_CH1_IRQ_EN
DMA_IRQ_HANDLER(2, 1);
#endif
#if DMA2_CH2_IRQ_EN
DMA_IRQ_HANDLER(2, 2);
#endif
#if DMA2_CH3_IRQ_EN
DMA_IRQ_HANDLER(2, 3);
#endif
#if DMA2_CH4_IRQ_EN
DMA_IRQ_HANDLER(2, 4);
#endif
#if DMA2_CH5_IRQ_EN
DMA_IRQ_HANDLER(2, 5);
#endif
#if DMA2_CH6_IRQ_EN
DMA_IRQ_HANDLER(2, 6);
#endif
#if DMA2_CH7_IRQ_EN
DMA_IRQ_HANDLER(2, 7);
#endif

#if DMA2_CH8_IRQ_EN
DMA2EXT_IRQ_HANDLER(8);
#endif
#if DMA2_CH9_IRQ_EN
DMA2EXT_IRQ_HANDLER(9);
#endif
#if DMA2_CH10_IRQ_EN
DMA2EXT_IRQ_HANDLER(10);
#endif
#if DMA2_CH11_IRQ_EN
DMA2EXT_IRQ_HANDLER(11);
#endif
} // extern C
#endif

#if 1 // ============================= Timer ===================================
void Timer_t::Init() const {
    if     (ITmr == TIM1)   { RCC->APB2PCENR |= 1UL << 11; }
    else if(ITmr == TIM2)   { RCC->APB1PCENR |= 1UL << 0; }
    else if(ITmr == TIM3)   { RCC->APB1PCENR |= 1UL << 1; }
    else if(ITmr == TIM4)   { RCC->APB1PCENR |= 1UL << 2; }
    else if(ITmr == TIM5)   { RCC->APB1PCENR |= 1UL << 3; }
    else if(ITmr == TIM6)   { RCC->APB1PCENR |= 1UL << 4; }
    else if(ITmr == TIM7)   { RCC->APB1PCENR |= 1UL << 5; }
    else if(ITmr == TIM8)   { RCC->APB2PCENR |= 1UL << 13; }
    else if(ITmr == TIM9)   { RCC->APB2PCENR |= 1UL << 19; }
    else if(ITmr == TIM10)  { RCC->APB2PCENR |= 1UL << 20; }
}
#endif

namespace Gpio { // =================== Gpio ========================

static void ClockEnable(const GPIO_TypeDef *PGpioPort) {
    if     (PGpioPort == GPIOA) RCC->APB2PCENR |= 1UL << 2;
    else if(PGpioPort == GPIOB) RCC->APB2PCENR |= 1UL << 3;
    else if(PGpioPort == GPIOC) RCC->APB2PCENR |= 1UL << 4;
    else if(PGpioPort == GPIOD) RCC->APB2PCENR |= 1UL << 5;
    else if(PGpioPort == GPIOE) RCC->APB2PCENR |= 1UL << 6;
}

void SetupOut(
        GPIO_TypeDef *PGpioPort,
        const uint16_t APinNumber,
        const OutMode_t PinOutMode,
        const Speed_t ASpeed
        ) {
    // Clock
    ClockEnable(PGpioPort);
    uint32_t CnfMode = ((uint32_t)PinOutMode << 2) | (uint32_t)ASpeed;
    if(APinNumber < 8) {
        uint32_t Offset = APinNumber * 4;
        PGpioPort->CFGLR &= ~(0b1111UL << Offset);  // Clear both mode and cnf
        PGpioPort->CFGLR |= CnfMode << Offset;
    }
    else {
        uint32_t Offset = (APinNumber - 8UL) * 4;
        PGpioPort->CFGHR &= ~(0b1111UL << Offset);  // Clear both mode and cnf
        PGpioPort->CFGHR |= CnfMode << Offset;
    }
}

void SetupInput(
        GPIO_TypeDef *PGpio,
        const uint16_t PinN,
        const PullUpDown_t PullUpDown,
        const Speed_t ASpeed) {
    // Clock
    ClockEnable(PGpio);
    uint32_t CnfMode;
    if(PullUpDown == pudNone) CnfMode = 0b0100; // Floating input
    else {
        CnfMode = 0b1000; // PullUp or PullDown enabled, set or clear ODR bit
        if(PullUpDown == pudPullDown) PGpio->BCR = (1UL << PinN);
        else PGpio->BSHR = (1UL << PinN);
    }
    if(PinN < 8) {
        uint32_t Offset = PinN*4;
        PGpio->CFGLR &= ~(0b1111UL << Offset);  // Clear both mode and cnf
        PGpio->CFGLR |= CnfMode << Offset;
    }
    else {
        uint32_t Offset = (PinN - 8) * 4;
        PGpio->CFGHR &= ~(0b1111UL << Offset);  // Clear both mode and cnf
        PGpio->CFGHR |= CnfMode << Offset;
    }
}

void SetupAnalog(GPIO_TypeDef *PGpioPort, const uint16_t APinNumber) {
    // Clock
    ClockEnable(PGpioPort);
    if(APinNumber < 8) {
        uint32_t Offset = APinNumber*4;
        PGpioPort->CFGLR &= ~(0b1111UL << Offset);  // Clear both mode and cnf
    }
    else {
        uint32_t Offset = (APinNumber - 8) * 4;
        PGpioPort->CFGHR &= ~(0b1111UL << Offset);  // Clear both mode and cnf
    }
}

void SetupAlterFunc(
        GPIO_TypeDef *PGpioPort,
        const uint16_t APinNumber,
        const OutMode_t PinOutMode,
        const Speed_t ASpeed) {
    // Clock
    ClockEnable(PGpioPort);
    RCC->APB2PCENR |= RCC_AFIOEN; // Enable AFIO clock to enable alternate functions
    // Setup
    uint32_t CnfMode = ((uint32_t)PinOutMode << 2) | 0b1000UL | (uint32_t)ASpeed;
    if(APinNumber < 8) {
        uint32_t Offset = APinNumber*4;
        PGpioPort->CFGLR &= ~(0b1111UL << Offset);  // Clear both mode and cnf
        PGpioPort->CFGLR |= CnfMode << Offset;
    }
    else {
        uint32_t Offset = (APinNumber - 8) * 4;
        PGpioPort->CFGHR &= ~(0b1111UL << Offset);  // Clear both mode and cnf
        PGpioPort->CFGHR |= CnfMode << Offset;
    }
}

} // namespace gpio


namespace Flash { // ======================== Flash ============================

//void EnablePrefetch() { FLASH->ACTLR |= FLASH_ACTLR_PRFTBE; }
//
//void SetupLatency(uint32_t FMHz) {
//    uint32_t tmp = FLASH->ACTLR & ~FLASH_ACTLR_LATENCY;
//    if(Clk::AHBFreqHz > 24 and Clk::AHBFreqHz <= 48) tmp |= 1; // 1 wait state
//    else if(Clk::AHBFreqHz > 48) tmp |= 2; // 2 wait states
//    FLASH->ACTLR = tmp;
//}

} // Flash


namespace Clk { // ===================== Clocking ==============================

uint8_t EnableHSE() {
    RCC->CTLR |= RCC_HSEON;
    for(volatile uint32_t i=0; i<9000; i++) {
        if(RCC->CTLR & RCC_HSERDY) return retvOk;
    }
    RCC->CTLR &= ~RCC_HSEON;
    return retvFail;
}

uint8_t EnablePll() {
    RCC->CTLR |= RCC_PLLON;
    for(volatile uint32_t i=0; i<9000; i++) {
        if(RCC->CTLR & RCC_PLLRDY) return retvOk;
    }
    RCC->CTLR &= ~RCC_PLLON;
    return retvFail;
}

bool IsHSEReady() { return (RCC->CTLR & RCC_HSERDY); }

void SetupPll(Pll1Mul_t PllMul1, PllSrc_t PllSrc, Prediv_t Prediv, Prediv1Src_t Prediv1Src) {
    uint32_t cfgr0 = RCC->CFGR0 & ~(RCC_PLLMULL | RCC_PLLXTPRE | RCC_PLLSRC);
    cfgr0 |= (uint32_t)PllMul1 << 18;
    if(PllSrc == pllSrcHSI) EXTEN->EXTEN_CTR |= EXTEN_PLL_HSI_PRE;
    else if(PllSrc == pllSrcHSIdiv2) EXTEN->EXTEN_CTR &= ~EXTEN_PLL_HSI_PRE;
    else { // Prediv1
        cfgr0 |= RCC_PLLSRC;
        RCC->CFGR2 = (RCC->CFGR2 & 0xFFFEFFF0) | (uint32_t)Prediv | (uint32_t)Prediv1Src;
    }
    RCC->CFGR0 = cfgr0;
}

void SelectSysClkSrc(ClkSrc_t ClkSrc) {
    RCC->CFGR0 = (RCC->CFGR0 & ~RCC_SW) | (uint32_t)ClkSrc;
    for(volatile uint32_t i=0; i<9000; i++) {
        if(((RCC->CFGR0  >> 2) & RCC_SW) == (uint32_t)ClkSrc) return;
    }
    RCC->CFGR0 &= ~RCC_SW; // Back to HSI
}

static const uint32_t PllMul_x2_Tbl[16] = {36, 6,8,10,12,14,16,18,20,22,24,26,28, 13, 30,32};
static const uint32_t Pll2Mul_x2_Tbl[16] = {5,25,8,10,12,14,16,18,20,22,24,26,28,30,32,40};
static const uint32_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
static const uint32_t APBPrescTable[8] = {0, 0, 0, 0, 1, 2, 3, 4};

// Frequency values
uint32_t AHBFreqHz, APB1FreqHz, APB2FreqHz;

void UpdateFreqValues() {
    uint32_t SysClkHz;
    // Figure out SysClk
    uint32_t tmp = (RCC->CFGR0 & RCC_SWS) >> 2;
    switch(tmp) {
        case 0b01: // HSE
            SysClkHz = CRYSTAL_FREQ_HZ;
            break;

        case 0b10: { // PLL used as system clock source
            // Get different PLL dividers. PllMul may be 6.5, so Mul_x2 is used.
            uint32_t PllMulIndx = ((RCC->CFGR0 & RCC_PLLMULL) >> 18); // [0; 15]
            uint32_t PllMul_x2 = PllMul_x2_Tbl[PllMulIndx];
            // Get PLL input: HSI, HSI/2 or PREDIV1
            uint32_t PllInputFreq;
            if(RCC->CFGR0 & RCC_PLLSRC) { // PREDIV1
            	uint32_t Prediv1 = (RCC->CFGR2 & 0b1111UL) + 1UL;
            	// Get Prediv1 src: PLL2 or HSE
            	if(RCC->CFGR2 & (1UL<<16)) { // PLL2MUL
            		uint32_t Pll2MulIndx = (RCC->CFGR2 >> 8) & 0b1111UL;
            		uint32_t Pll2Mul_x2 = Pll2Mul_x2_Tbl[Pll2MulIndx];
            		uint32_t Prediv2 = ((RCC->CFGR2 >> 4) & 0b1111UL) + 1UL;
            		PllInputFreq = ((CRYSTAL_FREQ_HZ / Prediv2) * Pll2Mul_x2) / 2;
            	}
            	else PllInputFreq = CRYSTAL_FREQ_HZ / Prediv1;
            }
            else { // HSI or HSI/2
            	PllInputFreq = (EXTEN->EXTEN_CTR & EXTEN_PLL_HSI_PRE)? HSI_VALUE : (HSI_VALUE / 2);
            }
            SysClkHz = (PllInputFreq * PllMul_x2) / 2;
        } break;

        default: // 0b00, HSI
            SysClkHz = HSI_VALUE;
            break;
    } // switch

    // AHB freq
    tmp = AHBPrescTable[((RCC->CFGR0 & RCC_HPRE) >> 4)];
    AHBFreqHz = SysClkHz >> tmp;
    // APB freq
    uint32_t APB1prs = (RCC->CFGR0 & RCC_PPRE1) >> 8;
    uint32_t APB2prs = (RCC->CFGR0 & RCC_PPRE2) >> 11;
    tmp = APBPrescTable[APB1prs];
    APB1FreqHz = AHBFreqHz >> tmp;
    tmp = APBPrescTable[APB2prs];
    APB2FreqHz = AHBFreqHz >> tmp;
}

void PrintFreqs() {
    Printf("AHBFreq=%uMHz; APB1Freq=%uMHz; APB2Freq=%uMHz\r",
            AHBFreqHz/1000000, APB1FreqHz/1000000, APB2FreqHz/1000000);
}

} // clocking
