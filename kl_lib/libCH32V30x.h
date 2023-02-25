/*
 * libV10x.h
 *
 *  Created on: 19 февр. 2023 г.
 *      Author: layst
 */

#ifndef KL_LIB_LIBV10X_H_
#define KL_LIB_LIBV10X_H_

#include "ch32v30x.h"
#include "board.h"
#include "shell.h"

#if 1 // ============================ General ==================================
// Note: ch32v307rc is ch32v30xd8c.

typedef void (*ftVoidVoid)(void);
typedef void (*ftVoidPVoid)(void*p);
typedef void (*ftVoidPVoidW32)(void*p, uint32_t W32);

void Printf(const char *format, ...);

#ifndef countof
#define countof(A)  (sizeof(A)/sizeof(A[0]))
#endif

// IRQ priorities: smaller value is higher prio. Top 4 bits are utilized here.
#define IRQ_PRIO_LOW            0xF0 // Minimum prio
#define IRQ_PRIO_MEDIUM         0x90
#define IRQ_PRIO_HIGH           0x40
#define IRQ_PRIO_VERYHIGH       0x00 // Top prio

// Return values
#define retvOk              0
#define retvFail            1
#define retvTimeout         2
#define retvBusy            3
#define retvInProgress      4
#define retvCmdError        5
#define retvCmdUnknown      6
#define retvBadValue        7
#define retvNew             8
#define retvSame            9
#define retvLast            10
#define retvEmpty           11
#define retvOverflow        12
#define retvNotANumber      13
#define retvWriteProtect    14
#define retvWriteError      15
#define retvEndOfFile       16
#define retvNotFound        17
#define retvBadState        18
#define retvDisconnected    19
#define retvCollision       20
#define retvCRCError        21
#define retvNACK            22
#define retvNoAnswer        23
#define retvOutOfMemory     24
#define retvNotAuthorised   25
#define retvNoChanges       26

enum RiseFall_t {risefallRising, risefallFalling, risefallNone, risefallBoth};
enum Inverted_t {invNotInverted, invInverted};

// ==== Build time ====
// Define symbol BUILD_TIME in main.cpp options with value ${current_date}.
// Printf("\r%S %S\r", APP_NAME, XSTRINGIFY(BUILD_TIME));
#define STRINGIFY(x)    # x
#define XSTRINGIFY(x)   STRINGIFY(x)

// ==== Math ====
#define MIN_(a, b)   ( ((a)<(b))? (a) : (b) )
#define MAX_(a, b)   ( ((a)>(b))? (a) : (b) )
#define ABS(a)      ( ((a) < 0)? -(a) : (a) )

#endif

namespace Gpio { // ======================= GPIO ===============================
enum OutMode_t {outmodePushPull = 0b00UL, outmodePOpenDrain = 0b01UL};
enum Speed_t {
    speed10MHz = 0b01UL,
    speed2MHz = 0b10UL,
    speed50MHz = 0b11UL
};
#define PIN_SPEED_DEFAULT   speed10MHz

enum PullUpDown_t {
    pudNone = 0b00,
    pudPullUp = 0b01,
    pudPullDown = 0b10
};

__attribute__((always_inline))
static inline void SetHi(GPIO_TypeDef *PGpio, uint32_t APin) { PGpio->BSHR = 1UL << APin; }
__attribute__((always_inline))
static inline void SetLo(GPIO_TypeDef *PGpio, uint32_t APin) { PGpio->BCR = 1UL << APin;  }
__attribute__((always_inline))
static inline void Toggle(GPIO_TypeDef *PGpio, uint32_t APin) { PGpio->OUTDR ^= 1UL << APin; }
// Check input
__attribute__((always_inline))
static inline bool IsHi(GPIO_TypeDef *PGpio, uint32_t APin) { return PGpio->INDR & (1UL << APin); }
__attribute__((always_inline))
static inline bool IsHi(const GPIO_TypeDef *PGpio, uint32_t APin) { return PGpio->INDR & (1UL << APin); }
__attribute__((always_inline))
static inline bool IsLo(GPIO_TypeDef *PGpio, uint32_t APin) { return !(PGpio->INDR & (1UL << APin)); }
__attribute__((always_inline))
static inline bool IsLo(const GPIO_TypeDef *PGpio, uint32_t APin) { return !(PGpio->INDR & (1UL << APin)); }

// Setup
void SetupOut(GPIO_TypeDef *PGpioPort, const uint16_t APinNumber, const OutMode_t PinOutMode, const Speed_t ASpeed = PIN_SPEED_DEFAULT);
void SetupInput(GPIO_TypeDef *PGpio, const uint16_t PinN, const PullUpDown_t PullUpDown, const Speed_t ASpeed = PIN_SPEED_DEFAULT);
void SetupAnalog(GPIO_TypeDef *PGpioPort, const uint16_t APinNumber);
void SetupAlterFunc(GPIO_TypeDef *PGpioPort, const uint16_t APinNumber, const OutMode_t PinOutMode, const Speed_t ASpeed = PIN_SPEED_DEFAULT);

} // Namespace pin

#if 1 // ============================== DMA ====================================
// DMA
#define DMA_PRIORITY_LOW        (0b00UL << 12)
#define DMA_PRIORITY_MEDIUM     (0b01UL << 12)
#define DMA_PRIORITY_HIGH       (0b10UL << 12)
#define DMA_PRIORITY_VERYHIGH   (0b11UL << 12)

#define DMA_MSIZE_8_BIT         (0b00UL << 10)
#define DMA_MSIZE_16_BIT        (0b01UL << 10)
#define DMA_MSIZE_32_BIT        (0b10UL << 10)

#define DMA_PSIZE_8_BIT         (0b00UL << 8)
#define DMA_PSIZE_16_BIT        (0b01UL << 8)
#define DMA_PSIZE_32_BIT        (0b10UL << 8)

#define DMA_MEM_INC             (1UL << 7)
#define DMA_PER_INC             (1UL << 6)
#define DMA_CIRC                (1UL << 5)

#define DMA_DIR_MEM2PER         (1UL << 4)
#define DMA_DIR_PER2MEM         (0UL << 4)

#define DMA_TCIE                (1UL << 1)

#define DMA_CHNL_CNT			18
// Translate dma1 chnls[1;7] and dma2 chnls [1;11] to chnls[0; 17]
#define DMA_CHNL(dma, chnl)		((dma - 1) * 7 + (chnl - 1))

class DMA_t {
private:
    uint32_t ChnlN;
    DMA_Channel_TypeDef *PChnl;
public:
    DMA_t(uint32_t AChnl,
            ftVoidPVoidW32 PIrqFunc = nullptr, void *PIrqParam = nullptr,
            uint32_t AIrqPrio = IRQ_PRIO_MEDIUM);
    void Init() const;
    void Init(volatile void* PeriphAddr, void* MemAddr, uint32_t AMode, uint16_t Cnt) const;
    void SetPeriphAddr(volatile void* Addr) const { PChnl->PADDR = (uint32_t)Addr; }
    void SetMemoryAddr(void* Addr)          const { PChnl->MADDR = (uint32_t)Addr; }
    void* GetMemoryAddr()                   const { return (void*)PChnl->MADDR; }
    void SetMode(uint32_t AMode)            const { PChnl->CFGR = AMode; }
    void SetTransferDataCnt(uint32_t Cnt)   const { PChnl->CNTR = Cnt; }
    uint16_t GetTransferDataCnt()           const { return PChnl->CNTR; }

    void Enable()                           const { PChnl->CFGR |=  (1UL << 0); }
    void Disable()                          const { PChnl->CFGR &= ~(1UL << 0); }
    void ClearIrq() const;
    void DisableAndClearIRQ() const;
};
#endif

#if 1 // =========================== HW Timers =================================
class Timer_t {
protected:
    TIM_TypeDef* ITmr;
public:
    Timer_t(TIM_TypeDef *APTimer) : ITmr(APTimer) {}
    void Init() const;
    void Deinit() const;
    void Enable()  const { ITmr->CTLR1 |=  TIM_CEN; }
    void Disable() const { ITmr->CTLR1 &= ~TIM_CEN; }
    void Reset() const;
    void SetUpdateFrequencyChangingPrescaler(uint32_t FreqHz) const;
    void SetUpdateFrequencyChangingTopValue(uint32_t FreqHz) const;
    void SetUpdateFrequencyChangingBoth(uint32_t FreqHz) const;
    void SetTopValue(uint16_t Value) const { ITmr->ATRLR = Value; }
    uint16_t GetTopValue() const { return ITmr->ATRLR; }
    void EnableArrBuffering()  const { ITmr->CTLR1 |=  TIM_ARPE; }
    void DisableArrBuffering() const { ITmr->CTLR1 &= ~TIM_ARPE; }
    void SetupPrescaler(uint32_t PrescaledFreqHz) const;
    void SetCounter(uint16_t Value) const { ITmr->CNT = Value; }
    uint16_t GetCounter() const { return ITmr->CNT; }

    // Compare
    void SetCCR1(uint16_t AValue) const { ITmr->CH1CVR = AValue; }
    void SetCCR2(uint16_t AValue) const { ITmr->CH2CVR = AValue; }
    void SetCCR3(uint16_t AValue) const { ITmr->CH3CVR = AValue; }
    void SetCCR4(uint16_t AValue) const { ITmr->CH4CVR = AValue; }
    uint16_t GetCCR1() const { return ITmr->CH1CVR; }
    uint16_t GetCCR2() const { return ITmr->CH2CVR; }
    uint16_t GetCCR3() const { return ITmr->CH3CVR; }
    uint16_t GetCCR4() const { return ITmr->CH4CVR; }

    // Inputs
    enum InputPrescaler_t{pscDiv1=0U, pscDiv2=1U, pscDiv4=2U, pscDiv8=3U};
    void SetupInput1(uint16_t Mode, InputPrescaler_t Psc, RiseFall_t Rsfll) const {
        ITmr->CHCTLR1 = (ITmr->CHCTLR1 & 0xFF00) | ((uint16_t)Psc << 2)  | (Mode << 0);
        if(Rsfll == risefallFalling) ITmr->CCER |= 1U << 1;
    }
    void SetupInput2(uint16_t Mode, InputPrescaler_t Psc, RiseFall_t Rsfll) const {
        ITmr->CHCTLR1 = (ITmr->CHCTLR1 & 0x00FF) | ((uint16_t)Psc << 10) | (Mode << 8);
        if(Rsfll == risefallFalling) ITmr->CCER |= 1U << 5;
    }
    void SetupInput3(uint16_t Mode, InputPrescaler_t Psc, RiseFall_t Rsfll) const {
        ITmr->CHCTLR2 = (ITmr->CHCTLR2 & 0xFF00) | ((uint16_t)Psc << 2)  | (Mode << 0);
        if(Rsfll == risefallFalling) ITmr->CCER |= 1U << 9;
    }
    void SetupInput4(uint16_t Mode, InputPrescaler_t Psc, RiseFall_t Rsfll) const {
        ITmr->CHCTLR2 = (ITmr->CHCTLR2 & 0x00FF) | ((uint16_t)Psc << 10) | (Mode << 8);
        if(Rsfll == risefallFalling) ITmr->CCER |= 1U << 13;
    }

    // Outputs. Pwm1 is ###___, PWM2 is inverse: ___###
    enum OutMode_t { outmodePwm1=0b110U, outmodePwm2=0b111U };
    void SetupCCOutput1(OutMode_t Mode = outmodePwm1, bool PreloadEn = true) const {
        ITmr->CHCTLR1 = (ITmr->CHCTLR1 & 0xFF00U) | ((uint16_t)Mode << 4) | (PreloadEn? TIM_OC1PE : 0);
    }
    void SetupCCOutput2(OutMode_t Mode = outmodePwm1, bool PreloadEn = true) const {
        ITmr->CHCTLR1 = (ITmr->CHCTLR1 & 0x00FFU) | ((uint16_t)Mode << 12) | (PreloadEn? TIM_OC2PE : 0);
    }
    void SetupCCOutput3(OutMode_t Mode = outmodePwm1, bool PreloadEn = true) const {
        ITmr->CHCTLR2 = (ITmr->CHCTLR2 & 0xFF00U) | ((uint16_t)Mode << 4) | (PreloadEn? TIM_OC3PE : 0);
    }
    void SetupCCOutput4(OutMode_t Mode = outmodePwm1, bool PreloadEn = true) const {
        ITmr->CHCTLR2 = (ITmr->CHCTLR2 & 0x00FFU) | ((uint16_t)Mode << 12) | (PreloadEn? TIM_OC4PE : 0);
    }
    void EnableCCOutput1() const { ITmr->CCER |= TIM_CC1E; ITmr->BDTR = 0x8000U; }
    void EnableCCOutput2() const { ITmr->CCER |= TIM_CC2E; ITmr->BDTR = 0x8000U; }
    void EnableCCOutput3() const { ITmr->CCER |= TIM_CC3E; ITmr->BDTR = 0x8000U; }
    void EnableCCOutput4() const { ITmr->CCER |= TIM_CC4E; ITmr->BDTR = 0x8000U; }

    // Master/Slave
    enum MasterMode_t {
        mmReset=0x00, mmEnable=0x10, mmUpdate=0x20, mmComparePulse=0x30,
        mmCompare1=0x40, mmCompare2=0x50, mmCompare3=0x60, mmCompare4=0x70};
    enum SlaveMode_t {smDisable=0, smEncoder1=1, smEncoder2=2, smEncoder3=3,
        smReset=4, smGated=5, smTrigger=6, smExternal=7};
    enum TrigInput_t {tiITR0=0x00, tiITR1=0x10, tiITR2=0x20, tiITR3=0x30,
        tiTIED=0x40, tiTI1FP1=0x50, tiTI2FP2=0x60, tiETRF=0x70};

    void SetTriggerInput(TrigInput_t TrgInput) const {
        ITmr->SMCFGR = (ITmr->SMCFGR & ~TIM_TS) | (uint16_t)TrgInput;
    }
    void SetEtrPolarity(Inverted_t AInverted) {
        if(AInverted == invInverted) ITmr->SMCFGR |= TIM_ETP;
        else ITmr->SMCFGR &= ~TIM_ETP;
    }
    void SelectMasterMode(MasterMode_t MasterMode) const {
        ITmr->CTLR2 = (ITmr->CTLR2 & ~TIM_MMS) | (uint16_t)MasterMode;
    }
    void SelectSlaveMode(SlaveMode_t SlaveMode) const {
        ITmr->SMCFGR = (ITmr->SMCFGR & ~TIM_SMS) | (uint16_t)SlaveMode;
    }

    // DMA, Irq, Evt
    enum FirstReg_t { regCTLR1=0, regCTLR2=1, regSMCFGR=2, regDMAINTENR=3,
        regINTFR=4, regSWEVGR=5, regCHCTLR1=6, regCHCTLR2=7, regCCER=8,
        regCNT=9, regPSC=10, regATRLR=11, regREPCTR=12,
        regCH1CVR=13, regCH2CVR=14, regCH3CVR=15, regCH4CVR=16,
        regBDTR=17, regDMACFGR=18
    };
    void SetupBurstDma(uint16_t BurstLen, FirstReg_t FirstRegOffset) const {
        ITmr->DMACFGR = ((BurstLen - 1U) << 8) | (uint16_t)FirstRegOffset;
    }

    void EnableDmaOnTrigger() const { ITmr->DMAINTENR |= TIM_TDE; }
    void EnableDmaOnUpdate() const { ITmr->DMAINTENR |= TIM_UDE; }
    void EnableDMAOnCapture(uint16_t CaptureReq) const { ITmr->DMAINTENR |= (1U << (CaptureReq + 8U)); }
    void GenerateUpdateEvt()  const { ITmr->SWEVGR = TIM_UG; }
    // Enables
    void EnableIrq(IRQn_Type IrqChnl, uint32_t IrqPriority) const {
        NVIC_SetPriority(IrqChnl, IrqPriority);
        NVIC_EnableIRQ(IrqChnl);
    }
    void EnableIrqOnUpdate()   const { ITmr->DMAINTENR |= TIM_UIE; }
    void EnableIrqOnCompare1() const { ITmr->DMAINTENR |= TIM_CC1IE; }
    void EnableIrqOnCompare2() const { ITmr->DMAINTENR |= TIM_CC2IE; }
    void EnableIrqOnCompare3() const { ITmr->DMAINTENR |= TIM_CC3IE; }
    void EnableIrqOnCompare4() const { ITmr->DMAINTENR |= TIM_CC4IE; }
    // Clears
    void ClearUpdateIrqPendingBit()   const { ITmr->INTFR &= ~TIM_UIF; }
    void ClearCompare1IrqPendingBit() const { ITmr->INTFR &= ~TIM_CC1IF; }
    void ClearCompare2IrqPendingBit() const { ITmr->INTFR &= ~TIM_CC2IF; }
    void ClearCompare3IrqPendingBit() const { ITmr->INTFR &= ~TIM_CC3IF; }
    void ClearCompare4IrqPendingBit() const { ITmr->INTFR &= ~TIM_CC4IF; }
    // Checks
    bool IsEnabled()            const { return (ITmr->CTLR1 & TIM_CEN); }
    bool IsCompare1IrqEnabled() const { return (ITmr->DMAINTENR & TIM_CC1IE); }
    bool IsCompare2IrqEnabled() const { return (ITmr->DMAINTENR & TIM_CC2IE); }
    bool IsCompare3IrqEnabled() const { return (ITmr->DMAINTENR & TIM_CC3IE); }
    bool IsCompare4IrqEnabled() const { return (ITmr->DMAINTENR & TIM_CC4IE); }
    bool IsUpdateIrqFired()   const { return (ITmr->INTFR & TIM_UIF); }
    bool IsCompare1IrqFired() const { return (ITmr->INTFR & TIM_CC1IF); }
    bool IsCompare2IrqFired() const { return (ITmr->INTFR & TIM_CC2IF); }
    bool IsCompare3IrqFired() const { return (ITmr->INTFR & TIM_CC3IF); }
    bool IsCompare4IrqFired() const { return (ITmr->INTFR & TIM_CC4IF); }
};
#endif


#if 0 // ========================= Time and Delay ==============================
class Time_t : private Timer_t {
public:
    Time_t(TIM_TypeDef *APTimer) : Timer_t(APTimer) {}
    void Init();
    uint32_t GetCurrent() { return GetCounter(); }
    void Wait(uint16_t Time_ms);
    uint32_t ElapsedSince(uint32_t Start);
};

// Place somewhere Time_t Time{TIME_TIMER};

static inline void DelayLoop(volatile uint32_t ACounter) { while(ACounter--); }
#endif


namespace Flash { // ======================== Flash ============================
// Flash freq must not be greater than 60 MHz

//void EnableEnhancedReadMode();


//void SetupLatency(uint32_t FMHz);

} // Flash

namespace Clk { // ======================= Clocking ============================

enum ClkSrc_t {clksrcHSI=0b00, clksrcHSE=0b01, clksrcPLL=0b10};
enum PllSrc_t {pllSrcHSI, pllSrcHSIdiv2, pllSrcPrediv};
enum Pll1Mul_t {
    pllmul3=0x1, pllmul4=0x2, pllmul5=0x3, pllmul6=0x4, pllmul6d5=0xD, pllmul7=0x5, pllmul8=0x6,
    pllmul9=0x7, pllmul10=0x8, pllmul11=0x9, pllmul12=0xA, pllmul13=0xB, pllmul14=0xC,
    pllmul15=0xE, pllmul16=0xF, pllmul18=0x0
};
enum Prediv_t {
    prediv1=0, prediv2=1, prediv3=2, prediv4=3, prediv5=4, prediv6=5, prediv7=6, prediv8=7,
    prediv9=8, prediv10=9, prediv11=10, prediv12=11, prediv13=12, prediv14=13, prediv15=14, prediv16=15
};
enum Prediv1Src_t { prediv1srcHSE=0, prediv1srcPLL2=(1UL<<16) };
enum Pll23Mul_t {
    pll23mul2d5=0x0, pll23mul12d5=0x1, pll23mul4=0x2, pll23mul5=0x3, pll23mul6=0x4, pll23mul7=0x5,
    pll23mul8=0x6, pll23mul9=0x7, pll23mul10=0x8, pll23mul11=0x9, pll23mul12=0xA, pll23mul13=0xB,
    pll23mul14=0xC, pll23mul15=0xD, pll23mul16=0xE, pll23mul20=0xF
};


// Frequency values
extern uint32_t AHBFreqHz, APB1FreqHz, APB2FreqHz;

// Enable / Disable
uint8_t EnableHSE();
uint8_t EnablePll();
bool IsHSEReady();

// Setups
void SetupPll(Pll1Mul_t PllMul1, PllSrc_t PllSrc, Prediv_t Prediv=prediv1, Prediv1Src_t Prediv1Src=prediv1srcHSE);
void SetupPll2Mul(Pll23Mul_t PllMul2);
void SetupPll3Mul(Pll23Mul_t PllMul3);
void SetupPrediv2(Prediv_t Prediv2);

void SelectSysClkSrc(ClkSrc_t ClkSrc);

void UpdateFreqValues();
void PrintFreqs();

} // clocking

#endif /* KL_LIB_LIBV10X_H_ */
