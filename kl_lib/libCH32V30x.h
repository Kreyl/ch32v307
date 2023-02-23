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
enum OutMode_t {omPushPull = 0b00UL, omOpenDrain = 0b01UL};
enum Speed_t {
    ps10MHz = 0b01UL,
    ps2MHz = 0b10UL,
    ps50MHz = 0b11UL
};
#define PIN_SPEED_DEFAULT   ps10MHz

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

void EnablePrefetch();
void SetupLatency(uint32_t FMHz);

} // Flash

namespace Clk { // ======================= Clocking ============================

enum ClkSrc_t {csHSI, csHSE, csPLL};
enum PllSrc_t {pllSrcHSIdiv2, pllSrcPrediv};


// Frequency values
extern uint32_t AHBFreqHz, APB1FreqHz, APB2FreqHz;

void UpdateFreqValues();
void PrintFreqs();

} // clocking

#endif /* KL_LIB_LIBV10X_H_ */
