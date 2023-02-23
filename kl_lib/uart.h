/*
 * uart.h
 *
 *  Created on: 19.02.2023
 *      Author: Kreyl
 */

#ifndef KL_LIB_UART_H_
#define KL_LIB_UART_H_

#include <inttypes.h>
#include "ch32v30x.h"
#include "shell.h"

struct UartParams_t {
    uint32_t Baudrate;
    USART_TypeDef* Uart;
    GPIO_TypeDef *PGpioTx, *PGpioRx;
    uint16_t PinTx, PinRx;
    // DMA
    uint32_t DmaChnlTx, DmaChnlRx, DmaModeTx, DmaModeRx;
    // MCU-specific
    UartParams_t(uint32_t ABaudrate, USART_TypeDef* AUart,
            GPIO_TypeDef *APGpioTx, uint16_t APinTx,
            GPIO_TypeDef *APGpioRx, uint16_t APinRx,
            uint32_t ADmaChnlTx, uint32_t ADmaChnlRx,
            uint32_t ADmaModeTx, uint32_t ADmaModeRx
    ) : Baudrate(ABaudrate), Uart(AUart),
            PGpioTx(APGpioTx), PinTx(APinTx), PGpioRx(APGpioRx), PinRx(APinRx),
            DmaChnlTx(ADmaChnlTx), DmaChnlRx(ADmaChnlRx),
            DmaModeTx(ADmaModeTx), DmaModeRx(ADmaModeRx)
    {}
};

void UartDmaTxIrqHandler(void *p, uint32_t flags);

class BaseUart_t {
protected:
    const UartParams_t *Params;
    char TXBuf[UART_TXBUF_SZ];
    char *PRead, *PWrite;
    bool ITxDmaIsIdle;
    uint32_t IFullSlotsCount, ITransSize;
    void ISendViaDMA();
    int32_t OldWIndx, RIndx;
    uint8_t IRxBuf[UART_RXBUF_SZ];
    DMA_t DmaTx, DmaRx;
    uint8_t IPutByte(uint8_t b);
    uint8_t IPutByteNow(uint8_t b);
    void IStartTransmissionIfNotYet();
    // ==== Constructor ====
    BaseUart_t(const UartParams_t &APParams) : Params(&APParams)
    , PRead(TXBuf), PWrite(TXBuf), ITxDmaIsIdle(true), IFullSlotsCount(0), ITransSize(0),
        OldWIndx(0), RIndx(0),
        DmaTx(Params->DmaChnlTx, UartDmaTxIrqHandler, this), DmaRx(Params->DmaChnlRx) {}
    uint8_t GetByte(uint8_t *b);
public:
    void Init();
    void Shutdown();
    void OnClkChange();
    // Enable/Disable
//    void EnableTx()  { Params->Uart->CR1 |= USART_CR1_TE; }
//    void DisableTx() { Params->Uart->CR1 &= ~USART_CR1_TE; }
//    void EnableRx()  { Params->Uart->CR1 |= USART_CR1_RE; }
//    void DisableRx() { Params->Uart->CR1 &= ~USART_CR1_RE; }
//    void FlushTx() { while(!IDmaIsIdle) chThdSleepMilliseconds(1); }  // wait DMA
//    void FlushRx();
//    void EnableTCIrq(const uint32_t Priority, ftVoidVoid ACallback);
    // Inner use
    void IRQDmaTxHandler();
//    uint8_t GetByte(uint8_t *b);
//    virtual void OnUartIrqI(uint32_t flags) = 0;
};

class CmdUart_t : public BaseUart_t, public PrintfHelper_t, public Shell_t {
private:
    uint8_t IPutChar(char c) { return IPutByte(c); }
    void IStartTransmissionIfNotYet() { BaseUart_t::IStartTransmissionIfNotYet(); }
public:
    CmdUart_t(const UartParams_t &APParams) : BaseUart_t(APParams) {}
//    uint8_t TryParseRxBuff() {
//        uint8_t b;
//        while(GetByte(&b) == retvOk) {
//            if(Cmd.PutChar(b) == pdrNewCmd) return retvOk;
//        } // while get byte
//        return retvFail;
//    }
    void Print(const char *format, ...) {
        va_list args;
        va_start(args, format);
        IVsPrintf(format, args);
        va_end(args);
    }
    uint8_t ReceiveBinaryToBuf(uint8_t *ptr, uint32_t Len, uint32_t Timeout_ms) {return 0;}
    uint8_t TransmitBinaryFromBuf(uint8_t *ptr, uint32_t Len, uint32_t Timeout_ms) {return 0;}
//    void OnUartIrqI(uint32_t flags);
};



#endif /* KL_LIB_UART_H_ */
