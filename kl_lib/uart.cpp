/*
 * uart.cpp
 *
 *  Created on: 19.02.2023
 *      Author: layst
 */

#include <libCH32V30x.h>
#include "uart.h"

// ==== TX DMA IRQ ====
// Wrapper for TX IRQ
void UartDmaTxIrqHandler(void *p, uint32_t flags) { ((BaseUart_t*)p)->IRQDmaTxHandler(); }

void BaseUart_t::IRQDmaTxHandler() {
//    USART1->DATAR = '#';
    DmaTx.Disable(); // Registers may be changed only when stream is disabled
    IFullSlotsCount -= ITransSize;
    PRead += ITransSize;
    if(PRead >= (TXBuf + UART_TXBUF_SZ)) PRead = TXBuf; // Circulate pointer
    if(IFullSlotsCount == 0) {  // Nothing left to send
        ITxDmaIsIdle = true;
    }
    else ISendViaDMA();
}

void BaseUart_t::ISendViaDMA() {
    uint32_t PartSz = (TXBuf + UART_TXBUF_SZ) - PRead; // Cnt from PRead to end of buf
    ITransSize = MIN_(IFullSlotsCount, PartSz);
    if(ITransSize != 0) {
        ITxDmaIsIdle = false;
        DmaTx.SetMemoryAddr(PRead);
        DmaTx.SetTransferDataCnt(ITransSize);
        Params->Uart->STATR &= ~USART_STATR_TC; // Clear TC flag
        DmaTx.Enable();
    }
}


uint8_t BaseUart_t::IPutByteNow(uint8_t b) {
    while(!(Params->Uart->STATR & USART_STATR_TXE));
    Params->Uart->DATAR = b;
    while(!(Params->Uart->STATR & USART_STATR_TXE));
    return retvOk;
}

uint8_t BaseUart_t::IPutByte(uint8_t b) {
    if(IFullSlotsCount >= UART_TXBUF_SZ) return retvOverflow;
    *PWrite++ = b;
    if(PWrite >= &TXBuf[UART_TXBUF_SZ]) PWrite = TXBuf;   // Circulate buffer
    IFullSlotsCount++;
    return retvOk;
}

void BaseUart_t::IStartTransmissionIfNotYet() {
    if(ITxDmaIsIdle) ISendViaDMA();
}

void BaseUart_t::Init() {
    // ==== TX ====
    Gpio::SetupAlterFunc(GPIOA, Params->PinTx, Gpio::outmodePushPull);
    Gpio::SetupAlterFunc(Params->PGpioTx, Params->PinTx, Gpio::outmodePushPull, Gpio::speed50MHz);

    // ==== Clock ====
    if     (Params->Uart == USART1) { RCC->APB2PCENR |= RCC_USART1EN; /*IUart1 = this; */}
    else if(Params->Uart == USART2) { RCC->APB1PCENR |= RCC_USART2EN; /*IUart2 = this; */}
//    else if(Params->Uart == USART3) { RCC->APB1PCENR |= RCC_USART3EN; /*IUart3 = this; */}

    OnClkChange();  // Setup baudrate

    Params->Uart->CTLR2 = 0;  // Nothing interesting there
    // ==== DMA TX ====
    DmaTx.Init();
    DmaTx.SetPeriphAddr(&Params->Uart->DATAR);
    DmaTx.SetMode(Params->DmaModeTx);
    ITxDmaIsIdle = true;

    // ==== RX ====
//    PinSetupAlterFunc(Params->PGpioRx, Params->PinRx, omOpenDrain, pudPullUp, PinAF);
    // Remap DMA request if needed
    // DMA
//    PDmaRx = dmaStreamAlloc(Params->DmaRxID, IRQ_PRIO_MEDIUM, nullptr, NULL);
//    dmaStreamSetPeripheral(PDmaRx, &Params->Uart->RDR);
//    dmaStreamSetMemory0   (PDmaRx, IRxBuf);
//    dmaStreamSetTransactionSize(PDmaRx, UART_RXBUF_SZ);
//    dmaStreamSetMode      (PDmaRx, Params->DmaModeRx);
//    dmaStreamEnable       (PDmaRx);
    // Final setup
    Params->Uart->CTLR1 = USART_CTLR1_TE | USART_CTLR1_RE;     // TX & RX en, 8bit, no parity
    Params->Uart->CTLR3 = USART_CTLR3_DMAT | USART_CTLR3_DMAR; // Enable DMA at TX & RX
    Params->Uart->CTLR1 |= USART_CTLR1_UE;    // Enable USART
}

void BaseUart_t::OnClkChange() {
    if(Params->Uart == USART1) Params->Uart->BRR = Clk::APB2FreqHz / Params->Baudrate;
    else                       Params->Uart->BRR = Clk::APB1FreqHz / Params->Baudrate;
}

