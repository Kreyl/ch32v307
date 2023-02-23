/*
 * board.h
 *
 *  Created on: 19.02.2023
 *      Author: Kreyl
 */

#ifndef BOARD_H_
#define BOARD_H_

#define BOARD_NAME          "V307EVB"
#define APP_NAME            "V307EVB"

#define MCU					CH32V30x

#ifndef TRUE
#define TRUE            1
#endif

// Freq of external crystal if any. Leave it here even if not used.
#define CRYSTAL_FREQ_HZ 8000000

#if 1 // ========================== GPIO =======================================
// PortMinTim_t: GPIO, Pin, Tim, TimChnl, invInverted, omPushPull, TopValue

// UART
#define UART_GPIO       GPIOA
#define UART_TX_PIN     9
#define UART_RX_PIN     10


#endif // GPIO

#if 1 // ============================ DMA ======================================
#define UART_DMA_TX     DMA_CHNL(1, 4)
#define DMA1_CH4_IRQ_EN TRUE
#define UART_DMA_RX     DMA_CHNL(1, 5)
#define UART_DMA_TX_MODE (DMA_PRIORITY_LOW | DMA_MSIZE_8_BIT | DMA_PSIZE_8_BIT | \
        DMA_MEM_INC | DMA_DIR_MEM2PER | DMA_TCIE)
#define UART_DMA_RX_MODE (DMA_PRIORITY_LOW | DMA_MSIZE_8_BIT | DMA_PSIZE_8_BIT | \
        DMA_MEM_INC | DMA_DIR_PER2MEM | DMA_CIRC)


#endif

#if 1 // =========================== UART ======================================
#define PRINTF_FLOAT_EN FALSE
#define UART_TXBUF_SZ   1024
#define UART_RXBUF_SZ   128
#define CMD_BUF_SZ      128
#define UART_RX_POLL_MS 99

#define CMD_UART        USART1


#define CMD_UART_PARAMS \
    CMD_UART, UART_GPIO, UART_TX_PIN, UART_GPIO, UART_RX_PIN, \
    UART_DMA_TX, UART_DMA_RX, UART_DMA_TX_MODE, UART_DMA_RX_MODE
#endif

#endif /* BOARD_H_ */
