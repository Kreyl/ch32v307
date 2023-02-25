
#include <inttypes.h>
#include "libCH32V30x.h"
#include "uart.h"

#if 1 // ======================== Variables and defines ========================
// Forever
static const UartParams_t CmdUartParams(115200, CMD_UART_PARAMS);
CmdUart_t Uart{CmdUartParams};

#endif


#define NPX_TIM         TIM9
#define NPX_TIM_TOP     120
#define NPX_TIM_SHORT   38
#define NPX_TIM_LONG    82

#define BUF_SZ  16
uint16_t IBuf1[BUF_SZ] = {
        0, 0, 0, 0,
        0, 0, 0, 0,
        NPX_TIM_SHORT, NPX_TIM_SHORT, NPX_TIM_SHORT, NPX_TIM_LONG,
        NPX_TIM_SHORT, NPX_TIM_LONG,  NPX_TIM_LONG,  NPX_TIM_SHORT,
};

const Timer_t Tmr = {NPX_TIM};

DMA_t DmaTmr(DMA_CHNL(2, 6));
uint32_t DmaTimMode = (DMA_PRIORITY_VERYHIGH | DMA_MSIZE_16_BIT | DMA_PSIZE_16_BIT | DMA_MEM_INC | DMA_DIR_MEM2PER | DMA_CIRC);

int main() {
#if 1 // ==== Clock setup ====
//    Flash::EnablePrefetch();
//    Clk
    if(Clk::EnableHSE() == retvOk) {
        Clk::SetupPll(Clk::pllmul12, Clk::pllSrcPrediv, Clk::prediv1, Clk::prediv1srcHSE);
        if(Clk::EnablePll() == retvOk) Clk::SelectSysClkSrc(Clk::clksrcPLL);
    }
    Clk::UpdateFreqValues();
#endif
	Uart.Init();
	Printf("\r%S %S\r", APP_NAME, XSTRINGIFY(BUILD_TIME));
	Clk::PrintFreqs();
	Printf(Clk::IsHSEReady()? "Crystal ok\r" : "Crystal FAIL\r");

	Gpio::SetupOut(GPIOA, 8, Gpio::outmodePushPull);

	Gpio::SetupAlterFunc(GPIOA, 2, Gpio::outmodePushPull, Gpio::speed2MHz);
	Gpio::SetupAlterFunc(GPIOA, 3, Gpio::outmodePushPull, Gpio::speed2MHz);

	Tmr.Init();
	Tmr.EnableArrBuffering();
	Tmr.SetTopValue(NPX_TIM_TOP);
	Tmr.EnableDmaOnUpdate();
	Tmr.EnableCCOutput1();
	Tmr.EnableCCOutput2();
	Tmr.SetupBurstDma(2, Timer_t::regCH1CVR);
	Tmr.SetupCCOutput1();
	Tmr.SetupCCOutput2();

    DmaTmr.Init(&NPX_TIM->DMAADR, IBuf1, DmaTimMode, BUF_SZ);
    DmaTmr.Enable();

    Tmr.Enable();

	while(true) {
		for(volatile uint32_t i=0; i<1000000; i++);
		Gpio::Toggle(GPIOA, 8);
//		Printf("%u ", i++);
	}
}

