#include "sam.h"

namespace System
{
   enum IRQ_NUM : uint32_t
   {
      IRQ_SYSTEM = SYSTEM_IRQn,
      IRQ_WDT = WDT_IRQn,
      IRQ_RTC = RTC_IRQn,
      IRQ_EIC = EIC_IRQn,
      IRQ_NVMCTRL = NVMCTRL_IRQn,
      IRQ_DMAC = DMAC_IRQn,
      IRQ_USB = USB_IRQn,
      IRQ_EVSYS = EVSYS_IRQn,
      IRQ_SERCOM0 = SERCOM0_IRQn,
      IRQ_SERCOM1 = SERCOM1_IRQn,
      IRQ_SERCOM2 = SERCOM2_IRQn,
      IRQ_SERCOM3 = SERCOM3_IRQn,
      IRQ_SERCOM4 = SERCOM4_IRQn,
      IRQ_SERCOM5 = SERCOM5_IRQn,
      IRQ_TCC0 = TCC0_IRQn,
      IRQ_TCC1 = TCC1_IRQn,
      IRQ_TCC2 = TCC2_IRQn,
      IRQ_TC0 = TC0_IRQn,
      IRQ_TC1 = TC1_IRQn, 
      IRQ_TC4 = TC4_IRQn,
      IRQ_ADC = ADC_IRQn,
      IRQ_AC = AC_IRQn,
      IRQ_DAC = DAC_IRQn,
      IRQ_PTC = PTC_IRQn,
      IRQ_AES = AES_IRQn,
      IRQ_TRNG = TRNG_IRQn,
   }
   
   typedef void (*IRQ_Handler_t) ()
   
   class Interrupt
   {
   private:
      IRQ_Handler_t IRQ_handlers[PERIPH_COUNT_IRQn];
      
   public:
      void Register(IRQ_NUM num, IRQ_Handler_t handler) {IRQ_Handlers[num] = handler;}
      virtual void ISR() = 0;
      
      static void SYSTEM_Handler();
      static void WDT_Handler();
      static void RTC_Handler();
      static void EIC_Handler();
      static void NVMCTRL_Handler();
      static void DMAC_Handler();
      #ifdef ID_USB
      static void USB_Handler();
      #endif
      static void EVSYS_Handler();
      static void SERCOM0_Handler();
      static void SERCOM1_Handler();
      static void SERCOM2_Handler();
      static void SERCOM3_Handler();
      #ifdef ID_SERCOM4
      static void SERCOM4_Handler();
      #endif
      #ifdef ID_SERCOM5
      static void SERCOM5_Handler();
      #endif
      static void TCC0_Handler();
      static void TCC1_Handler();
      static void TCC2_Handler();
      static void TC0_Handler();
      static void TC1_Handler();
      #ifdef ID_TC2
      static void TC2_Handler();
      #endif
      #ifdef ID_TC3
      static void TC3_Handler();
      #endif
      static void TC4_Handler();
      #ifdef ID_ADC
      static void ADC_Handler();
      #endif
      #ifdef ID_AC
      static void AC_Handler();
      #endif
      #ifdef ID_DAC
      static void DAC_Handler();
      #endif
      #ifdef ID_PTC
      static void PTC_Handler()
      #endif
      #ifdef ID_AES
      static void AES_Handler();
      #endif
      #ifdef ID_TRNG
      static void TRNG_Handler();
      #endif
}
