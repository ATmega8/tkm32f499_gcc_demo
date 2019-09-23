/**
  ******************************************************************************
  * @file      startup_stm32f401xc.s
  * @author    MCD Application Team
  * @version   V2.6.1
  * @date      14-February-2017
  * @brief     STM32F401xCxx Devices vector table for GCC based toolchains. 
  *            This module performs:
  *                - Set the initial SP
  *                - Set the initial PC == Reset_Handler,
  *                - Set the vector table entries with the exceptions ISR address
  *                - Branches to main in the C library (which eventually
  *                  calls main()).
  *            After Reset the Cortex-M4 processor is in Thread mode,
  *            priority is Privileged, and the Stack is set to Main.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
    
.syntax unified
.cpu cortex-m4
.fpu softvfp
.thumb

.global  g_pfnVectors
.global  Default_Handler

/* start address for the initialization values of the .data section. 
defined in linker script */
.word  _sidata
/* start address for the .data section. defined in linker script */  
.word  _sdata
/* end address for the .data section. defined in linker script */
.word  _edata
/* start address for the .bss section. defined in linker script */
.word  _sbss
/* end address for the .bss section. defined in linker script */
.word  _ebss
/* stack used for SystemInit_ExtMemCtl; always internal RAM used */

/**
 * @brief  This is the code that gets called when the processor first
 *          starts execution following a reset event. Only the absolutely
 *          necessary set is performed, after which the application
 *          supplied main() routine is called. 
 * @param  None
 * @retval : None
*/

.section  .text.Reset_Handler
.weak  Reset_Handler
.type  Reset_Handler, %function
Reset_Handler:
  ldr   sp, = _estack/* set stack pointer */

/* Copy the data segment initializers from flash to SRAM */
  movs  r1, #0
  b  LoopCopyDataInit

CopyDataInit:
  ldr  r3, =_sidata
  ldr  r3, [r3, r1]
  str  r3, [r0, r1]
  adds  r1, r1, #4

LoopCopyDataInit:
  ldr  r0, =_sdata
  ldr  r3, =_edata
  adds  r2, r0, r1
  cmp  r2, r3
  bcc  CopyDataInit
  ldr  r2, =_sbss
  b  LoopFillZerobss
/* Zero fill the bss segment. */
FillZerobss:
  movs  r3, #0
  str  r3, [r2], #4

LoopFillZerobss:
  ldr  r3, = _ebss
  cmp  r2, r3
  bcc  FillZerobss

/* Call static constructors */
  bl __libc_init_array
/* Call the clock system intitialization function.*/
  ldr     r0, =0xe000ed88    /*ʹ�ܸ������� CP10,CP11*/
  ldr     r1,[r0]
  orr     r1,r1,#(0xf << 20)
  str     r1,[r0]
  bl main
  bx lr
.size  Reset_Handler, .-Reset_Handler

/**
 * @brief  This is the code that gets called when the processor receives an 
 *         unexpected interrupt.  This simply enters an infinite loop, preserving
 *         the system state for examination by a debugger.
 * @param  None     
 * @retval None       
*/
    .section  .text.Default_Handler,"ax",%progbits
Default_Handler:
Infinite_Loop:
  b  Infinite_Loop
  .size  Default_Handler, .-Default_Handler
/******************************************************************************
*
* The minimal vector table for a Cortex M3. Note that the proper constructs
* must be placed on this to ensure that it ends up at physical address
* 0x0000.0000.
* 
*******************************************************************************/
   .section  .isr_vector,"a",%progbits
  .type  g_pfnVectors, %object
  .size  g_pfnVectors, .-g_pfnVectors
    
g_pfnVectors:
  .word  _estack
  .word  Reset_Handler
  .word  NMI_Handler
  .word  HardFault_Handler
  .word  MemManage_Handler
  .word  BusFault_Handler
  .word  UsageFault_Handler
  .word  0
  .word  0
  .word  0
  .word  0
  .word  SVC_Handler
  .word  DebugMon_Handler
  .word  0
  .word  PendSV_Handler
  .word  SysTick_Handler
  
  /* External Interrupts */
  .word     WWDG_IRQHandler                   /* Window WatchDog              */                                        
  .word     0                                 /* PVD through EXTI Line detection */                        
  .word     TAMP_STAMP_IRQHandler             /* Tamper and TimeStamps through the EXTI line */            
  .word     RTC_WKUP_IRQHandler               /* RTC Wakeup through the EXTI line */                      
  .word     0                                 /* FLASH                        */                                          
  .word     RCC_IRQHandler                    /* RCC                          */                                            
  .word     EXTI0_IRQHandler                  /* EXTI Line0                   */                        
  .word     EXTI1_IRQHandler                  /* EXTI Line1                   */                          
  .word     EXTI2_IRQHandler                  /* EXTI Line2                   */                          
  .word     EXTI3_IRQHandler                  /* EXTI Line3                   */                          
  .word     EXTI4_IRQHandler                  /* EXTI Line4                   */                          
  .word     DMA1_Channel1_IRQHandler          /* DMA1 Stream 0                */                  
  .word     DMA1_Channel2_IRQHandler          /* DMA1 Stream 1                */                   
  .word     DMA1_Channel3_IRQHandler          /* DMA1 Stream 2                */                   
  .word     DMA1_Channel4_IRQHandler          /* DMA1 Stream 3                */                   
  .word     DMA1_Channel5_IRQHandler          /* DMA1 Stream 4                */                   
  .word     DMA1_Channel6_IRQHandler          /* DMA1 Stream 5                */                   
  .word     DMA1_Channel7_IRQHandler          /* DMA1 Stream 6                */                   
  .word     ADC_IRQHandler                    /* ADC1                         */                   
  .word     CAN1_IRQHandler              	  /* Reserved                     */                         
  .word     0              					  /* Reserved                     */                          
  .word     0                                 /* Reserved                     */                          
  .word     0                                 /* Reserved                     */                          
  .word     EXTI9_5_IRQHandler                /* External Line[9:5]s          */                          
  .word     TIM1_BRK_IRQHandler               /* TIM1 Break and TIM9          */         
  .word     TIM1_UP_IRQHandler          /* TIM1 Update and TIM10        */         
  .word     TIM1_TRG_COM_IRQHandler     /* TIM1 Trigger and Commutation and TIM11 */
  .word     TIM1_CC_IRQHandler                /* TIM1 Capture Compare         */                          
  .word     TIM3_IRQHandler                   /* TIM2                         */                   
  .word     TIM4_IRQHandler                   /* TIM3                         */                   
  .word     TIM5_IRQHandler                   /* TIM4                         */                   
  .word     TIM6_IRQHandler                   /* I2C1 Event                   */                          
  .word     TIM7_IRQHandler
  .word     I2C1_IRQHandler                   /* I2C1 Error                   */                          
  .word     I2C2_IRQHandler                /* I2C2 Event                   */                          
  .word     SPI1_IRQHandler                   /* SPI1                         */                   
  .word     SPI2_IRQHandler                   /* SPI2                         */                   
  .word     UART1_IRQHandler                 /* USART1                       */                   
  .word     UART2_IRQHandler                 /* USART2                       */                   
  .word     UART3_IRQHandler               				  /* Reserved                     */                   
  .word     EXTI15_10_IRQHandler              /* External Line[15:10]s        */                          
  .word     RTC_Alarm_IRQHandler              /* RTC Alarm (A and B) through EXTI Line */                 
  .word     USBAwake_IRQHandler            /* USB OTG FS Wakeup through EXTI line */                       
  .word     TIM2_BRK_IRQHandler                                 /* Reserved     				        */         
  .word     TIM2_UP_IRQHandler                                 /* Reserved       			        */         
  .word     TIM2_TRG_COM_IRQHandler                                 /* Reserved 					          */
  .word     TIM2_CC_IRQHandler                                 /* Reserved                     */                          
  .word     DMA1_Channel8_IRQHandler           /* DMA1 Stream7                 */                          
  .word     TK80_IRQHandler                                 /* Reserved                     */                   
  .word     SDIO1_IRQHandler                   /* SDIO                         */                   
  .word     SDIO2_IRQHandler                   /* TIM5                         */                   
  .word     SPI3_IRQHandler                   /* SPI3                         */                   
  .word     UART4_IRQHandler                                /* Reserved                     */                   
  .word     UART5_IRQHandler                                /* Reserved                     */                   
  .word     0                                 /* Reserved                     */                   
  .word     TIM8_IRQHandler                                 /* Reserved                     */
  .word     DMA2_Channel1_IRQHandler           /* DMA2 Stream 0                */                   
  .word     DMA2_Channel2_IRQHandler           /* DMA2 Stream 1                */                   
  .word     DMA2_Channel3_IRQHandler           /* DMA2 Stream 2                */                   
  .word     DMA2_Channel4_IRQHandler           /* DMA2 Stream 3                */                   
  .word     DMA2_Channel5_IRQHandler           /* DMA2 Stream 4                */                   
  .word     TIM9_IRQHandler                    			        /* Reserved                     */                   
  .word     TIM10_IRQHandler             					          /* Reserved                     */                     
  .word     CAN2_IRQHandler             					          /* Reserved                     */                          
  .word     0             					          /* Reserved                     */                          
  .word     0              					          /* Reserved                     */                          
  .word     0              					          /* Reserved                     */                          
  .word     USB_IRQHandler                 /* USB OTG FS                   */                   
  .word     DMA2_Channel6_IRQHandler           /* DMA2 Stream 5                */                   
  .word     DMA2_Channel7_IRQHandler           /* DMA2 Stream 6                */                   
  .word     DMA2_Channel8_IRQHandler           /* DMA2 Stream 7                */                   
  .word     0                                 /* USART6                       */                    
  .word     I2C3_IRQHandler                /* I2C3 event                   */                          
  .word     I2C4_IRQHandler                /* I2C3 error                   */                          
  .word     0                                 /* Reserved                     */                   
  .word     0                                 /* Reserved                     */                   
  .word     0                                 /* Reserved                     */                         
  .word     0                                 /* Reserved                     */                   
  .word     0                                 /* Reserved                     */                   
  .word     0                                 /* Reserved                     */                   
  .word     0                                 /* Reserved                     */
  .word     FPU_IRQHandler                    /* FPU                          */
  .word     0                                 /* Reserved                     */                   
  .word     0                                 /* Reserved                     */
  .word     SPI4_IRQHandler                   /* SPI4                         */     
  .word     0
  .word     TOUCHPAD_IRQHandler
  .word     QSPI_IRQHandler
  .word     LTDC_IRQHandler
  .word     0
  .word     I2S1_IRQHandler
                    
/*******************************************************************************
*
* Provide weak aliases for each Exception handler to the Default_Handler. 
* As they are weak aliases, any function with the same name will override 
* this definition.
* 
*******************************************************************************/
   .weak      NMI_Handler
   .thumb_set NMI_Handler,Default_Handler
  
   .weak      HardFault_Handler
   .thumb_set HardFault_Handler,Default_Handler
  
   .weak      MemManage_Handler
   .thumb_set MemManage_Handler,Default_Handler
  
   .weak      BusFault_Handler
   .thumb_set BusFault_Handler,Default_Handler

   .weak      UsageFault_Handler
   .thumb_set UsageFault_Handler,Default_Handler

   .weak      SVC_Handler
   .thumb_set SVC_Handler,Default_Handler

   .weak      DebugMon_Handler
   .thumb_set DebugMon_Handler,Default_Handler

   .weak      PendSV_Handler
   .thumb_set PendSV_Handler,Default_Handler

   .weak      SysTick_Handler
   .thumb_set SysTick_Handler,Default_Handler              
  
   .weak      WWDG_IRQHandler                   
   .thumb_set WWDG_IRQHandler,Default_Handler      
                  
   .weak      TAMP_STAMP_IRQHandler            
   .thumb_set TAMP_STAMP_IRQHandler,Default_Handler
            
   .weak      RTC_WKUP_IRQHandler                  
   .thumb_set RTC_WKUP_IRQHandler,Default_Handler
            
   .weak      RCC_IRQHandler      
   .thumb_set RCC_IRQHandler,Default_Handler
                  
   .weak      EXTI0_IRQHandler         
   .thumb_set EXTI0_IRQHandler,Default_Handler
                  
   .weak      EXTI1_IRQHandler         
   .thumb_set EXTI1_IRQHandler,Default_Handler
                     
   .weak      EXTI2_IRQHandler         
   .thumb_set EXTI2_IRQHandler,Default_Handler 
                 
   .weak      EXTI3_IRQHandler         
   .thumb_set EXTI3_IRQHandler,Default_Handler
                        
   .weak      EXTI4_IRQHandler         
   .thumb_set EXTI4_IRQHandler,Default_Handler
                  
   .weak      DMA1_Channel1_IRQHandler               
   .thumb_set DMA1_Channel1_IRQHandler,Default_Handler
         
   .weak      DMA1_Channel2_IRQHandler               
   .thumb_set DMA1_Channel2_IRQHandler,Default_Handler
                  
   .weak      DMA1_Channel3_IRQHandler               
   .thumb_set DMA1_Channel3_IRQHandler,Default_Handler
                  
   .weak      DMA1_Channel4_IRQHandler               
   .thumb_set DMA1_Channel4_IRQHandler,Default_Handler 
                 
   .weak      DMA1_Channel5_IRQHandler              
   .thumb_set DMA1_Channel5_IRQHandler,Default_Handler
                  
   .weak      DMA1_Channel6_IRQHandler               
   .thumb_set DMA1_Channel6_IRQHandler,Default_Handler
                  
   .weak      DMA1_Channel7_IRQHandler               
   .thumb_set DMA1_Channel7_IRQHandler,Default_Handler
                  
   .weak      ADC_IRQHandler      
   .thumb_set ADC_IRQHandler,Default_Handler

   .weak      CAN1_IRQHandler      
   .thumb_set CNA1_IRQHandler,Default_Handler
            
   .weak      EXTI9_5_IRQHandler   
   .thumb_set EXTI9_5_IRQHandler,Default_Handler
            
   .weak      TIM1_BRK_IRQHandler            
   .thumb_set TIM1_BRK_IRQHandler,Default_Handler
            
   .weak      TIM1_UP_IRQHandler            
   .thumb_set TIM1_UP_IRQHandler,Default_Handler
      
   .weak      TIM1_TRG_COM_IRQHandler      
   .thumb_set TIM1_TRG_COM_IRQHandler,Default_Handler
      
   .weak      TIM1_CC_IRQHandler   
   .thumb_set TIM1_CC_IRQHandler,Default_Handler
                  
   .weak      TIM3_IRQHandler            
   .thumb_set TIM3_IRQHandler,Default_Handler
                  
   .weak      TIM4_IRQHandler            
   .thumb_set TIM4_IRQHandler,Default_Handler
                  
   .weak      TIM5_IRQHandler            
   .thumb_set TIM5_IRQHandler,Default_Handler

   .weak      TIM6_IRQHandler            
   .thumb_set TIM6_IRQHandler,Default_Handler

   .weak      TIM7_IRQHandler            
   .thumb_set TIM7_IRQHandler,Default_Handler
                  
   .weak      I2C1_IRQHandler   
   .thumb_set I2C1_IRQHandler,Default_Handler
                     
   .weak      I2C2_IRQHandler   
   .thumb_set I2C2_IRQHandler,Default_Handler
                  
   .weak      SPI1_IRQHandler            
   .thumb_set SPI1_IRQHandler,Default_Handler
                        
   .weak      SPI2_IRQHandler            
   .thumb_set SPI2_IRQHandler,Default_Handler
                  
   .weak      UART1_IRQHandler      
   .thumb_set UART1_IRQHandler,Default_Handler
                     
   .weak      UART2_IRQHandler      
   .thumb_set UART2_IRQHandler,Default_Handler
                                  
   .weak      UART3_IRQHandler      
   .thumb_set UART3_IRQHandler,Default_Handler                                

   .weak      EXTI15_10_IRQHandler               
   .thumb_set EXTI15_10_IRQHandler,Default_Handler
               
   .weak      RTC_Alarm_IRQHandler               
   .thumb_set RTC_Alarm_IRQHandler,Default_Handler
            
   .weak      USBAwake_IRQHandler         
   .thumb_set USBAwake_IRQHandler,Default_Handler

   .weak      TIM2_BRK_IRQHandler      
   .thumb_set TIM2_BRK_IRQHandler,Default_Handler                                

   .weak      TIM2_UP_IRQHandler      
   .thumb_set TIM2_UP_IRQHandler,Default_Handler                                

   .weak      TIM2_TRG_COM_IRQHandler      
   .thumb_set TIM2_TRG_COM_IRQHandler,Default_Handler                                

   .weak      TIM2_CC_IRQHandler      
   .thumb_set TIM2_CC_IRQHandler,Default_Handler                                           

   .weak      DMA1_Channel8_IRQHandler               
   .thumb_set DMA1_Channel8_IRQHandler,Default_Handler
                     
   .weak      TK80_IRQHandler            
   .thumb_set TK80_IRQHandler,Default_Handler

   .weak      SDIO1_IRQHandler            
   .thumb_set SDIO1_IRQHandler,Default_Handler
                     
   .weak      SDIO2_IRQHandler            
   .thumb_set SDIO2_IRQHandler,Default_Handler
                     
   .weak      SPI3_IRQHandler            
   .thumb_set SPI3_IRQHandler,Default_Handler

   .weak      UART4_IRQHandler      
   .thumb_set UART4_IRQHandler,Default_Handler
                     
   .weak      UART5_IRQHandler      
   .thumb_set UART5_IRQHandler,Default_Handler
                                  
   .weak      TIM8_IRQHandler      
   .thumb_set TIM8_IRQHandler,Default_Handler
                     
   .weak      DMA2_Channel1_IRQHandler               
   .thumb_set DMA2_Channel1_IRQHandler,Default_Handler
               
   .weak      DMA2_Channel2_IRQHandler               
   .thumb_set DMA2_Channel2_IRQHandler,Default_Handler
                  
   .weak      DMA2_Channel3_IRQHandler               
   .thumb_set DMA2_Channel3_IRQHandler,Default_Handler
            
   .weak      DMA2_Channel4_IRQHandler               
   .thumb_set DMA2_Channel4_IRQHandler,Default_Handler
            
   .weak      DMA2_Channel5_IRQHandler               
   .thumb_set DMA2_Channel5_IRQHandler,Default_Handler

   .weak      TIM9_IRQHandler               
   .thumb_set TIM9_IRQHandler,Default_Handler

   .weak      TIM10_IRQHandler               
   .thumb_set TIM10_IRQHandler,Default_Handler

   .weak      CAN2_IRQHandler               
   .thumb_set CAN2_IRQHandler,Default_Handler
            
   .weak      USB_IRQHandler      
   .thumb_set USB_IRQHandler,Default_Handler
                     
   .weak      DMA2_Channel6_IRQHandler               
   .thumb_set DMA2_Channel6_IRQHandler,Default_Handler

   .weak      DMA2_Channel7_IRQHandler               
   .thumb_set DMA2_Channel7_IRQHandler,Default_Handler
                  
   .weak      DMA2_Channel8_IRQHandler               
   .thumb_set DMA2_Channel8_IRQHandler,Default_Handler
                  
   .weak      I2C3_IRQHandler      
   .thumb_set I2C3_IRQHandler,Default_Handler
                        
   .weak      I2C4_IRQHandler   
   .thumb_set I2C4_IRQHandler,Default_Handler
                        
   .weak      FPU_IRQHandler                  
   .thumb_set FPU_IRQHandler,Default_Handler  

   .weak      SPI4_IRQHandler                  
   .thumb_set SPI4_IRQHandler,Default_Handler

   .weak      TOUCHPAD_IRQHandler      
   .thumb_set TOUCHPAD_IRQHandler,Default_Handler
                        
   .weak      QSPI_IRQHandler   
   .thumb_set QSPI_IRQHandler,Default_Handler
                        
   .weak      LTDC_IRQHandler                  
   .thumb_set LTDC_IRQHandler,Default_Handler  

   .weak      I2S1_IRQHandler                  
   .thumb_set I2S1_IRQHandler,Default_Handler

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

