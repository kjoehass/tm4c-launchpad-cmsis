@ Startup file for TM4C123GH6PM assembly programming
@ Compatible with GNU as assembler
@ (c) K. Joseph Hass
@
@ Tell the linker that the vector table begins here (and must be loaded
@   at absolute address 0x0!)
@
  .section .isr_vector
@
@ Create all of the exception/interrupt vectors.
@ Note that service routine addresses in this table
@ should be declared as .thumb_func so the LSB will be
@ set to 1 and they will execute in thumb mode.
@
  .global __StackTop
  .global _vectors
_vectors:
  .word __StackTop                  @ Initial Stack Pointer
  .word Reset_Handler               @ Start of executable code
  .word NMI_Handler                 @ Non-maskable Interrupt Handler
  .word HardFault_Handler           @ Hard Fault Handler
  .word MemManage_Handler           @ MPU Fault Handler
  .word BusFault_Handler            @ Bus Fault Handler
  .word UsageFault_Handler          @ Usage Fault Handler
  .word 0                           @ Reserved
  .word 0                           @ Reserved
  .word 0                           @ Reserved
  .word 0                           @ Reserved
  .word SVC_Handler                 @ SVCall Handler
  .word DebugMon_Handler            @ Debug Monitor Handler
  .word 0                           @ Reserved
  .word PendSV_Handler              @ PendSV Handler
  .word SysTick_Handler             @ SysTick Handler
@ External Interrupts
  .word GPIOA_Handler               @ GPIO Port A 
  .word GPIOB_Handler               @ GPIO Port B 
  .word GPIOC_Handler               @ GPIO Port C 
  .word GPIOD_Handler               @ GPIO Port D 
  .word GPIOE_Handler               @ GPIO Port E 
  .word UART0_Handler               @ UART0 Rx and Tx 
  .word UART1_Handler               @ UART1 Rx and Tx 
  .word SSI0_Handler                @ SSI0 Rx and Tx 
  .word I2C0_Handler                @ I2C0 Master and Slave 
  .word PMW0_FAULT_Handler          @ PWM Fault 
  .word PWM0_0_Handler              @ PWM Generator 0 
  .word PWM0_1_Handler              @ PWM Generator 1 
  .word PWM0_2_Handler              @ PWM Generator 2 
  .word QEI0_Handler                @ Quadrature Encoder 0 
  .word ADC0SS0_Handler             @ ADC Sequence 0 
  .word ADC0SS1_Handler             @ ADC Sequence 1 
  .word ADC0SS2_Handler             @ ADC Sequence 2 
  .word ADC0SS3_Handler             @ ADC Sequence 3 
  .word WDT_Handler                 @ Watchdog timers 0 and 1
  .word TIMER0A_Handler             @ Timer 0 subtimer A 
  .word TIMER0B_Handler             @ Timer 0 subtimer B 
  .word TIMER1A_Handler             @ Timer 1 subtimer A 
  .word TIMER1B_Handler             @ Timer 1 subtimer B 
  .word TIMER2A_Handler             @ Timer 2 subtimer A 
  .word TIMER2B_Handler             @ Timer 2 subtimer B 
  .word COMP0_Handler               @ Analog Comparator 0 
  .word COMP1_Handler               @ Analog Comparator 1 
  .word 0                           @ Reserved on TM4C123GH6PM
  .word SYSCTL_Handler              @ System Control (PLL, OSC, BO) 
  .word FLASH_Handler               @ FLASH and EEPROM Control 
  .word GPIOF_Handler               @ GPIO Port F 
  .word 0                           @ Reserved on TM4C123GH6PM
  .word 0                           @ Reserved on TM4C123GH6PM
  .word UART2_Handler               @ UART2 Rx and Tx 
  .word SSI1_Handler                @ SSI1 Rx and Tx 
  .word TIMER3A_Handler             @ Timer 3 subtimer A 
  .word TIMER3B_Handler             @ Timer 3 subtimer B 
  .word I2C1_Handler                @ I2C1 Master and Slave 
  .word QEI1_Handler                @ Quadrature Encoder 1 
  .word CAN0_Handler                @ CAN0 
  .word CAN1_Handler                @ CAN1 
  .word 0                           @ Reserved on TM4C123GH6PM
  .word 0                           @ Reserved on TM4C123GH6PM
  .word HIB_Handler                 @ Hibernate 
  .word USB0_Handler                @ USB0 
  .word PWM0_3_Handler              @ PWM Generator 3 
  .word UDMA_Handler                @ uDMA Software Transfer 
  .word UDMAERR_Handler             @ uDMA Error 
  .word ADC1SS0_Handler             @ ADC1 Sequence 0 
  .word ADC1SS1_Handler             @ ADC1 Sequence 1 
  .word ADC1SS2_Handler             @ ADC1 Sequence 2 
  .word ADC1SS3_Handler             @ ADC1 Sequence 3 
  .word 0                           @ Reserved on TM4C123GH6PM
  .word 0                           @ Reserved on TM4C123GH6PM
  .word 0                           @ Reserved on TM4C123GH6PM
  .word 0                           @ Reserved on TM4C123GH6PM
  .word 0                           @ Reserved on TM4C123GH6PM
  .word SSI2_Handler                @ SSI2 Rx and Tx 
  .word SSI3_Handler                @ SSI3 Rx and Tx 
  .word UART3_Handler               @ UART3 Rx and Tx 
  .word UART4_Handler               @ UART4 Rx and Tx 
  .word UART5_Handler               @ UART5 Rx and Tx 
  .word UART6_Handler               @ UART6 Rx and Tx 
  .word UART7_Handler               @ UART7 Rx and Tx 
  .word 0                           @ Reserved 
  .word 0                           @ Reserved 
  .word 0                           @ Reserved 
  .word 0                           @ Reserved 
  .word I2C2_Handler                @ I2C2 Master and Slave 
  .word I2C3_Handler                @ I2C3 Master and Slave 
  .word TIMER4A_Handler             @ Timer 4 subtimer A 
  .word TIMER4B_Handler             @ Timer 4 subtimer B 
  .word 0                           @ Reserved 
  .word 0                           @ Reserved 
  .word 0                           @ Reserved 
  .word 0                           @ Reserved 
  .word 0                           @ Reserved 
  .word 0                           @ Reserved 
  .word 0                           @ Reserved 
  .word 0                           @ Reserved 
  .word 0                           @ Reserved 
  .word 0                           @ Reserved 
  .word 0                           @ Reserved 
  .word 0                           @ Reserved 
  .word 0                           @ Reserved 
  .word 0                           @ Reserved 
  .word 0                           @ Reserved 
  .word 0                           @ Reserved 
  .word 0                           @ Reserved 
  .word 0                           @ Reserved 
  .word 0                           @ Reserved 
  .word 0                           @ Reserved 
  .word TIMER5A_Handler             @ Timer 5 subtimer A 
  .word TIMER5B_Handler             @ Timer 5 subtimer B 
  .word WTIMER0A_Handler            @ Wide Timer 0 subtimer A 
  .word WTIMER0B_Handler            @ Wide Timer 0 subtimer B 
  .word WTIMER1A_Handler            @ Wide Timer 1 subtimer A 
  .word WTIMER1B_Handler            @ Wide Timer 1 subtimer B 
  .word WTIMER2A_Handler            @ Wide Timer 2 subtimer A 
  .word WTIMER2B_Handler            @ Wide Timer 2 subtimer B 
  .word WTIMER3A_Handler            @ Wide Timer 3 subtimer A 
  .word WTIMER3B_Handler            @ Wide Timer 3 subtimer B 
  .word WTIMER4A_Handler            @ Wide Timer 4 subtimer A 
  .word WTIMER4B_Handler            @ Wide Timer 4 subtimer B 
  .word WTIMER5A_Handler            @ Wide Timer 5 subtimer A 
  .word WTIMER5B_Handler            @ Wide Timer 5 subtimer B 
  .word FPU_Handler                 @ FPU ?? System Exception (imprecise)
  .word 0                           @ Reserved on TM4C123GH6PM
  .word 0                           @ Reserved on TM4C123GH6PM
  .word 0                           @ Reserved on TM4C123GH6PM
  .word 0                           @ Reserved on TM4C123GH6PM
  .word 0                           @ Reserved on TM4C123GH6PM
  .word 0                           @ Reserved on TM4C123GH6PM
  .word 0                           @ Reserved on TM4C123GH6PM
  .word 0                           @ Reserved on TM4C123GH6PM
  .word 0                           @ Reserved on TM4C123GH6PM 
  .word 0                           @ Reserved on TM4C123GH6PM
  .word 0                           @ Reserved on TM4C123GH6PM
  .word 0                           @ Reserved on TM4C123GH6PM
  .word 0                           @ Reserved on TM4C123GH6PM
  .word 0                           @ Reserved on TM4C123GH6PM
  .word 0                           @ Reserved on TM4C123GH6PM
  .word 0                           @ Reserved on TM4C123GH6PM
  .word 0                           @ Reserved on TM4C123GH6PM
  .word 0                           @ Reserved on TM4C123GH6PM
  .word 0                           @ Reserved on TM4C123GH6PM
  .word 0                           @ Reserved on TM4C123GH6PM
  .word 0                           @ Reserved on TM4C123GH6PM
  .word 0                           @ Reserved on TM4C123GH6PM
  .word 0                           @ Reserved on TM4C123GH6PM
  .word 0                           @ Reserved on TM4C123GH6PM
  .word 0                           @ Reserved on TM4C123GH6PM
  .word 0                           @ Reserved on TM4C123GH6PM
  .word 0                           @ Reserved on TM4C123GH6PM
  .word PMW1_0_Handler              @ PWM 1 Generator 0 
  .word PWM1_1_Handler              @ PWM 1 Generator 1 
  .word PWM1_2_Handler              @ PWM 1 Generator 2 
  .word PWM1_3_Handler              @ PWM 1 Generator 3 
  .word PWM1_FAULT_Handler          @ PWM 1 Fault 

@
@ Dummy interrupt handlers are added here.
@
  .syntax unified
  .section .text

  .thumb_func
  .weak NMI_Handler
NMI_Handler:
   B .

  .thumb_func
  .weak HardFault_Handler
HardFault_Handler:
   B .

  .thumb_func
  .weak MemManage_Handler
MemManage_Handler:
   B .

  .thumb_func
  .weak BusFault_Handler
BusFault_Handler:
   B .

  .thumb_func
  .weak UsageFault_Handler
UsageFault_Handler:
   B .

  .thumb_func
  .weak SVC_Handler
SVC_Handler:
   B .

  .thumb_func
  .weak DebugMon_Handler
DebugMon_Handler:
   B .

  .thumb_func
  .weak PendSV_Handler
PendSV_Handler:
   B .

  .thumb_func
  .weak SysTick_Handler
SysTick_Handler:
   B .

  .thumb_func
  .weak Default_Handler
Default_Handler:
   B .

  .global NMI_Handler,HardFault_Handler,Default_Handler
  .global MemManage_Handler,BusFault_Handler,UsageFault_Handler,SVC_Handler
  .global DebugMon_Handler,PendSV_Handler,SysTick_Handler
@
@ Map the rest of the interrupts to the Default Handler, weakly
@
  .macro MapIRQ vector
  .weak \vector
  .thumb_set \vector, Default_Handler
  .global \vector
  .endm

  MapIRQ GPIOA_Handler
  MapIRQ GPIOB_Handler
  MapIRQ GPIOC_Handler
  MapIRQ GPIOD_Handler
  MapIRQ GPIOE_Handler
  MapIRQ UART0_Handler
  MapIRQ UART1_Handler
  MapIRQ SSI0_Handler
  MapIRQ I2C0_Handler
  MapIRQ PMW0_FAULT_Handler
  MapIRQ PWM0_0_Handler
  MapIRQ PWM0_1_Handler
  MapIRQ PWM0_2_Handler
  MapIRQ QEI0_Handler
  MapIRQ ADC0SS0_Handler
  MapIRQ ADC0SS1_Handler
  MapIRQ ADC0SS2_Handler
  MapIRQ ADC0SS3_Handler
  MapIRQ WDT_Handler
  MapIRQ TIMER0A_Handler
  MapIRQ TIMER0B_Handler
  MapIRQ TIMER1A_Handler
  MapIRQ TIMER1B_Handler
  MapIRQ TIMER2A_Handler
  MapIRQ TIMER2B_Handler
  MapIRQ COMP0_Handler
  MapIRQ COMP1_Handler
  MapIRQ SYSCTL_Handler
  MapIRQ FLASH_Handler
  MapIRQ GPIOF_Handler
  MapIRQ UART2_Handler
  MapIRQ SSI1_Handler
  MapIRQ TIMER3A_Handler
  MapIRQ TIMER3B_Handler
  MapIRQ I2C1_Handler
  MapIRQ QEI1_Handler
  MapIRQ CAN0_Handler
  MapIRQ CAN1_Handler
  MapIRQ HIB_Handler
  MapIRQ USB0_Handler
  MapIRQ PWM0_3_Handler
  MapIRQ UDMA_Handler
  MapIRQ UDMAERR_Handler
  MapIRQ ADC1SS0_Handler
  MapIRQ ADC1SS1_Handler
  MapIRQ ADC1SS2_Handler
  MapIRQ ADC1SS3_Handler
  MapIRQ SSI2_Handler
  MapIRQ SSI3_Handler
  MapIRQ UART3_Handler
  MapIRQ UART4_Handler
  MapIRQ UART5_Handler
  MapIRQ UART6_Handler
  MapIRQ UART7_Handler
  MapIRQ I2C2_Handler
  MapIRQ I2C3_Handler
  MapIRQ TIMER4A_Handler
  MapIRQ TIMER4B_Handler
  MapIRQ TIMER5A_Handler
  MapIRQ TIMER5B_Handler
  MapIRQ WTIMER0A_Handler
  MapIRQ WTIMER0B_Handler
  MapIRQ WTIMER1A_Handler
  MapIRQ WTIMER1B_Handler
  MapIRQ WTIMER2A_Handler
  MapIRQ WTIMER2B_Handler
  MapIRQ WTIMER3A_Handler
  MapIRQ WTIMER3B_Handler
  MapIRQ WTIMER4A_Handler
  MapIRQ WTIMER4B_Handler
  MapIRQ WTIMER5A_Handler
  MapIRQ WTIMER5B_Handler
  MapIRQ FPU_Handler
  MapIRQ PMW1_0_Handler
  MapIRQ PWM1_1_Handler
  MapIRQ PWM1_2_Handler
  MapIRQ PWM1_3_Handler
  MapIRQ PWM1_FAULT_Handler


@  Reset Handler
@
@ Need some symbols from the linker
@
  .global __bss_start__, __bss_end__, __data_start__, __data_end__, __etext
@
@ At reset we have to do the C run-time stuff:
@   Clear the bss segment in RAM
@   Copy the data segment from Flash to RAM
@   Setup the clocks
@   Go to the user's main
@
  .syntax unified
  .section .text
  .thumb_func
  .weak Reset_Handler
  .global Reset_Handler
Reset_Handler:
@
@ Enable the Usage, Bus, and Memory Management fault exceptions in the
@ System Handler Control and State Register
@
  MOVW    R0, #0xED24
  MOVT    R0, #0xE000
  LDR     R1, [R0]
  ORR     R1, #0x70000
  STR     R1, [R0]
@
@ Clear the bss segment
@
  LDR     R0, =__bss_start__
  LDR     R1, =__bss_end__
  MOV     R2, #0
ClearMore:
  CMP     R0, R1
  ITT     LT
  STRLT   R2, [R0], #4
  BLT     ClearMore
@
@ Move the data segment from Flash to RAM
@
  LDR     R0, =__etext
  LDR     R1, =__data_end__
  LDR     R2, =__data_start__
MoveMore:
  CMP     R2, R1
  ITT     LT
  LDRLT   R3, [R0], #4
  STRLT   R3, [R2], #4
  BLT     MoveMore
@
@ Setup clocks and then go to main
@
  LDR     R0, =SystemInit
  BLX     R0
  LDR     R0, =main
  BX      R0
  B       .
  .pool

  .end
