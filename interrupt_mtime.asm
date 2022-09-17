include gd32vf103.asm


#LED_RED_PIN GPIOC_PIN_13
#LED_GRN_PIN GPIOA_PIN_1
#LED_BLU_PIN GPIOA_PIN_2
#tested working , blue led flashes based on delayms updated by mtime interrupt
#use keyword "pack <l" with ISR label in the vector table , eg-- "eclic_mtip_handler"
#reserved vectors should not be disturbed



vtable:
  j reset_handler
  align 4

  longs 0x00000000 # RESERVED
  longs 0x00000000 # RESERVED
longs   0x00000000 #pack <l longs eclic_msip_handler
  longs 0x00000000 # RESERVED
  longs 0x00000000 # RESERVED
  longs 0x00000000 # RESERVED
pack <l eclic_mtip_handler		; place the address of the mtime ISR subroutine in the vector table position 7,assembler will store isr address here
  longs 0x00000000 # RESERVED
  longs 0x00000000 # RESERVED
  longs 0x00000000 # RESERVED
  longs 0x00000000 # RESERVED
  longs 0x00000000 # RESERVED
  longs 0x00000000 # RESERVED
  longs 0x00000000 # RESERVED
  longs 0x00000000 # RESERVED
  longs 0x00000000 # RESERVED
longs   0x00000000 #pack <l longs eclic_bwei_handler
longs   0x00000000 #pack <l longs eclic_pmovi_handler
longs   0x00000000 #pack <l longs watchdog_IRQn_handler
longs   0x00000000 #pack <l LVD_IRQn_handler
longs   0x00000000 #pack <l tamper_IRQn_handler
longs   0x00000000 #pack <l RTC_IRQn_handler
longs   0x00000000 #pack <l FMC_IRQn_handler
longs   0x00000000 #pack <l RCU_IRQn_handler
longs   0x00000000 #pack <l EXTI0_IRQn_handler
longs   0x00000000 #pack <l EXTI1_IRQn_handler
longs   0x00000000 #pack <l EXTI2_IRQn_handler
longs   0x00000000 #pack <l EXTI3_IRQn_handler
longs   0x00000000 #pack <l EXTI4_IRQn_handler
longs   0x00000000 #pack <l DMA0_chan0_IRQn_handler
longs   0x00000000 #pack <l DMA0_chan1_IRQn_handler
longs   0x00000000 #pack <l DMA0_chan2_IRQn_handler
longs   0x00000000 #pack <l DMA0_chan3_IRQn_handler
longs   0x00000000 #pack <l DMA0_chan4_IRQn_handler
longs   0x00000000 #pack <l DMA0_chan5_IRQn_handler
longs   0x00000000 #pack <l DMA0_chan6_IRQn_handler
longs   0x00000000 #pack <l ADC0_1_IRQn_handler
longs   0x00000000 #pack <l CAN0_TX_IRQn_handler
longs   0x00000000 #pack <l CAN0_RX0_IRQn_handler
longs   0x00000000 #pack <l CAN0_RX1_IRQn_handler
longs   0x00000000 #pack <l CAN0_EWMC_IRQn_handler
longs   0x00000000 #pack <l EXTI5_9_IRQn_handler	# assembler stores the ISR address here for the core to jump on interrupt
longs   0x00000000 #pack <l TIM0_break_IRQn_handler
longs   0x00000000 #pack <l TIM0_update_IRQn_handler
longs   0x00000000 #pack <l TIM0_trigger_commutation_IRQn_handler
longs   0x00000000 #pack <l TIM0_channel_IRQn_handler
longs   0x00000000 #pack <l TIM1_IRQn_handler
longs   0x00000000 #pack <l TIM2_IRQn_handler
longs   0x00000000 #pack <l TIM3_IRQn_handler
longs   0x00000000 #pack <l I2C0_EV_IRQn_handler
longs   0x00000000 #pack <l I2C0_ER_IRQn_handler
longs   0x00000000 #pack <l I2C1_EV_IRQn_handler
longs   0x00000000 #pack <l I2C1_ER_IRQn_handler
longs   0x00000000 #pack <l SPI0_IRQn_handler
longs   0x00000000 #pack <l SPI1_IRQn_handler
longs   0x00000000 #pack <l USART0_IRQn_handler
longs   0x00000000 #pack <l USART1_IRQn_handler
longs   0x00000000 #pack <l USART2_IRQn_handler
longs   0x00000000 #pack <l EXTI10_15_IRQn_handler
longs   0x00000000 #pack <l RTC_alarm_IRQn_handler
longs   0x00000000 #pack <l USB_wakeup_IRQn_handler
  longs 0x00000000 # RESERVED
  longs 0x00000000 # RESERVED
  longs 0x00000000 # RESERVED
  longs 0x00000000 # RESERVED
  longs 0x00000000 # RESERVED
longs   0x00000000 #pack <l EXMC_IRQn_handler
  longs 0x00000000 # RESERVED
longs   0x00000000 #pack <l TIM4_IRQn_handler
longs   0x00000000 #pack <l SPI2_IRQn_handler
longs   0x00000000 #pack <l UART3_IRQn_handler
longs   0x00000000 #pack <l UART4_IRQn_handler
longs   0x00000000 #pack <l TIM5_IRQn_handler
longs   0x00000000 #pack <l TIM6_IRQn_handler
longs   0x00000000 #pack <l DMA1_chan0_IRQn_handler
longs   0x00000000 #pack <l DMA1_chan1_IRQn_handler
longs   0x00000000 #pack <l DMA1_chan2_IRQn_handler
longs   0x00000000 #pack <l DMA1_chan3_IRQn_handler
longs   0x00000000 #pack <l DMA1_chan4_IRQn_handler
  longs 0x00000000 # RESERVED
  longs 0x00000000 # RESERVED
longs   0x00000000 #pack <l CAN1_TX_IRQn_handler
longs   0x00000000 #pack <l CAN1_RX0_IRQn_handler
longs   0x00000000 #pack <l CAN1_RX1_IRQn_handler
longs   0x00000000 #pack <l CAN1_EWMC_IRQn_handler
longs   0x00000000 #pack <l USB_IRQn_handler





align 2
reset_handler:

    	li sp, STACK			# load the stack pointer

	# NIM (non-maskable interrupts)
  	li t0, nmi_entry		# load the address of the NMI handler procedure
  	#csrs CSR_MNVEC, t0		# encoding format available in RSA non privilage ISA manual
longs	0x7c32a073			# machine code of assembly instruction CSRS CSR_MNVEC ,t0 is directly input as bronzebeard have not implemented the instructions 
  	li t0, (1<<9) 			##The Bumblebee kernel custom mmisc_ctl register is used to control the mcause values of mnvec and NMI.0 - reset, 1 - interrupt 0 - reset, 1 - interrupt
  	#csrs CSR_MMISC_CTL, t0		# set CSR MMISC_CTL with bit mask in t0 , bit 9 will be set all others unaffected , will only affect 1 all 0 no change
longs   0x7d02a073			# machine code of csrs CSR_MMISC_CTL, t0 placed at this address as bronzebeard doesnt support CSR instructions
	
	

	li   t0, vtable			# address of vector table
longs   0x30729073			# encoded csrrW x0 ,CSR_MTVT,t0 ( vector address loaded in t0)
 	#csrw CSR_MTVT, t0		# Set the vector table's base address.
	

	li t0, irq_entry		# exception handling routine , the address of this routine should be aligned at least 4 bytes so that last 2 bits are always 0
longs   0x7ec29073			# machine code of assembly instruction csrw CSR_MTVT2, t0
  	#csrw CSR_MTVT2, t0 		# alignment at least 4 bytes
longs   0x7ec0e073			# machine code of assembly instruction csrrsi CSR_MTVT2, 1
  	#csrs CSR_MTVT2, 1



	li t0, trap_entry		# The mtvec register is used to configure the entry address for interrupts and exception handlers. last 5 bits of the address should be 0 ,align accordingly
        li t1,0xFFFFFFC0
	and t0,t0,t1			# clear LSB of MTVEC to write interrupt mode
	ori t0,t0,0x00000003            # CSR_MTVEC_ECLIC	# set 2 LSB of MTVEC , ECLIC interrupt mode , if LSB =00 default interrupt mode
longs	0x30529073			# encoded csrrW x0 ,CSR_MTVEC,t0 ( vector address loaded in t0)
  	#csrw CSR_MTVEC, t0		# Exception entry base address,registeruse vectored interrupts for now ,When mtvec.MODE != 6'b000011, the processor uses the "default interrupt mode".This mode is recommended when the processor uses "ECLIC Interrupt Mode" when mtvec.MODE = 6'b000011.


setup:
#Enable portA and portb clocks
            	
    	li s0, RCU_BASE_ADDR
    	lw a5, RCU_APB2EN_OFFSET(s0)
    	li a6, ( (1<<RCU_APB2EN_PAEN_BIT) | (1<<RCU_APB2EN_PBEN_BIT)  | (1<<RCU_APB2EN_AFEN_BIT))
	or a5, a5,a6
    	sw a5, RCU_APB2EN_OFFSET(s0)

#enable PA1 & PA2 for led ,PA1 LED GREEN , PA2 LED BLUE on LONGAN NANO
	li a0,GPIO_BASE_ADDR_A						
	li a1,(( GPIO_MODE_PP_50MHZ << 8 | GPIO_MODE_PP_50MHZ << 4  )) 		# PA1 LED GREEN , PA2 LED BLUE
	sw a1,GPIO_CTL0_OFFSET(a0)
	li a1,(1 << 1) | (1 << 2 )			# both led off ,set bits	
	sw a1,GPIO_BOP_OFFSET(a0) 			# store word in GPIO_BOP register , led stayy off 


setup_mtime:						# setting up mtime registers to run on systick
	li a0,MTIMER_BASE				# load a0 with address of mtime base address
	li a1,0x00000000          #~(0xffffffff)	# clear a1 register
	sw a1,MTIMER_MTIME_LO_OFFSET(a0)		# zero mtime low register
	sw a1,MTIMER_MTIME_HI_OFFSET(a0)		# zero mtime high register
	sw a1,MTIMER_MTIMECMP_HI_OFFSET(a0)		# zero mtime compare high register
	li a1,2000					# load count equal to 1ms .  mtime frequency/1000 = (system clock/4)/1000 , (8000000/4)/1000 = 2000
	sw a1,MTIMER_MTIMECMP_LO_OFFSET(a0)		# store 2000 in mtimr compare low register ,mtime compare is 64 bit register split into lo and high
		

#vector_mode:
	li a6,0xD200101E    				#((ECLIC_BASE_ADDR + ECLIC_CLICINATTR_OFFSET )+ (MTIP_IRQn * 4)) = 0xD200101E is where vector is enabled
	lw a3, 0(a6)					# read contents of location 0xD200101E to a3
	andi a3,a3,~(0xff)				# clear the lower byte with bit mask ~0XFF
	ori a3,a3,0x1					# OR in 0x1 to enable vector mode
	sb a3, 0(a6)					# store the byte back to location 0xD20010AA 
	
#clear eclic pending interrupt
	li a6,0xD200101C				# ((ECLIC_BASE_ADDR + ECLIC_CLICINTIP_OFFSET) + (MTIP_IRQn * 4)) = 0xD200101C
	lw a3, 0(a6)
	andi a3,a3,~(0xff)				# writing 0 will clear pending MTIP interrupt in ECLIC pending flag register
	sb a3, 0(a6)

#ECLIC_IRQ_ENABLE:					# MTIP_IRQn = 7,MTIP_IRQn * 4 = 1C, 0x1001+1C = 0x101D , base address 0xD2000000. D2000000+0x1001+1C = 0xD200101D
	li a6,0xD200101D				# li a6 ,(ECLIC_BASE_ADDR + ECLIC_CLICINTIE_OFFSET + (MTIP_IRQn * 4))   = 0xD200101D
	lw a4,0(a6)
	andi a4,a4,~(0xff)				# clear lower byte
	ori a4,a4,0x1					# write 1 to enable MTIP interrupt
	sb a4,0(a6)					# store in MTIP interrupt enable offset 0xD200101D

#enble global interrupt

longs 0x30046073					# machine code of csrrsi x0 ,CSR_MSTATUS,MSTATUS_MIE , csrrsi Mstatus,bit3, enables global interrupt



#green LED flash once to confirm excecution reached here
	call LED1ON
	call delay					# delay loop based on counting loop
	call LED1OFF

ML:							# min loop blue LED flashes continouesly till boot button is pressed for interrupt
	call LED2ON					# blue led on
	li t0,100					# number of millis loaded in t0
	call delayms					# ms delay based on mtimer interrupt count
	call LED2OFF					# blue led off
	call delayms					# ms delay based on mtimer interrupt count
	j ML						# infinite loop


eclic_mtip_handler:					# ISR for mtip interrupt , 
	li a0,MTIMER_BASE				# load a0 with mtime base address
	#lw a5,systick		
	addi a5,a5,1					# increment a5 with 1 ( every ms the count increases)
	#sw a5,systick
	li a1,~(0xffffffff)				# clear a1
	sw a1,MTIMER_MTIME_LO_OFFSET(a0)		# rezero mtime lo register
	sw a1,MTIMER_MTIME_HI_OFFSET(a0)		# rezero mtime hi register
	
longs   0x30200073					# mret ( return from interrupt)


align 64
trap_entry:
default_interrupt_handler:
    default_interrupt_loop:
    j default_interrupt_loop


align 2
nmi_entry:
#mret
longs 0x30200073  


align 2
irq_entry:
#  push t0
  	addi	sp,sp,-4
  	sw	t0,0(sp)
#  push t1
  	addi	sp,sp,-4
  	sw	t1,0(sp)
#  push a0
  	addi	sp,sp,-4
  	sw	a0,0(sp)



longs  34202573
  	#csrr a0, CSR_MCAUSE		# interogate mcause to know which exception has occured
  	li t0, 0xFFF			# mask out lower 12 bits
  	and a0, a0, t0			# use the mask to isolate the lower 12 bits to identify the cause of exception
  	li t0, USART0_IRQn		# load deired exeption number to compare with t0
  	bne t0, a0, irq_end		# if not equal exit elsedo some thing
  
  	#do something
  
irq_end:
#  pop a0
  	lw	a0,0(sp)
 	addi	sp,sp,4
#  pop t1
 	lw	t1,0(sp)
 	addi	sp,sp,4
#  pop t0
 	lw	t0,0(sp)
 	addi	sp,sp,4
#mret
longs	0x30200073          	




delay:					# delay routine
	li t1,2000000			# load an arbitarary value 20000000 to t1 register		
loop:
	addi t1,t1,-1			# subtract 1 from t1
	bne t1,zero,loop		# if t1 not equal to 0 branch to label loop
	ret	


delayms:				#  number of millis in t0
	#lw a6,systick
	blt a5,t0 delayms		# branch to delayms if t0 is less than a5
	li a5,0x00000000		# clear a5 to keep it ready for next iteration/call
	ret				# return to caller

LED1ON:
	li a0,GPIO_BASE_ADDR_A		# GPIO A base address
	li a1, 1 << 1			# value of 1 lhs 2 OR 1 lhs 2 loaded in a1
	sw a1,GPIO_BC_OFFSET(a0)
	ret
LED1OFF:
	li a0,GPIO_BASE_ADDR_A		# GPIO A base address
	li a1, 1 << 1			# value of 1 lhs 2 OR 1 lhs 2 loaded in a1
	sw a1,GPIO_BOP_OFFSET(a0) 
	ret

LED2ON:
	li a0,GPIO_BASE_ADDR_A		# GPIO A base address
	li a1, 1 << 2			# value of 1 lhs 2 OR 1 lhs 2 loaded in a1
	sw a1,GPIO_BC_OFFSET(a0)
	ret
LED2OFF:
	li a0,GPIO_BASE_ADDR_A		# GPIO A base address
	li a1, 1 << 2			# value of 1 lhs 2 OR 1 lhs 2 loaded in a1
	sw a1,GPIO_BOP_OFFSET(a0) 
	ret




