# PASTE LINK TO TEAM VIDEO BELOW
#
#

  .syntax unified
  .cpu cortex-m4
  .fpu softvfp
  .thumb
  
  .global Main
  .global  SysTick_Handler
  .global EXTI0_IRQHandler

  @ Definitions are in definitions.s to keep this file "clean"
  .include "./src/definitions.s"


  .section .text

delay:
push {R0-R12, LR}
ldr r4, =ignore_delay
ldr r5, =1
str r5, [r4]
  MOV R6, #0x200000
.LdelayLoop:
  SUBS R6,R6,#1
  BNE .LdelayLoop

ldr r4, =ignore_delay
ldr r5, =0
str r5, [r4]
pop {R0-R12, PC}

Main:
   PUSH  {R4-R5,LR}


  @
  @ Prepare GPIO Port E Pin 9 for output (LED LD3)
  @ We'll blink LED LD3 (the orange LED)
  

  @ have 1 LED turn on, flash LED around clock in clockwise pattern until reach non flashing LED, 
  @ push button on LED if correct flash green and move onto next level, if not flash red and repeat level 
 

  @ Enable GPIO port E by enabling its clock       
  LDR     R4, =RCC_AHBENR
  LDR     R5, [R4]
  ORR     R5, R5, #(0b1 << (RCC_AHBENR_GPIOEEN_BIT))
  STR     R5, [R4]

  @ Initialise the first countdown
  LDR     R4, =blink_countdown
  LDR     R5, =game_speed
  STR     R5, [R4]  

  @ Configure SysTick Timer to generate an interrupt every 1ms
  LDR     R4, =SCB_ICSR               @ Clear any pre-existing interrupts
  LDR     R5, =SCB_ICSR_PENDSTCLR     @
  STR     R5, [R4]                    @
  LDR     R4, =SYSTICK_CSR            @ Stop SysTick timer
  LDR     R5, =0                      @   by writing 0 to CSR
  STR     R5, [R4]                    @   CSR is the Control and Status Register
  LDR     R4, =SYSTICK_LOAD           @ Set SysTick LOAD for 1ms delay
  LDR     R5, =7999                   @ Assuming 8MHz clock
  STR     R5, [R4]                    @ 
  LDR     R4, =SYSTICK_VAL            @   Reset SysTick internal counter to 0
  LDR     R5, =0x1                    @     by writing any value
  STR     R5, [R4]
  LDR     R4, =SYSTICK_CSR            @   Start SysTick timer by setting CSR to 0x7
  LDR     R5, =0x7                    @     set CLKSOURCE (bit 2) to system clock (1)
  STR     R5, [R4]                    @     set TICKINT (bit 1) to 1 to enable interrupts
                                      @     set ENABLE (bit 0) to 1


  @ Configure all other LD for output 

  LDR     R4, =GPIOE_MODER
  LDR     R5, [R4]                    @ Read ...
  BIC     R5, #(0b11<<(LD3_PIN*2))    @ Modify ...
  ORR     R5, #(0b01<<(LD3_PIN*2))    @ write 01 to bits 
  STR     R5, [R4]                    @ Write 


  //my code     // LD5
  LDR     R4, =GPIOE_MODER
  LDR     R5, [R4]                    @ Read ...
  BIC     R5, #(0b11<<(LD5_PIN*2))    @ Modify ...
  ORR     R5, #(0b01<<(LD5_PIN*2))    @ write 01 to bits 
  STR     R5, [R4]                    @ Write 

  
                // LD 7
  LDR     R4, =GPIOE_MODER
  LDR     R5, [R4]                    @ Read ...
  BIC     R5, #(0b11<<(LD7_PIN*2))    @ Modify ...
  ORR     R5, #(0b01<<(LD7_PIN*2))    @ write 01 to bits 
  STR     R5, [R4]                    @ Write
  
                // LD9
  LDR     R4, =GPIOE_MODER
  LDR     R5, [R4]                    @ Read ...
  BIC     R5, #(0b11<<(LD9_PIN*2))    @ Modify ...
  ORR     R5, #(0b01<<(LD9_PIN*2))    @ write 01 to bits 
  STR     R5, [R4]                    @ Write 
                // LD10
  LDR     R4, =GPIOE_MODER
  LDR     R5, [R4]                    @ Read ...
  BIC     R5, #(0b11<<(LD10_PIN*2))   @ Modify ...
  ORR     R5, #(0b01<<(LD10_PIN*2))   @ write 01 to bits 
  STR     R5, [R4]                    @ Write 
                // LD8
  LDR     R4, =GPIOE_MODER
  LDR     R5, [R4]                    @ Read ...
  BIC     R5, #(0b11<<(LD8_PIN*2))    @ Modify ...
  ORR     R5, #(0b01<<(LD8_PIN*2))    @ write 01 to bits 
  STR     R5, [R4]                    @ Write 
  
                // LD6
  LDR     R4, =GPIOE_MODER
  LDR     R5, [R4]                    @ Read ...
  BIC     R5, #(0b11<<(LD6_PIN*2))    @ Modify ...
  ORR     R5, #(0b01<<(LD6_PIN*2))    @ write 01 to bits 
  STR     R5, [R4]                    @ Write 
    
                // LD4
  LDR     R4, =GPIOE_MODER
  LDR     R5, [R4]                    @ Read ...
  BIC     R5, #(0b11<<(LD4_PIN*2))    @ Modify ...
  ORR     R5, #(0b01<<(LD4_PIN*2))    @ write 01 to bits 
  STR     R5, [R4]                    @ Write 
 


  @
  @ Prepare external interrupt Line 0 (USER pushbutton)
  @ We'll count the number of times the button is pressed
  @

  @ Initialise count to zero
  LDR   R4, =button_count             @ count = 0;
  MOV   R5, #0                        @
  STR   R5, [R4]                      @

  LDR   R4, =game_speed_countdown
  MOV   R5, #500
  STR   R5, [R4]

  LDR   R4, =current_LED_rotate
  MOV   R5, #1
  STR   R5, [R4]

  LDR   R4, =current_highlighted_LED
  MOV   R5, #6
  STR   R5, [R4]

  LDR   R4, =currentLED
  MOV   R5, #1
  STR   R5, [R4]

  LDR   R4, =rand
  LDR   R5, =3137465536
  STR   R5, [R4]

  ldr r4, =game_speed
  ldr r5, =1000
  str r5, [r4]

  ldr r4, =ignore_delay
  ldr r5, =0
  str r5, [r4]


  @ Configure USER pushbutton (GPIO Port A Pin 0 on STM32F3 Discovery
  @   kit) to use the EXTI0 external interrupt signal
  @ Determined by bits 3..0 of the External Interrrupt Control
  @   Register (EXTIICR)
  LDR     R4, =SYSCFG_EXTIICR1
  LDR     R5, [R4]
  BIC     R5, R5, #0b1111
  STR     R5, [R4]

  @ Enable (unmask) interrupts on external interrupt Line0
  LDR     R4, =EXTI_IMR
  LDR     R5, [R4]
  ORR     R5, R5, #1
  STR     R5, [R4]

  @ Set falling edge detection on Line0
  LDR     R4, =EXTI_FTSR
  LDR     R5, [R4]
  ORR     R5, R5, #1
  STR     R5, [R4]

  @ Enable NVIC interrupt #6 (external interrupt Line0)
  LDR     R4, =NVIC_ISER
  MOV     R5, #(1<<6)
  STR     R5, [R4]



@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@     MAIN      @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@


  BL .LtoggleLED6
  BL delay
  BL LightsOff

Idle_Loop:
  B     Idle_Loop
  
End_Main:
  POP   {R4-R5,PC}

@
@ SysTick interrupt handler (blink currentLED)
@
  .type  SysTick_Handler, %function
SysTick_Handler:

  PUSH  {R0-r12, LR}

  LDR   R4, =ignore_delay
  ldr   r5, [r4]
  CMP   R5, #1
  BEQ   .LendIfDelay

  LDR   R4, =game_speed_countdown        @ if (countdown != 0) {
   LDR   R5, [R4]                    @
  CMP   R5, #0                      @
  BEQ   .LelseFire                  @


  SUB   R5, R5, #1                  @   countdown = countdown - 1;
  STR   R5, [R4]                    @

  B     .LendIfDelay                @ }

.LelseFire:                         @ else {

  LDR   R6, =currentLED

  LDR   R4, =current_LED_rotate
  LDR   R5, [R4]
  CMP   R5, #1
  BNE   .Lcheck2
  LDR   R7, =1
  STR   R7, [R6]
  BL .LtoggleLED
  B      .LContinueGame
  
.Lcheck2:
  CMP   R5, #2
  BNE   .Lcheck3

  LDR   R7, =2
  STR   R7, [R6]

  BL .LtoggleLED2
  B      .LContinueGame
.Lcheck3:
  CMP   R5, #3
  BNE   .Lcheck4

  LDR   R7, =3
  STR   R7, [R6]

  BL .LtoggleLED3
  B      .LContinueGame
.Lcheck4:
  CMP   R5, #4
  BNE   .Lcheck5

  LDR   R7, =4
  STR   R7, [R6]

  BL .LtoggleLED4
  B      .LContinueGame
.Lcheck5:
  CMP   R5, #5
  BNE   .Lcheck6

  LDR   R7, =5
  STR   R7, [R6]

  BL .LtoggleLED5
  B      .LContinueGame
.Lcheck6:
  CMP   R5, #6
  BNE   .Lcheck7

  LDR   R7, =6
  STR   R7, [R6]

  BL .LtoggleLED6
  B      .LContinueGame
.Lcheck7:
  CMP   R5, #7
  BNE   .Lcheck8

  LDR   R7, =7
  STR   R7, [R6]

  BL .LtoggleLED7
  B      .LContinueGame
.Lcheck8:
  CMP   R5, #8
  BNE   .Lcheck9

  LDR   R7, =8
  STR   R7, [R6]

  BL .LtoggleLED8
  B      .LContinueGame
.Lcheck9:
  CMP   R5, #9
  BNE   .LuserMissedLEDs
  BL .LtoggleLED10
  B      .LContinueGame
.LuserMissedLEDs:

// if this code is running the user should have lost, maybe put the code for the red leds here
  BL    CloseGame
  @ Green LEDs
.LGreenLED:                         @ if won flash green
  @ LDR     R4, =GPIOE_ODR            @   Invert LD7
  @ LDR     R5, [R4]                  @
  @ EOR     R5, #(0b1<<(LD7_PIN))     @   GPIOE_ODR = GPIOE_ODR ^ (1<<LD7_PIN);
  @ STR     R5, [R4]                  @ 

  @ LDR     R4, =GPIOE_ODR            @   Invert LD6
  @ LDR     R5, [R4]                  @
  @ EOR     R5, #(0b1<<(LD6_PIN))     @   GPIOE_ODR = GPIOE_ODR ^ (1<<LD6_PIN);
  @ STR     R5, [R4]                  @ 
  @ MOV R1, #0
  @ B .Lskip


@ .equ currentPin, LD4_PIN
@ BL  enableLED

.LContinueGame:


  LDR   R4, =current_LED_rotate       // increment the current led by 1
  LDR   R5, [R4]    
  ADD   R5, R5, #1                  
  STR   R5, [R4]     
 
 
 .Lskip:

  LDR     R4, =game_speed_countdown      @   countdown = BLINK_PERIOD;
  LDR     R5, =game_speed              @
  LDr     R5, [R5]
  STR     R5, [R4]                  @

.LendIfDelay:                       @ }

  LDR     R4, =SCB_ICSR             @ Clear (acknowledge) the interrupt
  LDR     R5, =SCB_ICSR_PENDSTCLR   @
  STR     R5, [R4]                  @

  @ Return from interrupt handler
  POP  { R0-r12, PC}

/* Subroutines & Interrupts */
.LtoggleLED:
PUSH    {R4-R6, LR}

  LDR     R4, =GPIOE_ODR          @ Load address of GPIOE_ODR into R4
  LDR     R5, [R4]                @ Load value of GPIOE_ODR into R5

  MOV R6, #1
  LSL     R6, R6, LD3_PIN
  ORR     R5, R6
  STR     R5, [R4]



  @BL delay
    POP     {R4-R6, PC}

.LtoggleLED2:
PUSH    {R4-R6, LR}
  LDR     R4, =GPIOE_ODR          @ Load address of GPIOE_ODR into R4
  LDR     R5, [R4]                @ Load value of GPIOE_ODR into R5

  MOV R6, #1
  LSL     R6, R6, LD5_PIN
  ORR     R5, R6
  STR     R5, [R4]


  @BL delay
    POP     {R4-R6, PC}

  .LtoggleLED3:
PUSH    {R4-R6, LR}
  LDR     R4, =GPIOE_ODR          @ Load address of GPIOE_ODR into R4
  LDR     R5, [R4]                @ Load value of GPIOE_ODR into R5

  MOV R6, #1
  LSL     R6, R6, LD7_PIN
  ORR     R5, R6
  STR     R5, [R4]

  @BL delay
    POP     {R4-R6, PC}

  .LtoggleLED4:
PUSH    {R4-R6, LR}
  LDR     R4, =GPIOE_ODR          @ Load address of GPIOE_ODR into R4
  LDR     R5, [R4]                @ Load value of GPIOE_ODR into R5

  MOV R6, #1
  LSL     R6, R6, LD9_PIN
  ORR     R5, R6
  STR     R5, [R4]

  @BL delay
  POP     {R4-R6, PC}

  .LtoggleLED5:
PUSH    {R4-R6, LR}
  LDR     R4, =GPIOE_ODR          @ Load address of GPIOE_ODR into R4
  LDR     R5, [R4]                @ Load value of GPIOE_ODR into R5

  MOV R6, #1
  LSL     R6, R6, LD10_PIN
  ORR     R5, R6
  STR     R5, [R4]

  @BL delay
    POP     {R4-R6, PC}

  .LtoggleLED6:
PUSH    {R4-R6, LR}
  LDR     R4, =GPIOE_ODR          @ Load address of GPIOE_ODR into R4
  LDR     R5, [R4]                @ Load value of GPIOE_ODR into R5

  MOV R6, #1
  LSL     R6, R6, LD8_PIN
  ORR     R5, R6
  STR     R5, [R4]

  @BL delay
    POP     {R4-R6, PC}

  .LtoggleLED7:
PUSH    {R4-R6, LR}
  LDR     R4, =GPIOE_ODR          @ Load address of GPIOE_ODR into R4
  LDR     R5, [R4]                @ Load value of GPIOE_ODR into R5

  MOV R6, #1
  LSL     R6, R6, LD6_PIN
  ORR     R5, R6
  STR     R5, [R4]
  
  @BL delay
    POP     {R4-R6, PC}

  .LtoggleLED8:
  PUSH    {R4-R6, LR}
  LDR     R4, =GPIOE_ODR          @ Load address of GPIOE_ODR into R4
  LDR     R5, [R4]                @ Load value of GPIOE_ODR into R5

  MOV R6, #1
  LSL     R6, R6, LD4_PIN
  ORR     R5, R6
  STR     R5, [R4]
  POP {r4-R6,pc}


.LtoggleLED10:
  LDR     R4, =GPIOE_ODR            @   Invert LD3
  LDR     R5, [R4]                  @
  EOR     R5, #(0b1<<(LD5_PIN))     @   GPIOE_ODR = GPIOE_ODR ^ (1<<LD5_PIN);
  STR     R5, [R4]                  @
  
  LDR     R4, =GPIOE_ODR            @   Invert LD10
  LDR     R5, [R4]                  @
  EOR     R5, #(0b1<<(LD7_PIN))     @   GPIOE_ODR = GPIOE_ODR ^ (1<<LD7_PIN);
  STR     R5, [R4]                  @ 

  LDR     R4, =GPIOE_ODR            @   Invert LD3
  LDR     R5, [R4]                  @
  EOR     R5, #(0b1<<(LD9_PIN))     @   GPIOE_ODR = GPIOE_ODR ^ (1<<LD9_PIN);
  STR     R5, [R4]                  @
  
  LDR     R4, =GPIOE_ODR            @   Invert LD3
  LDR     R5, [R4]                  @
  EOR     R5, #(0b1<<(LD8_PIN))     @   GPIOE_ODR = GPIOE_ODR ^ (1<<LD8_PIN);
  STR     R5, [R4]                  @
  
  LDR     R4, =GPIOE_ODR            @   Invert LD10
  LDR     R5, [R4]                  @
  EOR     R5, #(0b1<<(LD6_PIN))     @   GPIOE_ODR = GPIOE_ODR ^ (1<<LD6_PIN);
  STR     R5, [R4]                  @ 
  
  LDR     R4, =GPIOE_ODR            @   Invert LD10
  LDR     R5, [R4]                  @
  EOR     R5, #(0b1<<(LD4_PIN))     @   GPIOE_ODR = GPIOE_ODR ^ (1<<LD4_PIN);
  STR     R5, [R4]                  @ 
  


  POP     {R4-R6, PC}






@
@ External interrupt line 0 interrupt handler
@   (count button presses)
@
  .type  EXTI0_IRQHandler, %function
EXTI0_IRQHandler:

  PUSH  {R0-R12,LR}

  @ LDR   R4, =button_count           @ count = count + 1
  @ LDR   R5, [R4]                    @
  @ ADD   R5, R5, #1                  @
  @ STR   R5, [R4]                    @

  LDR   R4, =currentLED
  LDR   R5, [R4]

  LDR   R6, =current_highlighted_LED
  LDR   R7, [R6]

  CMP   R7, R5
  BNE   CloseGame
    // code to progress next round here
  BL    LightsOff

  LDR     R4, =GPIOE_ODR            @   Invert LD3
  LDR     R5, [R4]                  @
  ORR     R5, #(0b1<<(LD7_PIN))     @   GPIOE_ODR = GPIOE_ODR ^ (1<<LD3_PIN);
  STR     R5, [R4]                  @
  
  LDR     R4, =GPIOE_ODR            @   Invert LD10
  LDR     R5, [R4]                  @
  ORR     R5, #(0b1<<(LD6_PIN))     @   GPIOE_ODR = GPIOE_ODR ^ (1<<LD10_PIN);
  STR     R5, [R4]                  @


  LDR   R4, =EXTI_PR                @ Clear (acknowledge) the interrupt
  MOV   R5, #(1<<0)                 @
  STR   R5, [R4]                    @

  bl delay
BL LightsOff
bl delay

  // Reset current led to 1
LDR   R4, =currentLED
LDR     r5, =1
str  r5, [r4]

ldr r4, =current_LED_rotate
ldr r5, =1
str r5, [r4]
  // change the highlighted led
LDR R4, =rand
LDR R5, [R4]
ADD R5, R5, #13
EOR R5, R5, R5,ROR #9
STR R5, [R4]

LDR r6, =7
AND R5, R6
ADD R5, #1

CMP R5, #2
BHS .lowerBoundGood
MOV R5, #2
.lowerBoundGood:
CMP R5, #7 
BLS .happy
MOV R5, #7

.happy:


ldr r6, =current_highlighted_LED
str r5, [r6]



CMP R5, #1
BNE .Lch1
BL .LtoggleLED
B   .LfH

.Lch1:
CMP R5, #2
BNE .Lch2
BL .LtoggleLED2
B   .LfH

.Lch2:
CMP R5, #3
BNE .Lch3
BL .LtoggleLED3
B   .LfH

.Lch3:
CMP R5, #4
BNE .Lch4
BL .LtoggleLED4
B   .LfH

.Lch4:
CMP R5, #5
BNE .Lch5
BL .LtoggleLED5
B   .LfH

.Lch5:
CMP R5, #6
BNE .Lch6
BL .LtoggleLED6
B   .LfH

.Lch6:
CMP R5, #7
BNE .Lch7
BL .LtoggleLED7
B   .LfH

.Lch7:
CMP R5, #8
BNE .LfH
BL .LtoggleLED8

.LfH:

BL delay
BL LightsOff

  // increase game speed
  ldr r4, =game_speed
  ldr r5, [r4]
  sub r5, #50
  str r5, [r4]

  ldr r4, =game_speed_countdown
  str r5, [r4]
  // make sure the game runs again


  @ Return from interrupt handler
  POP  {R0-R12,PC}

LightsOff:
  PUSH {R4-r12,LR}
  LDR     R4, =GPIOE_ODR            @   Invert LD3
  LDR     R5, [R4]                  @
  LDR     R6, =0XFFFFFFFE
  LDR     R7, =LD3_PIN
  LDR     R8, =32
  SUB     R7, R8, R7
  ROR     R6, R7
  AND     R5, R6
  STR     R5, [R4]                  @
   LDR     R4, =GPIOE_ODR            @   Invert LD3
  LDR     R5, [R4]                  @
  LDR     R6, =0XFFFFFFFE
  LDR     R7, =LD5_PIN
  LDR     R8, =32
  SUB     R7, R8, R7
  ROR     R6, R7
  AND     R5, R6
  STR     R5, [R4]                  @
   LDR     R4, =GPIOE_ODR            @   Invert LD3
  LDR     R5, [R4]                  @
  LDR     R6, =0XFFFFFFFE
  LDR     R7, =LD7_PIN
  LDR     R8, =32
  SUB     R7, R8, R7
  ROR     R6, R7
  AND     R5, R6
  STR     R5, [R4]                  @
   LDR     R4, =GPIOE_ODR            @   Invert LD3
  LDR     R5, [R4]                  @
  LDR     R6, =0XFFFFFFFE
  LDR     R7, =LD9_PIN
  LDR     R8, =32
  SUB     R7, R8, R7
  ROR     R6, R7
  AND     R5, R6
  STR     R5, [R4]                  @
   LDR     R4, =GPIOE_ODR            @   Invert LD3
  LDR     R5, [R4]                  @
  LDR     R6, =0XFFFFFFFE
  LDR     R7, =LD10_PIN
  LDR     R8, =32
  SUB     R7, R8, R7
  ROR     R6, R7
  AND     R5, R6
  STR     R5, [R4]                  @
   LDR     R4, =GPIOE_ODR            @   Invert LD3
  LDR     R5, [R4]                  @
  LDR     R6, =0XFFFFFFFE
  LDR     R7, =LD8_PIN
  LDR     R8, =32
  SUB     R7, R8, R7
  ROR     R6, R7
  AND     R5, R6
  STR     R5, [R4]                  @
   LDR     R4, =GPIOE_ODR            @   Invert LD3
  LDR     R5, [R4]                  @
  LDR     R6, =0XFFFFFFFE
  LDR     R7, =LD6_PIN
  LDR     R8, =32
  SUB     R7, R8, R7
  ROR     R6, R7
  AND     R5, R6
  STR     R5, [R4]                  @
   LDR     R4, =GPIOE_ODR            @   Invert LD3
  LDR     R5, [R4]                  @
  LDR     R6, =0XFFFFFFFE
  LDR     R7, =LD4_PIN
  LDR     R8, =32
  SUB     R7, R8, R7
  ROR     R6, R7
  AND     R5, R6
  STR     R5, [R4]                  @
POP {R4-r12,PC}


CloseGame:
// code to quit here
  push {r4-r12,lr}
.LRedLED:  
 bl LightsOff
                        @ if lost flash red and repeat level 
  LDR     R4, =GPIOE_ODR            @   Invert LD3
  LDR     R5, [R4]                  @
  ORR     R5, #(0b1<<(LD10_PIN))     @   GPIOE_ODR = GPIOE_ODR ^ (1<<LD3_PIN);
  STR     R5, [R4]                  @
  
  LDR     R4, =GPIOE_ODR            @   Invert LD10
  LDR     R5, [R4]                  @
  ORR     R5, #(0b1<<(LD3_PIN))     @   GPIOE_ODR = GPIOE_ODR ^ (1<<LD10_PIN);
  STR     R5, [R4]                  @

  
  pop {r4-r12,pc}
  b End_Main
  .section .data



button_count:
  .space  4

blink_countdown:
  .space  4

game_speed_countdown:
  .space 4

current_LED_rotate:
  .space 4

current_highlighted_LED:
  .space 4

currentLED:
  .space 4

rand:
  .space 4

game_speed: 
  .space 4

ignore_delay:
  .space 4

  .end