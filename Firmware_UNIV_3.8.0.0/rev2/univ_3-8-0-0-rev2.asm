;==============================================================================
;   HAPCAN - Home Automation Project Firmware (http://hapcan.com)
;   Copyright (C) 2016 hapcan.com
;
;   This program is free software: you can redistribute it and/or modify
;   it under the terms of the GNU General Public License as published by
;   the Free Software Foundation, either version 3 of the License, or
;   (at your option) any later version.
;
;   This program is distributed in the hope that it will be useful,
;   but WITHOUT ANY WARRANTY; without even the implied warranty of
;   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
;   GNU General Public License for more details.
;
;   You should have received a copy of the GNU General Public License
;   along with this program.  If not, see <http://www.gnu.org/licenses/>.
;==============================================================================
;   Filename:              univ_3-8-0-0.asm
;   Associated diagram:    univ_3-8-0-x.sch
;   Author:                Jacek Siwilo                          
;   Note:                  RGB LED Controller 
;   !!!Notice that PWM controlling of RGB LEDs is patented by Philips!!!
;==============================================================================
;   Revision History
;   Rev:  Date:     Details:
;   0     01.2014   Original version
;   1     11.2015   Updated with "univ3-routines-rev4.inc"
;   2     12.2016   Updated with "univ3-routines-rev6.inc"
;==============================================================================
;===  FIRMWARE DEFINITIONS  =================================================== 
;==============================================================================
    #define    ATYPE    .8                            ;application type [0-255]
    #define    AVERS    .0                         ;application version [0-255]
    #define    FVERS    .0                            ;firmware version [0-255]

    #define    FREV     .2                         ;firmware revision [0-65536]
;==============================================================================
;===  NEEDED FILES  ===========================================================
;==============================================================================
    LIST P=18F26K80                              ;directive to define processor
    #include <P18F26K80.INC>           ;processor specific variable definitions
    #include "univ_3-8-0-0-rev2.inc"                         ;project variables
INCLUDEDFILES   code  
    #include "univ3-routines-rev6.inc"                     ;UNIV 3 CPU routines
;==============================================================================
;===  FIRMWARE CHECKSUM  ======================================================
;==============================================================================
FIRMCHKSM   code    0x001000
    DB      0x61, 0x1C, 0x32, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF            
;==============================================================================
;===  FIRMWARE ID  ============================================================
;==============================================================================
FIRMID      code    0x001010
    DB      0x30, 0x00, 0x03,ATYPE,AVERS,FVERS,FREV>>8,FREV
;            |     |     |     |     |     |     |_____|_____ firmware revision
;            |     |     |     |     |     |__________________ firmware version
;            |     |     |     |     |_____________________ application version
;            |     |     |     |______________________________ application type
;            |     |     |________________________________ hardware version '3'
;            |_____|______________________________________ hardware type 'UNIV'
;==============================================================================
;===  MOVED VECTORS  ==========================================================
;==============================================================================
;PROGRAM RESET VECTOR
FIRMRESET   code    0x1020
        goto    Main
;PROGRAM HIGH PRIORITY INTERRUPT VECTOR
FIRMHIGHINT code    0x1030
        call    HighInterrupt
        retfie
;PROGRAM LOW PRIORITY INTERRUPT VECTOR
FIRMLOWINT  code    0x1040
        call    LowInterrupt
        retfie

;==============================================================================
;===  FIRMWARE STARTS  ========================================================
;==============================================================================
FIRMSTART   code    0x001050
;------------------------------------------------------------------------------
;---  LOW PRIORITY INTERRUPT  -------------------------------------------------
;------------------------------------------------------------------------------
LowInterrupt
        movff   STATUS,STATUS_LOW           ;save STATUS register
        movff   WREG,WREG_LOW               ;save working register
        movff   BSR,BSR_LOW                 ;save BSR register
        movff   FSR0L,FSR0L_LOW             ;save other registers used in low int
        movff   FSR0H,FSR0H_LOW
        movff   FSR1L,FSR1L_LOW
        movff   FSR1H,FSR1H_LOW

    ;main firmware ready flag
        banksel FIRMREADY
        btfss   FIRMREADY,0
        bra     ExitLowInterrupt            ;main firmware is not ready yet
    ;CAN buffer
        banksel CANFULL
        btfsc   CANFULL,0                   ;check if CAN received anything
        call    CANInterrupt                ;proceed with CAN interrupt

ExitLowInterrupt
        movff   BSR_LOW,BSR                 ;restore BSR register
        movff   WREG_LOW,WREG               ;restore working register
        movff   STATUS_LOW,STATUS           ;restore STATUS register
        movff   FSR0L_LOW,FSR0L             ;restore other registers used in low int
        movff   FSR0H_LOW,FSR0H
        movff   FSR1L_LOW,FSR1L
        movff   FSR1H_LOW,FSR1H
    return

;------------------------------------------------------------------------------
;---  HIGH PRIORITY INTERRUPT  ------------------------------------------------
;------------------------------------------------------------------------------
HighInterrupt
        movff   STATUS,STATUS_HIGH          ;save STATUS register
        movff   WREG,WREG_HIGH              ;save working register
        movff   BSR,BSR_HIGH                ;save BSR register
        movff   FSR0L,FSR0L_HIGH            ;save other registers used in high int
        movff   FSR0H,FSR0H_HIGH
        movff   FSR1L,FSR1L_HIGH
        movff   FSR1H,FSR1H_HIGH

    ;main firmware ready flag
        banksel FIRMREADY
        btfss   FIRMREADY,0
        bra     ExitHighInterrupt           ;main firmware is not ready yet
    ;Timer0
        btfsc   INTCON,TMR0IF               ;Timer0 interrupt? (1000ms)
        rcall   Timer0Interrupt
    ;Timer2    
        btfsc   PIR1,TMR2IF                 ;Timer2 interrupt? (20ms)
        rcall   Timer2Interrupt

ExitHighInterrupt
        movff   BSR_HIGH,BSR                ;restore BSR register
        movff   WREG_HIGH,WREG              ;restore working register
        movff   STATUS_HIGH,STATUS          ;restore STATUS register
        movff   FSR0L_HIGH,FSR0L            ;restore other registers used in high int
        movff   FSR0H_HIGH,FSR0H
        movff   FSR1L_HIGH,FSR1L
        movff   FSR1H_HIGH,FSR1H
    return

;------------------------------------------------------------------------------
; Routine:          CAN INTERRUPT
;------------------------------------------------------------------------------
; Overview:         Checks CAN message for response and RTR and saves to FIFO
;------------------------------------------------------------------------------
CANInterrupt
        banksel CANFRAME2
        btfsc   CANFRAME2,0                 ;response message?
    return                                  ;yes, so ignore it and exit
        btfsc   CANFRAME2,1                 ;RTR (Remote Transmit Request)?
    return                                  ;yes, so ignore it and exit
        call    Copy_RXB_RXFIFOIN           ;copies received message to CAN RX FIFO input buffer
        call    WriteToCanRxFIFO            ;saves message to FIFO
    return

;------------------------------------------------------------------------------
; Routine:          TIMER 0 INTERRUPT
;------------------------------------------------------------------------------
; Overview:         1000ms periodical interrupt
;------------------------------------------------------------------------------
Timer0Interrupt:
        call    Timer0Initialization8MHz    ;restart 1000ms Timer   
        call    UpdateUpTime                ;counts time from restart
        call    UpdateTransmitTimer         ;increment transmit timer (seconds after last transmission)
        banksel TIMER0_1000ms
        setf    TIMER0_1000ms               ;timer 0 interrupt occurred flag
    return

;------------------------------------------------------------------------------
; Routine:          TIMER 2 INTERRUPT
;------------------------------------------------------------------------------
; Overview:         20ms periodical interrupt
;------------------------------------------------------------------------------
Timer2Interrupt
        rcall   Timer2Initialization8MHz    ;restart timer
        banksel TIMER2_20ms
        setf    TIMER2_20ms                 ;timer 2 interrupt occurred flag
    return
;-------------------------------
Timer2Initialization8MHz
        movlb   0xF
        bcf     PMD1,TMR2MD                 ;enable timer 2
        movlw   0x3F          
        movwf   TMR2                        ;set 20ms (19.999500)
        movlw   b'01001111'                 ;start timer, prescaler=16, postscaler=10
        movwf   T2CON
        bsf     IPR1,TMR2IP                 ;high priority for interrupt
        bcf     PIR1,TMR2IF                 ;clear timer's flag
        bsf     PIE1,TMR2IE                 ;interrupt on
    return

;==============================================================================
;===  MAIN PROGRAM  ===========================================================
;==============================================================================
Main:
    ;disable global interrupts for startup
        call    DisAllInt                   ;disable all interrupt
    ;firmware initialization
        rcall   PortInitialization          ;prepare processor ports
        call    GeneralInitialization       ;read eeprom config, clear other registers
        call    FIFOInitialization          ;prepare FIFO buffers
        call    Timer0Initialization8MHz    ;Timer 0 initialization for 1s periodical interrupt 
        call    Timer2Initialization8MHz    ;Timer 2 initialization for 20ms periodical interrupt
        call    RGB_PWMInitialization       ;sets PWM registers
        call    RGB_PowerUpStates           ;sets power up states
    ;firmware ready
        banksel FIRMREADY
        bsf     FIRMREADY,0                 ;set flag "firmware started and ready for interrupts"
    ;enable global interrupts
        call    EnAllInt                    ;enable all interrupts

;-------------------------------
Loop:                                       ;main loop
        clrwdt                              ;clear Watchdog timer
        call    ReceiveProcedure            ;check if any msg in RX FIFO and if so - process the msg
        call    TransmitProcedure           ;check if any msg in TX FIFO and if so - transmit it
        call    RGB_Programs                ;manage RGB programs
        rcall   OnceA20ms                   ;do routines only after 20ms interrupt 
        rcall   OnceA1000ms                 ;do routines only after 1000ms interrupt
    bra     Loop

;-------------------------------
OnceA20ms                                   ;procedures executed once per 1000ms (flag set in interrupt)
        banksel TIMER2_20ms
        tstfsz  TIMER2_20ms                 ;flag set?
        bra     $ + 4
    return                                  ;no, so exit
        call    RGB_Control_20ms            ;update states of all channels
        banksel TIMER2_20ms
        clrf    TIMER2_20ms
    return
;-------------------------------
OnceA1000ms                                 ;procedures executed once per 1000ms (flag set in interrupt)
        banksel TIMER0_1000ms
        tstfsz  TIMER0_1000ms               ;flag set?
        bra     $ + 4
    return                                  ;no, so exit
        call    UpdateDelayTimers           ;updates channel timers 
        call    UpdateHealthRegs            ;saves health maximums to eeprom
        call    RGB_SaveSateToEeprom        ;save LED states into eeprom memory if needed
        banksel TIMER0_1000ms
        clrf    TIMER0_1000ms
    return


;==============================================================================
;===  FIRMWARE ROUTINES  ======================================================
;==============================================================================
;------------------------------------------------------------------------------
; Routine:          PORT INITIALIZATION
;------------------------------------------------------------------------------
; Overview:         It sets processor pins. All unused pins should be set as
;                   outputs and driven low
;------------------------------------------------------------------------------
PortInitialization                          ;default all pins set as analog (portA,B) or digital (portB,C) inputs 
    ;PORT A
        banksel ANCON0                      ;select memory bank
        ;0-digital, 1-analog input
        movlw   b'00000011'                 ;(x,x,x,AN4,AN3,AN2,AN1-boot_mode,AN0-volt)
        movwf   ANCON0
        ;output level
        clrf    LATA                        ;all low
        ;0-output, 1-input
        movlw   b'00000011'                 ;all outputs except, bit<1>-boot_mode, bit<0>-volt
        movwf   TRISA        
    ;PORT B
        ;0-digital, 1-analog input
        movlw   b'00000000'                 ;(x,x,x,x,x,AN10,AN9,AN8)
        movwf   ANCON1
        ;output level
        clrf    LATB                        ;all low
        ;0-output, 1-input
        movlw   b'00001000'                 ;all output except CANRX
        movwf   TRISB
    ;PORT C
        ;output level
        clrf    LATC                        ;all low
        ;0-output, 1-input
        movlw   b'00000000'                 ;all output 
        movwf   TRISC
    return

;------------------------------------------------------------------------------
; Routine:          NODE STATUS
;------------------------------------------------------------------------------
; Overview:         It prepares status messages when status request was
;                   received
;------------------------------------------------------------------------------
NodeStatusRequest
        banksel TXFIFOIN0
        ;------------------
        movlw   0x01                        ;this is Red
        movwf   TXFIFOIN6
        movff   PHASE1_WORK,TXFIFOIN7       ;send PHASE_WORK status
        setf    TXFIFOIN8                   ;unused
        movff   Instr1Ch1,TXFIFOIN9         ;info what instruction is waiting for execution
        movff   Instr2Ch1,TXFIFOIN10
        movff   TimerCh1,TXFIFOIN11         ;value of channel timer
        rcall   SendStatus
        ;------------------
        movlw   0x02                        ;this is Green
        movwf   TXFIFOIN6
        movff   PHASE2_WORK,TXFIFOIN7
        setf    TXFIFOIN8
        movff   Instr1Ch2,TXFIFOIN9           
        movff   Instr2Ch2,TXFIFOIN10
        movff   TimerCh2,TXFIFOIN11       
        rcall   SendStatus
        ;------------------
        movlw   0x03                        ;this is Blue
        movwf   TXFIFOIN6
        movff   PHASE3_WORK,TXFIFOIN7   
        setf    TXFIFOIN8   
        movff   Instr1Ch3,TXFIFOIN9           
        movff   Instr2Ch3,TXFIFOIN10
        movff   TimerCh3,TXFIFOIN11         
        rcall   SendStatus
        ;------------------
        movlw   0x04                        ;this is Master
        movwf   TXFIFOIN6
        movff   PHASE4_WORK,TXFIFOIN7      
        movff   RELAY_STATE,TXFIFOIN8 
        movff   Instr1Ch4,TXFIFOIN9           
        movff   Instr2Ch4,TXFIFOIN10
        movff   TimerCh4,TXFIFOIN11        
        rcall   SendStatus
    return
;-------------------------------
SendStatus
        movlw   0x30                        ;LED controller frame
        movwf   TXFIFOIN0
        movlw   0x80
        movwf   TXFIFOIN1
        bsf     TXFIFOIN1,0                 ;response bit
        movff   NODENR,TXFIFOIN2            ;node id
        movff   GROUPNR,TXFIFOIN3
        setf    TXFIFOIN4                   ;unused
        setf    TXFIFOIN5                   ;unused
        call    WriteToCanTxFIFO
    return

;------------------------------------------------------------------------------
; Routine:          DO INSTRUCTION
;------------------------------------------------------------------------------
; Overview:         Executes instruction immediately or sets timer for later
;                   execution
;------------------------------------------------------------------------------
DoInstructionRequest
        ;allow only known values
        banksel INSTR1
        movlw   0x26                        ;INSTR less than?
        cpfslt  INSTR1
    bra     ExitDoInstructionRequest        ;no, so exit
        ;Recognize instruction
        movf    INSTR1,W                    
        call    ComputedGoto                ;modify PC according to WREG value
        bra     Instr00                     ;instruction 00
        bra     Instr01                     ;instruction 01
        bra     Instr02                     ;instruction 02
        bra     Instr03                     ;instruction 03
        bra     Instr04                     ;instruction 04
        bra     Instr05                     ;instruction 05
        bra     Instr06                     ;instruction 06
        bra     Instr07                     ;instruction 07
        bra     Instr08                     ;instruction 08
        bra     Instr09                     ;instruction 09
        bra     Instr0A                     ;instruction 0A
        bra     Instr0B                     ;instruction 0B
        bra     Instr0C                     ;instruction 0C
        bra     Instr0D                     ;instruction 0D
        bra     Instr0E                     ;instruction 0E
        bra     Instr0F                     ;instruction 0F
        bra     Instr10                     ;instruction 10
        bra     Instr11                     ;instruction 11
        bra     Instr12                     ;instruction 12
        bra     Instr13                     ;instruction 13
        bra     Instr14                     ;instruction 14
        bra     Instr15                     ;instruction 15
        bra     Instr16                     ;instruction 16
        bra     Instr17                     ;instruction 17
        bra     Instr18                     ;instruction 18
        bra     Instr19                     ;instruction 19
        bra     Instr1A                     ;instruction 1A
        bra     Instr1B                     ;instruction 1B
        bra     Instr1C                     ;instruction 1C
        bra     Instr1D                     ;instruction 1D
        bra     Instr1E                     ;instruction 1E
        bra     Instr1F                     ;instruction 1F    
        bra     Instr20                     ;instruction 20    
        bra     Instr21                     ;instruction 21
        bra     Instr22                     ;instruction 22
        bra     Instr23                     ;instruction 23
        bra     Instr24                     ;instruction 24
        bra     Instr25                     ;instruction 25    

;-------------------------------
;Instruction execution
Instr00                                     ;SET RED to INSTR2
        tstfsz  INSTR3                      ;is timer = 0?
        bra     $ + .4                      ;no, so do instruction later
        bra     $ + .12                     ;yes, so do instruction now
        movff   INSTR3,TIMER                ;move timer from INSTR3
        call    DoInstLater_SetCh1          ;save instruction for later execution
        bra     ExitDoInstructionRequest 
        movff   INSTR2,PHASE1               ;move desired value
        bcf     PHASE_FLAG,Ch1ChangeSlow    ;change immediately flag
        call    DoInstLater_ClearCh1        ;clear any waiting instruction
        bra     ExitDoInstructionRequest 
Instr01                                     ;SET GREEN to INSTR2
        tstfsz  INSTR3
        bra     $ + .4
        bra     $ + .12
        movff   INSTR3,TIMER
        call    DoInstLater_SetCh2
        bra     ExitDoInstructionRequest 
        movff   INSTR2,PHASE2
        bcf     PHASE_FLAG,Ch2ChangeSlow
        call    DoInstLater_ClearCh2
        bra     ExitDoInstructionRequest 
Instr02                                     ;SET BLUE to INSTR2
        tstfsz  INSTR3
        bra     $ + .4
        bra     $ + .12
        movff   INSTR3,TIMER                 
        call    DoInstLater_SetCh3               
        bra     ExitDoInstructionRequest 
        movff   INSTR2,PHASE3
        bcf     PHASE_FLAG,Ch3ChangeSlow   
        call    DoInstLater_ClearCh3            
        bra     ExitDoInstructionRequest 
Instr03                                     ;SET MASTER to INSTR2
        tstfsz  INSTR3
        bra     $ + .4
        bra     $ + .12
        movff   INSTR3,TIMER                    
        call    DoInstLater_SetCh4                
        bra     ExitDoInstructionRequest 
        movff   INSTR2,PHASE4
        bcf     PHASE_FLAG,Ch4ChangeSlow    
        call    DoInstLater_ClearCh4             
        bra     ExitDoInstructionRequest 
;---------------
Instr04                                     ;TOGGLE RED
        tstfsz  INSTR3                      ;is timer = 0?
        bra     $ + .4                      ;no, so do instruction later
        bra     $ + .12                     ;yes, so do instruction now
        movff   INSTR3,TIMER                ;move timer from INSTR3
        call    DoInstLater_SetCh1          ;save instruction for later execution
        bra     ExitDoInstructionRequest 
        clrf    PHASE1_STARTCNT             ;clear START instr counter 
        clrf    PHASE1                      ;turn off    
        tstfsz  PHASE1_WORK                 ;test current value
        bra     $ + .16                     ;turn on
        ;memory
        banksel CONFIG16
        btfss   CONFIG16,4                  ;memory set?
        bra     $ + .8                      ;no
        movff   PHASE1_MEM,PHASE1           ;yes, so take value from last used
        bra     $ + .4
        setf    PHASE1                      ;change to max value
        bcf     PHASE_FLAG,Ch1ChangeSlow    ;change immediately flag
        call    DoInstLater_ClearCh1
        bra     ExitDoInstructionRequest
Instr05                                     ;TOGGLE GREEN
        tstfsz  INSTR3                    
        bra     $ + .4
        bra     $ + .12
        movff   INSTR3,TIMER                   
        call    DoInstLater_SetCh2               
        bra     ExitDoInstructionRequest 
        clrf    PHASE2_STARTCNT 
        clrf    PHASE2                    
        tstfsz  PHASE2_WORK              
        bra     $ + .16        
        ;memory
        banksel CONFIG16
        btfss   CONFIG16,5             
        bra     $ + .8                
        movff   PHASE2_MEM,PHASE2        
        bra     $ + .4
        setf    PHASE2                
        bcf     PHASE_FLAG,Ch2ChangeSlow
        call    DoInstLater_ClearCh2
        bra     ExitDoInstructionRequest
Instr06                                     ;TOGGLE BLUE
        tstfsz  INSTR3                    
        bra     $ + .4
        bra     $ + .12
        movff   INSTR3,TIMER                    
        call    DoInstLater_SetCh3               
        bra     ExitDoInstructionRequest 
        clrf    PHASE3_STARTCNT
        clrf    PHASE3                               
        tstfsz  PHASE3_WORK              
        bra     $ + .16           
        ;memory
        banksel CONFIG16
        btfss   CONFIG16,6               
        bra     $ + .8                
        movff   PHASE3_MEM,PHASE3      
        bra     $ + .4
        setf    PHASE3                
        bcf     PHASE_FLAG,Ch3ChangeSlow
        call    DoInstLater_ClearCh3
        bra     ExitDoInstructionRequest
Instr07                                     ;TOGGLE MASTER
        tstfsz  INSTR3                    
        bra     $ + .4
        bra     $ + .12
        movff   INSTR3,TIMER                    
        call    DoInstLater_SetCh4               
        bra     ExitDoInstructionRequest 
        clrf    PHASE4_STARTCNT 
        clrf    PHASE4                           
        tstfsz  PHASE4_WORK              
        bra     $ + .16                  
        ;memory
        banksel CONFIG16
        btfss   CONFIG16,7                
        bra     $ + .8                      
        movff   PHASE4_MEM,PHASE4          
        bra     $ + .4
        setf    PHASE4                        
        bcf     PHASE_FLAG,Ch4ChangeSlow
        call    DoInstLater_ClearCh4
        bra     ExitDoInstructionRequest
;---------------
Instr08                                     ;RED INSTR2 STEPS DOWN
        tstfsz  INSTR3                      ;is timer = 0?
        bra     $ + .4                      ;no, so do instruction later
        bra     $ + .12                     ;yes, so do instruction now
        movff   INSTR3,TIMER                ;move timer from INSTR3
        call    DoInstLater_SetCh1          ;save instruction for later execution
        bra     ExitDoInstructionRequest 
        tstfsz  INSTR2                      ;is counter = 0?
        bra     $ + .4
        bra     ExitDoInstructionRequest    ;yes, so exit
        clrf    WREG                        ;exit when PHASE = zero
        xorwf   PHASE1,W
        bz      $ + .8
        decf    PHASE1
        decfsz  INSTR2
        bra     $ - .10
        bcf     PHASE_FLAG,Ch1ChangeSlow    ;change immediately flag
        call    DoInstLater_ClearCh1
        bra     ExitDoInstructionRequest
Instr09                                     ;GREEN INSTR2 STEPS DOWN
        tstfsz  INSTR3                 
        bra     $ + .4
        bra     $ + .12
        movff   INSTR3,TIMER                 
        call    DoInstLater_SetCh2  
        bra     ExitDoInstructionRequest 
        tstfsz  INSTR2               
        bra     $ + .4
        bra     ExitDoInstructionRequest   
        clrf    WREG                
        xorwf   PHASE2,W
        bz      $ + .8
        decf    PHASE2
        decfsz  INSTR2
        bra     $ - .10
        bcf     PHASE_FLAG,Ch2ChangeSlow
        call    DoInstLater_ClearCh2
        bra     ExitDoInstructionRequest
Instr0A                                     ;BLUE INSTR2 STEPS DOWN
        tstfsz  INSTR3                   
        bra     $ + .4
        bra     $ + .12
        movff   INSTR3,TIMER                     
        call    DoInstLater_SetCh3 
        bra     ExitDoInstructionRequest  
        tstfsz  INSTR2               
        bra     $ + .4
        bra     ExitDoInstructionRequest               
        clrf    WREG                       
        xorwf   PHASE3,W
        bz      $ + .8
        decf    PHASE3
        decfsz  INSTR2
        bra     $ - .10
        bcf     PHASE_FLAG,Ch3ChangeSlow    
        call    DoInstLater_ClearCh3
        bra     ExitDoInstructionRequest
Instr0B                                     ;MASTER INSTR2 STEPS DOWN
        tstfsz  INSTR3                   
        bra     $ + .4
        bra     $ + .12
        movff   INSTR3,TIMER                     
        call    DoInstLater_SetCh4    
        bra     ExitDoInstructionRequest   
        tstfsz  INSTR2               
        bra     $ + .4
        bra     ExitDoInstructionRequest          
        clrf    WREG                       
        xorwf   PHASE4,W
        bz      $ + .8
        decf    PHASE4
        decfsz  INSTR2
        bra     $ - .10
        bcf     PHASE_FLAG,Ch4ChangeSlow    
        call    DoInstLater_ClearCh4
        bra     ExitDoInstructionRequest
;---------------
Instr0C                                     ;RED INSTR2 STEPS UP
        tstfsz  INSTR3                      ;is timer = 0?
        bra     $ + .4                      ;no, so do instruction later
        bra     $ + .12                     ;yes, so do instruction now
        movff   INSTR3,TIMER                ;move timer from INSTR3
        call    DoInstLater_SetCh1          ;save instruction for later execution
        bra     ExitDoInstructionRequest 
        tstfsz  INSTR2                      ;is counter = 0?
        bra     $ + .4
        bra     ExitDoInstructionRequest    ;yes, so exit
        setf    WREG                        ;exit when max value
        xorwf   PHASE1,W
        bz      $ + .8
        incf    PHASE1
        decfsz  INSTR2
        bra     $ - .10
        bcf     PHASE_FLAG,Ch1ChangeSlow
        call    DoInstLater_ClearCh1
        bra     ExitDoInstructionRequest 
Instr0D                                     ;GREEN INSTR2 STEPS UP
        tstfsz  INSTR3    
        bra     $ + .4
        bra     $ + .12
        movff   INSTR3,TIMER           
        call    DoInstLater_SetCh2           
        bra     ExitDoInstructionRequest 
        tstfsz  INSTR2               
        bra     $ + .4
        bra     ExitDoInstructionRequest  
        setf    WREG                 
        xorwf   PHASE2,W
        bz      $ + .8
        incf    PHASE2
        decfsz  INSTR2
        bra     $ - .10
        bcf     PHASE_FLAG,Ch2ChangeSlow
        call    DoInstLater_ClearCh2
        bra     ExitDoInstructionRequest 
Instr0E                                     ;BLUE INSTR2 STEPS UP
        tstfsz  INSTR3    
        bra     $ + .4
        bra     $ + .12
        movff   INSTR3,TIMER            
        call    DoInstLater_SetCh3           
        bra     ExitDoInstructionRequest 
        tstfsz  INSTR2               
        bra     $ + .4
        bra     ExitDoInstructionRequest  
        setf    WREG                 
        xorwf   PHASE3,W
        bz      $ + .8
        incf    PHASE3
        decfsz  INSTR2
        bra     $ - .10
        bcf     PHASE_FLAG,Ch3ChangeSlow
        call    DoInstLater_ClearCh3
        bra     ExitDoInstructionRequest 
Instr0F                                     ;MASTER INSTR2 STEPS UP
        tstfsz  INSTR3    
        bra     $ + .4
        bra     $ + .12
        movff   INSTR3,TIMER           
        call    DoInstLater_SetCh4           
        bra     ExitDoInstructionRequest 
        tstfsz  INSTR2               
        bra     $ + .4
        bra     ExitDoInstructionRequest  
        setf    WREG                 
        xorwf   PHASE4,W
        bz      $ + .8
        incf    PHASE4
        decfsz  INSTR2
        bra     $ - .10
        bcf     PHASE_FLAG,Ch4ChangeSlow
        call    DoInstLater_ClearCh4
        bra     ExitDoInstructionRequest 
;---------------
Instr10                                     ;SET SOFTLY RED to INSTR2
        tstfsz  INSTR3                      ;is timer = 0?
        bra     $ + .4                      ;no, so do instruction later
        bra     $ + .12                     ;yes, so do instruction now
        movff   INSTR3,TIMER                ;move timer from INSTR3
        call    DoInstLater_SetCh1          ;save instruction for later execution
        bra     ExitDoInstructionRequest 
        movff   INSTR2,PHASE1               ;move desired value
        bsf     PHASE_FLAG,Ch1ChangeSlow    ;change softly flag
        call    DoInstLater_ClearCh1        ;clear any waiting instruction
        bra     ExitDoInstructionRequest
Instr11                                     ;SET SOFTLY GREEN to INSTR2
        tstfsz  INSTR3
        bra     $ + .4
        bra     $ + .12
        movff   INSTR3,TIMER 
        call    DoInstLater_SetCh2
        bra     ExitDoInstructionRequest 
        movff   INSTR2,PHASE2
        bsf     PHASE_FLAG,Ch2ChangeSlow
        call    DoInstLater_ClearCh2
        bra     ExitDoInstructionRequest
Instr12                                     ;SET SOFTLY BLUE to INSTR2
        tstfsz  INSTR3
        bra     $ + .4
        bra     $ + .12
        movff   INSTR3,TIMER 
        call    DoInstLater_SetCh3
        bra     ExitDoInstructionRequest 
        movff   INSTR2,PHASE3
        bsf     PHASE_FLAG,Ch3ChangeSlow
        call    DoInstLater_ClearCh3
        bra     ExitDoInstructionRequest
Instr13                                     ;SET SOFTLY MASTER to INSTR2
        tstfsz  INSTR3
        bra     $ + .4
        bra     $ + .12
        movff   INSTR3,TIMER 
        call    DoInstLater_SetCh4
        bra     ExitDoInstructionRequest 
        movff   INSTR2,PHASE4
        bsf     PHASE_FLAG,Ch4ChangeSlow
        call    DoInstLater_ClearCh4
        bra     ExitDoInstructionRequest
;---------------
Instr14                                     ;STOP RED
        movlw   0x00                        ;stop after START? (400ms passed?)
        xorwf   PHASE1_STARTCNT,W
        bz      $ + .6                      ;yes, so stop dimming/brightening
        clrf    INSTR3                      ;no, clear TIMER for toggle instruction
        bra     Instr04                     ;go to toggle channel state
        movff   PHASE1_WORK,PHASE1          ;stop phase
        bcf     PHASE_FLAG,Ch1ChangeSlow    ;change immediately flag
        call    DoInstLater_ClearCh1
        bra     ExitDoInstructionRequest
Instr15                                     ;STOP GREEN
        movlw   0x00
        xorwf   PHASE2_STARTCNT,W
        bz      $ + .6
        clrf    INSTR3              
        bra     Instr05           
        movff   PHASE2_WORK,PHASE2   
        bcf     PHASE_FLAG,Ch2ChangeSlow
        call    DoInstLater_ClearCh2
        bra     ExitDoInstructionRequest
Instr16                                     ;STOP BLUE
        movlw   0x00
        xorwf   PHASE3_STARTCNT,W
        bz      $ + .6
        clrf    INSTR3     
        bra     Instr06        
        movff   PHASE3_WORK,PHASE3    
        bcf     PHASE_FLAG,Ch3ChangeSlow
        call    DoInstLater_ClearCh3
        bra     ExitDoInstructionRequest
Instr17                                     ;STOP MASTER
        movlw   0x00
        xorwf   PHASE4_STARTCNT,W
        bz      $ + .6
        clrf    INSTR3   
        bra     Instr07            
        movff   PHASE4_WORK,PHASE4   
        bcf     PHASE_FLAG,Ch4ChangeSlow
        call    DoInstLater_ClearCh4
        bra     ExitDoInstructionRequest
;---------------
Instr18                                     ;START RED
        movlw   .20                         ;R (move 20x 20ms = 400ms to counter)
        movwf   PHASE1_STARTCNT 
        clrf    TIMER                       ;instruction will be visible in status msg
        call    DoInstLater_SetCh1
        bra     ExitDoInstructionRequest
Instr19                                     ;START GREEN
        movlw   .20                
        movwf   PHASE2_STARTCNT 
        clrf    TIMER          
        call    DoInstLater_SetCh2
        bra     ExitDoInstructionRequest
Instr1A                                     ;START BLUE
        movlw   .20                    
        movwf   PHASE3_STARTCNT         
        clrf    TIMER  
        call    DoInstLater_SetCh3
        bra     ExitDoInstructionRequest
Instr1B                                     ;START MASTER
        movlw   .20                  
        movwf   PHASE4_STARTCNT         
        clrf    TIMER  
        call    DoInstLater_SetCh4
        bra     ExitDoInstructionRequest
;---------------
Instr1C                                     ;SET RED SPEED TO    
        movff   INSTR2,TIME1
        bra     ExitDoInstructionRequest
Instr1D                                     ;SET GREEN SPEED TO    
        movff   INSTR2,TIME2
        bra     ExitDoInstructionRequest
Instr1E                                     ;SET BLUE SPEED TO    
        movff   INSTR2,TIME3
        bra     ExitDoInstructionRequest
Instr1F                                     ;SET MASTER SPEED TO    
        movff   INSTR2,TIME4
        bra     ExitDoInstructionRequest
;---------------
Instr20                                     ;SET RGB TO
        tstfsz  INSTR5                      ;is timer = 0?
        bra     $ + .4                      ;no, so do instruction later
        bra     Instr20_DoNow               ;yes, so do instruction now
        movff   INSTR5,TIMER                ;move timer from INSTR5
        ;phase1
        movlw   0x00                        ;change instr to SET RED TO
        movwf   INSTR1  
        call    DoInstLater_SetCh1          ;save instruction for later execution
        ;phase2
        movlw   0x01                        ;change instr to SET GREEN TO
        movwf   INSTR1  
        movff   INSTR3,INSTR2
        call    DoInstLater_SetCh2 
        ;phase3
        movlw   0x02                        ;change instr to SET BLUE TO
        movwf   INSTR1  
        movff   INSTR4,INSTR2
        call    DoInstLater_SetCh3 
        bra     ExitDoInstructionRequest 
Instr20_DoNow
        movff   INSTR2,PHASE1
        movff   INSTR3,PHASE2
        movff   INSTR4,PHASE3
        bcf     PHASE_FLAG,Ch1ChangeSlow    ;change immediately flag
        bcf     PHASE_FLAG,Ch2ChangeSlow
        bcf     PHASE_FLAG,Ch3ChangeSlow
        call    DoInstLater_ClearCh1        ;clear any waiting instruction
        call    DoInstLater_ClearCh2
        call    DoInstLater_ClearCh3
        bra     ExitDoInstructionRequest
;---------------
Instr21                                     ;SET RGB SOFTLY TO
        tstfsz  INSTR5                      ;is timer = 0?
        bra     $ + .4                      ;no, so do instruction later
        bra     Instr21_DoNow               ;yes, so do instruction now
        movff   INSTR5,TIMER                ;move timer from INSTR5
        ;phase1
        movlw   0x10                        ;change instr to SET SOFTLY RED TO
        movwf   INSTR1  
        call    DoInstLater_SetCh1          ;save instruction for later execution
        ;phase2
        movlw   0x11                        ;change instr to SET SOFTLY GREEN TO
        movwf   INSTR1  
        movff   INSTR3,INSTR2
        call    DoInstLater_SetCh2 
        ;phase3
        movlw   0x12                        ;change instr to SET SOFTLY BLUE TO
        movwf   INSTR1  
        movff   INSTR4,INSTR2
        call    DoInstLater_SetCh3 
        bra     ExitDoInstructionRequest 
Instr21_DoNow
        movff   INSTR2,PHASE1
        movff   INSTR3,PHASE2
        movff   INSTR4,PHASE3
        bsf     PHASE_FLAG,Ch1ChangeSlow    ;change softly flag
        bsf     PHASE_FLAG,Ch2ChangeSlow
        bsf     PHASE_FLAG,Ch3ChangeSlow
        call    DoInstLater_ClearCh1        ;clear any waiting instruction
        call    DoInstLater_ClearCh2
        call    DoInstLater_ClearCh3
        bra     ExitDoInstructionRequest
;---------------
Instr22                                     ;SET RGB SPEED TO    
        movff   INSTR2,TIME1
        movff   INSTR2,TIME2
        movff   INSTR2,TIME3
        bra     ExitDoInstructionRequest
;---------------
Instr23                                     ;RGB SPEED UP
        tstfsz  TIME1                       ;do not dec if already zero
        bra     $ + .4
        bra     $ + .4
        decf    TIME1
        movff   TIME1,TIME2
        movff   TIME1,TIME3
        bra     ExitDoInstructionRequest
;---------------
Instr24                                     ;RGB SPEED DOWN
        movlw   0xFF                        ;256s
        cpfseq  TIME1
        incf    TIME1
        movff   TIME1,TIME2
        movff   TIME1,TIME3
        bra     ExitDoInstructionRequest
;---------------
Instr25                                     ;PROGRAM
        bsf     PROGRAM_CALL,Called         ;set program called flag
        bcf     PROGRAM_CALL,FirstCall      ;clear first time call flag
        movf    INSTR2,W                    ;program the same as before?
        xorwf   PROGRAM_NR,W
        bz      $ + 4                       ;yes
        bsf     PROGRAM_CALL,FirstCall      ;no, so set first time call flag
        movff   INSTR2,PROGRAM_NR           ;move program number
        clrf    TIMER                       ;instruction will be visible in status msg
        call    DoInstLater_SetCh1             
        call    DoInstLater_SetCh2
        call    DoInstLater_SetCh3
        bra     ExitDoInstructionRequest
;---------------
ExitDoInstructionRequest
        call    EepromToSave                ;save new states to eeprom in a few seconds
    return                        

;------------------------------------------------------------------------------
; Routine:          DO INSTRUCTION LATER
;------------------------------------------------------------------------------
; Overview:         It saves instruction for particular channel for later
;                   execution
;------------------------------------------------------------------------------
DoInstLater_SetCh1
        call    SetTimer                    ;update SUBTIMER1 & SUBTIMER2 registers
        movff   INSTR1,Instr1Ch1            ;copy registers
        movff   INSTR2,Instr2Ch1
        movff   TIMER,TimerCh1
        movff   SUBTIMER1,SubTmr1Ch1
        movff   SUBTIMER2,SubTmr2Ch1
    return
DoInstLater_SetCh2
        call    SetTimer
        movff   INSTR1,Instr1Ch2
        movff   INSTR2,Instr2Ch2
        movff   TIMER,TimerCh2
        movff   SUBTIMER1,SubTmr1Ch2
        movff   SUBTIMER2,SubTmr2Ch2
    return
DoInstLater_SetCh3
        call    SetTimer
        movff   INSTR1,Instr1Ch3
        movff   INSTR2,Instr2Ch3
        movff   TIMER,TimerCh3
        movff   SUBTIMER1,SubTmr1Ch3
        movff   SUBTIMER2,SubTmr2Ch3
    return
DoInstLater_SetCh4
        call    SetTimer
        movff   INSTR1,Instr1Ch4
        movff   INSTR2,Instr2Ch4
        movff   TIMER,TimerCh4
        movff   SUBTIMER1,SubTmr1Ch4
        movff   SUBTIMER2,SubTmr2Ch4
    return
DoInstLater_ClearCh1
        banksel Instr1Ch1
        setf    Instr1Ch1
        setf    Instr2Ch1
        clrf    TimerCh1
        clrf    SubTmr1Ch1
        clrf    SubTmr2Ch1
        clrf    PROGRAM_NR
    return
DoInstLater_ClearCh2
        banksel Instr1Ch2
        setf    Instr1Ch2
        setf    Instr2Ch2
        clrf    TimerCh2
        clrf    SubTmr1Ch2
        clrf    SubTmr2Ch2
        clrf    PROGRAM_NR
    return
DoInstLater_ClearCh3
        banksel Instr1Ch3
        setf    Instr1Ch3
        setf    Instr2Ch3
        clrf    TimerCh3
        clrf    SubTmr1Ch3
        clrf    SubTmr2Ch3
        clrf    PROGRAM_NR
    return
DoInstLater_ClearCh4
        banksel Instr1Ch4
        setf    Instr1Ch4
        setf    Instr2Ch4
        clrf    TimerCh4
        clrf    SubTmr1Ch4
        clrf    SubTmr2Ch4
    return

;==============================================================================
;                   RGB LED CONTOLLER PROCEDURES
;==============================================================================
;------------------------------------------------------------------------------
; Routine:          PWM INITIALIZATION
;------------------------------------------------------------------------------
; Overview:         Sets PWM
;------------------------------------------------------------------------------
RGB_PWMInitialization
        bcf     TRISC,RC2                   ;pwm pins must be outputs
        bcf     TRISC,RC6
        bcf     TRISC,RC7
        banksel PMD0
        bcf     PMD0,CCP2MD                 ;enable PWM 2 module
        bcf     PMD0,CCP3MD                 ;enable PWM 3 module
        bcf     PMD0,CCP4MD                 ;enable PWM 4 module
        bcf     PMD1,TMR4MD                 ;enable timer 4
        movlw   b'000000111'                ;timer on, prescaler 16
        movwf   T4CON
        movlw   b'00001110'                 ;use TIMER4 for PWM
        movwf   CCPTMRS
        movlw   0xFF                        ;timer4 counts only up to value of PR4, so pwm period:
        movwf   PR4                         ;PWM period    = (PR4+1)*4*Tosc*TMR4prescaler = (PR4+1)*4*0,125us*16 = (1/488Hz)s
        movlw   0                           ;PWM duty
        movwf   CCPR2L                    
        movwf   CCPR3L    
        movwf   CCPR4L    
        movlw   b'00001111'                 ;PWM mode
        movwf   CCP2CON
        movwf   CCP3CON
        movwf   CCP4CON
    return
;------------------------------------------------------------------------------
; Routine:          RGB LED CTRL POWER UP STATES
;------------------------------------------------------------------------------
; Overview:         Sets power up states according to configuration
;------------------------------------------------------------------------------
RGB_PowerUpStates
    ;CONFIG
        banksel CONFIG0
        ;min & max                          ;take minimum and maximum values
        movff   CONFIG0,PHASE1_MIN
        movff   CONFIG1,PHASE1_MAX
        movff   CONFIG2,PHASE2_MIN
        movff   CONFIG3,PHASE2_MAX
        movff   CONFIG4,PHASE3_MIN
        movff   CONFIG5,PHASE3_MAX
        movff   CONFIG6,PHASE4_MIN
        movff   CONFIG7,PHASE4_MAX
        ;state phase 1
        btfss   CONFIG16,0                  ;set 1?
        bra     $ + .8                      ;no
        movff   CONFIG9,PHASE1              ;bit set, so take value from last saved
        bra     $ + .4
        movff   CONFIG8,PHASE1              ;bit cleared, so take value from set reg
        ;phase 2
        btfss   CONFIG16,1              
        bra     $ + .8                      
        movff   CONFIG11,PHASE2         
        bra     $ + .4
        movff   CONFIG10,PHASE2        
        ;phase 3
        btfss   CONFIG16,2                
        bra     $ + .8                   
        movff   CONFIG13,PHASE3      
        bra     $ + .4
        movff   CONFIG12,PHASE3           
        ;master
        btfss   CONFIG16,3             
        bra     $ + .8                      
        movff   CONFIG15,PHASE4        
        bra     $ + .4
        movff   CONFIG14,PHASE4        
        ;speed                              ;take dimming/brightening speed values
        movff   CONFIG17,TIME1
        movff   CONFIG18,TIME2
        movff   CONFIG19,TIME3
        movff   CONFIG20,TIME4
    ;OTHER REGISTERS
        bcf     PHASE_FLAG,Ch1ChangeSlow    ;change immediately
        bcf     PHASE_FLAG,Ch2ChangeSlow
        bcf     PHASE_FLAG,Ch3ChangeSlow
        bsf     PHASE_FLAG,Ch4ChangeSlow    ;change slowly
        clrf    PHASE4_WORK                 ;to allow slow change at start
        setf    PHASE1_MEM                  ;remembered value equals 255
        setf    PHASE2_MEM
        setf    PHASE3_MEM
        setf    PHASE4_MEM
        clrf    PHASE1_STARTCNT             ;START instruction timers
        clrf    PHASE2_STARTCNT
        clrf    PHASE3_STARTCNT
        clrf    PHASE4_STARTCNT
        clrf    PROGRAM_NR                  ;program number reg
        clrf    PROGRAM_CALL                ;program called flag reg
    return

;------------------------------------------------------------------------------
; Routine:          RGB CONTROL
;------------------------------------------------------------------------------
; Overview:         It periodically (on every 20ms timer interrupt) updates 
;                   all RGB control settings
;------------------------------------------------------------------------------
RGB_Control_20ms
        ;count how long the START instruction is present
        rcall   RGB_DSCPhase1               ;decrement START counter
        rcall   RGB_DSCPhase2               ;decrement START counter
        rcall   RGB_DSCPhase3               ;decrement START counter
        rcall   RGB_DSCPhase4               ;decrement START counter
        ;update PHASE regs
        btfsc   PHASE_FLAG,Ch1ChangeSlow    ;phase 1 change slowly?
        bra     $ + .6                      ;yes
        rcall   RGB_UpdateImmediatelyCh1    ;no     
        bra     $ + .4
        rcall   RGB_UpdateSoftlyCh1
        btfsc   PHASE_FLAG,Ch2ChangeSlow    ;phase 2 change slowly?
        bra     $ + .6
        rcall   RGB_UpdateImmediatelyCh2             
        bra     $ + .4
        rcall   RGB_UpdateSoftlyCh2
        btfsc   PHASE_FLAG,Ch3ChangeSlow    ;phase 3 change slowly?
        bra     $ + .6
        rcall   RGB_UpdateImmediatelyCh3
        bra     $ + .4
        rcall   RGB_UpdateSoftlyCh3
        btfsc   PHASE_FLAG,Ch4ChangeSlow    ;phase 4 change slowly?
        bra     $ + .6
        rcall   RGB_UpdateImmediatelyCh4
        bra     $ + .4
        rcall   RGB_UpdateSoftlyCh4
        ;memory                             ;remember last value of master channel
        tstfsz  PHASE1_WORK
        movff   PHASE1_WORK,PHASE1_MEM      ;save phase in mem reg
        tstfsz  PHASE2_WORK
        movff   PHASE2_WORK,PHASE2_MEM 
        tstfsz  PHASE3_WORK
        movff   PHASE3_WORK,PHASE3_MEM 
        tstfsz  PHASE4_WORK
        movff   PHASE4_WORK,PHASE4_MEM 
        ;move phases to PWM
        rcall    RGB_UpdatePWM
        ;send new states
        rcall   LEDStates
RGB_ControlExit
    return

;------------------------------------------------------------------------------
; Routine:          RGB UPDATE PWM REGISTERS
;------------------------------------------------------------------------------
; Overview:         Moves PHASE registers to PWM
;------------------------------------------------------------------------------
RGB_UpdatePWM
        banksel CCPR2L
        movf    PHASE1_WORK,W               ;check channel 1 against master channel
        mulwf   PHASE4_WORK                 ;multiply by MASTER
        movf    PRODH,W                     ;take only PRODH (means divide by 256)
        movwf   CCPR2L                      ;load to PWM
        movf    PHASE2_WORK,W               ;check channel 2 against master channel
        mulwf   PHASE4_WORK
        movf    PRODH,W      
        movwf   CCPR3L
        movf    PHASE3_WORK,W               ;check channel 3 against master channel
        mulwf   PHASE4_WORK
        movf    PRODH,W    
        movwf   CCPR4L
    return

;------------------------------------------------------------------------------
; Routine:          RGB DECREMENT START COUNTERS
;------------------------------------------------------------------------------
; Overview:         Decrements counters of START instruction. If start is
;                   present for longer than 400ms then dimming/brightening starts.
;------------------------------------------------------------------------------
RGB_DSCPhase1
        tstfsz  PHASE1_STARTCNT             ;zero already?
        bra     $ + .4
        bra     RGB_DSCPhase1Exit           ;yes, so exit
        decfsz  PHASE1_STARTCNT             ;decrement counter
        bra     RGB_DSCPhase1Exit           ;and exit if not zero
        ;if counter zero, start dim/bright
        bsf     PHASE_FLAG,Ch1ChangeSlow    ;change softly flag
        movf    PHASE1_WORK,W               ;if channel is at maximum then start dimming
        cpfseq  PHASE1_MAX
        bra     $ + .4
        bra     Phase1Dim
        movf    PHASE1_WORK,W               ;if channel is at minimum then start brightening
        cpfseq  PHASE1_MIN
        bra     $ + .4
        bra     Phase1Bright
        tstfsz  PHASE1_WORK                 ;if channel is turned off start brightening
        bra     $ + .4
        bra     Phase1Bright
        btfss   PHASE_FLAG,0                ;if none of them, then check what was last 0-dimming 1-brightening?
        bra     $ + .6
        bra     Phase1Dim
        bra     $ + .4
        bra     Phase1Bright
Phase1Bright
        movff   PHASE1_MAX,PHASE1           ;brightening - set maximum as aim
        bra     RGB_DSCPhase1Exit
Phase1Dim
        movff   PHASE1_MIN,PHASE1           ;dimming - set mminimum as aim
        bra     RGB_DSCPhase1Exit
RGB_DSCPhase1Exit
    return
;-------------------------------
RGB_DSCPhase2
        tstfsz  PHASE2_STARTCNT   
        bra     $ + .4
        bra     RGB_DSCPhase2Exit
        decfsz  PHASE2_STARTCNT
        bra     RGB_DSCPhase2Exit
        bsf     PHASE_FLAG,Ch2ChangeSlow    
        movf    PHASE2_WORK,W               
        cpfseq  PHASE2_MAX
        bra     $ + .4
        bra     Phase2Dim
        movf    PHASE2_WORK,W                
        cpfseq  PHASE2_MIN
        bra     $ + .4
        bra     Phase2Bright
        tstfsz  PHASE2_WORK                    
        bra     $ + .4
        bra     Phase2Bright
        btfss   PHASE_FLAG,1              
        bra     $ + .6
        bra     Phase2Dim
        bra     $ + .4
        bra     Phase2Bright
Phase2Bright
        movff   PHASE2_MAX,PHASE2        
        bra     RGB_DSCPhase2Exit
Phase2Dim
        movff   PHASE2_MIN,PHASE2       
        bra     RGB_DSCPhase2Exit
RGB_DSCPhase2Exit
    return
;-------------------------------
RGB_DSCPhase3
        tstfsz  PHASE3_STARTCNT   
        bra     $ + .4
        bra     RGB_DSCPhase3Exit
        decfsz  PHASE3_STARTCNT
        bra     RGB_DSCPhase3Exit
        bsf     PHASE_FLAG,Ch3ChangeSlow   
        movf    PHASE3_WORK,W                
        cpfseq  PHASE3_MAX
        bra     $ + .4
        bra     Phase3Dim
        movf    PHASE3_WORK,W              
        cpfseq  PHASE3_MIN
        bra     $ + .4
        bra     Phase3Bright
        tstfsz  PHASE3_WORK                   
        bra     $ + .4
        bra     Phase3Bright
        btfss   PHASE_FLAG,2           
        bra     $ + .6
        bra     Phase3Dim
        bra     $ + .4
        bra     Phase3Bright
Phase3Bright
        movff   PHASE3_MAX,PHASE3         
        bra     RGB_DSCPhase3Exit
Phase3Dim
        movff   PHASE3_MIN,PHASE3        
        bra     RGB_DSCPhase3Exit
RGB_DSCPhase3Exit
    return
;-------------------------------
RGB_DSCPhase4
        tstfsz  PHASE4_STARTCNT   
        bra     $ + .4
        bra     RGB_DSCPhase4Exit
        decfsz  PHASE4_STARTCNT
        bra     RGB_DSCPhase4Exit
        bsf     PHASE_FLAG,Ch4ChangeSlow    
        movf    PHASE4_WORK,W               
        cpfseq  PHASE4_MAX
        bra     $ + .4
        bra     Phase4Dim
        movf    PHASE4_WORK,W             
        cpfseq  PHASE4_MIN
        bra     $ + .4
        bra     Phase4Bright
        tstfsz  PHASE4_WORK                
        bra     $ + .4
        bra     Phase4Bright
        btfss   PHASE_FLAG,3             
        bra     $ + .6
        bra     Phase4Dim
        bra     $ + .4
        bra     Phase4Bright
Phase4Bright
        movff   PHASE4_MAX,PHASE4         
        bra     RGB_DSCPhase4Exit
Phase4Dim
        movff   PHASE4_MIN,PHASE4          
        bra     RGB_DSCPhase4Exit
RGB_DSCPhase4Exit
    return


;------------------------------------------------------------------------------
; Routine:          RGB UPDATE LED STATES
;------------------------------------------------------------------------------
; Overview:         It updates immediately or slowly states for all channels
;                   acording to new settings
;------------------------------------------------------------------------------
RGB_UpdateImmediatelyCh1
        movff   PHASE1,PHASE1_WORK          ;move new PHASE value
        rcall   MAMMPhase1                  ;match against MIN & MAX              
        return
RGB_UpdateImmediatelyCh2
        movff   PHASE2,PHASE2_WORK
        rcall   MAMMPhase2            
        return
RGB_UpdateImmediatelyCh3
        movff   PHASE3,PHASE3_WORK
        rcall   MAMMPhase3            
        return
RGB_UpdateImmediatelyCh4
        movff   PHASE4,PHASE4_WORK
        rcall   MAMMPhase4               
        return
;------------------------------------------------------------------------------
RGB_UpdateSoftlyCh1
        movf    PHASE1,W                    ;check if PHASE=PHASE_WORK
        cpfseq  PHASE1_WORK                
        bra     $ + .4                      ;not equal
    return
        call    EepromToSave                ;pospone saving to eeprom
        movf    PHASE1,W                    ;check if PHASE<PHASE_WORK
        cpfslt  PHASE1_WORK
        bra     GoToDim1                    ;not less
        bra     GoToBright1                 ;less
GoToDim1
        rcall   DecPHASE1
        call    MAMMPhase1                  ;match against MIN & MAX  
        bcf     PHASE_FLAG,Ch1LastWasBright ;set mark=0 - last was dimming
    return
GoToBright1
        rcall   IncPHASE1
        call    MAMMPhase1                  ;match against MIN & MAX 
        bsf     PHASE_FLAG,Ch1LastWasBright ;set mark=1 - last was bright
    return
;-------------------------------
RGB_UpdateSoftlyCh2
        movf    PHASE2,W        
        cpfseq  PHASE2_WORK                
        bra     $ + .4
    return
        call    EepromToSave
        movf    PHASE2,W                  
        cpfslt  PHASE2_WORK
        bra     GoToDim2                 
        bra     GoToBright2                  
GoToDim2
        rcall   DecPHASE2
        call    MAMMPhase2                  
        bcf     PHASE_FLAG,Ch2LastWasBright 
    return
GoToBright2
        rcall   IncPHASE2
        call    MAMMPhase2                 
        bsf     PHASE_FLAG,Ch2LastWasBright    
    return
;-------------------------------
RGB_UpdateSoftlyCh3
        movf    PHASE3,W                 
        cpfseq  PHASE3_WORK                
        bra     $ + .4
    return
        call    EepromToSave
        movf    PHASE3,W                    
        cpfslt  PHASE3_WORK
        bra     GoToDim3                    
        bra     GoToBright3                
GoToDim3
        rcall   DecPHASE3
        call    MAMMPhase3                  
        bcf     PHASE_FLAG,Ch3LastWasBright    
    return
GoToBright3
        rcall   IncPHASE3
        call    MAMMPhase3               
        bsf     PHASE_FLAG,Ch3LastWasBright    
    return
;-------------------------------
RGB_UpdateSoftlyCh4
        movf    PHASE4,W                
        cpfseq  PHASE4_WORK                
        bra     $ + .4
    return
        call    EepromToSave
        movf    PHASE4,W                   
        cpfslt  PHASE4_WORK
        bra     GoToDim4                   
        bra     GoToBright4                 
GoToDim4
        rcall   DecPHASE4
        call    MAMMPhase4                 
        bcf     PHASE_FLAG,Ch4LastWasBright    
    return
GoToBright4
        rcall   IncPHASE4
        call    MAMMPhase4              
        bsf     PHASE_FLAG,Ch4LastWasBright    
    return
;-------------------------------
EepromToSave                                ;indicate that save to eeprom nedded
        banksel EEPROMTIMER
        movlw   0x06                        ;wait 6s before saving to eeprom
        movwf   EEPROMTIMER
    return

;------------------------------------------------------------------------------
; Decrement PHASEs                          ;decrements PHASE regs
;------------------------------------------------------------------------------
DecPhaseGet20msStep ;calculates PHASE delay step based on WREG reg (0-255) (1s-256s).
                    ;gives value which has to be added tp PHASE reg on every 20ms
                    ;24 bit result (8bit int, 16bit fraction) in PHASE_STEP=255/(WREG*50+50)
        mullw   .50                         ;step 50 times per second for 20ms interrupt
        movff   PRODL,DIVISOR_L             ;WREG*50
        movff   PRODH,DIVISOR_H
        movlw   .50                         ;WREG*50+50
        addwf   DIVISOR_L
        clrf    WREG                        ;check if carry
        addwfc  DIVISOR_H     
        setf    DIVIDEND_U                  ;set divident 255.00 = 0xFF.0000 (max PHASE value)
        clrf    DIVIDEND_H
        clrf    DIVIDEND_L
        clrf    PHASE_STEP_U                ;clear result         
        clrf    PHASE_STEP_H
        clrf    PHASE_STEP_L
        clrf    REMAINDER_H                 ;clear reminder
        clrf    REMAINDER_L

        movlw   .24                         ;255/(WREG*50+50)
        movwf   BIT_COUNTER                 ;shift 24 bits
DPG20S_Loop        
        ;rotate bit by bit
        rlcf    DIVIDEND_L                  ;rotate divident
        rlcf    DIVIDEND_H
        rlcf    DIVIDEND_U
        rlcf    REMAINDER_L,F               ;rotate remainder through MSBit of dividend
        rlcf    REMAINDER_H,F
        ;check if divisor can be substracted from remainder
        movf    DIVISOR_L,W
        subwf   REMAINDER_L,W
        movf    DIVISOR_H,W
        subwfb  REMAINDER_H,W
        btfss   STATUS,C
        bra     DPG20S_ShiftResult          ;C=!B=0, borrowing, so do not substruct
        ;substract divisor from remainder
        movf    DIVISOR_L,W
        subwf   REMAINDER_L,F
        movf    DIVISOR_H,W
        subwfb  REMAINDER_H,F
DPG20S_ShiftResult      
        rlcf    PHASE_STEP_L                ;rotate result         
        rlcf    PHASE_STEP_H
        rlcf    PHASE_STEP_U

        decfsz  BIT_COUNTER                 ;all bits done?
        bra     DPG20S_Loop                 ;not yet
    return

;-------------------------------
DecPHASE1AddStep
        movf    TIME1,W
        rcall   DecPhaseGet20msStep         ;calculates 20ms PHASE step based on TIME reg
        movf    PHASE_STEP_L,W              ;add step to PHASE_COUNTER
        addwf   PHASE1_CNT_L,F              ;PHASE_COUNTER fraction
        movf    PHASE_STEP_H,W
        addwfc  PHASE1_CNT_H,F              ;PHASE_COUNTER fraction
        movf    PHASE_STEP_U,W
        addwfc  PHASE1_CNT_U,F              ;PHASE_COUNTER whole
    return
;---------------
DecPHASE1
        rcall   DecPHASE1AddStep            ;add step into PHASE_COUNTER
        movf    PHASE1_CNT_U,W              ;decrement PHASE only PHASE_COUNTER whole number times
        bz      $ + .8                      ;whole part equals zero, so exit
        rcall   DecPHASE1Once
        decfsz  PHASE1_CNT_U
        bra     $ - .4    
    return
DecPHASE1Once
        tstfsz  PHASE1_WORK                ;do not dim if already zero
        bra     $ + .4
        bra     $ + .4
        decf    PHASE1_WORK
    return
;---------------
IncPHASE1
        rcall   DecPHASE1AddStep            ;add step into PHASE_COUNTER
        movf    PHASE1_CNT_U,W              ;increment PHASE only PHASE_COUNTER whole number times
        bz      $ + .8                      ;whole part equals zero, so exit
        rcall   IncPHASE1Once
        decfsz  PHASE1_CNT_U
        bra     $ - .4    
    return
IncPHASE1Once
        infsnz  PHASE1_WORK                 ;skip if not zero after increment
        decf    PHASE1_WORK                 ;if PHASE_WORK euqals 0x00 step back to 0xFF
    return
;-------------------------------
DecPHASE2AddStep
        movf    TIME2,W
        rcall   DecPhaseGet20msStep
        movf    PHASE_STEP_L,W
        addwf   PHASE2_CNT_L,F      
        movf    PHASE_STEP_H,W
        addwfc  PHASE2_CNT_H,F    
        movf    PHASE_STEP_U,W
        addwfc  PHASE2_CNT_U,F      
    return
;---------------
DecPHASE2
        rcall   DecPHASE2AddStep       
        movf    PHASE2_CNT_U,W     
        bz      $ + .8             
        rcall   DecPHASE2Once
        decfsz  PHASE2_CNT_U
        bra     $ - .4    
    return
DecPHASE2Once
        tstfsz  PHASE2_WORK            
        bra     $ + .4
        bra     $ + .4
        decf    PHASE2_WORK
    return
;---------------
IncPHASE2
        rcall   DecPHASE2AddStep        
        movf    PHASE2_CNT_U,W         
        bz      $ + .8                
        rcall   IncPHASE2Once
        decfsz  PHASE2_CNT_U
        bra     $ - .4    
    return
IncPHASE2Once
        infsnz  PHASE2_WORK        
        decf    PHASE2_WORK        
    return
;-------------------------------
DecPHASE3AddStep
        movf    TIME3,W
        rcall   DecPhaseGet20msStep
        movf    PHASE_STEP_L,W
        addwf   PHASE3_CNT_L,F      
        movf    PHASE_STEP_H,W
        addwfc  PHASE3_CNT_H,F    
        movf    PHASE_STEP_U,W
        addwfc  PHASE3_CNT_U,F      
    return
;---------------
DecPHASE3
        rcall   DecPHASE3AddStep       
        movf    PHASE3_CNT_U,W     
        bz      $ + .8             
        rcall   DecPHASE3Once
        decfsz  PHASE3_CNT_U
        bra     $ - .4    
    return
DecPHASE3Once
        tstfsz  PHASE3_WORK            
        bra     $ + .4
        bra     $ + .4
        decf    PHASE3_WORK
    return
;---------------
IncPHASE3
        rcall   DecPHASE3AddStep        
        movf    PHASE3_CNT_U,W         
        bz      $ + .8                
        rcall   IncPHASE3Once
        decfsz  PHASE3_CNT_U
        bra     $ - .4    
    return
IncPHASE3Once
        infsnz  PHASE3_WORK        
        decf    PHASE3_WORK        
    return
;-------------------------------
DecPHASE4AddStep
        movf    TIME4,W
        rcall   DecPhaseGet20msStep
        movf    PHASE_STEP_L,W
        addwf   PHASE4_CNT_L,F      
        movf    PHASE_STEP_H,W
        addwfc  PHASE4_CNT_H,F    
        movf    PHASE_STEP_U,W
        addwfc  PHASE4_CNT_U,F      
    return
;---------------
DecPHASE4
        rcall   DecPHASE4AddStep       
        movf    PHASE4_CNT_U,W     
        bz      $ + .8             
        rcall   DecPHASE4Once
        decfsz  PHASE4_CNT_U
        bra     $ - .4    
    return
DecPHASE4Once
        tstfsz  PHASE4_WORK            
        bra     $ + .4
        bra     $ + .4
        decf    PHASE4_WORK
    return

;---------------
IncPHASE4
        rcall   DecPHASE4AddStep        
        movf    PHASE4_CNT_U,W         
        bz      $ + .8                
        rcall   IncPHASE4Once
        decfsz  PHASE4_CNT_U
        bra     $ - .4    
    return
IncPHASE4Once
        infsnz  PHASE4_WORK        
        decf    PHASE4_WORK        
    return

;------------------------------------------------------------------------------
;  Match Wworking Phase Against Min & Max
;------------------------------------------------------------------------------
MAMMPhase1                                  ;PHASE 1
        movf    PHASE1_MIN,W                ;MIN  
        cpfslt  PHASE1_WORK                    ;is PHASE_WORK < PHASE_MIN ?
        bra     $ + .12                     ;no, so keep current value & go to MAX check
        movwf   PHASE1_WORK                 ;yes, so change PHASE_WORK to PHASE_MIN
        tstfsz  PHASE1                      ;is traget, value of PHASE=0?
        bra     $ + .6                      ;no, so keep PHASE_MIN value
        clrf    PHASE1_WORK                 ;yes, so change to zero
    return
        movf    PHASE1_MAX,W                ;MAX
        cpfsgt  PHASE1_WORK                    ;is PHASE_WORK > PHASE_MAX ?
        bra     $ + .6                      ;no, so keep current value       
        movwf   PHASE1_WORK                 ;yes, so change PHASE_WORK to PHASE_MAX
        movwf   PHASE1                      ;and change PHASE to PHASE_MAX, so will not be compared again
    return
;---------------
MAMMPhase2                                  ;PHASE 2
        movf    PHASE2_MIN,W                ;MIN  
        cpfslt  PHASE2_WORK                  
        bra     $ + .12                   
        movwf   PHASE2_WORK    
        tstfsz  PHASE2                    
        bra     $ + .6                   
        clrf    PHASE2_WORK         
    return
        movf    PHASE2_MAX,W                ;MAX
        cpfsgt  PHASE2_WORK    
        bra     $ + .6         
        movwf   PHASE2_WORK
        movwf   PHASE2
    return
;---------------
MAMMPhase3                                  ;PHASE 3
        movf    PHASE3_MIN,W                ;MIN  
        cpfslt  PHASE3_WORK                  
        bra     $ + .12                   
        movwf   PHASE3_WORK    
        tstfsz  PHASE3                    
        bra     $ + .6                   
        clrf    PHASE3_WORK         
    return
        movf    PHASE3_MAX,W                ;MAX
        cpfsgt  PHASE3_WORK    
        bra     $ + .6         
        movwf   PHASE3_WORK
        movwf   PHASE3
    return
;---------------
MAMMPhase4                                  ;PHASE 4
        movf    PHASE4_MIN,W                ;MIN  
        cpfslt  PHASE4_WORK                  
        bra     $ + .12                   
        movwf   PHASE4_WORK    
        tstfsz  PHASE4                    
        bra     $ + .6                   
        clrf    PHASE4_WORK         
    return
        movf    PHASE4_MAX,W                ;MAX
        cpfsgt  PHASE4_WORK    
        bra     $ + .6         
        movwf   PHASE4_WORK
        movwf   PHASE4
    return

;------------------------------------------------------------------------------
; Routine:          RGB LED STATES
;------------------------------------------------------------------------------
; Overview:         Sends new states after executing instruction
;------------------------------------------------------------------------------
LEDStates
        rcall   RGB_LEDStatesCh1            ;check if channel 1 state changed
        rcall   RGB_LEDStatesCh2            ;check if channel 2 state changed
        rcall   RGB_LEDStatesCh3            ;check if channel 3 state changed
        rcall   RGB_LEDStatesCh4            ;check if channel 4 state changed
    return

RGB_LEDStatesCh1                            ;LED1
        tstfsz  PROGRAM_NR,W                ;do not send if program is running
    return
        movf    PHASE1_WORK,W               ;check if PHASE=PHASE_WORK (PHASE is stable?)
        cpfseq  PHASE1                                 
    return                                  ;no, so exit
        movf    PHASE1_WORK,W               ;check if PHASE_SAVED=PHASE_WORK (previous state the same?)
        cpfseq  PHASE1_SAVED              
        bra     $ + .4
    return                                  ;yes, so exit
        banksel TXFIFOIN0
        movlw   0x01                        ;"LED1"
        movwf   TXFIFOIN6
        movff   PHASE1_WORK,TXFIFOIN7
        setf    TXFIFOIN8                   ;unused
        movff   Instr1Ch1,TXFIFOIN9         ;instruction of channel
        movff   Instr2Ch1,TXFIFOIN10
        movff   TimerCh1,TXFIFOIN11
        movff   PHASE1_WORK,PHASE1_SAVED    ;save state
        rcall   RGB_SendLEDStates
    return
;----------------
RGB_LEDStatesCh2                            ;LED2
        tstfsz  PROGRAM_NR,W
    return
        movf    PHASE2_WORK,W
        cpfseq  PHASE2                                 
    return                        
        movf    PHASE2_WORK,W  
        cpfseq  PHASE2_SAVED              
        bra     $ + .4
    return  
        banksel TXFIFOIN0
        movlw   0x02                        
        movwf   TXFIFOIN6
        movff   PHASE2_WORK,TXFIFOIN7
        setf    TXFIFOIN8
        movff   Instr1Ch2,TXFIFOIN9 
        movff   Instr2Ch2,TXFIFOIN10
        movff   TimerCh2,TXFIFOIN11
        movff   PHASE2_WORK,PHASE2_SAVED 
        rcall   RGB_SendLEDStates
    return
;----------------
RGB_LEDStatesCh3                            ;LED3
        tstfsz  PROGRAM_NR,W
    return
        movf    PHASE3_WORK,W
        cpfseq  PHASE3                                   
    return                        
        movf    PHASE3_WORK,W  
        cpfseq  PHASE3_SAVED              
        bra     $ + .4
    return 
        banksel TXFIFOIN0
        movlw   0x03                        
        movwf   TXFIFOIN6
        movff   PHASE3_WORK,TXFIFOIN7
        setf    TXFIFOIN8
        movff   Instr1Ch3,TXFIFOIN9      
        movff   Instr2Ch3,TXFIFOIN10
        movff   TimerCh3,TXFIFOIN11
        movff   PHASE3_WORK,PHASE3_SAVED  
        rcall   RGB_SendLEDStates
    return
;----------------
RGB_LEDStatesCh4                            ;MASTER
        ;test if relay changed
        rcall   RGB_PowerRelayTest          ;test power supply relay state
        movf    RELAY_STATE,W               ;previous state the same?
        cpfseq  RELAY_STATE_SAVED              
        bra     $ + .18                     ;no, so send frame
        ;test if phase is stable
        movf    PHASE4_WORK,W
        cpfseq  PHASE4                                  
    return                     
        ;test if phase changed
        movf    PHASE4_WORK,W  
        cpfseq  PHASE4_SAVED              
        bra     $ + .4
    return 
        ;send
        banksel TXFIFOIN0
        movlw   0x04                 
        movwf   TXFIFOIN6
        movff   PHASE4_WORK,TXFIFOIN7
        movff   RELAY_STATE,TXFIFOIN8
        movff   Instr1Ch4,TXFIFOIN9    
        movff   Instr2Ch4,TXFIFOIN10
        movff   TimerCh4,TXFIFOIN11
        movff   PHASE4_WORK,PHASE4_SAVED    ;save sent phase state
        movff   RELAY_STATE,RELAY_STATE_SAVED ;save sent relay state
        rcall   RGB_SendLEDStates
    return
;-------------------------------
RGB_PowerRelayTest                          ;for relay to turn LED power supply
        banksel TXFIFOIN0
        tstfsz  PHASE4_WORK                 ;master off?
        bra     $ + 4                       ;no, so check channels
        bra     RGB_PowerRelayOFF           ;yes
        tstfsz  PHASE1_WORK                 ;test WORK regs
        bra     RGB_PowerRelayON
        tstfsz  PHASE2_WORK
        bra     RGB_PowerRelayON
        tstfsz  PHASE3_WORK
        bra     RGB_PowerRelayON
        tstfsz  PHASE1                      ;test PHASE regs, so do not turn off if any PHASE is waiting to be set
        bra     RGB_PowerRelayON
        tstfsz  PHASE2
        bra     RGB_PowerRelayON
        tstfsz  PHASE3
        bra     RGB_PowerRelayON
        bra     RGB_PowerRelayOFF
RGB_PowerRelayON
        setf    RELAY_STATE
    return
RGB_PowerRelayOFF
        clrf    RELAY_STATE
    return
;-------------------------------
RGB_SendLEDStates
        banksel TXFIFOIN0
        movlw   0x30                        ;set relay frame
        movwf   TXFIFOIN0
        movlw   0x80
        movwf   TXFIFOIN1
        movff   NODENR,TXFIFOIN2            ;node id
        movff   GROUPNR,TXFIFOIN3
        setf    TXFIFOIN4                   ;unused
        setf    TXFIFOIN5                   ;unused
        call    WriteToCanTxFIFO
    ;node can respond to its own message
        bcf     INTCON,GIEL                 ;disable low priority intr to make sure RXFIFO buffer is not overwritten
        call    Copy_TXFIFOIN_RXFIFOIN
        call    WriteToCanRxFIFO
        bsf     INTCON,GIEL                 ;enable back interrupt
    return

;------------------------------------------------------------------------------
; Routine:          RGB PROGRAMS
;------------------------------------------------------------------------------
; Overview:         It manages programs when they are called
;------------------------------------------------------------------------------
RGB_Programs
        movlw   0x00                        ;program 00 (exit programs)?
        xorwf   PROGRAM_NR,W
        bnz     $ + .4
        rcall   RGB_Program00
        movlw   0x01                        ;program 01?
        xorwf   PROGRAM_NR,W
        bnz     $ + .4
        rcall   RGB_Program01
        movlw   0x02                        ;program 02?
        xorwf   PROGRAM_NR,W
        bnz     $ + .4
        rcall   RGB_Program02
    return

;------------------------------------------------------------------------------
; Routine:          PROGRAM 00
;------------------------------------------------------------------------------
; Overview:         STOP & EXIT ALL PROGRAMS
;------------------------------------------------------------------------------
RGB_Program00
        btfss   PROGRAM_CALL,Called         ;was instruction called?
        bra     RGB_Program00_Exit          ;no, so exit
        bcf     PROGRAM_CALL,Called         ;clear flag
        movff   PHASE1_WORK,PHASE1          ;stop changing phases
        movff   PHASE2_WORK,PHASE2
        movff   PHASE3_WORK,PHASE3
        call    DoInstLater_ClearCh1        ;clear any waiting instruction
        call    DoInstLater_ClearCh2
        call    DoInstLater_ClearCh3
RGB_Program00_Exit
    return

;------------------------------------------------------------------------------
; Routine:          BASIC COLOURS
;------------------------------------------------------------------------------
RGB_NULL
        clrf    PHASE1
        clrf    PHASE2
        clrf    PHASE3
    return
RGB_RED
        setf    PHASE1
        clrf    PHASE2
        clrf    PHASE3
    return
RGB_GREEN
        clrf    PHASE1        
        setf    PHASE2
        clrf    PHASE3
    return
RGB_BLUE
        clrf    PHASE1
        clrf    PHASE2
        setf    PHASE3
    return
RGB_WHITE
        setf    PHASE1
        setf    PHASE2
        setf    PHASE3
    return
RGB_RG
        setf    PHASE1
        setf    PHASE2
        clrf    PHASE3
    return
RGB_GB
        clrf    PHASE1
        setf    PHASE2
        setf    PHASE3
    return
RGB_RB
        setf    PHASE1
        clrf    PHASE2
        setf    PHASE3
    return

;------------------------------------------------------------------------------
; Routine:          PROGRAM 01
;------------------------------------------------------------------------------
; Overview:         It changes colors every time when program is called
;------------------------------------------------------------------------------
RGB_Program01
        btfss   PROGRAM_CALL,Called         ;was instruction called?
        bra     RGB_Program01_Exit          ;no, so exit
        bcf     PROGRAM_CALL,Called         ;clear flag
        btfsc   PROGRAM_CALL,FirstCall      ;was it first call?
        clrf    PROGRAM_FLAG1               ;yes, so start from begining     

        bcf     PHASE_FLAG,Ch1ChangeSlow    ;change immediately flag
        bcf     PHASE_FLAG,Ch2ChangeSlow
        bcf     PHASE_FLAG,Ch3ChangeSlow

        movlw   .7                          ;flag must be max 6
        cpfslt  PROGRAM_FLAG1
        bra     RGB_Program01_Exit          
        movf    PROGRAM_FLAG1,W
        call    ComputedGoto
        bra     RGB_Program01_0
        bra     RGB_Program01_1
        bra     RGB_Program01_2
        bra     RGB_Program01_3
        bra     RGB_Program01_4
        bra     RGB_Program01_5
        bra     RGB_Program01_6
RGB_Program01_0
        rcall   RGB_RED
        bra     RGB_Program01_FlagUpdate
RGB_Program01_1
        rcall   RGB_GREEN
        bra     RGB_Program01_FlagUpdate
RGB_Program01_2
        rcall   RGB_BLUE
        bra     RGB_Program01_FlagUpdate
RGB_Program01_3
        rcall   RGB_RG
        bra     RGB_Program01_FlagUpdate
RGB_Program01_4
        rcall   RGB_GB
        bra     RGB_Program01_FlagUpdate
RGB_Program01_5
        rcall   RGB_RB
        bra     RGB_Program01_FlagUpdate
RGB_Program01_6
        rcall   RGB_WHITE
        setf    PROGRAM_FLAG1               ;after all colours go to 0 again
        bra     RGB_Program01_FlagUpdate

RGB_Program01_FlagUpdate
        incf    PROGRAM_FLAG1               ;prepare for next colour
RGB_Program01_Exit
    return

;------------------------------------------------------------------------------
; Routine:          PROGRAM 02
;------------------------------------------------------------------------------
; Overview:         It changes colors smoothly and waits for stop
;------------------------------------------------------------------------------
;PROGRAM_FLAG1 flag meaning
Paused   EQU    0                           ;program paused flag
Reversed EQU    1                           ;program reversed flag

;-------------------------------
;PROGRAM 02 MACRO                           ;macro sets extrem for particular channel and test if extrem is achieved
RGB_Prgr02 macro phase,extr                 ;phase = 1,2,3; extr = 0 (min), 1 (max)
        if(phase==1 && extr==0)             ;PHASE1
        movf    PHASE1_MIN,W                ;set colour
        movwf   PHASE1
        cpfseq  PHASE1_WORK                 ;check if is PHASE_WORK=PHASE_MIN, compare and skip if equal
        bra     RGB_Program02_Exit          ;not equal yet, so exit
        rcall   RGB_Pr02_FlagUpdate         ;equal, so set new Program Flag
        bra     RGB_Program02_Exit
        endif                     
        if(phase==1 && extr==1)
        movf    PHASE1_MAX,W                ;set colour
        movwf   PHASE1
        cpfseq  PHASE1_WORK                 ;check if is PHASE_WORK=PHASE_MAX, compare and skip if equal
        bra     RGB_Program02_Exit          ;not equal yet, so exit
        rcall   RGB_Pr02_FlagUpdate         ;equal, so set new Program Flag
        bra     RGB_Program02_Exit
        endif 
        if(phase==2 && extr==0)             ;PHASE2
        movf    PHASE2_MIN,W
        movwf   PHASE2
        cpfseq  PHASE2_WORK
        bra     RGB_Program02_Exit
        rcall   RGB_Pr02_FlagUpdate
        bra     RGB_Program02_Exit
        endif        
        if(phase==2 && extr==1)
        movf    PHASE2_MAX,W
        movwf   PHASE2
        cpfseq  PHASE2_WORK
        bra     RGB_Program02_Exit
        rcall   RGB_Pr02_FlagUpdate
        bra     RGB_Program02_Exit
        endif 
        if(phase==3 && extr==0)             ;PHASE3
        movf    PHASE3_MIN,W
        movwf   PHASE3
        cpfseq  PHASE3_WORK
        bra     RGB_Program02_Exit
        rcall   RGB_Pr02_FlagUpdate
        bra     RGB_Program02_Exit
        endif        
        if(phase==3 && extr==1)
        movf    PHASE3_MAX,W
        movwf   PHASE3
        cpfseq  PHASE3_WORK
        bra     RGB_Program02_Exit
        rcall   RGB_Pr02_FlagUpdate
        bra     RGB_Program02_Exit
        endif 
    Endm
;---------------
RGB_Pr02_FlagUpdate                         ;program direction
        btfss   PROGRAM_FLAG1,Reversed      ;reversed flag?
        bra     RGB_Pr02_FlagUpdateInc      ;no
        bra     RGB_Pr02_FlagUpdateDec      ;yes
RGB_Pr02_FlagUpdateInc
        incf    PROGRAM_FLAG2
        movlw   .6                          ;program flag 6?
        xorwf   PROGRAM_FLAG2,W
        bnz     RGB_Pr02_FlagUpdateExit     ;no, so exit
        clrf    PROGRAM_FLAG2               ;yes, so set 0
        bra     RGB_Pr02_FlagUpdateExit
RGB_Pr02_FlagUpdateDec       
        decf    PROGRAM_FLAG2
        movlw   .255                        ;program flag 255?
        xorwf   PROGRAM_FLAG2,W        
        bnz     RGB_Pr02_FlagUpdateExit     ;no, so exit
        movlw   .5                          ;yes, so set 5              
        movwf   PROGRAM_FLAG2
RGB_Pr02_FlagUpdateExit      
    return

;------------------------------------------------------------------------------
;PROGRAM START
RGB_Program02
        btfss   PROGRAM_CALL,Called         ;was instruction called again?
        bra     RGB_Program02_Running       ;no, so continue running
        bcf     PROGRAM_CALL,Called         ;clear flag

        btfss   PROGRAM_CALL,FirstCall      ;was it first call?
        bra     RGB_Program02_CalledAgain   ;no
        bra     RGB_Program02_FirstCall     ;yes
RGB_Program02_Exit
    return

;-------------------------------
RGB_Program02_Running                       ;manage running program
        btfss   PROGRAM_FLAG1,Paused        ;is program paused?
        bra     RGB_Program02_Run           ;no, so keep running
        bra     RGB_Program02_Exit          ;yes, so exit program
;---------------
RGB_Program02_FirstCall                     ;start program from begining
        setf    PHASE1_WORK                 ;set red colour immediately
        clrf    PHASE2_WORK
        clrf    PHASE3_WORK
        setf    PHASE1
        clrf    PHASE2
        clrf    PHASE3
        bsf     PHASE_FLAG,Ch1ChangeSlow    ;change immediately flag
        bsf     PHASE_FLAG,Ch2ChangeSlow
        bsf     PHASE_FLAG,Ch3ChangeSlow    
        bcf     PROGRAM_FLAG1,Reversed      ;clear 'reversed program' flag
        clrf    PROGRAM_FLAG2               ;start changing from red colour
        bra     RGB_Program02_Run           ;run program from red colour
;---------------
RGB_Program02_CalledAgain                   ;program was called again
        btfss   PROGRAM_FLAG1,Paused        ;is program paused?
        bra     RGB_Program02_Pause         ;no, so pause program now
        bra     RGB_Program02_Run           ;yes, so continue to run program
;---------------
RGB_Program02_Pause                         ;pause program
        movff   PHASE1_WORK,PHASE1          ;stop changing phases
        movff   PHASE2_WORK,PHASE2
        movff   PHASE3_WORK,PHASE3
        btg     PROGRAM_FLAG1,Reversed      ;reverse program direction
        bsf     PROGRAM_FLAG1,Paused        ;set 'program paused' flag
        bra     RGB_Program02_Exit
;---------------
RGB_Program02_Run                           ;run program from last position
        bcf     PROGRAM_FLAG1,Paused        ;clear 'program paused' flag
        btfsc   PROGRAM_FLAG1,Reversed      ;reversed direction?
        bra     RGB_Program02_Reversed      ;yes
        bra     RGB_Program02_Normal        ;no

;-------------------------------
RGB_Program02_Normal                        ;runs program normal direction
        movlw   .6                          ;flag must be max 5
        cpfslt  PROGRAM_FLAG2
        bra     RGB_Program02_Exit          
        movf    PROGRAM_FLAG2,W
        call    ComputedGoto
        bra     Flag0
        bra     Flag1
        bra     Flag2
        bra     Flag3
        bra     Flag4
        bra     Flag5
Flag0   RGB_Prgr02 1, 1                     ;macro phase1 reached max? ExitProgram02 if not
Flag1   RGB_Prgr02 3, 0                     ;phase3 reached min?
Flag2   RGB_Prgr02 2, 1                     ;phase2 reached max?
Flag3   RGB_Prgr02 1, 0                     ;phase1 reached min?
Flag4   RGB_Prgr02 3, 1                     ;phase3 reached max?
Flag5   RGB_Prgr02 2, 0                     ;phase2 reached min?
;-------------------------------
RGB_Program02_Reversed                      ;runs pogram reversed direction
        movlw   .6                          ;flag must be max 5
        cpfslt  PROGRAM_FLAG2
        bra     RGB_Program02_Exit          
        movf    PROGRAM_FLAG2,W
        call    ComputedGoto
        bra     Flag0R
        bra     Flag1R
        bra     Flag2R
        bra     Flag3R
        bra     Flag4R
        bra     Flag5R
Flag0R  RGB_Prgr02 1, 0                     ;macro phase1 reached min? ExitProgram02 if not
Flag1R  RGB_Prgr02 3, 1                     ;phase3 reached max?
Flag2R  RGB_Prgr02 2, 0                     ;phase2 reached min?
Flag3R  RGB_Prgr02 1, 1                     ;phase1 reached max?
Flag4R  RGB_Prgr02 3, 0                     ;phase3 reached min?
Flag5R  RGB_Prgr02 2, 1                     ;phase2 reached max?


;------------------------------------------------------------------------------
; Routine:          SAVE STATES TO EEPROM
;------------------------------------------------------------------------------
; Overview:         It saves current relay states into EEPROM memory
;------------------------------------------------------------------------------
RGB_SaveSateToEeprom            
        banksel EEPROMTIMER
        ;wait 6s before saving
        tstfsz  EEPROMTIMER
        bra     $ + .4
        bra     ExitSaveSateToEeprom
        decfsz  EEPROMTIMER
        bra     ExitSaveSateToEeprom
        ;save to eeprom
        clrf    EEADRH                      ;point at high address
        ;PHASE1_WORK
        movlw   CONFIG9 & 0xFF              ;PHASE1_WORK
        movwf   EEADR
        movf    PHASE1_WORK,W               ;set data for EepromSaveWREG routine
        call    EepromSaveWREG
        ;PHASE2_WORK
        movlw   CONFIG11 & 0xFF             ;PHASE2_WORK
        movwf   EEADR
        movf    PHASE2_WORK,W
        call    EepromSaveWREG
        ;PHASE3_WORK
        movlw   CONFIG13 & 0xFF             ;PHASE3_WORK
        movwf   EEADR
        movf    PHASE3_WORK,W
        call    EepromSaveWREG
        ;PHASE4_WORK
        movlw   CONFIG15 & 0xFF             ;PHASE4_WORK
        movwf   EEADR
        movf    PHASE4_WORK,W
        call    EepromSaveWREG
ExitSaveSateToEeprom
    return

;==============================================================================
;===  END  OF  PROGRAM  =======================================================
;==============================================================================
    END