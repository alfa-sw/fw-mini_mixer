# MPLAB IDE generated this makefile for use with GNU make.
# Project: Actuators.mcp
# Date: Fri Mar 06 17:12:04 2015

AS = pic30-as.exe
CC = pic30-gcc.exe
LD = pic30-ld.exe
AR = pic30-ar.exe
HX = pic30-bin2hex.exe
RM = rm

out/Actuators.hex : out/Actuators.cof
	$(HX) "out\Actuators.cof"

out/Actuators.cof : objs/init.o objs/main.o objs/interrupts.o objs/traps.o objs/input.o objs/timerMg.o objs/stepper.o objs/const.o objs/ram.o objs/serialCom.o objs/color.o
	$(CC) -mcpu=33FJ32MC204 "objs\init.o" "objs\main.o" "objs\interrupts.o" "objs\traps.o" "objs\input.o" "objs\timerMg.o" "objs\stepper.o" "objs\const.o" "objs\ram.o" "objs\serialCom.o" "objs\color.o" -o"D:\Source_Code\PIC\Program\Alfa_Color_Tester\colorActuators\out\Actuators.cof" -Wl,-L"C:\Program Files\Microchip\MPLAB C30\lib",-Tp33FJ32MC204.gld,--defsym=__MPLAB_BUILD=1,-Map="D:\Source_Code\PIC\Program\Alfa_Color_Tester\colorActuators\out\Actuators.map",--report-mem

objs/init.o : const.h ram.h serialCom.h color.h timermg.h userparams.h RTDMUSER.h c:/program\ files\ (x86)/microchip/mplab\ c30/support/dsPIC33F/h/p33FJ32MC204.h c:/program\ files\ (x86)/microchip/mplab\ c30/support/dsPIC33F/h/p33Fxxxx.h c:/program\ files\ (x86)/microchip/mplab\ c30/support/peripheral_30F_24H_33F/UART.h c:/program\ files\ (x86)/microchip/mplab\ c30/include/string.h RTDM.h stepper.h debug.h init.h c:/program\ files\ (x86)/microchip/mplab\ c30/include/math.h c:/program\ files\ (x86)/microchip/mplab\ c30/include/yvals.h c:/program\ files\ (x86)/microchip/mplab\ c30/include/stdlib.h c:/program\ files\ (x86)/microchip/mplab\ c30/support/generic/h/dsp.h c:/program\ files\ (x86)/microchip/mplab\ c30/support/dsPIC33F/h/p33FJ32MC204.h stepper.h macro.h typedef.h init.c
	$(CC) -mcpu=33FJ32MC204 -x c -c "init.c" -o"D:\Source_Code\PIC\Program\Alfa_Color_Tester\colorActuators\objs\init.o" -g -Wall -mlarge-code -mlarge-data -mlarge-scalar -mconst-in-code -Os

objs/main.o : TIMERMG.h INPUT.h color.h serialCom.h userparams.h macro.h RTDMUSER.h c:/program\ files\ (x86)/microchip/mplab\ c30/support/dsPIC33F/h/p33FJ32MC204.h c:/program\ files\ (x86)/microchip/mplab\ c30/support/dsPIC33F/h/p33Fxxxx.h c:/program\ files\ (x86)/microchip/mplab\ c30/support/peripheral_30F_24H_33F/UART.h c:/program\ files\ (x86)/microchip/mplab\ c30/include/string.h RTDM.h stepper.h debug.h init.h c:/program\ files\ (x86)/microchip/mplab\ c30/include/math.h c:/program\ files\ (x86)/microchip/mplab\ c30/include/yvals.h c:/program\ files\ (x86)/microchip/mplab\ c30/include/stdlib.h c:/program\ files\ (x86)/microchip/mplab\ c30/support/generic/h/dsp.h c:/program\ files\ (x86)/microchip/mplab\ c30/support/dsPIC33F/h/p33FJ32MC204.h stepper.h main.c
	$(CC) -mcpu=33FJ32MC204 -x c -c "main.c" -o"D:\Source_Code\PIC\Program\Alfa_Color_Tester\colorActuators\objs\main.o" -g -Wall -mlarge-code -mlarge-data -mlarge-scalar -mconst-in-code -Os

objs/interrupts.o : serialCom.h color.h typedef.h ram.h timermg.h userparams.h macro.h RTDMUSER.h c:/program\ files\ (x86)/microchip/mplab\ c30/support/dsPIC33F/h/p33FJ32MC204.h c:/program\ files\ (x86)/microchip/mplab\ c30/support/dsPIC33F/h/p33Fxxxx.h c:/program\ files\ (x86)/microchip/mplab\ c30/support/peripheral_30F_24H_33F/UART.h c:/program\ files\ (x86)/microchip/mplab\ c30/include/string.h RTDM.h stepper.h debug.h init.h c:/program\ files\ (x86)/microchip/mplab\ c30/include/math.h c:/program\ files\ (x86)/microchip/mplab\ c30/include/yvals.h c:/program\ files\ (x86)/microchip/mplab\ c30/include/stdlib.h c:/program\ files\ (x86)/microchip/mplab\ c30/support/generic/h/dsp.h c:/program\ files\ (x86)/microchip/mplab\ c30/support/dsPIC33F/h/p33FJ32MC204.h stepper.h interrupts.c
	$(CC) -mcpu=33FJ32MC204 -x c -c "interrupts.c" -o"D:\Source_Code\PIC\Program\Alfa_Color_Tester\colorActuators\objs\Interrupts.o" -g -Wall -mlarge-code -mlarge-data -mlarge-scalar -mconst-in-code -Os

objs/traps.o : c:/program\ files\ (x86)/microchip/mplab\ c30/support/dsPIC33F/h/p33FJ32MC204.h c:/program\ files\ (x86)/microchip/mplab\ c30/support/dsPIC33F/h/p33fxxxx.h traps.c
	$(CC) -mcpu=33FJ32MC204 -x c -c "traps.c" -o"D:\Source_Code\PIC\Program\Alfa_Color_Tester\colorActuators\objs\traps.o" -g -Wall -mlarge-code -mlarge-data -mlarge-scalar -mconst-in-code -Os

objs/input.o : timermg.h input.h serialCom.h color.h typedef.h ram.h userparams.h macro.h RTDMUSER.h c:/program\ files\ (x86)/microchip/mplab\ c30/support/dsPIC33F/h/p33FJ32MC204.h c:/program\ files\ (x86)/microchip/mplab\ c30/support/dsPIC33F/h/p33Fxxxx.h c:/program\ files\ (x86)/microchip/mplab\ c30/support/peripheral_30F_24H_33F/UART.h c:/program\ files\ (x86)/microchip/mplab\ c30/include/string.h RTDM.h stepper.h debug.h init.h c:/program\ files\ (x86)/microchip/mplab\ c30/include/math.h c:/program\ files\ (x86)/microchip/mplab\ c30/include/yvals.h c:/program\ files\ (x86)/microchip/mplab\ c30/include/stdlib.h c:/program\ files\ (x86)/microchip/mplab\ c30/support/generic/h/dsp.h c:/program\ files\ (x86)/microchip/mplab\ c30/support/dsPIC33F/h/p33FJ32MC204.h stepper.h input.c
	$(CC) -mcpu=33FJ32MC204 -x c -c "input.c" -o"D:\Source_Code\PIC\Program\Alfa_Color_Tester\colorActuators\objs\input.o" -g -Wall -mlarge-code -mlarge-data -mlarge-scalar -mconst-in-code -Os

objs/timerMg.o : TimerMg.h MACRO.H timerMg.c
	$(CC) -mcpu=33FJ32MC204 -x c -c "timerMg.c" -o"D:\Source_Code\PIC\Program\Alfa_Color_Tester\colorActuators\objs\timerMg.o" -g -Wall -mlarge-code -mlarge-data -mlarge-scalar -mconst-in-code -Os

objs/stepper.o : typedef.h ram.h serialCom.h color.h timermg.H INIT.h INPUT.H tables.h MACRO.H userparams.h macro.h RTDMUSER.h c:/program\ files\ (x86)/microchip/mplab\ c30/support/dsPIC33F/h/p33FJ32MC204.h c:/program\ files\ (x86)/microchip/mplab\ c30/support/dsPIC33F/h/p33Fxxxx.h c:/program\ files\ (x86)/microchip/mplab\ c30/support/peripheral_30F_24H_33F/UART.h c:/program\ files\ (x86)/microchip/mplab\ c30/include/string.h RTDM.h stepper.h debug.h init.h c:/program\ files\ (x86)/microchip/mplab\ c30/include/math.h c:/program\ files\ (x86)/microchip/mplab\ c30/include/yvals.h c:/program\ files\ (x86)/microchip/mplab\ c30/include/stdlib.h c:/program\ files\ (x86)/microchip/mplab\ c30/support/generic/h/dsp.h c:/program\ files\ (x86)/microchip/mplab\ c30/support/dsPIC33F/h/p33FJ32MC204.h stepper.h stepper.c
	$(CC) -mcpu=33FJ32MC204 -x c -c "stepper.c" -o"D:\Source_Code\PIC\Program\Alfa_Color_Tester\colorActuators\objs\stepper.o" -g -Wall -mlarge-code -mlarge-data -mlarge-scalar -mconst-in-code -Os

objs/const.o : const.h serialCom.h color.h ram.h macro.h typedef.h const.c
	$(CC) -mcpu=33FJ32MC204 -x c -c "const.c" -o"D:\Source_Code\PIC\Program\Alfa_Color_Tester\colorActuators\objs\const.o" -g -Wall -mlarge-code -mlarge-data -mlarge-scalar -mconst-in-code -Os

objs/ram.o : serialCom.h color.h RAM.H macro.h typedef.h ram.c
	$(CC) -mcpu=33FJ32MC204 -x c -c "ram.c" -o"D:\Source_Code\PIC\Program\Alfa_Color_Tester\colorActuators\objs\ram.o" -g -Wall -mlarge-code -mlarge-data -mlarge-scalar -mconst-in-code -Os

objs/serialCom.o : c:/program\ files\ (x86)/microchip/mplab\ c30/include/math.h c:/program\ files\ (x86)/microchip/mplab\ c30/include/stdlib.h c:/program\ files\ (x86)/microchip/mplab\ c30/include/yvals.h c:/program\ files\ (x86)/microchip/mplab\ c30/include/string.h c:/program\ files\ (x86)/microchip/mplab\ c30/support/dsPIC33F/h/p33FJ32MC204.h c:/program\ files\ (x86)/microchip/mplab\ c30/support/dsPIC33F/h/p33Fxxxx.h c:/program\ files\ (x86)/microchip/mplab\ c30/support/peripheral_30F_24H_33F/UART.h timerMg.h const.h serialCom.h color.h ram.h typedef.h macro.h serialCom.c
	$(CC) -mcpu=33FJ32MC204 -x c -c "serialCom.c" -o"D:\Source_Code\PIC\Program\Alfa_Color_Tester\colorActuators\objs\serialCom.o" -g -Wall -mlarge-code -mlarge-data -mlarge-scalar -mconst-in-code -Os

objs/color.o : c:/program\ files\ (x86)/microchip/mplab\ c30/include/stdio.h timermg.h input.h userparams.h RTDMUSER.h c:/program\ files\ (x86)/microchip/mplab\ c30/support/dsPIC33F/h/p33FJ32MC204.h c:/program\ files\ (x86)/microchip/mplab\ c30/support/dsPIC33F/h/p33Fxxxx.h c:/program\ files\ (x86)/microchip/mplab\ c30/support/peripheral_30F_24H_33F/UART.h c:/program\ files\ (x86)/microchip/mplab\ c30/include/string.h RTDM.h stepper.h debug.h init.h c:/program\ files\ (x86)/microchip/mplab\ c30/include/math.h c:/program\ files\ (x86)/microchip/mplab\ c30/include/yvals.h c:/program\ files\ (x86)/microchip/mplab\ c30/include/stdlib.h c:/program\ files\ (x86)/microchip/mplab\ c30/support/generic/h/dsp.h c:/program\ files\ (x86)/microchip/mplab\ c30/support/dsPIC33F/h/p33FJ32MC204.h stepper.h const.h serialCom.h color.h ram.h macro.h typedef.h color.c
	$(CC) -mcpu=33FJ32MC204 -x c -c "color.c" -o"D:\Source_Code\PIC\Program\Alfa_Color_Tester\colorActuators\objs\color.o" -g -Wall -mlarge-code -mlarge-data -mlarge-scalar -mconst-in-code -Os

clean : 
	$(RM) "objs\init.o" "objs\main.o" "objs\interrupts.o" "objs\traps.o" "objs\input.o" "objs\timerMg.o" "objs\stepper.o" "objs\const.o" "objs\ram.o" "objs\serialCom.o" "objs\color.o" "out\Actuators.cof" "out\Actuators.hex"

