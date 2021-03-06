EESchema Schematic File Version 2
LIBS:pcb-rescue
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:atmel2
LIBS:w_relay
LIBS:bjt
LIBS:relay_ramway_ds903c
LIBS:scc
LIBS:pcb-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "Code Switch"
Date "2017-03-26"
Rev "1.1"
Comp "Kamworks"
Comment1 "Leon Hiemstra"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L ATSAMD11C14A-SS U2
U 1 1 58A8CEC6
P 4700 3400
F 0 "U2" H 4700 4100 50  0000 C CNN
F 1 "ATSAMD11C14A-SS" H 4700 4000 50  0000 C CNN
F 2 "so:SOIC-14_3.9x8.7mm_Pitch1.27mm" H 4700 2700 50  0001 C CNN
F 3 "http://www.atmel.com/Images/Atmel-42363-SAM-D11_Datasheet.pdf" H 4700 2600 50  0001 C CNN
F 4 "MCU ATMEL ATSAMD11C14A-SSU" H 4700 2800 50  0001 C CNN "BOM"
F 5 "ATSAMD11C14A-SSUT" H 4700 3400 60  0001 C CNN "Mfg_code"
	1    4700 3400
	1    0    0    -1  
$EndComp
$Comp
L Polyfuse F2
U 1 1 58A8D1C3
P 9690 2190
F 0 "F2" V 9590 2190 50  0000 C CNN
F 1 "Polyfuse 1.1A-hold 2.2A-trip" V 9790 2190 50  0001 C CNN
F 2 "Resistors_SMD:R_1812" H 9740 1990 50  0001 L CNN
F 3 "" H 9690 2190 50  0001 C CNN
F 4 "0ZCG0110BF2B" V 9690 2190 60  0001 C CNN "Mfg_code"
	1    9690 2190
	0    1    1    0   
$EndComp
$Comp
L CONN_01X03 P4
U 1 1 58A8D267
P 6880 1430
F 0 "P4" H 6880 1630 50  0000 C CNN
F 1 "VS1838b" V 6980 1430 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03_Pitch2.54mm" H 6880 1430 50  0001 C CNN
F 3 "https://www.elecrow.com/download/Infrared%20receiver%20vs1838b.pdf" H 6880 1430 50  0001 C CNN
F 4 "VS1838b" H 6880 1430 60  0001 C CNN "Mfg_code"
	1    6880 1430
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X02 P5
U 1 1 58A8D2D0
P 6930 2760
F 0 "P5" H 6930 2910 50  0000 C CNN
F 1 "PAYLED" V 7030 2760 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02_Pitch2.54mm" H 6930 2760 50  0001 C CNN
F 3 "" H 6930 2760 50  0000 C CNN
F 4 "M20-9990246" H 6930 2760 60  0001 C CNN "Mfg_code"
F 5 "add: LED red: Mfg_code:WP7113ID" H 6930 2760 60  0001 C CNN "Note"
	1    6930 2760
	1    0    0    -1  
$EndComp
$Comp
L CONN_02X05 P3
U 1 1 58A8D335
P 4240 1940
F 0 "P3" H 4240 2240 50  0000 C CNN
F 1 "Atmel-ICE" H 4240 1640 50  0000 C CNN
F 2 "myconnectors:Atmel_ICE" H 4240 740 50  0001 C CNN
F 3 "" H 4240 740 50  0000 C CNN
F 4 "3220-10-0100-00" H 4240 1940 60  0001 C CNN "Mfg_code"
	1    4240 1940
	1    0    0    -1  
$EndComp
$Comp
L D_ALT D3
U 1 1 58A8D600
P 8680 3960
F 0 "D3" H 8780 3860 50  0000 C CNN
F 1 "S1DB" H 8580 3860 50  0000 C CNN
F 2 "Diodes_SMD:D_SMB_Handsoldering" H 8680 3960 50  0001 C CNN
F 3 "" H 8680 3960 50  0000 C CNN
F 4 "S1DB-13-F" H 8680 3960 60  0001 C CNN "Mfg_code"
	1    8680 3960
	0    1    1    0   
$EndComp
$Comp
L D_ALT D4
U 1 1 58A8D6D5
P 8960 3960
F 0 "D4" H 9060 4060 50  0000 C CNN
F 1 "S1DB" H 8860 4060 50  0000 C CNN
F 2 "Diodes_SMD:D_SMB_Handsoldering" H 8960 3960 50  0001 C CNN
F 3 "" H 8960 3960 50  0000 C CNN
F 4 "S1DB-13-F" H 8960 3960 60  0001 C CNN "Mfg_code"
	1    8960 3960
	0    1    1    0   
$EndComp
$Comp
L C C1
U 1 1 58A8D77D
P 3700 5850
F 0 "C1" H 3725 5950 50  0000 L CNN
F 1 "4u7/25V" H 3725 5750 50  0000 L CNN
F 2 "capacitor:C_0805_reflow" H 3738 5700 50  0001 C CNN
F 3 "" H 3700 5850 50  0000 C CNN
F 4 "CL21A475KAQNNNG" H 3700 5850 60  0001 C CNN "Mfg_code"
	1    3700 5850
	1    0    0    -1  
$EndComp
$Comp
L Speaker_Crystal LS1
U 1 1 58A8D823
P 6860 1920
F 0 "LS1" H 7160 1970 50  0000 R CNN
F 1 "piezo" H 7210 1880 50  0000 R CNN
F 2 "buzzer:MagneticBuzzer_StarMicronics_HMB-06_HMB-12" H 6825 1870 50  0001 C CNN
F 3 "" H 6825 1870 50  0000 C CNN
F 4 "PT-1340P-PQ" H 6860 1920 60  0001 C CNN "Mfg_code"
	1    6860 1920
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X04 P1
U 1 1 58A8DFA7
P 1230 3150
F 0 "P1" H 1230 3400 50  0000 C CNN
F 1 "serial" V 1330 3150 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x04_Pitch2.54mm" H 1230 3150 50  0001 C CNN
F 3 "" H 1230 3150 50  0000 C CNN
F 4 "61300411121" H 1230 3150 60  0001 C CNN "Mfg_code"
	1    1230 3150
	-1   0    0    -1  
$EndComp
$Comp
L +12V #PWR01
U 1 1 58AB53F0
P 8680 3260
F 0 "#PWR01" H 8680 3110 50  0001 C CNN
F 1 "+12V" H 8680 3400 50  0000 C CNN
F 2 "" H 8680 3260 50  0000 C CNN
F 3 "" H 8680 3260 50  0000 C CNN
	1    8680 3260
	1    0    0    -1  
$EndComp
$Comp
L RAMWAY_DS903C-90A_12V_D RLY1
U 1 1 58AB8970
P 9460 3460
F 0 "RLY1" V 9520 3670 60  0000 C CNN
F 1 "RAMWAY_DS903C-90A_12V_D" H 9460 3175 60  0001 C CNN
F 2 "myrelays:RAMWAY_DS903C-90A_12V_D" H 9460 3460 60  0001 C CNN
F 3 "http://www.rwrelay.cn/tradeDetails.aspx?tid=213&&lang=1&&name=94" H 9460 3460 60  0001 C CNN
F 4 "DS903C-90A_12V_D" H 9460 3460 60  0001 C CNN "Mfg_code"
	1    9460 3460
	0    -1   -1   0   
$EndComp
$Comp
L CONN_01X01 P8
U 1 1 58ABA3CC
P 10100 3260
F 0 "P8" H 10100 3360 50  0000 C CNN
F 1 "(faston male 6.35mm)" V 10200 3260 50  0001 C CNN
F 2 "myconnectors:faston_m_63" H 10100 3260 50  0001 C CNN
F 3 "http://www.digikey.nl/product-detail/en/te-connectivity-amp-connectors/1217861-1/A100452TR-ND/1148769" H 10100 3260 50  0001 C CNN
F 4 "1217861-1" H 10100 3260 60  0001 C CNN "Mfg_code"
	1    10100 3260
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X01 P9
U 1 1 58ABACEF
P 10100 3660
F 0 "P9" H 10100 3760 50  0000 C CNN
F 1 "(faston male 6.35mm)" V 10200 3660 50  0001 C CNN
F 2 "myconnectors:faston_m_63" H 10100 3660 50  0001 C CNN
F 3 "http://www.digikey.nl/product-detail/en/te-connectivity-amp-connectors/1217861-1/A100452TR-ND/1148769" H 10100 3660 50  0001 C CNN
F 4 "1217861-1" H 10100 3660 60  0001 C CNN "Mfg_code"
	1    10100 3660
	1    0    0    -1  
$EndComp
$Comp
L R R6
U 1 1 58ABB16E
P 9420 4890
F 0 "R6" V 9500 4890 50  0000 C CNN
F 1 "1k" V 9420 4890 50  0000 C CNN
F 2 "resistor:R_0805_reflow" V 9350 4890 50  0001 C CNN
F 3 "" H 9420 4890 50  0000 C CNN
F 4 "RMCF0805FT1K00" V 9420 4890 60  0001 C CNN "Mfg_code"
	1    9420 4890
	0    1    1    0   
$EndComp
$Comp
L GND #PWR02
U 1 1 58AC19D0
P 8820 5190
F 0 "#PWR02" H 8820 4940 50  0001 C CNN
F 1 "GND" H 8820 5040 50  0000 C CNN
F 2 "" H 8820 5190 50  0000 C CNN
F 3 "" H 8820 5190 50  0000 C CNN
	1    8820 5190
	1    0    0    -1  
$EndComp
Text Label 7640 4890 0    60   ~ 0
RELAY_ON
Text Label 9630 4890 0    60   ~ 0
RELAY_OFF
$Comp
L +12V #PWR03
U 1 1 58AB6B94
P 3260 5110
F 0 "#PWR03" H 3260 4960 50  0001 C CNN
F 1 "+12V" H 3260 5250 50  0000 C CNN
F 2 "" H 3260 5110 50  0000 C CNN
F 3 "" H 3260 5110 50  0000 C CNN
	1    3260 5110
	1    0    0    -1  
$EndComp
$Comp
L D_ALT D2
U 1 1 58AB6DD1
P 3260 5470
F 0 "D2" H 3360 5370 50  0000 C CNN
F 1 "S1DB" H 3160 5370 50  0000 C CNN
F 2 "Diodes_SMD:D_SMB_Handsoldering" H 3260 5470 50  0001 C CNN
F 3 "" H 3260 5470 50  0000 C CNN
F 4 "S1DB-13-F" H 3260 5470 60  0001 C CNN "Mfg_code"
	1    3260 5470
	0    -1   -1   0   
$EndComp
$Comp
L NCP1117ST33T3G_MountingTab U1
U 1 1 58ABA294
P 4300 5700
F 0 "U1" H 4300 5925 50  0000 C CNN
F 1 "NCP1117ST33T3G" H 4300 5850 50  0000 C CNN
F 2 "TO_SOT_Packages_SMD:SOT-223" H 4350 5450 50  0001 L CNN
F 3 "" H 4300 5700 50  0000 C CNN
F 4 "NCP1117ST33T3G" H 4300 5700 60  0001 C CNN "Mfg_code"
	1    4300 5700
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR04
U 1 1 58ABB54A
P 4300 6200
F 0 "#PWR04" H 4300 5950 50  0001 C CNN
F 1 "GND" H 4300 6050 50  0000 C CNN
F 2 "" H 4300 6200 50  0000 C CNN
F 3 "" H 4300 6200 50  0000 C CNN
	1    4300 6200
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR05
U 1 1 58ABBC1F
P 5090 5600
F 0 "#PWR05" H 5090 5450 50  0001 C CNN
F 1 "+3.3V" H 5090 5740 50  0000 C CNN
F 2 "" H 5090 5600 50  0000 C CNN
F 3 "" H 5090 5600 50  0000 C CNN
	1    5090 5600
	1    0    0    -1  
$EndComp
$Comp
L R R3
U 1 1 58ABD80D
P 3740 3500
F 0 "R3" V 3820 3500 50  0000 C CNN
F 1 "10k" V 3740 3500 50  0000 C CNN
F 2 "resistor:R_0805_reflow" V 3670 3500 50  0001 C CNN
F 3 "" H 3740 3500 50  0000 C CNN
F 4 "RMCF0805FT10K0" V 3740 3500 60  0001 C CNN "Mfg_code"
	1    3740 3500
	0    -1   -1   0   
$EndComp
$Comp
L +3.3V #PWR06
U 1 1 58ABE0FE
P 3380 3500
F 0 "#PWR06" H 3380 3350 50  0001 C CNN
F 1 "+3.3V" H 3380 3640 50  0000 C CNN
F 2 "" H 3380 3500 50  0000 C CNN
F 3 "" H 3380 3500 50  0000 C CNN
	1    3380 3500
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR07
U 1 1 58ABEAFF
P 3890 2850
F 0 "#PWR07" H 3890 2700 50  0001 C CNN
F 1 "+3.3V" H 3890 2990 50  0000 C CNN
F 2 "" H 3890 2850 50  0000 C CNN
F 3 "" H 3890 2850 50  0000 C CNN
	1    3890 2850
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR08
U 1 1 58ABED6A
P 3880 3860
F 0 "#PWR08" H 3880 3610 50  0001 C CNN
F 1 "GND" H 3880 3710 50  0000 C CNN
F 2 "" H 3880 3860 50  0000 C CNN
F 3 "" H 3880 3860 50  0000 C CNN
	1    3880 3860
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR09
U 1 1 58ABF093
P 4400 4120
F 0 "#PWR09" H 4400 3870 50  0001 C CNN
F 1 "GND" H 4400 3970 50  0000 C CNN
F 2 "" H 4400 4120 50  0000 C CNN
F 3 "" H 4400 4120 50  0000 C CNN
	1    4400 4120
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR010
U 1 1 58ABF110
P 4960 4120
F 0 "#PWR010" H 4960 3970 50  0001 C CNN
F 1 "+3.3V" H 4960 4260 50  0000 C CNN
F 2 "" H 4960 4120 50  0000 C CNN
F 3 "" H 4960 4120 50  0000 C CNN
	1    4960 4120
	1    0    0    -1  
$EndComp
$Comp
L C C2
U 1 1 58ABF2E4
P 4710 4120
F 0 "C2" H 4735 4220 50  0000 L CNN
F 1 "100n" H 4735 4020 50  0000 L CNN
F 2 "capacitor:C_0805_reflow" H 4748 3970 50  0001 C CNN
F 3 "" H 4710 4120 50  0000 C CNN
F 4 "C0805C104K5RACTU" H 4710 4120 60  0001 C CNN "Mfg_code"
	1    4710 4120
	0    1    1    0   
$EndComp
$Comp
L C C3
U 1 1 58ACB474
P 4830 5850
F 0 "C3" H 4855 5950 50  0000 L CNN
F 1 "4u7/25V" H 4855 5750 50  0000 L CNN
F 2 "capacitor:C_0805_reflow" H 4868 5700 50  0001 C CNN
F 3 "" H 4830 5850 50  0000 C CNN
F 4 "CL21A475KAQNNNG" H 4830 5850 60  0001 C CNN "Mfg_code"
	1    4830 5850
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR011
U 1 1 58ACC3FE
P 3990 1640
F 0 "#PWR011" H 3990 1490 50  0001 C CNN
F 1 "+3.3V" H 3990 1780 50  0000 C CNN
F 2 "" H 3990 1640 50  0000 C CNN
F 3 "" H 3990 1640 50  0000 C CNN
	1    3990 1640
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR012
U 1 1 58ACCD15
P 3810 1840
F 0 "#PWR012" H 3810 1590 50  0001 C CNN
F 1 "GND" H 3810 1690 50  0000 C CNN
F 2 "" H 3810 1840 50  0000 C CNN
F 3 "" H 3810 1840 50  0000 C CNN
	1    3810 1840
	1    0    0    -1  
$EndComp
Text Label 4560 2140 0    60   ~ 0
nRESET
Text Label 4570 1840 0    60   ~ 0
SWCLK
Text Label 4580 1740 0    60   ~ 0
SWDIO
Text Label 3550 3660 0    60   ~ 0
nRESET
Text Label 3540 3200 0    60   ~ 0
SWCLK
Text Label 3540 3100 0    60   ~ 0
SWDIO
$Comp
L C C5
U 1 1 58ACEFEA
P 6460 3450
F 0 "C5" V 6510 3510 50  0000 L CNN
F 1 "15pF" V 6400 3520 50  0000 L CNN
F 2 "capacitor:C_0805_reflow" H 6498 3300 50  0001 C CNN
F 3 "" H 6460 3450 50  0000 C CNN
F 4 "CL21C150JBANNNC" H 6460 3450 60  0001 C CNN "Mfg_code"
	1    6460 3450
	0    1    1    0   
$EndComp
$Comp
L GND #PWR013
U 1 1 58AD018C
P 6830 3290
F 0 "#PWR013" H 6830 3040 50  0001 C CNN
F 1 "GND" H 6830 3140 50  0000 C CNN
F 2 "" H 6830 3290 50  0000 C CNN
F 3 "" H 6830 3290 50  0000 C CNN
	1    6830 3290
	1    0    0    -1  
$EndComp
$Comp
L R R2
U 1 1 58AD1621
P 1840 3200
F 0 "R2" V 1920 3200 50  0000 C CNN
F 1 "150" V 1840 3200 50  0000 C CNN
F 2 "resistor:R_0805_reflow" V 1770 3200 50  0001 C CNN
F 3 "" H 1840 3200 50  0000 C CNN
F 4 "RMCF0805FT150R" V 1840 3200 60  0001 C CNN "Mfg_code"
	1    1840 3200
	0    1    1    0   
$EndComp
$Comp
L R R1
U 1 1 58AD1CCE
P 1840 3100
F 0 "R1" V 1760 3100 50  0000 C CNN
F 1 "150" V 1840 3100 50  0000 C CNN
F 2 "resistor:R_0805_reflow" V 1770 3100 50  0001 C CNN
F 3 "" H 1840 3100 50  0000 C CNN
F 4 "RMCF0805FT150R" V 1840 3100 60  0001 C CNN "Mfg_code"
	1    1840 3100
	0    1    1    0   
$EndComp
Text Label 1450 3100 0    60   ~ 0
RXD
Text Label 1460 3200 0    60   ~ 0
TXD
$Comp
L +3.3V #PWR014
U 1 1 58AD3026
P 1430 2840
F 0 "#PWR014" H 1430 2690 50  0001 C CNN
F 1 "+3.3V" H 1430 2980 50  0000 C CNN
F 2 "" H 1430 2840 50  0000 C CNN
F 3 "" H 1430 2840 50  0000 C CNN
	1    1430 2840
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR015
U 1 1 58AD3281
P 1430 3460
F 0 "#PWR015" H 1430 3210 50  0001 C CNN
F 1 "GND" H 1430 3310 50  0000 C CNN
F 2 "" H 1430 3460 50  0000 C CNN
F 3 "" H 1430 3460 50  0000 C CNN
	1    1430 3460
	1    0    0    -1  
$EndComp
$Comp
L R R4
U 1 1 58AD3D80
P 6460 2810
F 0 "R4" V 6380 2810 50  0000 C CNN
F 1 "150" V 6460 2810 50  0000 C CNN
F 2 "resistor:R_0805_reflow" V 6390 2810 50  0001 C CNN
F 3 "" H 6460 2810 50  0000 C CNN
F 4 "RMCF0805FT150R" V 6460 2810 60  0001 C CNN "Mfg_code"
	1    6460 2810
	0    1    1    0   
$EndComp
$Comp
L +3.3V #PWR016
U 1 1 58AD48D5
P 6730 2650
F 0 "#PWR016" H 6730 2500 50  0001 C CNN
F 1 "+3.3V" H 6730 2790 50  0000 C CNN
F 2 "" H 6730 2650 50  0000 C CNN
F 3 "" H 6730 2650 50  0000 C CNN
	1    6730 2650
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR017
U 1 1 58AD58F1
P 6590 1920
F 0 "#PWR017" H 6590 1670 50  0001 C CNN
F 1 "GND" H 6590 1770 50  0000 C CNN
F 2 "" H 6590 1920 50  0000 C CNN
F 3 "" H 6590 1920 50  0000 C CNN
	1    6590 1920
	0    1    1    0   
$EndComp
$Comp
L USB_OTG-RESCUE-pcb P2
U 1 1 58AE0C3A
P 3260 6640
F 0 "P2" H 3585 6515 50  0000 C CNN
F 1 "USB micro B" H 3260 6840 50  0000 C CNN
F 2 "Connectors:USB_Micro-B_10103594-0001LF" V 3210 6540 50  0001 C CNN
F 3 "" V 3210 6540 50  0000 C CNN
F 4 "10103594-0001LF" H 3260 6640 60  0001 C CNN "Mfg_code"
	1    3260 6640
	0    1    1    0   
$EndComp
$Comp
L D_ALT D1
U 1 1 58AE100B
P 2960 5850
F 0 "D1" H 3060 5750 50  0000 C CNN
F 1 "S1DB" H 2860 5750 50  0000 C CNN
F 2 "Diodes_SMD:D_SMB_Handsoldering" H 2960 5850 50  0001 C CNN
F 3 "" H 2960 5850 50  0000 C CNN
F 4 "S1DB-13-F" H 2960 5850 60  0001 C CNN "Mfg_code"
	1    2960 5850
	0    1    1    0   
$EndComp
$Comp
L GND #PWR018
U 1 1 58AE1B78
P 3270 7040
F 0 "#PWR018" H 3270 6790 50  0001 C CNN
F 1 "GND" H 3270 6890 50  0000 C CNN
F 2 "" H 3270 7040 50  0000 C CNN
F 3 "" H 3270 7040 50  0000 C CNN
	1    3270 7040
	1    0    0    -1  
$EndComp
Text Label 5460 4110 1    60   ~ 0
USBD-
Text Label 5550 4110 1    60   ~ 0
USBD+
Text Label 2560 6540 0    60   ~ 0
USBD-
Text Label 2560 6640 0    60   ~ 0
USBD+
Text Label 5830 3980 0    60   ~ 0
RELAY_ON
Text Label 5830 3890 0    60   ~ 0
RELAY_OFF
$Comp
L GND #PWR019
U 1 1 58AE55E2
P 6220 720
F 0 "#PWR019" H 6220 470 50  0001 C CNN
F 1 "GND" H 6220 570 50  0000 C CNN
F 2 "" H 6220 720 50  0000 C CNN
F 3 "" H 6220 720 50  0000 C CNN
	1    6220 720 
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR020
U 1 1 58AE5F31
P 6590 1430
F 0 "#PWR020" H 6590 1180 50  0001 C CNN
F 1 "GND" H 6590 1280 50  0000 C CNN
F 2 "" H 6590 1430 50  0000 C CNN
F 3 "" H 6590 1430 50  0000 C CNN
	1    6590 1430
	0    1    1    0   
$EndComp
$Comp
L +3.3V #PWR021
U 1 1 58AE60D6
P 6220 1200
F 0 "#PWR021" H 6220 1050 50  0001 C CNN
F 1 "+3.3V" H 6220 1340 50  0000 C CNN
F 2 "" H 6220 1200 50  0000 C CNN
F 3 "" H 6220 1200 50  0000 C CNN
	1    6220 1200
	1    0    0    -1  
$EndComp
Text Notes 7060 1510 0    60   ~ 0
IR receiver\n(w/o pcb)
Text Notes 6800 2140 0    60   ~ 0
+
$Comp
L CONN_01X01 P7
U 1 1 58AE85A6
P 10100 2560
F 0 "P7" H 10100 2660 50  0000 C CNN
F 1 "(faston male 6.35mm)" V 10200 2560 50  0001 C CNN
F 2 "myconnectors:faston_m_63" H 10100 2560 50  0001 C CNN
F 3 "http://www.digikey.nl/product-detail/en/te-connectivity-amp-connectors/1217861-1/A100452TR-ND/1148769" H 10100 2560 50  0001 C CNN
F 4 "1217861-1" H 10100 2560 60  0001 C CNN "Mfg_code"
	1    10100 2560
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X01 P6
U 1 1 58AE877A
P 10100 2190
F 0 "P6" H 10100 2290 50  0000 C CNN
F 1 "(faston male 6.35mm)" V 10200 2190 50  0001 C CNN
F 2 "myconnectors:faston_m_63" H 10100 2190 50  0001 C CNN
F 3 "http://www.digikey.nl/product-detail/en/te-connectivity-amp-connectors/1217861-1/A100452TR-ND/1148769" H 10100 2190 50  0001 C CNN
F 4 "1217861-1" H 10100 2190 60  0001 C CNN "Mfg_code"
	1    10100 2190
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR022
U 1 1 58AE9236
P 9840 2560
F 0 "#PWR022" H 9840 2310 50  0001 C CNN
F 1 "GND" H 9840 2410 50  0000 C CNN
F 2 "" H 9840 2560 50  0000 C CNN
F 3 "" H 9840 2560 50  0000 C CNN
	1    9840 2560
	1    0    0    -1  
$EndComp
$Comp
L +12V #PWR023
U 1 1 58AE9776
P 9420 2170
F 0 "#PWR023" H 9420 2020 50  0001 C CNN
F 1 "+12V" H 9420 2310 50  0000 C CNN
F 2 "" H 9420 2170 50  0000 C CNN
F 3 "" H 9420 2170 50  0000 C CNN
	1    9420 2170
	1    0    0    -1  
$EndComp
$Comp
L Polyfuse F1
U 1 1 58AF63D6
P 2960 6190
F 0 "F1" V 2860 6190 50  0000 C CNN
F 1 "Polyfuse 250mA-hold 500mA-trip" V 3060 6190 50  0001 C CNN
F 2 "Fuse_Holders_and_Fuses:Fuse_SMD1206_HandSoldering" H 3010 5990 50  0001 L CNN
F 3 "" H 2960 6190 50  0001 C CNN
F 4 "0ZCJ0025AF2E" V 2960 6190 60  0001 C CNN "Mfg_code"
	1    2960 6190
	-1   0    0    1   
$EndComp
$Comp
L ESDA6V1W5 ZD1
U 1 1 58AF4F15
P 2490 3560
F 0 "ZD1" H 2340 3260 60  0000 C CNN
F 1 "ESDA6V1W5" H 2240 3360 60  0000 C CNN
F 2 "TO_SOT_Packages_SMD:SOT-353_SC-70-5_Handsoldering" H 2490 3560 60  0001 C CNN
F 3 "" H 2490 3560 60  0000 C CNN
F 4 "ESDA6V1W5" H 2490 3560 60  0001 C CNN "Mfg_code"
	1    2490 3560
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR024
U 1 1 58AF65A6
P 2740 4240
F 0 "#PWR024" H 2740 3990 50  0001 C CNN
F 1 "GND" H 2740 4090 50  0000 C CNN
F 2 "" H 2740 4240 50  0000 C CNN
F 3 "" H 2740 4240 50  0000 C CNN
	1    2740 4240
	1    0    0    -1  
$EndComp
$Comp
L Crystal_GND23 Y1
U 1 1 58AF6502
P 5930 3300
F 0 "Y1" V 5650 3240 50  0000 L CNN
F 1 "32.768kHz" V 5720 3090 50  0000 L CNN
F 2 "xtal:Crystal_SMD_Abracon_4pin_32768khz" H 5930 3300 50  0001 C CNN
F 3 "" H 5930 3300 50  0000 C CNN
F 4 "ABS25-32.768KHZ-T" V 5930 3300 60  0001 C CNN "Mfg_code"
	1    5930 3300
	0    1    1    0   
$EndComp
$Comp
L C C4
U 1 1 58ACEB9B
P 6460 3150
F 0 "C4" V 6510 3200 50  0000 L CNN
F 1 "15pF" V 6400 3200 50  0000 L CNN
F 2 "capacitor:C_0805_reflow" H 6498 3000 50  0001 C CNN
F 3 "" H 6460 3150 50  0000 C CNN
F 4 "CL21C150JBANNNC" H 6460 3150 60  0001 C CNN "Mfg_code"
	1    6460 3150
	0    1    1    0   
$EndComp
$Comp
L GND #PWR025
U 1 1 58B0C7C5
P 5710 3300
F 0 "#PWR025" H 5710 3050 50  0001 C CNN
F 1 "GND" V 5790 3270 50  0000 C CNN
F 2 "" H 5710 3300 50  0000 C CNN
F 3 "" H 5710 3300 50  0000 C CNN
	1    5710 3300
	0    1    1    0   
$EndComp
$Comp
L GND #PWR026
U 1 1 58B0CE3C
P 6150 3300
F 0 "#PWR026" H 6150 3050 50  0001 C CNN
F 1 "GND" V 6070 3260 50  0000 C CNN
F 2 "" H 6150 3300 50  0000 C CNN
F 3 "" H 6150 3300 50  0000 C CNN
	1    6150 3300
	0    -1   -1   0   
$EndComp
NoConn ~ 2960 6740
NoConn ~ 4490 1940
NoConn ~ 4490 2040
NoConn ~ 3990 2040
NoConn ~ 3990 2140
$Comp
L R R5
U 1 1 58ABB744
P 8220 4890
F 0 "R5" V 8300 4890 50  0000 C CNN
F 1 "1k" V 8220 4890 50  0000 C CNN
F 2 "resistor:R_0805_reflow" V 8150 4890 50  0001 C CNN
F 3 "" H 8220 4890 50  0000 C CNN
F 4 "RMCF0805FT1K00" V 8220 4890 60  0001 C CNN "Mfg_code"
	1    8220 4890
	0    1    1    0   
$EndComp
$Comp
L Q_NPN_BEC Q1
U 1 1 58B72CAE
P 8580 4890
F 0 "Q1" H 8280 5170 50  0000 L CNN
F 1 "MMBT2222" H 8270 5100 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-23" H 8780 4990 50  0001 C CNN
F 3 "http://www.digikey.com/product-detail/en/on-semiconductor/MMBT2222ALT1G/MMBT2222ALT1GOSCT-ND/1139806" H 8580 4890 50  0001 C CNN
F 4 "MMBT2222ALT1G" H 8580 4890 60  0001 C CNN "Mfg_code"
	1    8580 4890
	1    0    0    -1  
$EndComp
$Comp
L Q_NPN_BEC Q2
U 1 1 58B72FE7
P 9060 4890
F 0 "Q2" H 9060 5170 50  0000 L CNN
F 1 "MMBT2222" H 8760 5100 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-23" H 9260 4990 50  0001 C CNN
F 3 "http://www.digikey.com/product-detail/en/on-semiconductor/MMBT2222ALT1G/MMBT2222ALT1GOSCT-ND/1139806" H 9060 4890 50  0001 C CNN
F 4 "MMBT2222ALT1G" H 9060 4890 60  0001 C CNN "Mfg_code"
	1    9060 4890
	-1   0    0    -1  
$EndComp
$Comp
L R R9
U 1 1 58D80F12
P 6440 1230
F 0 "R9" V 6360 1230 50  0000 C CNN
F 1 "150" V 6440 1230 50  0000 C CNN
F 2 "resistor:R_0805_reflow" V 6370 1230 50  0001 C CNN
F 3 "" H 6440 1230 50  0000 C CNN
F 4 "RMCF0805FT150R" V 6440 1230 60  0001 C CNN "Mfg_code"
	1    6440 1230
	0    1    1    0   
$EndComp
$Comp
L C C6
U 1 1 58D81CFB
P 6590 870
F 0 "C6" H 6615 970 50  0000 L CNN
F 1 "4u7/25V" V 6460 670 50  0000 L CNN
F 2 "capacitor:C_0805_reflow" H 6628 720 50  0001 C CNN
F 3 "" H 6590 870 50  0000 C CNN
F 4 "CL21A475KAQNNNG" H 6590 870 60  0001 C CNN "Mfg_code"
	1    6590 870 
	1    0    0    -1  
$EndComp
$Comp
L R R7
U 1 1 58D824F1
P 1840 2920
F 0 "R7" V 1920 2920 50  0000 C CNN
F 1 "10k" V 1840 2920 50  0000 C CNN
F 2 "resistor:R_0805_reflow" V 1770 2920 50  0001 C CNN
F 3 "" H 1840 2920 50  0000 C CNN
F 4 "RMCF0805FT10K0" V 1840 2920 60  0001 C CNN "Mfg_code"
	1    1840 2920
	0    -1   -1   0   
$EndComp
$Comp
L +3.3V #PWR027
U 1 1 58D82CA8
P 2080 2920
F 0 "#PWR027" H 2080 2770 50  0001 C CNN
F 1 "+3.3V" H 2080 3060 50  0000 C CNN
F 2 "" H 2080 2920 50  0000 C CNN
F 3 "" H 2080 2920 50  0000 C CNN
	1    2080 2920
	1    0    0    -1  
$EndComp
$Comp
L C C7
U 1 1 58D83657
P 6870 870
F 0 "C7" H 6760 790 50  0000 L CNN
F 1 "100n" V 6990 790 50  0000 L CNN
F 2 "capacitor:C_0805_reflow" H 6908 720 50  0001 C CNN
F 3 "" H 6870 870 50  0000 C CNN
F 4 "C0805C104K5RACTU" H 6870 870 60  0001 C CNN "Mfg_code"
	1    6870 870 
	-1   0    0    1   
$EndComp
$Comp
L R R8
U 1 1 58D842DF
P 6220 1380
F 0 "R8" V 6140 1390 50  0000 C CNN
F 1 "10k" V 6220 1380 50  0000 C CNN
F 2 "resistor:R_0805_reflow" V 6150 1380 50  0001 C CNN
F 3 "" H 6220 1380 50  0000 C CNN
F 4 "RMCF0805FT10K0" V 6220 1380 60  0001 C CNN "Mfg_code"
	1    6220 1380
	1    0    0    -1  
$EndComp
Wire Wire Line
	8680 4110 8680 4690
Wire Wire Line
	8770 4110 8770 3660
Wire Wire Line
	8870 3660 8870 4110
Wire Wire Line
	8680 3260 8680 3810
Wire Wire Line
	8680 3260 8820 3260
Connection ~ 8680 3260
Connection ~ 8680 3810
Wire Wire Line
	8680 3810 8960 3810
Wire Wire Line
	8960 4110 8960 4690
Wire Wire Line
	9660 3260 9900 3260
Wire Wire Line
	9610 3660 9900 3660
Connection ~ 8960 4110
Connection ~ 8680 4110
Wire Wire Line
	8870 4110 8960 4110
Wire Wire Line
	8770 4110 8680 4110
Wire Wire Line
	9570 4890 10130 4890
Wire Wire Line
	3260 5110 3260 5320
Wire Wire Line
	3260 5700 3260 5620
Wire Wire Line
	3700 6000 3700 6080
Wire Wire Line
	3700 6080 4300 6080
Wire Wire Line
	4300 6000 4300 6200
Connection ~ 4830 5700
Wire Wire Line
	4600 5800 4600 5700
Connection ~ 4600 5700
Connection ~ 4300 6080
Connection ~ 4300 6030
Connection ~ 3260 5700
Connection ~ 3700 5700
Wire Wire Line
	5090 5700 5090 5600
Wire Wire Line
	3890 3500 4000 3500
Wire Wire Line
	3590 3500 3380 3500
Wire Wire Line
	4000 3000 3890 3000
Wire Wire Line
	3890 3000 3890 2850
Wire Wire Line
	4000 3800 3880 3800
Wire Wire Line
	3880 3800 3880 3860
Wire Wire Line
	4860 4120 4960 4120
Wire Wire Line
	4400 4120 4560 4120
Wire Wire Line
	4830 6000 4830 6030
Wire Wire Line
	4830 6030 4300 6030
Wire Wire Line
	3990 1740 3990 1640
Wire Wire Line
	3990 1940 3990 1840
Wire Wire Line
	3990 1840 3810 1840
Connection ~ 3990 1840
Wire Wire Line
	4490 2140 4920 2140
Wire Wire Line
	4490 1840 4920 1840
Wire Wire Line
	4490 1740 4920 1740
Wire Wire Line
	3940 3500 3940 3660
Wire Wire Line
	3940 3660 3550 3660
Connection ~ 3940 3500
Wire Wire Line
	1990 3200 4000 3200
Wire Wire Line
	1990 3100 4000 3100
Wire Wire Line
	5590 3300 5400 3300
Wire Wire Line
	5590 3150 5590 3300
Wire Wire Line
	5400 3400 5590 3400
Wire Wire Line
	5590 3400 5590 3450
Wire Wire Line
	6610 3150 6730 3150
Wire Wire Line
	6730 3150 6730 3450
Wire Wire Line
	6730 3450 6610 3450
Wire Wire Line
	6730 3290 6830 3290
Connection ~ 6730 3290
Wire Wire Line
	1430 3100 1690 3100
Wire Wire Line
	1430 3200 1690 3200
Wire Wire Line
	1430 3000 1430 2840
Wire Wire Line
	1430 3300 1430 3460
Wire Wire Line
	5400 3200 5530 3200
Wire Wire Line
	5530 3200 5530 2810
Wire Wire Line
	5530 2810 6310 2810
Wire Wire Line
	5470 3100 5400 3100
Wire Wire Line
	5470 2020 5470 3100
Wire Wire Line
	5470 2020 6660 2020
Wire Wire Line
	5400 3000 5400 1530
Wire Wire Line
	5400 1530 6680 1530
Wire Wire Line
	6610 2810 6730 2810
Wire Wire Line
	6730 2710 6730 2650
Wire Wire Line
	2960 6340 2960 6440
Wire Wire Line
	2960 7040 3360 7040
Wire Wire Line
	2960 7040 2960 6840
Wire Wire Line
	2960 6540 2560 6540
Wire Wire Line
	2960 6640 2560 6640
Wire Wire Line
	5400 3800 5460 3800
Wire Wire Line
	5460 3800 5460 4110
Wire Wire Line
	5400 3700 5550 3700
Wire Wire Line
	5550 3700 5550 4110
Wire Wire Line
	5400 3600 5650 3600
Wire Wire Line
	5650 3600 5650 3980
Wire Wire Line
	5650 3980 6290 3980
Wire Wire Line
	5400 3500 5730 3500
Wire Wire Line
	5730 3500 5730 3890
Wire Wire Line
	5730 3890 6290 3890
Wire Wire Line
	6680 1430 6590 1430
Wire Wire Line
	6590 1330 6680 1330
Wire Wire Line
	9900 2560 9840 2560
Wire Wire Line
	9900 2190 9840 2190
Wire Wire Line
	2960 6040 2960 6000
Wire Wire Line
	2590 3100 2590 3360
Connection ~ 2590 3100
Wire Wire Line
	2790 3360 2790 3200
Connection ~ 2790 3200
Wire Wire Line
	2740 4210 2740 4240
Wire Wire Line
	5590 3450 6310 3450
Wire Wire Line
	5590 3150 6310 3150
Wire Wire Line
	5710 3300 5730 3300
Wire Wire Line
	6130 3300 6150 3300
Connection ~ 5930 3150
Connection ~ 5930 3450
Connection ~ 3270 7040
Wire Wire Line
	2960 5700 4000 5700
Wire Wire Line
	9420 2170 9420 2190
Wire Wire Line
	9420 2190 9540 2190
Wire Wire Line
	9260 4890 9270 4890
Wire Wire Line
	8070 4890 7610 4890
Wire Wire Line
	2890 3360 2890 3100
Connection ~ 2890 3100
Wire Wire Line
	2690 3360 2690 3200
Connection ~ 2690 3200
Wire Wire Line
	6590 1920 6660 1920
Wire Wire Line
	8370 4890 8380 4890
Wire Wire Line
	8680 5090 8680 5170
Wire Wire Line
	8680 5170 8960 5170
Wire Wire Line
	8960 5170 8960 5090
Wire Wire Line
	8820 5190 8820 5170
Connection ~ 8820 5170
Wire Wire Line
	4600 5700 5090 5700
Wire Wire Line
	6590 1020 6590 1330
Wire Wire Line
	6220 720  6870 720 
Wire Wire Line
	1650 3100 1650 2920
Wire Wire Line
	1650 2920 1690 2920
Connection ~ 1650 3100
Wire Wire Line
	1990 2920 2080 2920
Connection ~ 6590 720 
Wire Wire Line
	6590 1020 6870 1020
Connection ~ 6590 1020
Wire Wire Line
	6220 1230 6290 1230
Wire Wire Line
	6220 1230 6220 1200
Connection ~ 6220 1230
Connection ~ 6220 1530
Connection ~ 6590 1230
Text Notes 2100 3100 0    60   ~ 0
-->
Text Notes 2090 3200 0    60   ~ 0
<--
$EndSCHEMATC
