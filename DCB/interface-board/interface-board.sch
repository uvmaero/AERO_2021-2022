EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Device:LED D?
U 1 1 62083CC8
P 4650 2400
F 0 "D?" V 4597 2480 50  0000 L CNN
F 1 "LED" V 4688 2480 50  0000 L CNN
F 2 "" H 4650 2400 50  0001 C CNN
F 3 "~" H 4650 2400 50  0001 C CNN
	1    4650 2400
	0    1    1    0   
$EndComp
$Comp
L Device:LED D?
U 1 1 620840B4
P 5400 2400
F 0 "D?" V 5347 2480 50  0000 L CNN
F 1 "LED" V 5438 2480 50  0000 L CNN
F 2 "" H 5400 2400 50  0001 C CNN
F 3 "~" H 5400 2400 50  0001 C CNN
	1    5400 2400
	0    1    1    0   
$EndComp
$Comp
L Device:R_POT_Small RV?
U 1 1 62084C31
P 5100 4150
F 0 "RV?" H 5040 4196 50  0000 R CNN
F 1 "R_POT_Small" H 5050 4300 50  0000 R CNN
F 2 "" H 5100 4150 50  0001 C CNN
F 3 "~" H 5100 4150 50  0001 C CNN
	1    5100 4150
	-1   0    0    -1  
$EndComp
$Comp
L Device:R_POT_Small RV?
U 1 1 62084EEF
P 5900 4150
F 0 "RV?" H 5841 4196 50  0000 R CNN
F 1 "R_POT_Small" H 5841 4105 50  0000 R CNN
F 2 "" H 5900 4150 50  0001 C CNN
F 3 "~" H 5900 4150 50  0001 C CNN
	1    5900 4150
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x14 J?
U 1 1 61F1F502
P 10100 2050
F 0 "J?" H 10180 2042 50  0000 L CNN
F 1 "Conn_01x14" H 10180 1951 50  0000 L CNN
F 2 "" H 10100 2050 50  0001 C CNN
F 3 "~" H 10100 2050 50  0001 C CNN
	1    10100 2050
	1    0    0    -1  
$EndComp
Text Notes 9700 1250 0    50   ~ 0
Driver Interface
Text Label 9400 1550 0    50   ~ 0
START_BTN
Text Label 9400 1650 0    50   ~ 0
RTD_LED
Text Label 9400 1750 0    50   ~ 0
IMD_LED
Text Label 9400 1850 0    50   ~ 0
AMS_LED
Text Label 9400 1950 0    50   ~ 0
POT_REGEN
Text Label 9400 2050 0    50   ~ 0
POT_BRAKE
$Comp
L power:+5V #PWR?
U 1 1 61FDABD7
P 9400 1450
F 0 "#PWR?" H 9400 1300 50  0001 C CNN
F 1 "+5V" H 9415 1623 50  0000 C CNN
F 2 "" H 9400 1450 50  0001 C CNN
F 3 "" H 9400 1450 50  0001 C CNN
	1    9400 1450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 61FE269F
P 9400 2750
F 0 "#PWR?" H 9400 2500 50  0001 C CNN
F 1 "GND" H 9405 2577 50  0000 C CNN
F 2 "" H 9400 2750 50  0001 C CNN
F 3 "" H 9400 2750 50  0001 C CNN
	1    9400 2750
	1    0    0    -1  
$EndComp
Wire Wire Line
	9400 2750 9900 2750
Wire Wire Line
	9900 1450 9400 1450
Wire Wire Line
	9400 1550 9900 1550
Wire Wire Line
	9900 1650 9400 1650
Wire Wire Line
	9400 1750 9900 1750
Wire Wire Line
	9900 1850 9400 1850
Wire Wire Line
	9400 1950 9900 1950
Wire Wire Line
	9900 2050 9400 2050
Wire Wire Line
	9900 2350 9400 2350
Wire Wire Line
	9400 2250 9900 2250
Text Label 9400 2350 0    50   ~ 0
LCD_SDA
Text Label 9400 2250 0    50   ~ 0
LCD_SCL
Wire Wire Line
	9900 3800 9400 3800
Wire Wire Line
	9400 3700 9900 3700
Text Label 9400 3800 0    50   ~ 0
LCD_SDA
Text Label 9400 3700 0    50   ~ 0
LCD_SCL
$Comp
L power:+5V #PWR?
U 1 1 620A9FAC
P 9400 3600
F 0 "#PWR?" H 9400 3450 50  0001 C CNN
F 1 "+5V" H 9415 3773 50  0000 C CNN
F 2 "" H 9400 3600 50  0001 C CNN
F 3 "" H 9400 3600 50  0001 C CNN
	1    9400 3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	9900 3600 9400 3600
Text Notes 9550 3350 0    50   ~ 0
To LCD Panel 
Wire Wire Line
	9900 2150 9400 2150
Text Label 9400 2150 0    50   ~ 0
LCD_BTN
$Comp
L Connector_Generic:Conn_01x05 J?
U 1 1 620B584B
P 10100 3800
F 0 "J?" H 10180 3842 50  0000 L CNN
F 1 "Conn_01x05" H 10180 3751 50  0000 L CNN
F 2 "" H 10100 3800 50  0001 C CNN
F 3 "~" H 10100 3800 50  0001 C CNN
	1    10100 3800
	1    0    0    -1  
$EndComp
Wire Wire Line
	9400 4000 9900 4000
$Comp
L power:GND #PWR?
U 1 1 620A73F4
P 9400 4000
F 0 "#PWR?" H 9400 3750 50  0001 C CNN
F 1 "GND" H 9405 3827 50  0000 C CNN
F 2 "" H 9400 4000 50  0001 C CNN
F 3 "" H 9400 4000 50  0001 C CNN
	1    9400 4000
	1    0    0    -1  
$EndComp
Wire Wire Line
	9900 3900 9400 3900
Text Label 9400 3900 0    50   ~ 0
LCD_BTN
Text Label 6850 2050 2    50   ~ 0
START_BTN
Text Label 5700 2050 0    50   ~ 0
RTD_LED
Text Label 4150 2150 0    50   ~ 0
IMD_LED
Text Label 4900 2150 0    50   ~ 0
AMS_LED
Text Label 4500 4150 0    50   ~ 0
POT_REGEN
Text Label 6500 4150 2    50   ~ 0
POT_BRAKE
Wire Wire Line
	6200 2050 5700 2050
Wire Wire Line
	4150 2150 4650 2150
Wire Wire Line
	5400 2150 4900 2150
Wire Wire Line
	4500 4150 5000 4150
Wire Wire Line
	6000 4150 6500 4150
Wire Wire Line
	7300 3850 6800 3850
Text Label 6800 3850 0    50   ~ 0
LCD_BTN
Wire Wire Line
	6200 2050 6200 2150
Wire Wire Line
	4650 2150 4650 2250
Wire Wire Line
	5400 2250 5400 2150
Wire Wire Line
	6300 2050 6300 2150
Wire Wire Line
	6300 2050 6850 2050
Wire Wire Line
	4650 2550 4650 2800
Wire Wire Line
	4650 2800 5400 2800
Wire Wire Line
	6300 2800 6300 2550
Wire Wire Line
	5400 2550 5400 2800
Connection ~ 5400 2800
Wire Wire Line
	5400 2800 6200 2800
Wire Wire Line
	6200 2550 6200 2800
Connection ~ 6200 2800
Wire Wire Line
	6200 2800 6300 2800
$Comp
L power:GND #PWR?
U 1 1 620E66DE
P 5400 2950
F 0 "#PWR?" H 5400 2700 50  0001 C CNN
F 1 "GND" H 5405 2777 50  0000 C CNN
F 2 "" H 5400 2950 50  0001 C CNN
F 3 "" H 5400 2950 50  0001 C CNN
	1    5400 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	5400 2950 5400 2800
$Comp
L Switch:SW_Push SW?
U 1 1 620F1601
P 7300 4050
F 0 "SW?" V 7254 4198 50  0000 L CNN
F 1 "SW_Push" V 7345 4198 50  0000 L CNN
F 2 "" H 7300 4250 50  0001 C CNN
F 3 "~" H 7300 4250 50  0001 C CNN
	1    7300 4050
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 620F3990
P 7300 4300
F 0 "#PWR?" H 7300 4050 50  0001 C CNN
F 1 "GND" H 7305 4127 50  0000 C CNN
F 2 "" H 7300 4300 50  0001 C CNN
F 3 "" H 7300 4300 50  0001 C CNN
	1    7300 4300
	1    0    0    -1  
$EndComp
Wire Wire Line
	7300 4300 7300 4250
$Comp
L Switch:SW_Push_LED SW?
U 1 1 620F5B16
P 6200 2350
F 0 "SW?" V 6154 2597 50  0000 L CNN
F 1 "SW_Push_LED" V 6245 2597 50  0000 L CNN
F 2 "" H 6200 2650 50  0001 C CNN
F 3 "~" H 6200 2650 50  0001 C CNN
	1    6200 2350
	0    1    1    0   
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 620F833C
P 5100 3850
F 0 "#PWR?" H 5100 3700 50  0001 C CNN
F 1 "+5V" H 5115 4023 50  0000 C CNN
F 2 "" H 5100 3850 50  0001 C CNN
F 3 "" H 5100 3850 50  0001 C CNN
	1    5100 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	5100 3850 5100 3900
Wire Wire Line
	5100 3900 5900 3900
Wire Wire Line
	5900 3900 5900 4050
Connection ~ 5100 3900
Wire Wire Line
	5100 3900 5100 4050
Wire Wire Line
	5900 4250 5900 4400
Wire Wire Line
	5900 4400 5500 4400
Wire Wire Line
	5100 4400 5100 4250
$Comp
L power:GND #PWR?
U 1 1 620FAC9A
P 5500 4400
F 0 "#PWR?" H 5500 4150 50  0001 C CNN
F 1 "GND" H 5505 4227 50  0000 C CNN
F 2 "" H 5500 4400 50  0001 C CNN
F 3 "" H 5500 4400 50  0001 C CNN
	1    5500 4400
	1    0    0    -1  
$EndComp
Connection ~ 5500 4400
Wire Wire Line
	5500 4400 5100 4400
Text Label 9400 2450 0    50   ~ 0
SW_COOLING
Wire Wire Line
	9400 2450 9900 2450
Text Label 9350 2550 0    50   ~ 0
SW_DIRECTION
Wire Wire Line
	9350 2550 9900 2550
$Comp
L Switch:SW_SPST SW?
U 1 1 61EB479C
P 3350 3800
F 0 "SW?" V 3304 3898 50  0000 L CNN
F 1 "SW_SPST" V 3395 3898 50  0000 L CNN
F 2 "" H 3350 3800 50  0001 C CNN
F 3 "~" H 3350 3800 50  0001 C CNN
	1    3350 3800
	0    1    1    0   
$EndComp
$Comp
L Switch:SW_SPST SW?
U 1 1 61EB4BEF
P 2600 3800
F 0 "SW?" V 2554 3898 50  0000 L CNN
F 1 "SW_SPST" V 2645 3898 50  0000 L CNN
F 2 "" H 2600 3800 50  0001 C CNN
F 3 "~" H 2600 3800 50  0001 C CNN
	1    2600 3800
	0    1    1    0   
$EndComp
Text Label 2850 3600 0    50   ~ 0
SW_COOLING
Wire Wire Line
	2850 3600 3350 3600
Text Label 2050 3600 0    50   ~ 0
SW_DIRECTION
Wire Wire Line
	2050 3600 2600 3600
$Comp
L power:GND #PWR?
U 1 1 61EBBA07
P 2950 4100
F 0 "#PWR?" H 2950 3850 50  0001 C CNN
F 1 "GND" H 2955 3927 50  0000 C CNN
F 2 "" H 2950 4100 50  0001 C CNN
F 3 "" H 2950 4100 50  0001 C CNN
	1    2950 4100
	1    0    0    -1  
$EndComp
Wire Wire Line
	2600 4000 2600 4100
Wire Wire Line
	2600 4100 2950 4100
Wire Wire Line
	2950 4100 3350 4100
Wire Wire Line
	3350 4100 3350 4000
Connection ~ 2950 4100
$EndSCHEMATC