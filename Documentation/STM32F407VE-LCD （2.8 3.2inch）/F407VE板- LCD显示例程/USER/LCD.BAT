SET PATH=C:\Keil\ARM\ARMCC\bin\;C:\Program Files (x86)\STMicroelectronics\st_toolset\asm;C:\Program Files (x86)\Intel\iCLS Client\;C:\Program Files\Intel\iCLS Client\;C:\Windows\system32;C:\Windows;C:\Windows\System32\Wbem;C:\Windows\System32\WindowsPowerShell\v1.0\;C:\Program Files\Intel\Intel(R) Management Engine Components\DAL;C:\Program Files\Intel\Intel(R) Management Engine Components\IPT;C:\Program Files (x86)\Intel\Intel(R) Management Engine Components\DAL;C:\Program Files (x86)\Intel\Intel(R) Management Engine Components\IPT;C:\Program Files (x86)\Altium Designer Summer 09\System;C:\Program Files (x86)\IDM Computer Solutions\UltraEdit\;C:\Program Files (x86)\COSMIC\CXSTM8_EVAL
SET CPU_TYPE=STM32F407VE
SET CPU_VENDOR=STMicroelectronics
SET UV2_TARGET=LCD
SET CPU_CLOCK=0x007A1200
"C:\Keil\ARM\ARMCC\bin\ArmCC" --Via "..\obj\main.__i"
"C:\Keil\ARM\ARMCC\bin\ArmCC" --Via "..\obj\stm32f4xx_it.__i"
"C:\Keil\ARM\ARMCC\bin\ArmCC" --Via "..\obj\system_stm32f4xx.__i"
"C:\Keil\ARM\ARMCC\bin\ArmCC" --Via "..\obj\led.__i"
"C:\Keil\ARM\ARMCC\bin\ArmCC" --Via "..\obj\lcd.__i"
"C:\Keil\ARM\ARMCC\bin\ArmCC" --Via "..\obj\delay.__i"
"C:\Keil\ARM\ARMCC\bin\ArmCC" --Via "..\obj\sys.__i"
"C:\Keil\ARM\ARMCC\bin\ArmCC" --Via "..\obj\usart.__i"
"C:\Keil\ARM\ARMCC\bin\ArmAsm" --Via "..\obj\startup_stm32f40_41xxx._ia"
"C:\Keil\ARM\ARMCC\bin\ArmCC" --Via "..\obj\misc.__i"
"C:\Keil\ARM\ARMCC\bin\ArmCC" --Via "..\obj\stm32f4xx_gpio.__i"
"C:\Keil\ARM\ARMCC\bin\ArmCC" --Via "..\obj\stm32f4xx_fsmc.__i"
"C:\Keil\ARM\ARMCC\bin\ArmCC" --Via "..\obj\stm32f4xx_rcc.__i"
"C:\Keil\ARM\ARMCC\bin\ArmCC" --Via "..\obj\stm32f4xx_syscfg.__i"
"C:\Keil\ARM\ARMCC\bin\ArmCC" --Via "..\obj\stm32f4xx_usart.__i"
"C:\Keil\ARM\ARMCC\bin\ArmLink" --Via "..\OBJ\LCD.lnp"
C:\Keil\ARM\ARMCC\bin\fromelf.exe "..\OBJ\LCD.axf" --i32combined --output "..\OBJ\LCD.hex"
