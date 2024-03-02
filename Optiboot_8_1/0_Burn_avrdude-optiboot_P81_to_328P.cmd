@echo Burn standard Uno 328P Bootloader V8.1 on a 328P by avrdude
rem unlock bootloader 0x3x No restrictions for SPM or LPM accessing the boot loader section. 0xDE is for 512 bytes bootsize.
avrdude -v -patmega328pb -cstk500v1 -PCOM6 -b19200 -e -Ulock:w:0x3F:m -Uefuse:w:0xFD:m -Uhfuse:w:0xDE:m -Ulfuse:w:0xFF:m
rem lock bootloader against overwriting with: lock=0x0x All restrictions for SPM or LPM accessing the boot loader section.
avrdude -v -patmega328p -cstk500v1 -PCOM6 -b19200 -u -Uflash:w:optiboot_atmega328P_81.hex:a -Ulock:w:0x0F:m
pause
