
openocd -f "stlink-v2-fekete.cfg" -f "stm32f1x.cfg"

rem run this first and leave the terminal open: 
rem itmdump -f itm.fifo -s 0
rem gdb port 5000
rem telnet port 4444
rem arm-none-eabi-gdb c:\Eclipse.portable\RustForSTM32\cortex-m-quickstart\target\thumbv7m-none-eabi\debug\libcortex_m_quickstart