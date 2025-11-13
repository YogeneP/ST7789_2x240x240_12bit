*** Under development ***

Driver for double 240x240 SPI display control with STM32F103C8T6 BluePill board.
The aim - the test/numbers interface.
Now mainly works, but extremely slow. BluePill has a 18 Mbits/s limitation of SPI performance, so full screen filling takes about 50 ms. 
Assuming 12bit colors to be faster. 
Adaptive screens areas refilling needed to provide some appropriable FPS.   
