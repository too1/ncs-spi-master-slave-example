NCS-SPI-Master-Slave-Example
############################

Overview
********
This example shows how to use the SPI master and slave async drivers in Zephyr, and sets up a separate master and slave interface in order to allow sending data from the master to the slave and vice versa. 

The SPI bus lines needs to be externally connected between the master and slave pins in order to test communication between them.
 
The SPI master will send a 2 byte transaction every second, including an incrementing counter, and check if anything is received on the SPI slave interface. 

Any sent or received data on the SPI master and slave will be printed to the terminal. 

Requirements
************

SDK: 
	- nRF Connect SDK v2.5.2
	
Supported boards: 
	- nrf52dk_nrf52832
	- nrf52833dk_nrf52833
	- nrf52840dk_nrf52840
	- nrf5340dk_nrf5340_cpuapp
	- nrf5340_audio_dk_nrf5340_cpuapp
