#include "MirfHardwareSpiDriver.h"
uint8_t MirfHardwareSpiDriver::transfer(uint8_t data){
	//return SPI.transfer(data);
	 uint8_t ret;
 
    SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
    ret = SPI.transfer(data);
    SPI.endTransaction();
    return ret;
}

void MirfHardwareSpiDriver::begin(){
	SPI.begin();
	//SPI.setDataMode(SPI_MODE0);
	//SPI.setClockDivider(SPI_2XCLOCK_MASK);
}

void MirfHardwareSpiDriver::end(){
}

MirfHardwareSpiDriver MirfHardwareSpi;
