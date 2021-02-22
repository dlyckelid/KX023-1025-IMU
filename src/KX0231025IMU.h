/*
  This file is part of the KX023-1025-IMU library.
  Copyright (c) 2021 Good Solutions Sweden AB. All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/
#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>


class KX0231025Class {
public:
	KX0231025Class(TwoWire& wire, uint8_t slaveAddress);
	KX0231025Class(SPIClass& spi, int csPin);
	virtual ~KX0231025Class();

	int begin(int powerMode = 1, int accelerationRange = 0, int outputDataRate = 2); //powerMode 0 = LowPower, 1 = HighPower; accelerationRange 0 = +/-2g, 1 = +/-4g, 2 = +/-8g; outputDataRate = 0(12.5Hz),1(25Hz),2(50Hz),3(100Hz),4(200Hz),5(400Hz),6(800Hz),7(1600Hz),8(0.781Hz),9(1.563Hz),10(3.125Hz),11(6.25Hz)
	void end();

	// Accelerometer
	virtual int readAcceleration(float& x, float& y, float& z); // Results are in G (earth gravity).

private:
	int readRegister(uint8_t address);
	int readRegisters(uint8_t address, uint8_t* data, size_t length);
	int writeRegister(uint8_t address, uint8_t value);

private:
	TwoWire* _wire;
	SPIClass* _spi;
	uint8_t _slaveAddress;
	int _csPin;
	int accelerationRange;

	SPISettings _spiSettings;
};


