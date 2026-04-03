/*
 * Code based on:
 * Dynamixel.cpp - Ax-12+ Half Duplex USART Comunication
 * Copyright (c) 2011 Savage Electronics.
 * And Dynamixel Pro library for OpenCM-9.04 made by ROBOTIS, LTD.
 * And Luis G III for HelloSpoon robot (http://hellospoonrobot.com).
 * 
 * Modified to work only with Dynamixel XL-330 actuator.
 * 
 * Modifications made by Rei Lee
 * Webpage: https://infosci.cornell.edu/~reilee/
 * Email: wl593@cornell.edu
 * 
 * This file can be used and be modified by anyone, 
 * don't forget to say thank you to OP!
 */

#include "Arduino.h"
#include "dxl_pro.h"
#include "XL330.h"
#include <stdlib.h>
#include <stdarg.h>

 // Macro for the selection of the Serial Port
#define sendData(args)  (this->stream->write(args))    // Write Over Serial
#define beginCom(args)      // Begin Serial Comunication
#define readData()		(this->stream->read())	

// Select the Switch to TX/RX Mode Pin
#define setDPin(DirPin,Mode)    
#define switchCom(DirPin,Mode)   // Switch to TX/RX Mode
#define NANO_TIME_DELAY 12000


XL330::XL330() {

}

XL330::~XL330() {
}

void XL330::begin(Stream& stream) {
	//setDPin(Direction_Pin=4,OUTPUT);
	//beginCom(1000000);
	this->stream = &stream;
}

void XL330::setBaudRate(int id, int value) {
	// 0: 9600; 1: 57600; 2: 115200; 3: 1M; 4: 2M; 5: 3M; 6: 4M (bps)
	int Address = XL_BAUD_RATE;

	sendPacket_1byte(id, Address, value);
	this->stream->flush();
	nDelay(NANO_TIME_DELAY);
}

void XL330::setID(int id, int value) {
	// 254: broadcast; id value: 0~252
	int Address = XL_ID;

	sendPacket_1byte(id, Address, value);
	this->stream->flush();
	nDelay(NANO_TIME_DELAY);
}

void XL330::setControlMode(int id, int value) {
	// 0: current control; 1: velocity control; 3: position control; 4: extended position control; 5: current-base position control; 16: PWM control
	int Address = XL_CONTROL_MODE;

	sendPacket_1byte(id, Address, value);
	this->stream->flush();
	nDelay(NANO_TIME_DELAY);
}

void XL330::setJointPosition(int id, int value) {
	int Address = XL_GOAL_POSITION;

	sendPacket_4bytes(id, Address, value);
	this->stream->flush();
	nDelay(NANO_TIME_DELAY);
}

void XL330::setJointSpeed(int id, int value) {
	int Address = XL_GOAL_PWM;

	sendPacket(id, Address, value);
	this->stream->flush();
	nDelay(NANO_TIME_DELAY);
}

void XL330::LEDON(int id) {
	int Address = XL_LED;
	int val = 1;

	sendPacket_1byte(id, Address, val);
	this->stream->flush();
	nDelay(NANO_TIME_DELAY);
}

void XL330::LEDOFF(int id) {
	int Address = XL_LED;
	int val = 0;

	sendPacket_1byte(id, Address, val);
	this->stream->flush();
	nDelay(NANO_TIME_DELAY);
}

void XL330::TorqueON(int id) {
	int Address = XL_TORQUE_ENABLE;
	int value = 1;

	sendPacket_1byte(id, Address, value);
	this->stream->flush();
	nDelay(NANO_TIME_DELAY);
}

void XL330::TorqueOFF(int id) {
	int Address = XL_TORQUE_ENABLE;
	int value = 0;

	sendPacket_1byte(id, Address, value);
	this->stream->flush();
	nDelay(NANO_TIME_DELAY);
}

int XL330::getJointPosition(int id) {
	unsigned char buffer[255];
	RXsendPacket(id, XL_PRESENT_POSITION, 4);
	this->stream->flush();
	nDelay(NANO_TIME_DELAY);

	int packetSize = this->readPacket(buffer, sizeof(buffer));
	if (packetSize <= 0) {
		return -2;
	}

	Packet p(buffer, packetSize);
	if (!p.isValid() || p.getInstruction() != 0x55 || p.getParameterCount() < 5 || p.getParameter(0) != 0) {
		return -1;
	}

	int value = (p.getParameter(1)) |
		(p.getParameter(2) << 8) |
		(p.getParameter(3) << 16) |
		(p.getParameter(4) << 24);

	return value;
}

int XL330::getJointSpeed(int id) {
	unsigned char buffer[255];
	RXsendPacket(id, XL_PRESENT_VELOCITY, 4);
	this->stream->flush();
	nDelay(NANO_TIME_DELAY);

	int packetSize = this->readPacket(buffer, sizeof(buffer));
	if (packetSize <= 0) {
		return -2;
	}

	Packet p(buffer, packetSize);
	if (!p.isValid() || p.getInstruction() != 0x55 || p.getParameterCount() < 5 || p.getParameter(0) != 0) {
		return -1;
	}

	int value = (p.getParameter(1)) |
		(p.getParameter(2) << 8) |
		(p.getParameter(3) << 16) |
		(p.getParameter(4) << 24);

	return value;
}

int XL330::getJointTemperature(int id) {
	unsigned char buffer[255];
	RXsendPacket(id, XL_PRESENT_TEMPERATURE, 1);
	this->stream->flush();
	nDelay(NANO_TIME_DELAY);

	int packetSize = this->readPacket(buffer, sizeof(buffer));
	if (packetSize <= 0) {
		return -2;
	}

	Packet p(buffer, packetSize);
	if (!p.isValid() || p.getInstruction() != 0x55 || p.getParameterCount() < 2 || p.getParameter(0) != 0) {
		return -1;
	}

	return p.getParameter(1);
}

int XL330::isJointMoving(int id) {
	unsigned char buffer[255];
	RXsendPacket(id, XL_MOVING, 1);
	this->stream->flush();
	nDelay(NANO_TIME_DELAY);

	int packetSize = this->readPacket(buffer, sizeof(buffer));
	if (packetSize <= 0) {
		return -2;
	}

	Packet p(buffer, packetSize);
	if (!p.isValid() || p.getInstruction() != 0x55 || p.getParameterCount() < 2 || p.getParameter(0) != 0) {
		return -1;
	}

	return p.getParameter(1);
}

int XL330::ping(int id) {
	const int bufsize = 10;
	byte txbuffer[bufsize];

	Packet p(txbuffer, bufsize, id, XL_INSTR_PING, 0);
	int size = p.getSize();
	stream->write(txbuffer, size);
	return size;
}

int XL330::action(int id) {
	const int bufsize = 10;
	byte txbuffer[bufsize];

	Packet p(txbuffer, bufsize, id, XL_INSTR_ACTION, 0);
	int size = p.getSize();
	stream->write(txbuffer, size);
	return size;
}

int XL330::reboot(int id) {
	const int bufsize = 10;
	byte txbuffer[bufsize];

	Packet p(txbuffer, bufsize, id, XL_INSTR_REBOOT, 0);
	int size = p.getSize();
	stream->write(txbuffer, size);
	return size;
}

int XL330::factoryReset(int id, int option) {
	const int bufsize = 11;
	byte txbuffer[bufsize];

	Packet p(txbuffer, bufsize, id, XL_INSTR_FACTORY_RESET, 1,
		DXL_0BYTE(option));

	int size = p.getSize();
	stream->write(txbuffer, size);
	return size;
}

int XL330::clearMultiTurnInfo(int id) {
	const int bufsize = 15;
	byte txbuffer[bufsize];

	Packet p(txbuffer, bufsize, id, XL_INSTR_CLEAR, 5,
		0x01,
		0x44,
		0x58,
		0x4C,
		0x22);

	int size = p.getSize();
	stream->write(txbuffer, size);
	return size;
}

int XL330::controlTableBackup(int id, int option) {
	const int bufsize = 15;
	byte txbuffer[bufsize];

	Packet p(txbuffer, bufsize, id, XL_INSTR_CONTROL_TABLE_BACKUP, 5,
		DXL_0BYTE(option),
		0x43,
		0x54,
		0x52,
		0x4C);

	int size = p.getSize();
	stream->write(txbuffer, size);
	return size;
}

int XL330::regWrite(int id, int Address, int value) {
	// For sending 2-byte data with REG_WRITE instruction.
	const int bufsize = 16;

	byte txbuffer[bufsize];

	Packet p(txbuffer, bufsize, id, XL_INSTR_REG_WRITE, 4,
		DXL_0BYTE(Address),
		DXL_1BYTE(Address),
		DXL_0BYTE(value),
		DXL_1BYTE(value));

	int size = p.getSize();
	stream->write(txbuffer, size);
	return size;
}

int XL330::regWrite_4bytes(int id, int Address, int value) {
	// For sending 4-byte data with REG_WRITE instruction.
	const int bufsize = 18;

	byte txbuffer[bufsize];

	Packet p(txbuffer, bufsize, id, XL_INSTR_REG_WRITE, 6,
		DXL_0BYTE(Address),
		DXL_1BYTE(Address),
		DXL_0BYTE(value),
		DXL_1BYTE(value),
		DXL_2BYTE(value),
		DXL_3BYTE(value));

	int size = p.getSize();
	stream->write(txbuffer, size);
	return size;
}

int XL330::regWrite_1byte(int id, int Address, int value) {
	// For sending 1 byte data with REG_WRITE instruction.
	const int bufsize = 18;

	byte txbuffer[bufsize];

	Packet p(txbuffer, bufsize, id, XL_INSTR_REG_WRITE, 3,
		DXL_0BYTE(Address),
		DXL_1BYTE(Address),
		DXL_0BYTE(value));

	int size = p.getSize();
	stream->write(txbuffer, size);
	return size;
}

int XL330::sendPacket(int id, int Address, int value) {
	/*Dynamixel 2.0 communication protocol
	  used by Dynamixel XL-320 and Dynamixel PRO only.
	*/
	// For sending 2-byte data
	const int bufsize = 16;

	byte txbuffer[bufsize];

	Packet p(txbuffer, bufsize, id, 0x03, 4,
		DXL_0BYTE(Address),
		DXL_1BYTE(Address),
		DXL_0BYTE(value),
		DXL_1BYTE(value));

	int size = p.getSize();
	stream->write(txbuffer, size);
	return size;
}

int XL330::sendPacket_4bytes(int id, int Address, int value) {
	/*Dynamixel 2.0 communication protocol
	  used by Dynamixel XL-330 and Dynamixel PRO only.
	*/
	// For sending 4-byte data
	const int bufsize = 18;

	byte txbuffer[bufsize];

	Packet p(txbuffer, bufsize, id, 0x03, 6,
		DXL_0BYTE(Address),
		DXL_1BYTE(Address),
		DXL_0BYTE(value),
		DXL_1BYTE(value),
		DXL_2BYTE(value),
		DXL_3BYTE(value));

	int size = p.getSize();
	stream->write(txbuffer, size);
	return size;
}

int XL330::sendPacket_1byte(int id, int Address, int value) {
	/*Dynamixel 2.0 communication protocol
	  used by Dynamixel XL-330 and Dynamixel PRO only.
	*/
	// For sending 1 byte data
	const int bufsize = 18;

	byte txbuffer[bufsize];

	Packet p(txbuffer, bufsize, id, 0x03, 3,
		DXL_0BYTE(Address),
		DXL_1BYTE(Address),
		DXL_0BYTE(value));

	int size = p.getSize();
	stream->write(txbuffer, size);
	return size;
}


void XL330::nDelay(uint32_t nTime) {
	/*
	uint32_t max;
	for( max=0; max < nTime; max++){

	}
	*/
}

void XL330::flush() {
	this->stream->flush();
}

int XL330::RXsendPacket(int id, int Address) {
	return this->RXsendPacket(id, Address, 2);
}

int XL330::RXsendPacket(int id, int Address, int size) {

	/*Dynamixel 2.0 communication protocol
	  used by Dynamixel XL-320 and Dynamixel PRO only.
	*/

	const int bufsize = 16;

	byte txbuffer[bufsize];

	Packet p(txbuffer, bufsize, id, XL_INSTR_READ, 4,
		DXL_0BYTE(Address),
		DXL_1BYTE(Address),
		DXL_0BYTE(size),
		DXL_1BYTE(size));

	int requestSize = p.getSize();
	stream->write(txbuffer, requestSize);

	return requestSize;
}

int XL330::readPacket(unsigned char* BUFFER, size_t SIZE) {
	if (!BUFFER || SIZE < 10) {
		return -1;
	}

	int i = 0;
	unsigned char byte = 0;

	// Find the protocol 2.0 header (FF FF FD 00).
	while (true) {
		if (!stream->readBytes(&byte, 1)) {
			return -2;
		}

		if (i == 0 && byte == 0xFF) {
			BUFFER[i++] = byte;
			continue;
		}

		if (i == 1) {
			if (byte == 0xFF) {
				BUFFER[i++] = byte;
				continue;
			}
			i = 0;
			if (byte == 0xFF) {
				BUFFER[i++] = byte;
			}
			continue;
		}

		if (i == 2) {
			if (byte == 0xFD) {
				BUFFER[i++] = byte;
				continue;
			}
			if (byte == 0xFF) {
				BUFFER[0] = 0xFF;
				BUFFER[1] = 0xFF;
				i = 2;
				continue;
			}
			i = 0;
			continue;
		}

		if (byte != 0x00) {
			i = 0;
			continue;
		}

		BUFFER[i++] = byte;
		break;
	}

	// Read id, len1, len2.
	while (i < 7) {
		if (!stream->readBytes(&BUFFER[i], 1)) {
			return -2;
		}
		i++;
	}

	int length = BUFFER[5] | (BUFFER[6] << 8);
	int packetSize = length + 7;
	if (length < 3 || packetSize > (int)SIZE) {
		return -1;
	}

	while (i < packetSize) {
		if (!stream->readBytes(&BUFFER[i], 1)) {
			return -2;
		}
		i++;
	}

	return packetSize;
}


XL330::Packet::Packet(
	unsigned char* data,
	size_t data_size,
	unsigned char id,
	unsigned char instruction,
	int parameter_data_size,
	...) {


	// [ff][ff][fd][00][id][len1][len2] { [instr][params(parameter_data_size)][crc1][crc2] }
	unsigned int length = 3 + parameter_data_size;
	if (!data) {
		// [ff][ff][fd][00][id][len1][len2] { [data(length)] }
		this->data_size = 7 + length;
		this->data = (unsigned char*)malloc(this->data_size);
		this->freeData = true;
	}
	else {
		this->data = data;
		this->data_size = data_size;
		this->freeData = false;
	}
	this->data[0] = 0xFF;
	this->data[1] = 0xFF;
	this->data[2] = 0xFD;
	this->data[3] = 0x00;
	this->data[4] = id;
	this->data[5] = length & 0xff;
	this->data[6] = (length >> 8) & 0xff;
	this->data[7] = instruction;
	va_list args;
	va_start(args, parameter_data_size);
	for (int i = 0; i < parameter_data_size; i++) {
		unsigned char arg = va_arg(args, int);
		this->data[8 + i] = arg;
	}
	unsigned short crc = update_crc(0, this->data, this->getSize() - 2);
	this->data[8 + parameter_data_size] = crc & 0xff;
	this->data[9 + parameter_data_size] = (crc >> 8) & 0xff;
	va_end(args);
}

XL330::Packet::Packet(unsigned char* data, size_t size) {
	this->data = data;
	this->data_size = size;
	this->freeData = false;
}


XL330::Packet::~Packet() {
	if (this->freeData == true) {
		free(this->data);
	}
}

void XL330::Packet::toStream(Stream& stream) {
	stream.print("id: ");
	stream.println(this->getId(), DEC);
	stream.print("length: ");
	stream.println(this->getLength(), DEC);
	stream.print("instruction: ");
	stream.println(this->getInstruction(), HEX);
	stream.print("parameter count: ");
	stream.println(this->getParameterCount(), DEC);
	for (int i = 0; i < this->getParameterCount(); i++) {
		stream.print(this->getParameter(i), HEX);
		if (i < this->getParameterCount() - 1) {
			stream.print(",");
		}
	}
	stream.println();
	stream.print("valid: ");
	stream.println(this->isValid() ? "yes" : "no");
}

unsigned char XL330::Packet::getId() {
	return data[4];
}

int XL330::Packet::getLength() {
	return data[5] + ((data[6] & 0xff) << 8);
}

int XL330::Packet::getSize() {
	return getLength() + 7;
}

int XL330::Packet::getParameterCount() {
	return getLength() - 3;
}

unsigned char XL330::Packet::getInstruction() {
	return data[7];
}

unsigned char XL330::Packet::getParameter(int n) {
	return data[8 + n];
}

bool XL330::Packet::isValid() {
	if (this->data_size < 7) {
		return false;
	}
	if (data[0] != 0xFF || data[1] != 0xFF || data[2] != 0xFD || data[3] != 0x00) {
		return false;
	}

	int length = getLength();
	int size = length + 7;
	if (length < 3 || size > (int)this->data_size) {
		return false;
	}

	unsigned short storedChecksum = data[length + 5] + (data[length + 6] << 8);
	return storedChecksum == update_crc(0, data, length + 5);
}
