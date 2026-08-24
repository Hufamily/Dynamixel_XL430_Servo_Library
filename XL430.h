/*
 * Code based on:
 * Dynamixel.cpp - Ax-12+ Half Duplex USART Comunication
 * Copyright (c) 2011 Savage Electronics.
 * And Dynamixel Pro library for OpenCM-9.04 made by ROBOTIS, LTD.
 * And Luis G III for HelloSpoon robot (http://hellospoonrobot.com).
 * 
 * Modified to work with Dynamixel XL-430 actuator (originally adapted for XL-330).
 *
 * Original XL-330 modifications made by Rei Lee
 * Webpage: https://infosci.cornell.edu/~reilee/
 * Email: wl593@cornell.edu
 * 
 * This file can be used and be modified by anyone, 
 * don't forget to say thank you to OP!
 */

#ifndef XL430_H_
#define XL430_H_

/*EEPROM Area*/
#define XL430_MODEL_NUMBER_L           0
#define XL430_MODEL_NUMBER_H           1
#define XL430_MODEL_INFO               2
#define XL430_VERSION                  6
#define XL430_ID                       7
#define XL430_BAUD_RATE                8
#define XL430_RETURN_DELAY_TIME        9
#define XL430_DRIVE_MODE               10
#define XL430_CONTROL_MODE             11
#define XL430_SECONDARY_ID             12
#define XL430_PROTOCOL_TYPE            13
#define XL430_HOMING_OFFSET            20
#define XL430_MOVING_THRESHOLD         24
#define XL430_TEMPERATURE_LIMIT        31
#define XL430_MAX_VOLTAGE_LIMIT        32
#define XL430_MIN_VOLTAGE_LIMIT        34
#define XL430_PWM_LIMIT                36
#define XL430_CURRENT_LIMIT            38
#define XL430_VELOCITY_LIMIT           44
#define XL430_MAX_POSITION_LIMIT       48
#define XL430_MIN_POSITION_LIMIT       52
#define XL430_STARTUP_CONFIGURATION    60
#define XL430_PWM_SLOPE                62
#define XL430_SHUTDOWN                 63
/*RAM Area*/
#define XL430_TORQUE_ENABLE            64
#define XL430_LED                      65
#define XL430_STATUS_RETURN_LEVEL      68
#define XL430_REGISTERED_INSTRUCTION   69
#define XL430_HARDWARE_ERROR_STATUS    70
#define XL430_VELOCITY_I_GAIN      	76
#define XL430_VELOCITY_P_GAIN    		78
#define XL430_POSITION_D_GAIN    	    80
#define XL430_POSITION_I_GAIN      	82
#define XL430_POSITION_P_GAIN    		84
#define XL430_FEEDFORWARD_2ND_GAIN    	88
#define XL430_FEEDFORWARD_1ST_GAIN 	90
#define XL430_BUS_WATCHDOG			 	98
#define XL430_GOAL_PWM                 100
#define XL430_GOAL_CURRENT             102
#define XL430_GOAL_VELOCITY            104
#define XL430_PROFILE_ACCELERATION     108
#define XL430_PROFILE_VELOCITY         112
#define XL430_GOAL_POSITION            116
#define XL430_REALTIME_TICK            120
#define XL430_MOVING                   122
#define XL430_MOVING_STATUS            123
#define XL430_PRESENT_PWM              124
#define XL430_PRESENT_CURRENT          126
#define XL430_PRESENT_VELOCITY         128
#define XL430_PRESENT_POSITION         132
#define XL430_VELOCITY_TRAJECTORY      136
#define XL430_POSITION_TRAJECTORY      140
#define XL430_PRESENT_INPUT_VOLTAGE    144
#define XL430_PRESENT_TEMPERATURE      146
#define XL430_BACKUP_READY             147

/*Protocol 2.0 Instructions*/
#define XL430_INSTR_PING               0x01
#define XL430_INSTR_READ               0x02
#define XL430_INSTR_WRITE              0x03
#define XL430_INSTR_REG_WRITE          0x04
#define XL430_INSTR_ACTION             0x05
#define XL430_INSTR_FACTORY_RESET      0x06
#define XL430_INSTR_REBOOT             0x08
#define XL430_INSTR_CLEAR              0x10
#define XL430_INSTR_CONTROL_TABLE_BACKUP 0x20

#include <inttypes.h>
#include <Stream.h>

class XL430 {
private:
	unsigned char Direction_Pin;
	volatile char gbpParamEx[130+10];
	Stream *stream;
	unsigned long rx_timeout_ms;
	int last_ping_result;
	int last_ping_packet_size;
	int last_ping_tx_size;
	int last_ping_tx_written;
	unsigned char last_ping_error_byte;
	unsigned char last_ping_response_id;
	unsigned char last_ping_instruction;
	int last_ping_parameter_count;
	unsigned char last_ping_raw[32];
	int last_ping_raw_size;

  void nDelay(uint32_t nTime);
	bool readByteWithTimeout(unsigned char* byte);
	void clearRxBuffer();


public:
	XL430(); 
	virtual ~XL430();	
	
	void begin(Stream &stream);
	void setRxTimeout(unsigned long timeoutMs);
	
	void setBaudRate(int id, int value);
	void setID(int id, int value);
	void setControlMode(int id, int value);
	
	void setJointPosition(int id, int32_t value);
	void setJointSpeed(int id, int value);
	
	void LEDON(int id);
	void LEDOFF(int id);
	void TorqueON(int id);
	void TorqueOFF(int id);

	int32_t getJointPosition(int id);
	int32_t getJointSpeed(int id);
	int getJointTemperature(int id);
	int isJointMoving(int id);

	int ping(int id);
	int getLastPingResult();
	int getLastPingPacketSize();
	void printLastPingDebug(Stream &out);
	int action(int id);
	int reboot(int id);
	int factoryReset(int id, int option);
	int clearMultiTurnInfo(int id);
	int controlTableBackup(int id, int option);

	int regWrite(int id, int Address, int value);
	int regWrite_4bytes(int id, int Address, int32_t value);
	int regWrite_1byte(int id, int Address, int value);

	int sendPacket(int id, int Address, int value);
	int readPacket(unsigned char *buffer, size_t size);
	int sendPacket_4bytes(int id, int Address, int32_t value);
	int sendPacket_1byte(int id, int Address, int value);

	int RXsendPacket(int id, int Address);
	int RXsendPacket(int id, int Address, int size);

	void flush();

	class Packet {
	  bool freeData;
	  public:
	    unsigned char *data;
	    size_t data_size;

	    // wrap a received data stream in an Packet object for analysis
	    Packet(unsigned char *data, size_t size);
	    // build a packet into the pre-allocated data array
	    // if data is null it will be malloc'ed and free'd on destruction.
	    
	    Packet(
	      unsigned char *data, 
	      size_t        size,
	      unsigned char id,
	      unsigned char instruction,
	      int           parameter_data_size,
	      ...);
	    ~Packet();
	    unsigned char getId();
	    int getLength();
	    int getSize();
	    int getParameterCount();
	    unsigned char getInstruction();
            unsigned char getParameter(int n);
	    bool isValid();

	    void toStream(Stream &stream);

	};
};

#endif