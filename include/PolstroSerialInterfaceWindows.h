#pragma once

#include "PolstroSerialInterface.h"

#include <windows.h>

namespace Polstro
{

class SerialInterfaceWindows : public SerialInterface
{
public:
	SerialInterfaceWindows( const std::string& portName, unsigned int baudRate );
	virtual ~SerialInterfaceWindows();

	virtual bool isOpen() const;

private:
	static HANDLE openPort( const std::string& portName, unsigned int baudRate );
	
	virtual bool writeBytes( const unsigned char* data, unsigned int dataSizeInBytes );
	virtual bool readBytes( unsigned char* data, unsigned int dataSizeInBytes );

	HANDLE	mPortHandle;
};

}