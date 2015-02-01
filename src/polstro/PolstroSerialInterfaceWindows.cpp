/*Copyright (c) 2013 Jacques Menuet

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/


#include "PolstroSerialInterfaceWindows.h"

#include <stdio.h>

namespace Polstro
{

SerialInterfaceWindows::SerialInterfaceWindows( const std::string& portName, unsigned int baudRate )
	:	SerialInterface(),
		mPortHandle(NULL)
{
	mPortHandle = openPort( portName, baudRate );
}

SerialInterfaceWindows::~SerialInterfaceWindows()
{
	if ( isOpen() )
	{
		// Before destroying the interface, we "go home"
		goHomeCP();
		
		CloseHandle( mPortHandle );
	}
	mPortHandle = NULL;
}

bool SerialInterfaceWindows::isOpen() const 
{ 
	return mPortHandle!=INVALID_HANDLE_VALUE; 
}

/* Opens a handle to a serial port in Windows using CreateFile.
		portName: The name of the port.
		baudRate: The baud rate in bits per second.
   Returns INVALID_HANDLE_VALUE if it fails.  Otherwise returns a handle to the port.
   Examples: "COM4", "\\\\.\\USBSER000", "USB#VID_1FFB&PID_0089&MI_04#6&3ad40bf600004#  */
HANDLE SerialInterfaceWindows::openPort( const std::string& portName, unsigned int baudRate)
{
	HANDLE port;
	DCB commState;
	BOOL success;
	COMMTIMEOUTS timeouts;

	// Open the serial port
	port = CreateFileA(portName.c_str(), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
	if (port == INVALID_HANDLE_VALUE)
	{
		switch(GetLastError())
		{
		case ERROR_ACCESS_DENIED:	
			fprintf(stderr, "Error: Access denied.  Try closing all other programs that are using the device.\n");
			break;
		case ERROR_FILE_NOT_FOUND:
			fprintf(stderr, "Error: Serial port not found.  "
				"Make sure that \"%s\" is the right port name.  "
				"Try closing all programs using the device and unplugging the "
				"device, or try rebooting.\n", portName.c_str());
			break;
		default:
			fprintf(stderr, "Error: Unable to open serial port.  Error code 0x%x.\n", GetLastError());
			break;
		}
		return INVALID_HANDLE_VALUE;
	}
	
	// Set the timeouts
	success = GetCommTimeouts(port, &timeouts);
	if (!success)
	{
		fprintf(stderr, "Error: Unable to get comm timeouts.  Error code 0x%x.\n", GetLastError());
		CloseHandle(port);
		return INVALID_HANDLE_VALUE;
	}
	timeouts.ReadIntervalTimeout = 1000;
	timeouts.ReadTotalTimeoutConstant = 1000;
	timeouts.ReadTotalTimeoutMultiplier = 0;
	timeouts.WriteTotalTimeoutConstant = 1000;
	timeouts.WriteTotalTimeoutMultiplier = 0;
	success = SetCommTimeouts(port, &timeouts);
	if (!success)
	{
		fprintf(stderr, "Error: Unable to set comm timeouts.  Error code 0x%x.\n", GetLastError());
		CloseHandle(port);
		return INVALID_HANDLE_VALUE;
	}

	// Set the baud rate
	success = GetCommState(port, &commState);
	if (!success)
	{
		fprintf(stderr, "Error: Unable to get comm state.  Error code 0x%x.\n", GetLastError());
		CloseHandle(port);
		return INVALID_HANDLE_VALUE;
	}
	commState.BaudRate = baudRate;
	success = SetCommState(port, &commState);
	if (!success)
	{
		fprintf(stderr, "Error: Unable to set comm state.  Error code 0x%x.\n", GetLastError());
		CloseHandle(port);
		return INVALID_HANDLE_VALUE;
	}

	// Flush out any bytes received from the device earlier
	success = FlushFileBuffers(port);
	if (!success)
	{
		fprintf(stderr, "Error: Unable to flush port buffers.  Error code 0x%x.\n", GetLastError());
		CloseHandle(port);
		return INVALID_HANDLE_VALUE;
	}

	return port;
}

bool SerialInterfaceWindows::writeBytes( const unsigned char* data, unsigned int numBytesToWrite )
{
	if ( !isOpen() )
		return false;

	DWORD bytesTransferred = 0;
	BOOL success = WriteFile( mPortHandle, data, numBytesToWrite, &bytesTransferred, NULL );
	if ( !success )
	{
		fprintf(stderr, "Error: Unable to write Set Target command to serial port.  Error code 0x%x.", GetLastError());
		return false;
	}
	if ( numBytesToWrite!=bytesTransferred )
	{
		fprintf(stderr, "Error: Expected to write %d bytes but only wrote %d.", numBytesToWrite, bytesTransferred);
		return false;
	}
	return true;
}

bool SerialInterfaceWindows::readBytes( unsigned char* data, unsigned int numBytesToRead )
{
	if ( !isOpen() )
		return false;

	DWORD bytesTransferred = 0;
	BOOL success = ReadFile( mPortHandle, data, numBytesToRead, &bytesTransferred, NULL );
	if ( !success )
	{
		fprintf(stderr, "Error: Unable to read Get Position response from serial port.  Error code 0x%x.", GetLastError());
		return false;
	}
	if ( numBytesToRead!=bytesTransferred )
	{
		fprintf(stderr, "Error: Expected to read %d bytes but only read %d (timeout). "
			"Make sure the Maestro's serial mode is USB Dual Port or USB Chained.", numBytesToRead, bytesTransferred);
		return false;
	}

	return true;
}

};
