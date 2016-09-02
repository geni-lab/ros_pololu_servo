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


#include <polstro/PolstroSerialInterfacePOSIX.h>

#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>

#ifdef _WIN32
#define O_NOCTTY 0
#else
#include <termios.h>
#endif

#include <errno.h>  

namespace Polstro
{
/* The port name can be:
	- Windows : "\\\\.\\USBSER000", "\\\\.\\COM6", etc...
	- Linux : "/dev/ttyACM0"
	- Mac OS X : "/dev/cu.usbmodem00034567"
*/
SerialInterfacePOSIX::SerialInterfacePOSIX( const std::string& portName )
	:	SerialInterface(),
		mFileDescriptor(-1)
{
	mFileDescriptor = openPort( portName );
}

SerialInterfacePOSIX::~SerialInterfacePOSIX()
{
	if ( isOpen() )
	{
		// Before destroying the interface, we "go home"
		goHomeCP();

		close( mFileDescriptor );
	}
	mFileDescriptor = -1;
}

bool SerialInterfacePOSIX::isOpen() const
{
	return mFileDescriptor!=-1;
}

int SerialInterfacePOSIX::openPort( const std::string& portName )
{
	int fd = open( portName.c_str(), O_RDWR | O_NOCTTY );
	if (fd == -1)
	{
		perror(portName.c_str());
		return -1;
	}

#ifndef _WIN32
	struct termios options;
	tcgetattr(fd, &options);
	options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	options.c_oflag &= ~(ONLCR | OCRNL);
	tcsetattr(fd, TCSANOW, &options);
#endif

	return fd;
}

bool SerialInterfacePOSIX::writeBytes( const unsigned char* data, unsigned int numBytesToWrite )
{
	if ( !isOpen() )
		return false;

	// See http://linux.die.net/man/2/write
	ssize_t ret = write( mFileDescriptor, data, numBytesToWrite );
	if ( ret==-1 )
	{
		printf("Error writing. errno=%d\n", errno );
		return false;
	}
	else if ( ret!=numBytesToWrite )
	{
		printf("Error writing. Wrote %ld bytes instead of %d\n", ret, numBytesToWrite );
		return false;
	}

	return true;
}

bool SerialInterfacePOSIX::readBytes( unsigned char* data, unsigned int numBytesToRead )
{
	if ( !isOpen() )
		return false;

	// See http://linux.die.net/man/2/read
	ssize_t ret = read( mFileDescriptor, data, numBytesToRead );
	if ( ret==-1 )
	{
		printf("Error reading. errno=%d\n", errno );
		return false;
	}
	else if ( ret!=numBytesToRead )
	{
		printf("Error reading. Read %ld bytes instead of %d\n", ret, numBytesToRead );
		return false;
	}
	return true;
}

};
