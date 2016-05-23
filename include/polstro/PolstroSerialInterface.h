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

#pragma once

#include <string>

namespace Polstro
{

/*
	The SerialInterface is the object used to communicate with one or
	more Maestro devices.  At creation time, the interface opens the
	specified port at the given baud rate.  If succesfull, it is possible
	after that to issue commands to the device(s) on that port.  Devices
	can be chained one to the other. The chain is connected to a single port.

	For detailled explanations about the commands, see the Pololu documentation
		http://www.pololu.com/docs/0J40/5.e
*/
class SerialInterface
{
public:
	SerialInterface();
	virtual ~SerialInterface();

	virtual bool isOpen() const = 0;

	static unsigned int getMinChannelValue()  { return mMinChannelValue; }
	static unsigned int getMaxChannelValue()  { return mMaxChannelValue; }

	// The target is given in units of 0.25�s
	bool setTargetCP( unsigned char channelNumber, unsigned short target );
	bool setTargetPP( unsigned char deviceNumber, unsigned char channelNumber, unsigned short target );

	// The normalizedTarget is between 0 and 255 and the actual value
	// it represents in �s depends on the neutral and range configured
	// for the channel of the device (and stored on the device).
	// This allow calibration to be done externally once (using the
	// Maestro Control Center for example) and have the application
	// manipulate values that are "good".  The miniSCC channel number
	// allows access to channels of chained devices (see doc)
	bool setTargetMSSCP( unsigned char miniSCCChannelNumber, unsigned char normalizedTarget );

	// On Mini Maestro 12, 18 and 24 only, so not supported here
	// bool setMultipleTargets(...)

	// The speed limit is given in units of (0.25�s)/(10ms)
	bool setSpeedCP( unsigned char channelNumber, unsigned short speed );
	bool setSpeedPP( unsigned char deviceNumber, unsigned char channelNumber, unsigned short speed );

	// The acceleration limit is a value from 0 to 255 in units of
	//(0.25�s)/(10ms)/(80ms)
	bool setAccelerationCP( unsigned char channelNumber, unsigned char acceleration );
	bool setAccelerationPP( unsigned char deviceNumber, unsigned char channelNumber, unsigned char acceleration );

	// For a servo channel, the position is the current pulse width
	// in units of 0.25�s For a digital output channel, a position
	// less than 6000 means the line is low, and above 6000 it's high
	// For an input channel, the position represents the voltage measure
	// on the channel (see doc)
	bool getPositionCP( unsigned char channelNumber, unsigned short& position );
	bool getPositionPP( unsigned char deviceNumber, unsigned char channelNumber, unsigned short& position );

	bool getMovingStateCP( bool& servosAreMoving );
	bool getMovingStatePP( unsigned char deviceNumber, bool& servosAreMoving );

	bool getErrorsCP( unsigned short& error);
	bool getErrorsPP( unsigned char deviceNumber, unsigned short& error );

	// The "go home" action set the channels to their startup/error state.
	// This state is defined on a per-channel. It can either be:
	// - ignore: the value is unchanged when we do a "go home". The PWM
	//   signal is continues to be generated
	// - go to: the channel is set to the specified value. Again the
	//   PWM signal is generated
	// - off: the channel is turned off. There's no more PWM signal
	//   generated for the channel
	bool goHomeCP();
	bool goHomePP( unsigned char deviceNumber );

	static SerialInterface* createSerialInterface( const std::string& portName, unsigned int baudRate );

private:
	static const unsigned int mMinChannelValue = 3240;
	static const unsigned int mMaxChannelValue = 8700;

	virtual bool writeBytes( const unsigned char* data, unsigned int dataSizeInBytes ) = 0;
	virtual bool readBytes( unsigned char* data, unsigned int dataSizeInBytes ) = 0;
};


}
