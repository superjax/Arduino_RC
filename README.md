# Description

RC_Read is an arduino-ros node that receives servo signals from a standard transmitter, packages them into a ROS message, and broadcasts them to a ROS server.  This package uses the example code MultiChannels by Duane B. at www.rcaudio.com and uses the PinChangeInt Library, which allows all 20 pins on an arduino uno to be interrupt controlled via change notifications.

This was demonstrated to work with a Naze32 Autopilot on a 600mm Hexacopter with an i7 running ROS on board.

This node publishes a non-standard message found in BYU's MAGICC lab Relative Navigation code stack.  If you decide to use this node, and require this message, request information from the maintainers and we can help you get this message.  However, it shouldn't be very hard to create your own custom message for your application.  

# License
Copyright (c) 2015, James Jackson, Paul Nyholm, All rights reserved

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.