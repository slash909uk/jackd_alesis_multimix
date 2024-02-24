This repo contains a user-space adapter to interface the Alesis Multimix USB 2.0 external mixing desk with Linux Jack.

The Alesis mixer supports 8 input channels plus stereo mix bus in and out via USB 2.0 using propietary protocols (NOT USB AUDIO CLASS COMPLIANT!)
This mixer was sold from ~2006 onward and has now been EOL'd by Alesis. 
There never existed any Linux support, only Windows and MacOS X drivers written by Ploytec GmbH (not Alesis themselves.)

Annoyed with this situation, [@phlash](https://github.com/phlash/phlash) and myself have reverse engineered the protocol and we are developing
a user-space Linux adapter that works with Jack.

This code depends on libm, libusb-1.0 and libjack. You'll need dev packages for these installed to compile it!

Compile me:

gcc alesis_jackd_plugin.c /usr/lib/x86_64-linux-gnu/libjack.so /usr/lib/x86_64-linux-gnu/libm.so /usr/lib/x86_64-linux-gnu/libusb-1.0.so  -o jackd_alesis_multimix

usage: ./jackd_alesis_multimix <client name> [-v|-vv]
(start jackd first!)

./jackd_alesis_multimix alesus

output:

OUT: drop:00000463 add:00001232 fb:-003 rbdata:00000324 IN: drop:00000549 add:00003108 ibdata:00001023

The status line updates at about 10Hz with statistics:
OUT - the stereo USB path from computer to Multimix
IN - the 10 channel USB path from Multimix to computer
drop: number of frames of audio dropped
add: number of frames of audio duplicated+added
fb: clock sync error feedback from Multimix
[x]bdata: frames stored in ring buffer

The code uses ring buffers and a simple add/drop frame method to manage the fact that the computer and mixer clocks run independantly
and further that the mixer gives feedback on the stereo bus output of frames required.

Given the clocks should be very close to each other, we don't bother with more sophisticated resampling methods. Feel free to add them if you like!

Tested on ONE computer running Ubuntu realtime kernel v22.04.1, Jack and Ardour.
