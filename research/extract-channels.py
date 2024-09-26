#! /usr/bin/env python3
# Test script to see if our guesswork on Alesis bulk transfer format is correct..
# use as a parser after extracting appropriate capture fields with tshark:
#
# tshark -r <pcap file> -Y 'usb.endpoint_address == 0x86 and usb.dst == host' \
#   -T fields -e usb.capdata | ./extract-channels.py <base name>
#
# where <base name> will be used to form channel file names: <base name><n>.wav

import sys, struct, wave

BITS=24
CHNS=10

outs = []

def frame(n, l):
    # the fun bit.. yank channel bits across each byte and reassemble sample words
    s = [0, 0, 0, 0, 0]
    for i in range(BITS):
        for b in range(5):
            s[b] <<= 1
            s[b] |= 1 if ((l[i]>>b) & 1)>0 else 0
    # emit the samples for each channel
    for ch in range(5):
        v = s[ch]
        c = ch+5 if n else ch
        outs[c][0].extend(struct.pack('<BBB', (v)&0xff, (v>>8)&0xff, (v>>16)&0xff))

def extract(base, strm):
    # open output files..
    for i in range(CHNS):
        out = wave.open(f'{base}{i}.wav', mode='wb')
        buf = bytearray()
        out.setnchannels(1)
        out.setsampwidth(3)
        out.setframerate(96000)
        outs.append((buf,out))
    n = 0
    l = []
    for line in strm:
        # ignore comments
        if line.startswith('#'):
            continue
        # detect standard hexdump (<addr> <hex> <hex> ...) and fixup
        line = line.strip()
        o = line.find(' ')
        if o>=0:
            line = line[o:].replace(' ', '')
        # chunk up the line into 2 char (hex) bytes..
        for h in (line[0+i:2+i] for i in range(0, len(line), 2)):
            l.append(int(h, 16))
            if len(l) >= 32:
                frame((n & 1) > 0, l)
                n += 1
                l.clear()
        # end of a line (USB bulk transfer), flush buffers to disk
        for o in outs:
            o[1].writeframes(o[0])
            o[0].clear()
        print(n)
    for o in outs:
        o[1].close()

if __name__ == "__main__":
    if len(sys.argv)<2:
        print(f'usage: {sys.argv[0]} <base name> [<file>]')
        sys.exit(0)
    base = sys.argv[1]
    if len(sys.argv)>2:
        with open(sys.argv[2], "r") as f:
            extract(base, f)
    else:
        extract(base, sys.stdin)
