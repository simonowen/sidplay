#!/usr/bin/env python

import glob
import struct
import argparse

parser = argparse.ArgumentParser("Display SID file details")
parser.add_argument("sidfiles", action='append')
args = parser.parse_args()

for wildcard in args.sidfiles:
    for filename in glob.glob(wildcard):
        with open(filename, 'rb') as sidfile:
            siddata = sidfile.read()

        HEADER_FMT = '>4s7HL32s32s32s'
        header_size = struct.calcsize(HEADER_FMT)
        sig, ver, offset, load, init, play, songs, start, speed, title, author, copy = \
            fields = struct.unpack(HEADER_FMT, siddata[:header_size])

        sig, title, author, copy = \
            [x.decode(errors='ignore').strip(' \0') for x in (sig, title, author, copy)]

        if sig not in ['PSID', 'RSID']:
            print("warning: {} is not a SID file".format(filename))
            continue

        if load == 0:
            load, = struct.unpack("H", siddata[header_size:header_size+2])

        print('''[{}]
  Signature:   {}
  Version:     {}
  Data offset: {}
  Load addr:   {:04X}
  Init addr:   {:04X}
  Play addr:   {:04X}
  Songs:       {}
  Start song:  {}
  Speed:       {:016b}
  Title:       {}
  Author:      {}
  Copyright:   {}
'''.format(filename, sig, ver, offset, load, init, play, songs, start, speed, title, author, copy))
