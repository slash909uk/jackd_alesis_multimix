# Alesis USB2.0 FX protocol reversing

This is a collection of experimental work, tooling and notes while researching the undocumented,
non-standard Alesis mixing desk multi-channel audio over USB protocol, with a view to creating an
open source driver (initially user-space, then likely Linux kernel).

## Stuff

 * bulk.txt : Stuart's notes so far
 * bulk-hex-sample.txt: Phil's notes on the bulk transfers, and
   a hexdump saved from Wireshark for experimenting with
 * `extract-channels.py` a hex dump parser that follows the guesswork
   to create a set of `.wav` files, one per channel.

