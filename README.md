MavlinkSerialSdLogger
=====================

External SD logger for the MavLink protocol

This code is based on the https://code.google.com/p/arducam-osd/ arducam-osd code
It requests all messages streams to the main board to log raw data in a file on the SD card.
It was developped and tested with an arduino micro http://arduino.cc/en/Main/arduinoBoardMicro as target, and the sd shield from adafruit http://www.adafruit.com/products/254

Algorithm:
  startup
    init sd card
    find out the name of the log file to use (one file by power-up sequence) (ie: TX.log, where X is a number)
    waiting for heartbeats
    request all datastream
  loop
    Write all raw data to the log file
    when complete messages are received, an action may be executed (it's up to you)

Each .bin file must be renamed to .tlog to be read by MissionPlanner. You should then be able to replay all your flight!


Credits http://lauters.fr
