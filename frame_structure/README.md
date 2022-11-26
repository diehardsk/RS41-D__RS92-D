# RS92-DL (DD)
Frame structure is same as [RS92-SGP](https://brmlab.cz/project/weathersonde/telemetry_decoding) except GPS data block which is omited.
[rs1729 decoder](https://github.com/rs1729/RS/blob/master/demod/mod/rs92mod.c) as well as [SondeMonitor](https://www.coaa.co.uk/sondemonitor.htm) does good job receiving all telemetry.

# RS41-D
Unlike other members of RS41 family, RS41-D use Manchester encoding (instead of data whitening) and data bytes are sent in form of asynchonous serial protocol (1 start bit, 8 bits of data, 1 stop bit). Transmition is continual. All these properties are shared with previous generation - RS92-D.
However frame format is more similiar to other RS41 models.

![RS41-D_frame](RS41-D_frame.png?raw=true "Randomly selected RS41-D frame")

| Offset | Size[bytes] | Description                            |
| ------ | ----------- | -------------------------------------- |
| 0x00   | 6           | Header (0xDB 0xDB 0xDB 0xDB 0xDB 0x64) |
| 0x06   | 24          | Reed-Solomon error correction block    |
| 0x1E   | 210         | Frame payload                          |

### Header
Frame header sequence is fixed. It's used for frame start synchronizing. 

### Reed-Solomon block
- whole input data byte order is reversed
- RS(255, 231)
- RS correction code bytes are also reversed
- Generator polynomial: 285 (0x11d)
- First consecutive root: 0

## Frame payload data
One frame contains several (3) blocks of data. Each of these have their own header and CRC.
| Offset          | Size[bytes] | Description             |
| --------------- | ----------- | ----------------------- |
| 0               | 1           | Block ID                |
| 1               | 1           | data length             |
| 2               | data length | block payload data      |
| 2 + data length | 2           | CRC16 of payload data   |

RS41-D is sending 3 data blocks inside one frame. (one frame takes one second)
| Block ID | Size[bytes] | Description                       |
| -------- | ----------- | --------------------------------- |
| 0x79     | 0x28 (40)   | Status and calib/config fragment  |
| 0x7A     | 0x2A (42)   | Measurement data                  |
| 0x76     | 0x74 (116)  | Padding only                      |

You can find detail description of RS41 data blocks [@bajzo/RS41_Decoding](https://github.com/bazjo/RS41_Decoding/tree/master/RS41-SGP)
