# linvinus_serial_protocol

Simple, multi platform binary protocol with low overhead and delivery guarantee.

## Quick description


Simple, multi platform binary protocol with very low overhead and delivery guarantee.
For easy communications between controllers over UART or SPI(not implemented yet)

 __Overhead:__
 * 1 start byte
 * 4 bytes header,
 * 1 additional byte for cobs encoding
 * total: 6bytes

minimal packet size is 6 bytes including 2bytes for user payload data (see fast message section)

Protocol is bidirectionall, host or controller may send any message in any time,

protocol message contain header + body, max body size is limited to 246 bytes.

body may contain any byte

4bits counter, command ID and body size is used for commands identification and synchronization

Body content is protected with checksum, messages with wrong checksum is rejected with error notification message.

GET commands is used to receive data from remote side

SET commands is used to set data on remote side, may be with confirmation or without confirmation

For confirmation or error notification purpose, special "system messages" are used.


To simplify user protocol, special array is used to store all commands (SD_CMDS[]),

this array must be the same on both sides of connection.

