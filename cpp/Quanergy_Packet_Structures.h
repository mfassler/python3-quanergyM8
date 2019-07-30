#ifndef __QUANERGY_PACKET_STRUCTURE_H
#define __QUANERGY_PACKET_STRUCTURE_H

/*
 *  Please refer to the "M8 Sensor User Guide",
 *   - chapter 5, "Getting TCP Ethernet Packets"
 *
 */


#include <stdint.h>

uint32_t M8_SIGNATURE = 0x75bd7e97;  // Magic start constant for every M8 packet

const int M8_FIRING_PER_PKT = 50;
const int M8_NUM_RETURNS = 3;
const int M8_NUM_LASERS = 8;


// The 8 beams are separated by 3.20 degrees
// angle #0 is the lowest downward-loooking beam
// These are in radians:

/*
const double M8_VERTICAL_ANGLES[] = {
	-0.318505,
	-0.2692,
	-0.218009,
	-0.165195,
	-0.111003,
	-0.0557982,
	0.f,
	0.0557982
};
*/


struct M8FiringData {
	uint16_t position; //    10400 steps per rotation
	uint16_t padding;
	uint32_t returns_distances[M8_NUM_RETURNS][M8_NUM_LASERS];   // 10 um resolution
	uint8_t  returns_intensities[M8_NUM_RETURNS][M8_NUM_LASERS]; // 255 indicates saturation
	uint8_t  returns_status[M8_NUM_LASERS];
}; // 132 bytes


// This is packet_type 0x00:
struct M8DataPacket {
	struct M8FiringData  data[M8_FIRING_PER_PKT];
	uint32_t seconds;     // seconds from Jan 1 1970
	uint32_t nanoseconds; // fractional seconds turned to nanoseconds
	uint16_t version;     // API version number.  Version 5 uses distance as units of 10 micrometers, <5 is 10mm
	uint16_t status;      // 0: good, 1: Sensor SW/FW mismatch

}; // 6612 bytes


struct M8PacketHeader {
	uint32_t signature;
	uint32_t size;
	uint32_t seconds;
	uint32_t nanoseconds;
	uint8_t version_major;
	uint8_t version_minor;
	uint8_t version_patch;
	uint8_t packet_type;
};  // 20 bytes for header


/* packet_type:

    The "M8 Sensor User Guide" only defines
      0x00 - multi-return data (the default, I guess)
      0x04 - single-return data (smaller packets)

    The source code from quanergy_client also includes another:
      0x01 - ... seems to have a more free-form definition of x/y angles...
 */


#endif // __QUANERGY_PACKET_STRUCTURE_H
