#include <usbcan/can.h>
	template<> void can::CanData::set<bool>(unsigned int start, unsigned int len, Endian endian, bool value) {
		unsigned int byte = start >> 0x3;
		unsigned int bit = start & 0x7;
		if (value) {
			_data[byte] |= (1 << bit);
		}
		else {
			_data[byte] &= ~(1 << bit);
		}

	}
	template<> bool can::CanData::get<bool>(unsigned int start, unsigned int len, Endian endian) const {
		unsigned int byte = start >> 0x3;
		unsigned int bit = start & 0x7;
		return _data[byte] & (1 << bit);
	}