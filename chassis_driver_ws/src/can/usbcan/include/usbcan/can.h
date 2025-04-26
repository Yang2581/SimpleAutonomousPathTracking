#pragma once
#include<vector>
#include<algorithm>
#include<climits>
#include<stdint.h>
#include <iostream>
#include <fstream>
#include <limits>
#undef LITTLE_ENDIAN
#undef BIG_ENDIAN
using namespace std;
namespace can {
	class CanData {
	private:
		std::vector<unsigned char> _data;
	public:
		enum Endian {
			LITTLE_ENDIAN, BIG_ENDIAN
		};
	public:
		explicit CanData(unsigned int len) :
				_data(len, 0) {

		}
		CanData(unsigned char* data, unsigned int len) :
				_data(len) {
			std::copy(data, data + len, _data.begin());
		}
		CanData(const CanData & other) :
				_data(other._data) {

		}
	public:
		std::vector<unsigned char> & data() {
			return _data;
		}
		const std::vector<unsigned char> & data() const {
			return _data;
		}
		template<typename T> void set(unsigned int start, unsigned int len,
				Endian endian, T value) {
			static_assert(std::numeric_limits<T>::is_integer, "T must be integer type");
			unsigned int byte = start >> 0x3;
			unsigned int bit = start & 0x7;

			for (unsigned int i = 0; i < len; i++) {
				T mask = 0x1 << i;
				if (value & mask) {
					_data[byte] |= (1 << bit);
				} else {
					_data[byte] &= ~(1 << bit);
				}
				bit++;
				if (bit >= 8) {
					bit = 0;
					byte += (endian == BIG_ENDIAN ? -1 : 1);
				}
			}

		}
		template<typename T> T get(unsigned int start, unsigned int len,
				Endian endian) const {
			static_assert(std::numeric_limits<T>::is_integer, "T must be integer type");
			T ret = T();
			unsigned int byte = start >> 0x3;
			unsigned int bit = start & 0x7;
			unsigned int temp,star = 0, endf = sizeof(_data)/sizeof(_data[0]-1);
			for (unsigned int i = 0; i < len; i++) {
				T mask = 0x1 << i;
				if (_data[byte] & (1 << bit)) {
					ret |= mask;
				} else {
					ret &= ~mask;
				}
				bit++;

				if (bit >= 8) {
					bit = 0;
					byte += (endian == BIG_ENDIAN ? -1 : 1);
				}	
			}

			if (std::numeric_limits<T>::is_signed) {

				T sign = ret & (1 << (len - 1));
				for (unsigned int i = len; i < sizeof(T) * CHAR_BIT; i++) {
					sign <<= 1;
					ret |= sign;
				}

			}
			return ret;

		}

	};
	template<> void CanData::set<bool>(unsigned int start, unsigned int len,
			Endian endian, bool value);
	template<> bool CanData::get<bool>(unsigned int start, unsigned int len,
			Endian endian) const;

	class CanMessage {
	private:
		uint32_t _id;
		CanData _data;
	public:
		CanMessage(uint32_t id, CanData & data) :
				_id(id), _data(data) {
		}
	public:
		uint32_t id() {
			return _id;
		}
		CanData & data() {
			return _data;
		}
		const CanData & data() const {
			return _data;
		}
	};
}
