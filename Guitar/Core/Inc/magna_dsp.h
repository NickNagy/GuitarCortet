/*
 * magna_dsp.h
 *
 *  Created on: Oct 28, 2021
 *      Author: Nick Nagy
 */

#ifndef INC_MAGNA_DSP_H_
#define INC_MAGNA_DSP_H_

#include <stdint.h>

namespace magna {


	template <class T> class zRegister {
	private:
		T * queue;
		int length;
	public:
		zRegister(int length) {
			this->length = length;
			queue = new T[length];
			for (int i = 0; i < length; i++) {
				queue[i] = 0;
			}
		}
		~zRegister() {
			delete[] queue;
		}
		void push(T data) {
			for (int i = 0; i < this->length-1; i++) {
				queue[i] = queue[i+1];
			}
			queue[length-1] = data;
		}
		T at(int idx) { return queue[idx]; }
		int getLength() { return length; }
	};

	template <class T> class FIR {
	private:
		zRegister<T> * zRegs;
		//int bufferLength;
		int windowSize;
		T sum;
	public:
		FIR(int windowSize) {
			zRegs = new zRegister<T>(windowSize);
			this->windowSize = windowSize;
			sum = 0;
		}
		~FIR() {
			delete[] zRegs;
		}
		T process(T xn, T yn) {
			// subtract element being removed from queue from sum; add the element about to be added
			// divide before adding to avoid overflow
			sum += xn/windowSize - zRegs->at(0)/windowSize;
			zRegs->push(xn);
			yn = sum;
			return yn;
		}
	};

};

#endif /* INC_MAGNA_DSP_H_ */
