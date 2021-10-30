/*
 * magna_dsp.h
 *
 *  Created on: Oct 28, 2021
 *      Author: Nick Nagy
 */

#ifndef INC_MAGNA_DSP_H_
#define INC_MAGNA_DSP_H_

#include <stdint.h>
#include <memory>
#include <deque>

namespace magna {


	/*template <class T> class zRegister {
	private:
		std::unique_ptr<std::dequeue<T>> queue;
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
	};*/

	/* TODO: base class */

	template <class T> class FIR_LPF {
	private:
		//zRegister<T> * zRegs;
		//int bufferLength;
		std::unique_ptr<std::deque<T>> zRegs;
		int windowSize;
		T sum;
	public:
		FIR_LPF(int windowSize) {
			zRegs = std::make_unique<std::deque<T>>();
			this->windowSize = windowSize;
			zRegs->resize(windowSize);
			sum = 0;
		}
		~FIR_LPF() {}
		T process(T xn, T yn) {
			// subtract element being removed from queue from sum; add the element about to be added
			// divide before adding to avoid overflow
			sum += xn/windowSize - zRegs->pop_front(0)/windowSize;
			zRegs->push_back(xn);
			yn = sum;
			return yn;
		}
	};

	template <class T> class FIR_CombFilter {
	private:
		//std::unique_ptr<std::deque<T>> zRegs;
		T * zRegs;
		int fs;
		int currDataPtr;
		int MDataPtr;
		int M;
		int offset;
		float b0, bM;

	public:
		FIR_CombFilter(int fs, int M, float b0, float bM) {
			this->fs = fs;
			this->b0 = 1.0f - bM; //b0;
			this->bM = bM;
			this->M = M;
			zRegs = new T[M];//std::make_unique<std::deque<T>>();
			for (int i = 0; i < M; i++) {
				zRegs[i] = 0;
			}
			currDataPtr = M;
			MDataPtr = 0;
			//setDelay(M);
		}
		~FIR_CombFilter() {
			delete[] zRegs;
		}

		/*void setDelay(int M) {
			//M = fs/1000 * delayInMillis;
			this-> M = M;
			int prevLength = zRegs->size();
			if (prevLength < M) {
				//zRegs->resize(M);
				offset = 0;
			} else {
				offset = prevLength - M;
			}
		}*/

		T process(T xn, T yn) {
			//yn = (T)(bM*zRegs->at(offset) + b0*xn);
			//zRegs->push_back(xn);
			//zRegs->pop_front();
			yn = (T)(b0*xn) + (T)(bM*zRegs[MDataPtr]);
			zRegs[currDataPtr] = xn;
			currDataPtr = (currDataPtr + 1) % M;
			MDataPtr = (MDataPtr + 1) % M;
			return yn;
		}
	};

};

#endif /* INC_MAGNA_DSP_H_ */
