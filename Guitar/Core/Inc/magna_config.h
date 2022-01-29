/*
 * magna_config.h
 *
 *  Created on: Dec 16, 2021
 *      Author: Nick Nagy
 */

#ifndef INC_MAGNA_CONFIG_H_
#define INC_MAGNA_CONFIG_H_

#define USING_FREERTOS 1
#define INDEPENDENT_UI_ITEMS 0

//
//
// DEBUGGING
//
//

#define THROW_ALLOC_EXCEPTIONS 1
#define DEBUG_ADD_DIAL 1

/* overload C++ new and delete operators to make them thread-safe */
void* operator new (std::size_t count) {
	void *p;
#if USING_FREERTOS
	if (uxTaskGetNumberOfTasks())
		p = pvPortMalloc(count);
	else
		p = malloc(count);
#if THROW_ALLOC_EXCEPTIONS
		if (p == 0)
			throw std::bad_alloc();
#endif
#else
	p = malloc(count);
#endif
	return p;
}

void operator delete(void* ptr) noexcept {
#if USING_FREERTOS
	if (uxTaskGetNumberOfTasks())
		vPortFree(ptr);
	else
		free(ptr);
#else
	free(ptr);
#endif
	ptr = NULL;
}

void *operator new[](std::size_t count) {
	void *p;
#if USING_FREERTOS
	if (uxTaskGetNumberOfTasks())
		p = pvPortMalloc(count);
	else
		p = malloc(count);
#if THROW_ALLOC_EXCEPTIONS
		if (p == 0)
			throw std::bad_alloc();
#endif
#else
	p = malloc(count);
#endif
	return p;
}

void operator delete[](void * ptr) noexcept {
#if USING_FREERTOS
	if (uxTaskGetNumberOfTasks())
		vPortFree(ptr);
	else
		free(ptr);
#else
	free(ptr);
#endif
	ptr = NULL;
}

#endif /* INC_MAGNA_CONFIG_H_ */
