/*
 * SingleMutex.h
 *
 *  Created on: Oct 22, 2009
 *      Author: martinc
 */

#ifndef SINGLEMUTEX_H_
#define SINGLEMUTEX_H_

#include <pthread.h>

/// A mutex which will appear as a singleton in the class in which it is instantiated.
class SingleMutex {
public:
	SingleMutex();
	virtual ~SingleMutex();
    static SingleMutex* create();
	static void lock();
	void unlock();
	static SingleMutex* _instance;
	static pthread_mutex_t* _m;
};


#endif /* SINGLEMUTEX_H_ */
