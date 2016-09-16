/*-
 * Copyright (c) 2016 Landon Fuller <landonf@FreeBSD.org>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer,
 *    without modification.
 * 2. Redistributions in binary form must reproduce at minimum a disclaimer
 *    similar to the "NO WARRANTY" disclaimer below ("Disclaimer") and any
 *    redistribution must be conditioned upon including a substantially
 *    similar Disclaimer requirement for further binary redistribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF NONINFRINGEMENT, MERCHANTIBILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGES.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include "work_queue.hh"
#include <cstdio>

using namespace std;

/**
 * Create a new work queue
 * 
 * @param numWorkers Number of workers to spawn.
 */
WorkQueue::WorkQueue(size_t numWorkers)
{
	_stop = false;
	_pending = 0;

	for (size_t i = 0; i < numWorkers; i++)
		_workers.emplace_back(bind(&WorkQueue::work, this));
}

/**
 * Create a new work queue with a worker count determined via
 * std::thread::hardware_concurrency().
 * 
 * @param numWorkers Number of workers to spawn.
 */
WorkQueue::WorkQueue():
    WorkQueue(std::max<size_t>(1, std::thread::hardware_concurrency()))
{
}

WorkQueue::~WorkQueue()
{
	/* Request termination */
	{
		unique_lock<mutex> lk(_lock);
		_stop = true;
		_queue.clear();
	}
	_cv_enqueued.notify_all();

	/* Wait for all threads to exit */
	for (auto &thr : _workers) {
		if (thr.joinable())
			thr.join();
	}
}

/**
 * Enqueue a new work item.
 * 
 * @param task A task function.
 */
void
WorkQueue::push_back (const WorkItem &item) {
	std::unique_lock<std::mutex> lk(_lock);
	assert(!_stop);
	_queue.push_back(item);
	_cv_enqueued.notify_one();
}

/**
 * Wait for all outstanding work items to be completed. 
 */
void WorkQueue::wait()
{
	unique_lock<mutex> lk(_lock);
	_cv_done.wait(lk, [&]{
		if (_stop)
			return (true);

		if (!_queue.empty())
			return (false);

		if (_pending)
			return (false);

		return (true);
	});
}

/** Thread work function */
void
WorkQueue::work()
{
	unique_lock<mutex> lk(_lock);

	while (!_stop) {
		/* Wait for wake-up */
		_cv_enqueued.wait(lk, [&] {
			return (!_queue.empty() || _stop);
		});

		/* Handle stop request */
		if (_stop)
			return;

		/* Take the first item and bump the pending count */
		auto item = _queue.front();
		_queue.pop_front();
		_pending++;

		/* Perform the work without the lock held */
		lk.unlock();
		item();
		lk.lock();

		/* Work complete; discard pending refcount
		 * and notify any listeners. */
		_pending--;
		_cv_done.notify_all();
	}
}
