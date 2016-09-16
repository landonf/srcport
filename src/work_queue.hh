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
 * 
 * $FreeBSD$
 */

#ifndef _SRCPORT_WORK_QUEUE_HH_
#define _SRCPORT_WORK_QUEUE_HH_

#include <cassert>

#include <deque>
#include <future>
#include <thread>
#include <vector>

/**
 * A simple threaded work queue.
 * 
 * @tparam W The type of work unit enqueued.
 * @tparam Fn The worker callback function type.
 */
class WorkQueue {
public:
	using WorkItem = std::function<void()>;

	WorkQueue (size_t numWorkers);
	WorkQueue ();

	/* disable copy and move constructors */
	WorkQueue (const WorkQueue &) = delete;
	WorkQueue (WorkQueue &&) = delete;

	void push_back (const WorkItem &item);

	void wait ();
	~WorkQueue ();
	
private:
	void work ();

	std::vector<std::thread>	_workers;	/**< worker threads */
	bool				_stop;		/**< if true, worker threads must exit */
	std::deque<WorkItem>		_queue;		/**< enqueued work items */
	size_t				_pending;	/**< number of in-flight executing tasks */

	std::mutex			_lock;		/**< lock over all shared state */
	std::condition_variable		_cv_enqueued;	/**< condition variable for work enqueue */
	std::condition_variable		_cv_done;	/**< condition variable for work completion (e.g. completed one work item) */
};

#endif /* _SRCPORT_WORK_QUEUE_HH_ */
