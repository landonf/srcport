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

#ifndef _SRCPORT_RESULT_HH_
#define _SRCPORT_RESULT_HH_

#include <ftl/either.h>

#include "error.hh"

/** Disjoint union representing either an Error, or a valid result of
 *  type @p T */
template<typename T> using result = ftl::either<Error, T>;

template<typename T> result<T> failed (const Error &error) {
	return (result<T>{ftl::constructor<Error>(), error});
}

/**
 * Produce an error result<T> value with the given message.
 */
template<typename T> result<T>
fail(const std::string &msg)
{
	return ftl::make_left<T>(Error(msg, ENXIO));
}

/**
 * Produce an error result<T> value with the given errno value.
 */
template<typename T> result<T>
fail(int err)
{
	return ftl::make_left<T>(Error(err));
}

/**
 * Produce a successful result<T> value.
 */
template<typename T> auto
yield(T &&t) -> decltype(ftl::make_right<Error>(std::forward<T>(t)))
{
	return ftl::make_right<Error>(std::forward<T>(t));
}

#endif /* _SRCPORT_RESULT_HH_ */
