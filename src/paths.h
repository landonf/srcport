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

#ifndef _SRCPORT_PATHS_H_
#define _SRCPORT_PATHS_H_

#include <string>
#include <vector>

#include <ftl/functional.h>
#include <ftl/vector.h>
#include <ftl/sum_type.h>

#include "error.hpp"
#include "result.h"

/**
 * File system path.
 */
class Path {
public:
	using size_type = size_t;

	explicit Path (const std::string &string);
	explicit Path (const std::vector<std::string> &components);

	/**
	 * Return the path's string value.
	 */
	const std::string &stringValue () const {
		return _str;
	}

	bool inNormalForm () const;
	std::vector<std::string> split (bool normalize = true) const;
	Path normalize () const;
	result<Path> resolve () const;

	bool hasPrefix (const Path &prefix) const;
	bool isPrefix (const Path &path) const;

	size_type size () { return (_size); }

	bool operator== (const Path &other) const;
	bool operator!= (const Path &other) const;

private:
	void parse_path ();

	std::string	_str;
	size_type	_size;
	bool		_normalForm;
};

/**
 * A set of path prefixes.
 */
class PathPattern {
public:
	PathPattern (std::initializer_list<std::string> &&prefix) : PathPattern(std::vector<std::string>(std::move(prefix))) {}
	PathPattern (std::initializer_list<Path> &&prefix) : _prefixes(std::move(prefix)) {}

	explicit PathPattern (const std::vector<std::string> &prefix);
	explicit PathPattern (std::vector<Path> &&prefix) : _prefixes(std::move(prefix)) {}

	/**
	 * Return true if the pattern matches the given path.
	 */
	inline bool matches (const Path &path)
	{
		/* TODO: We should probably build a match table from the normalized,
		 * path prefixes, rather than performing brute-force matching */
		for (const auto &p : _prefixes) {
			if (p.isPrefix(path))
				return (true);
		}

		return (false);
	}
	
	inline bool matches (std::string &&path) {
		return (matches(Path(std::move(path))));
	}

private:
	/** Prefixes to match against */
	std::vector<Path> _prefixes;
};

#endif /* _SRCPORT_PATHS_H_ */
