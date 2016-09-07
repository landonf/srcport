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

#include "paths.h"
#include <cstdlib>

using namespace ftl;
using namespace std;

std::string to_string (const Path &p) {
	return p.stringValue();
}

void
Path::parse_path ()
{
	string::size_type start, end;

	_normalForm = true;
	_size = 0;

	for (start = 0; start < _str.size(); start = end) {
		/* Skip leading '/' */
		if (_str[start] == '/') {
			end = start + 1;

			/* Reject '//' */
			if (end < _str.size() && _str[end] == '/')
				_normalForm = false;

			continue;
		}
		
		/* Find next '/' */
		end = _str.find_first_of('/', start);
		if (end == string::npos)
			end = _str.size();

		const auto &elem = _str.substr(start, end-start);

		if (elem == ".")
			_normalForm = false;
		else if (elem == "..")
			_normalForm = false;

		_size++;
	}

	/* Trailing '/'? */
	if (_str.size() > 0 && _str[_str.size() - 1] == '/')
		_normalForm = false;
}

/**
 * Construct a new path with the given string.
 * 
 * @param string Path string representation.
 */
Path::Path (const std::string &string)
{
	_str = string;

	parse_path();
}

/**
 * Construct a new path with the given path components.
 */
Path::Path
(const std::vector<std::string> &components)
{
	if (components.empty())
		return;

	string::size_type len = components.size() - 1;

	for (const auto &c : components)
		len += c.size();

	_str.reserve(len);

	for (const auto &c : components) {
		if (!_str.empty() && _str != "/")
			_str.append("/");

		_str.append(c);
	}

	parse_path();
}

bool
Path::operator== (const Path &other) const
{
	if (!inNormalForm())
		return (normalize() == other);

	if (!other.inNormalForm())
		return (*this == other.normalize());

	return (this->stringValue() == other.stringValue());
}

bool
Path::operator!= (const Path &other) const
{
	return !(*this == other);
}

/**
 * Returns true if this path starts with the given directory prefix.
 */
bool
Path::hasPrefix (const Path& prefix) const
{
	/* Ensure that we're in normal form */
	if (!this->inNormalForm())
		return (this->normalize().hasPrefix(prefix));

	if (!prefix.inNormalForm())
		return (hasPrefix(prefix.normalize()));

	const auto &p = prefix.stringValue();
	const auto &ours = this->stringValue();

	/* No match if prefix is larger than we are */
	if (ours.size() < p.size())
		return (false);

	/* Equality match if we're the same size */
	if (ours.size() == p.size())
		return (ours == p);

	/* Otherwise, perform a prefix match */
	const auto &m = mismatch(p.begin(), p.end(), ours.begin());

	if (m.first == p.end() && ours[p.size()] == '/')
		return (true);

	return (false);
}

/**
 * Returns true if the path is in fully normal form.
 */
bool
Path::inNormalForm () const
{
	return (_normalForm);
}

/**
 * Split a string into its path components, optionally performing
 * normalization.
 */
std::vector<std::string>
Path::split (bool normalize) const
{
	vector<string>		result;
	string::size_type	start, end;

	if (_str.size() == 0)
		return (result);

	/* Skip any number of leading '/', adding one '/' to the output  */
	for (start = 0; start < _str.size() && _str[start] == '/'; start++) {
		if (start == 0)
			result.emplace_back("/");
	}

	for (start = 0; start < _str.size(); start = end) {
		/* Skip leading '/' */
		if (_str[start] == '/') {
			end = start + 1;
			continue;
		}
		
		/* Find next '/' */
		end = _str.find_first_of('/', start);
		if (end == string::npos)
			end = _str.size();


		const auto &elem = _str.substr(start, end-start);

		if (normalize) {
			if (elem == ".") {
				continue;
			} else if (elem == "..") {
				if (!result.empty())
					result.pop_back();
				continue;
			}
		}

		result.emplace_back(elem);
	}

	return (result);
}

/**
 * Return a normalized representation of this path, trimming redundant '//'
 * sequences, removing any terminating '/', removing '/./', and applying
 * references to '/../'.
 */
Path
Path::normalize () const
{
	if (inNormalForm())
		return (*this);
	else
		return Path(split(true));
}


/**
 * Resolve all symbolic links and relative references, returning
 * an absolute path, or an error if the path does not exist or cannot be
 * resolved.
 */
result<Path>
Path::resolve () const
{
	char buf[PATH_MAX];

	if (realpath(_str.c_str(), buf) == NULL)
		return (result<Path>{constructor<Error>(), errno});
	else
		return (result<Path>{constructor<Path>(), Path(buf)});
}
