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

#include <functional>

#include "symbol_table.hh"

using namespace std;

namespace symtab {

/** Location to_string() support */
std::string to_string (const Location &l) {
	return (l.path()->stringValue() + ":" + std::to_string(l.line()) +
	    ":" + std::to_string(l.column()));
}

/**
 * Compare locations by by path, line, and finally column.
 */
bool
Location::operator< (const Location &rhs) const
{
	if (*_path != *rhs.path())
		return (*_path < *rhs.path());

	if (_line != rhs.line())
		return (_line < rhs.line());

	return (_column < rhs.column());
}

/**
 * Look up the symbol registered for @p USR, if any.
 */
ftl::maybe<SymbolRef>
SymbolTable::lookupUSR (const std::string &USR)
{
	lock_guard<mutex> lock(_lock);

	if (!hasUSR(USR, lock))
		return (ftl::Nothing());

	const auto &key = _usr_cache.at(std::cref(USR));
	return (ftl::just((_syms_usr.at(key))));
}

/**
 * Return true if a symbol is registered for @p USR.
 */
bool
SymbolTable::hasUSR (const std::string &USR)
{
	lock_guard<mutex> lock(_lock);
	return hasUSR(USR, lock);
}

/**
 * Private locked variant of hasUSR().
 */
bool
SymbolTable::hasUSR (const std::string &USR, lock_guard<mutex> &lock)
{
	if (_usr_cache.count(std::cref(USR)) == 0)
		return (false);

	const auto &key = _usr_cache.at(std::cref(USR));
	return (_syms_usr.count(key) > 0);
}


/**
 * Return the usage set for a symbol with @p USR.
 */
SymbolUseSet 
SymbolTable::usage(const std::string &USR)
{
	lock_guard<mutex>	lock(_lock);
	SymbolUseSet		result;

	if (_usr_cache.count(std::cref(USR)) == 0)
		return (result);

	const auto &key = _usr_cache.at(std::cref(USR));
	auto usages = _uses_usr.equal_range(key);

	for (auto &i = usages.first; i != usages.second; i++)
		result.insert((*i).second);

	return (result);
}

/**
 * Return true if any usages are registered for a symbol wth @p USR.
 */
bool
SymbolTable::hasUsage(const string &USR)
{
	lock_guard<mutex> lock(_lock);

	if (_usr_cache.count(std::cref(USR)) == 0)
		return (false);

	const auto &key = _usr_cache.at(std::cref(USR));
	return (_uses_usr.count(key) > 0);
}

/**
 * Return a path reference for the given path string, caching a new path
 * instance if necessary.
 */
PathRef
SymbolTable::getPath (const string &strval)
{
	lock_guard<mutex> lock(_lock);

	auto key = std::cref(strval);

	if (_path_cache.count(key) > 0)
		return (_path_cache[key]);

	auto normalized = Path(strval).normalize();
	key = std::cref(normalized.stringValue());

	if (_path_cache.count(key) > 0)
		return (_path_cache[key]);

	auto p = make_shared<Path>(normalized);
	_path_cache.emplace(std::cref(p->stringValue()), p);

	return (p);
}

/**
 * Return a USR reference for the given USR string, caching a new USR
 * instance if necessary.
 */
StrRef
SymbolTable::getUSR (const string &strval)
{
	lock_guard<mutex> lock(_lock);

	auto key = std::cref(strval);

	if (_usr_cache.count(key) > 0)
		return (_usr_cache[key]);

	auto cached = make_shared<string>(strval);
	_usr_cache.emplace(std::cref(*cached), cached);

	return (cached);
}

void
SymbolTable::addSymbol (SymbolRef symbol)
{
	lock_guard<mutex> lock(_lock);

	if (hasUSR(*symbol->USR(), lock))
		return;

	_syms.emplace(symbol);
	_usr_cache.emplace(std::cref(*symbol->USR()), symbol->USR());

	_syms_usr.emplace(make_pair(symbol->USR(), symbol));
	_syms_path.emplace(make_pair(symbol->location().path(), symbol));
}

void
SymbolTable::addSymbolUse (SymbolUseRef use)
{
	lock_guard<mutex> lock(_lock);

	if (_uses.count(use) > 0)
		return;

	_uses.emplace(use);

	_uses_usr.emplace(make_pair(use->symbol()->USR(), use));
	_uses_path.emplace(make_pair(use->location().path(), use));
}
	
} /* namespace symtab */
