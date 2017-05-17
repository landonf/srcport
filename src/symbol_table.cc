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
	
static SymbolUseSet SYMBOL_USE_EMPTY;

/** Location to_string() support */
std::string to_string (const Location &l) {
	return (l.path()->stringValue() + ":" + std::to_string(l.line()) +
	    ":" + std::to_string(l.column()));
}

/**
 * Return true if the macro reference is an argument expansion.
 */
bool
MacroRef::isMacroArgExpansion(clang::Preprocessor &cpp)
{
	auto loc = _stmt->getLocStart();
	return (cpp.getSourceManager().isMacroArgExpansion(loc));
}

/**
 * Return true if the macro reference is a body expansion.
 */
bool
MacroRef::isMacroBodyExpansion(clang::Preprocessor &cpp)
{
	auto loc = _stmt->getLocStart();
	return (cpp.getSourceManager().isMacroBodyExpansion(loc));
}

/**
 * Use the preprocessor to fetch macro info.
 */
clang::MacroInfo *
MacroRef::getMacroInfo (clang::Preprocessor &cpp)
{
	/* Extract macro info */
	auto loc = _stmt->getLocStart();

	if (this->isMacroArgExpansion(cpp))
		loc = cpp.getSourceManager().getImmediateSpellingLoc(loc);

	auto name = cpp.getImmediateMacroName(loc);
	auto *ident = cpp.getIdentifierInfo(name);

	auto mdef = cpp.getMacroDefinition(ident);
	return (mdef.getMacroInfo());
};

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
	unique_lock<mutex> lock(_lock);

	if (!hasSymbol(USR, lock))
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
	unique_lock<mutex> lock(_lock);
	return hasSymbol(USR, lock);
}

/**
 * Private locked variant of hasUSR().
 */
bool
SymbolTable::hasSymbol (const std::string &USR, unique_lock<mutex> &lock)
{
	if (_usr_cache.count(std::cref(USR)) == 0)
		return (false);

	const auto &key = _usr_cache.at(std::cref(USR));
	return (_syms_usr.count(key) > 0);
}

/**
 * Look up the definition registered for @p USR, if any.
 */
ftl::maybe<SymbolRef>
SymbolTable::definition (const std::string &USR)
{
	unique_lock<mutex> lock(_lock);

	if (!hasDefinition(USR, lock))
		return (ftl::Nothing());

	const auto &key = _usr_cache.at(std::cref(USR));
	return (ftl::just((_defs_usr.at(key))));
}

/**
 * Return true if a definition is registered for @p USR.
 */
bool
SymbolTable::hasDefinition (const std::string &USR)
{
	unique_lock<mutex> lock(_lock);
	return hasDefinition(USR, lock);
}

/**
 * Private locked variant of hasDefinition().
 */
bool
SymbolTable::hasDefinition (const std::string &USR, unique_lock<mutex> &lock)
{
	if (_usr_cache.count(std::cref(USR)) == 0)
		return (false);

	const auto &key = _usr_cache.at(std::cref(USR));
	return (_defs_usr.count(key) > 0);
}


/**
 * Return the usage set for a symbol with @p USR.
 */
const SymbolUseSet &
SymbolTable::usage(const std::string &USR)
{
	unique_lock<mutex>	lock(_lock);

	if (_usr_cache.count(std::cref(USR)) == 0)
		return (SYMBOL_USE_EMPTY);

	const auto &key = _usr_cache.at(std::cref(USR));

	if (_uses_usr.count(key) == 0)
		return (SYMBOL_USE_EMPTY);

	return (_uses_usr.at(key));
}

/**
 * Return true if any usages are registered for a symbol wth @p USR.
 */
bool
SymbolTable::hasUsage(const string &USR)
{
	unique_lock<mutex> lock(_lock);

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
	unique_lock<mutex> lock(_lock);

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
	unique_lock<mutex> lock(_lock);

	auto key = std::cref(strval);

	if (_usr_cache.count(key) > 0)
		return (_usr_cache[key]);

	auto cached = make_shared<string>(strval);
	_usr_cache.emplace(std::cref(*cached), cached);

	return (cached);
}

/**
 * Attempt to register a symbol, returning either the newly registered symbol
 * instance, or the existing instance if already registered.
 */
SymbolRef
SymbolTable::addSymbol (SymbolRef symbol)
{
	unique_lock<mutex> lock(_lock);

	if (hasSymbol(*symbol->USR(), lock))
		return (_syms_usr.at(symbol->USR()));

	_syms.emplace(symbol);
	_usr_cache.emplace(std::cref(*symbol->USR()), symbol->USR());

	_syms_usr.emplace(make_pair(symbol->USR(), symbol));

	const auto &p = symbol->location().path();
	if (_syms_path.count(p) == 0) {
		_syms_path.emplace(make_pair(p, rset<SymbolRef>({symbol})));
	} else {
		_syms_path.at(p).emplace(symbol);
	}

	return (symbol);
}

/**
 * Attempt to register a symbol definition, returning either the newly
 * registered symbol instance, or the existing instance if already registered.
 */
SymbolRef
SymbolTable::addDefinition(SymbolRef symbol)
{
	unique_lock<mutex> lock(_lock);

	if (hasDefinition(*symbol->USR(), lock))
		return (_defs_usr.at(symbol->USR()));

	_defs.emplace(symbol);
	_usr_cache.emplace(std::cref(*symbol->USR()), symbol->USR());

	_defs_usr.emplace(make_pair(symbol->USR(), symbol));

	const auto &p = symbol->location().path();
	if (_defs_path.count(p) == 0) {
		_defs_path.emplace(make_pair(p, rset<SymbolRef>({symbol})));
	} else {
		_defs_path.at(p).emplace(symbol);
	}

	return (symbol);
}


/**
 * Attempt to register a symbol use, returning either the newly registered use
 * instance, or the existing instance if already registered.
 */
SymbolUseRef
SymbolTable::addSymbolUse (SymbolUseRef use)
{
	unique_lock<mutex> lock(_lock);

	if (_uses.count(use) > 0)
		return (*_uses.find(use));

	_uses.emplace(use);

	const auto &USR = use->symbol()->USR();
	if (_uses_usr.count(USR) == 0) {
		_uses_usr.emplace(make_pair(USR, SymbolUseSet({use})));
	} else {
		_uses_usr.at(USR).emplace(use);
	}

	const auto &p = use->location().path();
	if (_uses_path.count(p) == 0) {
		_uses_path.emplace(make_pair(p, SymbolUseSet({use})));
	} else {
		_uses_path.at(p).emplace(use);
	}

	return (use);
}
	
} /* namespace symtab */
