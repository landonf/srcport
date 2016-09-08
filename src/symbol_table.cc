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

#include "symbol_table.hh"

using namespace std;

namespace symtab {

ftl::maybe<const Symbol *>
SymbolTable::lookupUSR (const StrRef &USR) const
{
	if (hasUSR(USR))
		return (ftl::just(&_syms_usr.at(USR)));

	return (ftl::Nothing());
}

bool
SymbolTable::hasUSR (const StrRef &USR) const
{
	return (_syms_usr.count(USR) > 0);
}

void
SymbolTable::addSymbol (const Symbol &symbol)
{
	if (hasUSR(symbol.USR()))
		return;

	auto iter = _syms.emplace(symbol).first;

	const Symbol &ref = *iter;

	_syms_usr.emplace(make_pair(ref.USR(), ref));
	_syms_path.emplace(make_pair(ref.location().path(), ref));
}

void
SymbolTable::addSymbolUse (const SymbolUse &use)
{
	if (_uses.count(use) > 0)
		return;

	auto iter = _uses.emplace(use).first;

	const SymbolUse &ref = *iter;

	_uses_usr.emplace(make_pair(ref.USR(), ref));
	_uses_path.emplace(make_pair(ref.location().path(), ref));
}

#if 0
void
SymbolTable::addSymbolUse (SymbolUse &&use)
{
	if (_uses.count(use) > 0)
		return;

	_uses.emplace(use);
	_usr_uses.emplace(make_pair(use.USR(), use));
	_path_syms.emplace(make_pair(use->location()->path(), use));
}
#endif
	
} /* namespace symtab */
