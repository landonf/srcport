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

#ifndef _SRCPORT_SYMBOL_TABLE_HH_
#define _SRCPORT_SYMBOL_TABLE_HH_

#include <unordered_map>
#include <unordered_set>

#include <clang/AST/AST.h>
#include <clang/Lex/PreprocessingRecord.h>

#include <ftl/sum_type.h>
#include <ftl/maybe.h>

#include "record_type.hpp"

#include "paths.hh"
#include "project.hh"

namespace symtab {

using StrRef = std::shared_ptr<const std::string>;
using PathRef = std::shared_ptr<const Path>;
using SymParent = ftl::maybe<clang::FunctionDecl *>;

PL_RECORD_STRUCT(Location,
	(PathRef,	path),
	(unsigned,	line),
	(unsigned,	column)
);

using LocRef = std::shared_ptr<const Location>;

PL_RECORD_STRUCT(LangSymbol,
	(StrRef,		name),
	(clang::NamedDecl *,	decl),
	(LocRef,		location),
	(StrRef,		USR)
);

PL_RECORD_STRUCT(MacroSymbol,
	(StrRef,		name),
	(clang::MacroInfo *,	info),
	(LocRef,		location),
	(StrRef,		USR)
);

using Symbol = ftl::sum_type<LangSymbol, MacroSymbol>;
using SymbolRef = std::shared_ptr<const Symbol>;

PL_RECORD_STRUCT(LangUse,
	(clang::DeclRefExpr *,	expr),
	(SymParent,		parent),
	(LocRef,		location),
	(StrRef,		USR)
);

PL_RECORD_STRUCT(MacroUse,
	(clang::Stmt *,		expr),
	(SymParent,		parent),
	(LocRef,		location),
	(StrRef,		USR)
);

using SymbolUse = ftl::sum_type<LangUse, MacroUse>;

class SymbolTable {
public:
	SymbolTable (const Project &project):
	    _proj(project)
	{}
	SymbolTable (Project &&project):
	    _proj(std::move(project))
	{}

	const Project &
	project () const
	{
		return (_proj);
	}
private:
	/* Map of symbols to their enclosing path. */
	std::unordered_multimap<PathRef, SymbolRef>	_syms;

	/** Map of paths to associated symbol uses */
	std::unordered_multimap<PathRef, SymbolUse>	_uses;

	/** USR symbol lookup table */
	std::unordered_map<StrRef, SymbolRef>		_symtab;

	/** USR symbol use lookup table */
	std::unordered_map<StrRef, SymbolRef>		_usetab;

	/** Project configuration */
	Project						_proj;
};

} /* namespace symtab */

#endif /* _SRCPORT_SYMBOL_TABLE_HH_ */
