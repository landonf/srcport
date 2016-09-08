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

#include <functional>
#include <string>
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

using SymbolDecl = ftl::sum_type<clang::NamedDecl *, clang::MacroInfo *>;
using SymbolUseExpr = ftl::sum_type<clang::DeclRefExpr *, clang::Stmt *>;

using StrRef = std::shared_ptr<const std::string>;
using PathRef = std::shared_ptr<Path>;
using SymParent = ftl::maybe<clang::FunctionDecl *>;

PL_RECORD_STRUCT(Location,
	(PathRef,	path),
	(unsigned,	line),
	(unsigned,	column)
);

PL_RECORD_STRUCT(Symbol,
	(StrRef,	name),
	(SymbolDecl,	decl),
	(Location,	location),
	(StrRef,	USR)
);

PL_RECORD_STRUCT(SymbolUse,
	(SymbolUseExpr,		expr),
	(SymParent,		parent),
	(Location,		location),
	(StrRef,		USR)
);

} /* namespace symtab */


namespace std {
	template<> struct hash<symtab::Location> {
		std::size_t
		operator()(symtab::Location const &l) const 
		{
			std::size_t h1 = hash<Path>()(*l.path());
			std::size_t h2 = l.line();
			return h1 ^ (h2 << 1);
		}
	};

	template<> struct hash<symtab::Symbol> {
		std::size_t
		operator()(symtab::Symbol const &s) const 
		{
			return (hash<string>()(*s.USR()));
		}
	};

	template<> struct hash<symtab::SymbolUse> {
		std::size_t
		operator()(symtab::SymbolUse const &u) const 
		{
			return (hash<symtab::Location>()(u.location()));
		}
	};
} /* namespace std */

namespace symtab {

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

	ftl::maybe<const Symbol *> lookupUSR (const StrRef &USR) const;
	bool hasUSR (const StrRef &USR) const;

	void addSymbol (const Symbol &symbol);
	void addSymbolUse (const SymbolUse &use);

private:
	/* All defined symbols. */
	std::unordered_set<Symbol>				_syms;

	/** Map of paths to associated symbols */
	std::unordered_multimap<PathRef, const Symbol&>		_syms_path;

	/** USR symbol lookup table */
	std::unordered_map<StrRef, const Symbol&>		_syms_usr;

	/** All used symbols */
	std::unordered_set<SymbolUse>				_uses;

	/** Map of paths to associated symbol uses */
	std::unordered_multimap<PathRef, const SymbolUse&>	_uses_path;

	/** USR symbol use lookup table */
	std::unordered_multimap<StrRef, const SymbolUse&>	_uses_usr;

	/** Project configuration */
	Project							_proj;
};

} /* namespace symtab */

#endif /* _SRCPORT_SYMBOL_TABLE_HH_ */
