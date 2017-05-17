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
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include <clang/AST/AST.h>
#include <clang/Frontend/ASTUnit.h>

#include <clang/Lex/Preprocessor.h>

#include <ftl/maybe.h>

#include "record_type.hpp"

#include "paths.hh"
#include "project.hh"

namespace symtab {

using StrRef = std::shared_ptr<std::string>;
using PathRef = std::shared_ptr<Path>;

/**
 * AST macro node. Provides the statements referencing the macro,
 * for later lookup of the macro info.
 */
class MacroRef {
	PL_RECORD_FIELDS(MacroRef,
		/** Referencing statement */
		(const clang::Stmt *,	stmt)
	)

public:
	bool isMacroArgExpansion(clang::Preprocessor &cpp);
	bool isMacroBodyExpansion(clang::Preprocessor &cpp);

	clang::MacroInfo *getMacroInfo (clang::Preprocessor &cpp);
};

/**
 * AST cursor node reference.
 */
using CursorNode = ftl::sum_type<
	const clang::Stmt *,
	const clang::Decl *,
	MacroRef
>;

/**
 * AST node reference.
 */
class Cursor {
public:
	PL_RECORD_FIELDS(Cursor,
		/** Referenced AST node */
		(CursorNode,		node),

		/** Containing AST unit */
		(clang::ASTUnit *,	unit)
	)

public:
	Cursor (const clang::Decl *decl, clang::ASTUnit *astUnit):
	    Cursor(CursorNode { ftl::constructor<const clang::Decl *>{}, decl },
		astUnit)
	{}

	Cursor (const clang::Stmt *stmt, clang::ASTUnit *astUnit):
	    Cursor(CursorNode { ftl::constructor<const clang::Stmt *>{}, stmt },
		astUnit)
	{}

	Cursor (const MacroRef &macro, clang::ASTUnit *astUnit):
	    Cursor(CursorNode { ftl::constructor<MacroRef>{}, macro }, astUnit)
	{}

	/**
	 * Attempt to cast the node to an instance to @tparam T, returning
	 * nullptr if not possible.
	 */
	template<
	    typename T, 
	    class = typename std::enable_if<std::is_base_of<clang::Decl, T>::value>::type
	>
	const T *
	as () {
		return (_node.match(
		    [](const clang::Decl *decl) {
			return (clang::dyn_cast<T>(decl));
		    },
		    [](const clang::Stmt *stmt) {
			return (nullptr);
		    },
		    [](const MacroRef &) {
			return (nullptr);
		    }
		));
	}
};

/**
 * Source code location.
 */
class Location {
	PL_RECORD_FIELDS(Location,
		(PathRef,	path),
		(unsigned,	line),
		(unsigned,	column)
	)

public:
	bool operator<	(const Location &other) const;
};

std::string to_string (const Location &l);

/**
 * Source code symbol definition.
 */
class Symbol {
public:
	PL_RECORD_FIELDS(Symbol,
		(StrRef,	name),
		(Location,	location),
		(Cursor,	cursor),
		(StrRef,	USR)
	)

public:
	/** Return the cursor AST unit */
	clang::ASTUnit *astUnit() { return (_cursor.unit()); };

	/** Return the cursor AST context */
	clang::ASTContext &astContext() { return (astUnit()->getASTContext()); }

	/** Is this an anonymous (zero-length name) symbol? */
	bool isAnonymous () const { return (_name->size() == 0); };
};

using SymbolRef = std::shared_ptr<Symbol>;

PL_RECORD_STRUCT(SymbolUse,
	(SymbolRef,	symbol),
	(Cursor,	cursor),
	(Location,	location)
);

using SymbolUseRef = std::shared_ptr<SymbolUse>;

} /* namespace symtab */


namespace std {
	template<> struct hash<symtab::Location> {
		std::size_t
		operator()(const symtab::Location &l) const 
		{
			std::size_t h1 = hash<Path>()(*l.path());
			std::size_t h2 = l.line();
			return h1 ^ (h2 << 1);
		}
	};

	template<> struct hash<symtab::Symbol> {
		std::size_t
		operator()(const symtab::Symbol &s) const 
		{
			return (hash<string>()(*s.USR()));
		}
	};

	template<> struct hash<symtab::SymbolUse> {
		std::size_t
		operator()(const symtab::SymbolUse &u) const 
		{
			return (hash<symtab::Location>()(u.location()));
		}
	};
} /* namespace std */

namespace symtab {

class SymbolTable {
public:
	template <typename T> struct sptr_hash {
		std::size_t operator() (const T &ptr) const
		{
			return (std::hash<ftl::Value_type<T>>()(*ptr));
		}
	};

	template <typename T> struct sptr_eqto {
		std::size_t operator() (const T &lhs, const T &rhs) const
		{
			return (std::equal_to<ftl::Value_type<T>>()(*lhs, *rhs));
		}
	};

	template <typename T> struct ref_hash {
		std::size_t operator() (const std::reference_wrapper<T> &ptr) const
		{
			std::hash<typename std::decay<T>::type> hash_fn;
			return (hash_fn(ptr.get()));
		}
	};

	template <typename T> struct ref_eqto {
		std::size_t operator() (const std::reference_wrapper<T> &lhs, const std::reference_wrapper<T> &rhs) const
		{
			return (lhs.get() == rhs.get());
		}
	};

	template <typename T>
	    using rset = std::unordered_set<T, sptr_hash<T>, sptr_eqto<T>>;

	template <typename K, typename V>
	    using rmap = std::unordered_map<K, V, sptr_hash<K>, sptr_eqto<K>>;

	using SymbolSet = rset<SymbolRef>;
	using SymbolUseSet = rset<SymbolUseRef>;

	SymbolTable (const ProjectRef &project):
	    _proj(project)
	{}

	ProjectRef
	project () const
	{
		return (_proj);
	}

	ftl::maybe<SymbolRef>	lookupUSR (const std::string &USR);
	bool			 hasUSR (const std::string &USR);

	ftl::maybe<SymbolRef>	 definition (const std::string &USR);
	bool			 hasDefinition (const std::string &USR);
	
	const SymbolUseSet	&usage (const std::string &USR);
	bool			 hasUsage (const std::string &USR);

	SymbolRef		 addSymbol (SymbolRef symbol);
	SymbolRef		 addDefinition (SymbolRef symbol);
	SymbolUseRef		 addSymbolUse (SymbolUseRef use);	

	PathRef			 getPath (const std::string &strval);
	StrRef			 getUSR (const std::string &strval);

	const rset<SymbolUseRef> &getSymbolUses () {
		return (_uses);
	}

	const rset<SymbolRef> &getSymbols () {
		return (_syms);
	}

	const rset<SymbolRef> &getDefinitions () {
		return (_defs);
	}

private:
	bool	hasSymbol(const std::string &USR,
		    std::unique_lock<std::mutex> &lock);
	bool	hasDefinition(const std::string &USR,
		    std::unique_lock<std::mutex> &lock);

	/** Mutex that must be held when performing any
	 *  access of our state */
	std::mutex				_lock;

	/** All referenced symbols */
	SymbolUseSet				_uses;

	/** All declared symbols. */
	SymbolSet				_syms;

	/** All defined symbols. */
	SymbolSet				_defs;

	/** USR cache */
	std::unordered_map<
		std::reference_wrapper<const std::string>,
		StrRef,
		ref_hash<const std::string>,
		ref_eqto<const std::string>
	> _usr_cache;

	/* Path cache */
	std::unordered_map<
		std::reference_wrapper<const std::string>,
		PathRef,
		ref_hash<const std::string>,
		ref_eqto<const std::string>
	> _path_cache;


	/** Symbol file lookup table */
	rmap<PathRef, SymbolSet>		_syms_path;

	/** Symbol USR lookup table */
	rmap<StrRef, SymbolRef>			_syms_usr;

	/** Definition USR lookup table */
	rmap<StrRef, SymbolRef>			_defs_usr;
	
	/** Definition file lookup table */
	rmap<PathRef, SymbolSet>		_defs_path;

	/** SymbolUse file lookup table */
	rmap<PathRef, SymbolUseSet>		_uses_path;

	/** SymbolUse USR lookup table */
	rmap<StrRef, SymbolUseSet>		_uses_usr;

	/** Project configuration */
	ProjectRef				_proj;
};

using SymbolTableRef = std::shared_ptr<SymbolTable>;
using SymbolSet = SymbolTable::SymbolSet;
using SymbolUseSet = SymbolTable::SymbolUseSet;

} /* namespace symtab */


#endif /* _SRCPORT_SYMBOL_TABLE_HH_ */
