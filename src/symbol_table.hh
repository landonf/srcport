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

#include <ftl/maybe.h>

#include "record_type.hpp"

#include "paths.hh"
#include "project.hh"

namespace symtab {

using StrRef = std::shared_ptr<std::string>;
using PathRef = std::shared_ptr<Path>;

class Location {
	PL_RECORD_FIELDS(Location,
		(PathRef,	path),
		(unsigned,	line),
		(unsigned,	column)
	)
};

std::string to_string (const Location &l);

PL_RECORD_STRUCT(Symbol,
	(StrRef,	name),
	(Location,	location),
	(StrRef,	USR)
);

using SymbolRef = std::shared_ptr<Symbol>;

PL_RECORD_STRUCT(SymbolUse,
	(SymbolRef,		symbol),
	(Location,		location),
	(StrRef,		USR)
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
			return (std::hash<T*>()(&ptr.get()));
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

	template <typename K, typename V>
	    using rmultimap = std::unordered_multimap<K, V, sptr_hash<K>, sptr_eqto<K>>;
	
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

	ftl::maybe<SymbolRef> lookupUSR (const std::string &USR) const;
	bool hasUSR (const std::string &USR) const;

	void addSymbol (SymbolRef symbol);
	void addSymbolUse (SymbolUseRef use);	

	PathRef getPath (const std::string &strval);
	StrRef getUSR (const std::string &strval);

	const rset<SymbolUseRef> &getSymbolUses () {
		return (_uses);
	}

	const rset<SymbolRef> &getSymbols () {
		return (_syms);
	}

private:
	/** All referenced symbols */
	rset<SymbolUseRef>			_uses;

	/** All defined symbols. */
	rset<SymbolRef>				_syms;

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
	rmultimap<PathRef, SymbolRef>		_syms_path;

	/** Symbol USR lookup table */
	rmap<StrRef, SymbolRef>			_syms_usr;

	/** SymbolUse file lookup table */
	rmultimap<PathRef, SymbolUseRef>	_uses_path;

	/** SymbolUse USR lookup table */
	rmultimap<StrRef, SymbolUseRef>		_uses_usr;

	/** Project configuration */
	Project					_proj;
};

} /* namespace symtab */

#endif /* _SRCPORT_SYMBOL_TABLE_HH_ */
