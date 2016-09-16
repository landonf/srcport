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

#ifndef _SRCPORT_AST_INDEX_HH_
#define _SRCPORT_AST_INDEX_HH_

#include <memory>

#include <clang/Frontend/FrontendAction.h>
#include <clang/Frontend/FrontendActions.h>
#include <clang/Frontend/CompilerInstance.h>

#include <clang/Tooling/Tooling.h>

#include <clang/AST/AST.h>

#include <clang/ASTMatchers/ASTMatchers.h>
#include <clang/ASTMatchers/ASTMatchFinder.h>

#include "compiler.hh"
#include "project.hh"
#include "symbol_table.hh"

class ASTIndex;
using ASTIndexRef = std::shared_ptr<ASTIndex>;

/**
 * AST Indexing utilities.
 */
class ASTIndexUtil {
public:
	ASTIndexUtil (symtab::SymbolTableRef &symtab, clang::ASTUnit &astUnit):
	    _symtab(symtab), _astUnit(astUnit), _ast(astUnit.getASTContext())
	{ }

	symtab::SymbolRef	registerDefinition(
				    const clang::NamedDecl &symbol);
	symtab::SymbolRef	registerSymbol(const clang::NamedDecl &symbol);
	symtab::SymbolRef	registerSymbol(const clang::Stmt *stmt,
				    const clang::IdentifierInfo &ident,
				    const clang::MacroDefinition &macro,
				    clang::Preprocessor &cpp);

	symtab::SymbolUseRef	registerSymbolUse(const clang::Stmt *symbolUse,
				    const symtab::SymbolRef &symbol);

	symtab::StrRef		cacheUSR(const clang::Decl &decl);
	symtab::StrRef		cacheUSR(const clang::MacroDefinitionRecord &macro);
	symtab::Location	generateLocation(const clang::SourceLocation &loc);

private:
	symtab::SymbolTableRef	_symtab;
	clang::ASTUnit		&_astUnit;
	clang::ASTContext	&_ast;
};

/**
 * AST Index builder.
 */
class ASTIndexBuilder {
public:
	ASTIndexBuilder (const CompilerRef &cc, const ProjectRef &project):
	    _project(project), _cc(cc)
	{
		_symtab = std::make_shared<symtab::SymbolTable>(project);
	}

	symtab::SymbolTableRef	build ();

private:
	void			indexReferences (clang::ASTUnit *au);
	void			indexDefinitions (clang::ASTUnit *au);

	ProjectRef		_project;	/**< project configuration */
	CompilerRef		_cc;		/**< compilation state */
	symtab::SymbolTableRef	_symtab;	/**< symbol index */
};


/**
 * AST Index
 */
class ASTIndex : public std::enable_shared_from_this<ASTIndex> {
private:
	struct AllocKey {};

public:
	static result<ASTIndexRef>	Build(const ProjectRef &project,
					    const CompilerRef &cc);

	ASTIndex (const CompilerRef &cc, const symtab::SymbolTableRef &symtab,
	    const AllocKey &key): 
	    _cc(cc), _symtab(symtab)
	{
	}

	const symtab::SymbolRef		 getCanonicalSymbol(
					     const symtab::SymbolRef &symbol);
	const symtab::SymbolSet		&getSymbols();
	const symtab::SymbolUseSet	&getSymbolUses();
	symtab::SymbolUseSet	 	 getSymbolUses(const std::string &USR);
	bool				 hasSymbolUses(const std::string &USR);

private:
	CompilerRef		_cc;		/**< compilation state */
	symtab::SymbolTableRef	_symtab;	/**< symbol index */
};

using ASTIndexRef = std::shared_ptr<ASTIndex>;

#endif /* _SRCPORT_AST_INDEX_HH_ */
