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

#include "project.hh"
#include "symbol_table.hh"

/**
 * AST Indexing utilities.
 */
class ASTIndexUtil {
public:
	ASTIndexUtil (symtab::SymbolTableRef &symtab, const clang::ASTContext &ast):
	    _symtab(symtab), _ast(ast)
	{ }

	symtab::SymbolRef	registerSymbol(const clang::NamedDecl &symbol);
	symtab::SymbolRef	registerSymbol(
				    const clang::IdentifierInfo &ident,
				    const clang::MacroDefinition &macro,
				    clang::Preprocessor &cpp);

	symtab::SymbolUseRef	registerSymbolUse(const clang::Stmt *symbolUse,
				    const symtab::SymbolRef &symbol);

	symtab::StrRef		cacheUSR(const clang::Decl &decl);
	symtab::StrRef		cacheUSR(const clang::MacroDefinitionRecord &macro);
	symtab::Location	generateLocation(const clang::SourceLocation &loc);

	symtab::SymbolDecl	generateDefinition(const clang::Decl &decl);
	symtab::SymbolDecl	generateDefinition(const clang::FunctionDecl &decl);
	symtab::SymbolDecl	generateDefinition(const clang::RecordDecl &decl);
	symtab::SymbolDecl	generateDefinition(const clang::EnumDecl &decl);
	symtab::SymbolDecl	generateDefinition(const clang::EnumConstantDecl &decl);

private:
	symtab::SymbolTableRef		_symtab;
	const clang::ASTContext		&_ast;
};

/**
 * AST Index builder.
 */
class ASTIndexBuilder {
	using ClangToolRef = std::shared_ptr<clang::tooling::ClangTool>;
public:
	ASTIndexBuilder (ClangToolRef &tool, symtab::SymbolTableRef &symtab):
	    _project(symtab->project()), _tool(tool), _symtab(symtab) {}

	void build ();

private:
	ProjectRef		_project;	/**< project configuration */
	ClangToolRef		_tool;		/**< tool state */
	symtab::SymbolTableRef	_symtab;	/**< symbol index */
};


/**
 * AST Index
 */
class ASTIndex : public std::enable_shared_from_this<ASTIndex> {
private:
	struct AllocKey {};
	using ClangToolRef = std::shared_ptr<clang::tooling::ClangTool>;

public:
	static std::shared_ptr<ASTIndex> Index (ProjectRef &project, ClangToolRef &tool);

	ASTIndex (ProjectRef &project, ClangToolRef &tool, const AllocKey &key): 
	    _project(project), _tool(tool), _symtab(std::make_shared<symtab::SymbolTable>(project))
	{
	}

private:
	ProjectRef		_project;	/**< project configuration */
	ClangToolRef		_tool;		/**< tool state */
	symtab::SymbolTableRef	_symtab;	/**< symbol index */
};

using ASTIndexRef = std::shared_ptr<ASTIndex>;

#endif /* _SRCPORT_AST_INDEX_HH_ */
