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

#ifndef _SRCPORT_CVISITOR_STATE_HH_
#define _SRCPORT_CVISITOR_STATE_HH_

#include <clang/Frontend/FrontendAction.h>
#include <clang/Frontend/FrontendActions.h>
#include <clang/Frontend/CompilerInstance.h>

#include <clang/AST/AST.h>

#include "symbol_table.hh"

class VisitorState {
public:
	VisitorState (std::shared_ptr<symtab::SymbolTable> &syms, clang::CompilerInstance &c): 
	    _syms(syms), _c(c), _ast(c.getASTContext()), _srcManager(_ast.getSourceManager())
	{
		assert(syms);
	}

	bool isHostRef (clang::DeclRefExpr *ref) const;
	bool isHostRef (clang::SourceLocation usedAt, clang::SourceLocation definedAt) const;

	bool isSourceLoc (const clang::SourceLocation &loc) const;
	bool isHostLoc (const clang::SourceLocation &loc) const;

	bool hasFileEntry (const clang::SourceLocation &loc) const;

	const clang::Decl *getTypeDecl (const clang::Type *t, const clang::TypeSourceInfo *info) const;

	std::string descLoc (const clang::SourceLocation &loc);
	void dumpLoc (const clang::SourceLocation &loc);

	std::shared_ptr<symtab::SymbolTable> &syms () { return (_syms); }
	clang::CompilerInstance &c () { return (_c); }
	clang::ASTContext &ast () { return (_ast); }
	clang::SourceManager &srcManager () { return (_srcManager); }

private:
	bool locMatches (const clang::SourceLocation &loc, const PathPattern &p) const;

	std::shared_ptr<symtab::SymbolTable>	 _syms;
	clang::CompilerInstance			&_c;
	clang::ASTContext			&_ast;
	clang::SourceManager			&_srcManager;
};


#endif /* _SRCPORT_CVISITOR_STATE_HH_ */
