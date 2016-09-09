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

#include <unordered_set>
#include <string>

#include "cvisitor.hh"

#include <clang/Index/USRGeneration.h>
#include <clang/Lex/Preprocessor.h>

using namespace clang;
using namespace clang::index;
using namespace std;
using namespace symtab;

// XXX temporary
static unordered_set<string> seen;

bool SourcePortASTVisitor::VisitDeclaratorDecl (clang::DeclaratorDecl *decl)
{	
	const clang::NamedDecl	*defn;
	const clang::Type	*t;

	if (!_state.isSourceLoc(decl->getLocation()))
		return (true);

	t = decl->getType().getTypePtr();
	defn = _state.getTypeDecl(t, decl->getTypeSourceInfo());
	if (defn == nullptr)
		return (true);

	if (!_state.isHostRef(decl->getLocation(), defn->getLocation()))
		return (true);

	recordSymbolUseVisit(decl, (NamedDecl *)defn);

	return (true);
}

void
SourcePortASTVisitor::recordSymbolUseVisit (DeclaratorDecl *useExpr, NamedDecl *declExpr)
{
	SmallString<255>	sbuf;
	shared_ptr<string>	USR, name;

	/* Generate USR */
	if (generateUSRForDecl(declExpr, sbuf))
		return;

	USR = make_shared<string>(sbuf.str());

	/* Fetch symbol name */
	name = make_shared<string>(declExpr->getName());

	/* Register symbol use */
	auto symbolUse = make_shared<SymbolUse>(
		SymbolUseExpr { ftl::constructor<DeclaratorDecl *>(), useExpr },
		_inFunc == nullptr ? ftl::Nothing() : ftl::just(_inFunc),
		getLocation(useExpr->getLocation()),
		USR
	);

	_state.syms()->addSymbolUse(symbolUse);

	/* If the symbol itself has previously been registered, nothing left
	 * to do. */
	if (_state.syms()->hasUSR(*USR))
		return;

	/* Register the symbol */
	auto symbol = make_shared<Symbol>(
		name,
		SymbolDecl { ftl::constructor<NamedDecl*>(), declExpr },
		getLocation(declExpr->getLocation()),
		USR
	);

	_state.syms()->addSymbol(symbol);
}

bool
SourcePortASTVisitor::TraverseCallExpr (clang::CallExpr *call)
{
	clang::CallExpr	*prev;
	bool		 result;


	prev = _inCall;
	_inCall = call;

	result = RecursiveASTVisitor::TraverseCallExpr(call);

	_inCall = prev;

	return (result);
}

bool
SourcePortASTVisitor::TraverseFunctionDecl (clang::FunctionDecl *func)
{
	clang::FunctionDecl	*prev;
	bool			 result;

	prev = _inFunc;
	_inFunc = func;

	result = RecursiveASTVisitor::TraverseFunctionDecl(func);

	_inFunc = prev;

	return (result);
}

bool
SourcePortASTVisitor::VisitStmt (Stmt *stmt)
{
	SourceLocation		usedAt;
	SmallString<255>	sbuf;
	string			usr;

	const auto &loc = stmt->getLocStart();
	if (loc.isInvalid())
		return (true);
	
	const auto &definedAt = _state.srcManager().getSpellingLoc(loc);
	if (loc.isMacroID())
		usedAt = _state.srcManager().getExpansionLoc(loc);
	else
		usedAt = loc;

	if (!_state.isHostRef(usedAt, definedAt))
		return (true);

	if (loc.isMacroID()) {
		auto &cpp = _state.c().getPreprocessor();
		auto mname = cpp.getImmediateMacroName(stmt->getLocStart());

		recordSymbolUseVisit(stmt, mname);
	} else {
		auto &diags = _state.ast().getDiagnostics();
		unsigned diagID = diags.getCustomDiagID(DiagnosticsEngine::Error, "unsupported statement");
		diags.Report(loc, diagID) << stmt->getSourceRange();
	}

	return (true);
}

symtab::Location
SourcePortASTVisitor::getLocation (const clang::SourceLocation &loc)
{
	unsigned line, column;

	line = 0;
	column = 0;

	assert(loc.isValid());

	const auto &smgr = _state.srcManager();
	auto spellLoc = smgr.getSpellingLoc(loc);
	auto locInfo = smgr.getDecomposedLoc(spellLoc);
	auto fid = locInfo.first;
	auto fileOffset = locInfo.second;

	assert(fid.isValid());

	auto file = smgr.getFileEntryForID(fid);
	assert (file != NULL);

	line = smgr.getLineNumber(fid, fileOffset);
	column = smgr.getColumnNumber(fid, fileOffset);

	auto path = _state.syms()->getPath(file->getName());
	return (Location(path, line, column));
}

void
SourcePortASTVisitor::recordSymbolUseVisit (Stmt *useExpr, StringRef macroName)
{
	SmallString<255>	sbuf;
	shared_ptr<string>	USR;

	auto usedAt = _state.srcManager().getExpansionLoc(useExpr->getLocStart());
	auto &cpp = _state.c().getPreprocessor();
	auto *mid = cpp.getIdentifierInfo(macroName);
	MacroDefinition mdef = cpp.getMacroDefinitionAtLoc(mid, useExpr->getLocStart());
	MacroInfo *info = mdef.getMacroInfo();

	auto mrange = SourceRange(info->getDefinitionLoc(), info->getDefinitionEndLoc());
	auto mrec = MacroDefinitionRecord(mid, mrange);

	/* Skip argument expansion of our own macros */
	if (_state.srcManager().isMacroArgExpansion(useExpr->getLocStart())) {
		if (_state.isSourceLoc(mrange.getBegin()) && !_state.isHostLoc(mrange.getBegin()))
			return;
	}

	/* Generate USR */
	if (generateUSRForMacro(&mrec, _state.srcManager(), sbuf))
		return;

	USR = make_shared<string>(sbuf.str());

	/* Register symbol use */
	auto symbolUse = make_shared<SymbolUse>(
		SymbolUseExpr { ftl::constructor<Stmt *>(), useExpr },
		_inFunc == nullptr ? ftl::Nothing() : ftl::just(_inFunc),
		getLocation(usedAt),
		USR
	);

	_state.syms()->addSymbolUse(symbolUse);

	/* If the macro symbol itself has previously been registered, nothing
	 * left to do. */
	if (_state.syms()->hasUSR(*USR))
		return;

	/* Register the symbol */
	auto symbol = make_shared<Symbol>(
		make_shared<string>(macroName),
		SymbolDecl { ftl::constructor<MacroInfo*>(), info },
		getLocation(info->getDefinitionLoc()),
		USR
	);

	_state.syms()->addSymbol(symbol);
}

void
SourcePortASTVisitor::recordSymbolUseVisit (DeclRefExpr *useExpr, NamedDecl *declExpr)
{
	SmallString<255>	sbuf;
	shared_ptr<string>	USR, name;

	/* Generate USR */
	if (generateUSRForDecl(declExpr, sbuf))
		return;

	USR = make_shared<string>(sbuf.str());

	/* Fetch symbol name */
	name = make_shared<string>(declExpr->getName());

	/* Register symbol use */
	auto symbolUse = make_shared<SymbolUse>(
		SymbolUseExpr { ftl::constructor<DeclRefExpr *>(), useExpr },
		_inFunc == nullptr ? ftl::Nothing() : ftl::just(_inFunc),
		getLocation(useExpr->getLocation()),
		USR
	);

	_state.syms()->addSymbolUse(symbolUse);

	/* If the symbol itself has previously been registered, nothing left
	 * to do. */
	if (_state.syms()->hasUSR(*USR))
		return;

	/* Track single-level macro expansion */
	auto declLoc = declExpr->getLocation();
	if (!_state.hasFileEntry(declLoc) && declLoc.isMacroID())
		declLoc = _state.srcManager().getExpansionLoc(declLoc);

	/* Register the symbol */
	auto symbol = make_shared<Symbol>(
		name,
		SymbolDecl { ftl::constructor<NamedDecl*>(), declExpr },
		getLocation(declLoc),
		USR
	);

	_state.syms()->addSymbol(symbol);
}

bool
SourcePortASTVisitor::VisitDeclRefExpr (clang::DeclRefExpr *decl)
{
	if (!_state.isHostRef(decl))
		return (true);

	recordSymbolUseVisit(decl, decl->getFoundDecl());

	return (true);
};
