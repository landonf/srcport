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

// XXX temporary
static unordered_set<string> seen;

bool SourcePortASTVisitor::VisitDeclaratorDecl (clang::DeclaratorDecl *decl)
{	
	const clang::Decl	*defn;
	const clang::Type	*t;
	SmallString<255>	 sbuf;
	string			 usr;

	if (!_state.isSourceLoc(decl->getLocation()))
		return (true);

	t = decl->getType().getTypePtr();
	defn = _state.getTypeDecl(t, decl->getTypeSourceInfo());
	if (defn == nullptr)
		return (true);

	if (!_state.isHostRef(decl->getLocation(), defn->getLocation()))
		return (true);

	if (generateUSRForDecl(defn, sbuf))
		return (true);

	usr = sbuf.str();
	if (seen.count(usr) > 0)
		return (true);
	else
		seen.emplace(usr);

	llvm::outs() << usr << "\n";

	return (true);
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

		auto *mid = cpp.getIdentifierInfo(mname);
		MacroDefinition mdef = cpp.getMacroDefinitionAtLoc(mid, stmt->getLocStart());
		MacroInfo *info = mdef.getMacroInfo();

		auto mrange = SourceRange(info->getDefinitionLoc(), info->getDefinitionEndLoc());
		auto mrec = MacroDefinitionRecord(mid, mrange);

		/* Skip argument expansion of our own macros */
		if (_state.srcManager().isMacroArgExpansion(loc)) {
			if (_state.isSourceLoc(mrange.getBegin()) && !_state.isHostLoc(mrange.getBegin()))
				return (true);
		}

		if (generateUSRForMacro(&mrec, _state.srcManager(), sbuf))
			return (true);

		usr = sbuf.str();
		if (seen.count(usr) > 0)
			return (true);
		else
			seen.emplace(usr);

		llvm::outs() << usr << "\n";
	} else {
		auto &diags = _state.ast().getDiagnostics();
		unsigned diagID = diags.getCustomDiagID(DiagnosticsEngine::Error, "unsupported statement");
		diags.Report(loc, diagID) << stmt->getSourceRange();
	}

	return (true);
}

bool
SourcePortASTVisitor::VisitDeclRefExpr (clang::DeclRefExpr *decl)
{
	SmallString<255>	sbuf;
	string			usr;

	if (generateUSRForDecl(decl->getFoundDecl()->getCanonicalDecl(), sbuf)) {
		return (true);
	}

	usr = sbuf.str();
	if (seen.count(usr) > 0)
		return (true);
	else
		seen.emplace(usr);

	if (!_state.isHostRef(decl))
		return (true);

	llvm::outs() << usr << "\n";

	if (_inFunc != nullptr) {
		auto *id = _inFunc->getIdentifier();
		llvm::outs() << "IN: " << id->getName() << "\n";
	}

	return (true);
};
