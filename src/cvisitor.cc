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

#include "cvisitor.hh"

#include <clang/Index/USRGeneration.h>

using namespace clang;
using namespace clang::index;
using namespace std;

bool
VisitorState::hasFileEntry (SourceLocation &loc) const
{
	if (loc.isInvalid())
		return (false);

	const auto &fid = _srcManager.getFileID(loc);
	if (fid.isInvalid())
		return (false);

	auto fentry = _srcManager.getFileEntryForID(fid);
	return (fentry != NULL);
};

bool
VisitorState::locMatches (clang::SourceLocation &loc, const PathPattern &p) const
{
	if (!loc.isValid())
		return (false);

	auto fid = _srcManager.getFileID(loc);
	if (fid.isInvalid())
		return (false);

	auto fentry = _srcManager.getFileEntryForID(fid);
	if (fentry == NULL)
		return (false);

	return (p.match(fentry->getName()));
}

/**
 * Returns true if this is a symbol reference that 1) occurs in the sources
 * to be ported, and 2) references a symbol defined by a host path.
 */
bool
VisitorState::isHostRef (clang::DeclRefExpr *ref) const
{
	/* Must be found within a source path */
	auto rloc = ref->getLocation();
	if (!locMatches(rloc, _syms->project().sourcePaths()))
		return (false);

	/* Must reference a symbol defined in a host path */
	auto target = ref->getFoundDecl()->getCanonicalDecl();
	auto tloc = target->getLocation();

	/* Track single-level macro expansion */
	if (!hasFileEntry(tloc) && tloc.isMacroID())
		tloc = _ast.getFullLoc(_srcManager.getExpansionLoc(tloc));

	/* Explicit host path match */
	if (locMatches(tloc, _syms->project().hostPaths()))
		return (true);

	/* Otherwise, exclude if source path */
	if (locMatches(tloc, _syms->project().sourcePaths()))
		return (false);

	/* Assume host symbol ref */
	return (true);
}

// XXX temporary
static unordered_set<string> seen;

bool
SourcePortASTVisitor::VisitDeclRefExpr (clang::DeclRefExpr *decl)
{
	SmallString<255>	sbuf;
	string			usr;

	if (!_state.isHostRef(decl))
		return (true);

	if (generateUSRForDecl(decl->getDecl()->getCanonicalDecl(), sbuf))
		return (true);

	usr = sbuf.str();
	if (seen.count(usr) > 0)
		return (true);
	else
		seen.emplace(usr);

	llvm::outs() << usr << "\n";


	return (true);
};
