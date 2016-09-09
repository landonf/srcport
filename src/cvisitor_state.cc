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

#include <string>

#include "cvisitor_state.hh"

using namespace clang;
using namespace std;

bool
VisitorState::hasFileEntry (const SourceLocation &loc) const
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
VisitorState::locMatches (const clang::SourceLocation &loc, const PathPattern &p) const
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
	SourceLocation	 usedAt, definedAt;
	Decl		*target;

	usedAt = ref->getLocation();

	target = ref->getFoundDecl()->getCanonicalDecl();
	definedAt = target->getLocation();

	/* Track single-level macro expansion */
	if (!hasFileEntry(definedAt) && definedAt.isMacroID())
		definedAt = _srcManager.getExpansionLoc(definedAt);

	return (isHostRef(usedAt, definedAt));
}

/**
 * Return true if @p usedAt falls within defined source paths, and @p definedAt
 * falls within defined host paths.
 */
bool
VisitorState::isHostRef (SourceLocation usedAt, SourceLocation definedAt) const
{
	if (usedAt.isInvalid() || definedAt.isInvalid())
		return (false);

	/* The use location must be within our defined source paths,
	 * but not explicitly excluded by a host path */
	if (!locMatches(usedAt, _syms->project().sourcePaths()))
		return (false);
	
	if (locMatches(usedAt, _syms->project().hostPaths()))
		return (false);

	/* The definition location must be within defined host path */
	if (!locMatches(definedAt, _syms->project().hostPaths()))
		return (false);

	return (true);
}

bool
VisitorState::isHostLoc (const SourceLocation &loc) const
{
	return (locMatches(loc, _syms->project().hostPaths()));
}

bool VisitorState::isSourceLoc (const SourceLocation &loc) const
{
	return (locMatches(loc, _syms->project().sourcePaths()));
}

string
VisitorState::descLoc (const SourceLocation &loc)
{
	string		desc;
	unsigned	line, column;

	line = 0;
	column = 0;

	if (loc.isInvalid())
		return ("<loc-invalid>:-1");

	auto spellLoc = _srcManager.getSpellingLoc(loc);
	auto locInfo = _srcManager.getDecomposedLoc(spellLoc);
	auto fid = locInfo.first;
	auto fileOffset = locInfo.second;

	if (fid.isInvalid())
		return ("<fid-invalid>:-1");

	auto file = _srcManager.getFileEntryForID(fid);
	if (file == NULL)
		return ("<null-file>:-1");
	
	line = _srcManager.getLineNumber(fid, fileOffset);
	column = _srcManager.getColumnNumber(fid, fileOffset);

	desc = file->getName();
	desc += ":" + to_string(line) + ":" + to_string(column);

	return (desc);
}

void
VisitorState::dumpLoc (const SourceLocation &loc)
{
	llvm::errs() << descLoc(loc) << "\n";
}

const NamedDecl *
VisitorState::getTypeDecl(const Type *t, const TypeSourceInfo *info) const
{
	if (const TagType *r = t->getAs<TagType>()) {
		return (r->getDecl()->getCanonicalDecl());

	} else if (const TypedefType *d = t->getAs<TypedefType>()) {
		return (d->getDecl());

	} else if (const PointerType *p = t->getAs<PointerType>()) {
		return (getTypeDecl(p->getPointeeType().getTypePtr(), info));

	} else if (t->isArrayType()) {
		return (getTypeDecl(t->castAsArrayTypeUnsafe()->getElementType().getTypePtr(), info));

	} else if (t->getAs<FunctionProtoType>()) {
		/* nothing */
		return (NULL);
	} else if (t->isBuiltinType()) {
		/* nothing */
		return (NULL);
	}

	auto &diags = _ast.getDiagnostics();
	unsigned diagID = diags.getCustomDiagID(DiagnosticsEngine::Error, "cannot analyze type");
	auto errLoc = info->getTypeLoc().getUnqualifiedLoc();

	diags.Report(errLoc.getLocStart(), diagID) << errLoc.getSourceRange();

	return (NULL);
}
