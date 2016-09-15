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

#include "ast_match.hh"

using namespace std;

using namespace clang;

string
ASTMatchUtil::describe (const SourceLocation &loc) {
	string		desc;
	unsigned	line, column;

	line = 0;
	column = 0;

	if (loc.isInvalid())
		return ("<loc-invalid>");
	else if (loc.isMacroID()) {
		if (_srcManager.isMacroArgExpansion(loc))
			return ("<macro-arg>");
		else
			return ("<macro-body>");
	}

	auto locInfo = _srcManager.getDecomposedLoc(loc);
	auto fid = locInfo.first;
	auto fileOffset = locInfo.second;

	if (fid.isInvalid())
		return ("<fid-invalid>");

	auto file = _srcManager.getFileEntryForID(fid);
	if (file == NULL)
		return ("<null-file>");
	
	line = _srcManager.getLineNumber(fid, fileOffset);
	column = _srcManager.getColumnNumber(fid, fileOffset);

	desc = file->getName();
	desc += ":" + to_string(line) + ":" + to_string(column);

	return (desc);
}

void
ASTMatchUtil::dump (const SourceLocation &loc)
{
	llvm::errs() << describe(loc) << "\n";
}

void
ASTMatchUtil::dumpTree (const SourceLocation &loc, std::string::size_type indent)
{
	auto &os = llvm::outs();
	auto istr = string(indent*2, ' ');

	os << istr << describe(loc) << "\n";
	
	auto nistr = string(indent+1*2, ' ');

	if (loc.isMacroID()) {
		SourceLocation next;

		if (_srcManager.isMacroArgExpansion(loc)) {
			next = _srcManager.getImmediateSpellingLoc(loc);
		} else if (_srcManager.isMacroBodyExpansion(loc)) {
			next = _srcManager.getImmediateExpansionRange(loc).first;
		} else {
			return;
		}

		dumpTree(next, indent+1);
	}
}

bool
ASTMatchUtil::locMatches (const clang::SourceLocation &loc, const PathPattern &p)
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

ASTMatchUtil::loc_type
ASTMatchUtil::getLocationType (clang::SourceLocation loc)
{
	if (loc.isInvalid())
		return (LOC_INVALID);
	else if (loc.isMacroID())
		return (LOC_MACRO);

	/* Anything located within a host path is a host location */
	if (locMatches(loc, _project->hostPaths()))
		return (LOC_HOST);

	/* A source location must be within our defined source paths,
	 * but not explicitly excluded by a host path */
	if (locMatches(loc, _project->sourcePaths()))
		return (LOC_SOURCE);

	return (LOC_EXTERNAL);
}

/**
 * Return true if @p usedAt falls within defined source paths, and @p definedAt
 * falls within defined host paths.
 */
bool
ASTMatchUtil::isHostRef (SourceLocation usedAt, SourceLocation definedAt)
{
	if (getLocationType(usedAt) != LOC_SOURCE)
		return (false);

	if (getLocationType(definedAt) != LOC_HOST)
		return (false);

	return (true);
}

