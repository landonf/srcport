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

#include "project.hh"

/**
 * Return true if @p path should be included in symbol reference indexing.
 */
bool
Project::isReferencePath (const Path &path) const
{
	return (!_excludePaths.match(path) && _sourcePaths.match(path));
}

/**
 * Return true if @p path should be included in symbol definition indexing.
 */
bool
Project::isDefinitionPath (const Path &path) const
{
	if (_excludePaths.match(path))
		return (false);

	if (_sourcePaths.match(path))
		return (false);

	return (_hostPaths.match(path));
}

/**
 * Return true if @p astUnit should be used to index symbol references.
 */
bool
Project::isReferenceAST(const clang::ASTUnit &astUnit) const
{
	auto file = astUnit.getMainFileName();
	return (isReferencePath(Path(file)));
}

/**
 * Return true if @p astUnit should be used to index symbol definitions.
 */
bool
Project::isDefinitionAST(const clang::ASTUnit &astUnit) const
{
	auto file = astUnit.getMainFileName();
	return (isDefinitionPath(Path(file)));
}
