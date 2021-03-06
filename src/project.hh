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

#ifndef _SRCPORT_PROJECT_HH_
#define _SRCPORT_PROJECT_HH_

#include <string>
#include <vector>

#include <clang/Frontend/ASTUnit.h>

#include <ftl/functional.h>
#include <ftl/vector.h>
#include <ftl/sum_type.h>

#include "error.hh"
#include "paths.hh"

class Project {
public:
    Project (PathPattern &&sourcePaths, PathPattern &&hostPaths,
	PathPattern &&excludePaths, PathPattern &&rootPaths) :
        _sourcePaths(std::move(sourcePaths)),
        _hostPaths(std::move(hostPaths)),
        _excludePaths(std::move(excludePaths)),
        _rootPaths(std::move(rootPaths)) {}

    const PathPattern &hostPaths () const { return _hostPaths; }
    const PathPattern &sourcePaths () const { return _sourcePaths; }
    const PathPattern &excludePaths () const { return _excludePaths; }
    const PathPattern &rootPaths () const { return _rootPaths; }

    bool isReferencePath (const Path &path) const;
    bool isDefinitionPath (const Path &path) const;

    bool isReferenceAST (const clang::ASTUnit &astUnit) const;
    bool isDefinitionAST (const clang::ASTUnit &astUnit) const;

private:
    PathPattern _sourcePaths;
    PathPattern _hostPaths;
    PathPattern _excludePaths;
    PathPattern _rootPaths;
};

using ProjectRef = std::shared_ptr<Project>;

#endif /* _SRCPORT_PROJECT_HH_ */
