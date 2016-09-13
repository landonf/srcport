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

#ifndef _SRCPORT_COMPILER_HH_
#define _SRCPORT_COMPILER_HH_

#include <clang/Tooling/Tooling.h>

#include <clang/AST/AST.h>

#include <clang/ASTMatchers/ASTMatchers.h>
#include <clang/ASTMatchers/ASTMatchFinder.h>

#include "result.hh"

class Compiler;
using CompilerRef = std::shared_ptr<Compiler>;

/**
 * Compiler State
 */
class Compiler : public std::enable_shared_from_this<Compiler> {
private:
	struct AllocKey {};

	using CompilationDatabase = clang::tooling::CompilationDatabase;

	using ClangToolPtr = std::unique_ptr<clang::tooling::ClangTool>;
	using ASTUnitPtr = std::unique_ptr<clang::ASTUnit>;
	using ASTUnitList = std::vector<ASTUnitPtr>;
	using ASTUnitListPtr = std::unique_ptr<ASTUnitList>;

public:
	static result<CompilerRef>	Create(const CompilationDatabase &cdb,
					     std::vector<std::string> sourcePaths);

	Compiler (ClangToolPtr &tool, ASTUnitListPtr &asts,
	    const AllocKey &key);

private:
	ClangToolPtr		_cctool;	/**< clang tool instance */
	ASTUnitListPtr		_asts;		/**< parsed AST objects for our
						     input source files; these
						     own all AST nodes
						     allocations */

};

using CompilerRef = std::shared_ptr<Compiler>;

#endif /* _SRCPORT_COMPILER_HH_ */
