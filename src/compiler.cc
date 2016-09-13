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

#include "compiler.hh"

using namespace std;

using namespace clang;
using namespace clang::tooling;

result<CompilerRef>
Compiler::Create(const Compiler::CompilationDatabase &cdb,
    vector<string> sourcePaths)
{
	int ret;

	/* Instantiate our clang tool instance */
	auto cctool = unique_ptr<ClangTool>(new ClangTool(cdb, sourcePaths));

	/* We need preprocessing information to track defines properly */
	cctool->appendArgumentsAdjuster(getInsertArgumentAdjuster(
	    {"-Xclang", "-detailed-preprocessing-record"},
	    ArgumentInsertPosition::END)
	);

	/* Try to build our AST list */
	auto asts = unique_ptr<ASTUnitList>(new ASTUnitList());
	if ((ret = cctool->buildASTs(*asts))) {
		return (failed<CompilerRef>(Error("Compiler invocation failed",
		    ENXIO)));
	}

	/* Return our compiler instance */
	return (success(make_shared<Compiler>(cctool, asts, AllocKey{})));
}

Compiler::Compiler(Compiler::ClangToolPtr &cctool, Compiler::ASTUnitListPtr &asts,
    const Compiler::AllocKey &key):
    _cctool(std::move(cctool)), _asts(std::move(asts))
{

}
