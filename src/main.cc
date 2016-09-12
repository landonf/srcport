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

#include "clang/Frontend/FrontendAction.h"
#include "clang/Frontend/FrontendActions.h"
#include "clang/Frontend/CompilerInstance.h"

#include "clang/Tooling/Tooling.h"
#include "clang/Tooling/CommonOptionsParser.h"

#include "llvm/Support/CommandLine.h"

#include "ast_index.hh"
#include "project.hh"

using namespace clang;
using namespace clang::tooling;
using namespace llvm;
using namespace std;
using namespace symtab;

static llvm::cl::OptionCategory PortToolCategory("port options");
static cl::extrahelp CommonHelp(CommonOptionsParser::HelpMessage);

static cl::list<string> HostPaths("host-path", cl::cat(PortToolCategory),
      cl::desc("Mark all definitions vended within this directory or file as unavailable on the target system"));

static cl::list<string> SourcePaths("src-path", cl::cat(PortToolCategory),
    cl::desc("Perform portability analysis within this directory or source file"));

int main(int argc, const char **argv) {
	CommonOptionsParser opts(argc, argv, PortToolCategory);
		
	/* Build our project configuration from our command line options */
	auto project = make_shared<Project>(
		PathPattern(*&SourcePaths), PathPattern(*&HostPaths)
	);

	/* Instantiate our clang tool instance */
	auto cctool = make_shared<ClangTool>(opts.getCompilations(), opts.getSourcePathList());

	/* We need preprocessing information to track defines properly */
	cctool->appendArgumentsAdjuster(getInsertArgumentAdjuster(
	    {"-Xclang", "-detailed-preprocessing-record"},
	    ArgumentInsertPosition::END)
	);

	/* Build our AST index */
	auto index = ASTIndex::Index(project, cctool);

	// TODO: Use the ASTIndex for something.

	return (0);
}
