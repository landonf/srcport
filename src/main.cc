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

#include <clang/Rewrite/Core/Rewriter.h>

#include <clang/Index/USRGeneration.h>

#include "llvm/Support/CommandLine.h"

#include "unit_type.hpp"

#include "compiler.hh"
#include "ast_index.hh"
#include "project.hh"

#include "default_printer.hh"
#include "bwn_printer.hh"

using namespace std;

using namespace llvm;

using namespace clang;
using namespace clang::tooling;
using namespace clang::index;

using namespace pl;

using namespace symtab;
using namespace srcport;

enum OutputFormatter {
	OUTPUT_COMPAT_HEADER,	/**< A generic compatibility header format */
	OUTPUT_BWN_STUBS,	/**< FreeBSD bwn(4) driver stubs */
	OUTPUT_NONE		/**< No output */
};

static llvm::cl::OptionCategory PortToolCategory("port options");
static cl::extrahelp CommonHelp(CommonOptionsParser::HelpMessage);

static cl::list<string> HostPaths("host-path", cl::cat(PortToolCategory),
      cl::desc("Mark all definitions vended within this directory or file as unavailable on the target system"));

static cl::list<string> SourcePaths("src-path", cl::cat(PortToolCategory),
    cl::desc("Perform portability analysis within this directory or source file"));

static cl::list<string> RootPaths("root-path", cl::cat(PortToolCategory),
    cl::desc("Trim this path prefix when emitting source locations"));

cl::opt<OutputFormatter> OutputFormat("format", cl::cat(PortToolCategory),
  cl::desc("Choose output format:"),
  cl::values(
    clEnumValN(OUTPUT_COMPAT_HEADER,	"compat_header",	"A generic compatibility header"),
    clEnumValN(OUTPUT_BWN_STUBS,	"bwn_stubs", 		"FreeBSD bwn(4) driver stubs"),
    clEnumValN(OUTPUT_NONE,		"none",			"Disable output (perform indexing only)"),
     clEnumValEnd)
);

int main(int argc, const char **argv) {
	using ftl::operator>>=;

	CommonOptionsParser opts(argc, argv, PortToolCategory);
		
	/* Build our project configuration from our command line options */
	auto project = make_shared<Project>(
		PathPattern(*&SourcePaths),
		PathPattern(*&HostPaths),
		PathPattern(*&RootPaths)
	);

	/* Instantiate our compiler instance */
	auto compiler = Compiler::Create(opts.getCompilations(),
	    opts.getSourcePathList());

	/* Build our AST index */
	auto index = compiler >>= [&](const CompilerRef &cc) {
		return (ASTIndex::Build(project, cc));
	};

	/* Emit our header */
	auto ret = index >>= [&](const ASTIndexRef &idx) {
		switch (OutputFormat.getValue()) {
		case OUTPUT_COMPAT_HEADER:
			return (emit_compat_header(project, idx, llvm::outs()));
		case OUTPUT_BWN_STUBS:
			return (emit_bwn_stubs(idx, llvm::outs()));
		case OUTPUT_NONE:
			return (yield(Unit()));
		}
	};

	return (ret.match(
		[](const Error &e) {
			llvm::errs() << "processing failed: " <<
			    e.message() << "\n";
			return (EXIT_FAILURE);
		},
		[](const ftl::otherwise &) {
			return (EXIT_SUCCESS);
		}
	));
}
