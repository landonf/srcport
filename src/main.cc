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

#include "unit_type.hpp"

#include "compiler.hh"
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

static result<pl::Unit>		emit_compat_header(const ASTIndexRef &idx);

int main(int argc, const char **argv) {
	using ftl::operator>>=;

	CommonOptionsParser opts(argc, argv, PortToolCategory);
		
	/* Build our project configuration from our command line options */
	auto project = make_shared<Project>(
		PathPattern(*&SourcePaths), PathPattern(*&HostPaths)
	);

	/* Instantiate our compiler instance */
	auto compiler = Compiler::Create(opts.getCompilations(),
	    opts.getSourcePathList());

	/* Build our AST index */
	auto index = compiler >>= [&](const CompilerRef &cc) {
		return (ASTIndex::Build(project, cc));
	};

	/* Emit our header */
	auto ret = index >>= emit_compat_header;

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


static void
printMacroArgs(raw_ostream &os, const MacroInfo &info)
{
	if (!info.isFunctionLike())
		return;

	os << "(";

	auto args = info.args();
	auto numArgs = args.size();
	for (size_t i = 0; i < numArgs; i++) {
		const auto &arg = args[i];
		if (i != 0)
			os << ", ";

		const auto &name = arg->getName();

		if (i+1 == numArgs && name == "__VA_ARGS__") {
			os << "...";
		} else {
			os << name;
		}
	}

	if (info.isGNUVarargs())
		os << "...";

	os << ")";
}

static void
printMacroDefinition(raw_ostream &os, const StrRef name, const MacroInfo &info,
    Preprocessor &cpp)
{
	os << "#define\t" << *name;
	printMacroArgs(os, info);

	if (info.getNumTokens() == 0)
		return;

	auto tokens = info.tokens();

	if (!tokens.begin()->hasLeadingSpace())
		os << "\t";

	for (const auto &token : tokens) {
		SmallString<128> sbuf;

		if (token.hasLeadingSpace())
			os << ' ';

		os << cpp.getSpelling(token, sbuf);
	}
}

/**
 * Emit our bwn/siba compatibility header. In some future iteration, we'll
 * replace this with a more general-purpose API.
 */
static result<pl::Unit>
emit_compat_header(const ASTIndexRef &idx)
{
	/* Sort symbols by file location */
	auto syms = vector<SymbolRef>(idx->getSymbols().begin(), idx->getSymbols().end());
	std::sort(syms.begin(), syms.end(), [&](const SymbolRef &lhs, const SymbolRef &rhs){
		return (lhs->location() < rhs->location());
	});

	/** Trims trailing newlines */
	auto rtrim = [](std::string &s) {
		s.erase(s.find_last_not_of("\r\n\t")+1);
	};

	/* Emit definitions */
	std::shared_ptr<Path> path;
	for (const auto &sym : syms) {
		std::string			output;
		llvm::raw_string_ostream	os(output);
		auto				USR = sym->USR();

		/* Emit declaration path  */
		auto symPath = sym->location().path();
		if (!path || *path != *symPath) {
			os << "\n/*\n * Declared in:\n" <<
			    " *    " << symPath->stringValue() << "\n */\n\n";
			path = symPath;
		}

		/* Don't bother printing individual enum constants */
		if (sym->cursor().node().match(
			[](const Decl *decl) {
				return (isa<EnumConstantDecl>(decl));
			},
			[](ftl::otherwise) { return (false); }
		))
		{
			continue;
		}

		auto &astUnit = *sym->cursor().unit();
		auto &langOpts = astUnit.getLangOpts();
		auto &cpp = astUnit.getPreprocessor();

		PrintingPolicy printPolicy(langOpts);
		printPolicy.PolishForDeclaration = 1;
		sym->cursor().node().matchE(
			[&](const Decl *decl) {
				decl->print(os, printPolicy);
				rtrim(os.str());
				os << ";";
			},
			[&](const Stmt *stmt) {
				stmt->printPretty(os, nullptr, printPolicy);
				rtrim(os.str());
				os << ";";
			},
			[&](MacroRef macro) {
				printMacroDefinition(os, sym->name(),
				   *macro.getMacroInfo(cpp), cpp);
			}
		);

		/* Flush and emit */
		os.str();
		if (output.size() > 0) {
			llvm::outs() << output;
			llvm::outs() << "\n\n";
		}
	}

	return (yield(pl::Unit()));
}
