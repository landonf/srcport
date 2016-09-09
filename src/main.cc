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

#include "cvisitor.hh"
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


class SourcePortASTConsumer : public clang::ASTConsumer {
public:
	explicit SourcePortASTConsumer (VisitorState &&state) : _visitor(state) {}
	virtual void HandleTranslationUnit(clang::ASTContext &ctx) {
		_visitor.TraverseDecl(ctx.getTranslationUnitDecl());
	}

private:
	SourcePortASTVisitor _visitor;
};

class SourcePortFrontendAction : public clang::ASTFrontendAction {
public:
	SourcePortFrontendAction (shared_ptr<SymbolTable> &syms) : _syms(syms) {}

	virtual unique_ptr<clang::ASTConsumer>
	CreateASTConsumer(clang::CompilerInstance &c, llvm::StringRef inFile)
	{
		llvm::outs() << "Processing " << inFile << "\n";

		return unique_ptr<clang::ASTConsumer>(
		    new SourcePortASTConsumer(VisitorState(_syms, c)));
	}
private:
	shared_ptr<SymbolTable> _syms;
};

unique_ptr<FrontendActionFactory> newFrontendAnalysisActionFactory(shared_ptr<SymbolTable> &syms) {
	class SimpleFrontendActionFactory : public FrontendActionFactory {
	public:
		SimpleFrontendActionFactory(shared_ptr<SymbolTable> &syms) : _syms(syms) {}

		clang::FrontendAction *
		create() override
		{
			return (new SourcePortFrontendAction(_syms));
		}
		  
	private:
		  shared_ptr<SymbolTable> _syms;
	};

	return unique_ptr<FrontendActionFactory>(new SimpleFrontendActionFactory(syms));
}

int main(int argc, const char **argv) {
	int ret;
	
	CommonOptionsParser opts(argc, argv, PortToolCategory);
	ClangTool tool(opts.getCompilations(), opts.getSourcePathList());

	/* We need preprocessing information to track defines properly */
	tool.appendArgumentsAdjuster(getInsertArgumentAdjuster(
	    {"-Xclang", "-detailed-preprocessing-record"},
	    ArgumentInsertPosition::END)
	);

	auto symtab = make_shared<SymbolTable>(
	    Project(PathPattern(*&SourcePaths), PathPattern(*&HostPaths))
	);

	ret = tool.run(newFrontendAnalysisActionFactory(symtab).get());
	if (ret != 0)
		return (ret);

	// TODO: make use of syms
	for (const auto &sym : symtab->getSymbols()) {
		llvm::outs() << *sym->name() << "\n";
	}

	return (0);
}
