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

#include "clang/AST/ASTConsumer.h"
#include "clang/AST/RecursiveASTVisitor.h"

#include "clang/Tooling/Tooling.h"
#include "clang/Tooling/CommonOptionsParser.h"

#include "llvm/Support/CommandLine.h"

#include "Project.hpp"

using namespace clang;
using namespace clang::tooling;
using namespace llvm;
using namespace std;


static llvm::cl::OptionCategory PortToolCategory("port options");
static cl::extrahelp CommonHelp(CommonOptionsParser::HelpMessage);

static cl::list<string> HostPaths("host-path", cl::cat(PortToolCategory), cl::desc("Mark all definitions vended within this directory or file as unavailable on the target system"));
static cl::list<string> SourcePaths("src-path", cl::cat(PortToolCategory), cl::desc("Perform portability analysis within this directory or source file"));

class PortabilityVisitor: public RecursiveASTVisitor<PortabilityVisitor> {
public:
	explicit PortabilityVisitor (shared_ptr<Project> &proj, ASTContext *ctx) : _proj(proj), _ctx(ctx), _smgr(ctx->getSourceManager())
	{}
	
	virtual bool VisitDeclRefExpr (DeclRefExpr *decl) {
		const auto &loc = _ctx->getFullLoc(decl->getLocStart());
		if (!loc.isValid())
			return (true);

		const FileEntry *fent = _smgr.getFileEntryForID(_smgr.getFileID(loc));
		
		return (true);
	};

private:
	shared_ptr<Project> _proj;
	ASTContext *_ctx;
	SourceManager &_smgr;
};

class PortabilityConsumer : public clang::ASTConsumer {
public:
	explicit PortabilityConsumer (shared_ptr<Project> &proj, ASTContext *ctx) : _visitor(proj, ctx) {}

	virtual void HandleTranslationUnit(clang::ASTContext &ctx) {
		_visitor.TraverseDecl(ctx.getTranslationUnitDecl());
	}

private:
	PortabilityVisitor _visitor;
};

class PortabilityAction : public clang::ASTFrontendAction {
public:
	PortabilityAction (shared_ptr<Project> &proj) : _proj(proj) {}

	virtual unique_ptr<clang::ASTConsumer> CreateASTConsumer(clang::CompilerInstance &c, llvm::StringRef inFile) {
		llvm::outs() << "In file: " << inFile << "\n";
		return unique_ptr<clang::ASTConsumer>(new PortabilityConsumer(_proj, &c.getASTContext()));
	}
private:
	shared_ptr<Project> _proj;
};


unique_ptr<FrontendActionFactory> newFrontendAnalysisActionFactory(shared_ptr<Project> &proj) {
  class SimpleFrontendActionFactory : public FrontendActionFactory {
  public:
	  SimpleFrontendActionFactory(shared_ptr<Project> &proj) : _proj(proj) {}

	  clang::FrontendAction *create() override { return new PortabilityAction(_proj); }
	  
  private:
	  shared_ptr<Project> _proj;
  };

  return unique_ptr<FrontendActionFactory>(new SimpleFrontendActionFactory(proj));
}


int main(int argc, const char **argv) {
	CommonOptionsParser opts(argc, argv, PortToolCategory);
	ClangTool tool(opts.getCompilations(), opts.getSourcePathList());

	auto proj = make_shared<Project>(*&SourcePaths, *&HostPaths);
	return (tool.run(newFrontendAnalysisActionFactory(proj).get()));
}
