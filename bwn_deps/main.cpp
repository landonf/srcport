//
//  main.mm
//  ccmach
//
//  Created by Landon Fuller on 12/31/15.
//  Copyright (c) 2015 Landon Fuller. All rights reserved.
//

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

// Apply a custom category to all command-line options so that they are the
// only ones displayed.
static llvm::cl::OptionCategory BwnToolCategory("bwn_deps options");

// CommonOptionsParser declares HelpMessage with a description of the common
// command-line options related to the compilation database and input files.
// It's nice to have this help message in all tools.
static cl::extrahelp CommonHelp(CommonOptionsParser::HelpMessage);

// A help message for this specific tool can be added afterwards.
static cl::extrahelp MoreHelp("\nMore help text...");

static cl::list<string> TargetPaths("target-path", cl::cat(BwnToolCategory));
static cl::list<string> SourcePaths("src-path", cl::cat(BwnToolCategory));

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
	CommonOptionsParser opts(argc, argv, BwnToolCategory);
	ClangTool tool(opts.getCompilations(), opts.getSourcePathList());

	auto proj = make_shared<Project>(*&SourcePaths, *&TargetPaths);
	return (tool.run(newFrontendAnalysisActionFactory(proj).get()));
}
