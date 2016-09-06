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

using namespace clang;
using namespace clang::tooling;
using namespace llvm;

// Apply a custom category to all command-line options so that they are the
// only ones displayed.
static llvm::cl::OptionCategory BwnToolCategory("bwn_deps options");

// CommonOptionsParser declares HelpMessage with a description of the common
// command-line options related to the compilation database and input files.
// It's nice to have this help message in all tools.
static cl::extrahelp CommonHelp(CommonOptionsParser::HelpMessage);

// A help message for this specific tool can be added afterwards.
static cl::extrahelp MoreHelp("\nMore help text...");

class FindNamedClassVisitor: public RecursiveASTVisitor<FindNamedClassVisitor> {
public:
	explicit FindNamedClassVisitor(ASTContext *ctx) : _ctx(ctx), _smgr(ctx->getSourceManager())
	{}
	
	virtual bool VisitDeclRefExpr (DeclRefExpr *decl) {
		const auto &loc = _ctx->getFullLoc(decl->getLocStart());
		if (!loc.isValid())
			return (true);

		const FileEntry *fent = _smgr.getFileEntryForID(_smgr.getFileID(loc));
		
		return (true);
	};

private:
	ASTContext *_ctx;
	SourceManager &_smgr;
};

class FindNamedClassConsumer : public clang::ASTConsumer {
public:
	explicit FindNamedClassConsumer(ASTContext *ctx) : _visitor(ctx) {}

	virtual void HandleTranslationUnit(clang::ASTContext &ctx) {
		_visitor.TraverseDecl(ctx.getTranslationUnitDecl());
	}

private:
	FindNamedClassVisitor _visitor;
};

class FindNamedClassAction : public clang::ASTFrontendAction {
public:
	virtual std::unique_ptr<clang::ASTConsumer> CreateASTConsumer(clang::CompilerInstance &c, llvm::StringRef inFile) {
		llvm::outs() << "In file: " << inFile << "\n";
		return std::unique_ptr<clang::ASTConsumer>(new FindNamedClassConsumer(&c.getASTContext()));
	}
};

int main(int argc, const char **argv) {
	CommonOptionsParser opts(argc, argv, BwnToolCategory);
	ClangTool tool(opts.getCompilations(), opts.getSourcePathList());
	return tool.run(newFrontendActionFactory<FindNamedClassAction>().get());
}
