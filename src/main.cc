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

#include "llvm/Support/CommandLine.h"

#include "unit_type.hpp"

#include "compiler.hh"
#include "ast_index.hh"
#include "project.hh"

using namespace std;

using namespace llvm;
using namespace clang::tooling;
using namespace clang;

using namespace pl;

using namespace symtab;

static llvm::cl::OptionCategory PortToolCategory("port options");
static cl::extrahelp CommonHelp(CommonOptionsParser::HelpMessage);

static cl::list<string> HostPaths("host-path", cl::cat(PortToolCategory),
      cl::desc("Mark all definitions vended within this directory or file as unavailable on the target system"));

static cl::list<string> SourcePaths("src-path", cl::cat(PortToolCategory),
    cl::desc("Perform portability analysis within this directory or source file"));

static result<Unit>	emit_compat_header(const ASTIndexRef &idx);

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

static void
printValueType(raw_ostream &os, QualType &type, const string &name,
    const PrintingPolicy &policy)
{
	QualType		 pointee = type;
	const PointerType	*ptype = nullptr;
	const ReferenceType	*rtype = nullptr;
	string			 ptrStr = "";

	/* Unwrap pointer/reference types */
	while (isa<PointerType>(pointee) || isa<ReferenceType>(pointee)) {
		while ((ptype = dyn_cast<PointerType>(pointee))) {
			ptrStr += "*";
			pointee = ptype->getPointeeType();
		}

		while ((rtype = dyn_cast<ReferenceType>(pointee))) {
			ptrStr += "&";
			pointee = rtype->getPointeeType();
		}
	}

	/* Emit the base type */
	pointee.print(os, policy);

	/* Emit indentation and pointer/reference string */
	if (ptrStr.size() > 0 || name.size() > 0) {
		os << " " << ptrStr << name;
	}
}

static result<Unit>
printAttributes(raw_ostream &os, const Decl *decl, const PrintingPolicy &policy,
    ASTContext &ast)
{
	auto &diags = ast.getDiagnostics();

	if (!decl->hasAttrs())
		return (yield(Unit()));

	for (auto &attr : decl->getAttrs()) {
		switch (attr->getKind()) {
		#define ATTR(X)
		#define PRAMGA_SPELLING_ATTR(_Name) case attr::_Name:
		#include <clang/Basic/AttrList.inc>
		{
			unsigned diagID = diags.getCustomDiagID(
			    DiagnosticsEngine::Error,
			    "MS #pragma-style attributes are unsupported");
			    auto loc = attr->getLocation();
			    diags.Report(loc, diagID) << attr->getRange();

			return (fail<Unit>(string("Error emitting attribute")));
		}
		default:
			attr->printPretty(os, policy);
		}
	}

	return (yield(Unit()));
}

static result<Unit>
printFunctionDecl(raw_ostream &os, const FunctionDecl *decl,
    const PrintingPolicy &policy, ASTContext &ast)
{
	const CXXConstructorDecl	*cxxConstrDecl;
	const CXXConversionDecl		*cxxConvDecl;
	const ParenType			*parenType;
	const FunctionProtoType		*fnType;
	string				 fnName;
	auto				&diags = ast.getDiagnostics();
	size_t				 nargs, nexceptions;
	bool				 isMethod;

	fnName = decl->getNameInfo().getAsString();
	nargs = decl->getNumParams();

	/* We handle method definitions distinctly from C functions */
	isMethod = isa<CXXMethodDecl>(decl);

	/* Storage class. */
	switch (decl->getStorageClass()) {
	case SC_None:
		break;
	case SC_Static:
		/* We ignore non-method 'static' designators (along with
		 * 'inline'), since the goal here is to emit a working
		 * forward declaration. */
		if (isMethod)
			os << "static ";
		break;
	case SC_PrivateExtern:
		/* clang extension; symbol is private to its enclosing module */
		os << "__module_private__ ";
		break;	
	case SC_Extern:
		os << "extern ";
		break;
	case SC_Auto:
	case SC_Register:
		unsigned diagID = diags.getCustomDiagID(
		    DiagnosticsEngine::Error, "unsupported storage class");
		auto loc = decl->getLocStart();
		diags.Report(loc, diagID) << decl->getSourceRange();

		return (fail<Unit>(string("Error emitting declaration for ") +
		    fnName));
	}

	/* Inline functions. (Intentionally ignored for non-methods. See
	 * SC_Static above) */
	if (isMethod && decl->isInlineSpecified())
		os << "inline ";

	/* C++ */
	if (decl->isVirtualAsWritten())
		os << "virtual ";
	if (decl->isConstexpr() && decl->isExplicitlyDefaulted())
		os << "constexpr ";

	cxxConstrDecl = dyn_cast<CXXConstructorDecl>(decl);
	cxxConvDecl = dyn_cast<CXXConversionDecl>(decl);
	if (cxxConstrDecl && cxxConvDecl) {
		if (cxxConstrDecl->isExplicitSpecified() &&
		    cxxConvDecl->isExplicit())
		{
			os << "explicit ";
		}
	}

	/* Unwrap type paren sugar */
	string parenName = fnName;
	auto declType = decl->getType();
	while ((parenType = dyn_cast<ParenType>(declType))) {
		parenName = "(" + parenName + ")";
		declType = parenType->getInnerType();
	}

	/* Fetch function prototype */
	if (!(fnType = dyn_cast<FunctionProtoType>(declType))) {
		unsigned diagID = diags.getCustomDiagID(
		    DiagnosticsEngine::Error,
		    "missing function prototype; can't emit declaration");
		auto loc = decl->getLocStart();
		diags.Report(loc, diagID) << decl->getSourceRange();

		return (fail<Unit>(string("Error emitting declaration for ") +
		    fnName));
	}

	/* Emit non-trailing return type (if any), and (possibly paren-wrapped)
	 * function name */
	if (!fnType->hasTrailingReturn()) {
		auto retType = fnType->getReturnType();
		printValueType(os, retType, parenName, policy);
	} else {
		os << parenName;
	}

	/* Emit arguments */
	os << "(";
	for (size_t i = 0; i < nargs; i++) {
		auto pdecl = decl->getParamDecl(i);
		auto type = pdecl->getTypeSourceInfo()->getType();

		if (i > 0)
			os << ", ";

		printValueType(os, type, pdecl->getNameAsString(), policy);
	}
	if (decl->isVariadic()) {
		if (nargs > 0)
			os << ", ";

		os << "...";
	}
	os << ")";

	/* 
	 * C++ qualifiers
	 */
	if (fnType->isConst())
		os << " const";
	if (fnType->isVolatile())
		os << " volatile";
	if (fnType->isRestrict())
		os << " restrict";
	switch (fnType->getRefQualifier()) {
	case RQ_None:
		break;
	case RQ_LValue:
		os << "&";
		break;
	case RQ_RValue:
		os << "&&";
		break;
	}

	nexceptions = fnType->getNumExceptions();
	switch (fnType->getExceptionSpecType()) {
	case EST_None:
		break;
	case EST_DynamicNone:
		break;
	case EST_Dynamic:
		os << " throw(";
		for (size_t i = 0; i < nexceptions; i++) {
			auto etype = fnType->getExceptionType(i);
			if (i > 0)
				os << ", ";
			os << etype.getAsString(policy);
		}
		os << ")";
		break;
	case EST_MSAny:
		os << " throw(...)";
		break;
	case EST_BasicNoexcept:
		os << " noexcept";
		break;
	case EST_ComputedNoexcept:
		os << " noexcept(";
		fnType->getNoexceptExpr()->printPretty(os, nullptr, policy);
		os << ")";
		break;
		
	case EST_Unevaluated:
	case EST_Uninstantiated:
	case EST_Unparsed:
		unsigned diagID = diags.getCustomDiagID(
		    DiagnosticsEngine::Error,
		    "unsupported exception spec; can't emit declaration");
		auto loc = decl->getLocStart();
		diags.Report(loc, diagID) << decl->getSourceRange();

		return (fail<Unit>(string("Error emitting declaration for ") +
		    fnName));
	}

	/* Attributes */
	auto attrResult = printAttributes(os, decl, policy, ast);
	if (!attrResult.is<ftl::Right<Unit>>())
		return (attrResult);

	/* C++ pure/default/delete */
	if (decl->isPure())
		os << " = 0";
	else if (decl->isExplicitlyDefaulted())
		os << " = default";
	else if (decl->isDeletedAsWritten())
		os << " = delete";

	return (yield(Unit()));
}

/**
 * Emit our bwn/siba compatibility header. In some future iteration, we'll
 * replace this with a more general-purpose API.
 */
static result<Unit>
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
		auto &astContext = astUnit.getASTContext();
		auto &langOpts = astUnit.getLangOpts();
		auto &cpp = astUnit.getPreprocessor();

		PrintingPolicy printPolicy(langOpts);
		printPolicy.PolishForDeclaration = 1;
		sym->cursor().node().matchE(
			[&](const Decl *decl) {
				if (isa<FunctionDecl>(decl)) {
					auto func = dyn_cast<FunctionDecl>(decl);
					printFunctionDecl(os, func,
					    printPolicy, astContext);
				} else {
					decl->print(os, printPolicy);
				}
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

	return (yield(Unit()));
}
