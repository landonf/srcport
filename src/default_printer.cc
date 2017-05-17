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

#include <clang/Index/USRGeneration.h>
#include <clang/AST/AST.h>

#include <cstdio>

#include "unit_type.hpp"

#include "ast_index.hh"

#include "default_printer.hh"

using namespace std;

using namespace llvm;
using namespace clang;
using namespace clang::tooling;
using namespace clang::index;

using namespace pl;
using namespace ftl;
using namespace symtab;

static void		printMacroArgs(raw_ostream &os, const MacroInfo &info);
static void		printMacroDefinition(raw_ostream &os, const StrRef name,
			    const MacroInfo &info, Preprocessor &cpp);
static void		printValueType(raw_ostream &os, QualType &type,
			    const string &name, const PrintingPolicy &policy);
static result<Unit>	printAttributes(raw_ostream &os, const Decl *decl,
			    const PrintingPolicy &policy, ASTContext &ast);
static result<Unit>	printFunctionDecl(raw_ostream &os,
			    const FunctionDecl *decl,
			    const PrintingPolicy &policy, ASTContext &ast);
static void		printEnumDecl(raw_ostream &os, const EnumDecl *decl,
			    const PrintingPolicy &policy, ASTContext &ast,
			    const ASTIndexRef &idx);

/**
 * Enumerate the symbol references in @p idx and emit a compatibility header
 * to @p out.
 * 
 * This mainly serves as an example of how to produce working headers from
 * an ASTIndex until I have time to implement a generic combinator API for
 * pretty printing.
 */
result<Unit>
srcport::emit_compat_header(const ProjectRef &project, const ASTIndexRef &idx,
    llvm::raw_ostream &out)
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

	/* Trims root prefixes from our source paths */
	const auto &rootPaths = project->rootPaths();
	auto trimRoots = [&rootPaths](const PathRef &p) {
		return (rootPaths.trimPrefix(*p, true).stringValue());
	};

	/* Emit definitions */
	std::shared_ptr<Path> path;
	for (const auto &sym : syms) {
		std::string			output;
		llvm::raw_string_ostream	os(output);
		auto				USR = sym->USR();

		/* Don't bother printing individual enum constants */
		if (sym->cursor().as<EnumConstantDecl>())
			continue;

		/* Find all usages */
		std::vector<SymbolUseRef> uses;
		for (const auto &use : idx->getSymbolUses()) {
			if (use->symbol()->USR() == USR)
				uses.push_back(use);
		};

		/* Emit defined/usage header */
		std::unordered_set<Location> locSeen;

		os << "/* " << *sym->name() << "\n";
		os << " * \n";
		os << " * Declared at:\n";
		{
			auto loc = sym->location();
			auto relPath = trimRoots(loc.path());

			os << " *   " << relPath << ":";
			os << to_string(loc.line()) << "\n";
		}
		os << " *\n";
	
		os << " * Referenced by:\n";
		PathRef lastUsePath;
		size_t pathLineCount = 0;
		for (const auto &use : uses) {
			auto loc = use->location().column(0);
			if (locSeen.count(loc) > 0)
				continue;
			else
				locSeen.emplace(loc);

			auto relPath = trimRoots(loc.path());

			if (!lastUsePath || *lastUsePath != *loc.path()) {
				pathLineCount = 0;
				if (lastUsePath)
					os << "\n";

				lastUsePath = loc.path();
				os << " *   " << relPath << ":" <<
				    to_string(loc.line());
			} else if (pathLineCount < 3) {
				os << ", " << to_string(loc.line());
				pathLineCount++;
			} else if (pathLineCount >= 3) {
				/* Skip remaining */
				if (pathLineCount == 3)
					os << "...";

				pathLineCount++;
			}
		}
		os << "\n */\n";

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
				} else if (isa<EnumDecl>(decl)) {
					auto edecl = dyn_cast<EnumDecl>(decl);
					printEnumDecl(os, edecl, printPolicy,
					    astContext, idx);
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
			out << output;
			out << "\n\n";
		}
	}

	return (yield(Unit()));
}

/* The source location must point to a real file for diagnostic reporting */
static bool
canEmitDiag(SourceLocation &loc, ASTContext &ast)
{
	if (loc.isInvalid())
		return (false);

	auto fid = ast.getSourceManager().getFileID(loc);
	if (fid.isInvalid())
		return (false);

	if (ast.getSourceManager().getFileEntryForID(fid) == NULL)
		return (false);

	return (true);
}

template <unsigned N>
static result<Unit>
emitError(const char (&fmt)[N], const Decl *decl, std::string declName,
    ASTContext &ast)
{
	auto &diags = ast.getDiagnostics();
	auto loc = decl->getLocStart();
	auto msgstr = declName + ": " + fmt;

	if (!canEmitDiag(loc, ast)) {
		llvm::errs() << msgstr << "\n";
		return (fail<Unit>(msgstr));
	}

	unsigned diagID = diags.getCustomDiagID(DiagnosticsEngine::Error, fmt);
	diags.Report(loc, diagID) << decl->getSourceRange();

	return (fail<Unit>(msgstr));
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
	if (!decl->hasAttrs())
		return (yield(Unit()));

	for (auto &attr : decl->getAttrs()) {
		switch (attr->getKind()) {
		#define ATTR(X)
		#define PRAMGA_SPELLING_ATTR(_Name) case attr::_Name:
		#include <clang/Basic/AttrList.inc>
		{
			return (emitError("MS #pragma-style attributes "
			    "are unsupported", decl, attr->getSpelling(), ast));
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
		return (emitError("cannot emit declaration for unsupported "
		    "storage class", decl, fnName, ast));
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
		return(emitError("missing function prototype; cannot emit "
		    "declaration", decl, fnName, ast));
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
		auto tsi = pdecl->getTypeSourceInfo();
		auto type = tsi != NULL ? tsi->getType() : pdecl->getType();

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
		return(emitError("unsupported exception spec; cannot emit "
		    "declaration", decl, fnName, ast));
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

static void
printEnumDecl(raw_ostream &os, const EnumDecl *decl,
    const PrintingPolicy &policy, ASTContext &ast, const ASTIndexRef &idx)
{
	auto istr = string(1, '\t');

	/* clang extension; symbol is private to its enclosing module */
	if (decl->isModulePrivate())
		os << "__module_private__ ";

	os << "enum ";

	/** C++ enum scope */
	if (decl->isScopedUsingClassTag())
		os << "class ";
	else if (decl->isScoped())
		os << "struct ";

	os << decl->getName();

	/* Fixed type (C++, ObjC, (C?)) */
	if (decl->isFixed())
		os << " : " << decl->getIntegerType().stream(policy);

	os << " {\n";

	bool wroteEnum = false;
	for (auto ec : decl->enumerators()) {
		SmallString<255>	sbuf;
		auto			initExpr = ec->getInitExpr();

		if (wroteEnum)
			os << ",\n";
		else
			wroteEnum = true;

		os << istr << ec->getName();
		if (initExpr != nullptr) {
			/* TODO: Nested indentation */
			os << " = ";
			initExpr->printPretty(os, nullptr, policy);
		}

		/* Is this constant actually used? */
		if (generateUSRForDecl(ec, sbuf))
			abort();

		if (!idx->hasSymbolUses(sbuf.str())) {
			os << "\t/* not referenced by brcmfmac */";
		}
	}

	if (wroteEnum)
		os << "\n";

	os << "}";
}
