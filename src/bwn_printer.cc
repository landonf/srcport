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

#include <clang/AST/AST.h>
#include <clang/Index/USRGeneration.h>

#include "unit_type.hpp"

#include "compiler.hh"
#include "ast_index.hh"
#include "project.hh"

#include "bwn_printer.hh"

using namespace std;

using namespace llvm;

using namespace clang;
using namespace clang::tooling;
using namespace clang::index;

using namespace pl;
using namespace symtab;

class StyleConfig;

enum CodeStyle {
	FREEBSD_FN_PROTO,	/**< function prototype */
	FREEBSD_FN_DEF,		/**< function definition */
	FREEBSD_FN_PTR,		/**< function pointer */
	FREEBSD_FN_PARAM,	/**< function parameter */
	FREEBSD_VAR_PROTO,	/**< variable prototype */
	FREEBSD_VAR_DEF,	/**< variable definition */
} ;

enum ParamStyle {
	PARAM_STYLE_OMIT_NAMES,	/**< omit parameter names. */
	PARAM_STYLE_DEFAULT,	/**< include parameter names; generate stand-in names if missing. */
};

static string		bwn_bus_op_name(const std::string &fnName);
static string		bwn_bus_impl_name(const std::string &fnName,
			    const std::string &provider);

static result<string>	bwn_bus_ops_struct(const ASTIndexRef &idx,
			    vector<SymbolRef> &syms);

static result<string>	bwn_bus_ops_impl(const std::string &name,
			    const std::string &provider, const ASTIndexRef &idx,
			    vector<SymbolRef> &syms);

static string		bwn_bus_op_redirects(const ASTIndexRef &idx,
			    vector<SymbolRef> &syms);

static void		printMacroArgs(raw_ostream &os, const MacroInfo &info);
static void		printMacroDefinition(raw_ostream &os, const StrRef name,
			    const MacroInfo &info, Preprocessor &cpp);
static void		printValueType(raw_ostream &os, QualType &type,
			    const string &name, const PrintingPolicy &policy,
			    const StyleConfig &style);
static result<Unit>	printAttributes(raw_ostream &os, const Decl *decl,
			    const PrintingPolicy &policy, ASTContext &ast);
static result<Unit>	printFunctionDecl(raw_ostream &os,
			    const FunctionDecl *decl,
			    const PrintingPolicy &policy, ASTContext &ast,
			    const StyleConfig &style);
static void		printEnumDecl(raw_ostream &os, const EnumDecl *decl,
			    const PrintingPolicy &policy, ASTContext &ast,
			    const ASTIndexRef &idx);

static bool		has_prefix (const string &str, const string &prefix);


class StyleConfig {
PL_RECORD_FIELDS(StyleConfig,
	(ftl::maybe<std::string>,	nameOverride),
	(CodeStyle,			symbolStyle),
	(ParamStyle,			paramStyle),
	(bool,				isStatic),
	(bool,				isInline)
)

public:
	/* Default configuration */
	StyleConfig (): StyleConfig(ftl::Nothing(), FREEBSD_FN_PROTO,
	    PARAM_STYLE_DEFAULT, false, false)
	{}

	/**
	 * Return a copy of this style configuration with the given name
	 * override set.
	 */
	StyleConfig nameOverride (const std::string &name) {
		return (nameOverride(ftl::just(name)));
	}

	StyleConfig nameOverride (std::string &&name) {
		return (nameOverride(ftl::just(std::move(name))));
	}
	
	/**
	 * Return a style configuration containing only the values that
	 * should be inherited for use with a function's return value.
	 */
	StyleConfig deriveRetValConfig () const {
		return (StyleConfig(ftl::Nothing(), _symbolStyle,
		    _paramStyle, false, false));
	}

	/**
	 * Return a style configuration containing only the values that
	 * should be inherited for use with a function parameter.
	 */
	StyleConfig deriveParamConfig () const {
		return (StyleConfig(ftl::Nothing(), FREEBSD_FN_PARAM,
		    _paramStyle, false, false));
	}
};

/**
 * Emit our bwn/siba compatibility header. In some future iteration, we'll
 * replace this with a more general-purpose API.
 */
result<pl::Unit>
srcport::emit_bwn_stubs(const ASTIndexRef &idx, llvm::raw_ostream &out)
{
	/* Sort symbols by file location */
	auto syms = vector<SymbolRef>(idx->getSymbols().begin(), idx->getSymbols().end());
	std::sort(syms.begin(), syms.end(), [&](const SymbolRef &lhs, const SymbolRef &rhs){
		return (lhs->location() < rhs->location());
	});

	/* Trims trailing newlines */
	auto rtrim = [](std::string &s) {
		s.erase(s.find_last_not_of("\r\n\t")+1);
	};

	/*
	 * Generate if_bwn_siba.h definitions.
	 */

	out << "/*\n * if_bwn_siba.h\n */\n\n";

	/* siba-defined constants */
	for (const auto &sym : syms) {
		std::string			output;
		llvm::raw_string_ostream	os(output);
		auto				USR = sym->USR();

		/* Don't bother printing individual enum constants */
		if (sym->cursor().as<EnumConstantDecl>() != nullptr)
			continue;

		auto &astUnit = *sym->cursor().unit();
		auto &astContext = astUnit.getASTContext();
		auto &langOpts = astUnit.getLangOpts();
		auto &cpp = astUnit.getPreprocessor();

		PrintingPolicy printPolicy(langOpts);
		sym->cursor().node().matchE(
			[&](const Decl *decl) {
				if (isa<EnumDecl>(decl)) {
					auto edecl = dyn_cast<EnumDecl>(decl);
					printEnumDecl(os, edecl, printPolicy,
					    astContext, idx);
					rtrim(os.str());
					os << ";";
				}
			},
			[&](const Stmt *stmt) {
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

	/* struct bwn_siba_bus_ops */
	auto busOps = bwn_bus_ops_struct(idx, syms) >>= [&](const string &str) {
		auto trimmed = str;

		rtrim(trimmed);
		out << trimmed << ";\n";

		return yield(trimmed);
	};

	if (!busOps.is<ftl::Right<string>>())
		return (fail<Unit>("generating bus ops failed"));

	/* siba API redirection definitions */
	out << bwn_bus_op_redirects(idx, syms);

	/*
	 * Generate if_bwn_siba.c implementations.
	 */
	out << "\n\n/*\n * if_bwn_siba.c\n */\n\n";

	auto busImplNative = bwn_bus_ops_impl("siba", "siba", idx, syms) >>= [&](const string &str) {
		auto trimmed = str;

		rtrim(trimmed);
		out << trimmed << ";\n";

		return yield(trimmed);
	};

	if (!busImplNative.is<ftl::Right<string>>())
		return (fail<Unit>("generating bus impl failed"));

	/*
	 * Generate if_bwn_siba_compat.c implementations.
	 */
	out << "\n\n/*\n * if_bwn_siba_compat.c\n */\n\n";

	/* Emit our bhnd_compat_* function definitions */
	for (const auto &sym : syms) {
		std::string			output;
		llvm::raw_string_ostream	os(output);
		auto				USR = sym->USR();
		auto				canonicalSym = idx->getCanonicalSymbol(sym);

		auto &astUnit = *sym->cursor().unit();
		auto &astContext = astUnit.getASTContext();
		auto &langOpts = astUnit.getLangOpts();

		PrintingPolicy printPolicy(langOpts);
		auto cfg = StyleConfig().symbolStyle(FREEBSD_FN_DEF).isStatic(
		    true);
		canonicalSym->cursor().node().matchE(
			[&](const Decl *decl) {
				auto func = dyn_cast<FunctionDecl>(decl);
				if (func == nullptr)
					return;

				auto fname = func->getName();
				auto implName = bwn_bus_impl_name(fname,
				    "bhnd_compat");
				auto funcCfg = cfg.nameOverride(implName);

				printFunctionDecl(os, func, printPolicy,
				    astContext, funcCfg);

				rtrim(os.str());
				os << "\n{\n\tpanic(\"" << fname <<
				    "() unimplemented\");\n}";
			},
			[&](const Stmt *stmt) { },
			[&](MacroRef macro) { }
		);

		/* Flush and emit */
		os.str();
		if (output.size() > 0) {
			out << output;
			out << "\n\n";
		}
	}

	auto busImplCompat = bwn_bus_ops_impl("bhnd", "bhnd_compat", idx, syms) >>= [&](const string &str) {
		auto trimmed = str;

		rtrim(trimmed);
		out << trimmed << ";\n";

		return yield(trimmed);
	};

	if (!busImplCompat.is<ftl::Right<string>>())
		return (fail<Unit>("generating bus impl failed"));


	return (yield(Unit()));
}

static string
bwn_bus_op_name(const std::string &fnName)
{
	if (has_prefix(fnName, "siba_"))
		return (fnName.substr(strlen("siba_")));
	else
		return (fnName);
}

static string
bwn_bus_impl_name(const std::string &fnName, const std::string &provider)
{
	return (provider + "_" + bwn_bus_op_name(fnName));
}

static result<string>
bwn_bus_ops_struct(const ASTIndexRef &idx, vector<SymbolRef> &syms)
{
	std::string			output;
	llvm::raw_string_ostream	os(output);
	auto				istr = string(1, '\t');
	StyleConfig			config;

	os << "struct bwn_bus_ops {\n";

	/* Emit function pointer members */
	config = config.symbolStyle(FREEBSD_FN_PTR).paramStyle(
	    PARAM_STYLE_OMIT_NAMES);
	for (const auto &sym : syms) {
		const FunctionDecl	*func;

		/* We're only interested in functions */
		if ((func = sym->cursor().as<FunctionDecl>()) == nullptr)
			continue;

		/* Determine member name */
		auto symConfig = config.nameOverride(
		    bwn_bus_op_name(*sym->name()));

		/* Emit function pointer decl */
		auto &astUnit = *sym->cursor().unit();
		auto &astContext = astUnit.getASTContext();
		auto &langOpts = astUnit.getLangOpts();

		PrintingPolicy printPolicy(langOpts);
		printPolicy.PolishForDeclaration = 1;

		os << istr;
		auto ret = printFunctionDecl(os, func, printPolicy, astContext,
		    symConfig);

		if (!ret.is<ftl::Right<Unit>>())
			return (fail<string>("generating bus ops failed"));

		os << ";" << "\n";
	}

	os << "}";

	os.str();
	return yield(std::move(output));
}

static result<string>
bwn_bus_ops_impl(const std::string &name, const std::string &provider,
     const ASTIndexRef &idx, vector<SymbolRef> &syms)
{
	std::string			output;
	llvm::raw_string_ostream	os(output);
	auto				istr = string(1, '\t');

	os << "const struct bwn_bus_ops bwn_" << name << "_bus_ops" << " = {\n";

	/* Emit function pointer members */
	for (const auto &sym : syms) {
		const FunctionDecl *func;

		/* We're only interested in functions */
		if ((func = sym->cursor().as<FunctionDecl>()) == nullptr)
			continue;

		/* Determine member name */
		auto opName = bwn_bus_op_name(*sym->name());
		string impName = bwn_bus_impl_name(*sym->name(), provider);

		os << istr << "." << opName << "\t\t= " << impName << ",\n";
	}

	os << "}";

	os.str();
	return yield(std::move(output));
}

static string
bwn_bus_op_redirects(const ASTIndexRef &idx, vector<SymbolRef> &syms)
{
	std::string			output;
	llvm::raw_string_ostream	os(output);

	for (const auto &sym : syms) {
		const FunctionDecl *func;

		/* We're only interested in functions */
		if ((func = sym->cursor().as<FunctionDecl>()) == nullptr)
			continue;

		/* Determine member name */
		auto opName = bwn_bus_op_name(*sym->name());
		
		os << "#define\t" << *sym->name() << "(_dev";

		/* Determine argument names and emit the macro arg list */
		vector<string> args;
		size_t nargs = func->getNumParams();
		for (size_t i = 1; i < nargs; i++) {
			auto pdecl = func->getParamDecl(i);
			auto name = pdecl->getName();
			if (name.size() == 0)
				name = "_arg" + to_string(i);

			args.push_back(name);
			os << ", " << name;
		}
		if (func->isVariadic())
			os << ", ...";

		os << ")" << "\t\\\n";

		/* Emit the bus op call */
		os << "\tBWN_BUS_OPS(_dev)->" << opName << "(_dev";
		for (const auto &arg : args)
			os << ", " << arg;

		if (func->isVariadic())
			os << ", ##__VA_ARGS__";

		os << ")" << "\n";
	}

	os.str();
	return (output);
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
    const PrintingPolicy &policy, const StyleConfig &style)
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
	switch (style.symbolStyle()) {
	case FREEBSD_FN_PROTO:
	case FREEBSD_FN_PTR:
	case FREEBSD_VAR_DEF:
	case FREEBSD_VAR_PROTO:
		os << "\t\t" << ptrStr;
		break;
	case FREEBSD_FN_DEF:
		if (ptrStr.size() > 0)
			os << " " << ptrStr;
		os << "\n";
		break;
	case FREEBSD_FN_PARAM:
		if (ptrStr.size() > 0)
			os << " " << ptrStr;
		else
			os << " ";
		break;
	}

	/* Emit name */
	os << style.nameOverride().match(
		[](const std::string &override)		{ return (override); },
		[&](ftl::Nothing)			{ return (name); }
	);
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

/**
 * Print a function decl, optionally formatting as a function pointer using
 * @p fptrName.
 */
static result<Unit>
printFunctionDecl(raw_ostream &os, const FunctionDecl *decl,
    const PrintingPolicy &policy, ASTContext &ast,
    const StyleConfig &style)
{
	const CXXConstructorDecl	*cxxConstrDecl;
	const CXXConversionDecl		*cxxConvDecl;
	const ParenType			*parenType;
	const FunctionProtoType		*fnType;
	string				 symbolName;
	auto				&diags = ast.getDiagnostics();
	size_t				 nargs, nexceptions;
	bool				 isMethod;

	symbolName = decl->getNameInfo().getAsString();
	const std::string &fnName = style.nameOverride().match(
		[](const std::string &name)	{ return (name); },
		[&](ftl::Nothing)		{ return (symbolName); }
	);
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
		if (isMethod && !style.isStatic())
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
		    symbolName));
	}

	/* Apply style override */
	if (style.isStatic())
		os << "static ";

	/* Inline functions. (Intentionally ignored for non-methods. See
	 * SC_Static above) */
	if ((isMethod && decl->isInlineSpecified()) || style.isInline())
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

	/* Function pointer output style */
	if (style.symbolStyle() == FREEBSD_FN_PTR)
		parenName = "(*" + fnName + ")";

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
		printValueType(os, retType, parenName, policy,
		    style.deriveRetValConfig());
	} else {
		os << parenName;
	}

	/* Emit arguments */
	auto pstyle = style.deriveParamConfig();
	os << "(";
	for (size_t i = 0; i < nargs; i++) {
		auto pdecl = decl->getParamDecl(i);
		auto type = pdecl->getTypeSourceInfo()->getType();
		auto pname = pdecl->getNameAsString();

		if (i > 0)
			os << ", ";

		switch (pstyle.paramStyle()) {
		case PARAM_STYLE_OMIT_NAMES:
			pname = "";
			break;
		case PARAM_STYLE_DEFAULT:
			if (pname.size() == 0)
				pname = "arg" + to_string(i);
			break;
		}
	
		printValueType(os, type, pname, policy, pstyle);
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

	if (decl->getName().size() > 0)
		os << decl->getName() << " ";

	/* Fixed type (C++, ObjC, (C?)) */
	if (decl->isFixed())
		os << " : " << decl->getIntegerType().stream(policy) << " ";

	os << "{\n";

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
			os << "\t/* unused */";
		}
	}

	if (wroteEnum)
		os << "\n";

	os << "}";
}

/* String prefix comparison */
static bool has_prefix (const string &str, const string &prefix) {
	if (prefix.size() > str.size())
		return (false);
	return (str.compare(0, prefix.size(), prefix) == 0);
}
