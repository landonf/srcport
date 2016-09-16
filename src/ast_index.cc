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

#include <string>
#include <thread>

#include "work_queue.hh"

#include "ast_index.hh"
#include "ast_match.hh"

using namespace std;

using namespace clang;
using namespace clang::tooling;
using namespace clang::ast_matchers;
using namespace clang::index;

using namespace symtab;
using namespace matchers;

/* Support for locking of the output stream */
static mutex sync_errs_lock;
static void
sync_errs(const std::string &msg) {
	unique_lock<mutex> lk(sync_errs_lock);
	llvm::errs() << msg  << "\n";
	llvm::errs().flush();
}

/**
 * Cache (or return a cached) declaration USR.
 */
StrRef ASTIndexUtil::cacheUSR(const Decl &decl)
{
	SmallString<255>	sbuf;
	
	/* Generate USR string */
	if (generateUSRForDecl(&decl, sbuf))
		abort();

	return (_symtab->getUSR(sbuf.str()));	
}


/**
 * Cache (or return a cached) macro definition USR.
 */
StrRef ASTIndexUtil::cacheUSR(const MacroDefinitionRecord &macro)
{
	SmallString<255>	sbuf;
	
	/* Generate USR string */
	if (generateUSRForMacro(&macro, _ast.getSourceManager(), sbuf))
		abort();

	return (_symtab->getUSR(sbuf.str()));
}

/**
 * Generate a symbol table location record for the given source location.
 */
symtab::Location
ASTIndexUtil::generateLocation (const clang::SourceLocation &loc)
{
	unsigned line, column;

	line = 0;
	column = 0;

	assert(loc.isValid());

	const auto &smgr = _ast.getSourceManager();
	auto fileLoc = smgr.getFileLoc(loc);
	auto locInfo = smgr.getDecomposedLoc(fileLoc);
	auto fid = locInfo.first;
	auto fileOffset = locInfo.second;

	assert(fid.isValid());

	auto file = smgr.getFileEntryForID(fid);
	assert (file != NULL);

	line = smgr.getLineNumber(fid, fileOffset);
	column = smgr.getColumnNumber(fid, fileOffset);

	auto path = _symtab->getPath(file->getName());
	return (Location(path, line, column));
}

/**
 * Register definition for @p symbol with the symbol table and return the new
 * registration, or return the existing registration.
 */
SymbolRef
ASTIndexUtil::registerDefinition(const clang::NamedDecl &decl)
{
	/* Register or fetch existing definition */
	auto USR = cacheUSR(decl);
	return (_symtab->definition(*USR).match(
		[](SymbolRef &s) { return (s); },
		[&](ftl::otherwise) {
			auto s = make_shared<Symbol>(
				make_shared<string>(decl.getName()),
				generateLocation(decl.getLocation()),
				Cursor(&decl, &_astUnit),
				USR
			);
			return (_symtab->addDefinition(s));
		}
	));
}

/**
 * Register @p symbol with the symbol table and return the new registration,
 * or return the existing registration.
 */
SymbolRef
ASTIndexUtil::registerSymbol(const clang::NamedDecl &decl)
{
	/* Register or fetch existing symbol */
	auto USR = cacheUSR(decl);

	return (_symtab->lookupUSR(*USR).match(
		[](SymbolRef &s) { return (s); },
		[&](ftl::otherwise) {
			auto s = make_shared<Symbol>(
				make_shared<string>(decl.getName()),
				generateLocation(decl.getLocation()),
				Cursor(&decl, &_astUnit),
				USR
			);
			return (_symtab->addSymbol(s));
		}
	));
}

SymbolRef
ASTIndexUtil::registerSymbol(const Stmt *stmt, const IdentifierInfo &ident,
    const MacroDefinition &macro, Preprocessor &cpp)
{
	auto minfo = macro.getMacroInfo();
	auto mrange = SourceRange(minfo->getDefinitionLoc(), minfo->getDefinitionEndLoc());
	auto mrec = MacroDefinitionRecord(&ident, mrange);

	/* Register or fetch existing symbol */
	auto USR = cacheUSR(mrec);

	return (_symtab->lookupUSR(*USR).match(
		[](SymbolRef &s) { return (s); },
		[&](ftl::otherwise) {
			auto s = make_shared<Symbol>(
				make_shared<string>(ident.getName()),
				generateLocation(minfo->getDefinitionLoc()),
				Cursor(MacroRef(stmt), &_astUnit),
				USR
			);
			return (_symtab->addSymbol(s));
		}
	));
}

/**
 * Register @p symbolUse for @p symbol to the symbol table.
 */
SymbolUseRef
ASTIndexUtil::registerSymbolUse(const Stmt *symbolUse, const SymbolRef &symbol)
{
	auto loc = symbolUse->getLocStart();
	if (loc.isMacroID())
		loc = _ast.getSourceManager().getExpansionLoc(loc);
	
	auto s = make_shared<SymbolUse>(
		symbol,
		Cursor(symbolUse, &_astUnit),
		generateLocation(loc)
	);

	return (_symtab->addSymbolUse(s));
}

/**
 * Scan a single ASTUnit for symbol references.
 */
void
ASTIndexBuilder::indexReferences(ASTUnit *au)
{
	ASTIndexMatch m;

	auto registerSymbolUse = [&](const MatchFinder::MatchResult &m) {
		ASTIndexUtil		 iu(_symtab, *au);
		ASTMatchUtil		 mu(_project, *m.Context);
		const Expr		*src;
		const NamedDecl		*target;
		const EnumDecl		*enumParent;

		if (!(src = m.Nodes.getNodeAs<Expr>("source")))
			return;
		
		if (!(target = m.Nodes.getNodeAs<NamedDecl>("target")))
			return;

		/* Register symbol use */
		iu.registerSymbolUse(src, iu.registerSymbol(*target));
		
		/* If we're referencing an enum constant, also include the
		 * enumeration declaration */
		enumParent = m.Nodes.getNodeAs<EnumDecl>("target-parent");
		if (enumParent) {
			iu.registerSymbolUse(src,
			    iu.registerSymbol(*enumParent));
		}
	};

	/* Find direct named symbol references */
	m.addMatcher(declRefExpr(allOf(
	    isHostSymbolReference(_project),
	    hasDeclaration(namedDecl(
		anyOf(
			hasParent(enumDecl().bind("target-parent")),
			anything()
		)
	    ).bind("target"))
	)).bind("source"), registerSymbolUse);

	/* Find type references, including implicit type references by way
	 * of enum constant usage. */
	m.addMatcher(expr(allOf(
	    isSourceExpr(_project),
	    hasType(qualType(
		hasDeclaration(namedDecl(isHostDecl(_project)).bind("target"))))
	)).bind("source"), registerSymbolUse);

	/* Find all macro expansions */
	auto findMacroRefs = stmt(allOf(
	    anyOf(isImmediateMacroBodyExpansion(), isMacroArgExpansion()),
	    isHostSymbolReference(_project)
	)).bind("macro");
	m.addMatcher(findMacroRefs, [&](const MatchFinder::MatchResult &m) {
		ASTIndexUtil	 iu(_symtab, *au);
		ASTMatchUtil	 mu(_project, *m.Context);
		auto		&cpp = au->getPreprocessor();
		auto		&smgr = mu.srcManager();
		SourceLocation	 loc;
		const Stmt	*stmt;

		if (!(stmt = m.Nodes.getNodeAs<Stmt>("macro")))
			return;

		/* Extract macro info */
		loc = stmt->getLocStart();
		if (smgr.isMacroArgExpansion(loc))
			loc = smgr.getImmediateSpellingLoc(loc);

		if (!loc.isMacroID())
			return;

		auto name = cpp.getImmediateMacroName(loc);
		auto *ident = cpp.getIdentifierInfo(name);
		MacroDefinition mdef = cpp.getMacroDefinition(ident);

		MacroInfo *info = mdef.getMacroInfo();
		if (info == nullptr)
			return;

		if (mu.getLocationType(info->getDefinitionLoc()) != ASTMatchUtil::LOC_HOST)
			return;
		
		/* Register symbol use */
		iu.registerSymbolUse(stmt, iu.registerSymbol(stmt, *ident, mdef,
		    cpp));
	});

	/* Perform scan */
	auto &finder = m.getFinder();
	finder.matchAST(au->getASTContext());
}

/**
 * Scan a single ASTUnit for symbol definitions.
 */
void
ASTIndexBuilder::indexDefinitions(ASTUnit *au)
{
	ASTIndexUtil	iu(_symtab, *au);
	ASTIndexMatch	m;

	/* Register definitions for all referenced functions */
	auto defnRule = functionDecl(allOf(
		isDefinition(),
		isHostDecl(_project),
		isSymbolUsed(_symtab)
	)).bind("defn");

	m.addMatcher(defnRule, [&](const MatchFinder::MatchResult &m) {
		const FunctionDecl *defn;
		if (!(defn = m.Nodes.getNodeAs<FunctionDecl>("defn")))
			return;

		iu.registerDefinition(*defn);

		unique_lock<mutex> lk(sync_errs_lock);
	});

	/* TODO: Add non-function definitions? */

	/* Perform scan */
	auto &finder = m.getFinder();
	finder.matchAST(au->getASTContext());
}

/**
 * Build the AST index
 */
symtab::SymbolTableRef
ASTIndexBuilder::build()
{
	WorkQueue	workQueue;
	mutex		msgLock;

	/* Locate all in-use symbols by scanning non-host sources first */
	sync_errs("Indexing symbol references...");
	for (const auto &astUnit : _cc->ASTUnits()) {
		auto file = astUnit->getMainFileName();

		if (_project->hostPaths().match(file))
			continue;
 
		workQueue.push_back([&] {
			sync_errs("  Indexing " +
			    astUnit->getMainFileName().str());
			indexReferences(&*astUnit);
		});
	}

	/* Wait for completion */
	workQueue.wait();
	sync_errs("\n");

	/* Now process any host sources, merging enhanced type information
	 * into the symbol table. */
	sync_errs("Indexing symbol definitions...");
	for (const auto &astUnit : _cc->ASTUnits()) {
		auto file = astUnit->getMainFileName();

		if (!_project->hostPaths().match(file))
			continue;

		workQueue.push_back([&] {
			sync_errs("  Indexing " +
			    astUnit->getMainFileName().str());
			indexDefinitions(&*astUnit);
		});
	}

	/* Wait for completion */
	workQueue.wait();

	return (_symtab);
}

/**
 * Build and return an index for @p project using @p cc.
 * 
 * @param project Project configuration.
 * @param cc Compiler instance initialized for use with @p project.
 */
result<ASTIndexRef>
ASTIndex::Build(const ProjectRef &project, const CompilerRef &cc)
{
	auto symtab = ASTIndexBuilder(cc, project).build();
	auto idx = make_shared<ASTIndex>(cc, symtab, AllocKey{});

	return (yield(idx));
}

/**
 * Return the "canonical" symbol instance for @p symbol. If a definition
 * for @p symbol is available, it will be returned; otherwise, the
 * original symbol value will be provided.
 */
const symtab::SymbolRef
ASTIndex::getCanonicalSymbol(const symtab::SymbolRef &symbol)
{
	return (_symtab->definition(*symbol->USR()).match(
		[](const SymbolRef &defn)	{ return (defn); },
		[&](ftl::Nothing)		{ return (symbol); }
	));
}

/**
 * Return all referenced symbols.
 */
const symtab::SymbolSet &
ASTIndex::getSymbols()
{
	return (_symtab->getSymbols());
}

/**
 * Return all symbol references.
 */
const symtab::SymbolUseSet &
ASTIndex::getSymbolUses()
{
	return (_symtab->getSymbolUses());
}

/**
 * Return all symbol references for a symbol with @p USR.
 */
SymbolUseSet ASTIndex::getSymbolUses(const string &USR) 
{
	return (_symtab->usage(USR));
}

/**
 * Return true if any symbol references exist for a symbol with @p USR.
 */
bool
ASTIndex::hasSymbolUses(const string &USR)
{
	return (_symtab->hasUsage(USR));
}
