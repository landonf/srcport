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

#include "ast_index.hh"
#include "ast_match.hh"

using namespace std;

using namespace clang;
using namespace clang::tooling;
using namespace clang::ast_matchers;
using namespace clang::index;

using namespace symtab;
using namespace matchers;

static std::unordered_set<string> seen;

/**
 * Cache (or return a cached) declaration USR.
 */
StrRef ASTIndexUtil::cacheUSR(const Decl *decl)
{
	SmallString<255>	sbuf;
	
	/* Generate USR string */
	if (generateUSRForDecl(decl, sbuf))
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
 * Lambda callback support for SourceFileCallbacks.
 */
template <typename BeginFn, typename EndFn> 
class LambdaSourceFileCallbacks : public SourceFileCallbacks {
public:
	LambdaSourceFileCallbacks (BeginFn begin, EndFn end):
	    _begin(begin), _end(end)
	{}

	virtual bool
	handleBeginSource(CompilerInstance &CI, StringRef Filename) override
	{
		return (_begin(CI, Filename));
	}

	virtual void
	handleEndSource() override
	{
		return (_end());
	}

private:
	BeginFn	_begin;
	EndFn	_end;
};

template <typename BeginFn, typename EndFn> std::unique_ptr<SourceFileCallbacks>
lambdaSourceFileCallbacks(BeginFn &&begin, EndFn &&end)
{
	return (unique_ptr<SourceFileCallbacks>(
	    new LambdaSourceFileCallbacks<BeginFn, EndFn>(
		    std::forward<BeginFn>(begin), std::forward<EndFn>(end))
	));
}

/**
 * Build the AST index
 */
void ASTIndexBuilder::build()
{
	ASTIndexMatch		 m;
	const CompilerInstance	*cc = nullptr;

	auto registerSymbolUse = [&](const MatchFinder::MatchResult &m) {
		ASTIndexUtil		 iu(_symtab, *m.Context);
		ASTMatchUtil		 mu(_project, *m.Context);
		const Expr		*src;
		const NamedDecl		*target;

		if (!(src = m.Nodes.getNodeAs<Expr>("source")))
			return;
		
		if (!(target = m.Nodes.getNodeAs<NamedDecl>("target")))
			return;

		/* Register or fetch existing symbol */
		auto USR = iu.cacheUSR(target);

		auto symbol = _symtab->lookupUSR(*USR).match(
			[](SymbolRef &s) { return (s); },
			[&](ftl::otherwise) {
				auto s = make_shared<Symbol>(
				    make_shared<string>(target->getName()),
				    iu.generateLocation(target->getLocation()),
				    USR
				);
				_symtab->addSymbol(s);
				return (s);
			}
		);

		/* Register symbol use */
		auto symbolUse = make_shared<SymbolUse>(
			symbol,
			iu.generateLocation(src->getLocStart()),
			USR
		);

		_symtab->addSymbolUse(symbolUse);
	};

	/* Find direct named symbol references */
	m.addMatcher(declRefExpr(allOf(
	    isHostSymbolReference(_project),
	    hasDeclaration(namedDecl().bind("target"))
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
	    isImmediateMacroBodyExpansion(),
	    isHostSymbolReference(_project))
	).bind("macro");
	m.addMatcher(findMacroRefs, [&](const MatchFinder::MatchResult &m) {
		ASTMatchUtil	 mu(_project, *m.Context);
		const Stmt	*stmt;

		if (!(stmt = m.Nodes.getNodeAs<Stmt>("macro")))
			return;

		auto name = cc->getPreprocessor().getImmediateMacroName(stmt->getLocStart());
		
		if (seen.count(name))
			return;
		else
			seen.emplace(name);

		llvm::outs() << "macro-ref: " << name << "\n";
	});

	/* Keep track of the compiler instance */
	auto callbacks = lambdaSourceFileCallbacks(
		[&](CompilerInstance &CI, StringRef Filename) {
			llvm::outs() << "Processing " << Filename << "\n";
			cc = &CI;
			return (true);
		},
		[&]() {
			cc = nullptr;
		}
	);

	_tool->run(newFrontendActionFactory(&m.getFinder(), callbacks.get()).get());	
}

/**
 * Build and return an index for @p project using @p tool.
 * 
 * @param project Project configuration.
 * @param tool Tool instance initialized for use with @p project.
 */
std::shared_ptr<ASTIndex>
ASTIndex::Index(ProjectRef &project, ASTIndex::ClangToolRef &tool)
{
	auto idx = make_shared<ASTIndex>(project, tool, AllocKey{});

	ASTIndexBuilder builder(idx->_tool, idx->_symtab);
	builder.build();

	// TODO
	for (const auto &sym : idx->_symtab->getSymbols())
		llvm::outs() << "found: " << *sym->name() << "\n";

	return (idx);
}
