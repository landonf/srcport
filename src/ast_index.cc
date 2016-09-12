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

#include <string>

#include "ast_index.hh"
#include "ast_match.hh"

using namespace std;

using namespace clang;
using namespace clang::tooling;
using namespace clang::ast_matchers;

using namespace symtab;
using namespace matchers;

static std::unordered_set<string> seen;

/**
 * Build the AST index
 */
void ASTIndexBuilder::build()
{
	ASTIndexMatch m;

	/* Find direct named symbol references */
	auto findDeclRefs = declRefExpr(
	    isHostSymbolReference(_project)
	).bind("ref");

	m.addMatcher(findDeclRefs, [&](const MatchFinder::MatchResult &m) {
		ASTMatchUtil	 mu(_project, *m.Context);

		if (const auto decl = m.Nodes.getNodeAs<DeclRefExpr>("ref")) {
			// TODO
			const auto &name = decl->getFoundDecl()->getName();

			if (seen.count(name))
				return;
			else
				seen.emplace(name);

			llvm::outs() << "found: '" << name << "'\n";
		}
	});

	/* Find type references, including implicit type references by way
	 * of enum constant usage. */
	auto findDeclTypeRefs = expr(allOf(
	    isSourceExpr(_project),
	    hasType(qualType(
		hasDeclaration(namedDecl(isHostDecl(_project)).bind("type"))))
	)).bind("expr");
	

	m.addMatcher(findDeclTypeRefs, [&](const MatchFinder::MatchResult &m) {
		ASTMatchUtil	 mu(_project, *m.Context);

		if (const auto e = m.Nodes.getNodeAs<Expr>("expr")) {
			e->dump();
		}

		if (const auto t = m.Nodes.getNodeAs<NamedDecl>("type")) {
			// TODO
			const auto &name = t->getName();

			if (seen.count(name))
				return;
			else
				seen.emplace(name);

			llvm::outs() << "type-ref: '" << name << "'\n";
			t->dump();
		}
	});

#if 0
	auto findMacroRefs = stmt(
            allOf(unless(declRefExpr()), isMacroBodyExpansion(), isNonPortableReference(_project))
	).bind("macro");
	m.addMatcher(findMacroRefs, [&](const MatchFinder::MatchResult &m) {
		ASTMatchUtil	 mu(_project, *m.Context);

		if (const auto decl = m.Nodes.getNodeAs<Stmt>("macro")) {
			//auto fileLoc = mu.srcManager().getFileLoc(decl->getLocStart());
			
			decl->dump();
		}
	});
#endif

	_tool->run(newFrontendActionFactory(&m.getFinder()).get());	
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

	return (idx);
}
