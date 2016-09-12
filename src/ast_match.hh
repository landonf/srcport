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
 * 
 * $FreeBSD$
 */

#ifndef _SRCPORT_AST_MATCH_HH_
#define _SRCPORT_AST_MATCH_HH_

#include <memory>

#include <clang/AST/AST.h>
#include <clang/AST/ASTContext.h>

#include <clang/ASTMatchers/ASTMatchers.h>
#include <clang/ASTMatchers/ASTMatchFinder.h>

#include "project.hh"
#include "symbol_table.hh"

/**
 * AST matching utility methods.
 */
class ASTMatchUtil {
public:
	ASTMatchUtil (const ProjectRef &project, const clang::ASTContext &ast):
	    _project(project), _srcManager(ast.getSourceManager())
	{
	}

	enum loc_type {
		LOC_SOURCE,
		LOC_HOST,
		LOC_MACRO,
		LOC_EXTERNAL,
		LOC_INVALID
	};

	std::string	describe (const clang::SourceLocation &loc);
	void		dump (const clang::SourceLocation &loc);
	void		dumpTree (const clang::SourceLocation &loc, std::string::size_type indent = 0);
	
	bool		locMatches (const clang::SourceLocation &loc, const PathPattern &p);

	loc_type	getLocationType (clang::SourceLocation definedAt);
	bool		isHostRef (clang::SourceLocation usedAt, clang::SourceLocation definedAt);

	const clang::SourceManager &srcManager () { return (_srcManager); }

private:
	ProjectRef _project;
	const clang::SourceManager &_srcManager;
};

/**
 * AST match rule and callback pairs.
 */
class ASTIndexMatch {
private:
	template<typename M> using Matcher = clang::ast_matchers::internal::Matcher<M>;
	using MatchFinder = clang::ast_matchers::MatchFinder;
	using MatchCallbackRef = std::shared_ptr<MatchFinder::MatchCallback>;

	/**
	 * A MatchCallback implementation that wraps an arbitrary function (such as a
	 * lambda).
	 */
	template <typename Fn> class MatchFn : public MatchFinder::MatchCallback {
	public:
		MatchFn (Fn fn) : _fn(fn) { }

		virtual void
		run (const clang::ast_matchers::MatchFinder::MatchResult &Result) override
		{
			_fn(Result);
		}

	private:
		Fn _fn;
	};

public:
	/**
	 * Register a @p matcher and callback function.
	 */
	template<typename M, typename Fn> void
	addMatcher (M &&matcher, Fn &&fn)
	{
		_callbacks.emplace_back(std::make_shared<MatchFn<Fn>>(std::forward<Fn>(fn)));
		_finder.addMatcher(std::forward<M>(matcher), &*_callbacks.back());
	}

	/**
	 * Return the match finder.
	 */
	clang::ast_matchers::MatchFinder &
	getFinder ()
	{
		return (_finder);
	}

private:
	std::vector<MatchCallbackRef>		_callbacks;
	clang::ast_matchers::MatchFinder	_finder;
};

/* Custom AST matchers */
namespace matchers {
	using namespace clang;
	namespace internal {

		template <typename... Ts> using TypeList = clang::ast_matchers::internal::TypeList<Ts...>;
	}

	AST_MATCHER(Stmt, isMacroBodyExpansion)
	{
		const auto &smgr = Finder->getASTContext().getSourceManager();
		return (smgr.isMacroBodyExpansion(Node.getLocStart()));
	}

	AST_MATCHER_P(Expr, isSourceExpr, ProjectRef, project)
	{
		SourceLocation		 usedAt;
		ASTMatchUtil		 m(project, Finder->getASTContext());
		const auto		&smgr = m.srcManager();

		usedAt = Node.getLocStart();

		if (usedAt.isMacroID()) {
			if (smgr.isMacroArgExpansion(usedAt))
				usedAt = smgr.getImmediateSpellingLoc(usedAt);
			else
				usedAt = smgr.getExpansionLoc(usedAt);
		}

		return (m.getLocationType(usedAt) == ASTMatchUtil::LOC_SOURCE);
	}

	AST_MATCHER_P(Decl, isHostDecl, ProjectRef, project)
	{
		ASTMatchUtil	 m(project, Finder->getASTContext());

		auto loc = m.srcManager().getFileLoc(Node.getLocStart());
		if (m.getLocationType(loc) == ASTMatchUtil::LOC_HOST)
			return (true);

		return (false);
	}

	AST_MATCHER_P(QualType, isHostType, ProjectRef, project)
	{
		return (false);
	}

	AST_POLYMORPHIC_MATCHER_P(isHostSymbolReference, void(internal::TypeList<DeclRefExpr, Stmt>), ProjectRef, project)
	{

		if (!isSourceExpr(project).matches(Node, Finder, Builder))
			return (false);

		if (auto expr = dyn_cast<DeclRefExpr>(&Node)) {
			const Decl	*target;

			target = expr->getFoundDecl()->getCanonicalDecl();

			return (isHostDecl(project).matches(
			    *target, Finder, Builder));
		}

		// XXX TODO
		Node.dump();
		abort();

		return (false);
	}

} /* namespace matchers */


#endif /* _SRCPORT_AST_MATCH_HH_ */
