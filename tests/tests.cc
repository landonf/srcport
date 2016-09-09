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

#define CATCH_CONFIG_MAIN
#include <catch.hpp>


/* XXX:
 * Test linking breaks due to missing symbols if we do not
 * include these symbol references.
 * 
 * libclangASTMatchers.a(ASTMatchFinder.cpp.o):(.data.rel.ro._ZTVN5clang12ast_matchers8internal12_GLOBAL__N_116MatchASTConsumerE+0x28): undefined reference to `clang::ASTConsumer::HandleTopLevelDecl(clang::DeclGroupRef)'
 * libclangASTMatchers.a(ASTMatchFinder.cpp.o):(.data.rel.ro._ZTVN5clang12ast_matchers8internal12_GLOBAL__N_116MatchASTConsumerE+0x38): undefined reference to `clang::ASTConsumer::HandleInterestingDecl(clang::DeclGroupRef)'
 * libclangASTMatchers.a(ASTMatchFinder.cpp.o):(.data.rel.ro._ZTVN5clang12ast_matchers8internal12_GLOBAL__N_116MatchASTConsumerE+0x60): undefined reference to `clang::ASTConsumer::HandleTopLevelDeclInObjCContainer(clang::DeclGroupRef)'
 * libclangASTMatchers.a(ASTMatchFinder.cpp.o):(.data.rel.ro._ZTVN5clang12ast_matchers8internal12_GLOBAL__N_116MatchASTConsumerE+0x68): undefined reference to `clang::ASTConsumer::HandleImplicitImportDecl(clang::ImportDecl*)'
 * 
 * Presumably they prevent stripping (or trigger generation) of ASTConsumer common symbols?
 */
#include "clang/Tooling/Tooling.h"
#include "clang/Tooling/CommonOptionsParser.h"

#include "llvm/Support/CommandLine.h"

using namespace clang::tooling;
using namespace llvm;

static cl::extrahelp CommonHelp(CommonOptionsParser::HelpMessage);
