.PATH: ${.CURDIR}/src

SRCS+=		ast_index.cc \
		ast_match.cc \
		cvisitor.cc \
		cvisitor_state.cc \
		paths.cc \
		project.cc \
		symbol_table.cc

CFLAGS_WARN?=	-Wall -Werror -Wextra \
		-Wno-unused-parameter \
		-Wno-gnu-zero-variadic-macro-arguments \
		-Wno-covered-switch-default

LLVM_VER?=	38
LLVM_CFG=	llvm-config${LLVM_VER}
LLVM_CXXFLAGS!=	${LLVM_CFG} --cxxflags | sed s/-fno-exceptions\ //g
LLVM_CFLAGS!=	${LLVM_CFG} --cflags
LLVM_LDFLAGS!=	${LLVM_CFG} --ldflags
LLVM_LIBS!=	${LLVM_CFG} --libs --system-libs

FTL_CFLAGS=	-I${.CURDIR}/dependencies/ftl
PLCPP_CFLAGS=	-I${.CURDIR}/dependencies/plstdcpp

CXXFLAGS+=	${LLVM_CXXFLAGS} \
		${CFLAGS_WARN}

CFLAGS+=	-pthread \
		${LLVM_CFLAGS} \
		${FTL_CFLAGS} \
		${PLCPP_CFLAGS} \
		${CFLAGS_WARN}

LDFLAGS+=	${LLVM_LDFLAGS}

LDADD+=		-lc++ \
		-lclangTooling \
		-lclangFrontendTool \
		-lclangFrontend \
		-lclangIndex \
		-lclangDriver \
		-lclangSerialization \
		-lclangCodeGen \
		-lclangParse \
		-lclangSema \
		-lclangStaticAnalyzerFrontend \
		-lclangStaticAnalyzerCheckers \
		-lclangStaticAnalyzerCore \
		-lclangAnalysis \
		-lclangARCMigrate \
		-lclangRewriteFrontend \
		-lclangRewrite \
		-lclangEdit \
		-lclangAST \
		-lclangASTMatchers \
		-lclangLex \
		-lclangBasic \
		-lclang \
		${LLVM_LIBS}
