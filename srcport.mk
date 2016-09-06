.PATH: ${.CURDIR}/src

SRCS+=		project.cpp

CFLAGS_WARN?=	-Wall -Werror -Wextra \
		-Wno-unused-parameter \
		-Wno-gnu-zero-variadic-macro-arguments

LLVM_VER?=	38
LLVM_CFG=	llvm-config${LLVM_VER}
LLVM_CXXFLAGS!=	${LLVM_CFG} --cxxflags | sed s/-fno-exceptions\ //g
LLVM_CFLAGS!=	${LLVM_CFG} --cflags
LLVM_LDFLAGS!=	${LLVM_CFG} --ldflags
LLVM_LIBS!=	${LLVM_CFG} --libs --system-libs

FTL_CXXFLAGS=	-isystem ${.CURDIR}/dependencies/ftl

PLCPP_CXXFLAGS=	-isystem ${.CURDIR}/dependencies/plstdcpp

CXXFLAGS+=	${LLVM_CXXFLAGS} \
		${FTL_CXXFLAGS} \
		${PLCPP_CXXFLAGS}

CFLAGS+=	-pthread \
		${LLVM_CFLAGS} \
		${CFLAGS_WARN}

LDFLAGS+=	${LLVM_LDFLAGS}

LDADD+=		-lc++ \
		-lclangTooling -lclangFrontendTool -lclangFrontend \
		-lclangDriver -lclangSerialization -lclangCodeGen \
		-lclangParse -lclangSema -lclangStaticAnalyzerFrontend \
		-lclangStaticAnalyzerCheckers -lclangStaticAnalyzerCore \
		-lclangAnalysis -lclangARCMigrate -lclangRewriteFrontend \
		-lclangRewrite -lclangEdit -lclangAST -lclangASTMatchers \
		-lclangLex -lclangBasic -lclang \
		${LLVM_LIBS}
