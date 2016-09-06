.PATH: ${.CURDIR}/src

PROG=		srcport
MAN=

SRCS=		main.cpp

CC=		clang++${LLVM_VER}

LLVM_VER?=	38
LLVM_CFG=	llvm-config${LLVM_VER}
LLVM_CXXFLAGS!=	${LLVM_CFG} --cxxflags | sed s/-fno-exceptions\ //g
LLVM_CFLAGS!=	${LLVM_CFG} --cflags
LLVM_LDFLAGS!=	${LLVM_CFG} --ldflags
LLVM_LIBS!=	${LLVM_CFG} --libs --system-libs

FTL_CXXFLAGS=	-isystem ${.CURDIR}/dependencies/ftl

CXXFLAGS=	${LLVM_CXXFLAGS} \
		${FTL_CXXFLAGS}
CFLAGS=		-pthread \
		${LLVM_CFLAGS}
LDFLAGS=	${LLVM_LDFLAGS} \

LDADD=		-lc++ \
		-lclangTooling -lclangFrontendTool -lclangFrontend \
		-lclangDriver -lclangSerialization -lclangCodeGen \
		-lclangParse -lclangSema -lclangStaticAnalyzerFrontend \
		-lclangStaticAnalyzerCheckers -lclangStaticAnalyzerCore \
		-lclangAnalysis -lclangARCMigrate -lclangRewriteFrontend \
		-lclangRewrite -lclangEdit -lclangAST -lclangASTMatchers \
		-lclangLex -lclangBasic -lclang \
		${LLVM_LIBS}

.include <bsd.prog.mk>
