PROG=		bwn_deps
MAN=

SRCS=		main.cpp

CC=		clang++${LLVM_VER}

LLVM_VER?=	38
LLVM_CFG=	llvm-config${LLVM_VER}
LLVM_CXXFLAGS!=	${LLVM_CFG} --cxxflags
LLVM_CFLAGS!=	${LLVM_CFG} --cflags
LLVM_LDFLAGS!=	${LLVM_CFG} --ldflags
LLVM_LIBS!=	${LLVM_CFG} --libs --system-libs

CXXFLAGS=	${LLVM_CXXFLAGS}
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

#	-lclangBasic \
	-lclangAST \
	-lclangASTMatchers \
	-lclangAnalysis \
	-lclangApplyReplacements \
	-lclangCodeGen \
	-lclangDriver \
	-lclangDynamicASTMatchers \
	-lclangEdit \
	-lclangFormat \
	-lclangFrontend \
	-lclangFrontendTool \
	-lclangIndex \
	-lclangLex \
	-lclangParse \
	-lclangQuery \
	-lclangRename \
	-lclangRewrite \
	-lclangRewriteFrontend \
	-lclangSema \
	-lclangSerialization \
	-lclangStaticAnalyzerCheckers \
	-lclangStaticAnalyzerCore \
	-lclangStaticAnalyzerFrontend \
	-lclangTooling \
	-lclangToolingCore \


.include <bsd.prog.mk>
