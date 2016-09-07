.PATH: ${.CURDIR}/../src

PROG_CXX=	tests
MAN=
CFLAGS=		-I${.CURDIR}/../src \
		-I${.CURDIR}/dependencies/catch

SRCS=		tests.cpp \
		paths_tests.cpp \
		project_tests.cpp

test: ${PROG_CXX}
	${.OBJDIR}/${PROG_CXX}

.PHONY: test

.include "../srcport.mk"
.include <bsd.prog.mk>
