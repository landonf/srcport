.PATH: ${.CURDIR}/../src

PROG_CXX=	tests
MAN=
CFLAGS=		-I${.CURDIR}/../src \
		-I${.CURDIR}/dependencies/catch

SRCS=		tests.cc \
		paths_tests.cc \
		project_tests.cc \
		work_queue_tests.cc

test: ${PROG_CXX}
	${.OBJDIR}/${PROG_CXX}

.PHONY: test

.include "../srcport.mk"
.include <bsd.prog.mk>
