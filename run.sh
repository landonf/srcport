#!/bin/sh

ARG0=$0
BASEDIR=`pwd`/`dirname $0`

SRCROOT=$(realpath $1)
OBJDIR=$(realpath $2)
KERNCONF=${3:-GENERIC}

BWN_SRC="${SRCROOT}/dev/bwn"
BWN_GNU_SRC="${SRCROOT}/gnu/dev/bwn/phy_n"
BWN_SRCS="\
	${BWN_SRC}/if_bwn.c \
	${BWN_SRC}/if_bwn_phy_common.c \
	${BWN_SRC}/if_bwn_phy_g.c \
	${BWN_SRC}/if_bwn_phy_lp.c \
	${BWN_SRC}/if_bwn_phy_n.c \
	${BWN_SRC}/if_bwn_util.c
	${BWN_GNU_SRC}/if_bwn_phy_n_core.c
	${BWN_GNU_SRC}/if_bwn_phy_n_ppr.c
	${BWN_GNU_SRC}/if_bwn_phy_n_tables.c
	${BWN_GNU_SRC}/if_bwn_radio_2055.c
	${BWN_GNU_SRC}/if_bwn_radio_2056.c
	${BWN_GNU_SRC}/if_bwn_radio_2057.c"

SIBA_SRC="${SRCROOT}/dev/siba"
SIBA_SRCS="\
	${SIBA_SRC}/siba_core.c \
	${SIBA_SRC}/siba_bwn.c"

SRCPORT="${BASEDIR}/obj/srcport"

usage() {
	echo "Usage: ${ARG0} <freebsd src> <obj dir> [kernconf]"
}


if [ -z "${SRCROOT}" ] || [ -z "${OBJDIR}" ]; then
	usage
	exit 1
fi

if [ ! -d "${SRCROOT}" ]; then
	echo "${SRCROOT}: directory not found"
	exit 1
fi

if [ ! -d "${OBJDIR}" ]; then
	echo "${OBJDIR}: directory not found"
	exit 1
fi

# Resolve absolute paths
KERNOBJS="${OBJDIR}/${SRCROOT}/${KERNCONF}"

# Build (or rebuild) the tool
mkdir -p "${BASEDIR}/obj" 1>&2 || exit 1
make -C "${BASEDIR}" 1>&2 || exit 1

if [ ! -x "${SRCPORT}" ]; then
	echo "${SRCPORT} not found"
	exit 1
fi

# Source cflags
CFLAGS="-I${KERNOBJS} -I${SRCROOT} -I${SRCROOT}/sys -fno-builtin -nostdinc -D_KERNEL -DINVARIANTS -DINVARIANT_SUPPORT -Wno-pointer-sign -Wno-shift-count-overflow"

${SRCPORT} -host-path="${SRCROOT}/dev/siba" -src-path="${OBJDIR}" -src-path="${SRCROOT}" --format=bwn_stubs ${BWN_SRCS} ${SIBA_SRCS} -- ${CFLAGS} || exit 1

echo "Analysis Complete" 1>&2
exit 0
