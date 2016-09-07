#!/bin/sh

BWN_SRCS="if_bwn.c if_bwn_phy_common.c if_bwn_phy_g.c \
    if_bwn_phy_lp.c if_bwn_phy_n.c if_bwn_util.c"

ARG0=$0
BASEDIR=`pwd`/`dirname $0`

SRCROOT=$1
OBJDIR=$2
KERNCONF=${3:-GENERIC}

BWN_DEPS="${BASEDIR}/obj/srcport"

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
SRCROOT=$(cd "$SRCROOT" && realpath `pwd`)
OBJDIR=$(cd "$OBJDIR" && realpath `pwd`)
KERNOBJS="${OBJDIR}/${SRCROOT}/sys/${KERNCONF}"

# Build (or rebuild) the tool
mkdir -p "${BASEDIR}/obj" || exit 1
make -C "${BASEDIR}" || exit 1

if [ ! -x "${BWN_DEPS}" ]; then
	echo "${BWN_DEPS} not found"
	exit 1
fi

# Source cflags
CFLAGS="-I${KERNOBJS} -I${SRCROOT}/sys -I${SRCROOT}/sys/sys -fno-builtin -nostdinc -D_KERNEL -Wno-pointer-sign"

cd "${SRCROOT}/sys/dev/bwn" || exit 1

${BWN_DEPS} -host-path="${SRCROOT}/sys/dev/siba" -src-path="${OBJDIR}" -src-path="${SRCROOT}" ${BWN_SRCS} -- ${CFLAGS}
RET=$?
echo "Done"

exit $RET
