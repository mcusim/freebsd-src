# Vendor sources and generated files
LDNSDIR= ${SRCTOP}/contrib/ldns
UNBOUNDDIR= ${SRCTOP}/contrib/unbound
LIBUNBOUNDDIR=${SRCTOP}/lib/libunbound

.PATH: ${UNBOUNDDIR} ${UNBOUNDDIR}/smallapp ${UNBOUNDDIR}/util ${UNBOUNDDIR}/doc

PROG=	local-unbound-checkconf
SRCS=	ub_event.c unbound-checkconf.c worker_cb.c
CFLAGS+=	-I${UNBOUNDDIR} -I${LDNSDIR}
CFLAGS+=	-I${LIBUNBOUNDDIR} -I${.CURDIR}
LIBADD=	unbound pthread
MAN=	local-unbound-checkconf.8

.include <bsd.prog.mk>
