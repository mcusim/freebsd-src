.PATH: ${SRCTOP}/sys/netpfil/pf

KMOD=	pfsync
SRCS=	if_pfsync.c pfsync_nv.c \
	opt_pf.h opt_inet.h opt_inet6.h opt_global.h
SRCS+=	bus_if.h device_if.h

.if !defined(KERNBUILDDIR)
.if defined(VIMAGE)
opt_global.h:
	echo "#define VIMAGE 1" >> ${.TARGET}
CFLAGS+=	-include opt_global.h
.endif
.endif

.include <bsd.kmod.mk>
