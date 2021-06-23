# $FreeBSD$

.PATH: ${SRCTOP}/sys/dev/dpaa2

KMOD=	dpaa2

SRCS=	device_if.h \
	bus_if.h \
	ofw_bus_if.h

SRCS+=	dpaa2_mc.c \
	dpaa2_rc.c

.include <bsd.kmod.mk>
