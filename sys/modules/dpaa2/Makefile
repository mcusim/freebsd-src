# $FreeBSD$

.PATH:	${SRCTOP}/sys/dev/dpaa2

KMOD=	dpaa2
SRCS=	bus_if.h \
	device_if.h \
	opt_platform.h \
	dpaa2_mc.c \
	dpaa2_rc.c \
	dpaa2_io.c \
	dpaa2_bp.c \
	dpaa2_mcp.c \
	dpaa2_swp.c

.if ${MACHINE_CPUARCH} == "aarch64" || \
    ${MACHINE_CPUARCH} == "amd64" || \
    ${MACHINE_CPUARCH} == "i386"
SRCS+=	dpaa2_mc_acpi.c \
	opt_acpi.h \
	acpi_if.h \
	acpi_bus_if.h \
	pcib_if.h \
	pci_if.h
.endif

.if !empty(OPT_FDT)
SRCS+=	dpaa2_mc_fdt.c \
	ofw_bus_if.h
.endif

.include <bsd.kmod.mk>
