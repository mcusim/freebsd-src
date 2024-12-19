.PATH: ${SRCTOP}/sys/dev/iicbus/pmic/rockchip

KMOD=	rk818
SRCS=	rk8xx.c \
	rk818.c \
	rk818_battery.c \
	rk8xx_clocks.c \
	rk8xx_rtc.c \
	rk8xx_regulators.c \

SRCS+=	\
	bus_if.h \
	device_if.h \
	iicbus_if.h \
	regnode_if.h \
	regdev_if.h \
	ofw_bus_if.h \
	opt_platform.h \

.include <bsd.kmod.mk>
