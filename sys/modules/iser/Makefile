# $FreeBSD$

.PATH:	${.CURDIR}/../../dev/iser/

.include <bsd.own.mk>

KMOD=	iser

SRCS=	icl_iser.c
SRCS+=	iser_initiator.c
SRCS+=	iser_memory.c
SRCS+=	iser_verbs.c
SRCS+=	vnode_if.h
SRCS+=	opt_inet.h
SRCS+=	opt_inet6.h
SRCS+=	opt_cam.h
SRCS+=	bus_if.h
SRCS+=	device_if.h
SRCS+=	icl_conn_if.h

CFLAGS+= -I${.CURDIR}/../../
CFLAGS+= -I${SYSDIR}/ofed/include
CFLAGS+= -DCONFIG_INFINIBAND_USER_MEM
CFLAGS+= -DINET6 -DINET
CFLAGS+= -fms-extensions

CFLAGS+=-DICL_KERNEL_PROXY

MFILES=	kern/bus_if.m kern/device_if.m dev/iscsi/icl_conn_if.m

.include <bsd.kmod.mk>
