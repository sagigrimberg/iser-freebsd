#!/bin/bash

DESTDIR=/usr
SHARE=/usr/share/mk
SYSDIR=/usr/src/sys
BINDIR=/bin
SBINDIR=/sbin
MANDIR=/share/man/man

usage()
{
        printf "Usage: ./${IAM} [OPTIONS] \n"
        printf "Script to build iser module\n"
        printf "Options:\n"
        printf "\t-u                  : Build user-space iscsi tools.\n"
        printf "\t-k                  : Build kernel-space iscsi stack.\n"
        printf "\t-S                  : Share directory path               (default: $SHARE).\n"
        printf "\t-D                  : sys directory path                 (default: $SYSDIR).\n"
        printf "\t-d                  : Install destination directory path (default: $DESTDIR).\n"
        printf "\t-m                  : Man directory path                 (default: $MANDIR).\n"
        printf "\t-b                  : bin directory path                 (default: $BINDIR).\n"
        printf "\t-s                  : sbin directory path                (default: $SBINDIR).\n"
        printf "\t-h                  : Show usage.\n"
}

read_args()
{
        while getopts :s:S:d:D:m:b::huk FLAG; do
                case ${FLAG} in
                        S)      SHARE=$OPTARG
                                ;;
                        D)      SYSDIR=$OPTARG
                                ;;
                        d)      DESTDIR=$OPTARG
                                ;;
                        m)      MANDIR=$OPTARG
                                ;;
                        b)      BINDIR=$OPTARG
                                ;;
                        s)      SBINDIR=$OPTARG
                                ;;
                        u)      USR=1
                                ;;

                        k)      KERNEL=1
                                ;;

                        h)      usage
                                exit
                                ;;
                        \?)	echo "Unknown Value $OPTARG"
				exit
                esac
        done

}

override_icl_conn_if()
{
	mv $SYSDIR/dev/iscsi/icl_conn_if.m $SYSDIR/dev/iscsi/icl_conn_if.m.orig
	cp $PWD/sys/dev/iscsi/icl_conn_if.m $SYSDIR/dev/iscsi/icl_conn_if.m
}

restore_icl_conn_if()
{
	mv -f $SYSDIR/dev/iscsi/icl_conn_if.m.orig $SYSDIR/dev/iscsi/icl_conn_if.m
}

main()
{
        read_args $*

	if [[ -n $KERNEL ]]; then
		override_icl_conn_if
		cmd="make -C $PWD/sys/modules/iscsi -m $SHARE SYSDIR=$SYSDIR all install clean cleandepend"
		echo $cmd
		$cmd
		restore_icl_conn_if
	fi

	if [[ -n $USR ]]; then
		cmd="make -C $PWD/usr.sbin/iscsid -m $SHARE DESTDIR=$DESTDIR BINDIR=$SBINDIR MANDIR=$MANDIR all install clean cleandepend"
		echo $cmd
		$cmd
		cmd="make -C $PWD/usr.bin/iscsictl -m $SHARE DESTDIR=$DESTDIR BINDIR=$BINDIR MANDIR=$MANDIR all install clean cleandepend"
		echo $cmd
		$cmd
	fi
}

IAM=`basename "$0"`
main "$@"
