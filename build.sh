#!/bin/bash

usage()
{
        printf "Usage: ./${IAM} [OPTIONS] \n"
        printf "Script to build iser module\n"
        printf "Options:\n"
        printf "\t-u                  : build user-space iscsi tools.\n"
        printf "\t-k                  : build kernel space iscsi stack.\n"
        printf "\t-s                  : share directory path.\n"
        printf "\t-d                  : sys directory path.\n"
        printf "\t-h                  : Show usage.\n"
}

read_args()
{
	SHARE=/usr/share/mk
	SYSDIR=/usr/src/sys

        while getopts :s:d:huk FLAG; do
                case ${FLAG} in
                        s)      SHARE=$OPTARG
                                ;;
                        d)      SYSDIR=$OPTARG
                                ;;
                        u)      USR=1
                                ;;

                        k)      KERNEL=1
                                ;;

                        h)      usage
                                exit
                                ;;
                        \?)     exit
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
		cmd="make -C $PWD/usr.sbin/iscsid -m $SHARE all install clean cleandepend"
		echo $cmd
		$cmd
		cmd="make -C $PWD/usr.bin/iscsictl -m $SHARE all install clean cleandepend"
		echo $cmd
		$cmd
	fi
}

main "$@"
