iSER
====

iSCSI Extensions for RDMA.

1. General
----------

iSER is a computer network protocol that extends the iSCSI storage networking protocol to use Remote Direct Memory Access (RDMA).
RDMA is provided by either RoCE (RDMA over converged Ethernet) or InfiniBand to enable efficient data movement using hardware offload capabilities.
RDMA infrastructure provides benefits such as Zero-Copy, CPU offload, Reliable transport, Fabric consolidation and many more.
The iSER protocol eliminates some of the bottlenecks in the traditional iSCSI/TCP stack, provides low latency and high throughput and is well suited
for latency aware workloads.
The iSER protocol permits data to be transferred directly into and out of SCSI computer memory buffers (which connects computers to storage devices) without
intermediate data copies.

2. iSER Prerequisites
---------------------

Prior to installing the iSER package for freeBSD, the following prerequisites are required:

- OS version 11.0 and above built with OFED.

3. Building and installation
----------------------------

Install iSCSI/iSER by following steps:

	- build user-space iscsi tools (iscsid and iscsictl with iSER support)
		$ ./build.sh -u -s <share directory path>
	- build kernel space iscsi stack (with iSER support)
		$ ./build.sh -k -s <share directory path> -d <sys directory path>

4. HOWTO
--------

The following example creates SCSI Direct Access (da) device over iSCSI/iSER protocol
vs. remote target.

	- steps:
		$ service iscsid start
		$ iscsictl -A -t <target-name> -p <target portal> -T iser

In this stage, after the login and initialize stages are finished,
an iSCSI/iSER type device (/dev/da<device_id>) is available and ready for data transfer.

The following example removes SCSI Direct Access (da) device over iSCSI/iSER protocol.

	- steps:
		$ iscsictl -R -t <target-name>

