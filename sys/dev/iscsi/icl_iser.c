/* $FreeBSD$ */
/*-
 * Copyright (c) 2015, Mellanox Technologies, Inc. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include "icl_iser.h"

SYSCTL_NODE(_kern, OID_AUTO, iser, CTLFLAG_RD, 0, "iSER module");
int iser_debug = 0;
SYSCTL_INT(_kern_iser, OID_AUTO, debug, CTLFLAG_RWTUN,
    &iser_debug, 0, "Enable iser debug messages");

static MALLOC_DEFINE(M_ICL_ISER, "icl_iser", "iSCSI iser backend");
static uma_zone_t icl_pdu_zone;

static volatile u_int	icl_iser_ncons;
struct iser_global ig;

static icl_conn_new_pdu_t	iser_conn_new_pdu;
static icl_conn_pdu_free_t	iser_conn_pdu_free;
static icl_conn_pdu_data_segment_length_t iser_conn_pdu_data_segment_length;
static icl_conn_pdu_append_data_t	iser_conn_pdu_append_data;
static icl_conn_pdu_queue_t	iser_conn_pdu_queue;
static icl_conn_handoff_t	iser_conn_handoff;
static icl_conn_free_t		iser_conn_free;
static icl_conn_close_t		iser_conn_close;
static icl_conn_connected_t	iser_conn_connected;
static icl_conn_task_setup_t	iser_conn_task_setup;
static icl_conn_task_done_t	iser_conn_task_done;
static icl_conn_pdu_get_data_t	iser_conn_pdu_get_data;

static kobj_method_t icl_iser_methods[] = {
	KOBJMETHOD(icl_conn_new_pdu, iser_conn_new_pdu),
	KOBJMETHOD(icl_conn_pdu_free, iser_conn_pdu_free),
	KOBJMETHOD(icl_conn_pdu_data_segment_length, iser_conn_pdu_data_segment_length),
	KOBJMETHOD(icl_conn_pdu_append_data, iser_conn_pdu_append_data),
	KOBJMETHOD(icl_conn_pdu_queue, iser_conn_pdu_queue),
	KOBJMETHOD(icl_conn_handoff, iser_conn_handoff),
	KOBJMETHOD(icl_conn_free, iser_conn_free),
	KOBJMETHOD(icl_conn_close, iser_conn_close),
	KOBJMETHOD(icl_conn_connected, iser_conn_connected),
	KOBJMETHOD(icl_conn_task_setup, iser_conn_task_setup),
	KOBJMETHOD(icl_conn_task_done, iser_conn_task_done),
	KOBJMETHOD(icl_conn_pdu_get_data, iser_conn_pdu_get_data),
	{ 0, 0 }
};

DEFINE_CLASS(icl_iser, icl_iser_methods, sizeof(struct iser_conn));

/**
 * iser_initialize_headers() - Initialize task headers
 * @pdu:       iser pdu
 * @iser_conn:    iser connection
 *
 * Notes:
 * This routine may race with iser teardown flow for scsi
 * error handling TMFs. So for TMF we should acquire the
 * state mutex to avoid dereferencing the IB device which
 * may have already been terminated (racing teardown sequence).
 */
int
iser_initialize_headers(struct icl_iser_pdu *pdu, struct iser_conn *iser_conn)
{
	struct iser_tx_desc *tx_desc = &pdu->desc;
	struct iser_device *device = iser_conn->ib_conn.device;
	u64 dma_addr;
	int ret = 0;

	dma_addr = ib_dma_map_single(device->ib_device, (void *)tx_desc,
				ISER_HEADERS_LEN, DMA_TO_DEVICE);
	if (ib_dma_mapping_error(device->ib_device, dma_addr)) {
		ret = -ENOMEM;
		goto out;
	}

	tx_desc->mapped = true;
	tx_desc->dma_addr = dma_addr;
	tx_desc->tx_sg[0].addr   = tx_desc->dma_addr;
	tx_desc->tx_sg[0].length = ISER_HEADERS_LEN;
	tx_desc->tx_sg[0].lkey   = device->mr->lkey;

out:

	return (ret);
}

int
iser_conn_pdu_append_data(struct icl_conn *ic, struct icl_pdu *request,
			  const void *addr, size_t len, int flags)
{
	struct iser_conn *iser_conn = icl_to_iser_conn(ic);

	if (request->ip_bhs->bhs_opcode & ISCSI_BHS_OPCODE_LOGIN_REQUEST ||
	    request->ip_bhs->bhs_opcode & ISCSI_BHS_OPCODE_TEXT_REQUEST) {
		ISER_DBG("copy to login buff");
		memcpy(iser_conn->login_req_buf, addr, len);
		request->ip_data_len = len;
	}

	return (0);
}

void
iser_conn_pdu_get_data(struct icl_conn *ic, struct icl_pdu *ip,
		       size_t off, void *addr, size_t len)
{
	/* If we have a receive data, copy it to upper layer buffer */
	if (ip->ip_data_mbuf)
		memcpy(addr, ip->ip_data_mbuf + off, len);
}

/*
 * Allocate icl_pdu with empty BHS to fill up by the caller.
 */
struct icl_pdu *
iser_new_pdu(struct icl_conn *ic, int flags)
{
	struct icl_iser_pdu *iser_pdu;
	struct icl_pdu *ip;
	struct iser_conn *iser_conn = icl_to_iser_conn(ic);

	iser_pdu = uma_zalloc(icl_pdu_zone, flags | M_ZERO);
	if (iser_pdu == NULL) {
		ISER_WARN("failed to allocate %zd bytes", sizeof(*iser_pdu));
		return (NULL);
	}

	iser_pdu->iser_conn = iser_conn;
	ip = &iser_pdu->icl_pdu;
	ip->ip_conn = ic;
	ip->ip_bhs = &iser_pdu->desc.iscsi_header;

	return (ip);
}

struct icl_pdu *
iser_conn_new_pdu(struct icl_conn *ic, int flags)
{
	return (iser_new_pdu(ic, flags));
}

void
iser_pdu_free(struct icl_conn *ic, struct icl_pdu *ip)
{
	struct icl_iser_pdu *iser_pdu = icl_to_iser_pdu(ip);

	uma_zfree(icl_pdu_zone, iser_pdu);
}

size_t
iser_conn_pdu_data_segment_length(struct icl_conn *ic,
				  const struct icl_pdu *request)
{
	return (ntoh24(request->ip_bhs->bhs_data_segment_len));
}

void
iser_conn_pdu_free(struct icl_conn *ic, struct icl_pdu *ip)
{
	iser_pdu_free(ic, ip);
}

static bool
is_control_opcode(uint8_t opcode)
{
	bool is_control = false;

	switch (opcode & ISCSI_OPCODE_MASK) {
		case ISCSI_BHS_OPCODE_NOP_OUT:
		case ISCSI_BHS_OPCODE_LOGIN_REQUEST:
		case ISCSI_BHS_OPCODE_LOGOUT_REQUEST:
		case ISCSI_BHS_OPCODE_TEXT_REQUEST:
			is_control = true;
			break;
		case ISCSI_BHS_OPCODE_SCSI_COMMAND:
			is_control = false;
			break;
		default:
			ISER_ERR("unknown opcode %d", opcode);
	}

	return (is_control);
}

void
iser_conn_pdu_queue(struct icl_conn *ic, struct icl_pdu *ip)
{
	struct iser_conn *iser_conn = icl_to_iser_conn(ic);
	struct icl_iser_pdu *iser_pdu = icl_to_iser_pdu(ip);
	int ret;

	ret = iser_initialize_headers(iser_pdu, iser_conn);
	if (ret) {
		ISER_ERR("Failed to map TX descriptor pdu %p", iser_pdu);
		return;
	}

	if (is_control_opcode(ip->ip_bhs->bhs_opcode))
		iser_send_control(iser_conn, iser_pdu);
	else
		iser_send_command(iser_conn, iser_pdu);
}

static struct icl_conn *
iser_new_conn(const char *name, struct mtx *lock)
{
	struct iser_conn *iser_conn;
	struct icl_conn *ic;

	refcount_acquire(&icl_iser_ncons);

	iser_conn = (struct iser_conn *)kobj_create(&icl_iser_class, M_ICL_ISER, M_WAITOK | M_ZERO);
	if (!iser_conn) {
		ISER_ERR("failed to allocate iser conn");
		refcount_release(&icl_iser_ncons);
		return (NULL);
	}

	mtx_init(&iser_conn->up_lock, "iser_lock", NULL, MTX_DEF);
	cv_init(&iser_conn->up_cv, "iser_cv");
	mtx_init(&iser_conn->state_mutex, "iser_conn_state_mutex", NULL, MTX_DEF);
	mtx_init(&iser_conn->ib_conn.flush_lock, "flush_lock", NULL, MTX_DEF);
	cv_init(&iser_conn->ib_conn.flush_cv, "flush_cv");
	mtx_init(&iser_conn->ib_conn.lock, "lock", NULL, MTX_DEF);

	ic = &iser_conn->icl_conn;
	ic->ic_lock = lock;
	ic->ic_name = name;
	ic->ic_offload = strdup("iser", M_TEMP);

	return (ic);
}

void
iser_conn_free(struct icl_conn *ic)
{
	struct iser_conn *iser_conn = icl_to_iser_conn(ic);

	cv_destroy(&iser_conn->ib_conn.flush_cv);
	mtx_destroy(&iser_conn->ib_conn.flush_lock);
	mtx_destroy(&iser_conn->state_mutex);
	cv_destroy(&iser_conn->up_cv);
	mtx_destroy(&iser_conn->up_lock);
	kobj_delete((struct kobj *)iser_conn, M_ICL_ISER);
	refcount_release(&icl_iser_ncons);
}

int
iser_conn_handoff(struct icl_conn *ic, int cmds_max)
{
	struct iser_conn *iser_conn = icl_to_iser_conn(ic);

	if (iser_alloc_rx_descriptors(iser_conn, cmds_max))
		goto out;

	if (iser_post_recvm(iser_conn, iser_conn->min_posted_rx))
		goto post_error;

	return (0);

post_error:
	iser_free_rx_descriptors(iser_conn);
out:
	ISER_ERR("fail in handoff stage");
	return (-ENOMEM);

}

void
iser_conn_close(struct icl_conn *ic)
{
	struct iser_conn *iser_conn = icl_to_iser_conn(ic);

	ISER_INFO("closing conn %p", iser_conn);

	iser_conn_terminate(iser_conn);
	iser_conn_release(iser_conn);

}

bool
iser_conn_connected(struct icl_conn *ic)
{
	struct iser_conn *iser_conn = icl_to_iser_conn(ic);
	bool connected;

	mtx_lock(&iser_conn->state_mutex);
	connected = (iser_conn->state == ISER_CONN_UP);
	mtx_unlock(&iser_conn->state_mutex);

	return (connected);
}

int
iser_conn_task_setup(struct icl_conn *ic, struct ccb_scsiio *csio,
		     uint32_t *task_tagp, void **prvp, struct icl_pdu *ip)
{
	struct icl_iser_pdu *iser_pdu = icl_to_iser_pdu(ip);

	*prvp = ip;
	iser_pdu->csio = csio;

	return (0);
}

void
iser_conn_task_done(struct icl_conn *ic, void *prv)
{
	struct icl_pdu *ip = prv;
	struct icl_iser_pdu *iser_pdu = icl_to_iser_pdu(ip);
	struct iser_device *device = iser_pdu->iser_conn->ib_conn.device;
	struct iser_tx_desc *tx_desc = &iser_pdu->desc;

	if (iser_pdu->dir[ISER_DIR_IN]) {
		iser_unreg_rdma_mem(iser_pdu, ISER_DIR_IN);
		iser_dma_unmap_task_data(iser_pdu,
					 &iser_pdu->data[ISER_DIR_IN],
					 DMA_FROM_DEVICE);
	}

	if (iser_pdu->dir[ISER_DIR_OUT]) {
		iser_unreg_rdma_mem(iser_pdu, ISER_DIR_OUT);
		iser_dma_unmap_task_data(iser_pdu,
					 &iser_pdu->data[ISER_DIR_OUT],
					 DMA_TO_DEVICE);
	}

	if (likely(tx_desc->mapped)) {
		ib_dma_unmap_single(device->ib_device, tx_desc->dma_addr,
				    ISER_HEADERS_LEN, DMA_TO_DEVICE);
		tx_desc->mapped = false;
	}

	iser_pdu_free(ic, ip);
}

static int
iser_limits(size_t *limitp)
{
	*limitp = 128 * 1024;

	return (0);
}

static int
icl_iser_load(void)
{
	int error;

	ISER_DBG("Starting iSER datamover...");

	icl_pdu_zone = uma_zcreate("icl_iser_pdu", sizeof(struct icl_iser_pdu),
				   NULL, NULL, NULL, NULL,
				   UMA_ALIGN_PTR, 0);
	/* FIXME: Check rc */

	refcount_init(&icl_iser_ncons, 0);

	error = icl_register("iser", 0, iser_limits, iser_new_conn);
	KASSERT(error == 0, ("failed to register iser"));

	memset(&ig, 0, sizeof(struct iser_global));

	/* device init is called only after the first addr resolution */
	sx_init(&ig.device_list_mutex,  "global_device_lock");
	INIT_LIST_HEAD(&ig.device_list);
	mtx_init(&ig.connlist_mutex, "global_conn_lock", NULL, MTX_DEF);
	INIT_LIST_HEAD(&ig.connlist);

	return (error);
}

static int
icl_iser_unload(void)
{
	ISER_DBG("Removing iSER datamover...");

	if (icl_iser_ncons != 0)
		return (EBUSY);

	mtx_destroy(&ig.connlist_mutex);
	sx_destroy(&ig.device_list_mutex);

	icl_unregister("iser");

	uma_zdestroy(icl_pdu_zone);

	return (0);
}

static int
icl_iser_modevent(module_t mod, int what, void *arg)
{
	switch (what) {
	case MOD_LOAD:
		return (icl_iser_load());
	case MOD_UNLOAD:
		return (icl_iser_unload());
	default:
		return (EINVAL);
	}
}

moduledata_t icl_iser_data = {
	.name = "icl_iser",
	.evhand = icl_iser_modevent,
	.priv = 0
};

DECLARE_MODULE(icl_iser, icl_iser_data, SI_SUB_DRIVERS, SI_ORDER_MIDDLE);
MODULE_DEPEND(icl_iser, icl, 1, 1, 1);
MODULE_DEPEND(icl_iser, ibcore, 1, 1, 1);
MODULE_DEPEND(icl_iser, linuxapi, 1, 1, 1);
MODULE_VERSION(icl_iser, 1);

