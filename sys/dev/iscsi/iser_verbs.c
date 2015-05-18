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

static MALLOC_DEFINE(M_ISER_VERBS, "iser_verbs", "iser verbs backend");
static int iser_cq_poll_limit = 512;

static void
iser_cq_event_callback(struct ib_event *cause, void *context)
{
	ISER_ERR("got cq event %d", cause->event);
}

static void
iser_qp_event_callback(struct ib_event *cause, void *context)
{
	ISER_ERR("got qp event %d", cause->event);
}

static void
iser_event_handler(struct ib_event_handler *handler,
				struct ib_event *event)
{
	ISER_ERR("async event %d on device %s port %d",
		 event->event, event->device->name,
		 event->element.port_num);
}

/**
 * is_iser_tx_desc - Indicate if the completion wr_id
 *     is a TX descriptor or not.
 * @iser_conn: iser connection
 * @wr_id: completion WR identifier
 *
 * Since we cannot rely on wc opcode in FLUSH errors
 * we must work around it by checking if the wr_id address
 * falls in the iser connection rx_descs buffer. If so
 * it is an RX descriptor, otherwize it is a TX.
 */
static inline bool
is_iser_tx_desc(struct iser_conn *iser_conn, void *wr_id)
{
	void *start = iser_conn->rx_descs;
	u64 len = iser_conn->num_rx_descs * sizeof(*iser_conn->rx_descs);
	void *end = (void *)((uintptr_t)start + (uintptr_t)len);

	if (start) {
		if (wr_id >= start && wr_id < end)
			return false;
	} else {
		return ((uintptr_t)wr_id != (uintptr_t)iser_conn->login_resp_buf);
	}

	return true;
}

/**
 * iser_handle_comp_error() - Handle error completion
 * @ib_conn:   connection RDMA resources
 * @wc:        work completion
 *
 * Notes: Update post_recv_buf_count in case of recv error completion.
 *        For non-FLUSH error completion we should also notify iscsi layer that
 *        connection is failed (in case we passed bind stage).
 */
static void
iser_handle_comp_error(struct ib_conn *ib_conn,
		       struct ib_wc *wc)
{
	void *wr_id = (void *)(uintptr_t)wc->wr_id;
	struct iser_conn *iser_conn = container_of(ib_conn, struct iser_conn,
						   ib_conn);

	if (is_iser_tx_desc(iser_conn, wr_id)) {
		ISER_DBG("got send comp error");
	} else {
		ISER_DBG("got recv comp error");
		ib_conn->post_recv_buf_count--;
	}
	if (wc->status != IB_WC_WR_FLUSH_ERR)
		iser_conn->icl_conn.ic_error(&iser_conn->icl_conn);
}

/**
 * iser_handle_wc - handle a single work completion
 * @wc: work completion
 *
 * Soft-IRQ context, work completion can be either
 * SEND or RECV, and can turn out successful or
 * with error (or flush error).
 */
static void iser_handle_wc(struct ib_wc *wc)
{
	struct ib_conn *ib_conn;
	struct iser_tx_desc *tx_desc;
	struct iser_rx_desc *rx_desc;

	ib_conn = wc->qp->qp_context;
	if (likely(wc->status == IB_WC_SUCCESS)) {
		if (wc->opcode == IB_WC_RECV) {
			rx_desc = (struct iser_rx_desc *)(uintptr_t)wc->wr_id;
			iser_rcv_completion(rx_desc, wc->byte_len,
					    ib_conn);
		} else
		if (wc->opcode == IB_WC_SEND) {
			tx_desc = (struct iser_tx_desc *)(uintptr_t)wc->wr_id;
			iser_snd_completion(tx_desc, ib_conn);
		} else {
			ISER_ERR("Unknown wc opcode %d", wc->opcode);
		}
	} else {
		if (wc->status != IB_WC_WR_FLUSH_ERR) {
			ISER_ERR("wr id %lx status %d vend_err %x",
				 wc->wr_id, wc->status, wc->vendor_err);
		} else {
			ISER_DBG("flush error: wr id %lx", wc->wr_id);
		}

		if (wc->wr_id == ISER_BEACON_WRID) {
			/* all flush errors were consumed */
			ISER_DBG("got ISER_BEACON_WRID");
			cv_signal(&ib_conn->flush_cv);
		} else {
			iser_handle_comp_error(ib_conn, wc);
		}
	}
}

static void
iser_cq_tasklet_fn(void *data, int pending)
{
	struct iser_comp *comp = (struct iser_comp *)data;
	struct ib_cq *cq = comp->cq;
	struct ib_wc *const wcs = comp->wcs;
	int completed = 0;
	int i;
	int n;

	while ((n = ib_poll_cq(cq, ARRAY_SIZE(comp->wcs), wcs)) > 0) {
		for (i = 0; i < n; i++)
			iser_handle_wc(&wcs[i]);

		completed += n;
		if (completed >= iser_cq_poll_limit)
			break;
	}

	/*
	 * It is assumed here that arming CQ only once its empty
	 * would not cause interrupts to be missed.
	 */
	ib_req_notify_cq(cq, IB_CQ_NEXT_COMP);
}

static void
iser_cq_callback(struct ib_cq *cq, void *cq_context)
{
	struct iser_comp *comp = cq_context;

	taskqueue_enqueue_fast(comp->tq, &comp->task);
}

/**
 * iser_create_device_ib_res - creates Protection Domain (PD), Completion
 * Queue (CQ), DMA Memory Region (DMA MR) with the device associated with
 * the adapator.
 *
 * returns 0 on success, -1 on failure
 */
static int
iser_create_device_ib_res(struct iser_device *device)
{
	struct ib_device_attr *dev_attr = &device->dev_attr;
	int ret, i, max_cqe;
	
	ret = ib_query_device(device->ib_device, dev_attr);
	if (ret) {
		ISER_ERR("Query device failed for %s", device->ib_device->name);
		return (ret);
	}

	if (!(dev_attr->device_cap_flags & IB_DEVICE_MEM_MGT_EXTENSIONS)) {
		ISER_ERR("device %s doesn't support Fastreg, "
			 "can't register memory", device->ib_device->name);
		return (-1);
	}
	
	device->comps_used = min(mp_ncpus, device->ib_device->num_comp_vectors);

	device->comps = malloc(device->comps_used * sizeof(*device->comps),
		M_ISER_VERBS, M_WAITOK | M_ZERO);
	if (!device->comps)
		goto comps_err;

	max_cqe = min(ISER_MAX_CQ_LEN, dev_attr->max_cqe);

	ISER_DBG("using %d CQs, device %s supports %d vectors max_cqe %d",
		 device->comps_used, device->ib_device->name,
		 device->ib_device->num_comp_vectors, max_cqe);

	device->pd = ib_alloc_pd(device->ib_device);
	if (IS_ERR(device->pd))
		goto pd_err;

	for (i = 0; i < device->comps_used; i++) {
		struct iser_comp *comp = &device->comps[i];

		comp->device = device;
		comp->cq = ib_create_cq(device->ib_device,
					iser_cq_callback,
					iser_cq_event_callback,
					(void *)comp,
					max_cqe, i);
		if (IS_ERR(comp->cq)) {
			comp->cq = NULL;
			goto cq_err;
		}

		if (ib_req_notify_cq(comp->cq, IB_CQ_NEXT_COMP))
			goto cq_err;

		TASK_INIT(&comp->task, 0, iser_cq_tasklet_fn, comp);
		comp->tq = taskqueue_create_fast("iser_taskq", M_NOWAIT,
				taskqueue_thread_enqueue, &comp->tq);
		taskqueue_start_threads(&comp->tq, 1, PI_NET, "iser taskq");
	}

	device->mr = ib_get_dma_mr(device->pd, IB_ACCESS_LOCAL_WRITE |
				   IB_ACCESS_REMOTE_WRITE |
				   IB_ACCESS_REMOTE_READ);
	if (IS_ERR(device->mr))
		goto dma_mr_err;

	INIT_IB_EVENT_HANDLER(&device->event_handler, device->ib_device,
				iser_event_handler);
	if (ib_register_event_handler(&device->event_handler))
		goto handler_err;

	return (0);

handler_err:
	ib_dereg_mr(device->mr);
dma_mr_err:
	for (i = 0; i < device->comps_used; i++)
		taskqueue_free(device->comps[i].tq);
cq_err:
	for (i = 0; i < device->comps_used; i++) {
		struct iser_comp *comp = &device->comps[i];
		if (comp->cq)
			ib_destroy_cq(comp->cq);
	}
	ib_dealloc_pd(device->pd);
pd_err:
	free(device->comps, M_ISER_VERBS);
comps_err:
	ISER_ERR("failed to allocate an IB resource");
	return (-1);
}

/**
 * iser_free_device_ib_res - destroy/dealloc/dereg the DMA MR,
 * CQ and PD created with the device associated with the adapator.
 */
static void
iser_free_device_ib_res(struct iser_device *device)
{
	int i;

	for (i = 0; i < device->comps_used; i++) {
		struct iser_comp *comp = &device->comps[i];

		taskqueue_free(comp->tq);
		ib_destroy_cq(comp->cq);
		comp->cq = NULL;
	}

	(void)ib_unregister_event_handler(&device->event_handler);
	(void)ib_dereg_mr(device->mr);
	(void)ib_dealloc_pd(device->pd);

	free(device->comps, M_ISER_VERBS);
	device->comps = NULL;

	device->mr = NULL;
	device->pd = NULL;
}

static int
iser_alloc_reg_res(struct ib_device *ib_device,
		   struct ib_pd *pd,
		   struct iser_reg_resources *res)
{
	int ret;

	res->frpl = ib_alloc_fast_reg_page_list(ib_device,
						ISCSI_ISER_SG_TABLESIZE + 1);
	if (IS_ERR(res->frpl)) {
		ret = PTR_ERR(res->frpl);
		ISER_ERR("Failed to allocate fast reg page list err=%d", ret);
		return PTR_ERR(res->frpl);
	}

	res->mr = ib_alloc_fast_reg_mr(pd, ISCSI_ISER_SG_TABLESIZE + 1);
	if (IS_ERR(res->mr)) {
		ret = PTR_ERR(res->mr);
		ISER_ERR("Failed to allocate  fast reg mr err=%d", ret);
		goto fast_reg_mr_failure;
	}
	res->mr_valid = 1;

	return (0);

fast_reg_mr_failure:
	ib_free_fast_reg_page_list(res->frpl);

	return (ret);
}

static void
iser_free_reg_res(struct iser_reg_resources *rsc)
{
	ib_dereg_mr(rsc->mr);
	ib_free_fast_reg_page_list(rsc->frpl);
}

static struct fast_reg_descriptor *
iser_create_fastreg_desc(struct ib_device *ib_device, struct ib_pd *pd)
{
	struct fast_reg_descriptor *desc;
	int ret;

	desc = malloc(sizeof(*desc), M_ISER_VERBS, M_WAITOK | M_ZERO);
	if (!desc) {
		ISER_ERR("Failed to allocate a new fastreg descriptor");
		return ERR_PTR(-ENOMEM);
	}

	ret = iser_alloc_reg_res(ib_device, pd, &desc->rsc);
	if (ret) {
		ISER_ERR("failed to allocate reg_resources");
		goto err;
	}

	return (desc);
err:
	free(desc, M_ISER_VERBS);
	return ERR_PTR(ret);
}

/**
 * iser_create_fmr_pool - Creates FMR pool and page_vector
 *
 * returns 0 on success, or errno code on failure
 */
int
iser_create_fastreg_pool(struct ib_conn *ib_conn, unsigned cmds_max)
{
	struct iser_device *device = ib_conn->device;
	struct fast_reg_descriptor *desc;
	int i, ret;

	INIT_LIST_HEAD(&ib_conn->fastreg.pool);
	ib_conn->fastreg.pool_size = 0;
	for (i = 0; i < cmds_max; i++) {
		desc = iser_create_fastreg_desc(device->ib_device, device->pd);
		if (IS_ERR(desc)) {
			ret = PTR_ERR(desc);
			ISER_ERR("Failed to create fastreg descriptor err=%d",
				 ret);
			goto err;
		}

		list_add_tail(&desc->list, &ib_conn->fastreg.pool);
		ib_conn->fastreg.pool_size++;
	}

	return (0);

err:
	iser_free_fastreg_pool(ib_conn);
	return (ret);
}

/**
 * iser_free_fmr_pool - releases the FMR pool and page vec
 */
void
iser_free_fastreg_pool(struct ib_conn *ib_conn)
{
	struct fast_reg_descriptor *desc, *tmp;
	int i = 0;

	if (list_empty(&ib_conn->fastreg.pool))
		return;

	ISER_DBG("freeing conn %p fr pool", ib_conn);

	list_for_each_entry_safe(desc, tmp, &ib_conn->fastreg.pool, list) {
		list_del(&desc->list);
		iser_free_reg_res(&desc->rsc);
		free(desc, M_ISER_VERBS);
		++i;
	}

	if (i < ib_conn->fastreg.pool_size)
		ISER_WARN("pool still has %d regions registered",
			  ib_conn->fastreg.pool_size - i);
}

/**
 * iser_create_ib_conn_res - Queue-Pair (QP)
 *
 * returns 0 on success, -1 on failure
 */
static int
iser_create_ib_conn_res(struct ib_conn *ib_conn)
{
	struct iser_conn *iser_conn;
	struct iser_device *device;
	struct ib_device_attr *dev_attr;
	struct ib_qp_init_attr init_attr;
	int index, min_index = 0;
	int ret = -ENOMEM;

	iser_conn = container_of(ib_conn, struct iser_conn, ib_conn);
	device = ib_conn->device;
	dev_attr = &device->dev_attr;

	mtx_lock(&ig.connlist_mutex);
	/* select the CQ with the minimal number of usages */
	for (index = 0; index < device->comps_used; index++) {
		if (device->comps[index].active_qps <
		    device->comps[min_index].active_qps)
			min_index = index;
	}
	ib_conn->comp = &device->comps[min_index];
	ib_conn->comp->active_qps++;
	mtx_unlock(&ig.connlist_mutex);
	ISER_INFO("cq index %d used for ib_conn %p", min_index, ib_conn);

	memset(&init_attr, 0, sizeof init_attr);
	init_attr.event_handler = iser_qp_event_callback;
	init_attr.qp_context	= (void *)ib_conn;
	init_attr.send_cq	= ib_conn->comp->cq;
	init_attr.recv_cq	= ib_conn->comp->cq;
	init_attr.cap.max_recv_wr  = ISER_QP_MAX_RECV_DTOS;
	init_attr.cap.max_send_sge = 2;
	init_attr.cap.max_recv_sge = 1;
	init_attr.sq_sig_type	= IB_SIGNAL_REQ_WR;
	init_attr.qp_type	= IB_QPT_RC;

	if (dev_attr->max_qp_wr > ISER_QP_MAX_REQ_DTOS) {
		init_attr.cap.max_send_wr  = ISER_QP_MAX_REQ_DTOS + 1;
		iser_conn->max_cmds =
			ISER_GET_MAX_XMIT_CMDS(ISER_QP_MAX_REQ_DTOS);
	} else {
		init_attr.cap.max_send_wr = dev_attr->max_qp_wr;
		iser_conn->max_cmds =
			ISER_GET_MAX_XMIT_CMDS(dev_attr->max_qp_wr);
	}
	ISER_DBG("device %s supports max_send_wr %d",
	         device->ib_device->name, dev_attr->max_qp_wr);

	ret = rdma_create_qp(ib_conn->cma_id, device->pd, &init_attr);
	if (ret)
		goto out_err;

	ib_conn->qp = ib_conn->cma_id->qp;
	ISER_DBG("setting conn %p cma_id %p qp %p",
		 ib_conn, ib_conn->cma_id,
		 ib_conn->cma_id->qp);

	return (ret);

out_err:
	mtx_lock(&ig.connlist_mutex);
	ib_conn->comp->active_qps--;
	mtx_unlock(&ig.connlist_mutex);
	ISER_ERR("unable to alloc mem or create resource, err %d", ret);

	return (ret);
}

/**
 * based on the resolved device node GUID see if there already allocated
 * device for this device. If there's no such, create one.
 */
static struct iser_device *
iser_device_find_by_ib_device(struct rdma_cm_id *cma_id)
{
	struct iser_device *device;

	sx_xlock(&ig.device_list_mutex);

	list_for_each_entry(device, &ig.device_list, ig_list)
		/* find if there's a match using the node GUID */
		if (device->ib_device->node_guid == cma_id->device->node_guid)
			goto inc_refcnt;

	device = malloc(sizeof *device, M_ISER_VERBS, M_WAITOK | M_ZERO);
	if (device == NULL)
		goto out;

	/* assign this device to the device */
	device->ib_device = cma_id->device;
	/* init the device and link it into ig device list */
	if (iser_create_device_ib_res(device)) {
		free(device, M_ISER_VERBS);
		device = NULL;
		goto out;
	}
	list_add(&device->ig_list, &ig.device_list);

inc_refcnt:
	device->refcount++;
out:
	sx_xunlock(&ig.device_list_mutex);
	return (device);
}

/* if there's no demand for this device, release it */
static void
iser_device_try_release(struct iser_device *device)
{
	sx_xlock(&ig.device_list_mutex);
	device->refcount--;
	ISER_INFO("device %p refcount %d", device, device->refcount);
	if (!device->refcount) {
		iser_free_device_ib_res(device);
		list_del(&device->ig_list);
		free(device, M_ISER_VERBS);
		device = NULL;
	}
	sx_xunlock(&ig.device_list_mutex);
}

/**
 * iser_free_ib_conn_res - release IB related resources
 * @iser_conn: iser connection struct
 * @destroy: indicator if we need to try to release the
 *     iser device and memory regoins pool (only iscsi
 *     shutdown and DEVICE_REMOVAL will use this).
 *
 * This routine is called with the iser state mutex held
 * so the cm_id removal is out of here. It is Safe to
 * be invoked multiple times.
 */
static void
iser_free_ib_conn_res(struct iser_conn *iser_conn,
				  bool destroy)
{
	struct ib_conn *ib_conn = &iser_conn->ib_conn;
	struct iser_device *device = ib_conn->device;

	ISER_INFO("freeing conn %p cma_id %p qp %p",
		  iser_conn, ib_conn->cma_id, ib_conn->qp);

	if (ib_conn->qp != NULL) {
		ib_conn->comp->active_qps--;
		rdma_destroy_qp(ib_conn->cma_id);
		ib_conn->qp = NULL;
	}

	if (destroy) {
		if (iser_conn->login_buf)
			iser_free_login_buf(iser_conn);

		if (iser_conn->rx_descs)
			iser_free_rx_descriptors(iser_conn);

		if (device != NULL) {
			iser_device_try_release(device);
			ib_conn->device = NULL;
		}
	}
}

/**
 * Frees all conn objects
 */
void
iser_conn_release(struct iser_conn *iser_conn)
{
	struct ib_conn *ib_conn = &iser_conn->ib_conn;
	struct iser_conn *curr, *tmp;

	mtx_lock(&ig.connlist_mutex);
	/*
	 * Search for iser connection in global list.
	 * It may not be there in case of failure in connection establishment
	 * stage.
	 */
	list_for_each_entry_safe(curr, tmp, &ig.connlist, conn_list) {
		if (iser_conn == curr) {
			ISER_WARN("found iser_conn %p", iser_conn);
			list_del(&iser_conn->conn_list);
		}
	}
	mtx_unlock(&ig.connlist_mutex);

	/*
	 * In case we never got to bind stage, we still need to
	 * release IB resources (which is safe to call more than once).
	 */
	iser_free_ib_conn_res(iser_conn, true);

	if (ib_conn->cma_id != NULL) {
		rdma_destroy_id(ib_conn->cma_id);
		ib_conn->cma_id = NULL;
	}

}

/**
 * triggers start of the disconnect procedures and wait for them to be done
 * Called with state mutex held
 */
int
iser_conn_terminate(struct iser_conn *iser_conn)
{
	struct ib_conn *ib_conn = &iser_conn->ib_conn;
	struct ib_send_wr *bad_wr;
	int err = 0;

	ISER_INFO("iser_conn %p", iser_conn);

	mtx_lock(&iser_conn->state_mutex);
	iser_conn->state = ISER_CONN_TERMINATING;
	mtx_unlock(&iser_conn->state_mutex);

	if (ib_conn->qp == NULL) {
		/* HOW can this be??? */
		ISER_WARN("qp wasn't created");
		return (err);
	}

	/*
	 * In case we didn't already clean up the cma_id (peer initiated
	 * a disconnection), we need to Cause the CMA to change the QP
	 * state to ERROR.
	 */
	if (ib_conn->cma_id) {
		err = rdma_disconnect(ib_conn->cma_id);
		if (err)
			ISER_ERR("Failed to disconnect, conn: 0x%p err %d",
				iser_conn, err);

		/* post an indication that all flush errors were consumed */
		err = ib_post_send(ib_conn->qp, &ib_conn->beacon, &bad_wr);
		if (err) {
			ISER_ERR("conn %p failed to post beacon", ib_conn);
			return (1);
		}

		ISER_DBG("before cv_wait: %p", iser_conn);
		mtx_lock(&ib_conn->flush_lock);
		cv_wait(&ib_conn->flush_cv, &ib_conn->flush_lock);
		mtx_unlock(&ib_conn->flush_lock);
		ISER_DBG("after cv_wait: %p", iser_conn);
	}

	return (err);
}

static void
iser_connect_error(struct rdma_cm_id *cma_id)
{
	struct iser_conn *iser_conn;

	iser_conn = cma_id->context;

	ISER_ERR("conn %p", iser_conn);

	mtx_lock(&iser_conn->state_mutex);
	iser_conn->state = ISER_CONN_TERMINATING;
	mtx_unlock(&iser_conn->state_mutex);
	cv_signal(&iser_conn->up_cv);
}

static void
iser_addr_handler(struct rdma_cm_id *cma_id)
{
	struct iser_device *device;
	struct iser_conn   *iser_conn;
	struct ib_conn   *ib_conn;
	int    ret;

	iser_conn = cma_id->context;

	ib_conn = &iser_conn->ib_conn;
	device = iser_device_find_by_ib_device(cma_id);
	if (!device) {
		ISER_ERR("conn %p device lookup/creation failed",
			 iser_conn);
		return;
	}

	ib_conn->device = device;

	ret = rdma_resolve_route(cma_id, 1000);
	if (ret) {
		ISER_ERR("conn %p resolve route failed: %d", iser_conn, ret);
		return;
	}
}

static void
iser_route_handler(struct rdma_cm_id *cma_id)
{
	struct rdma_conn_param conn_param;
	int    ret;
	struct iser_cm_hdr req_hdr;
	struct iser_conn *iser_conn = cma_id->context;
	struct ib_conn *ib_conn = &iser_conn->ib_conn;
	struct iser_device *device = ib_conn->device;

	ret = iser_create_ib_conn_res(ib_conn);
	if (ret)
		goto failure;

	memset(&conn_param, 0, sizeof conn_param);
	conn_param.responder_resources = device->dev_attr.max_qp_rd_atom;
	conn_param.retry_count	       = 7;
	conn_param.rnr_retry_count     = 6;
	/*
	 * Initiaotr depth should not be set, but in order to compat
	 * with old targets, we keep this value set.
	 */
	conn_param.initiator_depth     = 1;

	memset(&req_hdr, 0, sizeof(req_hdr));
	req_hdr.flags = (ISER_ZBVA_NOT_SUPPORTED |
			ISER_SEND_W_INV_NOT_SUPPORTED);
	conn_param.private_data		= (void *)&req_hdr;
	conn_param.private_data_len	= sizeof(struct iser_cm_hdr);

	ret = rdma_connect(cma_id, &conn_param);
	if (ret) {
		ISER_ERR("conn %p failure connecting: %d", iser_conn, ret);
		goto failure;
	}

	return;
failure:
	iser_connect_error(cma_id);
}

static void
iser_connected_handler(struct rdma_cm_id *cma_id)
{
	struct iser_conn *iser_conn;
	struct ib_qp_attr attr;
	struct ib_qp_init_attr init_attr;

	iser_conn = cma_id->context;

	(void)ib_query_qp(cma_id->qp, &attr, ~0, &init_attr);

	ISER_INFO("remote qpn:%x my qpn:%x",
		  attr.dest_qp_num, cma_id->qp->qp_num);

	mtx_lock(&iser_conn->state_mutex);
	iser_conn->state = ISER_CONN_UP;
	mtx_unlock(&iser_conn->state_mutex);
	cv_signal(&iser_conn->up_cv);
}

static void
iser_cleanup_handler(struct rdma_cm_id *cma_id, bool destroy)
{
	struct iser_conn *iser_conn = cma_id->context;

	mtx_lock(&iser_conn->state_mutex);
	if (iser_conn->state != ISER_CONN_TERMINATING)
		iser_conn->icl_conn.ic_error(&iser_conn->icl_conn);
	mtx_unlock(&iser_conn->state_mutex);
};

static int
iser_cma_handler(struct rdma_cm_id *cma_id, struct rdma_cm_event *event)
{
	struct iser_conn *iser_conn;
	int ret = 0;

	iser_conn = cma_id->context;
	ISER_INFO("event %d status %d conn %p id %p",
		  event->event, event->status, cma_id->context, cma_id);

	switch (event->event) {
	case RDMA_CM_EVENT_ADDR_RESOLVED:
		iser_addr_handler(cma_id);
		break;
	case RDMA_CM_EVENT_ROUTE_RESOLVED:
		iser_route_handler(cma_id);
		break;
	case RDMA_CM_EVENT_ESTABLISHED:
		iser_connected_handler(cma_id);
		break;
	case RDMA_CM_EVENT_ADDR_ERROR:
	case RDMA_CM_EVENT_ROUTE_ERROR:
	case RDMA_CM_EVENT_CONNECT_ERROR:
	case RDMA_CM_EVENT_UNREACHABLE:
	case RDMA_CM_EVENT_REJECTED:
		iser_connect_error(cma_id);
		break;
	case RDMA_CM_EVENT_DISCONNECTED:
	case RDMA_CM_EVENT_ADDR_CHANGE:
	case RDMA_CM_EVENT_TIMEWAIT_EXIT:
		iser_cleanup_handler(cma_id, false);
		break;
	default:
		ISER_ERR("Unexpected RDMA CM event (%d)", event->event);
		break;
	}

	return (ret);
}

int
iser_conn_connect(struct icl_conn *ic, int domain, int socktype,
		int protocol, struct sockaddr *from_sa, struct sockaddr *to_sa)
{
	struct iser_conn *iser_conn = icl_to_iser_conn(ic);
	struct ib_conn *ib_conn = &iser_conn->ib_conn;
	int err = 0;

	 /* the device is known only --after-- address resolution */
	ib_conn->device = NULL;

	mtx_lock(&iser_conn->state_mutex);
	iser_conn->state = ISER_CONN_PENDING;
	mtx_unlock(&iser_conn->state_mutex);

	ib_conn->beacon.wr_id = ISER_BEACON_WRID;
	ib_conn->beacon.opcode = IB_WR_SEND;

	ib_conn->cma_id = rdma_create_id(iser_cma_handler, (void *)iser_conn,
			RDMA_PS_TCP, IB_QPT_RC);
	if (IS_ERR(ib_conn->cma_id)) {
		err = PTR_ERR(ib_conn->cma_id);
		ISER_ERR("rdma_create_id failed: %d", err);
		goto id_failure;
	}

	err = rdma_resolve_addr(ib_conn->cma_id, from_sa, to_sa, 1000);
	if (err) {
		ISER_ERR("rdma_resolve_addr failed: %d", err);
		goto addr_failure;
	}

	ISER_DBG("before cv_wait: %p", iser_conn);
	mtx_lock(&iser_conn->up_lock);
	cv_wait(&iser_conn->up_cv, &iser_conn->up_lock);
	mtx_unlock(&iser_conn->up_lock);
	ISER_DBG("after cv_wait: %p", iser_conn);

	mtx_lock(&iser_conn->state_mutex);
	if (iser_conn->state != ISER_CONN_UP) {
		err =  -EIO;
		mtx_unlock(&iser_conn->state_mutex);
		goto addr_failure;
	}
	mtx_unlock(&iser_conn->state_mutex);

	err = iser_alloc_login_buf(iser_conn);
	if (err)
		goto addr_failure;

	mtx_lock(&ig.connlist_mutex);
	list_add(&iser_conn->conn_list, &ig.connlist);
	mtx_unlock(&ig.connlist_mutex);

	return (0);

id_failure:
	ib_conn->cma_id = NULL;
addr_failure:
	return (err);
}

int
iser_post_recvl(struct iser_conn *iser_conn)
{
	struct ib_recv_wr rx_wr, *rx_wr_failed;
	struct ib_conn *ib_conn = &iser_conn->ib_conn;
	struct ib_sge	  sge;
	int ib_ret;

	sge.addr   = iser_conn->login_resp_dma;
	sge.length = ISER_RX_LOGIN_SIZE;
	sge.lkey   = ib_conn->device->mr->lkey;

	rx_wr.wr_id   = (uintptr_t)iser_conn->login_resp_buf;
	rx_wr.sg_list = &sge;
	rx_wr.num_sge = 1;
	rx_wr.next    = NULL;

	ib_conn->post_recv_buf_count++;
	ib_ret	= ib_post_recv(ib_conn->qp, &rx_wr, &rx_wr_failed);
	if (ib_ret) {
		ISER_ERR("ib_post_recv failed ret=%d", ib_ret);
		ib_conn->post_recv_buf_count--;
	}

	return (ib_ret);
}

int
iser_post_recvm(struct iser_conn *iser_conn, int count)
{
	struct ib_recv_wr *rx_wr, *rx_wr_failed;
	int i, ib_ret;
	struct ib_conn *ib_conn = &iser_conn->ib_conn;
	unsigned int my_rx_head = iser_conn->rx_desc_head;
	struct iser_rx_desc *rx_desc;

	for (rx_wr = ib_conn->rx_wr, i = 0; i < count; i++, rx_wr++) {
		rx_desc		= &iser_conn->rx_descs[my_rx_head];
		rx_wr->wr_id	= (uintptr_t)rx_desc;
		rx_wr->sg_list	= &rx_desc->rx_sg;
		rx_wr->num_sge	= 1;
		rx_wr->next	= rx_wr + 1;
		my_rx_head = (my_rx_head + 1) % iser_conn->qp_max_recv_dtos;
	}

	rx_wr--;
	rx_wr->next = NULL; /* mark end of work requests list */

	ib_conn->post_recv_buf_count += count;
	ib_ret	= ib_post_recv(ib_conn->qp, ib_conn->rx_wr, &rx_wr_failed);
	if (ib_ret) {
		ISER_ERR("ib_post_recv failed ret=%d", ib_ret);
		ib_conn->post_recv_buf_count -= count;
	} else
		iser_conn->rx_desc_head = my_rx_head;

	return (ib_ret);
}

/**
 * iser_start_send - Initiate a Send DTO operation
 *
 * returns 0 on success, -1 on failure
 */
int iser_post_send(struct ib_conn *ib_conn, struct iser_tx_desc *tx_desc,
		   bool signal)
{
	int		  ib_ret;
	struct ib_send_wr send_wr, *send_wr_failed;

	ib_dma_sync_single_for_device(ib_conn->device->ib_device,
				      tx_desc->dma_addr, ISER_HEADERS_LEN,
				      DMA_TO_DEVICE);

	send_wr.next	   = NULL;
	send_wr.wr_id	   = (uintptr_t)tx_desc;
	send_wr.sg_list	   = tx_desc->tx_sg;
	send_wr.num_sge	   = tx_desc->num_sge;
	send_wr.opcode	   = IB_WR_SEND;
	send_wr.send_flags = signal ? IB_SEND_SIGNALED : 0;

	ib_ret = ib_post_send(ib_conn->qp, &send_wr, &send_wr_failed);
	if (ib_ret)
		ISER_ERR("ib_post_send failed, ret:%d", ib_ret);

	return (ib_ret);
}
