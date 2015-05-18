/*-
 * Copyright (c) 2012 The FreeBSD Foundation
 * All rights reserved.
 *
 * This software was developed by Edward Tomasz Napierala under sponsorship
 * from the FreeBSD Foundation.
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
 *
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/types.h>
#include <sys/ioctl.h>
#include <stdbool.h>
#include <string.h>
#include <netinet/in.h>

#include "iscsid.h"
#include "iscsi_proto.h"

static struct pdu *
text_receive(struct connection *conn)
{
	struct pdu *response;
	struct iscsi_bhs_text_response *bhstr;

	response = pdu_new(conn);
	pdu_receive(response);
	if (response->pdu_bhs->bhs_opcode != ISCSI_BHS_OPCODE_TEXT_RESPONSE)
		log_errx(1, "protocol error: received invalid opcode 0x%x",
		    response->pdu_bhs->bhs_opcode);
	bhstr = (struct iscsi_bhs_text_response *)response->pdu_bhs;
#if 0
	if ((bhstr->bhstr_flags & BHSTR_FLAGS_FINAL) == 0)
		log_errx(1, "received Text PDU without the \"F\" flag");
#endif
	/*
	 * XXX: Implement the C flag some day.
	 */
	if ((bhstr->bhstr_flags & BHSTR_FLAGS_CONTINUE) != 0)
		log_errx(1, "received Text PDU with unsupported \"C\" flag");
	if (ntohl(bhstr->bhstr_statsn) != conn->conn_statsn + 1) {
		log_errx(1, "received Text PDU with wrong StatSN: "
		    "is %u, should be %u", ntohl(bhstr->bhstr_statsn),
		    conn->conn_statsn + 1);
	}
	conn->conn_statsn = ntohl(bhstr->bhstr_statsn);

	return (response);
}

static struct pdu *
text_new_request(struct connection *conn)
{
	struct pdu *request;
	struct iscsi_bhs_text_request *bhstr;

	request = pdu_new(conn);
	bhstr = (struct iscsi_bhs_text_request *)request->pdu_bhs;
	bhstr->bhstr_opcode = ISCSI_BHS_OPCODE_TEXT_REQUEST |
	    ISCSI_BHS_OPCODE_IMMEDIATE;
	bhstr->bhstr_flags = BHSTR_FLAGS_FINAL;
	bhstr->bhstr_initiator_task_tag = 0;
	bhstr->bhstr_target_transfer_tag = 0xffffffff;

	bhstr->bhstr_initiator_task_tag = 0; /* XXX */
	bhstr->bhstr_cmdsn = 0; /* XXX */
	bhstr->bhstr_expstatsn = htonl(conn->conn_statsn + 1);

	return (request);
}

static void
kernel_add(const struct connection *conn, const char *target)
{
	struct iscsi_session_add isa;
	int error;

	memset(&isa, 0, sizeof(isa));
	memcpy(&isa.isa_conf, &conn->conn_conf, sizeof(isa.isa_conf));
	strlcpy(isa.isa_conf.isc_target, target,
	    sizeof(isa.isa_conf.isc_target));
	isa.isa_conf.isc_discovery = 0;
	error = ioctl(conn->conn_iscsi_fd, ISCSISADD, &isa);
	if (error != 0)
		log_warn("failed to add %s: ISCSISADD", target);
}

void
kernel_remove(const struct connection *conn)
{
	struct iscsi_session_remove isr;
	int error;

	memset(&isr, 0, sizeof(isr));
	isr.isr_session_id = conn->conn_session_id;
	error = ioctl(conn->conn_iscsi_fd, ISCSISREMOVE, &isr);
	if (error != 0)
		log_warn("ISCSISREMOVE");
}

void
discovery(struct connection *conn)
{
	struct pdu *request, *response;
	struct keys *request_keys, *response_keys;
	int i;

	log_debugx("beginning discovery session");
	request = text_new_request(conn);
	request_keys = keys_new();
	keys_add(request_keys, "SendTargets", "All");
	keys_save(request_keys, request);
	keys_delete(request_keys);
	request_keys = NULL;
	pdu_send(request);
	pdu_delete(request);
	request = NULL;

	log_debugx("waiting for Text Response");
	response = text_receive(conn);
	response_keys = keys_new();
	keys_load(response_keys, response);
	for (i = 0; i < KEYS_MAX; i++) {
		if (response_keys->keys_names[i] == NULL)
			break;

		if (strcmp(response_keys->keys_names[i], "TargetName") != 0)
			continue;

		log_debugx("adding target %s", response_keys->keys_values[i]);
		/*
		 * XXX: Validate the target name?
		 */
		kernel_add(conn, response_keys->keys_values[i]);
	}
	keys_delete(response_keys);
	pdu_delete(response);

	log_debugx("discovery session done");
}
