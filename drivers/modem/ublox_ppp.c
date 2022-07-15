/* © 2020 Endian Technologies AB
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#include <logging/log.h>
LOG_MODULE_DECLARE(modem_gsm, CONFIG_MODEM_LOG_LEVEL);


#include <zephyr.h>
#include <stdlib.h>
#include <stdio.h>
#include <drivers/gpio.h>

#include "modem_cmd_handler.h"
#include "modem_context.h"
#include "drivers/modem/gsm_ppp.h"

#define GSM_CMD_SETUP_TIMEOUT K_SECONDS(10)
#define GSM_CMD_CFUNC_TIMEOUT K_SECONDS(180)

struct modem_info {
	int mdm_signal;
	int mdm_simcard_status;
	int mdm_roaming;
	int mdm_service;
};

static struct modem_info minfo;

K_SEM_DEFINE(ublox_sem, 0, 1);

static int unquoted_atoi(const char *s, int base)
{
	if (*s == '"') {
		s++;
	}

	return strtol(s, NULL, base);
}

static int gsm_setup_disconnect(struct modem_context *ctx, struct k_sem *sem)
{
	int ret;
	struct setup_cmd disconnect_cmds[] = {
		SETUP_CMD_NOHANDLE("AT+CFUN=0"),
	};

	LOG_WRN("Disconnecting");
	ret = modem_cmd_handler_setup_cmds_nolock(&ctx->iface,
						  &ctx->cmd_handler,
						  disconnect_cmds,
						  ARRAY_SIZE(disconnect_cmds),
						  sem,
						  GSM_CMD_SETUP_TIMEOUT);
	if (ret < 0) {
		LOG_ERR("AT+CFUN=0 ret:%d", ret);
		ret = 0;
	}

	return ret;
}

static int gsm_setup_reset(struct modem_context *ctx, struct k_sem *sem)
{
	int ret;
	struct setup_cmd disconnect_cmds[] = {
		SETUP_CMD_NOHANDLE("AT+CFUN=15"),
	};

	LOG_WRN("Modem reset");
	ret = modem_cmd_handler_setup_cmds_nolock(&ctx->iface,
						  &ctx->cmd_handler,
						  disconnect_cmds,
						  ARRAY_SIZE(disconnect_cmds),
						  sem,
						  GSM_CMD_SETUP_TIMEOUT);
	if (ret < 0) {
		LOG_ERR("AT+CFUN=15 ret:%d", ret);
		ret = 0;
	}

	return ret;
}

/* Handler: +CIND: <battchg>,<signal>,<service>,<sounder>,
 *          <message>,<call>,<roam>,<smsfull>,<gprs>,
 *          <callsetup>,<callheld>,<simind>
 */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_cind)
{
	char buf[40];
	size_t out_len;

	out_len = net_buf_linearize(buf, sizeof(buf) - 1,
				    data->rx_buf, 0, len);
	buf[out_len] = '\0';

	char *p = buf;
	int i = 0;
	while (p) {
		int v = atoi(p);

		switch (i) {
		case 1:
			minfo.mdm_signal = v;
			LOG_INF("Signal strength: %d", minfo.mdm_signal);
			break;
		case 2:
			LOG_INF("Network service: %d", v);
			minfo.mdm_service = v;
			break;
		case 6:
			minfo.mdm_roaming = -1;
			if (v == 1) {
				minfo.mdm_roaming = 1;
			} else if(v == 0) {
				minfo.mdm_roaming = 0;
			}
			LOG_INF("Roaming: %d", v);
			break;
		case 11:
			LOG_INF("Simcard status: %d", v);
			minfo.mdm_simcard_status = v;
			break;
		}

		p = strchr(p, ',');
		if (p) p++;
		i++;
	}

	k_sem_give(&ublox_sem);
	return 0;
}

/* Poll the network status. Should return non-negative to indicate
 * that the network is ready to use.
 */
static int gsm_poll_network_status(struct modem_context *ctx, struct k_sem *sem)
{
	/* FIXME: During development, when you tend to be particularily rough
	 * with it, modem becomes unresponsive in this particular phase a lot.
	 * It's not recoverable apart from power cycling modem. This needs a
	 * solution.
	 */

	int ret;
	struct modem_cmd cind_cmd =
		MODEM_CMD("+CIND:", on_cmd_atcmdinfo_cind, 0U, "");

	ret = modem_cmd_send_nolock(&ctx->iface,
				    &ctx->cmd_handler,
				    &cind_cmd, 1,
				    "AT+CIND?",
				    &ublox_sem,
				    K_SECONDS(5));
	if (ret < 0) {
		LOG_ERR("Querying CIND: %d", ret);
		return ret;
	}

	if (minfo.mdm_service != 1) {
		return -EIO;
	}

	return 0;
}

int gsm_ppp_pre_connect_hook(struct modem_context *ctx, struct k_sem *sem)
{
	k_sleep(K_SECONDS(1));
	return gsm_poll_network_status(ctx, sem);
}

int gsm_ppp_setup_hook(struct modem_context *ctx, struct k_sem *sem)
{
	int ret;

	if (ctx->req_reset) {
		ret = gsm_setup_reset(ctx, sem);
		if (ret == 0) {
			ctx->req_reset = false;
		}
		ret = -EAGAIN;
	}

	return ret;
}
