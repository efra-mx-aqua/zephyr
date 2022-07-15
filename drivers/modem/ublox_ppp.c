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
#include "drivers/modem/ublox_sara_r4.h"

#define MDM_URAT_SIZE            3
#define MDM_UBANDMASKS_SIZE      2

#define GSM_CMD_SETUP_TIMEOUT K_SECONDS(10)
#define GSM_CMD_CFUNC_TIMEOUT K_SECONDS(180)

enum modem_rat {
	MODEM_RAT_GSM = 0,
	MODEM_RAT_UMTS = 2,
	MODEM_RAT_LTE = 3,
	MODEM_RAT_LTE_CAT_M1 =7,
	MODEM_RAT_NBIOT = 8,
	MODEM_RAT_GPRS = 8,
};

struct modem_info {
	int mdm_psm;
	uint16_t mdm_urat[MDM_URAT_SIZE];
	uint32_t mdm_lte_bandmask[MDM_UBANDMASKS_SIZE];
	uint32_t mdm_nb_bandmask[MDM_UBANDMASKS_SIZE];
	uint32_t mdm_gsm_bandmask[MDM_UBANDMASKS_SIZE];
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

/* Handler: +CPSMS: <mode>,[...] */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_psm)
{
	size_t out_len;
	char buf[32];
	char *psm;
	int ret;

	out_len = net_buf_linearize(buf,
				    sizeof(buf) - 1,
				    data->rx_buf, 0, len);
	buf[out_len] = '\0';

	psm = strchr(buf, ':');
	if (!psm) {
		ret = -EINVAL;
	} else {
		minfo.mdm_psm = *(psm + 1) - '0';
		LOG_INF("PSM mode: %d", minfo.mdm_psm);
	}

	k_sem_give(&ublox_sem);
	return ret;
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

static int gsm_setup_psm(struct modem_context *ctx, struct k_sem *sem)
{
	int ret;
	struct setup_cmd query_cmds[] = {
		SETUP_CMD("AT+CPSMS?", "", on_cmd_atcmdinfo_psm, 0U, ""),
	};
	struct setup_cmd set_cmds[] = {
		SETUP_CMD_NOHANDLE("AT+CPSMS=0"),
	};

	ret = modem_cmd_handler_setup_cmds_nolock(&ctx->iface,
						  &ctx->cmd_handler,
						  query_cmds,
						  ARRAY_SIZE(query_cmds),
						  &ublox_sem,
						  GSM_CMD_SETUP_TIMEOUT);
	if (ret < 0) {
		LOG_ERR("Querying PSM ret:%d", ret);
		return ret;
	}

	if (minfo.mdm_psm == 1) {
		gsm_setup_disconnect(ctx, sem);

		LOG_WRN("Disabling PSM");
		ret = modem_cmd_handler_setup_cmds_nolock(&ctx->iface,
							  &ctx->cmd_handler,
							  set_cmds,
							  ARRAY_SIZE(set_cmds),
							  &ublox_sem,
							  GSM_CMD_SETUP_TIMEOUT);
		if (ret < 0) {
			LOG_ERR("Setting PSM ret:%d", ret);
			return ret;
		}

		k_sleep(K_SECONDS(3));

		return -EAGAIN;
	}

	return ret;
}

/* Handler: +URAT: <rat1>,[...] */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_urat)
{
	size_t out_len;
	char buf[16];

	out_len = net_buf_linearize(buf,
				    sizeof(buf) - 1,
				    data->rx_buf, 0, len);
	buf[out_len] = '\0';
	memset(minfo.mdm_urat, 0, sizeof(minfo.mdm_urat));

	/* Get rid of "+URAT: " */
	char *p = strchr(buf, ' ');
	if (p) {
		size_t len = strlen(p + 1);
		memmove(buf, p + 1, len + 1);
	}
	p = buf;

	int index = 0;

	while (*p) {
		minfo.mdm_urat[index] = unquoted_atoi(p, 10);
		p = strchr(p, ',');
		if (p == NULL) {
			break;
		}
		p++;
		index++;
	}

	__ASSERT_NO_MSG(index <= ARRAY_SIZE(minfo.mdm_urat));

	for (int i = 0; i < index; i++) {
		LOG_INF("URAT[%d]: %u", i, minfo.mdm_urat[i]);
	}

	k_sem_give(&ublox_sem);
	return 0;
}

static int gsm_setup_urat(struct modem_context *ctx, struct k_sem *sem)
{
	int ret;
	bool change = false;
	struct setup_cmd query_cmds[] = {
		SETUP_CMD("AT+URAT?", "", on_cmd_atcmdinfo_urat, 0U, ""),
	};
	char urat_cmd[1 + sizeof("AT+URAT=0,1,2")];

	ret = modem_cmd_handler_setup_cmds_nolock(&ctx->iface,
						  &ctx->cmd_handler,
						  query_cmds,
						  ARRAY_SIZE(query_cmds),
						  &ublox_sem,
						  GSM_CMD_SETUP_TIMEOUT);
	if (ret < 0) {
		LOG_ERR("Querying URAT ret:%d", ret);
		return ret;
	}

#ifdef CONFIG_MODEM_GSM_CONFIGURE_URAT2
	sprintf(urat_cmd, "AT+URAT=%1u,%1u", CONFIG_MODEM_GSM_URAT1,
		CONFIG_MODEM_GSM_URAT2);
	change = change && (minfo.mdm_urat[1] == CONFIG_MODEM_GSM_URAT2);
#elif CONFIG_MODEM_GSM_CONFIGURE_URAT1
	sprintf(urat_cmd, "AT+URAT=%1u", CONFIG_MODEM_GSM_URAT1);
	change = change && (minfo.mdm_urat[0] == CONFIG_MODEM_GSM_URAT1);
#else
	change = false;
#endif

	if (change) {
		gsm_setup_disconnect(ctx, sem);

		LOG_WRN("Setting URAT");
		ret = modem_cmd_send_nolock(&ctx->iface,
					&ctx->cmd_handler,
					NULL, 0,
					urat_cmd,
					&ublox_sem,
					GSM_CMD_SETUP_TIMEOUT);
		if (ret < 0) {
			LOG_ERR("Setting URAT ret:%d", ret);
			return ret;
		}

		k_sleep(K_SECONDS(3));
		ctx->req_reset = 1;
	}

	return 0;
}

/* Handler: +UBANDMASK: <rat0>,<mask>,[...] */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_ubandmask)
{
	char buf[40];
	size_t out_len;
	int ret = 0;
	int params[8];
	int rat_count = 0;
	struct {
		int rat;
		int mask1;
		int mask2;
	} rat_masks[3];

	out_len = net_buf_linearize(buf, sizeof(buf) - 1,
				    data->rx_buf, 0, len);
	buf[out_len] = '\0';
	char *p = buf;

	/* Skip over "+UBANDMASK: " */
	if (strchr(buf, ' ')) {
		p = strchr(buf, ' ');
	}
	int i = 0;
	while (p) {
		int v = atoi(p);

		params[i] = v;
		p = strchr(p, ',');
		if (p) p++;
		i++;
	}

	if (i <= 0) {
		LOG_WRN("Bad format UBANDMASK reponse");
		ret = -EBADMSG;
	} else if(i == 8) {
		rat_masks[0].rat = params[0];
		rat_masks[0].mask1 = params[1];
		rat_masks[0].mask2 = params[2];
		rat_masks[1].rat = params[3];
		rat_masks[1].mask1 = params[4];
		rat_masks[1].mask2 = params[5];
		rat_masks[2].rat = params[6];
		rat_masks[2].mask1 = params[7];
		rat_count = 3;
	} else if(i == 6) {
		rat_masks[0].rat = params[0];
		rat_masks[0].mask1 = params[1];
		rat_masks[1].rat = params[2];
		rat_masks[1].mask1 = params[3];
		rat_masks[2].rat = params[4];
		rat_masks[2].mask1 = params[5];
		rat_count = 3;
	} else if(i == 4) {
		rat_masks[0].rat = params[0];
		rat_masks[0].mask1 = params[1];
		rat_masks[1].rat = params[2];
		rat_masks[1].mask1 = params[3];
		rat_count = 2;
	} else if(i == 2) {
		rat_masks[0].rat = params[0];
		rat_masks[0].mask1 = params[1];
		rat_count = 1;
	}

	__ASSERT_NO_MSG(rat_count <= ARRAY_SIZE(rat_masks));

	for (i = 0; i < rat_count; i++) {
		if (rat_masks[i].rat == 0) {
			minfo.mdm_lte_bandmask[0] = rat_masks[i].mask1;
			minfo.mdm_lte_bandmask[1] = rat_masks[i].mask2;
			LOG_INF("LTE band masks: %d, %d", rat_masks[i].mask1,
				rat_masks[i].mask2);
		} else if (rat_masks[i].rat == 1) {
			minfo.mdm_nb_bandmask[0] = rat_masks[i].mask1;
			minfo.mdm_nb_bandmask[1] = rat_masks[i].mask2;
			LOG_INF("NB band masks: %d, %d", rat_masks[i].mask1,
				rat_masks[i].mask2);
		} else if (rat_masks[i].rat == 3) {
			minfo.mdm_gsm_bandmask[0] = rat_masks[i].mask1;
			LOG_INF("GSM band mask: %d", rat_masks[i].mask1);
		}
	}
	k_sem_give(&ublox_sem);
	return ret;
}

static int gsm_setup_ubandmask(struct modem_context *ctx, struct k_sem *sem)
{
	int ret;
	struct setup_cmd query_cmds[] = {
		SETUP_CMD("AT+UBANDMASK?", "", on_cmd_atcmdinfo_ubandmask, 0U, ""),
	};
	struct setup_cmd set_cmds[] = {
		SETUP_CMD_NOHANDLE("AT+UBANDMASK=0,"
				   STRINGIFY(CONFIG_MODEM_GSM_UBANDMASK_LTE_CAT_M1)),
		SETUP_CMD_NOHANDLE("AT+UBANDMASK=1,"
				   STRINGIFY(CONFIG_MODEM_GSM_UBANDMASK_NB1)),
	};

	ret = modem_cmd_handler_setup_cmds_nolock(&ctx->iface,
						  &ctx->cmd_handler,
						  query_cmds,
						  ARRAY_SIZE(query_cmds),
						  &ublox_sem,
						  GSM_CMD_SETUP_TIMEOUT);
	if (ret < 0) {
		LOG_DBG("Querying UBANDMASK ret:%d", ret);
		return ret;
	}


	/* For LTE Cat M1 */
	if (minfo.mdm_lte_bandmask[0] != CONFIG_MODEM_GSM_UBANDMASK_LTE_CAT_M1 &&
	    (minfo.mdm_urat[0] == MODEM_RAT_LTE_CAT_M1 ||
	     minfo.mdm_urat[1] == MODEM_RAT_LTE_CAT_M1)) {
		LOG_WRN("Setting LTE-Cat M1 bandmask");
		ret = modem_cmd_handler_setup_cmds_nolock(&ctx->iface,
							  &ctx->cmd_handler,
							  &set_cmds[0], 1,
							  &ublox_sem,
							  GSM_CMD_SETUP_TIMEOUT);
		k_sleep(K_SECONDS(3));
		if (ret < 0) {
			LOG_DBG("%s ret:%d", set_cmds[0].send_cmd, ret);
			return ret;
		}

		ctx->req_reset = 1;
	}

	/* For NB-iot */
	if (minfo.mdm_nb_bandmask[0] != CONFIG_MODEM_GSM_UBANDMASK_NB1 &&
	    (minfo.mdm_urat[0] == MODEM_RAT_GPRS ||
	     minfo.mdm_urat[1] == MODEM_RAT_GPRS)) {
		LOG_WRN("Setting NB bandmask");
		ret = modem_cmd_handler_setup_cmds_nolock(&ctx->iface,
							  &ctx->cmd_handler,
							  &set_cmds[1], 1,
							  &ublox_sem,
							  GSM_CMD_SETUP_TIMEOUT);
		k_sleep(K_SECONDS(3));
		if (ret < 0) {
			LOG_DBG("%s ret:%d", set_cmds[1].send_cmd, ret);
			return ret;
		}

		ctx->req_reset = 1;
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

	ret = gsm_setup_psm(ctx, sem);
	if (ret < 0) {
		LOG_WRN("gsm_setup_psm returned %d", ret);
		return ret;
	}

	ret = gsm_setup_urat(ctx, sem);
	if (ret < 0) {
		LOG_WRN("gsm_setup_urat returned %d", ret);
		return ret;
	}

	ret = gsm_setup_ubandmask(ctx, sem);
	if (ret < 0) {
		LOG_WRN("gsm_setup_ubandmask returned %d", ret);
		return ret;
	}

	if (ctx->req_reset) {
		ret = gsm_setup_reset(ctx, sem);
		if (ret == 0) {
			ctx->req_reset = false;
		}
		ret = -EAGAIN;
	}

	return ret;
}
