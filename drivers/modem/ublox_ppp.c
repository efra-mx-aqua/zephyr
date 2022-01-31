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

K_SEM_DEFINE(ublox_sem, 0, 1);

#define MDM_URAT_LENGTH          16
#define MDM_UBANDMASKS           2

#define GSM_CMD_SETUP_TIMEOUT K_SECONDS(2)

struct modem_info {
	int mdm_mnoprof;
	int mdm_psm;
	char mdm_urat[MDM_URAT_LENGTH];
	uint64_t mdm_bandmask[MDM_UBANDMASKS];
	int mdm_rssi;
	int mdm_service;
};

static struct modem_info minfo;

/* Handler: +UMNOPROF: <mnoprof> */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_mnoprof)
{
	size_t out_len;
	char buf[16];
	char *prof;

	out_len = net_buf_linearize(buf,
				    sizeof(buf) - 1,
				    data->rx_buf, 0, len);
	buf[out_len] = '\0';
	prof = strchr(buf, ':');
	if (!prof || *(prof + 1) != ' ') {
		minfo.mdm_mnoprof = -1;
		return -EINVAL;
	}
	prof = prof + 2;
	minfo.mdm_mnoprof = atoi(prof);
	LOG_INF("MNO profile: %d", minfo.mdm_mnoprof);

	k_sem_give(&ublox_sem);
	return 0;
}

/* Handler: +CPSMS: <mode>,[...] */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_psm)
{
	size_t out_len;
	char buf[16];
	char *psm;

	out_len = net_buf_linearize(buf,
				    sizeof(buf) - 1,
				    data->rx_buf, 0, len);
	buf[out_len] = '\0';

	psm = strchr(buf, ':');
	if (!psm) {
		return -EINVAL;
	}
	minfo.mdm_psm = *(psm + 1) - '0';
	LOG_INF("PSM mode: %d", minfo.mdm_psm);

	k_sem_give(&ublox_sem);
	return 0;
}

static int gsm_setup_mnoprof(struct modem_context *ctx, struct k_sem *sem)
{
	int ret;
	struct setup_cmd cmds[] = {
		SETUP_CMD("AT+UMNOPROF?", "", on_cmd_atcmdinfo_mnoprof, 0U, ""),
	};

	ret = modem_cmd_handler_setup_cmds_nolock(&ctx->iface,
						  &ctx->cmd_handler,
						  cmds,
						  ARRAY_SIZE(cmds),
						  &ublox_sem,
						  GSM_CMD_SETUP_TIMEOUT);
	if (ret < 0) {
		LOG_ERR("AT+UMNOPROF ret:%d", ret);
		return ret;
	}

	if (minfo.mdm_mnoprof != -1 && minfo.mdm_mnoprof != CONFIG_MODEM_GSM_MNOPROF) {
		/* The wrong MNO profile was set, change it */
		LOG_WRN("Changing MNO profile from %d to %d",
			minfo.mdm_mnoprof, CONFIG_MODEM_GSM_MNOPROF);

		/* Detach from the network */
		ret = modem_cmd_send_nolock(&ctx->iface,
					    &ctx->cmd_handler,
					    NULL, 0,
					    "AT+CFUN=0",
					    sem,
					    K_SECONDS(2));
		if (ret < 0) {
			LOG_ERR("AT+CFUN=0 ret:%d", ret);
		}

		/* Set the profile */
		ret = modem_cmd_send_nolock(&ctx->iface,
					    &ctx->cmd_handler,
					    NULL, 0,
					    "AT+UMNOPROF=" STRINGIFY(CONFIG_MODEM_GSM_MNOPROF),
					    sem,
					    K_SECONDS(2));
		if (ret < 0) {
			LOG_ERR("AT+UMNOPROF ret:%d", ret);
		}

		/* Reboot */
		ret = modem_cmd_send_nolock(&ctx->iface,
					    &ctx->cmd_handler,
					    NULL, 0,
					    "AT+CFUN=15",
					    sem,
					    K_SECONDS(2));
		if (ret < 0) {
			LOG_ERR("AT+CFUN=15 ret:%d", ret);
		}
		k_sleep(K_SECONDS(3));

		return -EAGAIN;
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
		SETUP_CMD_NOHANDLE("ATE0"),
		SETUP_CMD_NOHANDLE("AT+CFUN=0"),
		SETUP_CMD_NOHANDLE("AT+CPSMS=0"),
		SETUP_CMD_NOHANDLE("AT+CFUN=15"),
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
		LOG_WRN("Disabling PSM");
		ret = modem_cmd_handler_setup_cmds_nolock(&ctx->iface,
							  &ctx->cmd_handler,
							  set_cmds,
							  ARRAY_SIZE(set_cmds),
							  sem,
							  GSM_CMD_SETUP_TIMEOUT);
		if (ret < 0) {
			LOG_ERR("Querying PSM ret:%d", ret);
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

	out_len = net_buf_linearize(minfo.mdm_urat,
				    sizeof(minfo.mdm_urat) - 1,
				    data->rx_buf, 0, len);
	minfo.mdm_urat[out_len] = '\0';

	/* Get rid of "+URAT: " */
	char *p = strchr(minfo.mdm_urat, ' ');
	if (p) {
		size_t len = strlen(p + 1);
		memmove(minfo.mdm_urat, p + 1, len + 1);
	}

	LOG_INF("URAT: %s", log_strdup(minfo.mdm_urat));

	k_sem_give(&ublox_sem);
	return 0;
}

static int gsm_setup_urat(struct modem_context *ctx, struct k_sem *sem)
{
	int ret;
	struct setup_cmd query_cmds[] = {
		SETUP_CMD("AT+URAT?", "", on_cmd_atcmdinfo_urat, 0U, ""),
	};
	struct setup_cmd set_cmds[] = {
		SETUP_CMD_NOHANDLE("ATE0"),
		SETUP_CMD_NOHANDLE("AT+CFUN=0"),
		SETUP_CMD_NOHANDLE("AT+URAT=" CONFIG_MODEM_GSM_URAT),
		SETUP_CMD_NOHANDLE("AT+CFUN=15"),
	};

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

	if (strcmp(minfo.mdm_urat, CONFIG_MODEM_GSM_URAT)) {
		LOG_WRN("Setting URAT");
		ret = modem_cmd_handler_setup_cmds_nolock(&ctx->iface,
							  &ctx->cmd_handler,
							  set_cmds,
							  ARRAY_SIZE(set_cmds),
							  sem,
							  GSM_CMD_SETUP_TIMEOUT);
		if (ret < 0) {
			LOG_ERR("Setting URAT ret:%d", ret);
			return ret;
		}

		k_sleep(K_SECONDS(3));

		return -EAGAIN;
	}

	return ret;
}

/* Handler: +UBANDMASK: <rat0>,<mask>,[...] */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_ubandmask)
{
	char buf[40];
	size_t out_len;

	out_len = net_buf_linearize(buf, sizeof(buf) - 1,
				    data->rx_buf, 0, len);
	buf[out_len] = '\0';
	char *p = buf;

	/* Skip over "+UBANDMASK: " */
	if (strchr(buf, ' ')) {
		p = strchr(buf, ' ');
	}
	int i = 0;
	int rat = -1;
	while (p) {
		int v = atoi(p);

		if (i % 2 == 0) {
			rat = v;
		} else if (rat >= 0 && rat < MDM_UBANDMASKS) {
			minfo.mdm_bandmask[rat] = v;
			LOG_INF("UBANDMASK for RAT %d: 0x%x", rat, v);
		}

		p = strchr(p, ',');
		if (p) p++;
		i++;
	}

	k_sem_give(&ublox_sem);
	return 0;
}

static int gsm_setup_ubandmask(struct modem_context *ctx, struct k_sem *sem)
{
	int ret;
	struct setup_cmd query_cmds[] = {
		SETUP_CMD("AT+UBANDMASK?", "", on_cmd_atcmdinfo_ubandmask, 0U, ""),
	};
	struct setup_cmd set_cmds[] = {
		SETUP_CMD_NOHANDLE("ATE0"),
		SETUP_CMD_NOHANDLE("AT+CFUN=0"),
		SETUP_CMD_NOHANDLE("AT+UBANDMASK=0,"
				   STRINGIFY(CONFIG_MODEM_GSM_UBANDMASK_M1)),
		SETUP_CMD_NOHANDLE("AT+UBANDMASK=1,"
				   STRINGIFY(CONFIG_MODEM_GSM_UBANDMASK_NB1)),
		SETUP_CMD_NOHANDLE("AT+CFUN=15"),
	};

	ret = modem_cmd_handler_setup_cmds_nolock(&ctx->iface,
						  &ctx->cmd_handler,
						  query_cmds,
						  ARRAY_SIZE(query_cmds),
						  &ublox_sem,
						  GSM_CMD_SETUP_TIMEOUT);
	if (ret < 0) {
		LOG_ERR("Querying UBANDMASK ret:%d", ret);
		return ret;
	}

	if (minfo.mdm_bandmask[0] != CONFIG_MODEM_GSM_UBANDMASK_M1 ||
	    minfo.mdm_bandmask[1] != CONFIG_MODEM_GSM_UBANDMASK_NB1) {
		LOG_WRN("Setting UBANDMASK");
		ret = modem_cmd_handler_setup_cmds_nolock(&ctx->iface,
							  &ctx->cmd_handler,
							  set_cmds,
							  ARRAY_SIZE(set_cmds),
							  sem,
							  GSM_CMD_SETUP_TIMEOUT);
		k_sleep(K_SECONDS(3));
		if (ret < 0) {
			LOG_ERR("Setting URAT ret:%d", ret);
			return ret;
		}


		return -EAGAIN;
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
			switch (v) {
			default:
			case 0:
				minfo.mdm_rssi = -106;
				break;
			case 1:
				minfo.mdm_rssi = -92;
				break;
			case 2:
				minfo.mdm_rssi = -82;
				break;
			case 3:
				minfo.mdm_rssi = -70;
				break;
			case 4:
				minfo.mdm_rssi = -58;
				break;
			case 5:
				minfo.mdm_rssi = -57;
				break;
			}
			if (v == 5) {
				LOG_INF("RSSI: >=%d dBm", minfo.mdm_rssi);
			} else {
				LOG_INF("RSSI: <%d dBm", minfo.mdm_rssi + 1);
			}
			break;
		case 2:
			LOG_INF("Network service: %d", v);
			minfo.mdm_service = v;
			break;
		}

		p = strchr(p, ',');
		if (p) p++;
		i++;
	}

	k_sem_give(&ublox_sem);
	return 0;
}

/* Handler: +COPS: <mode>[,<format>,<oper>[,<AcT>]]
 */
static char cached_operator[10];
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_cops)
{
	char buf[40];
	size_t out_len;
	char *operator;
	char *ptr;

	out_len = net_buf_linearize(buf, sizeof(buf) - 1,
				    data->rx_buf, 0, len);
	buf[out_len] = '\0';

	/* <oper> should always have quotation marks around it */
	operator = strchr(buf, '"');
	if (!operator) {
		return -EIO;
	}

	/* NULL terminate <oper> if needed */
	ptr = strchr(operator + 1, '"');
	if (!ptr) {
		return -EIO;
	}
	*(ptr + 1) = '\0';

	LOG_WRN("Operator is %s", log_strdup(operator));

	/* Fail-safe against operator format being wrong */
	if (atoi(operator + 1)) {
		strncpy(cached_operator, operator, sizeof(cached_operator));
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

	ctx->data_rssi = minfo.mdm_rssi;

	if (minfo.mdm_service != 1) {
		return -EIO;
	}

	/* If connected, attempt to cache the operator for later use. Note:
	 * don't return -errno on error here because it's on best-effort basis.
	 */

	/* Change operator format to numeric */
	ret = modem_cmd_send_nolock(&ctx->iface,
				    &ctx->cmd_handler,
				    NULL, 0,
				    "AT+COPS=3,2",
				    sem,
				    K_SECONDS(2));
	if (ret) {
		LOG_WRN("AT+COPS=3,2 timeout");
		return 0;
	}

	/* Read out operator and cache it for future connections */
	struct modem_cmd cops_cmd =
		MODEM_CMD("+COPS:", on_cmd_atcmdinfo_cops, 0U, "");
	ret = modem_cmd_send_nolock(&ctx->iface,
				    &ctx->cmd_handler,
				    &cops_cmd, 1,
				    "AT+COPS?",
				    &ublox_sem,
				    K_SECONDS(5));
	if (ret) {
		LOG_WRN("AT+COPS? timeout");
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
	static char manual_cops[1 + sizeof("AT+COPS=1,2,\"12345\"")];

	ret = gsm_setup_mnoprof(ctx, sem);
	if (ret < 0) {
		LOG_WRN("gsm_setup_mnoprof returned %d", ret);
		return ret;
	}

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

	if (!cached_operator[0]) {
		LOG_INF("No cached operator");
		return ret;
	}

	sprintf(manual_cops, "AT+COPS=1,2,%s", cached_operator);
	LOG_INF("Manual operator cmd: %s", log_strdup(manual_cops));

	/* Best effort basis ie don't signal failure if this fails */
	(void)modem_cmd_send_nolock(&ctx->iface,
				    &ctx->cmd_handler,
				    NULL, 0,
				    manual_cops,
				    sem,
				    K_SECONDS(2));
	return ret;
}

static const struct gpio_dt_spec pwr_on_pin =
	GPIO_DT_SPEC_GET(DT_NODELABEL(modem_pwr_ctrl), mdm_power_gpios);
static const struct gpio_dt_spec reset_pin =
	GPIO_DT_SPEC_GET(DT_NODELABEL(modem_pwr_ctrl), mdm_reset_gpios);
static const struct gpio_dt_spec vint_pin =
	GPIO_DT_SPEC_GET(DT_NODELABEL(modem_pwr_ctrl), mdm_vint_gpios);

/*----------------------------------------------------------------------------*/
/* Infrastructure for interrupt-based VINT checking */
/*----------------------------------------------------------------------------*/
K_SEM_DEFINE(vint_sem, 0, 1);
static void vint_handler(const struct device *port, struct gpio_callback *cb,
			 gpio_port_pins_t pins)
{
	k_sem_give(&vint_sem);
}
/*----------------------------------------------------------------------------*/

static struct gpio_callback vint_cb;
static void enable_vint_isr(void)
{
	gpio_init_callback(&vint_cb, vint_handler, BIT(vint_pin.pin));
	gpio_add_callback(vint_pin.port, &vint_cb);

	k_sem_reset(&vint_sem);
	gpio_pin_interrupt_configure_dt(&vint_pin, GPIO_INT_EDGE_BOTH);
}

static void disable_vint_isr(void)
{
	gpio_remove_callback(vint_pin.port, &vint_cb);
	gpio_pin_interrupt_configure_dt(&vint_pin, GPIO_INT_DISABLE);
}

int ublox_sara_r4_pwr_on(void)
{
	int ret = 0;
	if (gpio_pin_get_dt(&reset_pin) == 1) {
		gpio_pin_set_dt(&reset_pin, 0);
		k_sleep(K_SECONDS(3));
	}

	enable_vint_isr();
	gpio_pin_set_dt(&pwr_on_pin, 0);
	k_sleep(K_MSEC(200));
	gpio_pin_set_dt(&pwr_on_pin, 1);
	k_sleep(K_MSEC(200));

	LOG_DBG("Waiting for VINT high...");
	if (k_sem_take(&vint_sem, K_SECONDS(30))) {
		LOG_ERR("VINT timeout!");
		ret = -ETIMEDOUT;
		goto end;
	}

	if (gpio_pin_get_dt(&vint_pin) != 1) {
		LOG_ERR("VINT not high!!");
		ret = -EIO;
		goto end;
	}

	LOG_DBG("modem powered on");
end:
	disable_vint_isr();
	return ret;
}

int ublox_sara_r4_pwr_off(void)
{
	int ret = 0;

	if (gpio_pin_get_dt(&vint_pin) == 0) {
		LOG_DBG("modem already powered off");
		return 0;
	}

	enable_vint_isr();

	/* Toggle power control pin */
	gpio_pin_set_dt(&pwr_on_pin, 0);
	k_sleep(K_MSEC(3000));
	gpio_pin_set_dt(&pwr_on_pin, 1);

	LOG_DBG("Waiting for VINT low...");
	if (k_sem_take(&vint_sem, K_SECONDS(40))) {
		LOG_ERR("VINT timeout!");
		ret = -ETIMEDOUT;
		goto end;
	}

	if (gpio_pin_get_dt(&vint_pin) != 0) {
		LOG_ERR("VINT not low!!");
		ret = -EIO;
		goto end;
	}

	LOG_DBG("Powered off modem");
end:
	disable_vint_isr();
	return ret;
}

int ublox_sara_r4_pwr_off_force(void)
{
	int ret = 0;
	LOG_DBG("MODEM_RESET -> ACTIVE");

	enable_vint_isr();
	gpio_pin_set_dt(&reset_pin, 1);
	if (k_sem_take(&vint_sem, K_SECONDS(10))) {
		LOG_ERR("VINT low timeout!");
		ret = -ETIMEDOUT;
		goto end;
	}

	if (gpio_pin_get_dt(&vint_pin) != 0) {
		LOG_ERR("VINT now low!!!");
		ret = -EIO;
		goto end;
	}

	LOG_DBG("vint is low!");
	k_sleep(K_MSEC(500));
end:
	disable_vint_isr();
	return ret;
}

static int ublox_sara_r4_pwr_init(const struct device *dev)
{
	ARG_UNUSED(dev);

	gpio_pin_configure_dt(&reset_pin, GPIO_OUTPUT | GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&pwr_on_pin, GPIO_OUTPUT);
	gpio_pin_configure_dt(&vint_pin, GPIO_INPUT);

	/* Make sure modem is powered off at boot for maximum determinism */
	if (ublox_sara_r4_pwr_off()) {
		ublox_sara_r4_pwr_off_force();
	}

	return 0;
}

SYS_INIT(ublox_sara_r4_pwr_init, POST_KERNEL, 62);
