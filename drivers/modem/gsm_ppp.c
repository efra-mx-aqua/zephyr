/*
 * Copyright (c) 2020 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT zephyr_gsm_ppp

#include <logging/log.h>
LOG_MODULE_REGISTER(modem_gsm, CONFIG_MODEM_LOG_LEVEL);

#include <stdlib.h>
#include <kernel.h>
#include <device.h>
#include <sys/ring_buffer.h>
#include <sys/util.h>
#include <net/ppp.h>
#include <drivers/gsm_ppp.h>
#include <drivers/uart.h>
#include <drivers/console/uart_mux.h>

#include "modem_context.h"
#include "modem_iface_uart.h"
#include "modem_cmd_handler.h"
#include "../console/gsm_mux.h"

#include <stdio.h>

#define GSM_UART_NODE                   DT_INST_BUS(0)
#define GSM_CMD_READ_BUF                128
#define GSM_CMD_AT_TIMEOUT              K_SECONDS(2)
#define GSM_CMD_SETUP_TIMEOUT           K_SECONDS(6)
/* GSM_CMD_LOCK_TIMEOUT should be longer than GSM_CMD_AT_TIMEOUT & GSM_CMD_SETUP_TIMEOUT,
 * otherwise the gsm_ppp_stop might fail to lock tx.
 */
#define GSM_CMD_LOCK_TIMEOUT            K_SECONDS(10)
#define GSM_RECV_MAX_BUF                30
#define GSM_RECV_BUF_SIZE               128
#define GSM_ATTACH_RETRY_DELAY_MSEC     1000
#define GSM_RETRY_DELAY                 K_SECONDS(1)
#define GSM_REGISTER_DELAY_MSEC         1000
#define GSM_RETRY_DELAY                 K_SECONDS(1)
#define GSM_REGISTER_DELAY_MSEC         1000
#define GSM_DETECTION_RETRIES           50
#define GSM_AUTO_COPS_DELAY_MSEC     	5000
#define GSM_RETRY_DELAY			K_SECONDS(1)

#define GSM_RSSI_RETRY_DELAY_MSEC       2000
#define GSM_RSSI_RETRIES                10
#define GSM_RSSI_INVALID                -1000
#define GSM_RSSI_MAXVAL         	-51

#ifdef CONFIG_NET_PPP
#define GSM_RX_RINGBUF_SIZE             (PPP_MRU * 3)
#else
#define GSM_RX_RINGBUF_SIZE             1500
#endif

/* Modem network registration state */
enum network_state {
	GSM_NET_INIT = -1,
	GSM_NET_NOT_REGISTERED,
	GSM_NET_HOME_NETWORK,
	GSM_NET_SEARCHING,
	GSM_NET_REGISTRATION_DENIED,
	GSM_NET_UNKNOWN,
	GSM_NET_ROAMING,
	GSM_NET_SMS_ONLY,
	GSM_NET_SMS_ONLY_ROAMING,
	GSM_NET_EMERGENCY_ONLY,
	GSM_NET_CSFB_NOT_PREFERED_HOME,
	GSM_NET_CSFB_NOT_PREFERED_ROAMING,
};

/* EPS registration status */
enum eps_network_state {
	GSM_EPS_NET_INIT = -1,
	GSM_EPS_NET_NOT_REGISTERED,
	GSM_EPS_NET_HOME_NETWORK,
	GSM_EPS_NET_SEARCHING,
	GSM_EPS_NET_REGISTRATION_DENIED,
	GSM_EPS_NET_UNKNOWN,
	GSM_EPS_NET_ROAMING,
	GSM_EPS_NET_UNDEFINED1,
	GSM_EPS_NET_UNDEFINED2,
	GSM_EPS_NET_EMERGENCY_ONLY,
};

/* GPRS registration status */
enum gprs_network_state {
	GSM_GPRS_NET_INIT = -1,
	GSM_GPRS_NET_NOT_REGISTERED = 0,
	GSM_GPRS_NET_HOME_NETWORK,
	GSM_GPRS_NET_SEARCHING,
	GSM_GPRS_NET_REGISTRATION_DENIED,
	GSM_GPRS_NET_UNKNOWN,
	GSM_GPRS_NET_ROAMING,
	GSM_GPRS_NET_UNDEFINED1,
	GSM_GPRS_NET_UNDEFINED2,
	GSM_GPRS_NET_EMERGENCY_ONLY,
	GSM_GPRS_NET__ENUMCOUNT,
};

/* During the modem setup, we first create DLCI control channel and then
 * PPP and AT channels. Currently the modem does not create possible GNSS
 * channel.
 */
enum setup_state {
	STATE_INIT = 0,
	STATE_CONTROL_CHANNEL = 0,
	STATE_PPP_CHANNEL,
	STATE_AT_CHANNEL,
	STATE_DONE
};

static struct gsm_modem {
	struct k_mutex lock;
	const struct device *dev;
	struct modem_context context;

	struct modem_cmd_handler_data cmd_handler_data;
	uint8_t cmd_match_buf[GSM_CMD_READ_BUF];
	struct k_sem sem_response;
	struct k_sem sem_if_down;

	struct modem_iface_uart_data gsm_data;
	struct k_work_delayable gsm_configure_work;
	char gsm_rx_rb_buf[GSM_RX_RINGBUF_SIZE];
	k_tid_t gsm_rx_tid;

	uint8_t *ppp_recv_buf;
	size_t ppp_recv_buf_len;

	enum setup_state state;
	const struct device *ppp_dev;
	const struct device *at_dev;
	const struct device *control_dev;

	struct net_if *iface;

	struct k_thread rx_thread;
	struct k_work_q workq;
	struct k_work_delayable rssi_work_handle;
	struct gsm_ppp_modem_info minfo;

	int register_retries;
	int rssi_retries;
	int attach_retries;
	int auto_cops_retries;
	bool attached : 1;
	bool running : 1;
	bool modem_info_queried : 1;
	bool req_reset : 1;

	void *user_data;

	gsm_modem_power_cb modem_on_cb;
	gsm_modem_power_cb modem_off_cb;
	struct net_mgmt_event_callback gsm_mgmt_cb;
} gsm;

NET_BUF_POOL_DEFINE(gsm_recv_pool, GSM_RECV_MAX_BUF, GSM_RECV_BUF_SIZE, 0, NULL);
K_KERNEL_STACK_DEFINE(gsm_rx_stack, CONFIG_MODEM_GSM_RX_STACK_SIZE);
K_KERNEL_STACK_DEFINE(gsm_workq_stack, CONFIG_MODEM_GSM_WORKQ_STACK_SIZE);

static inline void gsm_ppp_lock(struct gsm_modem *gsm)
{
	(void)k_mutex_lock(&gsm->lock, K_FOREVER);
}

static inline void gsm_ppp_unlock(struct gsm_modem *gsm)
{
	(void)k_mutex_unlock(&gsm->lock);
}

static inline int gsm_work_reschedule(struct k_work_delayable *dwork, k_timeout_t delay)
{
	return k_work_reschedule_for_queue(&gsm.workq, dwork, delay);
}

#define ATOI(s_, value_, desc_) modem_atoi(s_, value_, desc_, __func__)

/**
 * @brief  Convert string to long integer, but handle errors
 *
 * @param  s: string with representation of integer number
 * @param  err_value: on error return this value instead
 * @param  desc: name the string being converted
 * @param  func: function where this is called (typically __func__)
 *
 * @retval return integer conversion on success, or err_value on error
 */
static int modem_atoi(const char *s, const int err_value,
				const char *desc, const char *func)
{
	int ret;
	char *endptr;

	ret = (int)strtol(s, &endptr, 10);
	if (!endptr || *endptr != '\0') {
		LOG_ERR("bad %s '%s' in %s", log_strdup(s),
			 log_strdup(desc), log_strdup(func));
		return err_value;
	}

	return ret;
}

static void gsm_rx(struct gsm_modem *gsm)
{
	LOG_DBG("starting");

	while (true) {
		(void)k_sem_take(&gsm->gsm_data.rx_sem, K_FOREVER);

		/* The handler will listen AT channel */
		gsm->context.cmd_handler.process(&gsm->context.cmd_handler,
						 &gsm->context.iface);
	}
}

MODEM_CMD_DEFINE(gsm_cmd_ok)
{
	modem_cmd_handler_set_error(data, 0);
	LOG_DBG("ok");
	k_sem_give(&gsm.sem_response);
	return 0;
}

MODEM_CMD_DEFINE(gsm_cmd_error)
{
	modem_cmd_handler_set_error(data, -EINVAL);
	LOG_DBG("error");
	k_sem_give(&gsm.sem_response);
	return 0;
}

static const struct modem_cmd response_cmds[] = {
	MODEM_CMD("OK", gsm_cmd_ok, 0U, ""),
	MODEM_CMD("ERROR", gsm_cmd_error, 0U, ""),
	MODEM_CMD("CONNECT", gsm_cmd_ok, 0U, ""),
};

static int unquoted_atoi(const char *s, int base)
{
	if (*s == '"') {
		s++;
	}

	return strtol(s, NULL, base);
}

/*
 * Handler: +COPS: <mode>[0],<format>[1],<oper>[2]
 */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_cops)
{
	uint16_t operator;
	if (argc > 0) {
		if (unquoted_atoi(argv[0], 10) == 0) {
			gsm.context.is_automatic_oper = true;
		} else {
			gsm.context.is_automatic_oper = false;
		}
	}
	if (argc > 2) {
		operator = unquoted_atoi(argv[2], 10);
		LOG_INF("operator: %u", operator);
		gsm.context.data_operator = operator;


#if defined(CONFIG_MODEM_CACHE_OPERATOR)
		/* Fail-safe against operator format being wrong */
		if (operator) {
			gsm.context.data_cached_operator = operator;
		}
#endif
	}

	return 0;
}

/*
 * Provide modem info if modem shell is enabled. This can be shown with
 * "modem list" shell command.
 */

/* Handler: <manufacturer> */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_manufacturer)
{
	size_t out_len;

	out_len = net_buf_linearize(gsm.minfo.mdm_manufacturer,
				    sizeof(gsm.minfo.mdm_manufacturer) - 1,
				    data->rx_buf, 0, len);
	gsm.minfo.mdm_manufacturer[out_len] = '\0';
	LOG_INF("Manufacturer: %s", log_strdup(gsm.minfo.mdm_manufacturer));

	return 0;
}

/* Handler: <model> */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_model)
{
	size_t out_len;

	out_len = net_buf_linearize(gsm.minfo.mdm_model,
				    sizeof(gsm.minfo.mdm_model) - 1,
				    data->rx_buf, 0, len);
	gsm.minfo.mdm_model[out_len] = '\0';
	LOG_INF("Model: %s", log_strdup(gsm.minfo.mdm_model));

	return 0;
}

/* Handler: <rev> */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_revision)
{
	size_t out_len;

	out_len = net_buf_linearize(gsm.minfo.mdm_revision,
				    sizeof(gsm.minfo.mdm_revision) - 1,
				    data->rx_buf, 0, len);
	gsm.minfo.mdm_revision[out_len] = '\0';
	LOG_INF("Revision: %s", log_strdup(gsm.minfo.mdm_revision));

	return 0;
}

/* Handler: <IMEI> */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_imei)
{
	size_t out_len;

	out_len = net_buf_linearize(gsm.minfo.mdm_imei, sizeof(gsm.minfo.mdm_imei) - 1,
				    data->rx_buf, 0, len);
	gsm.minfo.mdm_imei[out_len] = '\0';
	LOG_INF("IMEI: %s", log_strdup(gsm.minfo.mdm_imei));

	return 0;
}

#if defined(CONFIG_MODEM_SIM_NUMBERS)
/* Handler: <IMSI> */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_imsi)
{
	size_t out_len;

	out_len = net_buf_linearize(gsm.minfo.mdm_imsi, sizeof(gsm.minfo.mdm_imsi) - 1,
				    data->rx_buf, 0, len);
	gsm.minfo.mdm_imsi[out_len] = '\0';
	LOG_INF("IMSI: %s", log_strdup(gsm.minfo.mdm_imsi));

	return 0;
}

/* Handler: <ICCID> */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_iccid)
{
	size_t out_len;

	out_len = net_buf_linearize(gsm.minfo.mdm_iccid, sizeof(gsm.minfo.mdm_iccid) - 1,
				    data->rx_buf, 0, len);
	gsm.minfo.mdm_iccid[out_len] = '\0';
	if (gsm.minfo.mdm_iccid[0] == '+') {
		/* Seen on U-blox SARA: "+CCID: nnnnnnnnnnnnnnnnnnnn".
		 * Skip over the +CCID bit, which other modems omit.
		 */
		char *p = strchr(gsm.minfo.mdm_iccid, ' ');

		if (p) {
			size_t len = strlen(p+1);

			memmove(gsm.minfo.mdm_iccid, p+1, len+1);
		}
	}
	LOG_INF("ICCID: %s", log_strdup(gsm.minfo.mdm_iccid));

	return 0;
}
#endif /* CONFIG_MODEM_SIM_NUMBERS */

static const char *net_reg_state_string[] = {
	"Not registered",
	"Registered to home network",
	"Searching ...",
	"Network registration DENIED!",
	"Unknown",
	"Registered to network, roaming",
	"SMS only",
	"SMS only roaming",
	"Emergency only",
};

/*
 * Handler: +CREG: <n>[0],<stat>[1],<tac>[2],<ci>[3],<AcT>[4]
 *	    +CREG: <stat>[0],<tac>[1],<ci>[2],<AcT>[3]  (URC)
 */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_creg)
{
	int arg_idx;
	int argc_min;

	/* Check if it is a URC */
	if (argc == 1 || argc == 4) {
		arg_idx = 0;
		argc_min = 1;
	} else {
		arg_idx = 1;
		argc_min = 2;
	}

	if (argc >= argc_min) {
		int net_reg = unquoted_atoi(argv[arg_idx++], 16);
		gsm.context.data_gsm_reg = net_reg;
		if (net_reg < ARRAY_SIZE(net_reg_state_string)) {
			LOG_INF("GSM Network: %s", net_reg_state_string[net_reg]);
		}
	}
	if (argc > argc_min) {
		gsm.context.data_lac = unquoted_atoi(argv[arg_idx++], 16);
		gsm.context.data_cellid = unquoted_atoi(argv[arg_idx++], 16);
		LOG_INF("lac: %u, cellid: %u",
			gsm.context.data_lac,
			gsm.context.data_cellid);

	}

	return 0;
}

/*
 * Handler: +CEREG: <n>[0],<stat>[1],<tac>[2],<ci>[3],<AcT>[4]
 *	    +CEREG: <stat>[0],<tac>[1],<ci>[2],<AcT>[3]  (URC)
 */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_cereg)
{
	int arg_idx;
	int argc_min;

	/* Check if it is a URC */
	if (argc == 1 || argc == 4) {
		arg_idx = 0;
		argc_min = 1;
	} else {
		arg_idx = 1;
		argc_min = 2;
	}
	if (argc >= argc_min) {
		int eps_reg = unquoted_atoi(argv[arg_idx++], 16);
		gsm.context.data_eps_reg = eps_reg;
		if (eps_reg < ARRAY_SIZE(net_reg_state_string)) {
			LOG_INF("EPS Network: %s", net_reg_state_string[eps_reg]);
		}
	}
	if (argc > argc_min) {
		gsm.context.data_lac = unquoted_atoi(argv[arg_idx++], 16);
		gsm.context.data_cellid = unquoted_atoi(argv[arg_idx++], 16);
		LOG_INF("lac: %u, cellid: %u",
			gsm.context.data_lac,
			gsm.context.data_cellid);
	}

	return 0;
}

/*
 * Handler: +CGREG: <n>[0],<stat>[1],<lac>[2],<ci>[3],<AcT>[4],<rac>[5]
 *          +CGREG: <stat>[0],<lac>[1],<ci>[2],<AcT>[3] (UICR)
 */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_cgreg)
{
	int arg_idx;
	int argc_min;

	/* Check if it is a URC */
	if (argc == 1 || argc == 4) {
		arg_idx = 0;
		argc_min = 1;
	} else {
		arg_idx = 1;
		argc_min = 2;
	}

	if (argc >= argc_min) {
		int gprs_reg = unquoted_atoi(argv[arg_idx++], 16);
		gsm.context.data_gprs_reg = gprs_reg;
		if (gprs_reg < ARRAY_SIZE(net_reg_state_string)) {
			LOG_INF("GPRS Network: %s", net_reg_state_string[gprs_reg]);
		}
	}
	if (argc > argc_min) {
		gsm.context.data_lac = unquoted_atoi(argv[arg_idx++], 16);
		gsm.context.data_cellid = unquoted_atoi(argv[arg_idx++], 16);
		LOG_INF("lac: %u, cellid: %u",
			gsm.context.data_lac,
			gsm.context.data_cellid);
	}

	return 0;
}

static const struct setup_cmd query_cellinfo_cmds[] = {
	SETUP_CMD_ARGS_MAX("AT+CREG?", "", on_cmd_atcmdinfo_creg, 2U, 5U, ","),
	SETUP_CMD_ARGS_MAX("AT+CEREG?", "", on_cmd_atcmdinfo_cereg, 2U, 5U, ","),
	SETUP_CMD_ARGS_MAX("AT+CGREG?", "", on_cmd_atcmdinfo_cgreg, 2U, 6U, ","),
	SETUP_CMD_ARGS_MAX("AT+COPS?", "", on_cmd_atcmdinfo_cops, 1U, 3U, ","),
};

static int gsm_query_cellinfo(struct gsm_modem *gsm)
{
	int ret;

	ret = modem_cmd_handler_setup_cmds_nolock(&gsm->context.iface,
						  &gsm->context.cmd_handler,
						  query_cellinfo_cmds,
						  ARRAY_SIZE(query_cellinfo_cmds),
						  &gsm->sem_response,
						  GSM_CMD_SETUP_TIMEOUT);
	if (ret < 0) {
		LOG_WRN("modem query for cell info returned %d", ret);
	}

	return ret;
}

#if defined(CONFIG_MODEM_GSM_ENABLE_CESQ_RSSI)
/*
 * Handler: +CESQ: <rxlev>[0],<ber>[1],<rscp>[2],<ecn0>[3],<rsrq>[4],<rsrp>[5]
 */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_rssi_cesq)
{
	int rsrp, rscp, rxlev;

	rsrp = ATOI(argv[5], 0, "rsrp");
	rscp = ATOI(argv[2], 0, "rscp");
	rxlev = ATOI(argv[0], 0, "rxlev");

	if (rsrp >= 0 && rsrp <= 97) {
		gsm.minfo.mdm_rsrp = -140 + (rsrp - 1);
		LOG_INF("RSRP: %d", gsm.minfo.mdm_rsrp);
	} else {
		gsm.minfo.mdm_rsrp = GSM_RSSI_INVALID;
		LOG_INF("RSRP not known");
	}
	if (rscp >= 0 && rscp <= 96) {
		gsm.minfo.mdm_rscp = -120 + (rscp - 1);
		LOG_INF("RSCP: %d", gsm.minfo.mdm_rscp);
	} else {
		gsm.minfo.mdm_rscp = GSM_RSSI_INVALID;
		LOG_INF("RSCP not known");
	}
	if (rxlev >= 0 && rxlev <= 63) {
		gsm.minfo.mdm_rssi = -110 + (rxlev - 1);
		LOG_INF("RSSI: %d", gsm.minfo.mdm_rssi);
	} else if (!IS_ENABLED(CONFIG_MODEM_GSM_ENABLE_CSQ_RSSI)) {
		gsm.minfo.mdm_rssi = GSM_RSSI_INVALID;
		LOG_INF("RSSI not known");
	} else {
		/* preserve the RSSI */
	}

	return 0;
}
#endif
#if defined(CONFIG_MODEM_GSM_ENABLE_CSQ_RSSI)
/* Handler: +CSQ: <signal_power>[0],<qual>[1] */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_rssi_csq)
{
	/* Expected response is "+CSQ: <signal_power>,<qual>" */
	if (argc) {
		int rssi = atoi(argv[0]);

		/* If  CESQ returned a value, give a priority to that*/
		if (rssi >= 0 && rssi <= 31 &&
		    (!IS_ENABLED(CONFIG_MODEM_GSM_ENABLE_CESQ_RSSI) ||
		     GSM_RSSI_INVALID == gsm.minfo.mdm_rssi)) {
			rssi = -113 + (rssi * 2);
		} else if (!IS_ENABLED(CONFIG_MODEM_GSM_ENABLE_CESQ_RSSI)){
			gsm.minfo.mdm_rssi = rssi;
			rssi = GSM_RSSI_INVALID;
		} else {
			/* preserve the RSSI */
		}

		gsm.minfo.mdm_rssi = rssi;
		LOG_INF("RSSI(CSQ): %d", rssi);
	}

	k_sem_give(&gsm.sem_response);

	return 0;
}
#endif

#if defined(CONFIG_MODEM_GSM_ENABLE_CESQ_RSSI)
static const struct modem_cmd read_rssi_cesq_cmd =
	MODEM_CMD("+CESQ:", on_cmd_atcmdinfo_rssi_cesq, 6U, ",");
#endif
#if defined(CONFIG_MODEM_GSM_ENABLE_CSQ_RSSI)
static const struct modem_cmd read_rssi_csq_cmd =
	MODEM_CMD("+CSQ:", on_cmd_atcmdinfo_rssi_csq, 2U, ",");
#endif

static const struct setup_cmd setup_modem_info_cmds[] = {
	/* query modem info */
	SETUP_CMD("AT+CGMI", "", on_cmd_atcmdinfo_manufacturer, 0U, ""),
	SETUP_CMD("AT+CGMM", "", on_cmd_atcmdinfo_model, 0U, ""),
	SETUP_CMD("AT+CGMR", "", on_cmd_atcmdinfo_revision, 0U, ""),
	SETUP_CMD("AT+CGSN", "", on_cmd_atcmdinfo_imei, 0U, ""),
#if defined(CONFIG_MODEM_SIM_NUMBERS)
	SETUP_CMD("AT+CIMI", "", on_cmd_atcmdinfo_imsi, 0U, ""),
	SETUP_CMD("AT+CCID", "", on_cmd_atcmdinfo_iccid, 0U, ""),
#endif
};

static const struct setup_cmd setup_cmds[] = {
	/* no echo */
	SETUP_CMD_NOHANDLE("ATE0"),
	/* hang up */
	SETUP_CMD_NOHANDLE("ATH"),
	/* extender errors in numeric form */
	SETUP_CMD_NOHANDLE("AT+CMEE=1"),

	/* disable unsolicited network registration codes */
	SETUP_CMD_NOHANDLE("AT+CREG=0"),
	SETUP_CMD_NOHANDLE("AT+CEREG=0"),
	SETUP_CMD_NOHANDLE("AT+CGREG=0"),

	/* read the operator as numeric */
	SETUP_CMD_NOHANDLE("AT+COPS=3,2"),
};

MODEM_CMD_DEFINE(on_cmd_atcmdinfo_attached)
{
	int error = -EAGAIN;

	/* Expected response is "+CGATT: 0|1" so simply look for '1' */
	if (argc && atoi(argv[0]) == 1) {
		error = 0;
		LOG_INF("Attached to packet service!");
	}

	modem_cmd_handler_set_error(data, error);
	k_sem_give(&gsm.sem_response);

	return 0;
}


static const struct modem_cmd check_attached_cmd =
	MODEM_CMD("+CGATT:", on_cmd_atcmdinfo_attached, 1U, ",");

static const struct setup_cmd connect_cmds[] = {
	/* connect to network */
	SETUP_CMD_NOHANDLE("ATD*99#"),
};

static int gsm_query_modem_info(struct gsm_modem *gsm)
{
	int ret;

	if (gsm->modem_info_queried) {
		return 0;
	}

	ret =  modem_cmd_handler_setup_cmds_nolock(&gsm->context.iface,
						  &gsm->context.cmd_handler,
						  setup_modem_info_cmds,
						  ARRAY_SIZE(setup_modem_info_cmds),
						  &gsm->sem_response,
						  GSM_CMD_SETUP_TIMEOUT);

	if (ret < 0) {
		return ret;
	}

	gsm->modem_info_queried = true;

	return 0;
}

static int gsm_setup_mccmno(struct gsm_modem *gsm)
{
	int ret = 0;
	uint16_t operator = 0;
	static char manual_cops[1 + sizeof("AT+COPS=1,2,\"12345\"")];

#if defined(CONFIG_MODEM_CACHE_OPERATOR)
	if (!gsm->context.data_cached_operator) {
		LOG_INF("No cached operator");
	}
	operator = (uint16_t)gsm->context.data_cached_operator;
#endif
	if (CONFIG_MODEM_GSM_MANUAL_MCCMNO[0]) {
		operator = (uint16_t)strtol(CONFIG_MODEM_GSM_MANUAL_MCCMNO, NULL, 10);
	}

	/* If the operator is already known, we explicitly set the operator */
	if (operator > 0) {
		/* use manual MCC/MNO entry */
		LOG_INF("Connecting to operator: %d", operator);
		sprintf(manual_cops, "AT+COPS=1,2,%5u", operator);
		ret = modem_cmd_send_nolock(&gsm->context.iface,
					    &gsm->context.cmd_handler,
					    NULL, 0,
					    manual_cops,
					    &gsm->sem_response,
					    GSM_CMD_AT_TIMEOUT);
	} else {

		ret = gsm_query_cellinfo(gsm);
		if (ret < 0) {
			return ret;
		}

		/* If the modem is configured to explicit operator,
		 * we do send the command AT+COPS= 0,0.
		*/
		if (!gsm->context.is_automatic_oper) {
			/* register operator automatically */
			ret = modem_cmd_send_nolock(&gsm->context.iface,
						    &gsm->context.cmd_handler,
						    NULL, 0, "AT+COPS=0,0",
						    &gsm->sem_response,
						    K_SECONDS(30));
		}
	}

	if (ret < 0) {
		LOG_ERR("AT+COPS ret:%d", ret);
	}

	return ret;
}

#ifdef CONFIG_NET_PPP
static struct net_if *ppp_net_if(void)
{
	return net_if_get_first_by_type(&NET_L2_GET_NAME(PPP));
}

static void set_ppp_carrier_on(struct gsm_modem *gsm)
{
	static const struct ppp_api *api;
	const struct device *ppp_dev = device_get_binding(CONFIG_NET_PPP_DRV_NAME);
	struct net_if *iface = gsm->iface;
	int ret;

	if (!ppp_dev) {
		LOG_ERR("Cannot find PPP %s!", CONFIG_NET_PPP_DRV_NAME);
		return;
	}

	if (!api) {
		api = (const struct ppp_api *)ppp_dev->api;

		/* For the first call, we want to call ppp_start()... */
		ret = api->start(ppp_dev);
		if (ret) {
			LOG_ERR("ppp start returned %d", ret);
		}
	} else {
		/* ...but subsequent calls should be to ppp_enable() */
		ret = net_if_l2(iface)->enable(iface, true);
		if (ret) {
			LOG_ERR("ppp l2 enable returned %d", ret);
		}
	}
}
#endif

static void query_rssi(struct gsm_modem *gsm, bool lock)
{
	int ret;

#if defined(CONFIG_MODEM_GSM_ENABLE_CESQ_RSSI)
	ret = modem_cmd_send_ext(&gsm->context.iface, &gsm->context.cmd_handler,
				 &read_rssi_cesq_cmd, 1,
				 "AT+CESQ", &gsm->sem_response, GSM_CMD_SETUP_TIMEOUT,
				 lock ? 0 : MODEM_NO_TX_LOCK);
	if (ret < 0 && !IS_ENABLED(CONFIG_MODEM_GSM_ENABLE_CSQ_RSSI)) {
		LOG_DBG("No answer to RSSI(CESQ) readout, %s", "ignoring...");
	}
#endif
#if defined(CONFIG_MODEM_GSM_ENABLE_CSQ_RSSI)
	ret = modem_cmd_send_ext(&gsm->context.iface, &gsm->context.cmd_handler,
				 &read_rssi_csq_cmd, 1,
				 "AT+CSQ", &gsm->sem_response, GSM_CMD_SETUP_TIMEOUT,
				 lock ? 0 : MODEM_NO_TX_LOCK);
	if (ret < 0 && !IS_ENABLED(CONFIG_MODEM_GSM_ENABLE_CESQ_RSSI)) {
		LOG_DBG("No answer to RSSI(CSQ) readout, %s", "ignoring...");
	}
#endif

#if defined(CONFIG_MODEM_GSM_ENABLE_CESQ_RSSI) && \
    defined(CONFIG_MODEM_GSM_ENABLE_CSQ_RSSI)
	if (ret < 0) {
		LOG_DBG("No answer to RSSI readout, %s", "ignoring...");
	}
#endif

}

static inline void query_rssi_lock(struct gsm_modem *gsm)
{
	query_rssi(gsm, true);
}

static inline void query_rssi_nolock(struct gsm_modem *gsm)
{
	query_rssi(gsm, false);
}

static void rssi_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct gsm_modem *gsm = CONTAINER_OF(dwork, struct gsm_modem, rssi_work_handle);

	gsm_ppp_lock(gsm);
	query_rssi_lock(gsm);

#if defined(CONFIG_MODEM_CELL_INFO)
	(void)gsm_query_cellinfo(gsm);
#endif
	(void)gsm_work_reschedule(&gsm->rssi_work_handle,
				  K_SECONDS(CONFIG_MODEM_GSM_RSSI_POLLING_PERIOD));
	gsm_ppp_unlock(gsm);
}

static inline bool gsm_is_gsm_net_registered(struct gsm_modem *gsm)
{
	if ((gsm->context.data_gsm_reg == GSM_NET_ROAMING) ||
	    (gsm->context.data_gsm_reg == GSM_NET_HOME_NETWORK)) {
		return true;
	}

	return false;
}

static inline bool gsm_is_eps_net_registered(struct gsm_modem *gsm)
{
	if ((gsm->context.data_eps_reg == GSM_EPS_NET_ROAMING) ||
	    (gsm->context.data_eps_reg == GSM_EPS_NET_HOME_NETWORK)) {
		return true;
	}

	return false;
}

static inline bool gsm_is_gprs_net_registered(struct gsm_modem *gsm)
{
	if ((gsm->context.data_gprs_reg == GSM_GPRS_NET_ROAMING) ||
	    (gsm->context.data_gprs_reg == GSM_GPRS_NET_HOME_NETWORK)) {
		return true;
	}

	return false;
}

static inline bool gsm_is_registered(struct gsm_modem *gsm)
{
	return gsm_is_gsm_net_registered(gsm) ||
	       gsm_is_eps_net_registered(gsm) ||
	       gsm_is_gprs_net_registered(gsm);
}

static int gsm_setup_disconnect(struct gsm_modem *gsm)
{
	int ret;
	struct setup_cmd cmds[] = {
		SETUP_CMD_NOHANDLE("AT+CFUN=0"),
	};

	LOG_WRN("Disconnecting");
	ret = modem_cmd_handler_setup_cmds_nolock(&gsm->context.iface,
					&gsm->context.cmd_handler,
					&cmds[0],
					ARRAY_SIZE(cmds),
					&gsm->sem_response,
					GSM_CMD_SETUP_TIMEOUT);
	k_sleep(K_SECONDS(1));

	if (ret < 0) {
		LOG_DBG("AT+CFUN=0 ret:%d", ret);
		ret = 0;
	}

	return ret;
}

static int gsm_setup_reset(struct gsm_modem *gsm)
{
	int ret;
	struct setup_cmd cmds[] = {
		SETUP_CMD_NOHANDLE("AT+CFUN=15"),
	};

	LOG_WRN("Modem reset");
	ret = modem_cmd_handler_setup_cmds_nolock(&gsm->context.iface,
					&gsm->context.cmd_handler,
					&cmds[0],
					ARRAY_SIZE(cmds),
					&gsm->sem_response,
					GSM_CMD_SETUP_TIMEOUT);
	if (ret < 0) {
		LOG_DBG("AT+CFUN=15 ret:%d", ret);
		ret = 0;
	}

	return ret;
}

/* Handler: +UMNOPROF: <mnoprof>[,..] */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_mnoprof)
{
	if (argc > 0) {
		gsm.context.data_operator_profile = atoi(argv[0]);
		LOG_INF("MNO profile: %d", gsm.context.data_operator_profile);
	}

	k_sem_give(&gsm.sem_response);

	return 0;
}

#if defined(CONFIG_MODEM_GSM_MNOPROF)
static int gsm_setup_mnoprof(struct gsm_modem *gsm)
{
	int ret;
	struct setup_cmd set_cmd = SETUP_CMD_NOHANDLE("AT+UMNOPROF="
				   STRINGIFY(CONFIG_MODEM_GSM_MNOPROF));
	static const struct modem_cmd cmd =
		MODEM_CMD_ARGS_MAX("+UMNOPROF:", on_cmd_atcmdinfo_mnoprof, 1U, 4U, ",");

	ret = modem_cmd_send_nolock(&gsm->context.iface,
				    &gsm->context.cmd_handler,
				    &cmd, 1,
				    "AT+UMNOPROF?",
				    &gsm->sem_response,
				    GSM_CMD_SETUP_TIMEOUT);
	if (ret < 0) {
		LOG_DBG("AT+UMNOPROF? ret:%d", ret);
		return ret;
	}

	if (gsm->context.data_operator_profile != -1 &&
	    gsm->context.data_operator_profile != CONFIG_MODEM_GSM_MNOPROF) {
		gsm_setup_disconnect(gsm);

		/* The wrong MNO profile was set, change it */
		LOG_WRN("Changing MNO profile from %d to %d",
			gsm->context.data_operator_profile,
			CONFIG_MODEM_GSM_MNOPROF);

		/* Set the profile */
		ret = modem_cmd_handler_setup_cmds_nolock(&gsm->context.iface,
							  &gsm->context.cmd_handler,
							  &set_cmd, 1,
							  &gsm->sem_response,
							  GSM_CMD_SETUP_TIMEOUT);
		if (ret < 0) {
			LOG_DBG("%s ret:%d", set_cmd.send_cmd, ret);
		}

		/* Reboot */
		gsm_setup_reset(gsm);

		return -EAGAIN;
	}

	return ret;
}
#endif

/* Handler: +CPSMS: <mode>,[...] */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_psm)
{
	if (argc > 0) {
		gsm.context.data_psm = atoi(argv[0]);

		LOG_INF("PSM mode: %d", gsm.context.data_psm);
	}

	k_sem_give(&gsm.sem_response);
	return 0;
}

static int gsm_setup_psm(struct gsm_modem *gsm)
{
	int psm = 0;
	int ret;
	static const struct modem_cmd cmd =
		MODEM_CMD_ARGS_MAX("+CPSMS:", on_cmd_atcmdinfo_psm, 1U, 7U, ",");
#ifdef CONFIG_MODEM_GSM_PSM
	struct setup_cmd set_cmd = SETUP_CMD_NOHANDLE("AT+CPSMS=1");
	psm = 1;
#else
	struct setup_cmd set_cmd = SETUP_CMD_NOHANDLE("AT+CPSMS=0");
#endif

	ret = modem_cmd_send_nolock(&gsm->context.iface,
				    &gsm->context.cmd_handler,
				    &cmd, 1,
				    "AT+CPSMS?",
				    &gsm->sem_response,
				    GSM_CMD_SETUP_TIMEOUT);
	if (ret < 0) {
		LOG_ERR("Querying PSM ret:%d", ret);
		return ret;
	}

	if (gsm->context.data_psm != psm) {
		gsm_setup_disconnect(gsm);

		LOG_WRN("Setting PSM: %d", psm);
		ret = modem_cmd_handler_setup_cmds_nolock(&gsm->context.iface,
							  &gsm->context.cmd_handler,
							  &set_cmd, 1,
							  &gsm->sem_response,
							  GSM_CMD_SETUP_TIMEOUT);
		if (ret < 0) {
			LOG_DBG("%s ret:%d", log_strdup(set_cmd.send_cmd), ret);
			return ret;
		}
		k_sleep(K_SECONDS(3));

		return -EAGAIN;
	}

	return ret;
}

static int gsm_modem_setup(struct gsm_modem *gsm)
{
	int ret;

#if defined(CONFIG_MODEM_GSM_MNOPROF)
	ret = gsm_setup_mnoprof(gsm);
	if (ret < 0) {
		LOG_WRN("gsm_setup_mnoprof returned %d", ret);
		return ret;
	}
#endif

	ret = gsm_setup_psm(gsm);
	if (ret < 0) {
		LOG_WRN("gsm_setup_psm returned %d", ret);
		return ret;
	}

	if (gsm->req_reset) {
		ret = gsm_setup_reset(gsm);
		if (ret == 0) {
			gsm->req_reset = false;
		}
		ret = -EAGAIN;
	}

	return ret;
}

static void gsm_finalize_connection(struct k_work *work)
{
	int ret = 0;
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct gsm_modem *gsm = CONTAINER_OF(dwork, struct gsm_modem, gsm_configure_work);

	gsm_ppp_lock(gsm);

	/* If already attached, jump right to RSSI readout */
	if (gsm->attached) {
		goto attached;
	}

	/* If attach check failed, we should not redo every setup step */
	if (gsm->attach_retries) {
		goto attaching;
	}

	/* If waiting for the mccmno, we should not redo every setup step */
	if (gsm->auto_cops_retries) {
		goto auto_cops;
	}

	/* If modem is searching for network, we should skip the setup step */
	if (gsm->register_retries) {
		goto registering;
	}

	if (IS_ENABLED(CONFIG_GSM_MUX)) {
		ret = modem_cmd_send_nolock(&gsm->context.iface,
					    &gsm->context.cmd_handler,
					    &response_cmds[0],
					    ARRAY_SIZE(response_cmds),
					    "AT", &gsm->sem_response,
					    GSM_CMD_AT_TIMEOUT);
		if (ret < 0) {
			LOG_ERR("%s returned %d, %s", "AT", ret, "retrying...");
			(void)gsm_work_reschedule(&gsm->gsm_configure_work, GSM_RETRY_DELAY);
			goto unlock;
		}
	}

	if (IS_ENABLED(CONFIG_MODEM_GSM_FACTORY_RESET_AT_BOOT)) {
		(void)modem_cmd_send_nolock(&gsm->context.iface,
					    &gsm->context.cmd_handler,
					    &response_cmds[0],
					    ARRAY_SIZE(response_cmds),
					    "AT&F", &gsm->sem_response,
					    GSM_CMD_AT_TIMEOUT);
		k_sleep(K_SECONDS(1));
	}

	ret = modem_cmd_handler_setup_cmds_nolock(&gsm->context.iface,
						  &gsm->context.cmd_handler,
						  setup_cmds,
						  ARRAY_SIZE(setup_cmds),
						  &gsm->sem_response,
						  GSM_CMD_SETUP_TIMEOUT);
	if (ret < 0) {
		LOG_DBG("%s returned %d, %s", "setup_cmds", ret, "retrying...");
		(void)gsm_work_reschedule(&gsm->gsm_configure_work, GSM_RETRY_DELAY);
		goto unlock;
	}

	ret = gsm_query_modem_info(gsm);
	if (ret < 0) {
		LOG_DBG("Unable to query modem information %d", ret);
		(void)gsm_work_reschedule(&gsm->gsm_configure_work, GSM_RETRY_DELAY);
		goto unlock;
	}

	ret = gsm_modem_setup(gsm);
	if (ret < 0) {
		LOG_ERR("%s returned %d, %s", "gsm_modem_setup", ret, "retrying...");
		(void)gsm_work_reschedule(&gsm->gsm_configure_work, GSM_RETRY_DELAY);
		goto unlock;
	}

auto_cops:
	if (!gsm->auto_cops_retries) {
		gsm->auto_cops_retries = gsm->context.auto_cops_max_retries;
	} else {
		gsm->auto_cops_retries--;
	}
	ret = gsm_setup_mccmno(gsm);
	if (ret < 0) {
		LOG_ERR("%s returned %d, %s", "gsm_setup_mccmno", ret, "retrying...");

		(void)gsm_work_reschedule(&gsm->gsm_configure_work, GSM_RETRY_DELAY);
		goto unlock;
	}

	gsm->auto_cops_retries = 0;

	/* create PDP context */
	ret = modem_cmd_send_nolock(&gsm->context.iface,
				    &gsm->context.cmd_handler,
				    &response_cmds[0],
				    ARRAY_SIZE(response_cmds),
				    "AT+CGDCONT=1,\"IP\",\"" CONFIG_MODEM_GSM_APN "\"",
				    &gsm->sem_response,
				    GSM_CMD_AT_TIMEOUT);
	if (ret < 0) {
		LOG_DBG("Couldn't create PDP context (error %d), %s", ret,
			"retrying...");
	}

registering:
	/* Wait for cell tower registration */
	ret = gsm_query_cellinfo(gsm);
	if ((ret < 0) || !gsm_is_registered(gsm)) {
		if (!gsm->register_retries) {
			gsm->register_retries = CONFIG_MODEM_GSM_REGISTER_TIMEOUT *
				MSEC_PER_SEC / GSM_REGISTER_DELAY_MSEC;
		} else {
			gsm->register_retries--;
		}

		(void)gsm_work_reschedule(&gsm->gsm_configure_work,
					  K_MSEC(GSM_REGISTER_DELAY_MSEC));
		goto unlock;
	}

	gsm->register_retries = 0;

attaching:
	/* Don't initialize PPP until we're attached to packet service */
	ret = modem_cmd_send_nolock(&gsm->context.iface,
				    &gsm->context.cmd_handler,
				    &check_attached_cmd, 1,
				    "AT+CGATT?",
				    &gsm->sem_response,
				    GSM_CMD_SETUP_TIMEOUT);
	if (ret < 0) {
		/*
		 * attach_retries not set        -> trigger N attach retries
		 * attach_retries set            -> decrement and retry
		 * attach_retries set, becomes 0 -> trigger full retry
		 */
		if (!gsm->attach_retries) {
			gsm->attach_retries = gsm->context.attach_max_retries;
		} else {
			gsm->attach_retries--;
		}

		LOG_DBG("Not attached, %s", "retrying...");

		(void)gsm_work_reschedule(&gsm->gsm_configure_work,
					K_MSEC(GSM_ATTACH_RETRY_DELAY_MSEC));
		goto unlock;
	}

	/* Attached, clear retry counter */
	gsm->attached = true;
	gsm->attach_retries = 0;

	LOG_DBG("modem attach returned %d, %s", ret, "read RSSI");
	gsm->rssi_retries = GSM_RSSI_RETRIES;

 attached:

	if (!IS_ENABLED(CONFIG_GSM_MUX)) {
		/* Read connection quality (RSSI) before PPP carrier is ON */
		query_rssi_nolock(gsm);

		if (!(gsm->minfo.mdm_rssi && gsm->minfo.mdm_rssi != GSM_RSSI_INVALID &&
			gsm->minfo.mdm_rssi < GSM_RSSI_MAXVAL)) {

			LOG_DBG("Not valid RSSI, %s", "retrying...");
			if (gsm->rssi_retries-- > 0) {
				(void)gsm_work_reschedule(&gsm->gsm_configure_work,
							K_MSEC(GSM_RSSI_RETRY_DELAY_MSEC));
				goto unlock;
			}
		}
#if defined(CONFIG_MODEM_CELL_INFO)
		(void)gsm_query_cellinfo(gsm);
#endif
	}

	LOG_DBG("modem setup returned %d, %s", ret, "enable PPP");
	LOG_DBG("modem RSSI: %d, %s", *gsm->context.data_rssi, "enable PPP");

	ret = modem_cmd_handler_setup_cmds_nolock(&gsm->context.iface,
						  &gsm->context.cmd_handler,
						  connect_cmds,
						  ARRAY_SIZE(connect_cmds),
						  &gsm->sem_response,
						  GSM_CMD_SETUP_TIMEOUT);
	if (ret < 0) {
		LOG_DBG("%s returned %d, %s", "connect_cmds", ret, "retrying...");
		(void)gsm_work_reschedule(&gsm->gsm_configure_work, GSM_RETRY_DELAY);
		goto unlock;
	}

#ifdef CONFIG_NET_PPP
	set_ppp_carrier_on(gsm);
#endif

	if (IS_ENABLED(CONFIG_GSM_MUX)) {
		/* Re-use the original iface for AT channel */
		ret = modem_iface_uart_init_dev(&gsm->context.iface,
						gsm->at_dev);
		if (ret < 0) {
			LOG_DBG("iface %suart error %d", "AT ", ret);
		} else {
			/* Do a test and try to send AT command to modem */
			ret = modem_cmd_send_nolock(
				&gsm->context.iface,
				&gsm->context.cmd_handler,
				&response_cmds[0],
				ARRAY_SIZE(response_cmds),
				"AT", &gsm->sem_response,
				GSM_CMD_AT_TIMEOUT);
			if (ret < 0) {
				LOG_WRN("%s returned %d, %s", "AT", ret, "iface failed");
			} else {
				LOG_INF("AT channel %d connected to %s",
					DLCI_AT, gsm->at_dev->name);
			}
		}
		modem_cmd_handler_tx_unlock(&gsm->context.cmd_handler);
		(void)gsm_work_reschedule(&gsm->rssi_work_handle,
					  K_SECONDS(CONFIG_MODEM_GSM_RSSI_POLLING_PERIOD));
	}

unlock:
	gsm_ppp_unlock(gsm);
}

static int mux_enable(struct gsm_modem *gsm)
{
	int ret;

	/* Turn on muxing */
	if (IS_ENABLED(CONFIG_MODEM_GSM_SIMCOM)) {
		ret = modem_cmd_send_nolock(
			&gsm->context.iface,
			&gsm->context.cmd_handler,
			&response_cmds[0],
			ARRAY_SIZE(response_cmds),
#if defined(SIMCOM_LTE)
			/* FIXME */
			/* Some SIMCOM modems can set the channels */
			/* Control channel always at DLCI 0 */
			"AT+CMUXSRVPORT=0,0;"
			/* PPP should be at DLCI 1 */
			"+CMUXSRVPORT=" STRINGIFY(DLCI_PPP) ",1;"
			/* AT should be at DLCI 2 */
			"+CMUXSRVPORT=" STRINGIFY(DLCI_AT) ",1;"
#else
			"AT"
#endif
			"+CMUX=0,0,5,"
			STRINGIFY(CONFIG_GSM_MUX_MRU_DEFAULT_LEN),
			&gsm->sem_response,
			GSM_CMD_AT_TIMEOUT);
	} else if (IS_ENABLED(CONFIG_MODEM_GSM_QUECTEL)) {
		ret = modem_cmd_send_nolock(&gsm->context.iface,
				    &gsm->context.cmd_handler,
				    &response_cmds[0],
				    ARRAY_SIZE(response_cmds),
				    "AT+CMUX=0,0,5,"
				    STRINGIFY(CONFIG_GSM_MUX_MRU_DEFAULT_LEN),
				    &gsm->sem_response,
				    GSM_CMD_AT_TIMEOUT);

		/* Arbitrary delay for Quectel modems to initialize the CMUX,
		 * without this the AT cmd will fail.
		 */
		k_sleep(K_SECONDS(1));
	} else {
		/* Generic GSM modem */
		ret = modem_cmd_send_nolock(&gsm->context.iface,
				     &gsm->context.cmd_handler,
				     &response_cmds[0],
				     ARRAY_SIZE(response_cmds),
				     "AT+CMUX=0", &gsm->sem_response,
				     GSM_CMD_AT_TIMEOUT);
	}

	if (ret < 0) {
		LOG_ERR("AT+CMUX ret:%d", ret);
	}

	return ret;
}

static void mux_setup_next(struct gsm_modem *gsm)
{
	(void)gsm_work_reschedule(&gsm->gsm_configure_work, K_MSEC(1));
}

static void mux_attach_cb(const struct device *mux, int dlci_address,
			  bool connected, void *user_data)
{
	LOG_DBG("DLCI %d to %s %s", dlci_address, mux->name,
		connected ? "connected" : "disconnected");

	if (connected) {
		uart_irq_rx_enable(mux);
		uart_irq_tx_enable(mux);
	}

	mux_setup_next(user_data);
}

static int mux_attach(const struct device *mux, const struct device *uart,
		      int dlci_address, void *user_data)
{
	int ret = uart_mux_attach(mux, uart, dlci_address, mux_attach_cb,
				  user_data);
	if (ret < 0) {
		LOG_ERR("Cannot attach DLCI %d (%s) to %s (%d)", dlci_address,
			mux->name, uart->name, ret);
		return ret;
	}

	return 0;
}

static void mux_setup(struct k_work *work)
{
	struct gsm_modem *gsm = CONTAINER_OF(work, struct gsm_modem,
					     gsm_configure_work);
	const struct device *uart = DEVICE_DT_GET(GSM_UART_NODE);
	int ret;

	gsm_ppp_lock(gsm);

	/* We need to call this to reactivate mux ISR. Note: This is only called
	 * after re-initing gsm_ppp.
	 */
	if (IS_ENABLED(CONFIG_GSM_MUX) &&
	    gsm->ppp_dev && gsm->state == STATE_CONTROL_CHANNEL) {
		uart_mux_enable(gsm->ppp_dev);
	}

	switch (gsm->state) {
	case STATE_CONTROL_CHANNEL:
		/* Get UART device. There is one dev / DLCI */
		if (gsm->control_dev == NULL) {
			gsm->control_dev = uart_mux_alloc();
			if (gsm->control_dev == NULL) {
				LOG_DBG("Cannot get UART mux for %s channel",
					"control");
				goto fail;
			}
		}

		ret = mux_attach(gsm->control_dev, uart, DLCI_CONTROL, gsm);
		if (ret < 0) {
			goto fail;
		}

		gsm->state = STATE_PPP_CHANNEL;

		break;

	case STATE_PPP_CHANNEL:
		if (gsm->ppp_dev == NULL) {
			gsm->ppp_dev = uart_mux_alloc();
			if (gsm->ppp_dev == NULL) {
				LOG_DBG("Cannot get UART mux for %s channel",
					"PPP");
				goto fail;
			}
		}

		ret = mux_attach(gsm->ppp_dev, uart, DLCI_PPP, gsm);
		if (ret < 0) {
			goto fail;
		}

		gsm->state = STATE_AT_CHANNEL;

		break;

	case STATE_AT_CHANNEL:
		if (gsm->at_dev == NULL) {
			gsm->at_dev = uart_mux_alloc();
			if (gsm->at_dev == NULL) {
				LOG_DBG("Cannot get UART mux for %s channel",
					"AT");
				goto fail;
			}
		}

		ret = mux_attach(gsm->at_dev, uart, DLCI_AT, gsm);
		if (ret < 0) {
			goto fail;
		}

		gsm->state = STATE_DONE;

		break;

	case STATE_DONE:
		/* At least the SIMCOM modem expects that the Internet
		 * connection is created in PPP channel. We will need
		 * to attach the AT channel to context iface after the
		 * PPP connection is established in order to give AT commands
		 * to the modem.
		 */
		ret = modem_iface_uart_init_dev(&gsm->context.iface,
						gsm->ppp_dev);
		if (ret < 0) {
			LOG_DBG("iface %suart error %d", "PPP ", ret);
			goto fail;
		}

		LOG_INF("PPP channel %d connected to %s",
			DLCI_PPP, gsm->ppp_dev->name);

		k_work_init_delayable(&gsm->gsm_configure_work, gsm_finalize_connection);
		(void)gsm_work_reschedule(&gsm->gsm_configure_work, K_NO_WAIT);
		break;
	}

	goto unlock;
fail:
	gsm->state = STATE_INIT;
unlock:
	gsm_ppp_unlock(gsm);
}

static void gsm_configure(struct k_work *work)
{
	struct gsm_modem *gsm = CONTAINER_OF(work, struct gsm_modem,
					     gsm_configure_work);
	int ret = -1;

	gsm_ppp_lock(gsm);

	LOG_DBG("Starting modem %p configuration", gsm);

	if (gsm->modem_on_cb) {
		gsm->modem_on_cb(gsm->dev, gsm->user_data);
	}

	ret = modem_cmd_send_nolock(&gsm->context.iface,
				    &gsm->context.cmd_handler,
				    &response_cmds[0],
				    ARRAY_SIZE(response_cmds),
				    "AT", &gsm->sem_response,
				    GSM_CMD_AT_TIMEOUT);
	if (ret < 0) {
		LOG_DBG("modem not ready %d", ret);
		goto reschedule;
	}

	if (IS_ENABLED(CONFIG_GSM_MUX)) {
		if (mux_enable(gsm)) {
			LOG_DBG("GSM muxing %s", "disabled");
			goto reschedule;
		}

		LOG_DBG("GSM muxing %s", "enabled");

		gsm->state = STATE_INIT;

		k_work_init_delayable(&gsm->gsm_configure_work, mux_setup);
	} else {
		k_work_init_delayable(&gsm->gsm_configure_work,
				      gsm_finalize_connection);
	}

reschedule:
	(void)gsm_work_reschedule(&gsm->gsm_configure_work, K_NO_WAIT);
	gsm_ppp_unlock(gsm);
}

void gsm_ppp_start(const struct device *dev)
{
	int ret;
	struct gsm_modem *gsm = dev->data;

	if (gsm->running) {
		return;
	}

	gsm_ppp_lock(gsm);

	/* Re-init underlying UART comms */
	ret = modem_iface_uart_init_dev(&gsm->context.iface, DEVICE_DT_GET(GSM_UART_NODE));
	if (ret) {
		LOG_ERR("modem_iface_uart_init returned %d", ret);
		goto unlock;
	}

	LOG_INF("GSM PPP start");

	k_work_init_delayable(&gsm->gsm_configure_work, gsm_configure);
	(void)gsm_work_reschedule(&gsm->gsm_configure_work, K_NO_WAIT);

#if defined(CONFIG_GSM_MUX)
	k_work_init_delayable(&gsm->rssi_work_handle, rssi_handler);
#endif
	gsm->running = true;
unlock:
	gsm_ppp_unlock(gsm);
}

void gsm_ppp_stop(const struct device *dev)
{
	struct gsm_modem *gsm = dev->data;
	struct net_if *iface = gsm->iface;
	struct k_work_sync work_sync;

	if (!gsm->running) {
		return;
	}

	(void)k_work_cancel_delayable_sync(&gsm->gsm_configure_work, &work_sync);
	if (IS_ENABLED(CONFIG_GSM_MUX)) {
		(void)k_work_cancel_delayable_sync(&gsm->rssi_work_handle, &work_sync);
	}

	gsm_ppp_lock(gsm);

	/* wait for the interface to be properly down */
	if (net_if_is_up(iface)) {
		net_if_l2(iface)->enable(iface, false);
		(void)k_sem_take(&gsm->sem_if_down, K_FOREVER);
	}
	if (IS_ENABLED(CONFIG_GSM_MUX)) {
		/* Lower mux_enabled flag to trigger re-sending AT+CMUX etc */

		if (gsm->ppp_dev) {
			uart_mux_disable(gsm->ppp_dev);
		}
	}

	if (modem_cmd_handler_tx_lock(&gsm->context.cmd_handler, GSM_CMD_LOCK_TIMEOUT)) {
		LOG_WRN("Failed locking modem cmds!");
	}

	if (gsm->modem_off_cb) {
		gsm->modem_off_cb(gsm->dev, gsm->user_data);
	}

	gsm->running = false;
	gsm->attached = false;
	gsm->context.data_gsm_reg = GSM_NET_INIT;
	gsm->context.data_eps_reg = GSM_EPS_NET_INIT;
	gsm->context.data_gprs_reg = GSM_GPRS_NET_INIT;
	gsm_ppp_unlock(gsm);
}

void gsm_ppp_register_modem_power_callback(const struct device *dev,
					   gsm_modem_power_cb modem_on,
					   gsm_modem_power_cb modem_off,
					   void *user_data)
{
	struct gsm_modem *gsm = dev->data;

	gsm_ppp_lock(gsm);

	gsm->modem_on_cb = modem_on;
	gsm->modem_off_cb = modem_off;

	gsm->user_data = user_data;
	gsm_ppp_unlock(gsm);
}

bool gsm_ppp_is_running(const struct device *dev)
{
	struct gsm_modem *gsm = dev->data;

	return gsm->running;

}

int gsm_ppp_detect(const struct device *dev)
{
	struct gsm_modem *gsm = dev->data;
	int ret = -1;
	int counter = 0;

	/* Re-init underlying UART comms */
	ret = modem_iface_uart_init_dev(&gsm->context.iface,
					DEVICE_DT_GET(GSM_UART_NODE));
	if (ret) {
		LOG_ERR("modem_iface_uart_init returned %d", ret);
		return ret;
	}

	LOG_DBG("Detecting  modem %p ...", gsm);

	while (counter++ < GSM_DETECTION_RETRIES && ret < 0) {
		k_sleep(K_SECONDS(2));
		ret = modem_cmd_send_nolock(&gsm->context.iface,
					    &gsm->context.cmd_handler,
					    &response_cmds[0],
					    ARRAY_SIZE(response_cmds),
					    "AT", &gsm->sem_response,
					    GSM_CMD_AT_TIMEOUT);
		if (ret < 0 && ret != -ETIMEDOUT) {
			break;
		}
	}

	if (ret < 0) {
		LOG_ERR("MODEM WAIT LOOP ERROR: %d", ret);
		return -ENODEV;
	} else {
		LOG_DBG("modem found");
	}
	return 0;
}

int gsm_ppp_finalize(const struct device *dev)
{
	struct gsm_modem *gsm = dev->data;
	struct net_if *iface = gsm->iface;

	LOG_DBG("Finalizing modem %p ...", gsm);
	if (iface && gsm->attached) {
		gsm_ppp_stop(dev);
	}
	if (iface) {
		net_if_l2(iface)->enable(iface, false);
	}

	modem_context_unregister(&gsm->context);

	/* destroy the "gsm_rx" thread */
	if (gsm->gsm_rx_tid) {
		k_thread_abort(gsm->gsm_rx_tid);
		gsm->gsm_rx_tid = 0;
	}

	uart_irq_rx_disable(DEVICE_DT_GET(GSM_UART_NODE));
	uart_irq_tx_disable(DEVICE_DT_GET(GSM_UART_NODE));

	return 0;
}

const struct gsm_ppp_modem_info *gsm_ppp_modem_info(const struct device *dev)
{
	struct gsm_modem *gsm = dev->data;

	return &gsm->minfo;
}

static void gsm_mgmt_event_handler(struct net_mgmt_event_callback *cb,
			  uint32_t mgmt_event, struct net_if *iface)
{
	if ((mgmt_event & NET_EVENT_IF_DOWN) != mgmt_event) {
		return;
	}

	/* Right now we only support 1 GSM instance */
	if (iface != gsm.iface) {
		return;
	}

	if (mgmt_event == NET_EVENT_IF_DOWN) {
		LOG_INF("GSM network interface down");
		/* raise semaphore to indicate the interface is down */
		k_sem_give(&gsm.sem_if_down);
		return;
	}
}

static int gsm_init(const struct device *dev)
{
	struct gsm_modem *gsm = dev->data;
	int r;

	LOG_DBG("Generic GSM modem (%p)", gsm);

	(void)k_mutex_init(&gsm->lock);
	gsm->dev = dev;

	gsm->cmd_handler_data.cmds[CMD_RESP] = response_cmds;
	gsm->cmd_handler_data.cmds_len[CMD_RESP] = ARRAY_SIZE(response_cmds);
	gsm->cmd_handler_data.match_buf = &gsm->cmd_match_buf[0];
	gsm->cmd_handler_data.match_buf_len = sizeof(gsm->cmd_match_buf);
	gsm->cmd_handler_data.buf_pool = &gsm_recv_pool;
	gsm->cmd_handler_data.alloc_timeout = K_NO_WAIT;
	gsm->cmd_handler_data.eol = "\r";

	k_sem_init(&gsm->sem_response, 0, 1);
	k_sem_init(&gsm->sem_if_down, 0, 1);

	r = modem_cmd_handler_init(&gsm->context.cmd_handler,
				   &gsm->cmd_handler_data);
	if (r < 0) {
		LOG_DBG("cmd handler error %d", r);
		return r;
	}

	/* modem information storage */
	gsm->context.data_manufacturer = gsm->minfo.mdm_manufacturer;
	gsm->context.data_model = gsm->minfo.mdm_model;
	gsm->context.data_revision = gsm->minfo.mdm_revision;
	gsm->context.data_imei = gsm->minfo.mdm_imei;
#if defined(CONFIG_MODEM_SIM_NUMBERS)
	gsm->context.data_imsi = gsm->minfo.mdm_imsi;
	gsm->context.data_iccid = gsm->minfo.mdm_iccid;
#endif	/* CONFIG_MODEM_SIM_NUMBERS */
	gsm->context.data_rssi = &gsm->minfo.mdm_rssi;
#if defined(CONFIG_MODEM_GSM_ENABLE_CESQ_RSSI)
	gsm->context.data_rscp = &gsm->minfo.mdm_rscp;
	gsm->context.data_rsrp = &gsm->minfo.mdm_rsrp;
	gsm->context.data_rsrq = &gsm->minfo.mdm_rsrq;
#endif

	gsm->context.attach_max_retries = CONFIG_MODEM_GSM_ATTACH_TIMEOUT *
				MSEC_PER_SEC / GSM_ATTACH_RETRY_DELAY_MSEC;
	gsm->context.auto_cops_max_retries = CONFIG_MODEM_GSM_AUTO_COPS_TIMEOUT *
				MSEC_PER_SEC / GSM_ATTACH_RETRY_DELAY_MSEC;
	gsm->context.is_automatic_oper = false;
	gsm->gsm_data.rx_rb_buf = &gsm->gsm_rx_rb_buf[0];
	gsm->gsm_data.rx_rb_buf_len = sizeof(gsm->gsm_rx_rb_buf);

	r = modem_iface_uart_init(&gsm->context.iface, &gsm->gsm_data,
				DEVICE_DT_GET(GSM_UART_NODE));
	if (r < 0) {
		LOG_DBG("iface uart error %d", r);
		return r;
	}

	r = modem_context_register(&gsm->context);
	if (r < 0) {
		LOG_DBG("context error %d", r);
		return r;
	}

	gsm->context.data_gsm_reg = GSM_NET_INIT;
	gsm->context.data_eps_reg = GSM_EPS_NET_INIT;
	gsm->context.data_gprs_reg = GSM_GPRS_NET_INIT;

	LOG_DBG("iface->read %p iface->write %p",
		gsm->context.iface.read, gsm->context.iface.write);

	k_thread_create(&gsm->rx_thread, gsm_rx_stack,
			K_KERNEL_STACK_SIZEOF(gsm_rx_stack),
			(k_thread_entry_t) gsm_rx,
			gsm, NULL, NULL, K_PRIO_COOP(7), 0, K_NO_WAIT);
	k_thread_name_set(&gsm->rx_thread, "gsm_rx");

#ifdef CONFIG_NET_PPP
	/* initialize the work queue */
	k_work_queue_init(&gsm->workq);
	k_work_queue_start(&gsm->workq, gsm_workq_stack, K_THREAD_STACK_SIZEOF(gsm_workq_stack),
			   K_PRIO_COOP(7), NULL);
	k_thread_name_set(&gsm->workq.thread, "gsm_workq");

	if (IS_ENABLED(CONFIG_GSM_MUX)) {
		k_work_init_delayable(&gsm->rssi_work_handle, rssi_handler);
	}

	gsm->iface = ppp_net_if();
	if (!gsm->iface) {
		LOG_ERR("Couldn't find ppp net_if!");
		modem_context_unregister(&gsm->context);

		return -ENODEV;
	}
#endif

	if (gsm_ppp_detect(dev) < 0) {
		LOG_ERR("GSM ppp did not respond!!");
#ifdef CONFIG_NET_PPP
		net_if_l2(gsm->iface)->enable(gsm->iface, false);
#endif
		modem_context_unregister(&gsm->context);

		/* the thread is not required any longer */
		k_thread_abort(gsm->gsm_rx_tid);
		gsm->gsm_rx_tid = 0;
		return -ENODEV;
	}
	net_mgmt_init_event_callback(&gsm->gsm_mgmt_cb, gsm_mgmt_event_handler,
				     NET_EVENT_IF_DOWN);
	net_mgmt_add_event_callback(&gsm->gsm_mgmt_cb);

	if (IS_ENABLED(CONFIG_GSM_PPP_AUTOSTART)) {
		gsm_ppp_start(dev);
	}
	return 0;
}

DEVICE_DT_DEFINE(DT_INST(0, zephyr_gsm_ppp), gsm_init, NULL, &gsm, NULL,
		 POST_KERNEL, CONFIG_MODEM_GSM_INIT_PRIORITY, NULL);
