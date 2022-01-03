/*
 * Copyright (c) 2020 Endian Technologies AB
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef GSM_PPP_H_
#define GSM_PPP_H_

#define GSM_PPP_MDM_MANUFACTURER_LENGTH  10
#define GSM_PPP_MDM_MODEL_LENGTH         16
#define GSM_PPP_MDM_REVISION_LENGTH      64
#define GSM_PPP_MDM_IMEI_LENGTH          16
#define GSM_PPP_MDM_IMSI_LENGTH          16
#define GSM_PPP_MDM_ICCID_LENGTH         32

struct gsm_ppp_modem_info {
	char mdm_manufacturer[GSM_PPP_MDM_MANUFACTURER_LENGTH];
	char mdm_model[GSM_PPP_MDM_MODEL_LENGTH];
	char mdm_revision[GSM_PPP_MDM_REVISION_LENGTH];
	char mdm_imei[GSM_PPP_MDM_IMEI_LENGTH];
#if defined(CONFIG_MODEM_SIM_NUMBERS)
	char mdm_imsi[GSM_PPP_MDM_IMSI_LENGTH];
	char mdm_iccid[GSM_PPP_MDM_ICCID_LENGTH];
#endif
	int  mdm_rssi;
#if defined(CONFIG_MODEM_GSM_ENABLE_CESQ_RSSI)
	int  mdm_rscp;
	int  mdm_rsrp;
	float  mdm_rsrq;
#endif
};

/** @cond INTERNAL_HIDDEN */
struct device;

void gsm_ppp_start(const struct device *dev);
void gsm_ppp_stop(const struct device *dev);
bool gsm_ppp_is_running(const struct device *dev);

int gsm_ppp_detect(const struct device *dev);
int gsm_ppp_finalize(const struct device *dev);

/**
 * @brief Get GSM modem information.
 *
 * @param dev: GSM modem device.
 *
 * @retval struct gsm_ppp_modem_info * pointer to modem information structure.
 */
const struct gsm_ppp_modem_info *gsm_ppp_modem_info(const struct device *dev);

struct modem_context;

/**
 * @brief Weakly linked hook for augmenting modem setup.
 *
 * This will get called just before PDP context creation, but after initial
 * setup.
 *
 * @param ctx: Todo.
 * @param sem: Todo.
 *
 * @retval Todo.
 */
int gsm_ppp_setup_hook(struct modem_context *ctx, struct k_sem *sem);

/**
 * @brief Weakly linked hook for augmenting modem setup.
 *
 * This will get called after network connection has been established, just
 * before activating PPP.
 *
 * @param ctx: Todo.
 * @param sem: Todo.
 *
 * @retval Todo.
 */
int gsm_ppp_pre_connect_hook(struct modem_context *ctx, struct k_sem *sem);


/** @endcond */

#endif /* GSM_PPP_H_ */
