/*
 * Copyright (c) 2020 Endian Technologies AB
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef GSM_PPP_H_
#define GSM_PPP_H_

/** @cond INTERNAL_HIDDEN */
struct device;
void gsm_ppp_start(const struct device *dev);
void gsm_ppp_stop(const struct device *dev);

int gsm_ppp_detect(const struct device *dev);
int gsm_ppp_finalize(const struct device *dev);

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
