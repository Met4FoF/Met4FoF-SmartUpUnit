/*
 * lwip_return_ckeck.h
 *
 *  Created on: May 4, 2020
 *      Author: benedikt
 */

#ifndef LWIP_RETURN_CKECK_H_
#define LWIP_RETURN_CKECK_H_

#ifdef __cplusplus
extern "C" {
#endif
#include "SEGGER_RTT.h"
#include "lwip.h"
void Check_LWIP_RETURN_VAL(err_t);
#ifdef __cplusplus
}
#endif
#endif /* LWIP_RETURN_CKECK_H_ */
