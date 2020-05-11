#ifndef __HTTPSERVER_NETCONN_H__
#define __HTTPSERVER_NETCONN_H__

#ifdef __cplusplus
 extern "C" {
#endif

#include "lwip/opt.h"
#include "lwip/arch.h"
#include "lwip/api.h"
#include "string.h"
#include "cmsis_os.h"
#include "lwip_return_ckeck.h"
void http_server_netconn_init(void);
void DynWebPage(struct netconn *conn);
#ifdef __cplusplus
}
#endif
#endif /* __HTTPSERVER_NETCONN_H__ */
