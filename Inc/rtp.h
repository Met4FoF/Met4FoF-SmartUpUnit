#ifndef __RTP_H__
#define __RTP_H__
#ifdef __cplusplus
 extern "C" {
#endif
#if LWIP_SOCKET && LWIP_IGMP
void rtp_init(void);
#endif /* LWIP_SOCKET && LWIP_IGMP */
#ifdef __cplusplus
}
#endif
#endif /* __RTP_H__ */
