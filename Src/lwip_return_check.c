#include "lwip.h"
#include  "SEGGER_RTT.h"

void Check_LWIP_RETURN_VAL(err_t retVal) {
	static uint32_t LWIP_RRT_PRINT_ErrorCount = 0;
	if (retVal != ERR_OK) {
		switch (retVal) {
		case -1:
			SEGGER_RTT_printf(0, "%u LWIP ERR_MEM: Out of memory error.\r\n",
					LWIP_RRT_PRINT_ErrorCount);
			LWIP_RRT_PRINT_ErrorCount++;
			break;
		case -2:
			SEGGER_RTT_printf(0, "%u LWIP ERR_BUF: Buffer error.\r\n",
					LWIP_RRT_PRINT_ErrorCount);
			LWIP_RRT_PRINT_ErrorCount++;
			break;
		case -3:
			SEGGER_RTT_printf(0, "%u LWIP ERR_TIMEOUT: Time Out.\r\n",
					LWIP_RRT_PRINT_ErrorCount);
			LWIP_RRT_PRINT_ErrorCount++;
			break;
		case -4:
			SEGGER_RTT_printf(0, "%u LWIP ERR_RTE: Routing problem.\r\n",
					LWIP_RRT_PRINT_ErrorCount);
			LWIP_RRT_PRINT_ErrorCount++;
			break;
		case -5:
			SEGGER_RTT_printf(0,
					"%u LWIP ERR_INPROGRESS: Operation in progress.\r\n",
					LWIP_RRT_PRINT_ErrorCount);
			LWIP_RRT_PRINT_ErrorCount++;
			break;
		case -6:
			SEGGER_RTT_printf(0, "%u LWIP ERR_VAL: Illegal value.\r\n",
					LWIP_RRT_PRINT_ErrorCount);
			LWIP_RRT_PRINT_ErrorCount++;
			break;
		case -7:
			SEGGER_RTT_printf(0,
					"%u LWIP ERR_WOULDBLOCK: Operation would block.\r\n",
					LWIP_RRT_PRINT_ErrorCount);
			LWIP_RRT_PRINT_ErrorCount++;
			break;
		case -8:
			SEGGER_RTT_printf(0, "%u LWIP ERR_USE: Address in use.\r\n",
					LWIP_RRT_PRINT_ErrorCount);
			LWIP_RRT_PRINT_ErrorCount++;
			break;
		case -9:
			SEGGER_RTT_printf(0, "%u LWIP ERR_ALREADY: Already connecting.\r\n",
					LWIP_RRT_PRINT_ErrorCount);
			LWIP_RRT_PRINT_ErrorCount++;
			break;
		case -10:
			SEGGER_RTT_printf(0,
					"%u LWIP ERR_ISCONN: Conn already established.\r\n",
					LWIP_RRT_PRINT_ErrorCount);
			LWIP_RRT_PRINT_ErrorCount++;
			break;
		case -11:
			SEGGER_RTT_printf(0, "%u LWIP ERR_CONN: Not connected.\r\n",
					LWIP_RRT_PRINT_ErrorCount);
			LWIP_RRT_PRINT_ErrorCount++;
			break;
		case -12:
			SEGGER_RTT_printf(0, "%u LWIP ERR_IF: Low-level netif error.\r\n",
					LWIP_RRT_PRINT_ErrorCount);
			LWIP_RRT_PRINT_ErrorCount++;
			break;
		case -13:
			SEGGER_RTT_printf(0, "%u LWIP ERR_ABRT: Connection aborted.\r\n",
					LWIP_RRT_PRINT_ErrorCount);
			LWIP_RRT_PRINT_ErrorCount++;
			break;
		case -14:
			SEGGER_RTT_printf(0, "%u LWIP ERR_RST: Connection reset.\r\n",
					LWIP_RRT_PRINT_ErrorCount);
			LWIP_RRT_PRINT_ErrorCount++;
			break;
		case -15:
			SEGGER_RTT_printf(0, "%u LWIP ERR_CLSD: Connection closed.\r\n",
					LWIP_RRT_PRINT_ErrorCount);
			LWIP_RRT_PRINT_ErrorCount++;
			break;
		case -16:
			SEGGER_RTT_printf(0, "%u LWIP ERR_ARG: Illegal argument.\r\n",
					LWIP_RRT_PRINT_ErrorCount);
			LWIP_RRT_PRINT_ErrorCount++;
			break;
		}
	}
}
