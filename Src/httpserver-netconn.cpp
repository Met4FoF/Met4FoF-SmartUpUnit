/**
  ******************************************************************************
  * @file    LwIP/LwIP_HTTP_Server_Netconn_RTOS/Src/httpser-netconn.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    18-November-2015
  * @brief   Basic http server implementation using LwIP netconn API
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <configmanager.h>
#include <httpserver-netconn.hpp>
#include "../webpages/index.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define WEBSERVER_THREAD_PRIO    ( tskIDLE_PRIORITY + 4 )

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
u32_t nPageHits = 0;

/* Private function prototypes -----------------------------------------------*/
float peekValFormChannel(int channel);
/* Private functions ---------------------------------------------------------*/

/**
  * @brief serve tcp connection
  * @param conn: pointer on connection structure
  * @retval None
  */
void http_server_serve(struct netconn *conn)
{
  ConfigManager& configMan = ConfigManager::instance();
  struct netbuf *inbuf;
  err_t recv_err;
  char* buf;
  u16_t buflen;

  /* Read the data from the port, blocking if nothing yet there.
   We assume the request (the part we care about) is in one netbuf */
  recv_err = netconn_recv(conn, &inbuf);

  if (recv_err == ERR_OK)
  {
    if (netconn_err(conn) == ERR_OK)
    {
      netbuf_data(inbuf, (void**)&buf, &buflen);

      /* Is this an HTTP GET command? (only check the first 5 chars, since
      there are other formats for GET, and we're keeping it very simple )*/
      if ((buflen >=5) || (strncmp(buf, "GET /", 5) == 0))
      {
    	  //SEGGER_RTT_printf(0,"%s.\r\n",buf);
    	  if (strncmp((char const *)buf,"GET /index.html",15)==0) {
    		  netconn_write(conn, (const unsigned char*)index_html, index_html_len, NETCONN_NOCOPY);
    	  }

    	  if (strncmp((char const *)buf,"GET /led1", 9) == 0) {
    		  HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
    	  }
    	  if (strncmp((char const *)buf,"GET /led2", 9) == 0) {
    		  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    	  }
    	  if (strncmp((char const *)buf,"GET /led3", 9) == 0) {
    		  HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
    	  }
    	  //************************set ip
    	  if (strncmp((char const *)buf,"GET /ipv4", 12) == 0) {
    	      		  HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
    	      	  }

    	  if (strncmp((char const *)buf,"GET /sub1", 12) == 0) {
    	      	      		  HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
    	      	      	  }

    	  //************************
    	  if (strncmp((char const *)buf,"GET /btn1", 9) == 0) {
    		  if(HAL_GPIO_ReadPin(USER_Btn_GPIO_Port, USER_Btn_Pin) == GPIO_PIN_SET)
    			  netconn_write(conn, (const unsigned char*)"ON", 2, NETCONN_NOCOPY);
    		  else
    			  netconn_write(conn, (const unsigned char*)"OFF", 3, NETCONN_NOCOPY);
    	  }
    	  if (strncmp((char const *)buf,"GET /adc", 8) == 0) {
    		  sprintf(buf, "%2.1f °C",0.0);
    		  netconn_write(conn, (const unsigned char*)buf, strlen(buf), NETCONN_NOCOPY);
    	  }
    	  if (strncmp((char const *)buf,"GET /index.html?tbox1=",22)==0) {
    		  SEGGER_RTT_printf(0,"PARSING IP\n\r");
    		  const char *stop=strstr(buf,"&tbox2=");
    		  uint32_t len=(uint32_t)stop-(uint32_t)buf-22;
    		  SEGGER_RTT_printf(0,"LEN %llu \n\r",len);
    		  char * asciip[17]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
			  memcpy(asciip,buf+22,len);
			  SEGGER_RTT_printf(0,"PARSING >%s< \n\r",asciip);
    		  ip4_addr_t UDPip;
    		  ip4_addr_t UDPNetmask;
    		  ip4addr_aton((char const *)asciip,(ip4_addr_t*)&UDPip);

    		  char iPadressBuffer[17]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    		  char NetMaskBuffer[17]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

  			  ip4addr_ntoa_r((ip4_addr_t*)&UDPip, iPadressBuffer,sizeof(iPadressBuffer));
    		  uint32_t start=(uint32_t)buf+22+7+len;
  			  stop=strstr((const char *)start," ");
  			  len=(uint32_t)stop-(uint32_t)start;
    		  memcpy(&asciip,(const void *)start,len);
  			  ip4addr_aton((char const *)asciip,(ip4_addr_t*)&UDPNetmask);
  			  SEGGER_RTT_printf(0,"PARSING>%s<",asciip);
  			  ip4addr_ntoa_r((ip4_addr_t*)&UDPNetmask, NetMaskBuffer,sizeof(NetMaskBuffer));

  			  char HTMLBuff[300]={};
  			  sprintf(HTMLBuff,"<div class=\"panel-body\"><div id=\"IP_info\" class=\"alert alert-info\" role=\"alert\"><b>UDPTargetIP= %s NetMask= %s softreset (black button) to activate changes if battery is installed</b></div></div>\0",&iPadressBuffer,&NetMaskBuffer);
  			  configMan.setUDPSubnetmarsk(UDPNetmask);
  			  configMan.setUDPTargetIP(UDPip);
  			  netconn_write(conn, (const unsigned char*)HTMLBuff, strlen(HTMLBuff), NETCONN_NOCOPY);
    	  }
      }
    }
  }
  /* Close the connection (server closes in HTTP) */
  netconn_close(conn);

  /* Delete the buffer (netconn_recv gives us ownership,
   so we have to make sure to deallocate the buffer) */
  netbuf_delete(inbuf);
}


/**
  * @brief  http server thread
  * @retval None
  */
static void http_server_netconn_thread()
{
  struct netconn *conn, *newconn;
  err_t err, accept_err;
  SEGGER_RTT_printf(0,"Starting http_server_netconn_thread()\n\r");
  /* Create a new TCP connection handle */
  conn = netconn_new(NETCONN_TCP);
  if (conn!= NULL)
  {
    /* Bind to port 80 (HTTP) with default IP address */
    err = netconn_bind(conn, NULL, 80);
    Check_LWIP_RETURN_VAL(err);
    if (err == ERR_OK)
    {
      /* Put the connection into LISTEN state */
      netconn_listen(conn);
      SEGGER_RTT_printf(0,"Web Server TCP Connection up and running \r\n");
      while(1)
      {

    	  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        /* accept any icoming connection */
        accept_err = netconn_accept(conn, &newconn);
        Check_LWIP_RETURN_VAL(accept_err);
        if(accept_err == ERR_OK)
        {
          /* serve connection */
          http_server_serve(newconn);

          /* delete connection */
          netconn_delete(newconn);
        }
      }
    }
  }
}

/**
  * @brief  Initialize the HTTP server (start its thread)
  * @param  none
  * @retval None
  */
void http_server_netconn_init()
{
  sys_thread_new("HTTP",(lwip_thread_fn)http_server_netconn_thread, NULL, DEFAULT_THREAD_STACKSIZE, WEBSERVER_THREAD_PRIO);
}
