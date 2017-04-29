/*
 * tcpechoCopy.c
 *
 *  Created on: 29/04/2017
 *      Author: Alejandra Meza
 */

#include "lwip/opt.h"

uint8_t menuString[] = "Menu\r\n";
uint8_t optionString[] = "Opciones:\r\n";
uint8_t readMemoryString[] = "1) Leer Memoria\r\n";
uint8_t writeMemoryString[] = "2) Escribir Memoria\r\n";
uint8_t setHourString[] = "3) Establecer Hora\r\n";
uint8_t setDateString[] = "4) Establecer Fecha\r\n";
uint8_t hourFormatString[] = "5) Formato de Hora\r\n";
uint8_t readHourString[] = "6) Leer Hora\r\n";
uint8_t readDateString[] = "7) Leer Fecha\r\n";
uint8_t chatString[] = "8) Comunicacion con otro usuario\r\n";
uint8_t ecoString[] = "9) Eco en LCD\r\n";


#if LWIP_NETCONN

#include "lwip/sys.h"
#include "lwip/api.h"

/*-----------------------------------------------------------------------------------*/
static void tcpNewConnection_thread(void *arg)
{
	struct netbuf *buf;
	void *data;
	uint16_t len;


	struct netconn *newconn;
	newconn = (struct netconn*)(arg);
	netconn_write(newconn,menuString,sizeof(menuString), 0x01);
	netconn_write(newconn,optionString,sizeof(optionString), 0x01);
	netconn_write(newconn,readMemoryString,sizeof(readMemoryString), 0x01);
	netconn_write(newconn,writeMemoryString,sizeof(writeMemoryString), 0x01);
	netconn_write(newconn,setHourString,sizeof(setHourString), 0x01);
	netconn_write(newconn,setDateString,sizeof(setDateString), 0x01);
	netconn_write(newconn,hourFormatString,sizeof(hourFormatString), 0x01);
	netconn_write(newconn,readHourString,sizeof(readHourString), 0x01);
	netconn_write(newconn,readDateString,sizeof(readDateString), 0x01);
	netconn_write(newconn,chatString,sizeof(chatString), 0x01);
	netconn_write(newconn,ecoString,sizeof(ecoString), 0x01);

	for(;;){
		netconn_recv(newconn, &buf); //Esta esperandoa a que le llegue un mensaje
		//netconn_send(newconn, buf);
		netbuf_data(buf, &data, &len);
		if(0x31 == (netbuf_data(buf, &data, &len)))
		{
			PRINTF("1");
		}
		else if(0x32 == netbuf_data(buf, &data, &len))
		{
			PRINTF("2");
		}


		netconn_write(newconn,data,len, 0x01); /*len+3, 0x00*/
		//netconn_write(newconn, text, sizeof(text), 0x00);
		netbuf_delete(buf);
	}
	/* deallocate both connections */
	netconn_delete(newconn);
}

static void tcpecho_thread(void *arg)
{
	struct netconn *conn, *newconn;
	//err_t err;
	LWIP_UNUSED_ARG(arg);
	/* create a connection structure */
	conn = netconn_new(NETCONN_TCP);
	/* bind the connection to port 2000 on any local
	IP address */
	netconn_bind(conn, IP_ADDR_ANY, 50000);
	/* tell the connection to listen for incoming
	connection requests */
	netconn_listen(conn);
	//while(newconn == NULL);
	for(;;)
	{
		/* block until we get an incoming connection */
		netconn_accept(conn, &newconn);
		sys_thread_new("tcpNewConnection_thread", tcpNewConnection_thread, (void*)newconn, 300, DEFAULT_THREAD_PRIO);
	}

}


/*-----------------------------------------------------------------------------------*/
void
tcpecho_init(void)
{
	sys_thread_new("tcpecho_thread", tcpecho_thread, NULL, 300, DEFAULT_THREAD_PRIO);
}
/*-----------------------------------------------------------------------------------*/
#endif /* LWIP_NETCONN */


