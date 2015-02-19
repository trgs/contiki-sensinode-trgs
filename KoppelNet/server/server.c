/*
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */
#include "contiki.h"
#include "contiki-lib.h"
#include "contiki-net.h"

#include <string.h>
#define DEBUG DEBUG_PRINT // Required for printing to to serial port using PRINTF
#define DEBUG_PACKETS

//#define CC2530_RF_CONF_CHANNEL 21 // Required for Cooja simulation...

#include "dev/watchdog.h"
#include "dev/leds.h"
#include "net/rpl/rpl.h"
//#include "dev/button-sensor.h"
#include "net/uip-debug.h"
//#include "debug.h"

#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])
#define UIP_UDP_BUF  ((struct uip_udp_hdr *)&uip_buf[uip_l2_l3_hdr_len])

#define MAX_PAYLOAD_LEN 32
#define MAX_NODES 32

#define BOOL char
#define TRUE 1
#define FALSE 0

#define PRINTADDRESS(addr) PRINTF("%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x", ((uint8_t *)addr)[0], ((uint8_t *)addr)[1], ((uint8_t *)addr)[2], ((uint8_t *)addr)[3], ((uint8_t *)addr)[4], ((uint8_t *)addr)[5], ((uint8_t *)addr)[6], ((uint8_t *)addr)[7], ((uint8_t *)addr)[8], ((uint8_t *)addr)[9], ((uint8_t *)addr)[10], ((uint8_t *)addr)[11], ((uint8_t *)addr)[12], ((uint8_t *)addr)[13], ((uint8_t *)addr)[14], ((uint8_t *)addr)[15])

static struct nodeInfo {
        uint8_t address[16];
		unsigned long last_seen;
};

static struct uip_udp_conn *server_conn;

static char buf[MAX_PAYLOAD_LEN];
static uint16_t len;

#define LOCAL_CONN_PORT 3001
static struct uip_udp_conn *l_conn;

static uip_ipaddr_t ipaddr;

/* Vars for node list management */
static struct nodeInfo nodes[MAX_NODES];
static int node_count;

/* Vars for serial host interface */
extern process_event_t serial_line_event_message;

/*---------------------------------------------------------------------------*/
PROCESS(udp_server_process, "UDP server process");
PROCESS(cc2531_usb_process, "cc2531 USB serial interface process");
AUTOSTART_PROCESSES(&udp_server_process, &cc2531_usb_process);
/*---------------------------------------------------------------------------*/
static BOOL node_add(uint8_t *address)
{
	//PRINTF("NETWORK> adding new node!!\n");
	if ((node_count+1) >= MAX_NODES) {
		return FALSE;
	}
	
	memcpy(nodes[node_count].address, address, 16); 
	nodes[node_count].last_seen = clock_seconds();
	
	node_count++;
	
	return TRUE;
}
/*---------------------------------------------------------------------------*/
static int node_find(uint8_t *address)
{
	int i,x;
	
	for (i=0; i<node_count; i++) {
		for (x=0; x<16; x++) {
			if (nodes[i].address[x] != address[x]) {
				break;
			}
		}
		
		if (x == 16) {
			return i;
		}
	}
	
	return -1;
}
/*---------------------------------------------------------------------------*/
static BOOL node_update(uint8_t *address)
{
	int index = node_find(address);
	
	if (index == -1) {
		if (!node_add(address)) {
			return FALSE;
		}
	} else {
		nodes[index].last_seen = clock_seconds();
	}

	return TRUE;
}
/*---------------------------------------------------------------------------*/
static void udp_server_handler(void)
{
	int i;
	
	if(uip_newdata()) {
		leds_on(LEDS_GREEN);
		
		len = uip_datalen();
		//len = strlen((char*)uip_appdata); // uip_datalen(); is normally used, but we only send strings anyway..
		if (len >= MAX_PAYLOAD_LEN) // Buffer checking...
			len = MAX_PAYLOAD_LEN;

		memset(buf, 0, MAX_PAYLOAD_LEN);
		memcpy(buf, uip_appdata, len);

#ifdef DEBUG_PACKETS
		PRINTF("%u bytes from [", len);
		PRINTADDRESS(&UIP_IP_BUF->srcipaddr);
		PRINTF("]:%u");

		// String dump data
		PRINTF(" STRING => %s", (char *)buf);
		
		// Hex dump of data
		PRINTF(" HEX =>", UIP_HTONS(UIP_UDP_BUF->srcport));
		for (i=0; i<len; i++)
		{
			PRINTF(" 0x%x", buf[i]);
		}

		PRINTF("\n");
#endif
		node_update(UIP_IP_BUF->srcipaddr.u8);

#ifdef DEBUG_PACKETS
		PRINTF("Sending ACK to: ");
		PRINTADDRESS(&UIP_IP_BUF->srcipaddr);
		PRINTF("]:%u\n", UIP_HTONS(UIP_UDP_BUF->srcport));
#endif

		/* Send response to client */
		uip_ipaddr_copy(&server_conn->ripaddr, &UIP_IP_BUF->srcipaddr);
		server_conn->rport = UIP_UDP_BUF->srcport;
		uip_udp_packet_send(server_conn, buf, len);
		
		/* Restore server connection to allow data from any node */
		uip_create_unspecified(&server_conn->ripaddr);
		server_conn->rport = 0;
		
		leds_off(LEDS_GREEN);
	}
	
	return;
}
/*---------------------------------------------------------------------------*/
static void print_nodelist()
{
  // TODO
  PRINTF("> NODES: ");
  PRINTADDRESS(nodes[0].address);
  PRINTF("\n");
  //PRINTF(" (last update %lu seconds ago)\n", clock_seconds() - nodes[0].last_seen); // FIXME TIMESHIT IS BUGGED, crashed
}
/*---------------------------------------------------------------------------*/
static void print_stats()
{
	PRINTF("> STATS: tl=%lu, ts=%lu, bs=%lu, bc=%lu ", rimestats.toolong, rimestats.tooshort, rimestats.badsynch, rimestats.badcrc);
	PRINTF("llrx=%lu, lltx=%lu, rx=%lu, tx=%lu\n", rimestats.llrx, rimestats.lltx, rimestats.rx, rimestats.tx);
}
/*---------------------------------------------------------------------------*/
static void print_local_addresses(void)
{
	int i;
	uint8_t state;

	PRINTF("> INFO: Server IPv6 addresses:\n");
	for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
		state = uip_ds6_if.addr_list[i].state;
		if(uip_ds6_if.addr_list[i].isused && (state == ADDR_TENTATIVE || state == ADDR_PREFERRED)) {
			PRINTF("> INFO:   ");
			PRINTADDRESS(&uip_ds6_if.addr_list[i].ipaddr);
			PRINTF("\n");
			if(state == ADDR_TENTATIVE) {
				uip_ds6_if.addr_list[i].state = ADDR_PREFERRED;
			}
		}
	}
}
/*---------------------------------------------------------------------------*/
void create_rplroot()
{
	rpl_dag_t *dag;

	uip_ip6addr(&ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 0);
	uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
	uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);

	dag = rpl_set_root(RPL_DEFAULT_INSTANCE, &uip_ds6_get_global(ADDR_PREFERRED)->ipaddr);
	if(dag != NULL) {
		uip_ip6addr(&ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 0);
		rpl_set_prefix(dag, &ipaddr, 64);

		PRINTF("Created a new RPL dag with ID: ");
		PRINTADDRESS(&dag->dag_id);
		PRINTF("\n");
	}
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udp_server_process, ev, data)
{
	PROCESS_BEGIN();
	PRINTF("Starting UDP server\n");

	create_rplroot();

	server_conn = udp_new(NULL, UIP_HTONS(0), NULL);
	udp_bind(server_conn, UIP_HTONS(3000));

	PRINTF("Listen port: 3000, TTL=%u\n", server_conn->ttl);

	while(1) {
		PROCESS_YIELD();
		if(ev == tcpip_event) {
			udp_server_handler();
		}
	}

	PROCESS_END();
}
/*----------------------------------------------------------------------------*/
PROCESS_THREAD(cc2531_usb_process, ev, data)
{
	uip_ipaddr_t node_ipaddr;
	char *pch;
	int argc;

	PROCESS_BEGIN();

	data = 0;
	
	while(1) {
		PROCESS_WAIT_EVENT();

		if(ev == serial_line_event_message) {

		if (strstr(data,"help")) {
				PRINTF("> HELP: help - Display this message\n");
				PRINTF("> HELP: info - Print host info\n");
				PRINTF("> HELP: nodes - Print list of active nodes\n");
				PRINTF("> HELP: stats - Print RIME stats\n");
				PRINTF("> HELP: cmd <node_ip> <command> - Sends command to node\n");
			} else if (strstr(data,"stats")) {
				print_stats();
			} else if (strstr(data,"nodes")) {
				print_nodelist();
			} else if (strstr(data,"info")) {
				print_local_addresses();
				PRINTF("> INFO: UDP Server listening port: 3000, TTL=%u\n", server_conn->ttl);
				PRINTF("> INFO: RF Channel=%u PanID=0x%x\n", CC2530_RF_CONF_CHANNEL, IEEE802154_CONF_PANID);
			} else if (strstr(data,"cmd")) {
				/* split args */
			
				argc = 0;
				pch = strtok ((char *)data," ");
				memset(buf, 0, MAX_PAYLOAD_LEN);
				
				while (pch != NULL)
				{
					switch (argc)
					{
						case 0: // "cmd"
						break;
						case 1: // ipv6 address of target client node
							// fe80:0000:0000:0000:0212:4b00:014d:c34e
							uiplib_ipaddrconv(pch, &node_ipaddr);

							l_conn = udp_new(&node_ipaddr, UIP_HTONS(3000), NULL);
							udp_bind(l_conn, UIP_HTONS(LOCAL_CONN_PORT));
						break;
						default: // command to send to client node
							strcat((char *)buf, pch);
							strcat((char *)buf, " ");
						break;
					}

					pch = strtok(NULL, " ");
					argc++;
				}

				// and finally send command to client node
				buf[strlen((char *)buf)-1] = 0; // strip last " "
				uip_udp_packet_send(l_conn, buf, strlen((char*)buf));

				// and cleanup old connection data to prevent memleaks
				if(l_conn != NULL) {
					uip_udp_remove(l_conn);
				}
			} else {
				PRINTF("> ERROR: Unknown command, will implement soon ;)\n");
			}
		}
	}

	PROCESS_END();
}
