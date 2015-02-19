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
#include "clock.h"

#include <string.h>
#include "dev/leds.h"
#include "debug.h"

#define DEBUG DEBUG_PRINT
#include "net/uip-debug.h"

#define SEND_INTERVAL		2 * CLOCK_SECOND
#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])
#define UIP_UDP_BUF  ((struct uip_udp_hdr *)&uip_buf[uip_l2_l3_hdr_len])

#define MAX_PAYLOAD_LEN 32
const char * HOST_FIXED_IP = "fe80:0000:0000:0000:0212:4b00:014d:c34e";

static char buf[MAX_PAYLOAD_LEN];
static char button0_state = 0;

#define LOCAL_CONN_PORT 3001
static struct uip_udp_conn *l_conn;

static struct uip_udp_conn *server_conn;
static uint16_t len;

/*---------------------------------------------------------------------------*/
PROCESS(udp_server_process, "UDP server process");
PROCESS(udp_client_process, "UDP client process");
AUTOSTART_PROCESSES(&udp_server_process, &udp_client_process);
/*---------------------------------------------------------------------------*/
static void udp_client_handler(void)
{
	if(uip_newdata()) {
		leds_on(LEDS_GREEN);
		clock_delay_usec(200000);
		
		/*
		putstring("0x");
		puthex(uip_datalen());
		putstring(" bytes response=0x");
		puthex((*(uint16_t *) uip_appdata) >> 8);
		puthex((*(uint16_t *) uip_appdata) & 0xFF);
		putchar('\n');
		*/
		
		leds_off(LEDS_GREEN);
	}

	return;
}
/*---------------------------------------------------------------------------*/
static void send_state_update(void)
{
	struct uip_udp_conn /**this_conn*/;

	leds_on(LEDS_YELLOW);
	memset(buf, 0, MAX_PAYLOAD_LEN);
	strcat((char *)buf, "I'm ok");

	uip_udp_packet_send(l_conn, buf, strlen((char *)buf));
	leds_off(LEDS_YELLOW);
}
/*---------------------------------------------------------------------------*/
static void udp_server_handler(void)
{
	int i;
	
	if(uip_newdata()) {
		leds_on(LEDS_RED);

		len = uip_datalen();
		//len = strlen((char*)uip_appdata); // uip_datalen(); is normally used, but we only send strings anyway..
		if (len >= MAX_PAYLOAD_LEN) // Buffer checking...
			len = MAX_PAYLOAD_LEN - 1; // -1 because we always want to end with a \0

		memset(buf, 0, MAX_PAYLOAD_LEN);
		memcpy(buf, uip_appdata, len);

		// PARSE server commands... (buf)
		
		/* Send result to host*/
		uip_udp_packet_send(l_conn, buf, len);
		
		/* Restore server connection to allow data from any node */
		uip_create_unspecified(&server_conn->ripaddr);
		server_conn->rport = 0;

		clock_delay_usec(200000);	
		leds_off(LEDS_RED);
	}
	
	return;
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udp_server_process, ev, data)
{
	PROCESS_BEGIN();
	//PRINTF("Starting UDP server\n");

	//create_rplroot();

	server_conn = udp_new(NULL, UIP_HTONS(0), NULL);
	udp_bind(server_conn, UIP_HTONS(3000));

	//PRINTF("Listen port: 3000, TTL=%u\n", server_conn->ttl);

	while(1) {
		PROCESS_YIELD();
		if(ev == tcpip_event) {
			udp_server_handler();
		}
	}

	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udp_client_process, ev, data)
{
	static struct etimer timerStateUpdate;
	uip_ipaddr_t ipaddr_server;

	PROCESS_BEGIN();

	uiplib_ipaddrconv(HOST_FIXED_IP, &ipaddr_server);
	//uip_ip6addr(&ipaddr_server, 0xfe80, 0, 0, 0, 0x0212, 0x4b00, 0x014d, 0xc34e);
	/* new connection with remote host */
	l_conn = udp_new(&ipaddr_server, UIP_HTONS(3000), NULL);
	udp_bind(l_conn, UIP_HTONS(LOCAL_CONN_PORT));

	etimer_set(&timerStateUpdate, SEND_INTERVAL);

	while(1) {
		PROCESS_YIELD();
		if(etimer_expired(&timerStateUpdate)) {
			send_state_update();
			etimer_restart(&timerStateUpdate);
		} else if(ev == tcpip_event) {
			udp_client_handler();
		}
	}

	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
