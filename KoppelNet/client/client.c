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
#define MAX_PAYLOAD_LEN		40

static char buf[MAX_PAYLOAD_LEN];
static char button0_state = 0;

#define LOCAL_CONN_PORT 3001
static struct uip_udp_conn *l_conn;

/*---------------------------------------------------------------------------*/
PROCESS(udp_client_process, "UDP client process");
AUTOSTART_PROCESSES(&udp_client_process);
/*---------------------------------------------------------------------------*/
static void response_handler(void)
{
	if(uip_newdata()) {
		leds_on(LEDS_GREEN);
		clock_delay_usec(150 * 1000);
		
		/*
		putstring("0x");
		puthex(uip_datalen());
		putstring(" bytes response=0x");
		puthex((*(uint16_t *) uip_appdata) >> 8);
		puthex((*(uint16_t *) uip_appdata) & 0xFF);
		putchar('\n');
		*/
	}

	leds_off(LEDS_GREEN);
	return;
}
/*---------------------------------------------------------------------------*/
static void send_state_update(void)
{
	static int seq_id;
	struct uip_udp_conn /**this_conn*/;

	leds_on(LEDS_YELLOW);
	memset(buf, 0, MAX_PAYLOAD_LEN);
	memcpy(buf, &seq_id, sizeof(seq_id));
	seq_id++;

	uip_udp_packet_send(l_conn, buf, sizeof(seq_id));
	leds_off(LEDS_YELLOW);
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udp_client_process, ev, data)
{
	static struct etimer timerStateUpdate;
	uip_ipaddr_t ipaddr;

	PROCESS_BEGIN();

	uip_ip6addr(&ipaddr, 0xfe80, 0, 0, 0, 0x0212, 0x4b00, 0x014d, 0xc34e);
	/* new connection with remote host */
	l_conn = udp_new(&ipaddr, UIP_HTONS(3000), NULL);
	if(!l_conn) {
		leds_on(LEDS_RED);
	} else leds_off(LEDS_RED);

	udp_bind(l_conn, UIP_HTONS(LOCAL_CONN_PORT));

	etimer_set(&timerStateUpdate, SEND_INTERVAL);

	while(1) {
		PROCESS_YIELD();
		if(etimer_expired(&timerStateUpdate)) {
			send_state_update();
			etimer_restart(&timerStateUpdate);
		} else if(ev == tcpip_event) {
			response_handler();
		}
	}

	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
