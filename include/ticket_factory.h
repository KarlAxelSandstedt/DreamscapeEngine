/*
==========================================================================
    Copyright (C) 2025, 2026 Axel Sandstedt 

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
==========================================================================
*/

/*
ticket_factory.h
================
Thread-safe FIFO tickets for a ring of max_tickets slots: a ticket t owns slot t % max_tickets until it is
returned. At most max_tickets tickets are out at once; tickets are returned in order, in batches, by
whoever serves them (ds_log.c writes completed log slots, then returns them).

    struct ticketFactory tf;
    TicketFactoryInit(&tf, 512);
    u32 ticket;
    if (TicketFactoryTryGetTicket(&ticket, &tf) == 1) { ... fill slot ticket % 512 ... }
    TicketFactoryReturnTickets(&tf, served);      (the oldest served tickets, [a_serve, a_serve + served))

Tickets count up from 0 and wrap at 2^32; max_tickets is a power of two, so slot indices stay continuous
across the wrap. a_serve is the oldest ticket not yet returned, a_next the next ticket handed out.
a_open == 0 closes the factory (set it directly, release order): TryGetTicket calls that start after the store
see it closed; a call already past its check may still get a ticket.
*/

#ifndef __FIFO_TICKET_H__
#define __FIFO_TICKET_H__

#ifdef __cplusplus
extern "C" { 
#endif

#include "ds_semaphore.h"

#define TICKET_FACTORY_CLOSED	U32_MAX

struct ticketFactory
{
	semaphore 	available;	/* tickets that may still be handed out */
	u32 		a_serve;
	u32 		a_next;
	u32 		a_open;
	u32 		max_tickets;
};

/* Open tf with max_tickets (a power of two) tickets available. */
void 	TicketFactoryInit(struct ticketFactory *tf, const u32 max_tickets);
/* Free tf's semaphore; no thread may be waiting in or entering the factory. */
void	TicketFactoryDestroy(struct ticketFactory *tf);
/*
Hand out the next ticket into *ticket and return 1; return 0 (no ticket free: all max_tickets are out) or
TICKET_FACTORY_CLOSED (closed), *ticket then unwritten. Never blocks.
*/
u32	TicketFactoryTryGetTicket(u32 *ticket, struct ticketFactory *tf);
/*
Spin until a ticket is free and return it. Returns TICKET_FACTORY_CLOSED once the factory is closed; that
value is also a valid ticket after 2^32 - 1 others, so callers that close the factory use TryGetTicket.
*/
u32	TicketFactoryGetTicket(struct ticketFactory *tf);
/* Return the count oldest tickets out, [a_serve, a_serve + count), making them available again. */
void	TicketFactoryReturnTickets(struct ticketFactory *tf, const u32 count);

#ifdef __cplusplus
} 
#endif

#endif
