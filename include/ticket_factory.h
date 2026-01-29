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

#ifndef __FIFO_TICKET_H__
#define __FIFO_TICKET_H__

#ifdef __cplusplus
extern "C" { 
#endif

#include "ds_semaphore.h"

#define TICKET_FACTORY_CLOSED	U32_MAX

/*
 * multiple ticket consumers with a maximum ticket count. It is up to the user to determine
 * when a batch of tickets should be returned, or "served".
 */
struct ticketFactory
{
	semaphore 	available;	/* slots available for producers */
	u32 		a_serve;	/* useful to keep track of where we are in array */
	u32 		a_next;
	u32 		a_open;		/* when open, tickets can be retrieved */
	u32 		max_tickets;	/* debug */
};

/* Initalize ticket factory address */
void 	TicketFactoryInit(struct ticketFactory *tf, const u32 max_tickets);
/* Destroy ticket factory */
void	TicketFactoryDestroy(struct ticketFactory *tf);
/* Returns 0 on no ticket, 1 on ticket acquisition, or TICKET_FACTORY_CLOSED when no more tickets can be retrieved  */
u32	TicketFactoryTryGetTicket(u32 *ticket, struct ticketFactory *tf);
/* Wait on semaphore until we return a ticket.  UNDEFINED BEHAVIOUR when TICKET_FACTORY_CLOSED! */
u32	TicketFactoryGetTicket(struct ticketFactory *tf);
/* puts ticket range [a_serve, a_serve + count) up for use again  */
void	TicketFactoryReturnTickets(struct ticketFactory *tf, const u32 count);

#ifdef __cplusplus
} 
#endif

#endif
