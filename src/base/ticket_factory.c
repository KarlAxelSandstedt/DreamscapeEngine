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

#include "ds_base.h"
#include "ticket_factory.h"

void TicketFactoryInit(struct ticketFactory *tf, const u32 max_tickets)
{
	ds_Assert(max_tickets > 0 && PowerOfTwoCheck(max_tickets));

	tf->max_tickets = max_tickets;
	SemaphoreInit(&tf->available, max_tickets);
	AtomicStoreRel32(&tf->a_serve, 0);
	AtomicStoreRel32(&tf->a_next, 0);
	AtomicStoreRel32(&tf->a_open, 1);
}

void TicketFactoryDestroy(struct ticketFactory *tf)
{
	SemaphoreDestroy(&tf->available);
}

u32 TicketFactoryTryGetTicket(u32 *ticket, struct ticketFactory *tf)
{
	if (!AtomicLoadAcq32(&tf->a_open)) { return TICKET_FACTORY_CLOSED; }

	u32 got_ticket = 0;
	if (SemaphoreTryWait(&tf->available))
	{
		got_ticket = 1;
		*ticket = AtomicFetchAddRlx32(&tf->a_next, 1);
	}

	return got_ticket;
}

u32 TicketFactoryGetTicket(struct ticketFactory *tf)
{
	u32 ticket;
	u32 ret;
	/* spins: closing the factory must end the wait, which a blocking SemaphoreWait would not */
	while (!(ret = TicketFactoryTryGetTicket(&ticket, tf)));

	/* closed: ticket was never written */
	return (ret == TICKET_FACTORY_CLOSED) ? TICKET_FACTORY_CLOSED : ticket;
}

void TicketFactoryReturnTickets(struct ticketFactory *tf, const u32 count)
{
	ds_Assert(count <= tf->max_tickets);
	ds_Assert(count <= ((u32) AtomicLoadRlx32(&tf->a_next) - (u32) AtomicLoadRlx32(&tf->a_serve)));
	ds_Assert(((u32) AtomicLoadRlx32(&tf->a_next) - (u32) AtomicLoadRlx32(&tf->a_serve)) <= tf->max_tickets);

	/* sync_point */
	AtomicFetchAddRel32(&tf->a_serve, count);
	for (u32 i = 0; i < count; ++i)
	{
		SemaphorePost(&tf->available);
	}	
}
