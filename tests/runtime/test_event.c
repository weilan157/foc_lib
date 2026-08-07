/*
 * test_event.c —— Event SPSC 环形缓冲（§16.4）
 */
#include "runtime/event.h"
#include "../test_assert.h"

int main(void)
{
    EventQueue q;
    FocEvent ev;
    FocEvent out;

    event_queue_init(&q);

    /* publish → poll FIFO */
    ev.type = EVENT_MODE_CHANGED; ev.source = 0u; ev.timestamp_us = 1u; ev.data = 2u;
    CHECK(event_publish(&q, &ev));

    ev.type = EVENT_FAULT_RAISED; ev.source = 0u; ev.timestamp_us = 2u; ev.data = 3u;
    CHECK(event_publish(&q, &ev));

    CHECK(event_poll(&q, &out));
    CHECK(out.type == EVENT_MODE_CHANGED);
    CHECK(out.data == 2u);

    CHECK(event_poll(&q, &out));
    CHECK(out.type == EVENT_FAULT_RAISED);
    CHECK(out.data == 3u);

    /* 空队列 poll → false */
    CHECK(!event_poll(&q, &out));

    TEST_REPORT();
}
