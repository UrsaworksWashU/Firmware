/**
 * @file    msg_center.c
 * @author  eason
 * @brief   Latest-value publish/subscribe. See msg_center.h for the model and the rules.
 *
 * @version beta
 * @date    2026-11-12
 * @copyright Copyright (c) 2026
 *
 */

#include "msg_center.h"
#include "bsp_critical.h"
#include "bsp_time.h"
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

Msg_Status_e Msg_Publish(Msg_Topic_t *topic, const void *data, uint16_t size)
{
    if (topic == NULL || data == NULL || size != topic->size)
    {
        return MSG_ERR_PARAM;
    }

    const uint32_t now = BSP_Time_GetMs();

    const uint32_t state = BSP_EnterCritical();
    memcpy(topic->storage, data, size);
    topic->seq++;
    if (topic->seq == 0)
    {
        topic->seq = 1; /* after 2^32 publishes: skip 0, it means "never published" */
    }
    topic->stamp_ms = now;
    BSP_ExitCritical(state);

    return MSG_NEW;
}

void Msg_Subscribe(Msg_Sub_t *sub, const Msg_Topic_t *topic)
{
    if (sub == NULL)
    {
        return;
    }
    sub->topic = topic;
    sub->last_seq = 0; /* differs from any published seq, so the first Read reports NEW */
}

Msg_Status_e Msg_Read(Msg_Sub_t *sub, void *out, uint16_t size, uint32_t *age_ms)
{
    if (sub == NULL || sub->topic == NULL || out == NULL || size != sub->topic->size)
    {
        return MSG_ERR_PARAM;
    }
    const Msg_Topic_t *topic = sub->topic;

    /* Message, seq and stamp are copied together, so they always belong to the same publish. */
    const uint32_t state = BSP_EnterCritical();
    const uint32_t seq = topic->seq;
    const uint32_t stamp_ms = topic->stamp_ms;
    if (seq != 0)
    {
        memcpy(out, topic->storage, size);
    }
    BSP_ExitCritical(state);

    if (seq == 0)
    {
        return MSG_NONE;
    }

    /* "now" is read after the stamp, so now - stamp_ms never wraps (same rule as DT7_GetData). */
    if (age_ms != NULL)
    {
        *age_ms = BSP_Time_GetMs() - stamp_ms;
    }

    const bool is_new = (seq != sub->last_seq);
    sub->last_seq = seq;
    return is_new ? MSG_NEW : MSG_OLD;
}