/**
 * @file    msg_center.h
 * @author  eason
 * @brief   Latest-value publish/subscribe between applications (cmd, gimbal, chassis, ...).
 *
 *          Model: a topic holds ONE message, the latest one published. Publishing overwrites it.
 *          - Each subscriber remembers the sequence number it last read, so it can tell
 *            "new message" from "same message as last time".
 *          - The topic remembers when it was last published, so a reader can detect a publisher
 *            that stopped (timeout) and fall back to a safe state.
 *          HNU's message_center with QUEUE_SIZE 1 behaves the same way, but builds it from
 *          malloc'd linked lists found by string comparison at runtime.
 *
 *          Topics are created at compile time:
 *          - MSG_TOPIC_DEFINE(topic, type) in exactly one .c file (by convention, the publisher's),
 *          - MSG_TOPIC_DECLARE(topic) in a shared header for everyone else.
 *          A misspelled topic is then a compile or link error, not a silent new topic.
 *
 *          Why bother instead of calling Chassis_SetCommand() directly: the apps stay unaware of
 *          each other. On a two-board robot, a CAN bridge on the chassis board publishes the same
 *          topic the cmd app publishes on a one-board robot, and the chassis app does not change.
 *
 *          Safety: Publish and Read copy inside a short critical section, so a reader never gets
 *          half of one message and half of another, even if the publisher runs in another task or
 *          an interrupt. Keep messages small (tens of bytes): the copy runs with interrupts off.
 *
 * @version beta
 * @date    2026-11-12
 * @copyright Copyright (c) 2026
 *
 */

#ifndef MSG_CENTER_H
#define MSG_CENTER_H

#include <stdint.h>

/* ------------------------------ types ------------------------------ */

typedef struct
{
    const char *name;   /* for the debugger only */
    void *storage;      /* size bytes, created by MSG_TOPIC_DEFINE */
    uint16_t size;
    uint32_t seq;       /* number of publishes so far; 0 = never published */
    uint32_t stamp_ms;  /* BSP_Time_GetMs() of the latest publish */
} Msg_Topic_t;

typedef struct
{
    const Msg_Topic_t *topic;
    uint32_t last_seq;  /* topic->seq at this subscriber's last read */
} Msg_Sub_t;

typedef enum
{
    MSG_NEW = 0,    /* copied; published since this subscriber's last read */
    MSG_OLD,        /* copied; the same message as the last read */
    MSG_NONE,       /* never published; *out is left untouched */
    MSG_ERR_PARAM,  /* NULL pointer, or size does not match the topic's type */
} Msg_Status_e;

/* ------------------------------ topic macros ------------------------------ */

/** In a shared header: make the topic visible to other files. */
#define MSG_TOPIC_DECLARE(topic) extern Msg_Topic_t topic

/** In exactly one .c file: create the topic and its storage for one message of `type`. */
#define MSG_TOPIC_DEFINE(topic, type)              \
    static type topic##_storage;                   \
    Msg_Topic_t topic = {                          \
        .name = #topic,                            \
        .storage = &topic##_storage,               \
        .size = (uint16_t)sizeof(type),            \
        .seq = 0,                                  \
        .stamp_ms = 0,                             \
    }

/* ------------------------------ functions ------------------------------ */

/**
 * Overwrite the topic's message with *data.
 * @param size sizeof the message; must equal the topic's type size (catches a wrong type).
 * @return MSG_NEW on success, MSG_ERR_PARAM otherwise.
 */
Msg_Status_e Msg_Publish(Msg_Topic_t *topic, const void *data, uint16_t size);

/** Attach a subscriber to a topic. Its first Read reports MSG_NEW if anything was published. */
void Msg_Subscribe(Msg_Sub_t *sub, const Msg_Topic_t *topic);

/**
 * Copy the latest message.
 * @param size   sizeof(*out); must equal the topic's type size.
 * @param age_ms optional (may be NULL): milliseconds since that message was published.
 * @return MSG_NEW / MSG_OLD (both copy), MSG_NONE, or MSG_ERR_PARAM.
 */
Msg_Status_e Msg_Read(Msg_Sub_t *sub, void *out, uint16_t size, uint32_t *age_ms);

#endif /* MSG_CENTER_H */