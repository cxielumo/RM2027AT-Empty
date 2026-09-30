#ifndef RM2027AT_DAEMON_H
#define RM2027AT_DAEMON_H

#include <stdbool.h>
#include <stdint.h>

typedef void (*daemon_callback_t)(void *context);

/* One callback slot; registration replaces the previous callback.
 * NULL unregisters. Called once per timeout after DJI shutdown, outside
 * critical sections in the monitor task. Must return promptly.
 * Context must remain valid until any in-progress callback finishes. */
void daemon_registerCallback(daemon_callback_t callback, void *context);

/* Singleton, task context only. Timeout is in milliseconds (1..INT32_MAX).
 * Enable returns 0 or -1; invalid timeout leaves state unchanged.
 * Enable starts offline; reload marks online and starts the timeout.
 * Repeated enable updates timeout and resets the state to offline.
 * Enable immediately shuts down DJI motors. While offline, DJI commands
 * are rejected; a private task shuts down DJI motors on timeout.
 * Reload permits new explicit commands, never restores previous targets.
 * Before enable, daemon does not restrict motor outputs. */
int daemon_enable(uint32_t timeout_ms);
void daemon_reload(void);
bool daemon_isOnline(void);

#endif
