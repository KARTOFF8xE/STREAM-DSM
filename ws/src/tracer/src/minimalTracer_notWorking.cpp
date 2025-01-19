#include <lttng/lttng.h>

#include <assert.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define DEFAULT_DATA_AVAILABILITY_WAIT_TIME 200000 /* usec */

static volatile int quit = 0;

static int setup_session(const char *session_name, const char *path) {
	int ret;
	struct lttng_domain dom;
	struct lttng_event ev;
	struct lttng_handle *chan_handle = NULL;

	printf("Creating session %s\n", session_name);
	ret = lttng_create_session(session_name, path);
	if (ret) {
		fprintf(stderr, "Failed to create session, ret = %d\n", ret);
		goto end;
	}
	
	dom.type = LTTNG_DOMAIN_UST;
	dom.buf_type = LTTNG_BUFFER_PER_PID;

	chan_handle = lttng_create_handle(session_name, &dom);
	if (chan_handle == NULL) {
		ret = -1;
		goto end;
	}

struct lttng_channel chan;
memset(&chan, 0, sizeof(chan));
strcpy(chan.name, "mychan");
chan.attr.overwrite = 1;                 // Optional: Überschreibmodus
chan.attr.subbuf_size = 4096;            // Größe der Unterpuffer
chan.attr.num_subbuf = 8;                // Anzahl der Unterpuffer
chan.attr.switch_timer_interval = 0;     // Kein Wechsel-Timer
chan.attr.read_timer_interval = 200;     // Leseintervall
// chan.att = LTTNG_BUFFER_PER_PID; // Buffer-Typ

ret = lttng_enable_channel(chan_handle, &chan);
if (ret < 0) {
    fprintf(stderr, "Failed to enable channel (ret = %d)\n", ret);
    goto end;
}

	memset(&ev, 0, sizeof(ev));
	ev.type = LTTNG_EVENT_USERSPACE_PROBE;
	strcpy(ev.name, "ros2:rcl_node_init");
	ev.loglevel_type = LTTNG_EVENT_LOGLEVEL_ALL;

	ret = lttng_enable_event_with_exclusions(chan_handle, &ev, "mychan", NULL, 0, NULL);
	if (ret < 0) {
		fprintf(stderr, "Failed to enable events (ret = %i)\n", ret);
		goto end;
	}
	printf("Enabled all system call kernel events\n");

	ret = lttng_start_tracing(session_name);
	if (ret < 0) {
		fprintf(stderr, "Failed to start tracing\n");
		goto end;
	}

	ret = 0;

end:
	lttng_destroy_handle(chan_handle);
	return ret;
}

int main()
{
	int ret = setup_session("test_session", "/workspaces/DiplArbeitContainer/tmp");
    if (ret < 0) {
        fprintf(stderr, "failed setting up: %d\n", ret);
    }
}