/*
 * Session rotation example control application
 *
 * Copyright 2017, Julien Desfossez <jdesfossez@efficios.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/*
 * Compile with:
 *     gcc -o rotate-client rotate-client-example.c -llttng-ctl
 *
 * Run with the following command to rotate the session every second and
 * compress the chunk, until ctrl-c:
 *     ./rotate-client mysession 1 -1 ./rotate-client-compress.sh
 */

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

	dom.type = LTTNG_DOMAIN_KERNEL;
	dom.buf_type = LTTNG_BUFFER_GLOBAL;

	chan_handle = lttng_create_handle(session_name, &dom);
	if (chan_handle == NULL) {
		ret = -1;
		goto end;
	}

	memset(&ev, 0, sizeof(ev));
	ev.type = LTTNG_EVENT_SYSCALL;
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