#include "pipe.hpp"
#include "datamgmt/common.hpp"

#include <unistd.h>
#include <fcntl.h>

int getPipe(int p[2], bool blocking = false) {
    if (int ret = pipe(p) != 0) {
        return ret;
    }

    if (!blocking) {
        int retval = fcntl( p[0], F_SETFL, fcntl(p[0], F_GETFL) | O_NONBLOCK);
    }
}