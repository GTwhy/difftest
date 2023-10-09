#include "tools.h"

int fifo_handle = -1;

int init_fifo(const char* fifo_path) {
    // delete fifo if exists
    if (access(fifo_path, F_OK) == 0) {
        unlink(fifo_path);
    }

    mkfifo(fifo_path, 0666);

    fifo_handle = open(fifo_path, O_WRONLY);
    if (fifo_handle < 0) {
        printf("open fifo failed %d \n", fifo_handle);
        return fifo_handle;
    }

    return fifo_handle;
}

int init_fifo_handle() {
    fifo_handle = init_fifo(FIFO_PATH);
    return fifo_handle;
}

int send_event_to_fifo(Event event) {
    if (fifo_handle == -1) {
      init_fifo_handle();
    }
    int res = write(fifo_handle, &event, sizeof(event));
    if (res < 0) {
        printf("write fifo failed %d\n", res);
        return res;
    };
    return 0;
}

int close_fifo(const char* fifo_path, int fifoHandle) {
    
    int res = close(fifo_handle);
    res = unlink(fifo_path);
    if (res < 0) {
        printf("close or unlink fifo failed");
        return -1;
    }
    return 0;
}