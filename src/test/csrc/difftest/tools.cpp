#include "tools.h"
#include <queue>

int fifo_handle = -1;

std::queue<Event> event_buf;

int init_fifo_handle(const char* fifo_path) {
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

int init_fifo() {
    return init_fifo_handle(FIFO_PATH);
}


void put_event_in_buf(Event event) {
    event_buf.push(event);
}

int send_event_to_fifo() {
    while (!event_buf.empty()) {
        Event event = event_buf.front();
        int res = write(fifo_handle, &event, sizeof(event));
        if (res < 0) {
            printf("write fifo failed %d\n", res);
            return res;
        };
        std::cout << "[DUT] " << event << std::endl;
        event_buf.pop();
    }
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