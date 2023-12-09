#include "tools.h"
#include <cassert>
#include <queue>
#include <dlfcn.h>

#define MCM_CHECKER_LIB
// #define MCM_CHECKER_DEBUG

int fifo_handle = -1;

void* checker = nullptr;

std::queue<Event> event_buf;

pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

bool debug_output_flag = false;
bool mcm_lib_flag = false;
bool mcm_fifo_flag = false;

extern "C" {
    typedef void* (*checker_new_t)(unsigned int core_num);
    typedef char* (*checker_check_t)(void* checker_ptr, const Event* event);
    // typedef void (*checker_free_t)(void* ptr);
    // typedef void (*string_free_t)(char* s);
}

checker_new_t checker_new;
checker_check_t checker_check;

// checker_free_t checker_free;
// string_free_t string_free;

void mcm_checker_lib_init() {
    std::string mcm_home = std::string(getenv("MCM_HOME"));
    std::string lib_path = mcm_home + "/target/debug/libmcm_checker.so";
    void* handle = dlmopen(LM_ID_NEWLM, lib_path.c_str(), RTLD_LAZY | RTLD_DEEPBIND);
    // void* handle = dlopen("/home/why/zspace/cpu/xiangshan/xs-env/mcm-checker/target/debug/libmcm_checker.so", RTLD_LAZY | RTLD_DEEPBIND);


    if (!handle) {
        std::cerr << "Cannot open library: " << dlerror() << '\n';
        exit(1);
    }

    checker_new = (checker_new_t)dlsym(handle, "mcm_checker_new");
    assert(checker_new && "Function checker_new not found");

    checker_check = (checker_check_t)dlsym(handle, "mcm_checker_check");
    assert(checker_check && "Function checker_check not found");

    // checker_free = (checker_free_t)dlsym(handle, "mcm_checker_free");
    // assert(checker_free && "Function checker_free not found");

    // string_free = (string_free_t)dlsym(handle, "mcm_string_free");
    // assert(string_free && "Function string_free not found");
    checker = checker_new(NUM_CORES);
}



void mcm_checker_fifo_init(const char* fifo_path) {
    // delete fifo if exists
    if (access(fifo_path, F_OK) == 0) {
        unlink(fifo_path);
    }

    mkfifo(fifo_path, 0666);
    fifo_handle = open(fifo_path, O_WRONLY);
    if (fifo_handle < 0) {
        std::cerr << "open fifo failed" << std::endl;
        exit(-1);
    }
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

void mcm_checker_init() {
    if (getenv("MCM_CHECKER_LIB") != nullptr) {
        mcm_lib_flag = true;
        mcm_checker_lib_init();
    }
    if (getenv("MCM_CHECKER_FIFO") != nullptr) {
        mcm_fifo_flag = true;
        mcm_checker_fifo_init(FIFO_PATH);
    }
    if (getenv("DEBUG_OUTPUT") != nullptr) {
        debug_output_flag = true;
    }
}

void mcm_event_push(Event &event) {
    pthread_mutex_lock(&mutex);
    event_buf.push(event);
    pthread_mutex_unlock(&mutex);
}

int err_cnt = 0;

void mcm_check(){
    pthread_mutex_lock(&mutex);
    if (err_cnt > 0 && err_cnt++ == 100) {
        printf("exit after 100 cycles after error\n");
        exit(-1);
    }
    while (!event_buf.empty()) {
        Event event = event_buf.front();

        if (debug_output_flag) {
            std::cout << "[DUT] " << event << std::endl;
        }

        if (mcm_fifo_flag) {
            int res = write(fifo_handle, &event, sizeof(event));
            if (res < 0) {
                std::cerr << "write fifo failed" << std::endl;
                exit(-1);
            };
        }

        if (mcm_lib_flag) {
            char* res = checker_check(checker, &event);
            if (res != nullptr) {
                printf("[CHECKER] %s\n", res);
                // TODO: exit?
                err_cnt++;
            }
        }

        event_buf.pop();
    }
    pthread_mutex_unlock(&mutex);
}