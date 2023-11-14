// #ifndef __TOOLS_H__
// #define __TOOLS_H__

// #include <iostream>
// #include <fcntl.h>
// #include <sys/stat.h>
// #include <unistd.h>
// #include <cstdint>

// #define FIFO_PATH "/tmp/mcm_fifo"

// extern int fifo_handle;

// enum class EventType {
//     Invalid,
//     LoadCommit,
//     LoadComplete,
//     LoadSquash,
//     StoreAvailable,
//     StoreCommit,
//     StoreComplete,
//     StoreSquash,
//     StoreRemoteCommit,
//     StoreRemoteComplete,
// };

// inline EventType defaultEventType() {
//     return EventType::Invalid;
// }

// struct Event {
//     EventType ty = defaultEventType();
//     uint32_t core_id = 0;
//     uint64_t addr = 0;
//     uint64_t value = 0;
//     uint64_t time = 0;

//     // Overloading the equality operator for convenience
//     bool operator==(const Event& other) const {
//         return ty == other.ty && core_id == other.core_id && addr == other.addr &&
//                value == other.value && time == other.time;
//     }

//     // Overloading the not equal operator for convenience
//     bool operator!=(const Event& other) const {
//         return !(*this == other);
//     }

//     // New constructor to make it easier to create events
//     Event(EventType ty, uint32_t core_id, uint64_t addr, uint64_t value, uint64_t time)
//         : ty(ty), core_id(core_id), addr(addr), value(value), time(time) {}
// };

// // The following is just to provide the equivalent of Rust's Debug trait
// inline std::ostream& operator<<(std::ostream& os, const EventType& eventType) {
//     switch (eventType) {
//         case EventType::Invalid: os << "Invalid"; break;
//         case EventType::LoadCommit: os << "LoadCommit"; break;
//         case EventType::LoadComplete: os << "LoadComplete"; break;
//         case EventType::LoadSquash: os << "LoadSquash"; break;
//         case EventType::StoreAvailable: os << "StoreAvailable"; break;
//         case EventType::StoreCommit: os << "StoreCommit"; break;
//         case EventType::StoreComplete: os << "StoreComplete"; break;
//         case EventType::StoreSquash: os << "StoreSquash"; break;
//         case EventType::StoreRemoteCommit: os << "StoreRemoteCommit"; break;
//         case EventType::StoreRemoteComplete: os << "StoreRemoteComplete"; break;
//         default: os << "Unknown EventType";
//     }
//     return os;
// }

// inline std::ostream& operator<<(std::ostream& os, const Event& event) {
//     os << "Event { "
//        << "ty: " << event.ty << ", "
//        << "core_id: " << event.core_id << ", "
//        << "addr: " << event.addr << ", "
//        << "value: " << event.value << ", "
//        << "time: " << event.time
//        << " }";
//     return os;
// }

// int init_fifo(const char* fifo_path);
// int send_event_to_fifo(Event event);
// int close_fifo(const char* fifo_path, int fifoHandle);

// #endif // __TOOLS_H__

#ifndef __TOOLS_H__
#define __TOOLS_H__

#include <iostream>
#include <iomanip>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <cstdint>

#define FIFO_PATH "/tmp/mcm_fifo"

extern int fifo_handle;

enum class EventType {
    Invalid, // Default
    StoreCommit,
    StoreLocal,
    StoreGlobal,
    LoadGlobal,
    LoadLocal,
    LoadCommit,
};

inline EventType defaultEventType() {
    return EventType::Invalid;
}

struct Event {
    EventType ty = defaultEventType();
    uint32_t core_id = 0;
    uint64_t addr = 0;
    uint8_t value = 0;
    uint64_t time = 0;

    // Overloading the equality operator for convenience
    bool operator==(const Event& other) const {
        return ty == other.ty && core_id == other.core_id && addr == other.addr &&
               value == other.value && time == other.time;
    }

    // Overloading the not equal operator for convenience
    bool operator!=(const Event& other) const {
        return !(*this == other);
    }

    // New constructor to make it easier to create events
    Event(EventType ty, uint32_t core_id, uint64_t addr, uint8_t value, uint64_t time) // Changed parameter type for value
        : ty(ty), core_id(core_id), addr(addr), value(value), time(time) {}
};

inline std::ostream& operator<<(std::ostream& os, const EventType& eventType) {
    switch (eventType) {
        case EventType::Invalid: os << "Invalid"; break;
        case EventType::StoreCommit: os << "StoreCommit"; break;
        case EventType::StoreLocal: os << "StoreLocal"; break;
        case EventType::StoreGlobal: os << "StoreGlobal"; break;
        case EventType::LoadGlobal: os << "LoadGlobal"; break;
        case EventType::LoadLocal: os << "LoadLocal"; break;
        case EventType::LoadCommit: os << "LoadCommit"; break;
        default: os << "Unknown EventType";
    }
    return os;
}
inline std::ostream& operator<<(std::ostream& os, const Event& event) {

    os << "Event { "
       << "ty: " << event.ty << ", "
       << "core_id: " << event.core_id << ", "
       << "addr: 0x" << std::hex << std::setw(16) << std::setfill('0') << event.addr << ", "
       << "value: 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<uint64_t>(event.value) << ", "
       << "time: " << std::dec << event.time
       << " }";
    return os;
}
int init_fifo(const char* fifo_path);
int send_event_to_fifo(Event event);
int close_fifo(const char* fifo_path, int fifoHandle);

#endif // __TOOLS_H__