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

// Constants
constexpr uint64_t ROB_IDX_MUSK = 0xffff;
constexpr uint64_t REG_MUSK = 0xff;

// Enum class for RegOffset
enum class RegOffset {
    Lsrc0 = 0,
    Lsrc1 = 8,
    Lsrc2 = 16,
    Psrc0 = 24,
    Psrc1 = 32,
    Psrc2 = 40,
    Ldst0 = 48,
    Pdst0 = 56
};

// Enum class for XOffset
enum class XOffset {
    IsMMIO = 16,
    IsLrSc = 17,
    IsStore = 18,
    IsLoad = 19,
    IsRobIdx = 20
};


enum class EventType {
    Invalid, // Default
    StoreCommit,
    StoreLocal,
    StoreGlobal,
    LoadGlobal,
    LoadLocal,
    LoadCommit,
    Fence,
    Dependency,
};

inline EventType defaultEventType() {
    return EventType::Invalid;
}

struct Event {
    EventType ty = defaultEventType();
    uint32_t core_id = 0;
    uint64_t addr = 0;
    uint8_t value = 0;
    uint64_t x = 0;
    uint64_t time = 0;

    // Overloading the equality operator for convenience
    bool operator==(const Event& other) const {
        return ty == other.ty && core_id == other.core_id && addr == other.addr &&
               value == other.value  && x == other.x && time == other.time;
    }

    // Overloading the not equal operator for convenience
    bool operator!=(const Event& other) const {
        return !(*this == other);
    }

    // New constructor to make it easier to create events
    Event(EventType ty, uint32_t core_id, uint64_t addr, uint8_t value, uint64_t x, uint64_t time) // Changed parameter type for value
        : ty(ty), core_id(core_id), addr(addr), value(value), x(x), time(time) {}
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
        case EventType::Fence: os << "Fence"; break;
        case EventType::Dependency: os << "Dependency"; break;
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
       << "x: 0x" << std::hex << std::setw(16) << std::setfill('0') << event.x << ", "
       << "time: " << std::dec << event.time
       << " }";
    return os;
}
void mcm_checker_init();
void mcm_event_push(Event &event);
void mcm_check();
void mcm_checker_destroy();

#endif // __TOOLS_H__