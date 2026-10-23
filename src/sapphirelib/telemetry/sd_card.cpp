#include "sapphirelib/telemetry/sd_card.hpp"

#include <algorithm>
#include <atomic>
#include <cstdio>
#include <memory>

#include "pros/misc.hpp"
#include "pros/rtos.h"

namespace sapphirelib::telemetry {

namespace {

// made by whichever task gets here first, without a static's initialization guard (which two
// tasks starting at once mustn't rely on): one wins the exchange, any other deletes its spare
pros::mutex_t cardMutex() {
    static std::atomic<pros::mutex_t> mutex{nullptr};
    pros::mutex_t current = mutex.load(std::memory_order_acquire);
    if (current != nullptr) return current;
    pros::mutex_t made = pros::c::mutex_create();
    if (mutex.compare_exchange_strong(current, made, std::memory_order_acq_rel)) return made;
    pros::c::mutex_delete(made);
    return current;
}

} // namespace

SdCardLock::SdCardLock(std::uint32_t timeoutMs) {
    pros::mutex_t mutex = cardMutex();
    held_ = mutex != nullptr && pros::c::mutex_take(mutex, timeoutMs);
}

SdCardLock::~SdCardLock() {
    if (held_) pros::c::mutex_give(cardMutex());
}

bool SdCardLock::held() const { return held_; }

bool cardInserted(std::uint32_t waitMs) {
    const SdCardLock card(waitMs);
    return card.held() && pros::usd::is_installed() != 0;
}

bool readCardFile(const char* path, std::string& text, std::size_t maxBytes, std::uint32_t waitMs) {
    std::FILE* file = nullptr;
    {
        const SdCardLock card(waitMs);
        if (!card.held()) return false;
        file = std::fopen(path, "r");
    }
    if (file == nullptr) return false;
    text.clear();
    const std::unique_ptr<char[]> buffer = std::make_unique<char[]>(kCardChunkBytes);
    bool ok = true;
    while (text.size() <= maxBytes) {
        const SdCardLock card(waitMs);
        if (!card.held()) {
            ok = false;
            break;
        }
        const std::size_t read = std::fread(buffer.get(), 1, kCardChunkBytes, file);
        text.append(buffer.get(), read);
        if (read < kCardChunkBytes) break;
    }
    const SdCardLock card(waitMs);
    // never got the card back to close it: leaking the FILE beats using the card unheld
    if (card.held()) std::fclose(file);
    return ok;
}

bool writeCardFile(const char* path, std::string_view text, std::uint32_t waitMs) {
    // the FILE's buffer: one chunk, so each fwrite() + fflush() is one write to the card, as the
    // Logger's are, rather than newlib splitting it into 1KB writes, each with its own sync
    const std::unique_ptr<char[]> buffer = std::make_unique<char[]>(kCardChunkBytes);
    std::FILE* file = nullptr;
    {
        const SdCardLock card(waitMs);
        if (!card.held()) return false;
        file = std::fopen(path, "w");
        if (file == nullptr) return false;
        std::setvbuf(file, buffer.get(), _IOFBF, kCardChunkBytes);
    }
    bool ok = true;
    for (std::size_t at = 0; ok && at < text.size(); at += kCardChunkBytes) {
        const SdCardLock card(waitMs);
        const std::size_t length = std::min(kCardChunkBytes, text.size() - at);
        ok = card.held() && std::fwrite(text.data() + at, 1, length, file) == length &&
             std::fflush(file) == 0;
    }
    const SdCardLock card(waitMs);
    if (!card.held()) return false; // leaked rather than closed unheld, as in readCardFile()
    return std::fclose(file) == 0 && ok;
}

} // namespace sapphirelib::telemetry
