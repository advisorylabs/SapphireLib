#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>

namespace sapphirelib::telemetry {

/**
 * @brief Holds the SD card for a moment, so two tasks never use it at once
 *
 * PROS's file functions go straight to the VEX SDK, with nothing between tasks: its SD driver
 * calls vexFileOpen(), vexFileWrite(), vexFileSync() and vexFileClose() bare, and those jump
 * straight into VEXos. Nothing says VEXos's file system copes with a second task arriving
 * mid-write. So every use of the card takes this first: the Logger holds it for each of its
 * writer's passes, and anything else on another task while the Logger runs (the auton selector
 * saving its pick) holds it a chunk at a time, so the Logger's writer is never kept waiting long.
 *
 * Hold it from tasks of your own, not competition tasks: PROS deletes those on a mode change, and
 * a lock held by a deleted task stays held. Logger copes with that (see Logger's writer), others
 * time out
 *
 * @b Example
 * @code {.cpp}
 * sapphirelib::telemetry::SdCardLock card(2000);
 * if (!card.held()) return false; // the card's been busy for 2s
 * std::FILE* file = std::fopen("/usd/NOTES.TXT", "w");
 * // ...
 * @endcode
 */
class SdCardLock {
public:
    /**
     * @brief Wait for the card, up to a timeout
     *
     * @param timeoutMs the longest to wait, in milliseconds. 0xFFFFFFFF waits forever
     */
    explicit SdCardLock(std::uint32_t timeoutMs);

    /**
     * @brief Let the card go, if it was held
     */
    ~SdCardLock();

    SdCardLock(const SdCardLock&) = delete;
    SdCardLock& operator=(const SdCardLock&) = delete;

    /**
     * @brief Whether the card is held: false if the timeout ran out first
     */
    bool held() const;

private:
    bool held_ = false;
};

/** the most of the card readCardFile() and writeCardFile() take at once, in bytes */
inline constexpr std::size_t kCardChunkBytes = 4096;

/**
 * @brief Whether there's an SD card, asked with the card held (even that goes to VEXos's file
 * system)
 *
 * @param waitMs the longest to wait for the card, in milliseconds
 * @return true if there's a card. False if there isn't, or it stayed busy
 */
bool cardInserted(std::uint32_t waitMs);

/**
 * @brief Read a whole file, a chunk at a time, holding the card (SdCardLock) for each
 *
 * @param path the file, e.g. "/usd/TUNE.CFG"
 * @param text set to what's in it, up to one byte past maxBytes
 * @param maxBytes stop reading past this
 * @param waitMs the longest to wait for the card for any one chunk, in milliseconds
 * @return true if it read. False if there's no such file, or the card stayed busy
 */
bool readCardFile(const char* path, std::string& text, std::size_t maxBytes, std::uint32_t waitMs);

/**
 * @brief Write a whole file, replacing it, a chunk at a time, holding the card (SdCardLock) for
 * each so a Logger writing at the same time takes turns with it
 *
 * Each chunk is one write and one sync: the file's buffer is exactly one chunk
 *
 * @param path the file, e.g. "/usd/TUNE.CFG". The folder must exist
 * @param text what to write
 * @param waitMs the longest to wait for the card for any one chunk, in milliseconds
 * @return true if every chunk was written and the file closed
 */
bool writeCardFile(const char* path, std::string_view text, std::uint32_t waitMs);

} // namespace sapphirelib::telemetry
