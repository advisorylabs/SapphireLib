#include "sapphirelib/telemetry/file_naming.hpp"

namespace sapphirelib::telemetry {

namespace {

// "SL" + six digits + ".CSV"
constexpr std::size_t kNameLength = 12;

char upper(char c) { return c >= 'a' && c <= 'z' ? static_cast<char>(c - 'a' + 'A') : c; }

bool isDigit(char c) { return c >= '0' && c <= '9'; }

} // namespace

bool formatLogFileName(char* out, std::size_t size, std::uint32_t index) {
    if (index > kMaxLogIndex || size < kNameLength + 1) return false;
    out[0] = 'S';
    out[1] = 'L';
    for (int i = 7; i >= 2; --i) {
        out[i] = static_cast<char>('0' + index % 10);
        index /= 10;
    }
    out[8] = '.';
    out[9] = 'C';
    out[10] = 'S';
    out[11] = 'V';
    out[12] = '\0';
    return true;
}

bool formatLogFilePath(char* out, std::size_t size, std::string_view directory,
                       std::uint32_t index) {
    while (!directory.empty() && directory.back() == '/') directory.remove_suffix(1);
    // directory + '/' + name + NUL
    if (index > kMaxLogIndex || size < directory.size() + 1 + kNameLength + 1) return false;
    std::size_t length = directory.copy(out, directory.size());
    out[length++] = '/';
    return formatLogFileName(out + length, size - length, index);
}

std::int32_t parseLogFileIndex(std::string_view name) {
    const std::size_t slash = name.find_last_of("/\\");
    if (slash != std::string_view::npos) name.remove_prefix(slash + 1);
    if (name.size() != kNameLength) return -1;
    if (upper(name[0]) != 'S' || upper(name[1]) != 'L' || name[8] != '.' || upper(name[9]) != 'C' ||
        upper(name[10]) != 'S' || upper(name[11]) != 'V') {
        return -1;
    }
    std::int32_t index = 0;
    for (std::size_t i = 2; i < 8; ++i) {
        if (!isDigit(name[i])) return -1;
        index = index * 10 + (name[i] - '0');
    }
    return index;
}

std::int32_t highestLogFileIndex(std::string_view listing) {
    std::int32_t highest = -1;
    while (!listing.empty()) {
        const std::size_t newline = listing.find('\n');
        std::string_view line = listing.substr(0, newline);
        listing.remove_prefix(newline == std::string_view::npos ? listing.size() : newline + 1);
        while (!line.empty() && (line.back() == '\r' || line.back() == ' ')) line.remove_suffix(1);
        const std::int32_t index = parseLogFileIndex(line);
        if (index > highest) highest = index;
    }
    return highest;
}

std::string_view listingPath(std::string_view directory) {
    constexpr std::string_view kPrefix = "/usd";
    if (directory.substr(0, kPrefix.size()) == kPrefix &&
        (directory.size() == kPrefix.size() || directory[kPrefix.size()] == '/')) {
        directory.remove_prefix(kPrefix.size());
    }
    while (directory.size() > 1 && directory.back() == '/') directory.remove_suffix(1);
    return directory.empty() ? std::string_view("/") : directory;
}

} // namespace sapphirelib::telemetry
