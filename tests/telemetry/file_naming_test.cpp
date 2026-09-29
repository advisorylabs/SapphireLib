// Host-side unit test for sapphirelib::telemetry's log file naming — no
// PROS/embedded dependencies, so it builds and runs with a normal desktop
// compiler.
//
// Build & run:
// clang-format off
//   g++ -std=c++20 -Wall -Wextra -Iinclude tests/telemetry/file_naming_test.cpp src/sapphirelib/telemetry/file_naming.cpp -o file_naming_test && ./file_naming_test
// clang-format on

#include <cassert>
#include <cstdio>
#include <cstring>
#include <string>

#include "sapphirelib/telemetry/file_naming.hpp"

using sapphirelib::telemetry::formatLogFileName;
using sapphirelib::telemetry::formatLogFilePath;
using sapphirelib::telemetry::highestLogFileIndex;
using sapphirelib::telemetry::kMaxLogIndex;
using sapphirelib::telemetry::listingPath;
using sapphirelib::telemetry::parseLogFileIndex;

namespace {

void testFormatAndParseRoundTrip() {
    char name[13];
    assert(formatLogFileName(name, sizeof(name), 42));
    assert(std::strcmp(name, "SL000042.CSV") == 0);
    assert(formatLogFileName(name, sizeof(name), 0));
    assert(std::strcmp(name, "SL000000.CSV") == 0);
    assert(formatLogFileName(name, sizeof(name), kMaxLogIndex));
    assert(std::strcmp(name, "SL999999.CSV") == 0);
    for (std::uint32_t index : {0u, 1u, 9u, 10u, 42u, 12345u, 999999u}) {
        assert(formatLogFileName(name, sizeof(name), index));
        assert(parseLogFileIndex(name) == static_cast<std::int32_t>(index));
    }
}

void testFormatRefusals() {
    char name[13] = "untouched";
    assert(!formatLogFileName(name, sizeof(name), kMaxLogIndex + 1)); // 1000000
    assert(!formatLogFileName(name, 12, 42));                         // no room for the NUL
    assert(std::strcmp(name, "untouched") == 0);
}

void testParseIsCaseInsensitiveAndStripsPaths() {
    assert(parseLogFileIndex("sl000042.csv") == 42);
    assert(parseLogFileIndex("Sl000042.cSv") == 42);
    assert(parseLogFileIndex("/usd/sl/SL000042.CSV") == 42);
    assert(parseLogFileIndex("/sl/SL000007.CSV") == 7);
    assert(parseLogFileIndex("\\sl\\SL000007.CSV") == 7);
}

void testParseRejectsNearMisses() {
    // An unrelated file must never push the index.
    assert(parseLogFileIndex("SL00042.CSV") == -1);   // five digits
    assert(parseLogFileIndex("SL0000042.CSV") == -1); // seven digits
    assert(parseLogFileIndex("SL000042.TXT") == -1);
    assert(parseLogFileIndex("XSL000042.CSV") == -1);
    assert(parseLogFileIndex("SL00004A.CSV") == -1);
    assert(parseLogFileIndex("SL000042CSV") == -1);
    assert(parseLogFileIndex("SL000042.CSV ") == -1);
    assert(parseLogFileIndex("SL-00042.CSV") == -1);
    assert(parseLogFileIndex("") == -1);
    assert(parseLogFileIndex("/usd/sl/") == -1);
}

void testHighestIndexFromListing() {
    assert(highestLogFileIndex("") == -1);
    assert(highestLogFileIndex("notes.txt\nconfig.json\n") == -1);
    assert(highestLogFileIndex("SL000001.CSV\nSL000003.CSV\nSL000002.CSV\n") == 3);
    // Windows line endings, non-log files mixed in, no trailing newline.
    assert(highestLogFileIndex("SL000010.CSV\r\nREADME.TXT\r\nsl000011.csv\r\nSL00099.CSV") == 11);
    // Order doesn't matter; the highest wins.
    assert(highestLogFileIndex("SL000500.CSV\nSL000020.CSV\n") == 500);
    // A listing cut short mid-name (small buffer) just can't count that name.
    assert(highestLogFileIndex("SL000004.CSV\nSL0000") == 4);
    assert(highestLogFileIndex("SL999999.CSV\n") == 999999);
}

void testFullPath() {
    char path[64];
    assert(formatLogFilePath(path, sizeof(path), "/usd/sl", 42));
    assert(std::strcmp(path, "/usd/sl/SL000042.CSV") == 0);
    assert(formatLogFilePath(path, sizeof(path), "/usd/sl/", 42));
    assert(std::strcmp(path, "/usd/sl/SL000042.CSV") == 0);
    assert(formatLogFilePath(path, sizeof(path), "/usd", 7));
    assert(std::strcmp(path, "/usd/SL000007.CSV") == 0);
    assert(!formatLogFilePath(path, sizeof(path), "/usd", kMaxLogIndex + 1));
    // Exactly enough room, then one byte short.
    const std::string expected = "/usd/sl/SL000042.CSV";
    assert(formatLogFilePath(path, expected.size() + 1, "/usd/sl", 42));
    assert(!formatLogFilePath(path, expected.size(), "/usd/sl", 42));
}

void testListingPath() {
    assert(listingPath("/usd/sl") == "/sl");
    assert(listingPath("/usd/sl/") == "/sl");
    assert(listingPath("/usd/logs/run") == "/logs/run");
    assert(listingPath("/usd") == "/");
    assert(listingPath("/usd/") == "/");
    assert(listingPath("/sl") == "/sl");           // already relative to the card
    assert(listingPath("/usdlogs") == "/usdlogs"); // not the /usd prefix
    assert(listingPath("") == "/");
}

} // namespace

int main() {
    testFormatAndParseRoundTrip();
    testFormatRefusals();
    testParseIsCaseInsensitiveAndStripsPaths();
    testParseRejectsNearMisses();
    testHighestIndexFromListing();
    testFullPath();
    testListingPath();
    std::printf("file_naming_test: all tests passed\n");
    return 0;
}
