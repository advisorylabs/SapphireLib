#!/usr/bin/env python3
"""Checks that every SD card call in the program holds telemetry::SdCardLock.

PROS's file functions reach VEXos with nothing between tasks: its SD driver (usd_driver.c in
libpros.a) calls vexFileOpen(), vexFileWrite(), vexFileSync() and vexFileClose() bare, and those
are jumps straight into VEXos. Whether VEXos's file system copes with a second task arriving
mid-write is VEXos's business, so the program never lets it happen: the Logger holds SdCardLock
for each of its writer's passes, and everything else (TUNE.CFG, the auton selector's pick) holds
it a chunk at a time through telemetry::readCardFile() and writeCardFile(). This script fails if a card call
turns up where the lock isn't held, so a new one can't slip in unnoticed.

A call counts as held if an SdCardLock is declared, earlier in the same function, in a scope
still open at the call; or the function is one the Logger's writer pass reaches, all of which run
inside writerPass()'s hold (WRITER_PASS below).

Run:  python tools/telemetry/check_sd_access.py            (checks src/)
      python tools/telemetry/check_sd_access.py --list     (and lists every card call, and how)
      python tools/telemetry/check_sd_access.py --selftest
"""

import re
import sys
from pathlib import Path

# anything that goes to the card: stdio on a FILE, and PROS's own card calls
CARD_CALL = re.compile(
    r"\b(?:std::)?(fopen|freopen|fread|fwrite|fflush|fclose|fseek|ftell|fgets|fputs|fgetc|fputc"
    r"|rewind)\s*\(|\bfprintf\s*\(\s*(?!stdout|stderr)|\busd::(is_installed|list_files)\s*\("
    r"|\bvexFile\w+\s*\(")
LOCK = re.compile(r"\bSdCardLock\s+\w+\s*[({]")
NAMESPACE = re.compile(r"\bnamespace\b[^;{]*$")

# Logger functions only ever called from Logger::writerPass() (directly or through each other),
# which holds the card for its whole pass. Keep in step with src/sapphirelib/telemetry/logger.cpp
WRITER_PASS = {
    "src/sapphirelib/telemetry/logger.cpp": {
        "Logger::closeFile", "Logger::manageFile", "Logger::tryOpen", "Logger::openIn",
        "Logger::writeStaged", "Logger::fault",
    },
}

# ...and how the pass reaches them: every function on the way, each of which may only be called
# from another of these (or writerPass() from writerLoop()), which check_reach() proves from the
# call sites rather than trusting this list
WRITER_REACH = {
    "src/sapphirelib/telemetry/logger.cpp": {
        "entry": ("Logger::writerLoop", "Logger::writerPass"),
        "inside": {"Logger::writerPass", "Logger::stage", "Logger::announceChannels",
                   "Logger::drain", "Logger::writeHealth"},
    },
}


def strip(text):
    """The source with comments, string and character literals blanked, lines kept."""
    out = []
    i = 0
    n = len(text)
    while i < n:
        c = text[i]
        if text.startswith("//", i):
            j = text.find("\n", i)
            i = n if j < 0 else j
        elif text.startswith("/*", i):
            j = text.find("*/", i + 2)
            j = n if j < 0 else j + 2
            out.append("".join(ch if ch == "\n" else " " for ch in text[i:j]))
            i = j
        elif c in "\"'":
            j = i + 1
            while j < n and text[j] != c:
                j += 2 if text[j] == "\\" else 1
            out.append(c + " " * max(0, j - i - 1) + c)
            i = j + 1
        else:
            out.append(c)
            i += 1
    return "".join(out)


def check(source, allowed=frozenset(), found=None):
    """Each unheld card call in a C++ source: (line, function, call). Every call, held or not,
    goes in `found` too if it's given, with how: (line, function, call, 'lock'|'writer'|None)."""
    code = strip(source)
    problems = []
    depth = 0  # braces inside functions and types, namespaces not counted
    namespaces = []  # depth stack entries that are namespaces
    pending_namespace = False
    function = None
    header = ""  # the text since the last top-level ';' or '}', for a function's name
    locks = []  # depths of the SdCardLocks in scope
    line = 1
    i = 0
    while i < len(code):
        c = code[i]
        if c == "\n":
            line += 1
        if c == "{":
            if pending_namespace or (depth == 0 and NAMESPACE.search(header)):
                namespaces.append(True)
                pending_namespace = False
            else:
                namespaces.append(False)
                if depth == 0:
                    names = re.findall(r"([\w:~]+)\s*\(", header)
                    function = names[0] if names else None
                depth += 1
            header = ""
        elif c == "}":
            is_namespace = namespaces.pop() if namespaces else False
            if not is_namespace:
                depth -= 1
                locks = [d for d in locks if d <= depth]
                if depth == 0:
                    function = None
            header = ""
        elif c == ";" and depth == 0:
            header = ""
        else:
            if depth == 0:
                header += c
            match = None
            if depth > 0:
                lock = LOCK.match(code, i)
                if lock:
                    locks.append(depth)
                    i = lock.end()
                    continue
                match = CARD_CALL.match(code, i)
            # not the tail of a longer name (myfread); a qualified one (pros::usd::) is fine
            if match and (i == 0 or not (code[i - 1].isalnum() or code[i - 1] == "_")):
                how = "lock" if locks else ("writer" if function in allowed else None)
                call = match.group(0).strip(" (")
                if found is not None:
                    found.append((line, function, call, how))
                if how is None:
                    problems.append((line, function, call))
                i = match.end()
                continue
        i += 1
    return problems


def functions(source):
    """Each function body at namespace scope: (start, end, name), as offsets into strip(source)."""
    code = strip(source)
    out = []
    depth = 0
    namespaces = []
    header = ""
    start = 0
    name = None
    for i, c in enumerate(code):
        if c == "{":
            if depth == 0 and NAMESPACE.search(header):
                namespaces.append(True)
            else:
                namespaces.append(False)
                if depth == 0:
                    names = re.findall(r"([\w:~]+)\s*\(", header)
                    name = names[0] if names else None
                    start = i
                depth += 1
            header = ""
        elif c == "}":
            if not (namespaces.pop() if namespaces else False):
                depth -= 1
                if depth == 0:
                    out.append((start, i, name))
            header = ""
        elif c == ";" and depth == 0:
            header = ""
        elif depth == 0:
            header += c
    return code, out


def check_reach(source, reach, card_functions):
    """Calls into the writer pass's functions from anywhere else: (line, caller, callee)."""
    code, bodies = functions(source)
    entry_caller, entry = reach["entry"]
    pass_functions = set(reach["inside"]) | set(card_functions)
    callees = {name.split("::")[-1]: name for name in pass_functions}
    problems = []
    for start, end, caller in bodies:
        for match in re.finditer(r"(?<![\w.>])(" + "|".join(callees) + r")\s*[(<]",
                                 code[start:end]):
            callee = callees[match.group(1)]
            allowed = caller in pass_functions or (caller == entry_caller and callee == entry)
            if not allowed:
                line = code.count("\n", 0, start + match.start()) + 1
                problems.append((line, caller, callee))
    return problems


def check_tree(root, listing=None):
    problems = []
    for path in sorted((root / "src").rglob("*.cpp")):
        rel = path.relative_to(root).as_posix()
        text = path.read_text(encoding="utf-8").replace("\r\n", "\n")
        found = []
        for line, function, call in check(text, frozenset(WRITER_PASS.get(rel, ())), found):
            problems.append(f"{rel}:{line}: {call}() in {function or 'namespace scope'} "
                            "without SdCardLock held")
        if rel in WRITER_REACH:
            for line, caller, callee in check_reach(text, WRITER_REACH[rel], WRITER_PASS[rel]):
                problems.append(f"{rel}:{line}: {callee}() called from {caller}, outside "
                                "Logger::writerPass()'s hold of the card")
            # and the pass itself holds the card
            code, bodies = functions(text)
            passes = [code[s:e] for s, e, name in bodies if name == WRITER_REACH[rel]["entry"][1]]
            if not passes or not LOCK.search(passes[0]):
                problems.append(f"{rel}: {WRITER_REACH[rel]['entry'][1]}() doesn't hold SdCardLock")
        if listing is not None:
            for line, function, call, how in found:
                held = {"lock": "SdCardLock in scope", "writer": "inside Logger::writerPass()'s hold",
                        None: "NOT HELD"}[how]
                listing.append(f"{rel}:{line}: {call}() in {function}: {held}")
    return problems


def selftest():
    held = """
namespace a {
namespace {
bool readIt(const char* path) {
    const SdCardLock card(2000);
    if (!card.held()) return false;
    std::FILE* f = std::fopen(path, "r"); // fclose( in a comment
    std::fclose(f);
    return true;
}
} // namespace
} // namespace a
"""
    assert check(held) == [], check(held)

    # one held, one not: the lock's scope closed before the second call
    scoped = """
void save() {
    std::FILE* f = nullptr;
    {
        const SdCardLock card(10);
        f = std::fopen("/usd/X", "w");
    }
    std::fwrite("x", 1, 1, f);
}
"""
    assert check(scoped) == [(8, "save", "std::fwrite")], check(scoped)

    # no lock at all, a string that only looks like a call, PROS's card calls, terminal output
    bare = """
struct Thing {
    int x;
};
bool Thing::loaded() {
    printf("fopen(x)\\n");
    fprintf(stdout, "%d", 1);
    if (pros::usd::is_installed() == 0) return false;
    return vexFileOpen("x", "r") != nullptr;
}
"""
    got = check(bare)
    assert [(p[1], p[2]) for p in got] == [("Thing::loaded", "usd::is_installed"),
                                           ("Thing::loaded", "vexFileOpen")], got

    # a function the writer pass reaches is allowed; the same call elsewhere isn't
    writer = """
bool Logger::writeStaged() { return std::fflush(file_) == 0; }
bool Logger::somethingElse() { return std::fflush(file_) == 0; }
"""
    got = check(writer, frozenset({"Logger::writeStaged"}))
    assert [(p[1], p[2]) for p in got] == [("Logger::somethingElse", "std::fflush")], got

    # a lambda's lock covers its own body only
    lam = """
void f() {
    const bool ok = [] {
        const SdCardLock card(10);
        return pros::usd::is_installed() != 0;
    }();
    std::fclose(file);
}
"""
    got = check(lam)
    assert [(p[1], p[2]) for p in got] == [("f", "std::fclose")], got
    # the writer pass's functions may only be reached from inside it
    reach = {"entry": ("Logger::writerLoop", "Logger::writerPass"),
             "inside": {"Logger::writerPass", "Logger::stage"}}
    logger = """
void Logger::writerLoop() { while (true) writerPass(); }
void Logger::writerPass() { const SdCardLock card(1000); stage(1); writeStaged(); }
template <typename F> bool Logger::stage(F f) { return writeStaged(); }
bool Logger::writeStaged() { return std::fflush(file_) == 0; }
void Logger::stop() { writeStaged(); }
"""
    got = check_reach(logger, reach, {"Logger::writeStaged"})
    assert got == [(6, "Logger::stop", "Logger::writeStaged")], got
    print("check_sd_access selftest: all checks passed")


def main():
    if len(sys.argv) == 2 and sys.argv[1] == "--selftest":
        selftest()
        return 0
    root = Path(__file__).resolve().parents[2]
    listing = [] if "--list" in sys.argv else None
    problems = check_tree(root, listing)
    for entry in listing or []:
        print(entry)
    for problem in problems:
        print(problem)
    if problems:
        print(f"{len(problems)} SD card call(s) without SdCardLock held: see "
              "include/sapphirelib/telemetry/sd_card.hpp")
        return 1
    print("every SD card call in src/ holds SdCardLock")
    return 0


if __name__ == "__main__":
    sys.exit(main())
