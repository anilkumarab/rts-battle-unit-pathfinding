#pragma once

// A deliberately tiny test framework: no external dependency to fetch or
// build, just enough to write clear, readable TDD-style tests. Not meant to
// compete with GoogleTest/Catch2 in a real production codebase — but it
// keeps this assessment's build trivially reproducible on any machine with
// a C++17 compiler and CMake, which matters more here than test-framework
// features.

#include <functional>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace minitest {

struct TestCase {
    std::string name;
    std::function<void()> fn;
};

inline std::vector<TestCase>& registry() {
    static std::vector<TestCase> tests;
    return tests;
}

struct Registrar {
    Registrar(const std::string& name, std::function<void()> fn) {
        registry().push_back({name, std::move(fn)});
    }
};

struct AssertionFailure {
    std::string message;
};

inline int runAll() {
    int passed = 0, failed = 0;
    for (const auto& test : registry()) {
        try {
            test.fn();
            std::cout << "[PASS] " << test.name << "\n";
            ++passed;
        } catch (const AssertionFailure& f) {
            std::cout << "[FAIL] " << test.name << " - " << f.message << "\n";
            ++failed;
        } catch (const std::exception& e) {
            std::cout << "[FAIL] " << test.name << " - unexpected exception: " << e.what() << "\n";
            ++failed;
        }
    }
    std::cout << "\n" << passed << " passed, " << failed << " failed, out of " << registry().size() << " total.\n";
    return failed == 0 ? 0 : 1;
}

} // namespace minitest

#define TEST(name)                                                                     \
    static void name();                                                                \
    static minitest::Registrar registrar_##name(#name, name);                          \
    static void name()

#define ASSERT_TRUE(cond)                                                              \
    do {                                                                               \
        if (!(cond)) {                                                                 \
            std::ostringstream oss;                                                    \
            oss << "ASSERT_TRUE failed: " #cond " at " << __FILE__ << ":" << __LINE__; \
            throw minitest::AssertionFailure{oss.str()};                               \
        }                                                                              \
    } while (0)

#define ASSERT_FALSE(cond) ASSERT_TRUE(!(cond))

#define ASSERT_EQ(a, b)                                                                \
    do {                                                                               \
        if (!((a) == (b))) {                                                           \
            std::ostringstream oss;                                                    \
            oss << "ASSERT_EQ failed: " #a " != " #b " at " << __FILE__ << ":" << __LINE__; \
            throw minitest::AssertionFailure{oss.str()};                               \
        }                                                                              \
    } while (0)
