// Tests for the paho connected() callback deadlock fix.
// See task/race.md §2 for root cause, §4 for fix (Option A).
//
// Invariant under test: MqttCallback::connected() must NOT call publish()
// directly, because paho holds its callback mutex while invoking connected()
// and publish() re-locks that same mutex — under contention this deadlocks.
// The callback must defer the initial publish burst to the main loop via the
// needs_initial_publish flag; the main loop drains the flag and publishes.

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <atomic>
#include <functional>
#include <string>

// Symbols exported from xbot_monitoring.cpp (compiled with XBOT_MONITORING_NO_MAIN).
extern std::atomic<bool> needs_initial_publish;
extern std::function<void(const std::string&, const std::string&, bool)> try_publish_fn;
extern std::function<void(const std::string&, const void*, size_t, bool)> try_publish_binary_fn;
void invoke_connected_for_test();
bool drain_initial_publish_if_needed();

class ConnectedCallbackTest : public ::testing::Test {
 protected:
    static std::atomic<int> publish_count;

    void SetUp() override {
        publish_count.store(0);
        try_publish_fn = [](const std::string&, const std::string&, bool) {
            publish_count.fetch_add(1);
        };
        try_publish_binary_fn = [](const std::string&, const void*, std::size_t, bool) {
            publish_count.fetch_add(1);
        };
        needs_initial_publish.store(false);
    }
};

std::atomic<int> ConnectedCallbackTest::publish_count{0};

// RED without the fix, GREEN with it.
TEST_F(ConnectedCallbackTest, ConnectedMustNotPublish) {
    invoke_connected_for_test();
    EXPECT_EQ(publish_count.load(), 0)
        << "MqttCallback::connected() invoked publish() "
        << publish_count.load()
        << " times; paho's callback contract forbids this (see task/race.md).";
}

TEST_F(ConnectedCallbackTest, ConnectedSetsInitialPublishFlag) {
    invoke_connected_for_test();
    EXPECT_TRUE(needs_initial_publish.load())
        << "connected() must signal the main loop to perform the initial publish burst.";
}

TEST_F(ConnectedCallbackTest, DrainPublishesWhenFlagSet) {
    needs_initial_publish.store(true);
    const bool drained = drain_initial_publish_if_needed();
    EXPECT_TRUE(drained);
    EXPECT_GT(publish_count.load(), 0)
        << "drain_initial_publish_if_needed() must issue the deferred publishes.";
    EXPECT_FALSE(needs_initial_publish.load())
        << "drain must consume the flag.";
}

TEST_F(ConnectedCallbackTest, DrainNoopWhenFlagClear) {
    needs_initial_publish.store(false);
    const bool drained = drain_initial_publish_if_needed();
    EXPECT_FALSE(drained);
    EXPECT_EQ(publish_count.load(), 0);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
