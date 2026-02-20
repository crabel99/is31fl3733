#include <Arduino.h>
#include <unity_fixture.h>

static void RunAllTests(void) {
    RUN_TEST_GROUP(EmbeddedBasic);
    RUN_TEST_GROUP(EmbeddedAdvanced);
}

void setup() {
    const char *testName[] = {"Embedded", "-v"};
    UnityMain(2, testName, RunAllTests);
}

void loop() {}
