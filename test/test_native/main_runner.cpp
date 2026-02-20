/*---------------------------------------------------------------------------------------------
 *  Copyright (c) Cal Abel. All rights reserved.
 *  Licensed under the MIT License. See LICENSE in the project root for license information.
 *
 *  Native test runner - runs all native unit tests with Unity Fixtures.
 *--------------------------------------------------------------------------------------------*/

#include <unity_fixture.h>

static void RunAllTests(void) {
    RUN_TEST_GROUP(NativeAsyncLogic);
    // RUN_TEST_GROUP(ABMAsyncLogic);
}

int main(int argc, const char *argv[]) {
    (void)argc;
    (void)argv;
    const char *unityArgs[] = {"Native", "-v"};
    return UnityMain(2, unityArgs, RunAllTests);
}
