---
sidebar_position: 3
---

# 5.5.3 Using Breakpad

## Functional Background

Breakpad is a more powerful toolkit than the Linux core dump mechanism for capturing crash information from applications. It can be used to inspect crash details even for stripped binaries—applications from which compiler debugging symbols have been removed. When a program crashes, Breakpad records the crash data into a compact "minidump" file and sends it back to a server. C/C++ stack traces can then be reconstructed using these minidump files together with symbol files.

## Prerequisites

Breakpad is located in the [code repository](https://github.com/D-Robotics/breakpad.git) on the `develop` branch. The directory contains cross-compiled binaries that can run on RDK, including folders such as `bin`, `lib`, and `includes`, which respectively contain Breakpad tools, static libraries, and header files.

## Task Description

### 1. Create, compile, and run the test program

After downloading the source code, create a new test program `test.cpp` under the Breakpad directory and compile it into an executable named `test`, ensuring the `-g` flag is included. Create the `/tmp` directory, then run the executable `test`.

```c++
// test.cpp

// Include Breakpad's core header file
#include "client/linux/handler/exception_handler.h"

// Callback function invoked upon crash
static bool dumpCallback(const google_breakpad::MinidumpDescriptor& descriptor,
                          void* context, bool succeeded) {
  printf("Dump path: %s\n", descriptor.path());
  return succeeded;
}

// Crash-triggering function
void crash() { volatile int* a = (int*)(nullptr); *a = 1; }

int main(int argc, char* argv[]) {
  // Initialize descriptor and set minidump file path to /tmp
  google_breakpad::MinidumpDescriptor descriptor("/tmp");
  google_breakpad::ExceptionHandler eh(descriptor, NULL, dumpCallback, NULL,
                                        true, -1);
  crash();
  return 0;
}
```

```shell
root@ubuntu:~/cc_ws/tros_ws/src/tools/breakpad# g++ ./test.cpp -o test -g \
  -I ./include/breakpad/ \
  -L ./lib/ \
  -lbreakpad -lbreakpad_client -lpthread

root@ubuntu:~/cc_ws/tros_ws/src/tools/breakpad# mkdir /tmp
root@ubuntu:~/cc_ws/tros_ws/src/tools/breakpad# ./test
Dump path: /tmp/4113ab89-7169-49df-963945b3-383e8364.dmp
Segmentation fault
```

### 2. Use Breakpad to generate a dump file

Grant execute permissions to the programs, then use the `dump_syms` tool to extract symbol information from the executable `test` into a file named `test.sym`.

```shell
root@ubuntu:~/cc_ws/tros_ws/src/tools/breakpad# chmod +x ./bin/*
root@ubuntu:~/cc_ws/tros_ws/src/tools/breakpad# ./bin/dump_syms ./test > test.sym
```

Inspect the first line of `test.sym` and create the corresponding directory structure:

```shell
root@ubuntu:~/cc_ws/tros_ws/src/tools/breakpad# head -n1 test.sym
MODULE Linux arm64 3816BF7138E87673BEE70E2C86F5FAC80 test
root@ubuntu:~/cc_ws/tros_ws/src/tools/breakpad# mkdir -p ./symbols/test/3816BF7138E87673BEE70E2C86F5FAC80 
root@ubuntu:~/cc_ws/tros_ws/src/tools/breakpad# cp test.sym ./symbols/test/3816BF7138E87673BEE70E2C86F5FAC80 
```

Run the executable `test` again to generate a `.dmp` minidump file. Then execute the following command to retrieve the program’s stack trace. Note that the `.dmp` filename may differ—it should match the one generated in Step 1.

```shell
root@ubuntu:~/cc_ws/tros_ws/src/tools/breakpad# ./bin/minidump_stackwalk /tmp/4113ab89-7169-49df-963945b3-383e8364.dmp ./symbols
```

### 3. Analysis

The output of the command in Step 2 above is shown below. It clearly indicates that the program crashed at line 11 of `test.cpp`, which aligns with expectations.

```text
Thread 0 (crashed)
  0  test!crash() [test.cpp : 11 + 0x8]
      x0 = 0x0000000000000000    x1 = 0x0000000000000001
      x2 = 0x0000000000000000    x3 = 0x0000000000000001
      x4 = 0x0000005571754448    x5 = 0x0000005571754458
      x6 = 0x000000000000017f    x7 = 0x0000000000000000
      x8 = 0x0000000000000010    x9 = 0x0000000000000000
    x10 = 0x0000000000000000   x11 = 0x0000000000000000
    x12 = 0x0000007fb68d6e48   x13 = 0x0000000000000000
    x14 = 0x0000000000000000   x15 = 0x0000000000000020
    x16 = 0x0000005571753df8   x17 = 0x0000007fb6c5a418
    x18 = 0x0000000000000000   x19 = 0x00000055717333d0
    x20 = 0x0000000000000000   x21 = 0x0000005571710470
    x22 = 0x0000000000000000   x23 = 0x0000000000000000
    x24 = 0x0000000000000000   x25 = 0x0000000000000000
    x26 = 0x0000000000000000   x27 = 0x0000000000000000
    x28 = 0x0000000000000000    fp = 0x0000007ffb82b550
      lr = 0x0000005571710668    sp = 0x0000007ffb82b540
      pc = 0x00000055717105c4
    Found by: given as instruction pointer in context
  1  test!main [test.cpp : 18 + 0x0]
    x19 = 0x00000055717333d0   x20 = 0x0000000000000000
    x21 = 0x0000005571710470   x22 = 0x0000000000000000
    x23 = 0x0000000000000000   x24 = 0x0000000000000000
    x25 = 0x0000000000000000   x26 = 0x0000000000000000
    x27 = 0x0000000000000000   x28 = 0x0000000000000000
      fp = 0x0000007ffb82b550    sp = 0x0000007ffb82b550
      pc = 0x0000005571710668
    Found by: call frame info
  2  libc.so.6 + 0x20d4c
    x19 = 0x00000055717333d0   x20 = 0x0000000000000000
    x21 = 0x0000005571710470   x22 = 0x0000000000000000
    x23 = 0x0000000000000000   x24 = 0x0000000000000000
    x25 = 0x0000000000000000   x26 = 0x0000000000000000
    x27 = 0x0000000000000000   x28 = 0x0000000000000000
      fp = 0x0000007ffb82b700    sp = 0x0000007ffb82b700
      pc = 0x0000007fb68f3d50
    Found by: call frame info
```

## Summary

This section demonstrates how to use the Breakpad framework to generate crash dump files and analyze stack traces. An application initializes Breakpad by specifying the directory where dump files will be stored and registering a callback function to be invoked upon crash.  
Next, the `dump_syms` utility is used to generate symbol files, and the required symbol directory structure is created accordingly. Finally, the `minidump_stackwalk` tool parses the dump file and produces a detailed stack trace for analysis.

For more detailed information, please refer to the official Breakpad website: https://chromium.googlesource.com/breakpad/breakpad/