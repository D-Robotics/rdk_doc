---
sidebar_position: 3
---
# 5.5.3 Using Breakpad

## Background

Breakpad is a tool suite for recording crash information in applications, which is more powerful than the Linux core mechanism. It can be used to view crash information of applications that have been stripped of compiler debugging information. When a program crashes, the crash information is recorded in a compact "minidump" file and sent back to the server. It can generate C and C++ stack traces from these minidumps and symbol files. Breakpad is located in the tools folder of TogetheROS.Bot.

## Preparation

Breakpad is located in the code repository  `https://github.com/HorizonRDK/breakpad.git`, with the develop branch. The directory contains the bin, lib, includes, and other folders that have been cross-compiled and can run on the Horizon RDK, which include the breakpad tools, static libraries, header files, and other contents.

## Usage
### 1. Creating, compiling, and running the test program
Create a new test program named test.cpp in the Breakpad directory and compile it into an executable program named test, making sure to include the -g option. Create a new directory /tmp, and then run the executable program test.

```c++
//  test.cpp

//  Include the core Breakpad header file
#include "client/linux/handler/exception_handler.h"

//  Callback function when a crash occurs
static bool dumpCallback(const google_breakpad::MinidumpDescriptor& descriptor,
                          void* context, bool succeeded) {
  printf("Dump path: %s\n", descriptor.path());
  return succeeded;
}

//  Crash function
void crash() { volatile int* a = (int*)(nullptr); *a = 1; }

int main(int argc, char* argv[]) {
  //  Initialize the descriptor and set the coredump file path to /tmp
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
  -lbreakpad -lbreakpad_client -lpthreadroot@ubuntu:~/cc_ws/tros_ws/src/tools/breakpad# mkdir /tmp
root@ubuntu:~/cc_ws/tros_ws/src/tools/breakpad# ./test
Dump path: /tmp/4113ab89-7169-49df-963945b3-383e8364.dmp
Segmentation fault
```

### 2. Generate dump file using breakpad

Give execute permission to the program, and use the dump_syms tool to dump the symbols information of the executable program "test" to a file named "test.sym".

```shell
root@ubuntu:~/cc_ws/tros_ws/src/tools/breakpad# chmod +x ./bin/*
root@ubuntu:~/cc_ws/tros_ws/src/tools/breakpad# ./bin/dump_syms ./test > test.sym
```

View the first line of the test.sym file and create the related folders.

```shell
root@ubuntu:~/cc_ws/tros_ws/src/tools/breakpad# head -n1 test.sym
MODULE Linux arm64 3816BF7138E87673BEE70E2C86F5FAC80 test
root@ubuntu:~/cc_ws/tros_ws/src/tools/breakpad# mkdir -p ./symbols/test/3816BF7138E87673BEE70E2C86F5FAC80 
root@ubuntu:~/cc_ws/tros_ws/src/tools/breakpad# cp test.sym ./symbols/test/3816BF7138E87673BEE70E2C86F5FAC80 
```

Run the executable program "test" to generate a minidump.dmp file. Run the following command to get the program's stack trace information. Note that the .dmp filename may be different. Here, we use the dmp file generated in the first step.

```shell
root@ubuntu:~/cc_ws/tros_ws/src/tools/breakpad# ./bin/minidump_stackwalk /tmp/4113ab89-7169-49df-963945b3-383e8364.dmp ./symbols
```

### 3. Analysis

The output of the command in the previous step is shown below, which indicates that the program crashes at line 11 of test.cpp, which is expected.

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
```x26 = 0x0000000000000000
x27 = 0x0000000000000000
x28 = 0x0000000000000000
fp = 0x0000007ffb82b550
lr = 0x0000005571710668
sp = 0x0000007ffb82b540
pc = 0x00000055717105c4
Found by: given as instruction pointer in context
1 test!main [test.cpp : 18 + 0x0]
x19 = 0x00000055717333d0
x20 = 0x0000000000000000
x21 = 0x0000005571710470
x22 = 0x0000000000000000
x23 = 0x0000000000000000
x24 = 0x0000000000000000
x25 = 0x0000000000000000
x26 = 0x0000000000000000
x27 = 0x0000000000000000
x28 = 0x0000000000000000
fp = 0x0000007ffb82b550
sp = 0x0000007ffb82b550
pc = 0x0000005571710668
Found by: call frame info
2 libc.so.6 + 0x20d4c
x19 = 0x00000055717333d0
x20 = 0x0000000000000000
x21 = 0x0000005571710470
x22 = 0x0000000000000000
x23 = 0x0000000000000000
x24 = 0x0000000000000000
x25 = 0x0000000000000000
x26 = 0x0000000000000000
x27 = 0x0000000000000000
x28 = 0x0000000000000000
fp = 0x0000007ffb82b700
sp = 0x0000007ffb82b700
pc = 0x0000007fb68f3d50
Found by: call frame info
```

## Summary

This chapter introduces how to use the breakpad framework to generate crash files and analyze stack information. The application initializes the breakpad by specifying the directory generated by the dump file and registering a callback function for crashes. Use the dump_syms tool of breakpad again to generate a symbol file and create a symbol directory. Finally, use the minidump_stackwalk tool to parse the dump file and analyze the stack information.
For more detailed information, please refer to the official website of Breakpad: https://chromium.googlesource.com/breakpad/breakpad/