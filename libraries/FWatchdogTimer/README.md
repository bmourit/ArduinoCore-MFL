## FreeWatchdogTimer Library V1.0.0 for Arduino core on GD32F303RE

**Written by:** _BMourit_
GD32F303RE and MFL C++ library support

### Requirement
* [ArduinoCore_MFL](https://github.com/bnmguy/ArduinoCore_GD32_MFL) 

### What is the FreeWatchdogTimer library.

Th FreeWatchdogTimer library provides an interface to the free watchdog timer module (FWDGT) inside the GD32F303RE mcu.
The FWDGT module is used in to generate a reset signal for the CPU when encountering 
a catastrophic event that causes the software to become unresponsive.

The FWDGT module contains a countdown timer. The module will generate a reset condition when the timer reaches zero. In normal operation mode the software running on the CPU would reload the timer periodically to prevent the reset condition from happening. However if a software bug or other error causes the CPU to execute a different code path for too long, the reload will not happen and the FWDGT module will reset the CPU.

### How to use
The FreeWatchdogTimer is a built-in library included in the MFL C++ library. Since the library headers are included in the Arduino.h header file, you only need to add: `#include <Arduino.h>` to your sketch file.

```Arduino
#include <Arduino.h>

void setup() {
    ...
    // Initialize the FWDGT with 4 seconds timeout.
    // This would cause a CPU reset if the FWDGT
    // is not reloaded in approximately 4 seconds.
    FWatchdogTimer.begin(4000000);
}

void loop() {
    ...your code here...
    // make sure the code in this loop is executed in
    // less than 2 seconds to leave 50% headroom for
    // the timer reload.
    FWatchdogTimer.reload();
}

```

### Library functions

#### Preinstantiate Object

A default instance is available: `FWatchdogTimer`

```Arduino
FreeWatchdogTimer FWatchdogTimer = FreeWatchdogTimer();
```

#### Predefined values

 * Minimum timeout in microseconds: `minTimeout`
 * Maximum timeout in microseconds: `maxTimeout`

#### `void begin(uint32_t timeout)`

The `begin()` function will initialize the FWDGT hardware block.

The `timeout` parameter is in microseconds and sets the timer reset timeout.
When the timer reaches zero the hardware block will generate a reset signal
for the CPU.

When specifying timeout value, plan to refresh the timer at least twice
as often. The `reload()` operation is not expensive.

The downside of selecting a very large timeout value is that your system
may be left in an unresponsive state before the reset is generated.

Valid timeout values depends of the IRC40K clock value. Typically, its 32kH value is
between 125µs and 32,768ms (~32.8 seconds). The precision depends of the timeout values:

 | timeout value range | timeout value precision |
 | ------------------- |:-----------------------:|
 | 125µs - 512ms       | 125µs
 | 513ms - 1024ms      | 250µs
 | 1025ms - 2048ms     | 500µs
 | 2049ms - 4096ms     | 1ms
 | 4097ms - 8192ms     | 2ms
 | 8193ms - 16384ms    | 4ms
 | 16385ms - 32768ms   | 8ms

Calling the `begin()` method with a value outside of the valid range
will return without initializing the free watchdog timer.

**WARNING:**
*Once started, the FWDGT can not be stopped. If you are
planning to debug the live system, the watchdog timer may cause the
system to be reset while you are stepping in the debugger. Also consider
the FWDGT implications if you are designing a system which puts the CPU
into sleep mode.*

#### `void reload()`

The `reload()` method reloads the counter value.

Once you have initialized the FWDGT you **HAVE** to call `reload()`
periodically to prevent the CPU being reset.

#### `void set(uint32_t timeout)`

The `set()` method allows to set the timeout value.

The `timeout` parameter is the same as in the `begin()` method.

#### `void get(uint32_t* timeout)`

The `get()` method allows to get the currently set timeout value.

The `timeout` pointer refers to values in microseconds.

#### `bool isEnabled()`

The `isEnabled()` method checks if the FWDGT is enabled and returns true/false

#### `bool isReset(bool clear)`

The `isReset()` method checks if the system has resumed from FWDGT reset.

The optional `clear` parameter allow to clear FWDGT reset flag if true. Default: false.

#### `void clearReset()`

The `clearReset()` method clears FWDGT reset flag.
