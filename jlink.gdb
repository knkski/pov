# print demangled symbols by default
set print asm-demangle on

set verbose on
break DefaultHandler
break HardFault
break rust_begin_unwind
# Connect to the JLink GDB server
target remote :2331

# Enable SWO output
monitor SWO EnableTarget 0 0 1 0

# reset to start
monitor reset

# Load the program
load
