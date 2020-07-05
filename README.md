pov
===

Getting Started
---------------

rustup.rs


Running
-------

Plug jlink debugger into nrf52840, and the nrf52840 into a power source.

Run `./jlinkgdb` in one terminal
Optionally, run `telnet localhost 2333` in another terminal, for logging
Run `cargo run --release` in another terminal
 - This will start up GDB and pause the program. Press `c` to continue
