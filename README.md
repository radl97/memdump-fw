## First running



```
memdump-fw$ mkdir cmake-build-debug
$ cmake -DCMAKE_BUILD_TYPE=Debug ..
$ make
... memdump.elf gets created ...
```

```
cmake-build-debug$ sudo openocd -f ../memdump-stlink.cfg -c 'program memdump.elf'
```

Two remarks:

- `sudo` is only needed because I have not added myself to devices
- If `memdump-stlink.cfg` does not work (does not find a file), try with `memdump-stlink-focal.cfg`.

## First debugging

### Setting it up with the hardware

SWD has to be wired to the programmer board.

Somewhy the target board was not powered, so the power was supplied by USB to the target board *and* the SWD pin was disconnected which would have supplied voltage.

Thus, both the programmer board and the target board was plugged into the computer.

### On the software side

We will need `gdb-multiarch`.

```
memdump-fw$ openocd -c memdump-stlink.cfg
```

(or the same command with `memdump-stlink-focal.cfg`) starts the fancy gdbserver on `localhost:3333`.

```
cmake-build-debug$ gdb-multiarch --nx
> file memdump.elf
> target remote localhost:3333
0x08000b5c in Reset_Handler ()
```

`\o/`

### Testing I2C

`ri 50 400`
