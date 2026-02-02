OS gdb 

(gdb) target remote localhost:1234
Remote debugging using localhost:1234
The target architecture is assumed to be i8086
[f000:fff0]    0xffff0:	ljmp   $0x3630,$0xf000e05b
0x0000fff0 in ?? ()
(gdb) stepi
[f000:e05b]    0xfe05b:	cmpw   $0xff88,%cs:(%esi)
0x0000e05b in ?? ()
(gdb) 
[f000:e062]    0xfe062:	jne    0xd241d092
0x0000e062 in ?? ()
(gdb) 
[f000:e066]    0xfe066:	xor    %edx,%edx
0x0000e066 in ?? ()
(gdb) break *0x7c00
Breakpoint 1 at 0x7c00
(gdb) c
Continuing.
[   0:7c00] => 0x7c00:	sub    %eax,%eax ----- first instruction

Breakpoint 1, 0x00007c00 in ?? ()
(gdb) 
