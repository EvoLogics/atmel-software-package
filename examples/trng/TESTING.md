TRNG EXAMPLE
============

# Objectives
------------
This example aims to test TRNG(True Random Number Generator).


# Example Description
---------------------
This example demonstrates how to generate random data with TRNG peripheral.


# Test
------

## Setup
--------
Step needed to set up the example.

* Build the program and download it inside the evaluation board.
* On the computer, open and configure a terminal application (e.g. HyperTerminal
 on Microsoft Windows) with these settings:
	- 115200 bauds
	- 8 bits of data
	- No parity
	- 1 stop bit
	- No flow control
* Start the application.
* In the terminal window, the following text should appear (values depend on the
 board and chip used):
```
 -- TRNG Example xxx --
 -- SAMxxxxx-xx
 -- Compiled: xxx xx xxxx xx:xx:xx --
```

Tested with IAR and GCC (sram and ddram configurations)

In order to test this example, the process is the following:

Step | Description | Expected Result | Result
-----|-------------|-----------------|-------
`Nothing to do` | Print random numbers generated by TRNG on screen | PASSED | PASSED


# Log
------

## Current version
--------
 - v1.3

## History
--------