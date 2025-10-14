Lab 3 Part 1 -- LCD Module
==========================

In this lab, you will implement a basic LCD driver that supports basic operations like:

* Setting the backlight and contrast
* Writing text by converting ASCII character codes to glpyhs
* Scrolling through the page buffer

...and so on.

A framework has been provided for you, including an implementation of a main function which tests some of the features of your LCD.

You will need to refer to the:

* ATMega32U4 Specification
* Ben's Board Schematic
* ST7565R Specification

Communication with the LCD module uses SPI. You will need to configure this and figure out how to send commands and data to the LCD module.

Controlling the backlight requires configuring PWM on the appropriate pin, using the appropriate timer. Refer to the ATMega spec sheet and Schematics for guidance.

Be mindful of timing requirements (e.g. the required RST signal hold time)

Checkoff on implementing the functionality tested by the main() function;

Additional extra credit for creative works--implement pixel graphics and draw something :^)

Compiling
---------

Since this is a fairly large project, I opted to use autotools to handle cross-compilation. It's a very nice toolset for the purpose, alough a bit overkill.

To compile an autotools project,

1. From the base directory (where the `configure` script is housed), create a new directory (e.g. `build/`)
2. From within the `build/` directory, execute the configuration script,

    $ ls 
    configure
    configure.ac
    ...
    src/
    ...etc...

    $ mkdir build/
    $ cd build/
    $ ../configure --host=avr

   Here, the `--host` flag is used to specify cross-compilation to the AVR architecture.

3. The configuration script will generate a Makefile for your project. From within the build directory you may then run `make`.
4. To program your board, run `make install` (it is no longer `make program`).

The nice thing is this won't leave any build artefacts in your prisine source tree. This is called an out-of-source build, and tools like this are the norm for projects of any significance :)
