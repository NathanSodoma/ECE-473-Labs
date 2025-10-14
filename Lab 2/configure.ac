# Initialize Autoconf
AC_INIT([ece_473_lab], [1.0], [Ryan.Gambord@oregonstate.edu])
AM_INIT_AUTOMAKE([-Wall -Werror foreign subdir-objects])

AC_CONFIG_SRCDIR([src/main.c])
AC_CONFIG_FILES([Makefile src/Makefile])

AC_CONFIG_HEADERS([config.h])

# Check for required programs
AC_PROG_CC      # C compiler
AC_PROG_CC_C99 
AC_PROG_CPP     # C preprocessor
AC_PROG_RANLIB
AM_PROG_AR      # archiver

AC_PATH_PROG([BEAR], [bear])
AC_SUBST([BEAR])

# avrdude programmer
AC_PATH_PROG([AVRDUDE], [avrdude])
if test -z "$AVRDUDE"; then
  AC_MSG_ERROR([avrdude is required but not found])
fi

# Assembler
AC_PATH_TOOL([AS], [as])
if test -z "$AS"; then
  AC_MSG_ERROR([$host_alias-as is required but not found])
fi

# Assembler
AC_PATH_TOOL([OBJCOPY], [objcopy])
if test -z "$OBJCOPY"; then
  AC_MSG_ERROR([$host_alias-objcopy is required but not found])
fi

LT_INIT([disable-shared])

AC_CONFIG_MACRO_DIRS([m4])

AC_CANONICAL_HOST
AC_MSG_NOTICE("host_cpu = "$host_cpu")
if test "$host_cpu" != "avr"; then
  AC_MSG_ERROR([This configuration requires --host=avr])
fi

AC_LANG_PUSH([C])
AC_CHECK_HEADERS([avr/version.h])
AC_MSG_CHECKING([for avr-libc >= 2.2.0])
AC_PREPROC_IFELSE(
  [AC_LANG_SOURCE([
#include <avr/version.h>
#if !((__AVR_LIBC_MAJOR__ > 2) || \
      (__AVR_LIBC_MAJOR__ == 2 && __AVR_LIBC_MINOR__ > 2) || \
      (__AVR_LIBC_MAJOR__ == 2 && __AVR_LIBC_MINOR__ == 2 && __AVR_LIBC_REVISION__ >= 0))
#error __AVR_LIBC_VERSION__
#endif
  ])],
  [AC_MSG_RESULT([yes])],
  [AC_MSG_ERROR([avr-libc >= 2.2.0 is required])]
)

AC_ARG_VAR([MCU], [Specify AVR instruction set architecture (ISA) or device type. (defualt: atmega32u4)])
if test -z "$MCU"; then
  MCU=atmega32u4
fi
AC_SUBST([MCU])

AC_ARG_VAR([PROGRAMMER], [Specify avrdude programmer. (default: avr109)])
if test -z "$PROGRAMMER"; then
  PROGRAMMER=avr109
fi
AC_SUBST([PROGRAMMER])

AC_ARG_VAR([PRGRAMMER_PORT], [Specify avrdude programmer port. (default: /dev/ttyACM0)])
if test -z "$PROGRAMMER_PORT"; then
  PROGRAMMER_PORT=/dev/ttyACM0
fi
AC_SUBST([PROGRAMMER_PORT])

AC_ARG_VAR([F_CPU], [Specify CPU clock frequency in Hz. (default: 8MHz)])
if test -z "$F_CPU"; then
  F_CPU=8000000
fi
AC_DEFINE_UNQUOTED([F_CPU], [$F_CPU], [CPU clock frequency in Hz.])

AC_SUBST([AM_CFLAGS], ["-g -Og -mmcu=\"\$(MCU)\""])

AC_OUTPUT
