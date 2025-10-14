SUBDIRS = src

 ifeq ($(MAKELEVEL), 0)
 ifdef BEAR

all:
	$(BEAR) --output "$(srcdir)/compile_commands.json" --append -- $(MAKE) $@

 endif
 endif

CLEANFILES = $(srcdir)/compile_commands.json
