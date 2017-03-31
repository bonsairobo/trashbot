SUBDIRS = rexarm_c java rexarm_python

MAKEFLAGS += --no-print-directory

all:
	@for dir in $(SUBDIRS); do \
	echo "[$$dir]"; $(MAKE) -C $$dir all || exit 2; done

clean:
	@for dir in $(SUBDIRS); do \
	echo "clean [$$dir]"; $(MAKE) -C $$dir clean || exit 2; done
	@if test -d solns; then echo "clean [solns]"; $(MAKE) -C solns clean || exit 2; fi
	@rm -f *~
