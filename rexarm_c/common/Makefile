include ../common.mk

CFLAGS  = $(CFLAGS_STD)  $(CFLAGS_GLIB)  $(CFLAGS_GTK) -O4 -DNDEBUG
LDFLAGS = $(LDFLAGS_STD) $(LDFLAGS_GLIB) $(LDFLAGS_GTK)
LIBDEPS = $(call libdeps, $(LDFLAGS))

LIB_COMMON = $(LIB_PATH)/libcommon.a
LIBCOMMON_OBJS = \
	getopt.o \
	ioutils.o \
	serial.o \
	timestamp.o \
    zhash.o \
    zarray.o

all: $(LIB_COMMON)

$(LIB_COMMON): $(LIBCOMMON_OBJS) $(LIBDEPS)
	@echo "\t$@"
	@ar rc $@ $^

clean:
	@rm -f *.o *~ *.a
	@rm -f $(LIB_COMMON)
