LCM = lcm-gen

LCMTYPES_NAMES := $(shell cd ../lcmtypes && ls *.lcm)

# Construct pathnames to python types
PY_PACKAGE = lcmtypes
LCMTYPES_PY := $(LCMTYPES_NAMES:%.lcm=%.py)
LCMTYPES_PY := $(addprefix lcmtypes/,$(LCMTYPES_PY))

all: | lcmtypes $(LCMTYPES_PY)

lcmtypes/%.py: ../lcmtypes/%.lcm
	@$(LCM) -p --package-prefix $(PY_PACKAGE) $<

lcmtypes:
	mkdir -p lcmtypes

clean:
	rm -rf *~ lcmtypes/* *.pyc
