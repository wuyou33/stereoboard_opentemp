
MAKE = make

# Quiet
Q=@

all: pprzlink libs

# update (and init if needed) all submodules
update_submodules:
	$(Q)if [ -d .git ]; then \
		git submodule update --init; \
	fi

# update (and init if needed) a specific submodule
%.update:
	$(Q)if [ -d .git ]; then \
		git submodule update --init $*; \
	fi

pprzlink: pprzlink.update pprzlink.build

pprzlink.build:
	$(Q)$(MAKE) -C pprzlink MESSAGES_INSTALL="stereoboard"

libs: libs.update

clean:
	$(Q)$(MAKE) -C pprzlink clean
	
.PHONY: all update_submodules pprzlink.update pprzlink.build

