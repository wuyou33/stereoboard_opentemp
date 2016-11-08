
MAKE = make

# Quiet
Q=@

all: libs pprzlink drone_vision
#stereoboard_testing

# update (and init if needed) all submodules
update_submodules:
	$(Q)if [ -d ../.git ]; then     \
	  cd ../                        \
  	git submodule sync;           \
  	git submodule update --init;  \
	fi

# update (and init if needed) a specific submodule
#check this, not working correctly yet
%.update:
	$(Q)if [ -d ../.git ]; then       \
	  cd ../                          \
	  git submodule sync;             \
	  git submodule update --init ext/$*; \
	fi

pprzlink: pprzlink.update 
#pprzlink.build

pprzlink.build:
	$(Q)$(MAKE) -C pprzlink MESSAGES_INSTALL="stereoboard"

libs: libs.update

drone_vision: drone_vision.update

stereoboard_testing: stereoboard_testing.update

clean:
	$(Q)if [ -d $*/.git ]; then       \
		$(Q)$(MAKE) -C pprzlink clean;  \
	fi
	
.PHONY: all clean update_submodules pprzlink pprzlink.update pprzlink.build libs stereoboard_testing stereoboard_testing.update

