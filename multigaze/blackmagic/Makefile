# Hey Emacs, this is a -*- makefile -*-

BLACKSPHERE = http://blacksphere.co.nz/builds/archive/2014
BLACKMAGIC = blackmagic-20141017.bin

JTAG = ./blackmagic_14jtags.bin

Q=@

all:
	@echo "Flashing $(JTAG)"
	$(Q)./stm32_mem.py $(JTAG)

original: $(BLACKMAGIC) dfu.py
	@echo "Flashing $(BLACKMAGIC)"
	$(Q)./stm32_mem.py ./$(BLACKMAGIC)

$(BLACKMAGIC):
	@echo "Downloading latest BMP firmware"
	wget -O ./$(BLACKMAGIC) $(BLACKSPHERE)/$(BLACKMAGIC)

clean:
	$(Q) rm -rf ./$(BLACKMAGIC)
