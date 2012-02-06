DISK=sidplay.dsk

.PHONY: clean

$(DISK): sidplay.asm
	pyz80.py --exportfile=sidplay.sym sidplay.asm

clean:
	rm -f $(DISK) sidplay.sym
