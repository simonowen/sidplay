NAME=sidplay
DEPS=
UNAME := $(shell uname -s)

.PHONY: clean

$(NAME).tap: $(NAME).asm $(DEPS)
	pasmo -v --tapbas $(NAME).asm $(NAME).tap $(NAME).sym

run: $(NAME).tap
ifeq ($(UNAME),Darwin)
	open $(NAME).tap
else
	xdg-open $(NAME).tap
endif

clean:
	rm -f $(NAME).tap $(NAME).sym
