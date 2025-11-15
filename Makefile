#####################################################################
# Config
#####################################################################
BIN := emu

DEFS += -D__8052__

CFLAGS += -O2
CFLAGS += -pipe
CFLAGS += -g -Wall -Wextra -Wno-unused-parameter -Wshadow
CFLAGS += -D__8052__

# Uncomment to activate LTO
#CFLAGS += -flto

LDLIBS += -lcurses

#####################################################################
# Rules
#####################################################################
HEADERS := $(wildcard *.h)
SRC := $(wildcard *.c)
OBJ := $(SRC:.c=.o)

%.o: %.c $(HEADERS)
	 $(CC) $(CFLAGS) $(LDFLAGS) -c -o $@ $<

$(BIN): $(OBJ)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LDLIBS)

clean:
	-rm -f $(BIN) $(OBJ)
	-rm -rf out

.PHONY: clean all

all: $(BIN)

lib: $(OBJ)
	mkdir -p out/lib
	mkdir -p out/include
	ar rcs out/lib/libemu8051.a *.o
	unifdef $(DEFS) -x 2 -o out/include/emu8051.h emu8051.h
