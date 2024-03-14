CC = gcc
CFLAGS = -Wall -O2 -pthread
LDFLAGS =
SOURCES = main.c slip/slip.c
EXECUTABLE = slip-netif
INSTALL_PATH = /usr/local/bin

all: $(EXECUTABLE)

$(EXECUTABLE): $(SOURCES)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^

clean:
	rm -f $(EXECUTABLE)

install: $(EXECUTABLE)
	cp $< $(INSTALL_PATH)/$<

.PHONY: all clean install $(EXECUTABLE)