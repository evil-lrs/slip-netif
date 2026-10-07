CC = gcc
CFLAGS = -Wall -O2 -pthread
LDFLAGS =
SOURCES = main.c slip/slip.c
EXECUTABLE = slip-netif
TEST_EXECUTABLE = test/worst_case_escaping
INSTALL_PATH = /usr/local/bin

all: $(EXECUTABLE)

$(EXECUTABLE): $(SOURCES)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^

$(TEST_EXECUTABLE): test/worst_case_escaping.c slip/slip.c
	$(CC) $(CFLAGS) -I. -o $@ $^

test: $(TEST_EXECUTABLE)
	./$(TEST_EXECUTABLE)

clean:
	rm -f $(EXECUTABLE) $(TEST_EXECUTABLE)

install: $(EXECUTABLE)
	cp $< $(INSTALL_PATH)/$<

.PHONY: all clean install test $(EXECUTABLE)