CC = g++
CFLAGS = -Wall
PROG = assignment1

SRCS = test.cpp


LIBS = -framework OpenGL -framework GLUT -L/usr/local/lib -I/usr/local/. -lgsl
all: $(PROG)

$(PROG):	$(SRCS)
	$(CC) $(CFLAGS) -o $(PROG) $(SRCS) $(LIBS)

clean:
	rm -f $(PROG)
