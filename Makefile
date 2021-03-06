DEBUG		= -g
CCC		= g++ -Wall
CC		= gcc -ansi -Wall
CCFLAGS		= $(DEBUG)  -D__MSDOS__ -O2
CPPFLAGS	=
LDFLAGS		=

SRCS		= usim.cc misc.cc \
		  mc6809.cc mc6809in.cc \
		  mc6850.cc term.cc \
		  main.cc pit82c54.cc
OBJS		= $(SRCS:.cc=.o)
BIN		= usim
LIBS		= -lwinmm 

$(BIN):		$(OBJS)
	$(CCC) -o $(@) $(CCFLAGS) $(LDFLAGS) $(OBJS) $(LIBS)

.SUFFIXES:	.cc

.cc.o:
	$(CCC) $(CPPFLAGS) $(CCFLAGS) -c $<

$(OBJS):	machdep.h

machdep:	machdep.o
	$(CC) -o $(@) $(CCFLAGS) $(LDFLAGS) machdep.o

machdep.h:	machdep
	./machdep $(@)

clean:
	$(RM) -f machdep.h machdep.o machdep $(BIN) $(OBJS)

depend:		machdep.h
	makedepend $(SRCS)

# DO NOT DELETE THIS LINE -- make depend depends on it.
