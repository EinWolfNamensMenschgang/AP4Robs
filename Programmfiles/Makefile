CPPFLAG = g++ -std=c++17 -Wall -Wextra -pedantic -pthread
CPPDEBUG = -g -DDEBUG

programm : main.o Subscriber.o Publisher.o Functions.o Parsing.o
	$(CPPFLAG) $(CPPDEBUG) *.o -o programm

Subscriber.o: Subscriber.cpp Subscriber.h
	$(CPPFLAG) -c Subscriber.cpp

Publisher.o: Publisher.cpp Publisher.h
	$(CPPFLAG) -c Publisher.cpp

Functions.o: Functions.cpp Functions.h
	$(CPPFLAG) -c Functions.cpp

Parsing.o: parsing.cpp parsing.h
	$(CPPFLAG) -c parsing.cpp

main.o: main.cpp
	$(CPPFLAG) -c main.cpp

clean :
	rm *.o
