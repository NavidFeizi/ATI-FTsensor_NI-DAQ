#makefile for ftconvert and calinfo
#use 'make ftconvert' to make ftconvert sample and 'make calinfo' to make calinfo
#uses gcc, needs modifications for other compilers.
object_compile = gcc -c 
shared_lib_compile = gcc -shared -o
remove_command = rm 

# Define the dependencies for the shared library target
libatidaq.so: ftconfig.o ftrt.o dom.o expatls.o node.o stack.o xmlparse.o xmlrole.o xmltok.o
	$(shared_lib_compile) libatidaq.so ftconfig.o ftrt.o dom.o expatls.o node.o stack.o xmlparse.o xmlrole.o xmltok.o -lm

# Define the object file dependencies and their compilation rules
atidaq.o: ../atidaq/ftconfig.h ../atidaq/ftrt.h ../atidaq/dom.h ../atidaq/expatls.h ../atidaq/node.h ../atidaq/stack.h ../atidaq/xmlparse.h ../atidaq/xmlrole.h ../atidaq/xmltok.h
	$(object_compile) -fPIC atidaq.h

ftconfig.o: ../atidaq/ftconfig.c ../atidaq/ftconfig.h 
	$(object_compile) -fPIC ../atidaq/ftconfig.c 

ftrt.o: ../atidaq/ftrt.c ../atidaq/ftrt.h  ../atidaq/ftsharedrt.h 
	$(object_compile) -fPIC ../atidaq/ftrt.c 

dom.o: ../atidaq/dom.c 
	$(object_compile) -fPIC ../atidaq/dom.c 

expatls.o: ../atidaq/expatls.c 
	$(object_compile) -fPIC ../atidaq/expatls.c 

node.o: ../atidaq/node.c 
	$(object_compile) -fPIC ../atidaq/node.c 

stack.o: ../atidaq/stack.c 
	$(object_compile) -fPIC ../atidaq/stack.c 

xmlparse.o: ../atidaq/xmlparse.c 
	$(object_compile) -fPIC ../atidaq/xmlparse.c 

xmlrole.o: ../atidaq/xmlrole.c 
	$(object_compile) -fPIC ../atidaq/xmlrole.c 

xmltok.o: ../atidaq/xmltok.c 
	$(object_compile) -fPIC ../atidaq/xmltok.c 

clean:
	$(remove_command) *.o libatidaq.so
