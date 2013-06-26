include Makefile.include

PROJECT_ID = 310
SOURCES = main.cpp MEXU.cpp $(MEXU_GENERATED_FILES)
LIB_PACKAGE = Makefile Makefile.include cookie.txt.in main.cpp main.h MEXU.cpp MEXU.h

all: clean main

downloadAndExpand:
	wget --load-cookies=cookie.txt http://group.clooca.com/download/c/${PROJECT_ID} -O downloaded.zip
	unzip -o downloaded.zip

main: main.cpp $(SOURCES)
	g++ -o main $(SOURCES)

clean:

pack:
	tar jcvf mexu.tar.bz2 ${LIB_PACKAGE}


