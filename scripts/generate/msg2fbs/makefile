PYTHON?=/usr/bin/python

# if you have flatbuffers installed, you can change this path
FLATC?=build/flatbuffers/flatc

.SUFFIXES: # disable default rules

.PHONY: all
all: schema_generated.h build/flatbuffers/flatc build/flatbuffers/Makefile

.PHONY: clean
clean:
	-rm -f *_generated.*
	-rm -f schema.fbs
	-rm -rf build

schema_generated.h: schema.fbs $(FLATC)
	$(FLATC) --gen-object-api -c schema.fbs

build/flatbuffers/flatc: build/flatbuffers/Makefile
	(cd build/flatbuffers; make -j flatc)

build/flatbuffers/Makefile: flatbuffers/CMakeLists.txt
	mkdir -p build/flatbuffers
	(cd build/flatbuffers; cmake ../../flatbuffers)
