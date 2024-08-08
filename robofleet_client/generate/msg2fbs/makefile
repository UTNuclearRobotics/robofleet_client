PYTHON?=/usr/bin/python

# if you have flatbuffers installed, you can change this path
FLATC?=build/flatbuffers/flatc

.SUFFIXES: # disable default rules

.PHONY: ros-all
ros-all:
	build/flatbuffers/Makefile
	build/flatbuffers/flatc
	

.PHONY: all
all: $(SCHEMA_FILE).h build/flatbuffers/flatc build/flatbuffers/Makefile

.PHONY: clean
clean:
	-rm -f *_generated.*
	-rm -f schema.fbs
	-rm -rf build

$(SCHEMA_FILE).h: $(SCHEMA_FILE).fbs $(FLATC)
	$(FLATC) --gen-object-api -c -n $(SCHEMA_FILE).fbs

build/flatbuffers/flatc: build/flatbuffers/Makefile
	(cd build/flatbuffers; make -j flatc)

build/flatbuffers/Makefile: flatbuffers/CMakeLists.txt
	mkdir -p build/flatbuffers
	(cd build/flatbuffers; cmake ../../flatbuffers)
