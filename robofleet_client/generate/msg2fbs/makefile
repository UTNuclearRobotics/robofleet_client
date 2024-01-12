PYTHON?=/usr/bin/python3

# if you have flatbuffers installed, you can change this path
FLATC?=build/flatbuffers/flatc

.SUFFIXES: # disable default rules

.PHONY: all
all: schema_generated# build/flatbuffers/flatc build/flatbuffers/Makefile

.PHONY: clean
clean:
	-rm -f *_generated.*
	-rm -f schema.fbs
	-rm -rf build

schema_generated: $(SCHEMA_FILE).fbs
	flatc --gen-object-api -c $(SCHEMA_FILE).fbs

# build/flatbuffers/flatc: build/flatbuffers/Makefile
# 	(cd build/flatbuffers; make -j flatc)

# build/flatbuffers/Makefile: flatbuffers/CMakeLists.txt
# 	mkdir -p build/flatbuffers
# 	(cd build/flatbuffers; cmake ../../flatbuffers)
