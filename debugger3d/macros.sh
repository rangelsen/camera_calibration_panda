#! /bin/bash

TARGET='informant'
SOURCE_DIR='src'
BUILD_DIR='build/linux_x86_64'
DOC_DIR='doc'
BIN_DIR='bin/linux_x86-64'

tup-diagram() {
	tup graph $SOURCE_DIR $INCLUDE_DIR $OBJECT_DIR | dot -Tpng > $DOC_DIR/dependency_tree.png
    eog $DOC_DIR/dependency_tree.png &
}

clean() {
    rm $BUILD_DIR/*.o
}

line_count() {

    find src -name '*.cpp' -o \
             -name '*.hpp' -o \
             -name '*.h' -o \
             -name '*.c' -o \
             -name '*.vs' -o \
             -name '*.fs' | xargs wc -l
}

run() {

    ./$BIN_DIR/$1 $2
}

debug() {

    gdb $BIN_DIR/$1
}

