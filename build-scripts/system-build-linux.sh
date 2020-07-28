#!/bin/sh

fail()
{
    echo "$1" 1>&2
    exit 1
}

BUILD_ARCHIVE_NAME="../OUT/opt/casparcgserver/"

# Clean and enter shadow build folder
echo Cleaning...
if [ -e ../build ]; then
    rm -Rf ../build || fail "Could not delete ../build"
fi

mkdir ../build || fail "Could not create ../build"
cd ../build || fail "Could not enter ../build"

# Run cmake
echo Running cmake...
cmake -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=RelWithDebInfo \
-DUSE_SYSTEM_BOOST=ON    \
-DUSE_SYSTEM_FFMPEG=ON    \
-DUSE_SYSTEM_TBB=ON       \
-DUSE_SYSTEM_GLEW=ON      \
-DUSE_SYSTEM_FREETYPE=ON  \
-DUSE_SYSTEM_FREEIMAGE=ON \
-DUSE_SYSTEM_OPENAL=ON    \
-DUSE_SYSTEM_SFML=ON      \
-DUSE_SYSTEM_FONTS=ON     \
.. || fail "cmake failed"

# Run make using the number of hardware threads in BUILD_PARALLEL_THREADS
echo Building...
time make -j${BUILD_PARALLEL_THREADS:-24} || fail "make failed"

# Create client folder to later zip
SERVER_FOLDER="$BUILD_ARCHIVE_NAME"
if [ -f "$SERVER_FOLDER" ]; then
    rm -Rf "$SERVER_FOLDER" || fail "Could not delete $SERVER_FOLDER"
fi
BIN_DIR="$SERVER_FOLDER/bin"
LIB_DIR="$SERVER_FOLDER/lib"
DOC_DIR="$SERVER_FOLDER/doc/"
FONT_DIR="$SERVER_FOLDER/fonts"

mkdir -p "$SERVER_FOLDER" || fail "Could not create $SERVER_FOLDER"
mkdir -p "$BIN_DIR"    || fail "Could not create $BIN_DIR"
mkdir -p "$LIB_DIR"    || fail "Could not create $LIB_DIR"
mkdir -p "$FONT_DIR"   || fail "Could not create $FONT_DIR"

# Copy compiled binaries
echo Copying binaries...
cp -fa  shell/lib* "$LIB_DIR/" 2>/dev/null         || echo "Did not copy server libraries"
cp -fa  ../ndilibs/* "$LIB_DIR/" 2>/dev/null       || echo "Could not copy NDI libraries"
cp -fa  shell/*.ttf "$FONT_DIR/" 2>/dev/null       || echo "Did not copy fonts"
cp -fa  ../deploy/general/server/font/LiberationSans-Regular.ttf "$SERVER_FOLDER" 2>/dev/null || echo "Could not copy liberation font"
cp -fa  shell/casparcg "$BIN_DIR/casparcg-server"  || fail "Could not copy server executable"
cp -fa  shell/casparcg.config "$DOC_DIR/"          || fail "Could not copy server config"
cp -faR shell/locales "$BIN_DIR/" 2>/dev/null      || echo "Did not copy server CEF locales"
cp -Rf shell/swiftshader "$SERVER_FOLDER/bin/" || fail "Could not copy server CEF swiftshader"
cp -f  shell/*.pak "$SERVER_FOLDER/bin/" || fail "Could not copy CEF resources"
cp -f  shell/*.bin "$SERVER_FOLDER/bin/" || fail "Could not copy V8 resources"
cp -f  shell/*.dat "$SERVER_FOLDER/bin/" || fail "Could not copy ICU resources"

# Remove empty directories
rmdir "$LIB_DIR" 2>/dev/null
rmdir "$FONT_DIR" 2>/dev/null


