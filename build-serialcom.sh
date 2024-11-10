#!/bin/sh
# Builds serialcom for a different architecture using Docker and copies the resulting lib into the bin folder

arch=aarch64
lib=libserialcom.so
output=/serial-native/build/lib/main/release/stripped/$lib

cmd="buildx build --platform=linux/$arch"

echo
echo "Building serialcom for $arch..."
docker $cmd -t serialcom:latest-$arch . $1
docker cp $(docker create --platform=linux/$arch --name copy-serialcom serialcom:latest-$arch):$output ./bin/linux-$arch/$lib \
    && docker rm copy-serialcom \
    && echo "$lib copied to bin/linux-$arch"
