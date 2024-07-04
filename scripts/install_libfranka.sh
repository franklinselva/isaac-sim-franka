#! /bin/bash

set -e

print_color() {
    local color=$1
    shift
    echo -e "\033[${color}m$@\033[0m"
}

print_color "1;32" "Installing dependencies for libfranka..."
sudo apt install -y libpoco-dev libeigen3-dev

cd /tmp
print_color "1;32" "Cloning libfranka repository..."
git clone https://github.com/frankaemika/libfranka.git --recursive
cd libfranka
print_color "1;32" "Checking out version 0.13.2..."
git checkout 0.13.2
mkdir build && cd build
print_color "1;32" "Building libfranka..."
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF ..
cmake --build . -j"$(nproc)"
cpack -G DEB
print_color "1;32" "Installing libfranka..."
sudo dpkg -i libfranka-*.deb
print_color "1;32" "Cleaning up..."
cd /tmp
rm -rf libfranka
print_color "1;32" "Done."
