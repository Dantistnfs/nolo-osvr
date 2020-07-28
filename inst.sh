#!/bin/sh

cd build
make
sudo cp ./lib/osvr-plugins-0/com_osvr_Nolo.so  /usr/lib/osvr-plugins-0/
