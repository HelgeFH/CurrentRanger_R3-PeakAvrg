# /bin/bash

cd ../CurrentRanger_R3/build
python ../../Bootloader/uf2conv.py CurrentRanger_R3.ino.hex \
   -c -f 0x68ed2b88 -o CurrentRanger_R3.ino.uf2

