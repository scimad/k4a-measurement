# Azure Kinect Sensor SDK Starter Template

This is a starting point for k4a-C++ projects, kept here for reference.

Instruction:

- Install Azure Kinect Sensor SDK (C++ Library).
    - Compile from source
    - `sudo make install` after compilation
- `mkdir build`
- `cd build`
- `cmake ..`
- `make -j$(nproc)`

You should see the `fastpcd` executable if everything works fine.