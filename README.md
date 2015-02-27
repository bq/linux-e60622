Building instructions:

Checkout branch topic/fixes with

    git checkout topic/fixes

There is already a .config file present in this branch.

Build the kernel by issuing
    make ARCH=arm CROSS_COMPILE=/path/to/cross/compiler/arm-none-eabi- uImage

Copy arch/arm/boot/uImage to /boot/uImage on your ereader rootfs.
