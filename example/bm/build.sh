if [ -n "$1" ]; then
    rm startup_m3.o main.o startup.elf startup.bin
else
    arm-none-eabi-as -mcpu=cortex-m3 -mthumb startup_m3.s -g -o startup_m3.o
    arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -c main.c -o main.o
    arm-none-eabi-ld -T m3.ld  startup_m3.o main.o -o startup.elf
    arm-none-eabi-objcopy -O binary startup.elf startup.bin
fi


