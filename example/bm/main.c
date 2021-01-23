

#define USART1_BASE_ADDR 0x40011000
#define USART_DR   0x04

void print_func(char* s)
{
    while (*s != '\x00')
    {
        *(volatile unsigned int*)(USART1_BASE_ADDR + USART_DR) = *s;
        s++;
    }
}

void main_func()
{
    print_func("main_func!\n");
    return;
}