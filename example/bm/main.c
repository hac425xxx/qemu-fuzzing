

#define USART1_BASE_ADDR 0x40011000
#define USART_DR   0x04

#define IRQ_DEMO_BASE 0x88990000

#define NVIC_MEM_BASE 0xe000e000

void print_func(unsigned char* s)
{
    while (*s != 0)
    {
        *(volatile unsigned char*)(USART1_BASE_ADDR + USART_DR) = *s;
        s++;
    }
}

void enable_demo_irq()
{
    unsigned int irq = 20 + 16;
    unsigned int offset = (irq - 16) / 8;
    offset += 0x180;
    offset -= 0x80;
    *(volatile unsigned char*)(NVIC_MEM_BASE + offset) = 1 << 4;
}


void main_func()
{
    print_func("main_func!\n");
    enable_demo_irq();  // 配置 nvic 的mmio，让 20号 irq 的 enabled=1

    *(volatile unsigned int*)(IRQ_DEMO_BASE + 4) = 33;  // 触发 demo_irq， 下面进入 demo_irq_func
    print_func("end main_func!\n");
    return;
}


void demo_irq_func()
{
    print_func("demo_irq_func!\n");
    return;
}


