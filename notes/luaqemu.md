# 概述

在嵌入式安全领域常常需要分析各种不同形态的固件，如果需要动态执行某些代码或者对固件进行Fuzzing测试，则需要对固件代码进行仿真，常用的仿真工具一般为qemu和unicorn。unicorn适合模拟执行固件中的某些代码片段，而对于中断、异步执行则不支持，而大量的嵌入式固件都是以中断驱动的，对于中断的模拟则需要依赖于qemu的全系统模拟。

本文将以luaqemu为例介绍在使用qemu来模拟固件、外设时可以借鉴的技术，代码地址

```
https://github.com/Comsecuris/luaqemu
```

简单的说luaqemu通过修改部分qemu代码并在一些关键的执行点增加回调函数，使得用户可以通过lua脚本来加载固件、监控固件代码的执行，设置观察点、断点等。

示例luaqemu脚本如下

```
require('hw.arm.luaqemu')
 
 machine_cpu = 'cortex-r5'
 
 memory_regions = {
     region_rom = {
         name = 'mem_rom',
         start = 0x0,
         size = 0x180000
     },
     region_ram = {
         name = 'mem_ram',
         start = 0x180000,
         size = 0xC0000
     },
 }
 
 file_mappings = {
     main_rom = {
         name = 'rom.bin',
         start = 0x0,
         size = 0x180000
     },
     main_ram = {
         name = 'kernel.bin',
         start = 0x180000,
         size = 0xC0000
     }
 }
 
 cpu = {
     env = {
         thumb = true,
     },
     reset_pc = 0
 }
```

该脚本的作用如下

1. `machine_cpu`指定cpu的类型
2. 利用`memory_regions`初始化两块内存，起始地址和内存大小分别为：`(0x0, 0x180000)` 和 `(0x180000, 0xC0000)`
3.  `file_mappings`将特定的文件加载到内存中指定的位置，代码中将`rom.bin`文件加载到`0x0`地址，大小为`0x180000`，将`kernel.bin`文件加载到`0x180000`地址，大小为`0xC0000`。
4. `cpu`关键字指定cpu的属性，设置了cpu的指令类型为thumb，`reset_pc`设置虚拟机启动后的pc寄存器的值为0，表示虚拟机启动后执行的第一条指令。

# 初始化

luaqemu新增了一个luaarm的机器，代码位于

```
hw/arm/luaarm.c
```

代码通过宏和数据结构指定machine的类型和初始化函数

```
static void lua_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "Lua ARM Meta Machine";
    mc->init = lua_init;
}

static const TypeInfo lua_machine_type = {
    .name = MACHINE_TYPE_NAME("luaarm"),
    .parent = TYPE_MACHINE,
    .class_init = lua_class_init,
};

static void lua_machine_init(void)
{
    type_register_static(&lua_machine_type);
}

type_init(lua_machine_init)
```

可以看到`lua_init`为machine的入口函数

```
static void lua_init(MachineState *machine)
{	
	// 加载 lua_script 指定的脚本，命令行参数指定
    luaL_loadfile(lua_state, lua_script)

	// 设置 cpu 的类型
    machine->cpu_model = lua_tostring(lua_state, -1);
    
    // 根据lua脚本来设置虚拟机的状态、固件的加载、回调函数注册
 
    init_memory_regions();
    init_luastate(machine);
    init_file_mappings();
    init_cpu_state();
    init_vm_states();

    // 执行 lua 脚本的 post_init 函数
}
```

该函数会根据lua脚本函数设置虚拟机的状态、固件的加载、回调函数注册等，函数的主要流程如下

1. 首先根据命令行参数加载指定的lua脚本
2. 设置CPU的类型，内存映射关系
3. 加载文件到虚拟机的内存
4. 初始化CPU的状态（寄存器）
5. 注册一系列回调函数，比如设置断点、指令执行回调等

通过搜索`lua_script`关键字可以找到设置命令行参数的位置位于`vl.c`的main函数里面

```
            case QEMU_OPTION_lua:
                lua_script = optarg;
                break;
```

我们也可以通过类似的方式注册需要的命令行参数

`init_luastate` 会把虚拟机的cpu对象保存到`luastate`全局变量里面

```
static void init_luastate(MachineState *machine)
{
    ARMCPU *cpu;
    ObjectClass *cpu_oc;
    CPUState *cs;

    cpu_oc = cpu_class_by_name(TYPE_ARM_CPU, machine->cpu_model);
    if (!cpu_oc) {
        error_report("machine \"%s\" not found, exiting\n", machine->cpu_model);
        exit(1);
    }

    cpu = ARM_CPU(object_new(object_class_get_name(cpu_oc)));
    cs = CPU(cpu);

    luastate.cpu = cpu;
    luastate.cs = cs;
    luastate.machine = machine;
    luastate.bp_pc = 0;
    luastate.bp_pc_ptr = NULL;
    luastate.old_wp_ptr = NULL;

    g_hash_table_foreach(breakpoints, add_cpu_breakpoints, NULL);
}
```





## 内存申请

### qemu内存模型

qemu 使用 `MemoryRegion ` 组织虚拟机的物理内存空间，`MemoryRegion ` 表示一段逻辑内存区域，它的类型如下：

1. RAM：普通内存，qemu通过向主机申请虚拟内存来实现。
2. MMIO：`MMIO`内存在读写时会调用初始化`mr`时指定的回调函数，回调函数由`MemoryRegionOps`指定，在`memory_region_init_io`时指定
3. ROM：只读内存，只读内存的读操作和RAM相同，禁止写操作。
4. ROM device：只读设备，读操作和RAM行为相同，只读设备的允许写操作，写操作和MMIO行为相同，会触发callback。
5. IOMMU region：将对一段内存的访问转发到另一段内存上，这种类型的内存只用于模拟IOMMU的场景。
6. container：容器，管理多个MR的MR，用于将多个MR组织成一个内存区域，比如整个虚机的内存地址区域，它被抽象成一个容器，包括了所有虚拟的内存区间。
7. alias：主要是让不同物理地址映射到同一个 `MemoryRegion ` ，类似于memory banking。

下面介绍一些常用内存的使用方式

#### 申请ram

qemu使用`memory_region_init_ram`初始化`MemoryRegion`为ram类型

```
void memory_region_init_ram(MemoryRegion *mr,
                            struct Object *owner,
                            const char *name,
                            uint64_t size,
                            Error **errp)
```

**使用实例**

```
MemoryRegion *system_memory = get_system_memory();
MemoryRegion *flash = g_new(MemoryRegion, 1);
memory_region_init_ram(flash, NULL, "STM32F205.flash", FLASH_SIZE, &error_fatal);
memory_region_add_subregion(system_memory, 0, flash);
```

qemu通过`MemoryRegion`的组合来表示虚拟机的物理内存空间，qemu在启动时会创建一个system_memory的`MemoryRegion`，system_memory是一个全局变量可以通过`get_system_memory`函数获取。

```
static void memory_map_init(void)
{
    system_memory = g_malloc(sizeof(*system_memory));

    memory_region_init(system_memory, NULL, "system", UINT64_MAX);
    address_space_init(&address_space_memory, system_memory, "memory");

    system_io = g_malloc(sizeof(*system_io));
    memory_region_init_io(system_io, NULL, &unassigned_io_ops, NULL, "io",
                          65536);
    address_space_init(&address_space_io, system_io, "I/O");
}
```

system_memory的大小为`UINT64_MAX`， 表示了整个物理内存空间，这个只是一个初始化，如果物理地址空间中的某些区域是`ram`，`rom`或者是`mmio`内存就可以通过`memory_region_add_subregion`来定义子区域的`MemoryRegion`类型。

```
void memory_region_add_subregion(MemoryRegion *mr,
                                 hwaddr offset,
                                 MemoryRegion *subregion)
其中mr为父MemoryRegion
subregion 为子MemoryRegion
offset表示 相对于 mr 起始地址的偏移

函数的作用：mr 的 offset 处内存由 subregion 重新定义
```

回到本节的实例，流程如下

1. 首先使用get_system_memory获取表示整个物理内存空间的MemoryRegion。
2. 然后新建一个`flash`的MemoryRegion并使用memory_region_init_ram指定该MemoryRegion是一个RAM类型的，大小为FLASH_SIZE。
3. 使用memory_region_add_subregion把`flash`挂载到`system_memory`起始地址偏移0处。

由于system_memory表示的是虚拟机的整个物理内存空间，执行完之后虚拟机物理地址0处的内存是RAM类型，大小为FLASH_SIZE，可以像内存使用一样直接读写。

#### 申请rom

使用方式和申请ram的一样，不同的申请得到的内存为只读的

```
void memory_region_init_rom(MemoryRegion *mr,
                            struct Object *owner,
                            const char *name,
                            uint64_t size,
                            Error **errp)
```

**使用实例**

```
memory_region_init_rom(&s->rom, NULL, "imx6ul.rom", FSL_IMX6UL_ROM_SIZE, &error_abort);
memory_region_add_subregion(get_system_memory(), FSL_IMX6UL_ROM_ADDR, &s->rom);
```

执行之后虚拟机 `[FSL_IMX6UL_ROM_ADDR, FSL_IMX6UL_ROM_ADDR + FSL_IMX6UL_ROM_SIZE]` 这段物理地址空间为ROM内存，只读。



#### 申请mmio

使用的函数为memory_region_init_io

```
void memory_region_init_io(MemoryRegion *mr,
                           Object *owner,
                           const MemoryRegionOps *ops,
                           void *opaque,
                           const char *name,
                           uint64_t size)
```

申请之后，对mr内存区域的读写会调用ops指定回调函数进行处理，这种类型的内存是模拟外设时常用的内存类型，因为在ARM芯片中外设的寄存器空间会挂载在系统内存总线上，所以可以通过访问内存来访问外设的寄存器空间，从而控制外设的行为。

**使用实例**

```
static uint64_t demo_read(void *opaque, hwaddr offset,
                                               unsigned size)
{
	
    return data[offset];
}

static void demo_write(void *opaque, hwaddr offset,
                                            uint64_t value, unsigned size)
{
	// 进行具体的写操作
    return;
}

static const MemoryRegionOps demo_ops = {
    .read = demo_read,
    .write = demo_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

memory_region_init_io(&demo_mr, NULL, &demo_ops, NULL, "demo-mmio",  0x1000);
memory_region_add_subregion(system_mem, 0x110000, &demo_mr);
```

执行完后虚拟机物理内存 `[0x110000, 0x110000+0x1000]` 这块区域为 `mmio`内存，当对这块内存进行读写操作时会调用`demo_ops`中指定的回调函数，比如读内存时调用demo_read， 写内存时调用demo_write。

以写内存为例

```
static void demo_write(void *opaque, hwaddr offset,
                                            uint64_t value, unsigned size)
{
	// 进行具体的写操作
    return;
}
```

offset为虚拟机写内存的地址相对MemoryRegion起始地址的偏移，比如现在写的地址是 `0x110012`，写的数据大小为2个字节，值为 `0xaabb`。

那么进入demo_write时的参数信息如下

```
offset: 0x110012-0x110000 --> 0x12
value: 0xaabb
size: 2
```

#### 申请alias内存

申请函数

```
void memory_region_init_alias(MemoryRegion *mr,
                              Object *owner,
                              const char *name,
                              MemoryRegion *orig,
                              hwaddr offset,
                              uint64_t size)
```

**使用实例**

```
memory_region_init_ram(flash, NULL, "flash", FLASH_SIZE, &error_fatal);
memory_region_init_alias(flash_alias, NULL, "flash.alias", flash, 0, FLASH_SIZE);
memory_region_add_subregion(system_memory, 0x08000000, flash);
memory_region_add_subregion(system_memory, 0, flash_alias);
```

执行完后 `0x08000000` 和 0 地址的内存是同一块，对 `0x08000000` 写数据，0地址也可以读到修改后的数据。



### luaqemu申请内存的实现

lua脚本通过memory_regions定义内存申请

```
memory_regions = {
    region_rom = {
        name = 'mem_rom',
        start = 0x0,
        size = 0x180000
    },
    region_ram = {
        name = 'mem_ram',
        start = 0x180000,
        size = 0xC0000
    },
}
```



```
static void init_memory_regions(void)
{
    MemoryRegion *sysmem = get_system_memory();
    lua_get_global("memory_regions", THROW_ERROR);
    
    // 遍历 memory_regions
    while (lua_next(lua_state, -2)) { 
        add_memory_region(sysmem);
        lua_pop(lua_state, 1);
    }
}
```

遍历memory_regions然后对其中的每一项使用 add_memory_region 处理每一个内存映射

```
static void add_memory_region(MemoryRegion *sm)
{
    region_start = lua_get_unsigned("start", THROW_ERROR);
    region_size  = lua_get_unsigned("size", THROW_ERROR);
    region_name  = lua_get_string("name", THROW_ERROR);

    memory_region = g_new(MemoryRegion, 1);
    memory_region_allocate_system_memory(memory_region, NULL, region_name, region_size);
    memory_region_add_subregion(sm, region_start, memory_region);
}
```

主要逻辑就是根据lua脚本的memory_region定义，使用memory_region_allocate_system_memory分配内存，然后使用memory_region_add_subregion挂载到system_memory中。



## 文件加载

lua脚本使用file_mappings定义文件加载的路径、地址和大小

```
 file_mappings = {
     main_rom = {
         name = 'examples/bcm4358/bcm4358.rom.bin',
         start = 0x0,
         size = 0x180000
     },
     main_ram = {
         name = 'kernel',
         start = 0x180000,
         size = 0xC0000
     }
 }
```

处理文件加载的逻辑位于init_file_mappings函数

```
static void init_file_mappings(void)
{
    lua_get_global("file_mappings", THROW_ERROR);

    while (lua_next(lua_state, -2)) {
        add_file();  // 具体处理
        lua_pop(lua_state, 1);
    }
}
```

遍历file_mappings每一项，然后调用add_file处理每一个文件映射

```
static void add_file(void)
{
    mapping_fn    = lua_get_string("name", THROW_ERROR);
    mapping_type  = lua_get_string("type", NOTHROW_ERROR); 
    if (!strcasecmp(mapping_fn, "kernel")) {
        mapping_fn = luastate.machine->kernel_filename;
    }

    if (mapping_type && !strcasecmp(mapping_type, "elf")) {
        load_arm_elf(mapping_fn);
    } else {
        load_flat_file(mapping_fn, mapping_start, mapping_size);
    }
}
```

主要是分两种情况进行处理，如果name为kernel，就调用load_arm_elf加载elf文件到内存，否则就使用load_flat_file把文件直接加载到内存的指定位置。

```
static void load_flat_file(const char *file_path, hwaddr start, uint64_t size)
{
    char *fn = NULL;
    if (NULL == (fn = qemu_find_file(QEMU_FILE_TYPE_BIOS, file_path))) {
        error_report("Couldn't find rom image '%s'.", file_path);
        exit(4);
    }

    if (0 > load_image_targphys(fn, start, size)) {
        error_report("Couldn't map file to memory\n");
        exit(5);
    }
    g_free(fn);
}
```



`load_flat_file`主要就是先`qemu_find_file`找到文件，然后使用`load_image_targphys`加载到指定的位置。

## 设置CPU状态和执行回调

init_cpu_state为处理函数

```
static const keyword_table_t kwt[] =
{
    {"reset_pc", init_reset_addr},
    {"env", init_cpu_env},
    {"callbacks", init_cpu_callbacks},
    {{0, 0}}
};

static int handle_keyword(int type, const char *key)
{
    unsigned int n = sizeof(kwt) / sizeof(*kwt);
    int i = 0;
    for (;i < n; i++) {
        if (!strcmp(kwt[i].keyword, key)) {
            kwt[i].fptr(type);
            return 0;
        }
    }
    error_report("keyword '%s' not known", key);
    return -1;
}

static void init_cpu_state(void)
{
    int m_type = 0;
    const char *m_name = NULL;

    lua_get_global("cpu", NOTHROW_ERROR);

    while (lua_next(lua_state, -2)) {
        m_name = lua_tostring(lua_state, -2);
        m_type = lua_type(lua_state, -1);

        handle_keyword(m_type, m_name);

        lua_pop(lua_state, 1);
    }

}
```

主要就是根据关键字来调用对应的处理函数



### init_reset_addr

用于设置系统启动后的PC值

```
static void init_reset_addr(int type)
{
    double d = 0;
    uint64_t addr;
    ARMCPU *cpu = ARM_CPU(luastate.cs);

    if (type != LUA_TNUMBER) {
        return;
    }
    d = lua_tonumber(lua_state, -1);
    lua_number2unsigned(addr, d);

    cpu->rvbar = addr;

    return;
}
```

主要就是从lua脚本中提取reset_pc的值，暂时保存在cpu->rvbar，后面会在init_cpu_env设置pc。

```
static void init_cpu_env(int type)
{
    ........
    if (cpu->rvbar) {
        cpu_set_pc(luastate.cs, cpu->rvbar); // 设置 pc寄存器
    }
```



### init_cpu_env

函数首先根据`cpu->rvbar`设置cpu的pc，然后会设置是否使用thumb指令集，后面会设置miss_max用于检测死循环，最后调用`init_cpu_env_registers`设置其他的回调函数。

```
static void init_cpu_env(int type)
{

    if (cpu->rvbar) {
        cpu_set_pc(luastate.cs, cpu->rvbar);
    }

    luastate.cpu->env.thumb   = lua_get_boolean("thumb", 0);
    luastate.cs->crs.miss_max = lua_get_unsigned("stuck_max", 0);

    init_cpu_env_registers();
}
```



#### miss_max固件代码死循环检测

在嵌入式固件中，在执行过程中如果发生了异常（比如发现某个硬件设备工作不正常），会进入死循环

```
Infinite_Loop:
    b   Infinite_Loop
```

最开始仿真固件时，就会由于某些硬件设备没有仿真正确，从而让固件代码进入了死循环，luaqemu实现了一种方式可以快速的检测发生死循环的位置.

首先在 `init_cpu_env` 中设置 `miss_max`，表示同一个状态进入次数的最大值，状态通过arm_cpu_state_hash计算得到

```
uint64_t arm_cpu_state_hash(CPUState *cs, int flags)
{
    ARMCPU *cpu = ARM_CPU(cs);
    CPUARMState *env = &cpu->env;

    int max_regs = !is_a64(env) ? sizeof(env->regs) / sizeof(env->regs[0]) : sizeof(env->xregs) / sizeof(env->xregs[0]);
    uint64_t hash = !is_a64(env) ? env->regs[15] : env->pc;
    int i = 0;

    if (!is_a64(env)) {
        for (; i < max_regs; i++) {
            hash += env->regs[i];
        }
    } else {
        for (; i < max_regs; i++) {
            hash += env->xregs[i];
        }
    }
    return hash;
}
```

该函数其实就是把所有cpu寄存器的值加在一起作为hash，用于标识每个状态。

然后在`cpu_tb_exec`中每个`tb`执行前会去计算当前状态的执行次数

```
static inline tcg_target_ulong cpu_tb_exec(CPUState *cpu, TranslationBlock *itb)
{
 
    if (cpu->crs.miss_max != 0) {
        record_cpu_state(cpu, 0);
    }
```

record_cpu_state的逻辑相对简单，根据当前cpu的寄存器状态计算hash(`arm_cpu_state_hash`)，然后更新对应hash的执行次数，如果次数达到阈值（miss_count），就调用回调函数

```
void record_cpu_state(CPUState *cpu, int flags)
{
    CPUClass *cc = CPU_GET_CLASS(cpu);
    uint64_t hash = 0;

    if (cc->cpu_state_hash) {
        hash = cc->cpu_state_hash(cpu, flags);
        if (g_hash_table_contains(cpu->crs.cpu_states, GUINT_TO_POINTER(hash))) {
            cpu->crs.miss_count++;
            if (cpu->crs.miss_count >= cpu->crs.miss_max && cpu->crs.state_cb != NULL) {
                cpu->crs.state_cb(cpu);
            }
        } else {
            cpu->crs.ns++;
            if (cpu->crs.ns >= cpu->crs.miss_max) {
                g_hash_table_remove_all(cpu->crs.cpu_states);
                cpu->crs.miss_count = 0;
            }
            g_hash_table_insert(cpu->crs.cpu_states, GUINT_TO_POINTER(hash), GUINT_TO_POINTER(hash));
        }
    }
}
```

回调函数定义

```
static void set_cpu_stuck_state_cb(void)
{
    printf("Found stuck state callback. Make sure to set \"stuck_max\" in env block.\n");
    luastate.cs->crs.state_cb = cpu_stuck_callback;
}
```

cpu_stuck_callback就是调用lua脚本里面指定的回调函数

lua脚本示例

```
cpu = {
    env = {
        stuck_max = 200000,
        stuck_cb = lua_stuck_cb,
... }
}
```

此外还有一个比较关键的点，由于qemu在执行时会把有跳转关系的`tb`链接到一起，所以如果程序一直死循环，则正常情况下`cpu_tb_exec`不会被执行多次，因为如果tb链接到一起后就不会进入`cpu_tb_exec`了。

`luaqemu`的做法是在`tb_find`里面当需要检测死循环时**禁用`tb`链接**。

```
    /* See if we can patch the calling TB. */
    if (last_tb && !qemu_loglevel_mask(CPU_LOG_TB_NOCHAIN) && !cpu->crs.miss_max) {
        if (!tb->invalid) {
            tb_add_jump(last_tb, tb_exit, tb);
        }
    }
```

#### 设置CPU寄存器初始值

```
/* target/arm/cpu.h */
static void init_cpu_env_registers(void)
{
    int reg_i, reg_v = 0;
    char reg_s[4] = {0};

    lua_get_field("regs", 0);

    for (reg_i = 0; reg_i < sizeof(luastate.cpu->env.regs) / sizeof(*(luastate.cpu->env.regs)); reg_i++) {
        snprintf(reg_s, sizeof(reg_s), "r%d", reg_i);
        lua_pushstring(lua_state, reg_s);
        lua_gettable(lua_state, -2); /* get table[name] */

        if (lua_isnil(lua_state, -1)) {
            lua_pop(lua_state, 1);
            continue;
        } else {
            reg_v = lua_tointeger(lua_state, -1);
            debug_print("'%s' -> %x\n", reg_s, reg_v);
            luastate.cpu->env.regs[reg_i] = reg_v;
        }
```

关键逻辑就是根据`regs`的值，设置对应寄存器。



### init_cpu_callbacks

该函数主要处理针对cpu事件的回调函数，比如指令执行、基本块执行等

```
static const cb_keyword_table_t cb_kwt[] =
{
    {"stuck_state_cb",     &luastate.stuck_state_cb, set_cpu_stuck_state_cb},
    {"exec_insn_cb",       &luastate.exec_insn_cb, NULL},
    {"exec_block_cb",      &luastate.exec_block_cb, NULL},
    {"post_exec_block_cb", &luastate.post_exec_block_cb, NULL},
    {{0, 0, 0}}
};
```

其中`stuck_state_cb`在上一节中已经说过

#### 指令执行回调

luaqemu在gen_intermediate_code翻译指令的位置插入了lua_cpu_exec_insn_callback回调函数

```
#ifdef CONFIG_LUAJIT
        uint64_t insn_bytes;
        if (dc->thumb) {
            insn_bytes = arm_lduw_code(env, dc->pc, dc->sctlr_b);
            if (insn_bytes >> 12 == 15 ||
                (insn_bytes >> 12 == 14 && (insn_bytes & (1 << 11)))) { // thumb2, see disas_thumb2_insn use
                insn_bytes = arm_ldl_code(env, dc->pc, dc->sctlr_b);
            }
        } else {
            insn_bytes = arm_ldl_code(env, dc->pc, dc->sctlr_b);
        }

        lua_cpu_exec_insn_callback(dc->pc, insn_bytes);
#endif
```

`lua_cpu_exec_insn_callback` 其实就是调用lua侧的函数

#### 基本块执行回调

在`cpu_tb_exec`中基本块执行前调用`lua_cpu_post_exec_block_callback`，基本块执行后调用`lua_cpu_post_exec_block_callback`。

lua_cpu_post_exec_block_callback和lua_cpu_post_exec_block_callback最后都是调用lua侧的函数

```
/* Execute a TB, and fix up the CPU state afterwards if necessary */
static inline tcg_target_ulong cpu_tb_exec(CPUState *cpu, TranslationBlock *itb)
{


#ifdef CONFIG_LUAJIT
    lua_cpu_exec_block_callback(itb->pc);
#endif

    ret = tcg_qemu_tb_exec(env, tb_ptr);

#ifdef CONFIG_LUAJIT
    lua_cpu_post_exec_block_callback(itb->pc);
#endif
```



## 设置执行断点

处理函数为init_vm_states

```
static void init_vm_states(void)
{
    qemu_add_vm_change_state_handler(lua_vm_state_change, NULL);
    
    lua_get_global("breakpoints", NOTHROW_ERROR);

    while (lua_next(lua_state, -2)) {
        util_breakpoint_insert(lua_tointeger(lua_state, -2), bp_func);
        lua_pop(lua_state, 1);
    }
}
```

主要逻辑就是调用`qemu_add_vm_change_state_handler`注册一个回调函数，当虚拟机状态变化时（比如命中断点、观察点等）调用对应函数。

然后处理breakpoints，用util_breakpoint_insert给地址插入断点，当断点命中执行 bp_func

```
static void util_breakpoint_insert(uint64_t addr, int bp_func)
{
    if(luastate.cs) {
        cpu_breakpoint_insert(luastate.cs, addr, BP_LUA, NULL);
    }
    g_hash_table_insert(breakpoints, GUINT_TO_POINTER(addr), GINT_TO_POINTER(bp_func));
}
```

主要就是调用cpu_breakpoint_insert插入断点，然后把断点的地址和回调函数保存到全局哈希表里面。

下面看一下断点处理流程

```
static void lua_vm_state_change(void *opaque, int running, RunState state)
{
   // 获取当前pc值
    uint64_t old_pc = lua_current_pc();

    switch (state) {
        case RUN_STATE_DEBUG:
                handle_vm_state_breakpoint(old_pc);
            break;
```

在`init_vm_states`处注册了`lua_vm_state_change`回调函数，当命中断点、命中`watchpoint`、单步执行时会触发`RUN_STATE_DEBUG`事件，断点就是在这里处理，最后会进入`handle_vm_state_breakpoint`处理断点事件

```
static inline void handle_vm_state_breakpoint(uint64_t pc)
{
    int bp_func;
    bp_func = GPOINTER_TO_INT(g_hash_table_lookup(breakpoints, GUINT_TO_POINTER(pc)));
    if (bp_func) {
        trigger_breakpoint(bp_func);
        if (pc == lua_current_pc()) {
            cpu_breakpoint_remove(luastate.cs, pc, BP_LUA);
            luastate.bp_pc = pc;
            luastate.bp_pc_ptr = &luastate.bp_pc;
            cpu_single_step(luastate.cs, 1);
        }
        tb_flush(luastate.cs);
    } else {
        if (!luastate.bp_pc_ptr) {
            return;
        }
        cpu_single_step(luastate.cs, 0);
        cpu_breakpoint_insert(luastate.cs, luastate.bp_pc, BP_LUA, NULL);
        vm_start(); 
        luastate.bp_pc_ptr = NULL;
    }
}
```

下面简单介绍下断点的处理流程

1. 断点触发时进入`handle_vm_state_breakpoint`，然后根据pc搜索回调函数，然后`trigger_breakpoint`调用回调函数。
2. 如果回调函数里面没有修改cpu的pc指针，则会调用`cpu_breakpoint_remove`临时删除该断点，并设置`luastate.bp_pc`和`luastate.bp_pc_ptr`为此时的pc，即触发断点的pc。
3. 然后`cpu_single_step`启用cpu的单步模式，下次执行一条指令后，会再次触发断点事件，进入该函数，此时`bp_func`为NULL。
4. 然后会进入`else`分支，首先调用`cpu_single_step`关闭单步执行模式，然后调用`cpu_breakpoint_insert`重新把断点插到之前删除的位置。

至此luaqemu的初始化工作完成，下面分析luaqemu提供给lua脚本的一些api的实现



# Luaqemu API实现分析

## 断点相关

设置断点

```
void lua_breakpoint_insert(uint64_t addr, void (*func)(void))
{
    int bp_func = luaL_ref(lua_state, LUA_REGISTRYINDEX);
    util_breakpoint_insert(addr, bp_func);
}
```

删除断点

```
void lua_breakpoint_remove(uint64_t addr)
{
    util_breakpoint_remove(addr);
}

static void util_breakpoint_remove(uint64_t addr)
{
    int bp_fun = GPOINTER_TO_INT(g_hash_table_lookup(breakpoints, GUINT_TO_POINTER(addr)));

    cpu_breakpoint_remove(luastate.cs, addr, BP_LUA);
    g_hash_table_remove(breakpoints, GUINT_TO_POINTER(addr));
}
```

首先调用cpu_breakpoint_remove把断点撤销，然后从哈希表中删除断点。

## watchpoint

### 新增

lua_watchpoint_insert调用util_watchpoint_insert进行具体的观察点设置

```
void lua_watchpoint_insert(uint64_t addr, uint64_t size, int flags, watchpoint_cb func)
{
  util_watchpoint_insert(addr, size, flags, func);
}

static void util_watchpoint_insert(uint64_t addr, uint64_t size, int flags, watchpoint_cb cb)
{
    watchpoint_t *wp;

    flags |= BP_STOP_BEFORE_ACCESS;
    wp = g_malloc0(sizeof(*wp));
    wp->addr = addr;
    wp->len = size;
    wp->flags = flags;
    wp->fptr = cb;

    cpu_watchpoint_insert(luastate.cs, addr, size, flags, NULL);

    if (NULL == (watchpoints = g_list_append(watchpoints, wp))) {
        error_report("%s error adding watchpoint\n", __func__);
        g_free(wp);
    }
}
```

`util_watchpoint_insert`会调用`cpu_watchpoint_insert`设置观察点，最后把观察点的信息设置到watchpoints列表中

watchpoint在lua_vm_state_change中进行处理，命中观察点时`check_watchpoint`函数会触发RUN_STATE_DEBUG事件

```
static inline void handle_vm_state_watchpoint(CPUWatchpoint *wpt, watchpoint_t *owp)
{
    GList *iterator;
    watchpoint_t *wp;

    if (luastate.old_wp_ptr && owp && luastate.old_wp_ptr == owp) {
        cpu_single_step(luastate.cs, 0);
        cpu_watchpoint_insert(luastate.cs, owp->addr, owp->len, owp->flags, NULL);
        vm_start(); /* this is expensive */
        return;
    }
    for(iterator = watchpoints; iterator; iterator = iterator->next) {
        wp = iterator->data;
        if (wp->addr == wpt->vaddr && wp->len == wpt->len && (wp->flags & wpt->flags)) {

            watchpoint_args_t arg;
            arg.len = wpt->len;
            arg.flags = wpt->flags;
            arg.addr = wpt->vaddr;
            wp->fptr(&arg);

            cpu_watchpoint_remove(luastate.cs, wp->addr, wp->len, wp->flags);
            luastate.old_wp_ptr = wp;
            cpu_single_step(luastate.cs, 1);
            // TODO: introduce flag potentially to control this behavior
            tb_flush(luastate.cs);
            return;
        }
    }
}

static void lua_vm_state_change(void *opaque, int running, RunState state)
{
    switch (state) {
        case RUN_STATE_DEBUG:
            if (luastate.old_wp_ptr) {
                handle_vm_state_watchpoint(NULL, luastate.old_wp_ptr);
                luastate.old_wp_ptr = NULL;
                return;
            }
            if (luastate.cs->watchpoint_hit) {
                handle_vm_state_watchpoint(luastate.cs->watchpoint_hit, NULL);
                luastate.cs->watchpoint_hit = NULL;
            } 
            break;



```

流程如下：

1. 第一次进入`old_wp_ptr`为空，`watchpoint_hit`为命中的观察点结构
2. `handle_vm_state_watchpoint` 函数会遍历观察点列表，找到回调函数进行调用，然后调用`cpu_watchpoint_remove`临时删除观察点
3. `cpu_single_step`启用单步模式，让单步执行一条指令
4. 再次进入`lua_vm_state_change`，此时`luastate.old_wp_ptr`为上次触发观察点的结构
5. 此时关闭单步模式，然后重新把观察点插入

### 删除

从全局watchpoints列表中删除并调用`cpu_watchpoint_remove`撤销观察点。

```
void lua_watchpoint_remove(uint64_t addr, uint64_t size, int flags)
{
  util_watchpoint_remove(addr, size, flags);
}


static void util_watchpoint_remove(uint64_t addr, uint64_t size, int flags)
{
    GList *iterator;
    watchpoint_t *wp;

    for(iterator = watchpoints; iterator; iterator = iterator->next) {
        wp = iterator->data;
        if (wp->addr == addr && wp->len == size && wp->flags == flags) {
            watchpoints = g_list_delete_link(watchpoints, iterator);
            cpu_watchpoint_remove(luastate.cs, wp->addr, wp->len, wp->flags);
            g_free(wp);
            return;
        }
    }
    error_report("%s could not find matching watchpoint\n", __func__);
}
```



## 执行相关

lua_continue让虚拟机继续运行

```
void lua_continue(void)
{
    vm_start();
}
```



## 寄存器操作

lua_set_pc 设置pc寄存器的值

```
void lua_set_pc(uint64_t addr)
{
    if (!is_a64(&luastate.cpu->env)) {
        luastate.cpu->env.regs[15] = addr;
    } else {
        luastate.cpu->env.pc = addr;
    }
}
```



lua_get_register 获取寄存器的值，就是根据索引去cpu->env结构里面取

```
uint64_t lua_get_register(uint8_t reg)
{
    if (!is_a64(&luastate.cpu->env)) {
        if (reg >= sizeof(luastate.cpu->env.regs) / sizeof(*(luastate.cpu->env.regs))) {
            error_report("%s '%d' exceeds cpu registers", __func__, reg);
            return 0;
        }
        return luastate.cpu->env.regs[reg];
    } else {
        if (reg >= sizeof(luastate.cpu->env.xregs) / sizeof(*(luastate.cpu->env.xregs))) {
            error_report("%s '%d' exceeds cpu registers", __func__, reg);
            return 0;
        }
        return luastate.cpu->env.xregs[reg];
    }
}
```

lua_set_register 设置寄存器的值，实现类似。

## 读写虚拟机内存

API列表

```
uint8_t lua_read_byte(uint64_t);
uint16_t lua_read_word(uint64_t);
uint32_t lua_read_dword(uint64_t);
uint64_t lua_read_qword(uint64_t);
void lua_read_memory(uint8_t *, uint64_t, size_t);
void lua_write_byte(uint64_t, uint8_t);
void lua_write_word(uint64_t, uint16_t);
void lua_write_dword(uint64_t, uint32_t);
void lua_write_qword(uint64_t, uint64_t);
void lua_write_memory(uint64_t, uint8_t *, size_t);
```

以lua_write_memory为例，这个是往某个地址写一段内存

```
static inline int lua_memory_rw(target_ulong addr, uint8_t *buf, int len, bool is_write)
{
	CPUClass *cc = CPU_GET_CLASS(luastate.cs);
	if (cc->memory_rw_debug) {
		return cc->memory_rw_debug(luastate.cs, addr, buf, len, is_write);
	}
	return cpu_memory_rw_debug(luastate.cs, addr, buf, len, is_write);
}

void lua_write_memory(uint64_t addr, uint8_t *src, size_t len)
{
	lua_memory_rw(addr, src, len, 1);
}

void lua_read_memory(uint8_t *dest, uint64_t addr, size_t size)
{
	lua_memory_rw(addr, dest, size, 0);
}
```

主要就是调用 `cpu_memory_rw_debug` 进行内存的写



## MMIO内存处理

### 注册

```
void lua_trapped_physregion_add(uint64_t addr, uint64_t size, TprReadCb readCb, TprWriteCb writeCb)
{
    util_trapped_physregion_add(addr, size, readCb, writeCb);    
}

static void util_trapped_physregion_add(uint64_t addr, uint64_t size, TprReadCb readCb, TprWriteCb writeCb)
{
    MemoryRegion *sysmem = get_system_memory();    

    tpr = g_malloc0(sizeof(TrappedPhysRegion));

    tpr->readCb = readCb;
    tpr->writeCb = writeCb;

    tpr->ops.read = trapped_physregion_read;
    tpr->ops.write = trapped_physregion_write;
    tpr->ops.endianness = DEVICE_NATIVE_ENDIAN;
    snprintf(tpr->name, TPR_NAME_SIZE, "TPR_%" PRIx64 "-%" PRIx64 , addr, (addr+size));

    memory_region_init_io(&tpr->region, NULL, &tpr->ops, tpr, tpr->name, size);
    memory_region_add_subregion(sysmem, addr, &tpr->region);

    if (NULL == (trapped_physregions = g_list_append(trapped_physregions, tpr))) {
        memory_region_del_subregion(sysmem, &tpr->region);
        g_free(tpr);
    }
}
```

核心点就是调用`memory_region_init_io`注册`mmio`内存，使得对该内存的读写会调用对应的回调函数，并把`tpr`作为第一个参数传入。



当对内存读写时会调用 `trapped_physregion_read` 和 `trapped_physregion_write`

```
/* function stolen from memory.c */
static hwaddr memory_region_to_absolute_addr(MemoryRegion *mr, hwaddr offset)
{
    MemoryRegion *root;
    hwaddr abs_addr = offset;

    abs_addr += mr->addr;
    for (root = mr; root->container; ) {
        root = root->container;
        abs_addr += root->addr;
    }

    return abs_addr;
}

uint64_t trapped_physregion_read(void *opaque, hwaddr addr, unsigned size)
{
    TrappedPhysRegion *tpr = opaque;
    TprReadCbArgs cbArgs;
    hwaddr addr2;

    addr2 = memory_region_to_absolute_addr(&tpr->region, addr);    

    cbArgs.opaque = opaque;
    cbArgs.addr = addr2;
    cbArgs.size = size;
    tpr->readCb(&cbArgs);

    return 0;
}

void trapped_physregion_write(void *opaque, hwaddr addr, uint64_t data, unsigned size)
{
    TrappedPhysRegion *tpr = opaque;
    TprWriteCbArgs cbArgs;
    hwaddr addr2;

    addr2 = memory_region_to_absolute_addr(&tpr->region, addr);

    cbArgs.opaque = opaque;
    cbArgs.addr = addr2;
    cbArgs.data = data;
    cbArgs.size = size;
    tpr->writeCb(&cbArgs);
}
```

核心逻辑就是首先获取访问内存的地址，然后调用`TrappedPhysRegion`里面的回调函数。



### 删除

找到对应的region，然后调用`memory_region_del_subregion`删掉。

```
void lua_trapped_physregion_remove(uint64_t addr, uint64_t size)
{
    util_trapped_physregion_remove(addr, size);
}

static void util_trapped_physregion_remove(uint64_t addr, uint64_t size)
{
    GList *iterator;
    TrappedPhysRegion *tpr;
    MemoryRegion *sysmem = get_system_memory();

    for(iterator = trapped_physregions; iterator; iterator = iterator->next) {
        tpr = iterator->data;
        if (tpr->region.addr == addr && tpr->region.size == size) {
            trapped_physregions = g_list_delete_link(trapped_physregions, iterator);
            memory_region_del_subregion(sysmem, &tpr->region);
            g_free(tpr);
            return;
        }
    }
}
```



# 总结

本文分析了luaqemu的实现，luaqemu支持监控基本块、指令级别的监控，支持观察点、断点的设置，支持mmio内存的申请，而且提供了友好的用户接口，可以简单的对虚拟机内存进行读写，唯一不足的是没有中断相关的API。

# 参考链接

```
https://blog.csdn.net/huang987246510/article/details/104012839
https://comsecuris.com/blog/posts/luaqemu_bcm_wifi/
```































