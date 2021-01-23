### 编译

```
./build.sh clean
./build.sh
```


### 运行

```
../../build/arm-softmmu/qemu-system-arm -M netduino2 -kernel startup.bin -nographic
```


输出示例

```
$ ../../build/arm-softmmu/qemu-system-arm -M netduino2 -kernel startup.bin -nographic 
main_func!
demo_irq_func!
```
