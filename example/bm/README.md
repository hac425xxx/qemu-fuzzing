### 编译

```
./build.sh clean
./build.sh
```


### 运行

```
../../build/arm-softmmu/qemu-system-arm -M netduino2 -kernel startup.bin -nographic
```