## STM32F103C8T6 简单框架

> 由Horizon战队框架简化而来。
> 

### 注意事项

- freertos报错：替换路径问题(生成代码为arm编译器，实际使用应为gcc)：

`D:\RoboMaster\2026\lidar\Horizon_frame_f1\Middlewares\Third_Party\FreeRTOS\Source\portable\RVDS\ARM_CM3` 中的两个文件替换为 `C:\Users\17273\STM32Cube\Repository\STM32Cube_FW_F1_V1.8.6\Middlewares\Third_Party\FreeRTOS\Source\portable\GCC\ARM_CM3`中的文件即可。

- eide 工具链编译失败，内存不够（keil可正常烧录）

修改申请空间大小：
1. IRAM1  0x20000000 0x20000
2. IROM1 0x08000000 0x80000