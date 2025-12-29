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

- 视觉通信协议
> 使用串口助手测试正常
1. 接收
```c
uint8_t test_packet[15] = {
    // 0. 帧头 (必须是 0xCD)
    0xCD, 
    // 1-4. Pitch 角度 (float: 10.5) -> Hex: 0x41280000 (小端: 00 00 28 41)
    0x00, 0x00, 0x28, 0x41,

    // 5-8. Yaw 角度 (float: -5.0) -> Hex: 0xC0A00000 (小端: 00 00 A0 C0)
    0x00, 0x00, 0xA0, 0xC0,

    // 9. 标志位字节
    // bit0-2: State=1 (001)
    // bit3:   Shoot=1 (1) -> 0x08
    // bit4:   Target=1 (1) -> 0x10
    // 组合: 0x01 | 0x08 | 0x10 = 0x19
    0x19,

    // 10-13. 时间戳 (uint32: 1000) -> Hex: 0x000003E8 (小端: E8 03 00 00)
    0xE8, 0x03, 0x00, 0x00,
    
    // 14. 帧尾 (必须是 0xDC)
    0xDC
};
```
2. 发送
```c
uint8_t test_packet[16]={
	0xCD, 
	0x00, 0x0C, 0x28, 0x41,  // pitch float
	0x00, 0x20, 0xA3, 0xC0,  // yaw float
	0x01, 0x0A, 0x00, 0x00,  // 时间int
	0x00, 0x00, 
	0xDC
}
```
---
## TODO List

- [X] 更新通信协议
- [ ] 添加陀螺仪对云台进行更加精细控制
- [ ] 添加角速度收发


---
> [!TIP]
> 角速度收发与处理

视觉：接收 **实时角速度**，发送  **角速度前馈**

电控：接收 **角速度前馈**，发送  **实时角速度**


电控接收实时角速度后处理（单位环）： 角速度前馈先乘dt积分成角度，再加到PID的ref上。

**前馈角速度 × dt → 角度增量 → 加到角度指令 → 原 PID 不动**

	


