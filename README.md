# 三波段红外 + 紫外火焰探测器

基于 STM32L151、STM32Cube HAL 的裸机固件。使用 3.8/4.5/5.0 um 三路红外 ADC 和紫外脉冲共同识别火焰；USART1 用于命令行与调试，USART2 作为树莓派配置通信口。

## 当前功能

- **红外检测**：100 Hz 采集三路 ADC；每通道 IIR 滤波，最近 50 点去直流后计算均方功率，最近 256 点通过定点 FFT 提取闪烁频率与频带能量。以 4.5 um 为主通道，结合光谱比和达到有效功率的参考通道进行判定。`IDLE -> WARNING -> FIRE` 状态机使用确认积分及掉线迟滞。
- **点火包络识别**：与红外确认过程并行，比较点火峰值与后续稳定能量，识别近距离打火机式快速衰减。`LIGHTER` 可在持续能量满足条件后单向升级为真实火焰；开关可配置。
- **紫外检测**：TIM3 捕获脉冲宽度与时间戳，按可配置脉宽过滤，并在滑动时间窗内计数。紫外状态机同样具有 `IDLE/WARNING/FIRE`、确认积分和掉线迟滞。
- **最终报警**：正常模式要求 UV 与 IR 同时处于 `FIRE`；演示模式只要求 UV。`ALM` 低电平表示报警，高电平表示正常。
- **故障与恢复**：任一 ADC 通道连续处于 0 或 4095 达 20 秒时，`BUG` 输出低电平；三路恢复有效值并持续 2 秒后解除。`RECOVER` 输入下降沿在主循环中清除 IR、UV 和最终报警状态。
- **参数保存**：灵敏度、检测阈值、模式等配置保存在片内 DATA EEPROM，读取时进行版本、范围和 CRC 校验。
- **配置通信**：USART2 通过 DMA 收发树莓派二进制协议，支持握手、读取全部配置、逐项修改及固定格式结果应答。协议为严格停等，重复请求重放缓存响应，不重复写 EEPROM。

## 运行路径

上电初始化外设、BSP、EEPROM 和检测器后，主循环按以下顺序运行：

```text
RECOVER处理 -> USART2协议收/发/超时 -> ADC故障监控 -> USART1命令行
  -> 测试模式：定时采集与可控打印，不推进火焰状态机
  -> 应用模式：UV检测 -> 非演示时IR检测 -> 组合报警输出
  -> 喂看门狗
```

| 模式 | 报警判定 | 用途 |
| --- | --- | --- |
| 正常模式 | UV `FIRE` 且 IR `FIRE` | 正式双传感器探测 |
| 演示模式 | UV `FIRE` | 仅UV快速演示，IR任务暂停 |
| 测试模式 | 不执行火焰报警判定 | 采样、打印及串口硬件测试 |

演示模式、测试模式和红外点火包络开关均可通过 USART1 命令切换并保存，重启后从 EEPROM 恢复。测试模式下可分别开关 IR/UV 打印；`uart raw on` 会暂停 USART2 配置协议，由本地串口测试命令独占该端口，关闭后树莓派需要重新握手。

## 代码结构

```text
Core/             CubeMX 生成的外设初始化、中断入口和主循环
Code/bsp/         ADC、TIM、UART、EEPROM、GPIO、报警等硬件封装
Code/ap/          IR/UV检测、参数管理、配置协议和应用工具
Code/modules/     USART1命令行、CRC等通用模块
Drivers/          STM32 HAL 与 CMSIS
docs/             通信协议、算法说明和测试资料
MDK-ARM/          Keil 工程
```

主要入口为 `Core/Src/main.c`；红外、紫外和配置协议分别位于 `Code/ap/src/ap_ir.c`、`Code/ap/src/ap_uv.c`、`Code/ap/src/ap_uart_protocol.c`。

## 构建与调试

使用项目根目录的 `Makefile` 和 ARM GNU 工具链执行 `make -j8`，生成的 ELF、HEX、BIN 位于 `build/`。也可使用 `MDK-ARM/fire_source_identification.uvprojx` 打开 Keil 工程；引脚和外设配置以 `fire_source_identification.ioc` 为准。

USART1 波特率为 115200，输入 `help` 查看完整命令。常用命令包括 `show mode on/off`、`test mode on/off`、`profile on/off` 和 `uart raw on/off`。USART2 为 115200 8N1，仅传输配置协议，不输出文本日志；协议格式、MessageID 和配置字段见 [树莓派通信协议](docs/树莓派与火焰探测板通信协议.md)。

> 当前阈值基于已有火源与干扰测试逐步标定，部署前仍需按目标燃料、距离、角度及环境光条件做整机验证。
