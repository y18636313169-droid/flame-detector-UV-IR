# 火源识别模块 — 代码架构总览

永远使用中文回答；
只修改必要的函数注释，每次修改不要把无关注释加来删去；

## 目录结构

```
fire_source_identification/
├── Core/                          # CubeMX 生成层
│   ├── Src/
│   │   ├── main.c                 # 入口 + HAL 弱回调覆盖（USER CODE 4）
│   │   ├── tim.c                  # TIM3/5/6 初始化（CubeMX）
│   │   ├── usart.c                # USART1/USART2 初始化 + DMA（CubeMX）
│   │   ├── adc.c                  # ADC1 初始化 + DMA（CubeMX）
│   │   ├── dma.c                  # DMA 中断配置
│   │   ├── gpio.c                 # GPIO 初始化
│   │   ├── iwdg.c                 # IWDG 初始化
│   │   ├── stm32l1xx_it.c         # ISR 向量（TIM3/5/6, USART1/2, DMA1）
│   │   └── stm32l1xx_hal_msp.c   # MSP 初始化/反初始化
│   └── Inc/                       # 对应头文件
├── Drivers/STM32L1xx_HAL_Driver/  # HAL 库
├── Code/
│   ├── bsp/                       # 板级支持包 — 硬件抽象层
│   │   ├── inc/
│   │   │   ├── hal_gpio.h         # GPIO 读写/翻转
│   │   │   ├── hal_led.h          # LED 闪烁/工作/停止（TIM5 中断驱动）
│   │   │   ├── hal_uart.h         # UART DMA 环形缓冲驱动
│   │   │   ├── hal_tim.h          # TIM3 双通道脉宽捕获 + 环形缓冲
│   │   │   ├── hal_adc.h          # ADC DMA 3通道连续转换
│   │   │   ├── hal_alarm.h        # ALM1/ALM2 报警输出驱动
│   │   │   ├── hal_eeprom.h       # DATA EEPROM 扇区读写 + CRC校验
│   │   │   ├── hal_iwdg.h         # 独立看门狗 ISR置标志→主循环喂狗
│   │   │   ├── hal_board.h        # BSP_BoardInit 聚合
│   │   │   └── ring_buffer.h      # 通用环形缓冲区（多实例/ISR安全）
│   │   └── src/                   # 对应实现
│   ├── ap/                        # 应用层
│   │   ├── inc/
│   │   │   ├── ap_adc.h           # ADC DMA 启动 + 原始值读取
│   │   │   ├── ap_util.h          # 工具函数(lerp) + IR_TEST_MODE 开关
│   │   │   ├── ap_eeprom.h        # 参数分组(ADC/UV/IR) + 默认值 + 掉电保存
│   │   │   ├── ap_uart_protocol.h # 串口协议组包/解析/ACK
│   │   │   ├── ap_uv.h            # 紫外火焰检测（窗口计数+状态机）
│   │   │   └── ap_ir.h            # 红外三波段多光谱融合检测（五判据+状态机）
│   │   └── src/                   # 对应实现
│   └── modules/
│       ├── cmdline/               # 命令行框架 (cmd_parser_task)
│       └── crc/                   # CRC16-CCITT + Modbus 双算法
├── MDK-ARM/                       # Keil 项目文件
├── .eide/                         # EIDE 项目文件
└── AGENTS.md                      # 本文件
```

## 中断分发链

```
stm32l1xx_it.c
  ├── TIM3_IRQHandler → HAL_TIM_IRQHandler
  │     └── main.c (USER CODE 4):
  │           HAL_TIM_IC_CaptureCallback → BSP_TIM_IC_CaptureHandler   [hal_tim.c]
  │           HAL_TIM_PeriodElapsedCallback:
  │             ├── TIM5 → BSP_LED_TickHandler                         [hal_led.c]
  │             ├── TIM6 → AP_UART_CheckTimeout() + task_10ms()        [main.c]
  │             │          task_10ms: 仅置位标志 (AP_IR_FeedIsr,       [main.c]
  │             │                    test_print_pending)
  │             └── TIM3 → BSP_TIM_IC_PeriodHandler                    [hal_tim.c]
  ├── USART1_IRQHandler → HAL_UART_IRQHandler
  ├── USART2_IRQHandler → HAL_UART_IRQHandler
  ├── DMA1_Channel1_IRQHandler → hdma_adc
  ├── DMA1_Channel4_IRQHandler → hdma_usart1_tx
  ├── DMA1_Channel5_IRQHandler → hdma_usart1_rx
  ├── DMA1_Channel6_IRQHandler → hdma_usart2_rx
  └── DMA1_Channel7_IRQHandler → hdma_usart2_tx
```

## BSP 层 API 总览

### hal_gpio.h — GPIO I/O
| 函数 | 说明 |
|------|------|
| `BSP_GPIO_Init()` | GPIO 补充初始化 |
| `BSP_GPIO_WritePin(port, pin, level)` | 设置引脚电平 |
| `BSP_GPIO_ReadPin(port, pin)` | 读取引脚电平 |
| `BSP_GPIO_TogglePin(port, pin)` | 翻转引脚电平 |

### hal_led.h — LED 控制（TIM5 中断驱动）
| 函数 | 说明 |
|------|------|
| `BSP_LED_On()` / `BSP_LED_Off()` / `BSP_LED_Toggle()` | 内联函数，即时操作 |
| `BSP_LED_Blink(count, interval_ms)` | 闪烁 N 次后自动停止，非阻塞 |
| `BSP_LED_Work(interval_ms)` | 持续心跳灯，直到调 Stop |
| `BSP_LED_Stop()` | 停止所有闪烁，熄灭 LED |
| `BSP_LED_TickHandler()` | ISR 回调，由 TIM5 更新中断触发 |

### hal_uart.h — UART DMA 环形缓冲驱动
| 函数 | 说明 |
|------|------|
| `BSP_UART_Init(id)` | 初始化串口（COM=USART1, DBG=USART2），启动 RX DMA 循环 |
| `BSP_UART_Write(id, data, len)` | 写入 TX 环形缓冲，DMA 自动搬运（非阻塞） |
| `BSP_UART_WriteBlock(id, data, len, ms)` | 阻塞发送，带超时 |
| `BSP_UART_Read(id, buf, len)` | 从 RX DMA 循环缓冲读取 |
| `BSP_UART_GetRxDataLen(id)` | 查询 RX 缓冲可读数据长度 |
| `BSP_UART_Peek(id, out, offset)` | 查看 RX 缓冲指定偏移 1 字节（不移动读指针） |
| `BSP_UART_PeekPacket(id, out, off, len)` | 查看 RX 缓冲一段连续数据 |
| `BSP_UART_Consume(id, len)` | 推进 RX 读指针 |
| `BSP_UART_IsTxComplete(id)` | 查询 TX 是否发送完成 |
| `BSP_UART_Printf(fmt, ...)` | 通过 DBG 串口格式化打印 |

### hal_tim.h — TIM3 双通道脉宽捕获
| 函数 | 说明 |
|------|------|
| `BSP_TIM_IC_Init(id)` | 绑定 htim3，初始化环形缓冲 |
| `BSP_TIM_IC_Start(id)` | 启动捕获 |
| `BSP_TIM_IC_Stop(id)` | 停止捕获 |
| `BSP_TIM_IC_ReadPulse(id, &pulse)` | 从环形缓冲读取一条脉冲 |
| `BSP_TIM_IC_ReadAllPulse(id, pulses, max)` | 批量读取所有脉冲，返回数量 |
| `BSP_TIM_IC_ClearAllPulse(id)` | 清空环形缓冲 |
| `BSP_TIM_IC_CaptureHandler(htim)` | 双通道捕获状态机（由 main.c 回调转发） |
| `BSP_TIM_IC_PeriodHandler(htim)` | 溢出计数（由 main.c 回调转发） |

### hal_adc.h — ADC DMA 3通道连续转换
| 函数 | 说明 |
|------|------|
| `BSP_ADC_Init()` | 检查 hadc 句柄 |
| `BSP_ADC_StartDMA()` | 启动 DMA 连续转换（3通道，DMA1_CH1 CIRCULAR） |
| `BSP_ADC_StopDMA()` | 停止 DMA 转换 |
| `BSP_ADC_ReadValue(ch, &raw)` | 读取指定通道最新原始值 |

### hal_alarm.h — ALM1/ALM2 报警输出
| 函数 | 说明 |
|------|------|
| `BSP_ALARM_Init()` | 初始化报警引脚，默认高电平（无报警） |
| `BSP_ALARM_Set()` | 置报警 → ALM1+ALM2 输出低电平 |
| `BSP_ALARM_Reset()` | 清报警 → ALM1+ALM2 恢复高电平 |
| `BSP_ALARM_SetSingle(id)` | 单路置报警 |
| `BSP_ALARM_ResetSingle(id)` | 单路清报警 |

### hal_eeprom.h — DATA EEPROM 扇区读写
| 函数 | 说明 |
|------|------|
| `BSP_EEPROM_Read(addr, buf, size)` | 内存映射读（4字节对齐） |
| `BSP_EEPROM_Write(addr, buf, size)` | 字写入 DATA EEPROM |
| `BSP_EEPROM_LoadSector(addr, buf, size, magic, def_fn, crc_off)` | 加载+魔数+CRC校验，失败写默认值 |
| `BSP_EEPROM_SaveSector(addr, buf, size, crc_off)` | 计算CRC并写入扇区 |

### hal_iwdg.h — 独立看门狗（ISR置标志→主循环喂狗）
| 函数 | 说明 |
|------|------|
| `BSP_IWDG_RequestFeed()` | ISR中调用，置喂狗请求标志 |
| `BSP_IWDG_CheckAndRefresh()` | 主循环调用，检查标志并喂狗 |

### ring_buffer.h — 通用环形缓冲区
| 函数 | 说明 |
|------|------|
| `ring_buffer_init(rb, pool, elem_sz, cap)` | 初始化 |
| `ring_buffer_push(rb, elem)` | 压入（满则丢弃+溢出标志） |
| `ring_buffer_push_overwrite(rb, elem)` | 压入（满覆盖最旧） |
| `ring_buffer_pop(rb, elem)` | 弹出 FIFO |
| `ring_buffer_peek(rb, elem, offset)` | 查看偏移元素，不移动读指针 |
| `ring_buffer_count(rb)` | 有效元素个数 |
| `ring_buffer_space(rb)` | 剩余空间 |
| `ring_buffer_clear(rb)` | 清空 |
| `ring_buffer_overflowed(rb)` | 检查并清除溢出标志 |

### hal_board.h — BSP 初始化聚合
| 函数 | 说明 |
|------|------|
| `BSP_BoardInit()` | 依次调 BSP_GPIO/ALARM/ADC/TIM/UART Init |

## AP 层 API 总览

### ap_util.h — 通用工具
| 函数/宏 | 说明 |
|---------|------|
| `lerp_u32(min, max, level, levels)` | 无符号 32 位线性插值（static inline） |
| `IR_TEST_MODE` | 测试模式开关宏（定义在 ap_util.h） |

### ap_adc.h — ADC 原始值读取
| 函数 | 说明 |
|------|------|
| `AP_ADC_Init()` | 启动 ADC DMA 转换 |
| `AP_ADC_GetLatest(ch)` | 读取指定通道最新原始值 |

> 注：IR 算法直接通过 BSP_ADC_ReadValue 读取原始 DMA 值，不经过 AP_ADC 层。ap_adc 仅提供最简封装。

### ap_eeprom.h — EEPROM参数存储管理
| 函数 | 说明 |
|------|------|
| `AP_EEPROM_Init()` | 加载所有参数组（ADC/UV/IR），魔数+CRC校验，失败写默认值 |
| `AP_EEPROM_ADC_Get/Save/Reset` | ADC 阈值参数存取 |
| `AP_EEPROM_UV_Get/Save/Reset` | UV 检测参数存取（灵敏度+4组min/max） |
| `AP_EEPROM_IR_Get/Save/Reset` | IR 多光谱融合参数存取（灵敏度+4组min/max+2固定频率） |

### ap_uart_protocol.h — 串口通信协议
| 函数 | 说明 |
|------|------|
| `AP_UART_ProtocolInit()` | 清 TX 缓冲，复位解析状态机 |
| `AP_UART_Send(fcode, data, len)` | 发送一帧（SEQ 内部自增，默认需应答） |
| `AP_UART_TxTask()` | TX 状态机：IDLE→SENDING→WAIT_ACK |
| `AP_UART_RxTask()` | RX 状态机：三步入解析→CRC→FCode分发 |
| `AP_UART_CheckTimeout()` | ACK 超时检查（TIM6 每 1ms 调用） |
| `AP_UART_IsIdle()` | 查询 TX+RX 是否空闲 |

#### TX 状态机
```
IDLE:    tx_buf_check() → 非空且 entry_len 合法 → SENDING
SENDING: tx_buf_pop() → SEQ++ → 组帧→CRC→发送 → WAIT_ACK
WAIT_ACK: tx_ack_flag 被 RX 置位 → tx_buf_consume() → IDLE
          CheckTimeout 超时 → 重传 / 丢弃
```

#### RX 状态机
```
STEP_STX:  Peek offset 0=STX → 匹配 → STEP_LEN（不 consume）
STEP_LEN:  Peek offset 2=LEN → 合法性检查 → STEP_DATA
STEP_DATA: PeekPacket 整帧 → CRC 校验 → ETX 检查 → Consume → process_rx_frame
```

### ap_uv.h — 紫外火焰检测
| 函数 | 说明 |
|------|------|
| `AP_UV_Init(on_fire)` | 从 EEPROM 加载参数初始化紫外检测 |
| `AP_UV_Feed()` | 从 BSP 读取脉冲加入历史队列 |
| `AP_UV_Process(now)` | 窗口计数 → 状态机决策 |
| `AP_UV_Task()` | Feed + Process 打包（主循环调用） |
| `AP_UV_SetLevel(level)` | 运行时调整灵敏度 0~9 |
| `AP_UV_Process_Reset()` | 重置状态机到 IDLE |
| `AP_UV_GetState()` | 返回当前状态 |

#### 紫外状态机
```
IDLE ──(窗口计数≥threshold)──→ WARNING ──(持续≥confirm_ms)──→ FIRE
  ↑                             ↑ (掉线)                       │
  └─────────────────────────────┘                               │
                                                   (窗口=0 超 clear_ms)──┘
```
灵敏度 0~9 级线性插值：window_ms=3500~1000, threshold=35~5, confirm_ms=3500~0, clear_ms=10000~3000

### ap_ir.h — 红外三波段多光谱融合火焰检测
| 函数 | 说明 |
|------|------|
| `AP_IR_Init()` | 从EEPROM加载参数，初始化历史窗口和状态机 |
| `AP_IR_FeedIsr()` | ISR(TIM6)调用，仅置位 volatile 标志 |
| `AP_IR_Feed()` | 从 ADC 读最新值推入 50 点历史窗口（不进状态机） |
| `AP_IR_Task()` | 主循环调用，检查标志→Feed→Process(全流程检测) |
| `AP_IR_SetLevel(level)` | 运行时调整灵敏度 0~9，线性插值所有参数 |
| `AP_IR_GetState()` | 返回当前状态 IR_STATE_IDLE/WARNING/FIRE |
| `AP_IR_Reset()` | 重置状态机到 IDLE，清历史窗口 |
| `AP_IR_GetFeatures(power, zcr, &r45_38, &r45_50)` | 获取实时特征值（调试用） |
| `AP_IR_SetConfig(...)` | 设置 4+2 组参数 min/max 范围 |
| `AP_IR_GetParams(...)` | 获取当前运行参数 |
| `AP_IR_DebugProcess(now)` | 测试模式：信号链处理+打印，不进状态机 |

#### 算法流程
```
Feed(ADC值→50点历史窗口) → 窗口满后:
  线性化 → 去直流(减均值) → 40Hz IIR低通滤波(Q15定点)
  → 平均功率(P3.8/P4.5/P5.0 ×1000)
  → 过零率(ZCR3.8/ZCR4.5/ZCR5.0 Hz)
  → 光谱比(R4.5/3.8, R4.5/5.0 ×1000)
  → 五判据串联:
      ① P4.5 > 功率阈值
      ② R4.5/3.8 > 光谱比阈值(高温热源鉴别)
      ③ R4.5/5.0 > 光谱比阈值(背景辐射鉴别)
      ④ ZCR4.5 ∈ [1.5Hz, 20Hz] (闪烁频率验证)
      ⑤ |ZCR4.5-ZCR3.8| < 5Hz (频率一致性)
  → 全部通过: IDLE→WARNING→FIRE
```

#### 状态机
```
IDLE ──(五判据通过)──→ WARNING ──(持续≥confirm_ms)──→ FIRE
  ↑                      ↑ (中断)                     │
  └──────────────────────┘                            │
                                          (等待外部复位)──┘
```

#### 参数管理
- **随灵敏度 0~9 插值**: 功率阈值(pwr)、光谱比(r38/r50)、确认时长(cfm) — 各带 min/max
- **固定值(不插值)**: 频率下限(1.5Hz)、频率上限(20Hz) — 火焰频率与灵敏度无关

### 串口协议帧格式

```
┌──────┬──────┬──────┬──────┬──────────┬──────┬──────┐
│ STX  │ SEQ  │ LEN  │ FCODE│  DATA    │ CRC16│ ETX  │
│ 1B   │ 1B   │ 1B   │ 1B   │  nB      │ 2B   │ 1B   │
└──────┴──────┴──────┴──────┴──────────┴──────┴──────┘
```

- STX=0x02, ETX=0x03（宏定义可改）
- LEN = FCODE(1B) + DATA(nB) + CRC16(2B) = n+3（LEN 不包含自身）
- CRC 范围：从 LEN 到 DATA 末尾，共(LEN-1)字节
- SEQ 在 `AP_UART_Send` 内部统一自增，外部不涉足
- 所有发送默认需要 ACK，超时重传（TIM6 每 1ms 递减 `timeout_cnt`）
- FCode 回调表：GET_PARAM / SET_PARAM / HEARTBEAT，未注册 FCode 回复 data[0]=1

## 初始化顺序（main.c）

```c
MX_GPIO_Init();     MX_DMA_Init();      MX_ADC_Init();
MX_IWDG_Init();     MX_USART1_UART_Init();
MX_TIM5_Init();     MX_TIM6_Init();     MX_USART2_UART_Init();
MX_TIM3_Init();
BSP_BoardInit();                    // BSP 各模块初始化 (GPIO→ALARM→ADC→TIM→UART)
AP_EEPROM_Init();                   // 加载 ADC/UV/IR 3组参数(EEPROM→内存)
AP_IR_Init();                       // 从 EEPROM 加载参数并初始化红外检测
AP_UV_Init(NULL);                   // 从 EEPROM 读取参数并初始化紫外检测
AP_ADC_Init();                      // 启动 ADC DMA
AP_UART_ProtocolInit();             // 清协议缓冲
```

## 主循环框架（while(1)）

```c
#if !defined(IR_TEST_MODE)
  // 正常模式: 串口通信
  AP_UART_TxTask();      // TX 状态机
  AP_UART_RxTask();      // RX 状态机
#endif

cmd_parser_task();     // 命令行解析

#if defined(IR_TEST_MODE)
  // 测试模式: 仅 Feed ADC 填历史窗口, 不进状态机
  AP_IR_Feed();
  if (test_print_pending && test_print_enabled) {
      test_print_pending = 0;
      test_print_data();          // 紧凑格式打印 UV 脉冲
      AP_IR_DebugProcess(now);    // 打印 DC/Power/ZCR/光谱比
  }
#else
  // 正常模式: 紫外检测 + 红外检测 + 双重确认
  AP_UV_Task();                   // Feed+Process → 状态机
  AP_IR_Task();                   // Feed+Process → 五判据 → 状态机

  // 双重确认: UV && IR 同时触发 → 火警上报 + 硬件报警输出
  if (UV_STATE_FIRE && IR_STATE_FIRE) {
      BSP_ALARM_Set();
      AP_UART_Send(FIRE_ALARM);
  } else {
      BSP_ALARM_Reset();
  }
#endif

BSP_IWDG_CheckAndRefresh();   // 喂狗
```

## 关键数据流示意

```
ADC 硬件 → DMA → adc_dma_buf[3] (BSP DMA1_CH1 CIRCULAR)
                    ↓
            AP_IR_Feed() / AP_IR_Feed()
            → history_push() 到 50 点历史窗口 (3通道独立)
            → AP_IR_Process() / AP_IR_DebugProcess()
            → 去除DC + IIR 40Hz Q15 定点滤波
            → 特征提取(Power/ZCR/光谱比)
            → 五判据串联 → 状态机(IDLE→WARNING→FIRE)
            → 与 UV 状态逻辑与 → 最终火警

TIM3 CH1/CH2 → 双通道捕获 (BSP)
                    ↓ ISR: 硬件捕获→ring_buffer(20组)
                    主循环: AP_UV_Task() → Feed → 历史队列
                    → 窗口计数 → 状态机(IDLE→WARNING→FIRE)

AP_UART_Send → TX事件缓冲 → TxTask → 组帧→CRC→BSP_UART_Write→等ACK
                                            ↑ CheckTimeout(TIM6 1ms)

RX → BSP_UART DMA循环缓冲 → RxTask → 三步解析→CRC→FCode表→回复ACK
                                            └── ACK匹配 → tx_ack_flag → tx_complete

TIM5 → BSP_LED_TickHandler → LED 翻转
TIM6 → AP_UART_CheckTimeout() + 置位标志(IR_feed_pending, test_print_pending)
```

## 测试模式 (IR_TEST_MODE)

测试模式开关定义在 `Code/ap/inc/ap_util.h`，取消 `#define IR_TEST_MODE` 注释进入：

- **ISR 仅置标志**：`AP_IR_FeedIsr()` + `test_print_pending`
- **主循环只跑**：命令行解析 + `AP_IR_Feed()`（仅推ADC填窗口，不进状态机）+ 条件打印 + 喂狗
- **不运行**：UV 状态机、IR 五判据算法、双重确认逻辑、串口通信协议(TxTask/RxTask)
- **CLI 控制**：`debug on/off` 启停 100Hz 数据流打印
- **场景分隔**：`mark [text]` 插入标记行
- **串口测试**：`uart loop <n>` 发环路测试 / `uart recv` 显示 COM 接收数据
- **LED 控制**：`led work <ms>` / `led blink <n> <ms>` / `led stop`
- **打印格式**：紧凑单行，PC 端 Python/Excel 可解析
  每 100Hz 输出 2 行：
  ```
  T<ms> UV <n> <w1> <w2>...
  T<ms> IRD DC=<d0>,<d1>,<d2> P=<p0>,<p1>,<p2> Z=<z0>,<z1>,<z2> R=<r38>,<r50>
  ```
  - DC: 原始 50 点窗口均值 (ADC 偏置)
  - P: 平均功率 ×1000 (均方值缩放)
  - Z: 过零率 (Hz)
  - R: 光谱比 ×1000

## 项目进度

| 模块 | 状态 | 说明 |
|------|------|------|
| ADC DMA 3通道采集 | ✅ 完成 | 连续转换，DMA1_CH1 CIRCULAR |
| USART1/2 DMA | ✅ 完成 | COM+DBG 双串口，环形缓冲 |
| TIM3 双通道脉宽捕获 | ✅ 完成 | CH1上升沿/CH2下降沿，16-bit回绕补偿 |
| UV 紫外检测 | ✅ 完成 | 三级状态机，灵敏度0~9线性插值 |
| IR 红外三波段检测 | ✅ 完成 | 多光谱融合 五判据串联 三级状态机，Q15定点IIR |
| EEPROM 参数存储 | ✅ 完成 | ADC/UV/IR 三扇区，魔数+CRC校验 |
| IWDG 看门狗 | ✅ 完成 | ISR置标志→主循环喂狗模式 |
| 串口协议 | ✅ 完成 | TX三态机+RX三步入解析，CRC16，ACK重传 |
| 命令行框架 | ✅ 完成 | help/adc/uv/ir/param/state/reset/debug/mark/led/uart |
| 硬件报警(ALM1/ALM2) | ✅ 完成 | BSP驱动，火警低电平输出 |
| IR 信号链调试打印 | ✅ 完成 | DC偏置+Power×1000+ZCR+光谱比，不进状态机 |
| 双重确认(IR&&UV) | ✅ 完成 | 逻辑与→FIRE_ALARM上报+ALM硬件输出 |
| 参数现场标定 | ⏳ 待定 | 需真实火焰数据标定阈值 |
