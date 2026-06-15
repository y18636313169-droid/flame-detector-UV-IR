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
│   │   │   ├── hal_uart.h         # UART DMA 环形缓冲驱动（COM=USART1, DBG=USART2）
│   │   │   ├── hal_tim.h          # TIM3 单通道脉宽捕获 + 环形缓冲输出
│   │   │   ├── hal_adc.h          # ADC DMA 连续转换 + 单通道读取
│   │   │   ├── hal_board.h        # BSP_BoardInit 聚合
│   │   │   └── ring_buffer.h      # 通用环形缓冲区（多实例/ISR安全）
│   │   └── src/                   # 对应实现
│   └── ap/                        # 应用层
│       ├── inc/
│       │   ├── ap_adc.h           # ADC 滑动滤波 + Debug打印
│       │   ├── ap_uart_protocol.h # 串口协议组包/解析/ACK
│       │   └── ap_uv.h            # 紫外火焰检测（窗口计数+状态机）
│       └── src/                   # 对应实现
├── MDK-ARM/                       # Keil 项目文件
├── .eide/                         # EIDE 项目文件
└── CLAUDE.md                      # 本文件
```

## 中断分发链

```
stm32l1xx_it.c
  ├── TIM3_IRQHandler → HAL_TIM_IRQHandler
  │     └── main.c (USER CODE 4):
  │           HAL_TIM_IC_CaptureCallback → BSP_TIM_IC_CaptureHandler   [hal_tim.c]
  │           HAL_TIM_PeriodElapsedCallback:
  │             ├── TIM5 → BSP_LED_TickHandler                         [hal_led.c]
  │             ├── TIM6 → AP_UART_CheckTimeout() + task_10ms()        [ap_uart_protocol.c]
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

### hal_tim.h — TIM3 脉宽捕获（单通道极性切换）
| 函数 | 说明 |
|------|------|
| `BSP_TIM_IC_Init(id)` | 绑定 htim3，初始化环形缓冲 |
| `BSP_TIM_IC_Start(id)` | 启动捕获（上升沿） |
| `BSP_TIM_IC_Stop(id)` | 停止捕获 |
| `BSP_TIM_IC_ReadPulse(id, &pulse)` | 从环形缓冲读取一条脉冲 |
| `BSP_TIM_IC_ReadAllPulse(id, pulses, max)` | 批量读取所有脉冲，返回数量 |
| `BSP_TIM_IC_CaptureHandler(htim)` | 捕获状态机（由 main.c 回调转发） |
| `BSP_TIM_IC_PeriodHandler(htim)` | 溢出计数（由 main.c 回调转发） |

### hal_adc.h — ADC DMA 连续转换
| 函数 | 说明 |
|------|------|
| `BSP_ADC_Init()` | 检查 hadc 句柄 |
| `BSP_ADC_StartDMA()` | 启动 DMA 连续转换（单通道→内部缓冲） |
| `BSP_ADC_StopDMA()` | 停止 DMA 转换 |
| `BSP_ADC_ReadValue(&raw)` | 读取最新原始值 |

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
| `BSP_BoardInit()` | 依次调 BSP_GPIO/ADC/TIM/UART Init，在 CubeMX MX_* 后调用 |

## AP 层 API 总览

### ap_adc.h — ADC 滑动滤波（单通道）
| 函数 | 说明 |
|------|------|
| `AP_ADC_Init()` | 调 BSP_ADC_StartDMA，清滑动窗口 |
| `AP_ADC_Update()` | 读最新值 → 推入滑动窗口 → 置打印标志 |
| `AP_ADC_GetFiltered()` | 获取滑动滤波结果 |
| `AP_ADC_GetLatest()` | 获取最新原始值 |
| `AP_ADC_IsPrintPending()` | 检查是否需要打印（由主循环消费） |
| `AP_ADC_PrintDebug()` | DBG 串口打印原始值+滤波值 |

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
| `AP_UV_Init(level, on_fire)` | 初始化灵敏度+注册报警回调 |
| `AP_UV_Feed()` | 从 BSP 读取脉冲加入历史队列 |
| `AP_UV_Process(now)` | 内部调 Feed → 窗口计数 → 状态机 |
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

## 串口协议帧格式

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
BSP_BoardInit();                    // BSP 各模块初始化
BSP_TIM_IC_Start(BSP_TIM_UV);      // 启动 TIM3 捕获
AP_ADC_Init();                      // 启动 ADC DMA
AP_UART_ProtocolInit();             // 清协议缓冲
AP_UV_Init(5, NULL);                // 紫外检测（中灵敏度）
```

## 主循环框架（while(1)）

```c
AP_UART_TxTask();      // 串口 TX 状态机
AP_UART_RxTask();      // 串口 RX 状态机
AP_UV_Feed();          // 读 UV 脉冲 → 历史队列
AP_UV_Process(now);    // 窗口计数 → 状态机决策
AP_ADC_IsPrintPending() → AP_ADC_PrintDebug();  // ADC 调试打印
```

## 关键数据流示意

```
ADC 硬件 → DMA → adc_dma_buf[0] (BSP)
                    ↓ BSP_ADC_ReadValue()
                 AP_ADC_Update() → 滑动窗口 → GetFiltered()
                     ↑ TIM6 10ms 定时调用

TIM3 CH1 → 捕获状态机 (BSP) → ring_buffer(20组)
                    ↓ BSP_TIM_IC_ReadAllPulse()
                 AP_UV_Feed() → 历史队列 → AP_UV_Process()
                     ├── 窗口计数 → 状态机 → on_fire 回调
                     └── DBG 打印

AP_UART_Send → TX事件缓冲 → TxTask → 组帧→CRC→BSP_UART_Write→等ACK
                                            ↑ CheckTimeout(TIM6 1ms)

RX → BSP_UART DMA循环缓冲 → RxTask → 三步解析→CRC→FCode表→回复ACK
                                            └── ACK匹配 → tx_ack_flag → tx_complete

TIM5 → BSP_LED_TickHandler → LED 翻转
```
