# 火源识别模块 — 代码架构总览

永远使用中文回答；
只修改必要的函数注释，每次修改不要把无关注释加来删去；
修改算法、状态机、参数布局或并发逻辑时，必须在代码旁备注修改原因和关键约束，禁止无说明地修改逻辑；

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
│   │   │   ├── ap_util.h          # 工具函数(lerp) + 算法调试日志宏
│   │   │   ├── ap_eeprom.h        # 参数分组(ADC/UV/IR) + 默认值 + 掉电保存
│   │   │   ├── ap_uart_protocol.h # 树莓派配置协议、会话、去重与应答
│   │   │   ├── ap_uv.h            # 紫外火焰检测（窗口计数+状态机）
│   │   │   └── ap_ir.h            # 红外三波段多光谱融合检测（五判据+状态机）
│   │   └── src/                   # 对应实现
│   └── modules/
│       ├── cmdline/               # 命令行框架 (cmd_parser_task)
│       └── crc/                   # CRC16-CCITT + Modbus 双算法
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
  │             ├── TIM6 → task_10ms()                                 [main.c]
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
| `BSP_UART_Init(id)` | 初始化串口（COM=USART2, DBG=USART1），启动 RX DMA 循环 |
| `BSP_UART_Write(id, data, len)` | 写入 TX 环形缓冲，DMA 自动搬运（非阻塞） |
| `BSP_UART_WriteBlock(id, data, len, ms)` | 阻塞发送，带超时 |
| `BSP_UART_Read(id, buf, len)` | 从 RX DMA 循环缓冲读取 |
| `BSP_UART_GetRxDataLen(id)` | 查询 RX 缓冲可读数据长度 |
| `BSP_UART_RxOverflowed(id)` | 查询并清除RX DMA覆盖或接收错误标志 |
| `BSP_UART_Peek(id, out, offset)` | 查看 RX 缓冲指定偏移 1 字节（不移动读指针） |
| `BSP_UART_PeekPacket(id, out, off, len)` | 查看 RX 缓冲一段连续数据 |
| `BSP_UART_Consume(id, len)` | 推进 RX 读指针 |
| `BSP_UART_IsTxComplete(id)` | 查询 TX 是否发送完成 |
| `BSP_UART_TxFaulted(id)` | 查询并清除TX DMA故障锁存标志 |
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
| `AP_ALGO_DEBUG_ENABLE` | 应用模式算法事件及500ms特征快照编译开关 |

### ap_adc.h — ADC 原始值读取
| 函数 | 说明 |
|------|------|
| `AP_ADC_Init()` | 启动 ADC DMA 转换 |
| `AP_ADC_GetLatest(ch)` | 读取指定通道最新原始值 |

> 注：IR 算法依次读取三通道最新 DMA 值，不使用 AP_ADC 滤波。

### ap_eeprom.h — EEPROM参数存储管理
| 函数 | 说明 |
|------|------|
| `AP_EEPROM_Init()` | 加载所有参数组（ADC/UV/IR），魔数+CRC校验，失败写默认值 |
| `AP_EEPROM_ADC_Get/Save/Reset` | ADC 阈值参数存取 |
| `AP_EEPROM_UV_Get/Save/Reset` | UV 检测参数存取（v3：灵敏度+3组min/max+脉宽/打印窗口） |
| `AP_EEPROM_IR_Get/Save/Reset` | IR 多光谱融合参数存取（v7：功率min/max+固定光谱比/FFT频带+确认时间min/max；保留旧ZCR字段以兼容v6/v7布局） |

### ap_uart_protocol.h — 串口通信协议
| 函数 | 说明 |
|------|------|
| `AP_UART_ProtocolInit()` | 清解析器、响应发送器、会话和去重缓存 |
| `AP_UART_RxTask()` | 有界读取 USART2 DMA 数据，每次最多处理一帧请求 |
| `AP_UART_TxTask()` | 非阻塞推进单帧或双帧响应，兼容 TX 队列部分接收 |
| `AP_UART_CheckTimeout()` | 主循环处理100ms半帧、500ms发送和10s会话超时 |
| `AP_UART_IsIdle()` | 查询解析、响应队列和 USART2 物理发送是否均空闲 |
| `AP_UART_SetDiagnosticMode()` | 测试模式下暂停/恢复USART2协议；切换时清会话和收发残留 |
| `AP_UART_GetDiagnosticMode()` | 查询USART2是否处于原始诊断模式 |

协议采用严格停等：树莓派握手后逐项请求，从机完成校验和 EEPROM 写后读回才应答。最近一次请求及完整响应被缓存；相同请求只重放响应，不重复写 EEPROM，相同序号但内容不同或旧序号均拒绝执行。

### ap_uv.h — 紫外火焰检测
| 函数 | 说明 |
|------|------|
| `AP_UV_Init(on_fire)` | 从 EEPROM 加载参数初始化紫外检测 |
| `AP_UV_Feed()` | 从 BSP 读取脉冲加入历史队列 |
| `AP_UV_Process(now)` | 窗口计数 → 状态机决策 |
| `AP_UV_Task()` | Feed + Process 打包（主循环调用） |
| `AP_UV_SetLevel(level)` | 运行时调整灵敏度，0最灵敏、9最迟钝 |
| `AP_UV_Process_Reset()` | 重置状态机到 IDLE |
| `AP_UV_GetState()` | 返回当前状态 |

#### 紫外状态机
```
IDLE ──(窗口计数≥threshold)──→ WARNING ──(有效累计≥confirm_ms)──→ FIRE
  ↑                             │                                │
  └──(低于80%阈值持续2秒)───────┘                                │
                                                   (保持30分钟超时)──┘
```
等级 0~9 按 min→max 线性插值：window_ms=1000~3500, threshold=5~32, confirm_ms=0~3500；**0最灵敏，9最迟钝**。WARNING进入后使用当前阈值80%的向上取整值作为掉线下限，低于下限时有效确认时间回退，连续2秒不足才退回IDLE；FIRE固定保持30分钟。

### ap_ir.h — 红外三波段多光谱融合火焰检测
| 函数 | 说明 |
|------|------|
| `AP_IR_Init()` | 从EEPROM加载参数，初始化历史窗口和状态机 |
| `AP_IR_FeedIsr()` | ISR(TIM6)调用，仅置位 volatile 标志 |
| `AP_IR_Feed()` | 从 ADC 读最新值，经连续IIR后推入256点历史窗口（不进状态机） |
| `AP_IR_Task()` | 主循环调用，检查标志→Feed→Process(全流程检测) |
| `AP_IR_SetLevel(level)` | 运行时调整等级0~9，仅插值确认时长 |
| `AP_IR_GetState()` | 返回当前状态 IR_STATE_IDLE/WARNING/FIRE |
| `AP_IR_Reset()` | 重置状态机到 IDLE，清历史窗口 |
| `AP_IR_GetFeatures(power, dominant_freq, &r45_38, &r45_50)` | 获取功率、FFT主频和光谱比（调试用） |
| `AP_IR_SetConfig(...)` | 设置进场功率范围、固定光谱比/频率及确认时间范围 |
| `AP_IR_GetParams(...)` | 获取当前运行参数 |
| `AP_IR_DebugProcess(now)` | 测试模式：信号链处理+打印，不进状态机 |

#### 算法流程
```
Feed(ADC值→每通道连续20Hz IIR→256点历史缓存) → FFT窗口满后:
  最近50点 → 去直流(减均值) → 平均功率(P3.8/P4.5/P5.0，均方值)
  最近256点 → 独立去直流 + Hann窗 + Q30定点FFT（每100ms更新）
  → 光谱比(R4.5/3.8, R4.5/5.0 ×1000；参考功率仅在严格为0时比例置0)
  → 五判据串联:
      ① P4.5 ≥ 当前等级进场阈值12000~22000（退出阈值为当前值的40%）
      ② R4.5/3.8 > 光谱比阈值(高温热源鉴别)
      ③ R4.5/5.0 > 光谱比阈值(背景辐射鉴别)
      ④ 在配置火焰频带内搜索4.5um FFT峰值主频
      ⑤ 频带能量占比≥50%，1.5Hz以上核心能量占比≥15%，带内峰值≥全频最大峰值的50%
      ⑥ 3.8/5.0um达到有效功率后，才要求其FFT主频与4.5um相差不超过2Hz
  → 五判据全部通过: IDLE→WARNING
  → P45安静布防后第一次跨过当前等级ON时，并行启动最长3秒主通道包络分类
      0~1.5秒: 持续记录0.5秒滚动均方值的启动最高峰early_peak
      1.5~3.0秒: 最多采1.5秒稳定数据，late_mean只统计P45≥当前OFF的有效样本
      WARNING证据先满足: FIRE迁移前立即使用当前已采稳定样本完成分类，不等待3秒终点
      PEAK≥100000、有有效后段样本且late/peak<50%: 分类为LIGHTER并阻止FIRE
      其他情况或高灵敏度下后段数据不足: 分类为SUSTAINED并放行FIRE
      LIGHTER后每1秒检查恢复；有效占比≥60%后，相对路径(恢复至PEAK的30%)连续2窗或固定路径(均值≥50000)连续5窗通过后升级
      SUSTAINED禁止反向降级；两者P45<当前OFF连续2秒后回BYPASS并重新布防
      BYPASS期间若当周期P45≥ON且正常IR光谱/频率判据通过: 按热启动接纳为SUSTAINED
      WARNING掉线超时且PROFILE仍为OBS: 取消本次观察并回BYPASS
      PROFILE不阻塞或清除WARNING证据积分，只在最终FIRE迁移点读取结论
```

#### 状态机
```
IDLE ──(功率≥当前ON且五判据通过)──→ WARNING ──(积分满足且PROFILE非LIGHTER)──→ FIRE
  ↑                                  │                                           │
  └──(判据失败或功率<当前OFF持续2秒)─┘                       (保持超时)──────────┘
```

#### 参数管理
- **随等级 0~9 插值**: 4.5um进场功率阈值(power=12000~22000)、确认时长(cfm)
- **固定值(不插值)**: 光谱比阈值(r38/r50)、频率下限(1.0Hz)、频率上限(20Hz)
- **FFT固定特征**: 256点窗口在100Hz采样下覆盖2.56秒，频率分辨率约0.390625Hz；FB定义为约1~20Hz能量/非直流全频能量，FC定义为约1.5~20Hz能量/约1~20Hz能量，入场频带占比、核心占比、带内峰值支持度门槛分别为50%/15%/50%，均不随灵敏度变化。另计算三通道频带FFT幅值和X及频域通道比XR，当前只用于样本标定
- **参考通道策略**: 4.5um始终计算FFT并强制满足火焰频域特征；3.8/5.0um功率分别达到900/400后才计算FFT，并要求与主通道主频差不超过2Hz。低于有效功率的参考通道清空旧频谱且不参与否决
- **功率抗抖**: ON按等级在12000~22000插值，OFF固定为当前ON的40%，两者仅约束4.5μm主通道；WARNING内主通道功率达到ON或处于OFF~ON迟滞区时有效积分每周期+10ms，功率低于OFF时每周期-10ms，连续2秒后退出WARNING
- **点火包络分类**: BYPASS下P45低于当前OFF满1秒后进入ARMED；第一次跨过当前ON立即记录启动事件，使用0~1.5秒峰值及其后最多1.5秒稳定数据分类。稳定均值只统计P45≥OFF的有效样本，低于OFF的低谷不参与均值。PROFILE和WARNING积分每10ms并行运行；若WARNING在稳定窗中途先达到confirm_ms，必须在FIRE迁移前立即使用当前已采稳定样本收口，不等待3秒最大终点；若确认早于稳定窗且没有稳定样本，则按SUSTAINED放行。仅PEAK≥100000、有有效后段样本且late/peak<50%时判为LIGHTER并阻止FIRE，其余情况按SUSTAINED放行。LIGHTER后续按互不重叠的1秒窗口检查恢复，两条路径分别累计且均要求P45≥OFF有效占比≥60%：相对路径要求有效均值恢复至启动PEAK的30%以上并连续2窗，固定路径要求有效均值≥50000并连续5窗；任一路径完成后单向升级为SUSTAINED，任一窗口失败只清零对应路径计数。SUSTAINED禁止反向降级。BYPASS热启动仍要求当周期P45≥ON且正常IR光谱/频率判据通过，不读取掉线尾段遗留的旧WARNING。P45连续低于OFF满2秒后回BYPASS，再安静1秒布防
- **频域抗抖**: IDLE使用严格频带及FB/FC/FP=500/150/500；WARNING保持配置的主频下限，仅将上限放宽一个FFT频点，并使用350/80/350维持门槛。短时频域失败按每10ms回退积分，连续失败2秒才退出WARNING；频谱最多滞后100ms，功率与状态机仍保持10ms周期
- **EEPROM版本**: UV为v3；IR为v7，为避免现场参数失效，旧ZCR死区三个字仍保留为布局兼容占位，FFT算法不读取

### 串口协议帧格式

```
AA 55 | Version(0x10) | PayloadLen | SequenceLE16 | MessageID | Data | CRC16LE
```

- 单帧线长上限为128 B；固定开销9 B，因此业务Data最大119 B、PayloadLen最大122 B
- CRC 使用 CCITT-FALSE，范围为 Sequence 至 Data，在线帧中按小端发送
- `0x80` 建联；`0x81`空Data读取配置并以同一ID返回84 B纯配置快照；各配置项使用独立SET ID
- `0xF0` 固定携带原请求序号、原 ID、成功/失败及保留错误码，共 6 B；收到 F0 不再响应
- GET 成功依次返回 `0x81` 配置数据帧和 `0xF0` 完成帧，重发请求时两帧整体重放

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
AP_UART_ProtocolInit();             // 清解析器、会话和请求去重状态
```

## 主循环框架（while(1)）

```c
recover_input_task();
AP_UART_RxTask();
AP_UART_TxTask();
AP_UART_CheckTimeout(); // 收包优先，避免10s边界误清已到达请求
AP_ADC_FaultMonitorTask(now);
cmd_parser_task();      // USART1 命令行

if (APP_GetTestMode()) {
  // 运行时测试模式: 仅按标志采集/打印，不推进报警状态机
  AP_IR_DebugProcess(now);
  AP_UV_PrintData(now);
} else {
  AP_UV_Task();                   // Feed+Process → 状态机
  if (!APP_GetShowMode()) AP_IR_Task();

  // 正常模式为 UV&&IR；演示模式只依据 UV，报警输出低电平有效
  update_alarm_output();
}

BSP_IWDG_CheckAndRefresh();   // 喂狗
```

## 关键数据流示意

```
ADC 硬件 → DMA → adc_dma_buf[3] (BSP DMA1_CH1 CIRCULAR)
                    ↓
            AP_IR_Feed() / AP_IR_Feed()
            → 每通道连续 IIR → history_push() 到 256 点历史窗口
            → AP_IR_Process() / AP_IR_DebugProcess()
            → 最近50点提取DC/Power，最近256点提取FFT特征
            → 特征提取(Power/FFT主频/频带能量/光谱比)
            → 五判据串联 → 状态机(IDLE→WARNING→FIRE)
            → 与 UV 状态逻辑与 → 最终火警

TIM3 CH1/CH2 → 双通道捕获 (BSP)
                    ↓ ISR: 硬件捕获→ring_buffer(128组)
                    主循环: AP_UV_Task() → Feed → AP历史队列(64组)
                    → 窗口计数 → 状态机(IDLE→WARNING→FIRE)

树莓派请求 → USART2 RX DMA循环缓冲 → 有界流式解析 → CRC/会话/序号检查
           → 参数整组校验 → EEPROM写后读回 → 运行时生效
           → SET/HANDSHAKE回复F0；GET回复0x81配置数据帧+F0
           → TxTask按发送偏移推进USART2 DMA，缓存响应供重复请求重放

TIM5 → BSP_LED_TickHandler → LED 翻转
TIM6 → 仅置位标志(IR_feed_pending, test_print_pending)
```

## 运行时测试模式

测试模式由 USART1 命令行切换并保存到 SYSTEM EEPROM，不再依赖重新编译和烧录：

- **ISR 仅置标志**：`AP_IR_FeedIsr()` + `test_print_pending`
- **主循环保留**：恢复输入、树莓派配置协议、ADC故障监测、命令行、定时采集/打印和喂狗
- **不运行**：UV 状态机、IR 火焰识别状态机和最终组合报警判断
- **CLI 控制**：`debug on/off` 启停 100Hz 数据流打印
- **场景分隔**：`mark [text]` 插入标记行
- **串口测试**：先用`uart raw on`清会话并暂停协议，再用`uart loop <n>`/`uart recv`测试COM；`uart raw off`后树莓派需重新握手
- **LED 控制**：`led work <ms>` / `led blink <n> <ms>` / `led stop`
- **打印格式**：紧凑单行，PC 端 Python/Excel 可解析
  每 100Hz 输出 2 行：
  ```
  T<ms> UV <n> <w1> <w2>...
  T<ms> IRD DC=<...> P=<...> F10=<...> FB1000=<...> FC1000=<...> FP1000=<...> X=<...> XR1000=<...> REFV=<...> R1000=<...>
  ```
  - DC: 原始最近50点窗口均值 (ADC 偏置)
  - P: 最近50点平均功率（去直流信号均方值）
  - F10: 最近256点FFT主频 ×10（整数12约表示1.2Hz，16约表示1.6Hz）
  - FB1000/FC1000: 配置频带能量占比/1.5Hz以上核心能量占比，均为×1000
  - FP1000: 带内主峰能量/全频最大单点能量 ×1000
  - X/XR1000: 三通道频带FFT幅值和及4.5/参考通道频域比值；无效参考通道对应比值为0
  - R1000: 光谱比 ×1000（整数1500表示1.500；参考功率严格为0时为0）

## 应用算法调试 (AP_ALGO_DEBUG_ENABLE)

取消 `Code/ap/inc/ap_util.h` 中该宏的注释后，IR算法输出初始化参数、256点FFT窗口就绪、每500ms特征快照、限频后的判据失败、功率掉线/恢复、点火包络分类、状态迁移及FIRE超时；UV算法输出每500ms窗口计数/覆盖统计和状态迁移；最终IR&&UV报警沿输出独立的`[ALARM]`日志。宏关闭时相关计时变量和日志函数不参与编译。

特征快照输出 `F10`、`FB1000`、`FC1000` 以及 `PROF=BYPASS|ARMED|OBS|LIGHTER|SUSTAINED`。首次跨过当前等级ON输出`PROFILE onset P=<...> ON=<...>`；其余点火包络日志格式保持不变。

## 项目进度

| 模块 | 状态 | 说明 |
|------|------|------|
| ADC DMA 3通道采集 | ✅ 完成 | 连续转换，DMA1_CH1 CIRCULAR |
| USART1/2 DMA | ✅ 完成 | COM+DBG 双串口，环形缓冲 |
| TIM3 双通道脉宽捕获 | ✅ 完成 | CH1上升沿/CH2下降沿，16-bit回绕补偿 |
| UV 紫外检测 | ✅ 完成 | 三级状态机，灵敏度0~9线性插值 |
| IR 红外三波段检测 | ✅ 完成 | 多光谱融合五判据、并行点火包络分类、三级状态机、Q15定点IIR |
| EEPROM 参数存储 | ✅ 完成 | ADC/UV/IR 三扇区，魔数+CRC校验 |
| IWDG 看门狗 | ✅ 完成 | ISR置标志→主循环喂狗模式 |
| 树莓派配置协议 | ✅ 完成 | USART2从机、AA55流式解析、10s会话、序号去重、配置读写及F0应答 |
| 命令行框架 | ✅ 完成 | help/adc/uv/ir/param/state/reset/debug/mark/led/uart |
| 硬件报警(ALM1/ALM2) | ✅ 完成 | BSP驱动，火警低电平输出 |
| IR 信号链调试打印 | ✅ 完成 | 测试模式打印DC/Power/FFT/光谱比；应用模式打印限频特征快照和状态机事件 |
| 双重确认(IR&&UV) | ✅ 完成 | 逻辑与→FIRE_ALARM上报+ALM硬件输出 |
| 参数现场标定 | ⏳ 待定 | 需真实火焰数据标定阈值 |
