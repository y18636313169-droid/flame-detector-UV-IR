/**
  ******************************************************************************
  * @file    ring_buffer.h
  * @brief   通用环形缓冲区（循环队列）
  *
  *          == 特性 ==
  *          - 通用：操作裸字节，通过 element_size 支持任意数据类型
  *          - 多实例：调用方自行声明 ring_buffer_t 和存储池，互不干扰
  *          - ISR 安全：push / pop 均使用临界区保护（__disable_irq）
  *          - 溢出检测：push 在满时丢弃新数据并置 overflowed 标志
  *          - 覆盖模式：push_overwrite 在满时覆盖最旧数据
  *
  *          == 典型用法 ==
  *
  *          // 声明自己的实例
  *          static ring_buffer_t  my_rb;
  *          static my_data_t      my_pool[16];
  *
  *          void init(void) {
  *              ring_buffer_init(&my_rb, my_pool, sizeof(my_data_t), 16);
  *          }
  *
  *          // ISR（生产者）
  *          void ISR(void) {
  *              my_data_t d = { ... };
  *              ring_buffer_push(&my_rb, &d);
  *          }
  *
  *          // 主循环（消费者）
  *          void main_loop(void) {
  *              my_data_t d;
  *              while (ring_buffer_pop(&my_rb, &d) == 0) {
  *                  process(&d);
  *              }
  *          }
  ******************************************************************************
  */
#ifndef __RING_BUFFER_H__
#define __RING_BUFFER_H__

#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                              类型定义                                       */
/* ========================================================================== */

/**
  * @brief  环形缓冲区控制块
  *
  *         head — 写入偏移（生产者移动，单位：元素个数）
  *         tail — 读取偏移（消费者移动，单位：元素个数）
  *         有效元素范围：[tail, head)，环形取模 capacity
  *         空：head == tail
  *         满：(head - tail) == capacity
  *
  *         所有成员可被 ISR 和主循环并发访问，读写操作内部自带临界区保护。
  */
typedef struct {
    uint8_t         *pool;              /* 存储池指针（用户提供）          */
    uint16_t        element_size;       /* 单个元素字节数                  */
    uint16_t        capacity;           /* 最大元素个数                    */
    volatile uint16_t head;             /* 写入偏移（ISR/生产者修改）      */
    volatile uint16_t tail;             /* 读取偏移（主循环/消费者修改）   */
    volatile uint8_t  overflowed;       /* 溢出标志（满时置位，读后清零）  */
} ring_buffer_t;

/* ========================================================================== */
/*                              API 函数                                       */
/* ========================================================================== */

/**
  * @brief  初始化环形缓冲区
  * @param  rb:     控制块指针
  * @param  pool:   存储池（由调用方提供 static 数组）
  * @param  elem_sz: 单个元素字节数（sizeof(your_type)）
  * @param  capacity: 元素个数（pool 数组长度）
  */
void ring_buffer_init(ring_buffer_t *rb, void *pool,
                      uint16_t elem_sz, uint16_t capacity);

/**
  * @brief  压入一个元素（满则丢弃新数据，置 overflowed 标志）
  * @param  rb:      控制块指针
  * @param  element: 待压入的元素数据指针
  * @retval 0: 成功
  * @retval -1: 缓冲区满，元素未被写入
  */
int ring_buffer_push(ring_buffer_t *rb, const void *element);

/**
  * @brief  压入一个元素（满则覆盖最旧元素）
  * @param  rb:      控制块指针
  * @param  element: 待压入的元素数据指针
  * @note   覆盖模式下 overflowed 标志不会被置位。
  */
void ring_buffer_push_overwrite(ring_buffer_t *rb, const void *element);

/**
  * @brief  弹出一个元素（FIFO 顺序）
  * @param  rb:      控制块指针
  * @param  element: 接收数据的内存指针
  * @retval 0: 成功
  * @retval -1: 缓冲区空
  */
int ring_buffer_pop(ring_buffer_t *rb, void *element);

/**
  * @brief  查看（peek）从读指针偏移 offset 的元素，不移动读指针
  * @param  rb:     控制块指针
  * @param  element: 接收数据的内存指针
  * @param  offset: 相对于当前读指针的偏移（0 = 最旧元素）
  * @retval 0: 成功
  * @retval -1: 偏移超出有效数据范围
  */
int ring_buffer_peek(const ring_buffer_t *rb, void *element, uint16_t offset);

/**
  * @brief  获取当前有效元素个数
  */
uint16_t ring_buffer_count(const ring_buffer_t *rb);

/**
  * @brief  获取剩余可用空间（元素个数）
  */
uint16_t ring_buffer_space(const ring_buffer_t *rb);

/**
  * @brief  清空缓冲区（重置 head = tail = 0，不清零存储池）
  */
void ring_buffer_clear(ring_buffer_t *rb);

/**
  * @brief  检查并清除溢出标志
  * @retval 0: 未发生溢出
  * @retval 1: 自上次调用后发生溢出
  */
uint8_t ring_buffer_overflowed(ring_buffer_t *rb);

/**
  * @brief  直接读取尾指针（用于零拷贝场景，与 ring_buffer_pop_direct 配合）
  */
uint16_t ring_buffer_tail(const ring_buffer_t *rb);

/**
  * @brief  获取读指针位置的元素指针（零拷贝读，不移动读指针）
  * @param  rb: 控制块指针
  * @return 指向当前最旧元素的指针，空时返回 NULL
  * @note   此接口不涉及拷贝，适用于 ISR 中零拷贝消费。
  *         调用后必须配合 ring_buffer_pop_commit 推进读指针。
  */
void *ring_buffer_front(const ring_buffer_t *rb);

/**
  * @brief  推进读指针（配合 ring_buffer_front 使用）
  */
void ring_buffer_pop_commit(ring_buffer_t *rb);

#ifdef __cplusplus
}
#endif

#endif /* __RING_BUFFER_H__ */
