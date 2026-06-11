/**
  ******************************************************************************
  * @file    ring_buffer.c
  * @brief   通用环形缓冲区实现
  *
  *          基于字节数组 + element_size 的泛型环形队列。
  *          所有读写操作均使用 __disable_irq / __set_PRIMASK 临界区保护，
  *          确保 ISR 和主循环并发安全。
  *
  *          == 容量约束 ==
  *          element_size × capacity ≤ 65535 字节（uint16_t 寻址）
  *          实际可用元素 = capacity - 1（空/满判断需要保留一个槽位）
  *
  *          == 空/满判断 ==
  *          head == tail → 空
  *          (head - tail) & (2*cap - 1) >= cap → 满
  *          使用 2*capacity 的虚拟模数，无需保留一个空槽位。
  *          实现上使用 uint16_t 自然溢出做差。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "ring_buffer.h"
#include <stdint.h>

/* ---------------------------------------------------------------------------*/
/*                        内部辅助宏                                           */
/* ---------------------------------------------------------------------------*/

/* 编译器兼容：__get_PRIMASK / __disable_irq / __set_PRIMASK
 *   ARMCC / ARMCLANG (Keil)  → 内建函数，无需额外头文件
 *   GCC (EIDE)                → cmsis_gcc.h 提供声明
 */
#if defined(__GNUC__) && !defined(__ARMCC_VERSION) && !defined(__ARM_CLANG_ARM_COMPILER)
#include "cmsis_gcc.h"
#endif

/** @brief  临界区进入 —— 保存 PRIMASK 并关中断 */
#define RB_CRIT_ENTER()  uint32_t __rb_primask__ = __get_PRIMASK(); \
                         __disable_irq()

/** @brief  临界区退出 —— 恢复 PRIMASK */
#define RB_CRIT_EXIT()   __set_PRIMASK(__rb_primask__)

/* ---------------------------------------------------------------------------*/
/*                        公有 API 实现                                        */
/* ---------------------------------------------------------------------------*/

void ring_buffer_init(ring_buffer_t *rb, void *pool,
                      uint16_t elem_sz, uint16_t capacity)
{
    if (rb == NULL || pool == NULL || elem_sz == 0 || capacity == 0) {
        return;
    }

    rb->pool          = (uint8_t *)pool;
    rb->element_size  = elem_sz;
    rb->capacity      = capacity;
    rb->head          = 0;
    rb->tail          = 0;
    rb->overflowed    = 0;
}

int ring_buffer_push(ring_buffer_t *rb, const void *element)
{
    if (rb == NULL || element == NULL) return -1;

    RB_CRIT_ENTER();

    uint16_t count = rb->head - rb->tail;

    if (count >= rb->capacity) {
        /* 缓冲区满，丢弃新数据，置溢出标志 */
        rb->overflowed = 1;
        RB_CRIT_EXIT();
        return -1;
    }

    uint16_t head = rb->head % rb->capacity;
    memcpy(rb->pool + head * rb->element_size, element, rb->element_size);
    rb->head++;

    RB_CRIT_EXIT();
    return 0;
}

void ring_buffer_push_overwrite(ring_buffer_t *rb, const void *element)
{
    if (rb == NULL || element == NULL) return;

    RB_CRIT_ENTER();

    uint16_t head = rb->head % rb->capacity;
    memcpy(rb->pool + head * rb->element_size, element, rb->element_size);
    rb->head++;

    /* 如果覆盖了最旧元素，推进 tail */
    if ((rb->head - rb->tail) > rb->capacity) {
        rb->tail++;
    }

    RB_CRIT_EXIT();
}

int ring_buffer_pop(ring_buffer_t *rb, void *element)
{
    if (rb == NULL || element == NULL) return -1;

    RB_CRIT_ENTER();

    if (rb->head == rb->tail) {
        RB_CRIT_EXIT();
        return -1;
    }

    uint16_t tail = rb->tail % rb->capacity;
    memcpy(element, rb->pool + tail * rb->element_size, rb->element_size);
    rb->tail++;

    RB_CRIT_EXIT();
    return 0;
}

int ring_buffer_peek(const ring_buffer_t *rb, void *element, uint16_t offset)
{
    if (rb == NULL || element == NULL) return -1;

    int ret = -1;

    RB_CRIT_ENTER();

    uint16_t count = rb->head - rb->tail;
    if (offset < count) {
        uint16_t idx = (rb->tail + offset) % rb->capacity;
        memcpy(element, rb->pool + idx * rb->element_size, rb->element_size);
        ret = 0;
    }

    RB_CRIT_EXIT();
    return ret;
}

uint16_t ring_buffer_count(const ring_buffer_t *rb)
{
    if (rb == NULL) return 0;

    uint16_t cnt;

    RB_CRIT_ENTER();
    cnt = (rb->head - rb->tail) % rb->capacity;
    RB_CRIT_EXIT();

    return cnt;
}

uint16_t ring_buffer_space(const ring_buffer_t *rb)
{
    if (rb == NULL) return 0;

    uint16_t cnt, space;

    RB_CRIT_ENTER();
    cnt = ring_buffer_count(rb);
    space = (cnt >= rb->capacity) ? 0 : (rb->capacity - cnt);
    RB_CRIT_EXIT();

    return space;
}

void ring_buffer_clear(ring_buffer_t *rb)
{
    if (rb == NULL) return;

    RB_CRIT_ENTER();
    rb->head = 0;
    rb->tail = 0;
    RB_CRIT_EXIT();
}

uint8_t ring_buffer_overflowed(ring_buffer_t *rb)
{
    if (rb == NULL) return 0;

    uint8_t flag;

    RB_CRIT_ENTER();
    flag = rb->overflowed;
    rb->overflowed = 0;     /* 读后清零 */
    RB_CRIT_EXIT();

    return flag;
}
/*
uint16_t ring_buffer_tail(const ring_buffer_t *rb)
{
    if (rb == NULL) return 0;
    return rb->tail % rb->capacity;
}

void *ring_buffer_front(const ring_buffer_t *rb)
{
    if (rb == NULL) return NULL;

    void *ptr = NULL;

    RB_CRIT_ENTER();
    if (rb->head != rb->tail) {
        uint16_t tail = rb->tail % rb->capacity;
        ptr = rb->pool + tail * rb->element_size;
    }
    RB_CRIT_EXIT();

    return ptr;
}

void ring_buffer_pop_commit(ring_buffer_t *rb)
{
    if (rb == NULL) return;

    RB_CRIT_ENTER();
    if (rb->head != rb->tail) {
        rb->tail++;
    }
    RB_CRIT_EXIT();
}*/
