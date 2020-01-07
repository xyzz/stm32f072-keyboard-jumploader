#include <inttypes.h>

#include "proxy.h"
#include "stm32f0xx.h"
#include "config.h"

#define SECOND_STAGE_VECTOR (void*)0x08002000
#define BOOTROM_VECTOR (void*)0x1FFFC800

/* Value to place in RTC backup register BKP4R to enter the bootloader */
#define RTC_BOOTLOADER_FLAG 0x424C

/* Jumps to the next stage given its vector */
__attribute__((noreturn)) static void jump_vector(uint32_t *vector) {
    uint32_t sp = vector[0];
    uint32_t pc = vector[1];

    __asm__ volatile (
        "mov sp, %0\n"
        "bx %1\n" :: "r" (sp), "r" (pc)
    );

    __builtin_trap();
}

/* Check if the firmware is valid, same simple SP check as in stm32duino-bootloader */
static int firmware_valid(void) {
    uint32_t *vector = SECOND_STAGE_VECTOR;
    return (vector[0] & 0x2FFE0000) == 0x20000000;
}

/* Check if QMK asked us to reboot into bootloader mode */
static int bootloader_requested(void) {
    uint32_t *magic_address = (void*)0x20000FFC;

    /* "bldr" */
    if (*magic_address == 0x626c6472) {
        *magic_address = 0;
        return 1;
    }

    return 0;
}

/* Check if a keyboard key is held (specified in config.h) */
static int key_held(void) {
#ifdef BLKEY_OUTPUT_GPIO
    int res;
    uint32_t clock;

    clock = RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN;

    /* enable GPIO clocks */
    RCC->AHBENR |= clock;

    /* configure the input - pulldown input */
    BLKEY_INPUT_GPIO->MODER &= ~(0x3 << (BLKEY_INPUT_PIN * 2));
    BLKEY_INPUT_GPIO->PUPDR &= ~(0x3 << (BLKEY_INPUT_PIN * 2));
    BLKEY_INPUT_GPIO->PUPDR |= (0x2 << (BLKEY_INPUT_PIN * 2));

    /* configure the output - push-pull output */
    BLKEY_OUTPUT_GPIO->MODER &= ~(0x3 << (BLKEY_OUTPUT_PIN * 2));
    BLKEY_OUTPUT_GPIO->MODER |= (0x1 << (BLKEY_OUTPUT_PIN * 2));
    BLKEY_OUTPUT_GPIO->OTYPER &= ~(1 << BLKEY_OUTPUT_PIN);

    /* send 1 on output */
    BLKEY_OUTPUT_GPIO->ODR |= (1 << BLKEY_OUTPUT_PIN);

    /* wait a bit */
    for (volatile uint32_t i = 0; i < 0x1000; ++i) {}

    /* read input */
    res = !!(BLKEY_INPUT_GPIO->IDR & (1 << BLKEY_INPUT_PIN));

    /* disable GPIO clocks */
    RCC->AHBENR &= ~clock;

    return res;
#else
    return 0;
#endif
}

void _start(void) {
    if (!firmware_valid() || bootloader_requested() || key_held()) {
        /* Jump to the embedded bootloader */
        jump_vector(BOOTROM_VECTOR);
    } else {
        /* Jump to the firmware */
        jump_vector(SECOND_STAGE_VECTOR);
    }
}

static void* vectors[0xC0/4] __attribute__((used, section (".vectors"))) = {
    [0] = (void*)0x20002000,   /* stack pointer */
    [1] = _start,              /* reset */
    [2] = Proxy_0x08,          /* NMI */
    [3] = Proxy_0x0C,          /* HardFault */

    [11] = Proxy_0x2C,         /* SVCall */

    [14] = Proxy_0x38,         /* PendSV */
    [15] = Proxy_0x3C,         /* SysTick */

    [16] = Proxy_0x40,         /* IRQ0 */
    [17] = Proxy_0x44,         /* IRQ1 */
    [18] = Proxy_0x48,         /* IRQ2 */
    [19] = Proxy_0x4C,         /* IRQ3 */
    [20] = Proxy_0x50,         /* IRQ4 */
    [21] = Proxy_0x54,         /* IRQ5 */
    [22] = Proxy_0x58,         /* IRQ6 */
    [23] = Proxy_0x5C,         /* IRQ7 */
    [24] = Proxy_0x60,         /* IRQ8 */
    [25] = Proxy_0x64,         /* IRQ9 */
    [26] = Proxy_0x68,         /* IRQ10 */
    [27] = Proxy_0x6C,         /* IRQ11 */
    [28] = Proxy_0x70,         /* IRQ12 */
    [29] = Proxy_0x74,         /* IRQ13 */
    [30] = Proxy_0x78,         /* IRQ14 */
    [31] = Proxy_0x7C,         /* IRQ15 */
    [32] = Proxy_0x80,         /* IRQ16 */
    [33] = Proxy_0x84,         /* IRQ17 */
    [34] = Proxy_0x88,         /* IRQ18 */
    [35] = Proxy_0x8C,         /* IRQ19 */
    [36] = Proxy_0x90,         /* IRQ20 */
    [37] = Proxy_0x94,         /* IRQ21 */
    [38] = Proxy_0x98,         /* IRQ22 */
    [39] = Proxy_0x9C,         /* IRQ23 */
    [40] = Proxy_0xA0,         /* IRQ24 */
    [41] = Proxy_0xA4,         /* IRQ25 */
    [42] = Proxy_0xA8,         /* IRQ26 */
    [43] = Proxy_0xAC,         /* IRQ27 */
    [44] = Proxy_0xB0,         /* IRQ28 */
    [45] = Proxy_0xB4,         /* IRQ29 */
    [46] = Proxy_0xB8,         /* IRQ30 */
    [47] = Proxy_0xBC,         /* IRQ31 */
};
