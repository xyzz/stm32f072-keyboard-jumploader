#pragma once

#if defined(TARGET_GENERIC)

#elif defined(TARGET_XIAOMI_MK02)

/* Spacebar - B0 row, B4 col, ROW2COL diode direction */
#define BLKEY_OUTPUT_GPIO GPIOB
#define BLKEY_OUTPUT_PIN 0
#define BLKEY_INPUT_GPIO GPIOB
#define BLKEY_INPUT_PIN 4

#else
#error Not configured for this target.
#endif
