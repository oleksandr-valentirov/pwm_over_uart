#include <stdio.h>
#include "stm32f1xx_ll_usart.h"

int _write(int file, char *ptr, int len) {
    (void)file;

    for (int i = 0; i < len; i++) {
        while (!(LL_USART_IsActiveFlag_TC(USART1) || LL_USART_IsActiveFlag_TXE(USART1))) {}
        LL_USART_TransmitData8(USART1, *(ptr++));
    }

    return len;
}
