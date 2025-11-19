🧲 Sensor Magnético MC-38 con STM32 NUCLEO-H503RB
Lectura por Interrupciones EXTI + UART Debug

Este proyecto implementa un sistema de detección de apertura/cierre de puerta usando un sensor magnético MC-38, manejado mediante interrupciones de hardware (EXTI) en un microcontrolador STM32 NUCLEO-H503RB.

Se evita completamente el uso de polling del GPIO.
El sistema reacciona inmediatamente al cambio de estado del sensor gracias a las líneas de interrupción.
