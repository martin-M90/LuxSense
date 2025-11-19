🧲 Sensor Magnético MC-38 con STM32 NUCLEO-H503RB
Lectura por Interrupciones EXTI + UART Debug

Este proyecto implementa un sistema de detección de apertura/cierre de puerta usando un sensor magnético MC-38, manejado mediante interrupciones de hardware (EXTI) en un microcontrolador STM32 NUCLEO-H503RB.

Se evita completamente el uso de polling del GPIO.
El sistema reacciona inmediatamente al cambio de estado del sensor gracias a las líneas de interrupción.

🛠️ Hardware
🧲 Sensor MC-38 (Reed Switch)

Cuando la puerta está cerrada → contacto cerrado (0 lógico).

Cuando la puerta está abierta → contacto abierto (1 lógico).

🔌 Conexiones
MC-38 cable 1 → GND  
MC-38 cable 2 → PC10 (GPIO_EXTI10)


El pin PC10 se configura con Pull-Up interno, por lo que el sensor funciona así:

Puerta cerrada → PC10 = LOW (RESET)

Puerta abierta → PC10 = HIGH (SET)

⚙️ Configuración Importante (STM32CubeIDE / .ioc)
➤ GPIO
Pin	Función	Configuración
PC10	DOOR_SW	GPIO_MODE_IT_RISING_FALLING + PULLUP
PA5	LED	Output Push-Pull
PA3	UART RX	Async
PA4	UART TX	Async
PC13	BTN	GPIO_MODE_IT_RISING
➤ NVIC (Interrupciones activadas)
IRQ Line	Función
EXTI10	Sensor MC-38
EXTI13	Botón de usuario
🧩 Arquitectura del Software

Este proyecto sigue un diseño correcto de sistemas embebidos:

1️⃣ Interrupción EXTI — ISR mínima

La rutina EXTI NUNCA hace UART ni lógica pesada.
Solo marca una bandera:

door_event = 1;


Esto evita:

Bloqueos en interrupciones

Problemas con HAL_UART_Transmit dentro del ISR

Retardos innecesarios

2️⃣ Bucle principal — Lógica de aplicación

En main() se revisa la bandera:

if (door_event)
{
    door_event = 0;

    door_state = HAL_GPIO_ReadPin(DOOR_SW_GPIO_Port, DOOR_SW_Pin);

    if (door_state != last_state)
    {
        last_state = door_state;

        if (door_state == GPIO_PIN_RESET)
        {
            UART_SendString("Puerta CERRADA\r\n");
            HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_SET);
        }
        else
        {
            UART_SendString("Puerta ABIERTA\r\n");
            HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_RESET);
        }
    }
}


Así logramos:

Respuesta inmediata (la EXTI despierta el sistema)

Sin polling directo del pin

Sin cargar al CPU con lecturas constantes

UART estable y sin bloqueos

🖥️ UART Debug

Se usa USART3 a 115200 bps, modo asíncrono:

Mensajes de inicio:

Demo STM32H503 + Sensor MC-38
USART3 OK a 115200 bps
Inicio: Puerta CERRADA


Mensajes dinámicos:

Puerta ABIERTA
Puerta CERRADA

🟢 LED Indicador (PA5)

Encendido → Puerta cerrada

Apagado → Puerta abierta

🧪 Ejemplo de salida real
Demo STM32H503 + Sensor MC-38
USART3 OK a 115200 bps
Inicio: Puerta CERRADA
Puerta ABIERTA
Puerta CERRADA
Puerta ABIERTA

📘 Explicación de la Lógica

El MC-38 cambia de estado → PC10 detecta transición.

EXTI10 se dispara → NVIC ejecuta EXTI10_IRQHandler.

La ISR solo pone door_event = 1.

El while(1) detecta la bandera:

Lee el pin

Actualiza LED

Manda mensaje por UART

Este patrón es el estándar en sistemas embebidos profesionales.

🚀 Cómo Compilar y Cargar

Abrir el proyecto en STM32CubeIDE.

Seleccionar target NUCLEO-H503RB.

Build Project (Ctrl+B).

Run → Debug o Run As → STM32 Application.

Conectar monitor serie a 115200.
