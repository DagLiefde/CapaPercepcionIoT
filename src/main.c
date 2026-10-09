
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"

// ======================================
// 1. CONFIGURACION DE PINES
// ======================================

// Sensores infrarrojos E18-D80NK
#define SENSOR_A GPIO_NUM_18
#define SENSOR_B GPIO_NUM_19

// Semaforo de aforo
#define LED_VERDE    GPIO_NUM_25
#define LED_AMARILLO GPIO_NUM_26
#define LED_ROJO     GPIO_NUM_27

// Servomotor SG90
#define SERVO_GPIO GPIO_NUM_23

// Fotorresistencia: GPIO34 = ADC1 canal 6
#define CANAL_LDR ADC_CHANNEL_6

// ======================================
// 2. PARAMETROS DEL SISTEMA
// ======================================

#define AFORO_MAXIMO 10
#define UMBRAL_AMARILLO (AFORO_MAXIMO - 2)

// Sensores IR
#define PERIODO_LECTURA_MS 10
#define DEBOUNCE_MS 40
#define TIMEOUT_SECUENCIA_MS 2500
#define TIMEOUT_LIBERACION_MS 3000
#define TIEMPO_REARME_MS 300

// Tiempo de apertura temporal para salida
#define ESPERA_SALIDA_MS 2000

// Fotorresistencia
#define MUESTRAS_MEDIA 10
#define PERIODO_LDR_MS 100

// Telemetria JSON
#define PERIODO_JSON_MS 1000

// PWM del servomotor
#define SERVO_PWM_HZ 50
#define SERVO_RESOLUCION_BITS 14

#define SERVO_ABIERTO_US 1900
#define SERVO_CERRADO_US 1100

#define SERVO_MODO LEDC_LOW_SPEED_MODE
#define SERVO_TIMER LEDC_TIMER_0
#define SERVO_CANAL LEDC_CHANNEL_0

// ======================================
// 3. ESTRUCTURAS DE DATOS
// ======================================

// Filtro debounce de los sensores
typedef struct {
    int valor_estable;
    int valor_anterior;
    TickType_t ultimo_cambio;
} filtro_ir_t;

// Estados de la maquina de aforo
typedef enum {
    ESPERANDO_PRIMER_SENSOR,
    ESPERANDO_SENSOR_B,
    ESPERANDO_SENSOR_A,
    CONFIRMAR_ENTRADA,
    CONFIRMAR_SALIDA,
    REARME
} estado_aforo_t;

// Comandos del servomotor
typedef enum {
    SERVO_ABRIR,
    SERVO_CERRAR
} comando_servo_t;

// Datos destinados al empaquetado JSON
typedef struct {
    int ocupacion;

    bool sensor_a;
    bool sensor_b;

    int ldr_raw;
    int ldr_filtrado;
    bool ldr_valida;

    bool barrera_abierta;

} datos_iot_t;

// ======================================
// 4. VARIABLES GLOBALES
// ======================================

// Solo tarea_aforo modifica este contador
static int ocupacion = 0;

// Colas FreeRTOS
static QueueHandle_t cola_aforo = NULL;
static QueueHandle_t cola_servo = NULL;

// Mutex para proteger datos compartidos
static SemaphoreHandle_t mutex_datos = NULL;

// Estructura compartida para JSON
static datos_iot_t datos_iot = {
    .ocupacion = 0,
    .sensor_a = false,
    .sensor_b = false,
    .ldr_raw = 0,
    .ldr_filtrado = 0,
    .ldr_valida = false,
    .barrera_abierta = true
};

// ======================================
// 5. FUNCIONES PARA COMPARTIR DATOS
// ======================================

// Guardar estado de los sensores IR
void guardar_sensores(bool a, bool b)
{
    if (xSemaphoreTake(
            mutex_datos,
            portMAX_DELAY) == pdTRUE)
    {
        datos_iot.sensor_a = a;
        datos_iot.sensor_b = b;

        xSemaphoreGive(mutex_datos);
    }
}

// Guardar ocupacion actual
void guardar_ocupacion(int personas)
{
    if (xSemaphoreTake(
            mutex_datos,
            portMAX_DELAY) == pdTRUE)
    {
        datos_iot.ocupacion = personas;

        xSemaphoreGive(mutex_datos);
    }
}

// Guardar lectura y promedio LDR
void guardar_ldr(int lectura, int media)
{
    if (xSemaphoreTake(
            mutex_datos,
            portMAX_DELAY) == pdTRUE)
    {
        datos_iot.ldr_raw = lectura;
        datos_iot.ldr_filtrado = media;
        datos_iot.ldr_valida = true;

        xSemaphoreGive(mutex_datos);
    }
}

// Guardar posicion ordenada al servo
void guardar_servo(bool abierto)
{
    if (xSemaphoreTake(
            mutex_datos,
            portMAX_DELAY) == pdTRUE)
    {
        datos_iot.barrera_abierta = abierto;

        xSemaphoreGive(mutex_datos);
    }
}

// ======================================
// 6. FILTRO DEBOUNCE IR
// ======================================

void iniciar_filtro(filtro_ir_t *filtro,
                    gpio_num_t pin)
{
    int lectura = gpio_get_level(pin);

    filtro->valor_estable = lectura;
    filtro->valor_anterior = lectura;
    filtro->ultimo_cambio = xTaskGetTickCount();
}

bool actualizar_filtro(filtro_ir_t *filtro,
                       int lectura,
                       TickType_t ahora)
{
    // Registrar cambio de lectura
    if (lectura != filtro->valor_anterior)
    {
        filtro->valor_anterior = lectura;
        filtro->ultimo_cambio = ahora;
    }

    // Confirmar lectura estable
    if (lectura != filtro->valor_estable &&
        (ahora - filtro->ultimo_cambio) >=
        pdMS_TO_TICKS(DEBOUNCE_MS))
    {
        filtro->valor_estable = lectura;
        return true;
    }

    return false;
}

// ======================================
// 7. MOSTRAR OCUPACION
// ======================================

void mostrar_ocupacion(void)
{
    printf("\n===========================\n");
    printf("OCUPACION ACTUAL: %d\n", ocupacion);
    printf("AFORO MAXIMO: %d\n", AFORO_MAXIMO);

    if (ocupacion >= AFORO_MAXIMO)
    {
        printf("ESTADO: AFORO COMPLETO\n");
    }
    else if (ocupacion >= UMBRAL_AMARILLO)
    {
        printf("ESTADO: CASI LLENO\n");
    }
    else
    {
        printf("ESTADO: DISPONIBLE\n");
    }

    printf("===========================\n\n");
}

// ======================================
// 8. COMUNICACION ENTRE TAREAS
// ======================================

void solicitar_servo(comando_servo_t comando)
{
    if (xQueueOverwrite(
            cola_servo,
            &comando) != pdPASS)
    {
        printf("ERROR enviando comando al servo\n");
    }
}

void notificar_actuadores(bool mantener_abierto)
{
    // Actualizar estructura compartida
    guardar_ocupacion(ocupacion);

    // Enviar ocupacion al semaforo
    xQueueOverwrite(
        cola_aforo,
        &ocupacion
    );

    // Control de barrera
    if (mantener_abierto ||
        ocupacion < AFORO_MAXIMO)
    {
        solicitar_servo(SERVO_ABRIR);
    }
    else
    {
        solicitar_servo(SERVO_CERRAR);
    }
}

// ======================================
// 9. CONTROL DEL SEMAFORO
// ======================================

void actualizar_semaforo(int personas)
{
    int verde = 0;
    int amarillo = 0;
    int rojo = 0;

    if (personas >= AFORO_MAXIMO)
    {
        rojo = 1;

        printf(
            "SEMAFORO: ROJO - AFORO COMPLETO\n"
        );
    }
    else if (personas >= UMBRAL_AMARILLO)
    {
        amarillo = 1;

        printf(
            "SEMAFORO: AMARILLO - CASI LLENO\n"
        );
    }
    else
    {
        verde = 1;

        printf(
            "SEMAFORO: VERDE - DISPONIBLE\n"
        );
    }

    gpio_set_level(LED_VERDE, verde);
    gpio_set_level(LED_AMARILLO, amarillo);
    gpio_set_level(LED_ROJO, rojo);
}

// ======================================
// 10. TAREA FREERTOS: SEMAFORO
// ======================================

void tarea_semaforo(void *pvParameters)
{
    int personas_recibidas = 0;

    printf("TAREA SEMAFORO INICIADA\n");

    while (1)
    {
        if (xQueueReceive(
                cola_aforo,
                &personas_recibidas,
                portMAX_DELAY) == pdTRUE)
        {
            actualizar_semaforo(
                personas_recibidas
            );
        }
    }
}

// ======================================
// 11. CONTROL PWM DEL SERVOMOTOR
// ======================================

void mover_servo_us(uint32_t pulso_us)
{
    const uint32_t periodo_us =
        1000000UL / SERVO_PWM_HZ;

    uint32_t duty =
        (pulso_us *
        (1UL << SERVO_RESOLUCION_BITS))
        / periodo_us;

    ESP_ERROR_CHECK(
        ledc_set_duty(
            SERVO_MODO,
            SERVO_CANAL,
            duty
        )
    );

    ESP_ERROR_CHECK(
        ledc_update_duty(
            SERVO_MODO,
            SERVO_CANAL
        )
    );
}

// ======================================
// 12. TAREA FREERTOS: SERVOMOTOR
// ======================================

void tarea_servo(void *pvParameters)
{
    comando_servo_t comando;

    int estado_anterior = -1;

    printf("TAREA SERVOMOTOR INICIADA\n");

    while (1)
    {
        if (xQueueReceive(
                cola_servo,
                &comando,
                portMAX_DELAY) == pdTRUE)
        {
            if ((int)comando != estado_anterior)
            {
                if (comando == SERVO_ABRIR)
                {
                    mover_servo_us(
                        SERVO_ABIERTO_US
                    );

                    printf(
                        "SERVO: BARRERA ABIERTA\n"
                    );
                }
                else
                {
                    mover_servo_us(
                        SERVO_CERRADO_US
                    );

                    printf(
                        "SERVO: BARRERA CERRADA\n"
                    );
                }

                estado_anterior = (int)comando;

                // Actualizar estado para JSON
                guardar_servo(
                    comando == SERVO_ABRIR
                );
            }
        }
    }
}

// ======================================
// 13. TAREA FREERTOS: AFORO
// ======================================

void tarea_aforo(void *pvParameters)
{
    filtro_ir_t filtro_a;
    filtro_ir_t filtro_b;

    estado_aforo_t estado =
        ESPERANDO_PRIMER_SENSOR;

    TickType_t ahora;
    TickType_t inicio_estado = 0;
    TickType_t inicio_libres = 0;

    bool salida_temporal = false;

    iniciar_filtro(&filtro_a, SENSOR_A);
    iniciar_filtro(&filtro_b, SENSOR_B);

    printf("\nSISTEMA DE AFORO INICIADO\n");
    printf("Sensor A: GPIO18\n");
    printf("Sensor B: GPIO19\n");
    printf("Aforo maximo: %d\n\n",
           AFORO_MAXIMO);

    while (1)
    {
        ahora = xTaskGetTickCount();

        // Lecturas de los sensores IR
        int lectura_a = gpio_get_level(SENSOR_A);
        int lectura_b = gpio_get_level(SENSOR_B);

        // Aplicar debounce
        bool cambio_a = actualizar_filtro(
            &filtro_a,
            lectura_a,
            ahora
        );

        bool cambio_b = actualizar_filtro(
            &filtro_b,
            lectura_b,
            ahora
        );

        // E18-D80NK activos en nivel bajo
        bool a = (filtro_a.valor_estable == 0);
        bool b = (filtro_b.valor_estable == 0);

        // Actualizar datos compartidos
        guardar_sensores(a, b);

        // Mostrar cambios individuales
        if (cambio_a)
        {
            printf(
                "Sensor A: %s\n",
                a ? "DETECTADO" : "LIBRE"
            );
        }

        if (cambio_b)
        {
            printf(
                "Sensor B: %s\n",
                b ? "DETECTADO" : "LIBRE"
            );
        }

        // ==================================
        // MAQUINA DE ESTADOS
        // ==================================

        switch (estado)
        {
            // ------------------------------
            // ESPERAR PRIMER SENSOR
            // ------------------------------

            case ESPERANDO_PRIMER_SENSOR:

                if (a && !b)
                {
                    printf("A detecto primero\n");

                    inicio_estado = ahora;
                    estado = ESPERANDO_SENSOR_B;
                }
                else if (b && !a)
                {
                    printf("B detecto primero\n");

                    // Permitir intento de salida
                    // cuando el aforo esta completo
                    if (ocupacion >= AFORO_MAXIMO)
                    {
                        printf(
                            "SALIDA SOLICITADA: "
                            "ABRIENDO BARRERA\n"
                        );

                        salida_temporal = true;

                        solicitar_servo(
                            SERVO_ABRIR
                        );
                    }

                    inicio_estado = ahora;
                    estado = ESPERANDO_SENSOR_A;
                }
                else if (a && b)
                {
                    printf(
                        "Activacion simultanea: "
                        "ignorada\n"
                    );

                    inicio_libres = ahora;
                    estado = REARME;
                }

                break;

            // ------------------------------
            // ESPERAR SENSOR B
            // ------------------------------

            case ESPERANDO_SENSOR_B:

                if ((ahora - inicio_estado) >
                    pdMS_TO_TICKS(
                        TIMEOUT_SECUENCIA_MS))
                {
                    printf(
                        "Entrada incompleta: "
                        "ignorada\n"
                    );

                    inicio_libres = ahora;
                    estado = REARME;
                }
                else if (b)
                {
                    printf(
                        "Secuencia A -> B detectada\n"
                    );

                    inicio_estado = ahora;
                    estado = CONFIRMAR_ENTRADA;
                }

                break;

            // ------------------------------
            // ESPERAR SENSOR A
            // ------------------------------

            case ESPERANDO_SENSOR_A:

                if ((ahora - inicio_estado) >
                    pdMS_TO_TICKS(
                        TIMEOUT_SECUENCIA_MS))
                {
                    printf(
                        "Salida incompleta: "
                        "ignorada\n"
                    );

                    inicio_libres = ahora;
                    estado = REARME;
                }
                else if (a)
                {
                    printf(
                        "Secuencia B -> A detectada\n"
                    );

                    inicio_estado = ahora;
                    estado = CONFIRMAR_SALIDA;
                }

                break;

            // ------------------------------
            // CONFIRMAR ENTRADA
            // ------------------------------

            case CONFIRMAR_ENTRADA:

                if (!a && !b)
                {
                    ocupacion++;

                    printf(
                        "\nENTRADA CONFIRMADA\n"
                    );

                    mostrar_ocupacion();

                    // Actualizar semaforo y servo
                    notificar_actuadores(false);

                    inicio_libres = ahora;
                    estado = REARME;
                }
                else if (
                    (ahora - inicio_estado) >
                    pdMS_TO_TICKS(
                        TIMEOUT_LIBERACION_MS))
                {
                    printf(
                        "Entrada cancelada: "
                        "sensores bloqueados\n"
                    );

                    inicio_libres = ahora;
                    estado = REARME;
                }

                break;

            // ------------------------------
            // CONFIRMAR SALIDA
            // ------------------------------

            case CONFIRMAR_SALIDA:

                if (!a && !b)
                {
                    if (ocupacion > 0)
                    {
                        ocupacion--;

                        printf(
                            "\nSALIDA CONFIRMADA\n"
                        );
                    }
                    else
                    {
                        printf(
                            "\nSALIDA DETECTADA: "
                            "ocupacion ya es cero\n"
                        );
                    }

                    mostrar_ocupacion();

                    bool mantener_abierto =
                        salida_temporal &&
                        ocupacion >= AFORO_MAXIMO;

                    notificar_actuadores(
                        mantener_abierto
                    );

                    if (ocupacion < AFORO_MAXIMO)
                    {
                        salida_temporal = false;
                    }

                    inicio_libres = ahora;
                    estado = REARME;
                }
                else if (
                    (ahora - inicio_estado) >
                    pdMS_TO_TICKS(
                        TIMEOUT_LIBERACION_MS))
                {
                    printf(
                        "Salida cancelada: "
                        "sensores bloqueados\n"
                    );

                    inicio_libres = ahora;
                    estado = REARME;
                }

                break;

            // ------------------------------
            // REARME DEL SISTEMA
            // ------------------------------

            case REARME:

                if (a || b)
                {
                    inicio_libres = ahora;
                }
                else
                {
                    TickType_t espera =
                        pdMS_TO_TICKS(
                            salida_temporal ?
                            ESPERA_SALIDA_MS :
                            TIEMPO_REARME_MS
                        );

                    if ((ahora - inicio_libres)
                        >= espera)
                    {
                        if (salida_temporal)
                        {
                            if (ocupacion >=
                                AFORO_MAXIMO)
                            {
                                solicitar_servo(
                                    SERVO_CERRAR
                                );
                            }

                            salida_temporal = false;
                        }

                        estado =
                            ESPERANDO_PRIMER_SENSOR;

                        printf(
                            "Sistema listo "
                            "para otro paso\n"
                        );
                    }
                }

                break;
        }

        vTaskDelay(
            pdMS_TO_TICKS(
                PERIODO_LECTURA_MS
            )
        );
    }
}

// ======================================
// 14. TAREA FREERTOS: LDR
// ======================================

void tarea_ldr(void *pvParameters)
{
    adc_oneshot_unit_handle_t adc_handle =
        (adc_oneshot_unit_handle_t)pvParameters;

    int ventana[MUESTRAS_MEDIA] = {0};

    int indice = 0;
    int cantidad = 0;
    int suma = 0;
    int nuevas_lecturas = 0;

    printf("\nTAREA LDR INICIADA\n");
    printf("GPIO34 - ADC1 Canal 6\n");
    printf(
        "Media movil de %d muestras\n",
        MUESTRAS_MEDIA
    );

    while (1)
    {
        int lectura = 0;

        esp_err_t resultado =
            adc_oneshot_read(
                adc_handle,
                CANAL_LDR,
                &lectura
            );

        if (resultado == ESP_OK)
        {
            // Retirar la muestra mas antigua
            if (cantidad == MUESTRAS_MEDIA)
            {
                suma -= ventana[indice];
            }
            else
            {
                cantidad++;
            }

            // Incorporar muestra nueva
            ventana[indice] = lectura;
            suma += lectura;

            // Indice circular
            indice =
                (indice + 1) % MUESTRAS_MEDIA;

            // Filtro de media movil
            int media = suma / cantidad;

            // Actualizar datos para JSON
            guardar_ldr(lectura, media);

            nuevas_lecturas++;

            // Mostrar aproximadamente cada segundo
            if (nuevas_lecturas >= MUESTRAS_MEDIA)
            {
                printf(
                    "LDR -> Lectura: %d | "
                    "Media movil: %d\n",
                    lectura,
                    media
                );

                nuevas_lecturas = 0;
            }
        }
        else
        {
            printf(
                "Error leyendo LDR: %s\n",
                esp_err_to_name(resultado)
            );
        }

        vTaskDelay(
            pdMS_TO_TICKS(
                PERIODO_LDR_MS
            )
        );
    }
}

// ======================================
// 15. TAREA FREERTOS: JSON
// ======================================

void tarea_json(void *pvParameters)
{
    datos_iot_t copia;

    char mensaje[384];

    printf("TAREA JSON INICIADA\n");

    while (1)
    {
        vTaskDelay(
            pdMS_TO_TICKS(
                PERIODO_JSON_MS
            )
        );

        // Obtener una copia consistente
        // de los datos compartidos
        if (xSemaphoreTake(
                mutex_datos,
                portMAX_DELAY) != pdTRUE)
        {
            continue;
        }

        copia = datos_iot;

        xSemaphoreGive(mutex_datos);

        // Determinar estado logico del semaforo
        const char *semaforo;

        if (copia.ocupacion >= AFORO_MAXIMO)
        {
            semaforo = "rojo";
        }
        else if (
            copia.ocupacion >= UMBRAL_AMARILLO)
        {
            semaforo = "amarillo";
        }
        else
        {
            semaforo = "verde";
        }

        // Tiempo transcurrido desde arranque
        unsigned long tiempo_ms =
            (unsigned long)xTaskGetTickCount() *
            portTICK_PERIOD_MS;

        // Construir mensaje JSON
        int longitud = snprintf(
            mensaje,
            sizeof(mensaje),

            "{"
            "\"dispositivo\":\"esp32_aforo_01\","
            "\"tiempo_ms\":%lu,"
            "\"ocupacion\":%d,"
            "\"aforo_maximo\":%d,"
            "\"sensor_a\":%s,"
            "\"sensor_b\":%s,"
            "\"ldr_raw\":%d,"
            "\"ldr_filtrado\":%d,"
            "\"ldr_valida\":%s,"
            "\"semaforo\":\"%s\","
            "\"barrera\":\"%s\""
            "}",

            tiempo_ms,
            copia.ocupacion,
            AFORO_MAXIMO,

            copia.sensor_a ? "true" : "false",
            copia.sensor_b ? "true" : "false",

            copia.ldr_raw,
            copia.ldr_filtrado,

            copia.ldr_valida ?
                "true" : "false",

            semaforo,

            copia.barrera_abierta ?
                "abierta" : "cerrada"
        );

        // Validar que el buffer fue suficiente
        if (longitud > 0 &&
            longitud < (int)sizeof(mensaje))
        {
            // Imprimir JSON en una sola linea
            printf("%s\n", mensaje);
        }
        else
        {
            printf(
                "ERROR construyendo JSON\n"
            );
        }
    }
}

// ======================================
// 16. FUNCION PRINCIPAL ESP-IDF
// ======================================

void app_main(void)
{
    printf("\n=================================\n");
    printf("    CAPA DE PERCEPCION IoT\n");
    printf("    ESP32 - ESP-IDF - FreeRTOS\n");
    printf("=================================\n\n");

    // ==================================
    // CONFIGURACION DEL SEMAFORO
    // ==================================

    gpio_config_t config_leds = {
        .pin_bit_mask =
            (1ULL << LED_VERDE) |
            (1ULL << LED_AMARILLO) |
            (1ULL << LED_ROJO),

        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    ESP_ERROR_CHECK(
        gpio_config(&config_leds)
    );

    // Inicialmente todos los LED apagados
    gpio_set_level(LED_VERDE, 0);
    gpio_set_level(LED_AMARILLO, 0);
    gpio_set_level(LED_ROJO, 0);

    // ==================================
    // CONFIGURACION SENSORES IR
    // ==================================

    gpio_config_t config_sensores = {
        .pin_bit_mask =
            (1ULL << SENSOR_A) |
            (1ULL << SENSOR_B),

        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    ESP_ERROR_CHECK(
        gpio_config(&config_sensores)
    );

    // ==================================
    // CONFIGURACION ADC PARA LDR
    // ==================================

    adc_oneshot_unit_handle_t adc_handle = NULL;

    adc_oneshot_unit_init_cfg_t config_adc = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE
    };

    ESP_ERROR_CHECK(
        adc_oneshot_new_unit(
            &config_adc,
            &adc_handle
        )
    );

    adc_oneshot_chan_cfg_t config_canal = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12
    };

    ESP_ERROR_CHECK(
        adc_oneshot_config_channel(
            adc_handle,
            CANAL_LDR,
            &config_canal
        )
    );

    // ==================================
    // CONFIGURACION PWM SERVO SG90
    // ==================================

    ledc_timer_config_t config_servo_timer = {
        .speed_mode = SERVO_MODO,
        .duty_resolution = LEDC_TIMER_14_BIT,
        .timer_num = SERVO_TIMER,
        .freq_hz = SERVO_PWM_HZ,
        .clk_cfg = LEDC_AUTO_CLK
    };

    ESP_ERROR_CHECK(
        ledc_timer_config(
            &config_servo_timer
        )
    );

    ledc_channel_config_t config_servo_canal = {
        .gpio_num = SERVO_GPIO,
        .speed_mode = SERVO_MODO,
        .channel = SERVO_CANAL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = SERVO_TIMER,
        .duty = 0,
        .hpoint = 0
    };

    ESP_ERROR_CHECK(
        ledc_channel_config(
            &config_servo_canal
        )
    );

    // ==================================
    // CREAR MUTEX DE DATOS
    // ==================================

    mutex_datos = xSemaphoreCreateMutex();

    if (mutex_datos == NULL)
    {
        printf(
            "ERROR creando mutex de datos\n"
        );
        return;
    }

    // ==================================
    // CREAR COLAS FREERTOS
    // ==================================

    cola_aforo = xQueueCreate(
        1,
        sizeof(int)
    );

    cola_servo = xQueueCreate(
        1,
        sizeof(comando_servo_t)
    );

    if (cola_aforo == NULL ||
        cola_servo == NULL)
    {
        printf(
            "ERROR creando colas FreeRTOS\n"
        );
        return;
    }

    // ==================================
    // ESTADOS INICIALES
    // ==================================

    // Ocupacion inicial = 0
    xQueueOverwrite(
        cola_aforo,
        &ocupacion
    );

    // Barrera inicialmente abierta
    comando_servo_t comando_inicial =
        SERVO_ABRIR;

    xQueueOverwrite(
        cola_servo,
        &comando_inicial
    );

    // ==================================
    // CREAR LAS CINCO TAREAS FREERTOS
    // ==================================

    // Tarea 1: Semaforo
    BaseType_t resultado_semaforo = xTaskCreate(
        tarea_semaforo,
        "Tarea_Semaforo",
        3072,
        NULL,
        4,
        NULL
    );

    // Tarea 2: Servomotor
    BaseType_t resultado_servo = xTaskCreate(
        tarea_servo,
        "Tarea_Servo",
        3072,
        NULL,
        4,
        NULL
    );

    // Tarea 3: Conteo de personas
    BaseType_t resultado_aforo = xTaskCreate(
        tarea_aforo,
        "Tarea_Aforo",
        4096,
        NULL,
        5,
        NULL
    );

    // Tarea 4: Fotorresistencia
    BaseType_t resultado_ldr = xTaskCreate(
        tarea_ldr,
        "Tarea_LDR",
        3072,
        adc_handle,
        3,
        NULL
    );

    // Tarea 5: Empaquetado JSON
    BaseType_t resultado_json = xTaskCreate(
        tarea_json,
        "Tarea_JSON",
        4096,
        NULL,
        2,
        NULL
    );

    // ==================================
    // VERIFICAR CREACION DE TAREAS
    // ==================================

    if (resultado_semaforo != pdPASS ||
        resultado_servo != pdPASS ||
        resultado_aforo != pdPASS ||
        resultado_ldr != pdPASS ||
        resultado_json != pdPASS)
    {
        printf(
            "ERROR creando tareas FreeRTOS\n"
        );
    }
    else
    {
        printf("\n=============================\n");
        printf("SISTEMA IoT INICIADO\n");
        printf("5 tareas FreeRTOS creadas\n");
        printf("2 colas de comunicacion\n");
        printf("1 mutex de proteccion\n");
        printf("Servo PWM: GPIO23 - 50 Hz\n");
        printf("LDR: GPIO34\n");
        printf("JSON: cada 1000 ms\n");
        printf("=============================\n\n");
    }
}
