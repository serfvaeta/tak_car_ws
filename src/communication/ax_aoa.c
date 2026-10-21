#include "ax_aoa.h"
#include "ax_uart3.h"
#include "openctr_features.h"
#include "FreeRTOS.h"
#include "task.h"

#if OPENCTR_ENABLE_AOA_RECEIVER

#define AOA_RX_BUFFER_SIZE 512u
#define AOA_USB_CHUNK_SIZE 28u
#define AOA_MAX_CHUNKS_PER_STEP 4u

static uint8_t rx_buffer[AOA_RX_BUFFER_SIZE];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;
static volatile uint32_t rx_total;
static volatile uint16_t rx_dropped;
static AX_AOA_Parser parser;
static AX_AOA_Sample latest_sample;
static uint8_t latest_valid;

void AX_AOA_OnRxByte(uint8_t byte)
{
    uint16_t next = (uint16_t)((rx_head + 1u) % AOA_RX_BUFFER_SIZE);
    rx_total++;
    if (next == rx_tail) {
        rx_dropped++;
        return;
    }
    rx_buffer[rx_head] = byte;
    rx_head = next;
}

void AX_AOA_TaskStep(void)
{
    static uint8_t status_ticks;
    uint8_t chunk[AOA_USB_CHUNK_SIZE];
    uint8_t status[6];
    uint8_t frame;
    uint8_t count;
    uint8_t index;
    uint32_t total;
    uint16_t dropped;
    uint32_t primask;
    AX_AOA_Measurement measurement;
    uint32_t received_tick = (uint32_t)xTaskGetTickCount();

    for (frame = 0; frame < AOA_MAX_CHUNKS_PER_STEP; frame++) {
        count = 0;
        primask = __get_PRIMASK();
        __disable_irq();
        while (count < AOA_USB_CHUNK_SIZE && rx_tail != rx_head) {
            chunk[count++] = rx_buffer[rx_tail];
            rx_tail = (uint16_t)((rx_tail + 1u) % AOA_RX_BUFFER_SIZE);
        }
        __set_PRIMASK(primask);
        if (count == 0) break;
        for (index = 0; index < count; ++index) {
            if (AX_AOA_ParserPushByte(&parser, chunk[index], &measurement)) {
                primask = __get_PRIMASK();
                __disable_irq();
                latest_sample.measurement = measurement;
                latest_sample.received_tick = received_tick;
                latest_valid = 1;
                __set_PRIMASK(primask);
            }
        }
        AX_UART3_SendPacket(chunk, count, ID_UTX_AOA_RAW);
    }

    if (++status_ticks < 50u) return;
    status_ticks = 0;
    primask = __get_PRIMASK();
    __disable_irq();
    total = rx_total;
    dropped = rx_dropped;
    __set_PRIMASK(primask);
    status[0] = (uint8_t)(total >> 24);
    status[1] = (uint8_t)(total >> 16);
    status[2] = (uint8_t)(total >> 8);
    status[3] = (uint8_t)total;
    status[4] = (uint8_t)(dropped >> 8);
    status[5] = (uint8_t)dropped;
    AX_UART3_SendPacket(status, sizeof(status), ID_UTX_AOA_STATUS);
}

uint8_t AX_AOA_GetLatest(AX_AOA_Sample *out, uint32_t max_age_ms)
{
    AX_AOA_Sample snapshot;
    uint8_t valid;
    uint32_t primask;

    if (out == 0) return 0;
    primask = __get_PRIMASK();
    __disable_irq();
    snapshot = latest_sample;
    valid = latest_valid;
    __set_PRIMASK(primask);
    if (!valid || (uint32_t)(xTaskGetTickCount() - snapshot.received_tick) >
                  (uint32_t)pdMS_TO_TICKS(max_age_ms))
        return 0;
    *out = snapshot;
    return 1;
}

#else

void AX_AOA_OnRxByte(uint8_t byte) { (void)byte; }
void AX_AOA_TaskStep(void) {}
uint8_t AX_AOA_GetLatest(AX_AOA_Sample *out, uint32_t max_age_ms)
{
    (void)out;
    (void)max_age_ms;
    return 0;
}

#endif
