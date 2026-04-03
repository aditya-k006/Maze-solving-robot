#ifndef TFMINI_H
#define TFMINI_H

#include <stdint.h>
#include <stdbool.h>

// ----------------------------------------------------------------
//  TFMini / TFMini-S LiDAR driver.
//
//  Protocol: 9-byte UART frame at 115200 8N1
//    [0] 0x59  header byte 1
//    [1] 0x59  header byte 2
//    [2] DIST_L   distance low  byte  (unit: cm)
//    [3] DIST_H   distance high byte
//    [4] STR_L    signal strength low  byte
//    [5] STR_H    signal strength high byte
//    [6] TEMP_L   (reserved on basic TFMini, temp on TFMini-S)
//    [7] TEMP_H
//    [8] CHECKSUM = (sum of bytes 0..7) & 0xFF
//
//  Output unit returned by this driver: millimetres (cm × 10).
//
//  Sensor mapping:
//    TF_FRONT → Hardware UART0 (GP1 RX)
//    TF_LEFT  → Hardware UART1 (GP21 RX)
//    TF_RIGHT → PIO soft-UART  (GP22 RX)  ← uses uart_rx.pio
// ----------------------------------------------------------------

typedef enum {
    TF_FRONT = 0,
    TF_LEFT  = 1,
    TF_RIGHT = 2
} tf_id_t;

// Initialise all three sensors (UARTs + PIO).
void tfmini_init_all(void);

// Blocking read of one complete frame.
// Returns true and writes distance (mm) and strength on success.
// Returns false on checksum error or timeout (~50 ms).
bool tfmini_read_frame(tf_id_t id, uint16_t *dist_mm, uint16_t *strength);

// Convenience: return distance in mm, or 0 on error.
uint16_t tfmini_get_distance(tf_id_t id);

#endif // TFMINI_H
