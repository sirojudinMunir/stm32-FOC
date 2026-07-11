#ifndef COM_H
#define COM_H

#include <stdint.h>
#include "string.h"
#include "motor.h"
#include "AS5047P.h"
#include "FOC_utils.h"
#include "self_commissioning.h"
#include "storage.h"

typedef enum {
    COM_DATA_TYPE_UINT8,
    COM_DATA_TYPE_INT8,
    COM_DATA_TYPE_UINT16,
    COM_DATA_TYPE_INT16,
    COM_DATA_TYPE_UINT32,
    COM_DATA_TYPE_INT32,
    COM_DATA_TYPE_FLOAT32,
}com_data_type_t;

typedef struct {
    _Bool incomming_data_flag;
    uint8_t *data_rx;
    uint32_t data_rx_len;
    foc_t *pfoc;
    storage_t *pstorage;
    self_commissioning_t *psc;
    _Bool send_data_pending;
    int (*recv_data)(uint8_t*, uint16_t);
    int (*send_data)(uint8_t*, uint16_t);
}com_t;

void com_init(com_t *com, int (*recv_data)(uint8_t*, uint16_t), int (*send_data)(uint8_t*, uint16_t),
              foc_t *pfoc, storage_t *pstorage, self_commissioning_t *psc);
void com_update(com_t *com);

#endif // COM_H