#ifndef VALIDATION
#define VALIDATION

#include <stdint.h>

/* PROTOTYPES */

uint8_t calculate_parity(uint8_t header);
uint8_t validate_parity(uint8_t header);

uint8_t calcular_checkbyte(const uint8_t *bytes, size_t longitud);
uint8_t validate_checkbyte(const uint8_t *bytes, size_t longitud, uint8_t checkbyte_received);

#endif
