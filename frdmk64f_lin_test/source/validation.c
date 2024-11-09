/*
 * validation.c
 *
 *  Created on: 6 Nov 2024
 *      Author: Luis Cardenas
 */

/* INCLUDE */
#include "validation.h"

/* Include only if you want to use the "PRINTF" function */
#include "fsl_debug_console.h"

/* "CALCULATE_PARITY FUNCTION" 
 * The next function calculates the 2 parity bits
 * this has the header byte as parameter
 * the function returns the header byte plus the 2 parity bits
 */
uint8_t calculate_parity(uint8_t header)
{
    uint8_t header_for_calculate;

    /* ALL THE PRINTF FUNCTIONS ARE ONLY FOR DEBUGGING YOU CAN DELETE THEM */

    /* PRINTF("Segundo byte en binario: \r\n");
    for (int i = 7; i >= 0; i--) {
        PRINTF("%d", (header >> i) & 1);
    }
    PRINTF("\r\n"); */

    header_for_calculate = header & 0b11111100;

    /* PRINTF("filtrado en binario: \r\n");
    for (int i = 7; i >= 2; i--) {
        PRINTF("%d", (header_for_calculate >> i) & 1);
    }
    PRINTF("\r\n"); */

    uint8_t P0 = 0;
    
    P0 = ((header_for_calculate >> 7) & 1) ^  // Bit 0 (más a la izquierda)
                           ((header_for_calculate >> 6) & 1) ^  // Bit 1
                           ((header_for_calculate >> 5) & 1) ^  // Bit 2
                           ((header_for_calculate >> 3) & 1);

    //PRINTF("P0: %d\r\n",P0);

    uint8_t P1 = 0;

    P1 = ((header_for_calculate >> 6) & 1) ^  // Bit 0 (más a la izquierda)
                           ((header_for_calculate >> 4) & 1) ^  // Bit 1
                           ((header_for_calculate >> 3) & 1) ^  // Bit 2
                           ((header_for_calculate >> 2) & 1);

   //PRINTF("P1: %d\r\n",P1);

    uint8_t total = ((P0<<1) | P1);

    /* PRINTF("2 bits de paridad sumados: \r\n");
    for (int i = 7; i >= 0; i--) {
        PRINTF("%d", (total >> i) & 1);
    }
    PRINTF("\r\n"); */
    
    uint8_t header_with_paritybits = header | total;

    /* PRINTF("Header final dentro de validation: \r\n");
    for (int i = 7; i >= 0; i--) {
        PRINTF("%d", (header_with_paritybits >> i) & 1);
    }
    PRINTF("\r\n"); */

    return header_with_paritybits;
}


/* "VALIDATE_PARITY FUNCTION" 
 * The next function calculates the 2 parity bits
 * this has the header byte as parameter, this header already has the parity bits (header received)
 * the function returns "1" if the parity bits are the same, and returns "0" if the parity bits don't match
 */
uint8_t validate_parity(uint8_t header)
{
    uint8_t parity_bits;
    uint8_t parity_bits_calculated;

    parity_bits = header & 0b00000011;
    
    /* PRINTF("Bits de paridad dentro del header: \r\n");
    for (int i = 7; i >= 0; i--) {
        PRINTF("%d", (parity_bits >> i) & 1);
    }
    PRINTF("\r\n"); */

    parity_bits_calculated =  calculate_parity(header) & 0b00000011;

    /* PRINTF("Bits de paridad calculados: \r\n");
    for (int i = 7; i >= 0; i--) {
        PRINTF("%d", (parity_bits_calculated >> i) & 1);
    }
    PRINTF("\r\n"); */

    uint8_t parity_check = (parity_bits_calculated == parity_bits) ? 1 : 0;

    //PRINTF("Validacion en validation = %d\r\n", parity_check);

    return parity_check;

}


/* "CALCULATE_CHECKBYTE FUNCTION" 
 * This function calculates the checksum of the bytes for transmission
 * this has the header byte as parameter, also has the length used in the "for" loop
 * This function returns a byte that represents the checkbyte
 */
uint8_t calcular_checkbyte(const uint8_t *bytes, size_t longitud)
{
    uint16_t suma = 0;

    for (size_t i = 0; i < longitud; i++) {
        suma += bytes[i];

        // Si hay un acarreo, sumarlo al byte inferior
        suma = (suma & 0xFF) + (suma >> 8);
    }

    // Complemento a uno del resultado truncado a 8 bits
    uint8_t checksum = ~((uint8_t)suma);

    return checksum;
}


/* "VALIDATE_CHECKBYTE FUNCTION" 
 * This function calculates the checksum of the received bytes and then compares it with the received check byte
 * this has the header byte as parameter, also has the length used in the "for" loop, finally you need to add the received checkbyte
 * the function returns "1" if the check_bytes are the same, and returns "0" if the check_bytes don't match
 */
uint8_t validate_checkbyte(const uint8_t *bytes, size_t longitud, uint8_t checkbyte_received)
{
    uint16_t suma = 0;

    for (size_t i = 0; i < longitud; i++) {
        suma += bytes[i];

        // Si hay un acarreo, sumarlo al byte inferior
        suma = (suma & 0xFF) + (suma >> 8);
    }

    // Complemento a uno del resultado truncado a 8 bits
    uint8_t checksum = ~((uint8_t)suma);

    if (checkbyte_received == checksum)
    {
        return 1;
    }
    else
    {
        return 0;
    }
    
}