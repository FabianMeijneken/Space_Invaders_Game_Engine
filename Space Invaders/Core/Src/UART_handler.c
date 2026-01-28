/**
  ******************************************************************************
  * @file           : UART_handler.c
  * @brief          : Alle functies betreft de UART naar FPGA
  ******************************************************************************
  *
  * @author 		: Fabian Meijneken
  ******************************************************************************
  */


#include "main.h"
#include "UART_handler.h"

// Een functie die true of false returned bij een respectievelijk negatieve of positieve waarde
// Deze functie bestaat alleen maar om de leesbaarheid van code te bevorderen.
// Gemaakt door Fabian Meijneken 10/01/2026
bool is_negative(int value)
{
    if (value < 0)
        return true;
    return false;
}


// Een functie die uit een aantal variabelen een UART-string maakt die naar de FPGA kan worden gestuurd.
// Deze functie bepaalt de hoeveelheid bits, volgorde en locatie van elk variabel zodat de FPGA deze kan begrijpen
// Gemaakt door Fabian Meijneken 10/01/2026
void update_FPGA(uint8_t obj_ID, int obj_x, int obj_y, uint8_t obj_render_bits)
{
	// Alle game engine uart communicatie kan worden uitgezet door OUTPUT_FPGA niet te definieren.
	#ifdef OUTPUT_FPGA
		const bool y_neg = is_negative(obj_y);
		const bool x_neg = is_negative(obj_x);

		const unsigned int obj_x_abs = abs(obj_x);
		const unsigned int obj_y_abs = abs(obj_y);

		// in uart_string worden alle variabelen samengevoegd tot 1 commando. (unsigned aangezien we geen gedoe willen met de MSB (die aangeeft of het neg of pos is))
		uint32_t uart_string = 0;

		// Zet alle bits op de juiste plek in de uart_string. (bits gaan van 31 naar 0)
		// Dit doen we door een masker te maken op specifieke plekken
		// Uitleg aan de hand van de regel "uart_string |= (bin_y_neg << 31)":
		//  - (bin_y_neg << 31) → verschuif de bin_y_neg bit 31 plekken naar links.
		//    Je krijgt dus een 32 bitjes lang masker waarop op de 32e plek (in dit geval de MSB dus) bin_y_neg staat.
		//  - |= → We voegen de uart string en het masker samen met een or operator.
		//    Als het masker of de originele waarde 1 is, maak dan de nieuwe bit 1.

		uart_string |= (y_neg << 31);                           // bit  31              = y_neg
		uart_string |= ((obj_y_abs & 0b111111111) << 22);       // bits 22 t/m 30 (9)   = y
		uart_string |= (x_neg << 21);                           // bit  21              = x_neg
		uart_string |= ((obj_x_abs & 0b1111111111) << 11);      // bits 11 t/m 20 (10)  = x
		uart_string |= ((obj_render_bits & 0b111111) << 5);     // bits 5 t/m 10 (6)    = obj_render
		uart_string |= ((obj_ID & 0b1111) << 1);                // bits 1 t/m 4 (4)     = obj_id
		uart_string |= (0b1);									// bit 	1				= 1

		// Zet het UART commando in de tx_buffer, en hoog de uart2_upload_index met 1.
		tx_buffer[uart2_upload_index++] = uart_string;

		// Zorg dat de uart2_upload_index niet buiten de buffer staat.
		if (uart2_upload_index > (UART2_TX_BUFFER_SIZE - 1))
			uart2_upload_index = 0;

	#endif
}
