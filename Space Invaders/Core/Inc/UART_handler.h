/**
  ******************************************************************************
  * @file           : UART_handler.h
  * @brief          : UART_handler header file.
  ******************************************************************************
  *
  * @author 		: Fabian Meijneken
  ******************************************************************************
  */

// External variables
extern uint32_t tx_buffer[UART2_TX_BUFFER_SIZE];
extern uint8_t uart2_DMA_index, uart2_upload_index;
extern bool UART2_busy;


// Functie prototypes
bool is_negative(int value);
void update_FPGA(uint8_t obj_ID, int obj_x, int obj_y, uint8_t obj_render_bits);
