# include "ax_controller.h"
# include "usart.h"
#include <stdint.h>
#include <sys/types.h>


# define AX_INDEX_ID 2
# define AX_INDEX_LEN 3
# define AX_INDEX_INSTRUCTION 4


void ax_init() {}

// byte per byte :
// <header> <header> <id> <length> <instruction> ... <checksum>
uint8_t ax_instruction_msg[] = {0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};


/**
 * @brief Reads the length of the packet and calculates the checksum in the correct place.
 */
void ax_calculate_checksum()
{
	uint8_t packet_len = ax_instruction_msg[AX_INDEX_LEN];

	uint32_t chksum = 0;
	for (int i = 2; i < AX_INDEX_LEN + packet_len; i++) chksum += ax_instruction_msg[i];
	ax_instruction_msg[AX_INDEX_LEN + packet_len] = (uint8_t)(chksum & 0xFF);
	ax_instruction_msg[AX_INDEX_LEN + packet_len] = ~ ax_instruction_msg[AX_INDEX_LEN + packet_len]; // complement
}


/**
 * @brief Sends the current packet.
 * Adjusts the size by reading the contents of the array.
 */
void ax_send_packet()
{
	uint16_t total_packet_len = AX_INDEX_LEN + ax_instruction_msg[AX_INDEX_LEN] + 1;

	HAL_GPIO_WritePin(AX_DIR_GPIO, AX_DIR_GPIO_PIN, GPIO_PIN_RESET); // switch half duplex adapter to TX
	HAL_UART_Transmit_IT(AX_UART, ax_instruction_msg, total_packet_len);
}



/**
 * @brief Sends a write instruction of two bytes.
 * 
 * @param id The ID of the destination AX
 * @param address The address in the AX's control table the data will be written to
 * @param data The data to write to the AX
 */
void ax_send_instruction_write_1_2B(uint8_t id, enum AX_ADDR_2B_RW address, uint16_t data)
{
	ax_instruction_msg[AX_INDEX_ID] = id;
	ax_instruction_msg[AX_INDEX_LEN] = 0x05;
	ax_instruction_msg[AX_INDEX_INSTRUCTION] = 0x03;

	// sets the address 
	ax_instruction_msg[5] = (uint8_t)address;

	// sets the data
	ax_instruction_msg[6] = (uint8_t)(data) & 0xFF;
	ax_instruction_msg[7] = (uint8_t)(data >> 8) & 0xFF;

	ax_calculate_checksum();
	ax_send_packet();
}



/**
 * @brief Sends a write instruction of two times two bytes.
 * 
 * @param id The ID of the destination AX
 * @param address The address in the AX's control table the data will be written to
 * @param data The data to write to the AX
 */
void ax_send_instruction_write_2_2B(uint8_t id, enum AX_ADDR_2B_RW_DBL address, uint16_t data1, uint16_t data2)
{
	ax_instruction_msg[AX_INDEX_ID] = id;
	ax_instruction_msg[AX_INDEX_LEN] = 0x07;
	ax_instruction_msg[AX_INDEX_INSTRUCTION] = 0x03;

	// sets the address 
	ax_instruction_msg[5] = (uint8_t)address;

	// sets the data
	ax_instruction_msg[6] = (uint8_t)(data1) & 0xFF;
	ax_instruction_msg[7] = (uint8_t)(data1 >> 8) & 0xFF;
	ax_instruction_msg[8] = (uint8_t)(data2) & 0xFF;
	ax_instruction_msg[9] = (uint8_t)(data2 >> 8) & 0xFF;

	ax_calculate_checksum();
	ax_send_packet();
}


/**
 * @brief Write to the "Goal Position" table entry of AX ID
 * 
 * @param position The position between 0 and 1023
 */
void ax_write_position(uint8_t id, uint16_t position)
{
	ax_send_instruction_write(id, GOAL_POSITION, position);
}

void ax_write_speed(uint8_t id, uint16_t position)
{
	ax_send_instruction_write(id, MOVING_SPEED, position);
}

void ax_write_position_and_speed(uint8_t id, uint16_t position, uint16_t speed)
{
	ax_send_instruction_write_2_2B(id, GOAL_POSITION_MOVING_SPEED, position, speed);
}


