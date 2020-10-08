//File Name: motor-driver-data-module.hpp
//Description: This holds the headers for all the different types of motor driver messages

//Header Guards
#ifndef MOTOR_DRIVER_DATA_MODULE_H_
#define MOTOR_DRIVER_DATA_MODULE_H_

//Include Files
#include "subsystem-data-module.hpp"
//C Interface
#ifdef __cplusplus
extern "C" {
#endif
//C Public Constants
/*This is an example description for constants and variables. Delete this if it is unused here and copy and paste it to where it is needed. */
/**
 * @brief This is a brief description
 */

//C Public Variables

//C Public Function Prototypes
/*This is an example description for function prototypes. Delete this if it is unused here and copy and paste it to where it is needed. */
/**
 * @brief This is a brief description
 * @param Input_Param_Name - Description of input parameter
 * @ret Description of return value
 */


#ifdef __cplusplus
}
#endif //End C Interface

//C++ Interface
//Class Definitions
/*****************TX Classes*****************/
struct MOTOR_DRIVER_TX_RL_DATA_PACKET
{
    bool requestFrame0;
    bool requestFrame1;
    bool requestFrame2;
};

class MOTOR_DRIVER_TX_RL_MESSAGE final: public SUBSYSTEM_DATA_MODULE_TEMPLATE_INTERFACE<MOTOR_DRIVER_TX_RL_MESSAGE, MOTOR_DRIVER_TX_RL_DATA_PACKET>
{
public:
//Constructors
MOTOR_DRIVER_TX_RL_MESSAGE();
//Public Constants
static constexpr uint8_t NUM_BYTES = 1;
//Public Function Prototypes
/**
 * @brief This function converts @input to fill the encoded @output array
 * @param input: Data to be converted
 * @param output: Array that should be allocated at least @ARRAY_SIZE bytes
 */
static void dataPacketToArray(MOTOR_DRIVER_TX_RL_DATA_PACKET input, uint8_t output[NUM_BYTES]);
/**
 * @brief This converts the encoded @input array to a data packet
 * @param input: Encoded array of @ARRAY_SIZE bytes
 * @retval A data packet
 */
static MOTOR_DRIVER_TX_RL_DATA_PACKET arrayToDataPacket(uint8_t input[NUM_BYTES]);
//Public Variables
/**
 * @brief Fill this out prior to calling SendData()
 */
MOTOR_DRIVER_TX_RL_DATA_PACKET txData;
};

/*****************RX Classes*****************/
struct MOTOR_DRIVER_RX_FRAME_0_DATA_PACKET
{
	uint16_t motorRPM;
};

class MOTOR_DRIVER_RX_FRAME_0 final: public SUBSYSTEM_DATA_MODULE_TEMPLATE_INTERFACE<MOTOR_DRIVER_RX_FRAME_0, MOTOR_DRIVER_RX_FRAME_0_DATA_PACKET>
{
public:
//Constructors
MOTOR_DRIVER_RX_FRAME_0();
//Public Constants
static constexpr uint8_t NUM_BYTES = 8;
//Public Function Prototypes
/**
 * @brief This function converts @input to fill the encoded @output array
 * @param input: Data to be converted
 * @param output: Array that should be allocated at least @ARRAY_SIZE bytes
 */
static void dataPacketToArray(MOTOR_DRIVER_RX_FRAME_0_DATA_PACKET input, uint8_t output[NUM_BYTES]);
/**
 * @brief This converts the encoded @input array to a data packet
 * @param input: Encoded array of @ARRAY_SIZE bytes
 * @retval A data packet
 */
static MOTOR_DRIVER_RX_FRAME_0_DATA_PACKET arrayToDataPacket(uint8_t input[NUM_BYTES]);
//Public Variables
/**
 * @brief This variable does nothing
 */
MOTOR_DRIVER_RX_FRAME_0_DATA_PACKET txData;
};

struct MOTOR_DRIVER_RX_FRAME_2_DATA_PACKET
{
	bool adSensorError;
	bool powerSystemError;
	bool motorSystemError;
	uint8_t overHeatLevel;
};

class MOTOR_DRIVER_RX_FRAME_2 final: public SUBSYSTEM_DATA_MODULE_TEMPLATE_INTERFACE<MOTOR_DRIVER_RX_FRAME_2, MOTOR_DRIVER_RX_FRAME_2_DATA_PACKET>
{
public:
//Constructors
MOTOR_DRIVER_RX_FRAME_2();
//Public Constants
static constexpr uint8_t NUM_BYTES = 5;
//Public Function Prototypes
/**
 * @brief This function converts @input to fill the encoded @output array
 * @param input: Data to be converted
 * @param output: Array that should be allocated at least @ARRAY_SIZE bytes
 */
static void dataPacketToArray(MOTOR_DRIVER_RX_FRAME_2_DATA_PACKET input, uint8_t output[NUM_BYTES]);
/**
 * @brief This converts the encoded @input array to a data packet
 * @param input: Encoded array of @ARRAY_SIZE bytes
 * @retval A data packet
 */
static MOTOR_DRIVER_RX_FRAME_2_DATA_PACKET arrayToDataPacket(uint8_t input[NUM_BYTES]);
//Public Variables
/**
 * @brief This variable does nothing
 */
MOTOR_DRIVER_RX_FRAME_2_DATA_PACKET txData;
};

#endif //End Header Guard