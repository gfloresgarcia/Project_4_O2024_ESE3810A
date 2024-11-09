#ifndef LIN1D3_DRIVER_CFG_H_
#define LIN1D3_DRIVER_CFG_H_

#define lin1d3_max_supported_messages_per_node_cfg_d (16)


//Definitions_to bits/pins in register
typedef enum
{
	BIT0,
	BIT1,
	BIT2,
	BIT3,
	BIT4,
	BIT5,
	BIT6,
	BIT7,
	BIT8,
	BIT9,
	BIT10,
	BIT11,
	BIT12,
	BIT13,
	BIT14,
	BIT15,
	BIT16,
	BIT17,
	BIT18,
	BIT19,
	BIT20,
	BIT21,
	BIT22,
	BIT23,
	BIT24,
	BIT25,
	BIT26,
	BIT27,
	BIT28,
	BIT29,
	BIT30,
	BIT31
} BitsType;

typedef enum {FALSE, TRUE} boolean_t;
typedef enum {BIT_OFF, BIT_ON} BIT_ON_OFF_Type; //Bits for turn_on/off any bit

#endif /* LIN1D3_DRIVER_CFG_H_ */
