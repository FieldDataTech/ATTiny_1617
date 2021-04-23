/*
 * initTinyBoard.h
 *
 * Created: 2/17/2018 9:42:09 PM
 *  Author: doug
 */


#ifndef INITTINYBOARD_H_
#define INITTINYBOARD_H_

void tinyInitBoard(void);

static inline void LED_set_dir(const enum port_dir dir)
{
	PORTC_set_pin_dir(2, dir);
}
static inline void LED_set_pull_mode(const enum port_pull_mode pull_mode)
{
	PORTC_set_pin_pull_mode(2, pull_mode);
}
static inline void TPA1_set_dir(const enum port_dir dir)
{
	PORTA_set_pin_dir(1, dir);
}

static inline void TPA2_set_dir(const enum port_dir dir)
{
	PORTA_set_pin_dir(2, dir);
}
static inline void TPA3_set_dir(const enum port_dir dir)
{
	PORTA_set_pin_dir(3, dir);
}
static inline void TPA4_set_dir(const enum port_dir dir)
{
	PORTA_set_pin_dir(4, dir);
}

static inline void TPB5_set_dir(const enum port_dir dir)
{
	PORTB_set_pin_dir(5, dir);
}
static inline void TPB7_set_dir(const enum port_dir dir)
{
	PORTB_set_pin_dir(7, dir);
}
static inline void TPA7_set_dir(const enum port_dir dir)
{
	PORTA_set_pin_dir(7, dir);
}
static inline void TPC5_set_dir(const enum port_dir dir)
{
	PORTC_set_pin_dir(5, dir);
}
static inline void TPB6_set_dir(const enum port_dir dir)
{
	PORTB_set_pin_dir(6, dir);
}

static inline void TPA5_set_dir(const enum port_dir dir)
{
	PORTA_set_pin_dir(5, dir);
}
static inline void TPA6_set_dir(const enum port_dir dir)
{
	PORTA_set_pin_dir(6, dir);
}
static inline void TPC2_set_dir(const enum port_dir dir)
{
	PORTC_set_pin_dir(2, dir);
}

static inline void TPB1_set_dir(const enum port_dir dir)
{
	PORTB_set_pin_dir(1, dir);
}

static inline void TPB2_set_dir(const enum port_dir dir)
{
	PORTB_set_pin_dir(2, dir);
}

static inline void TPC1_set_dir(const enum port_dir dir)
{
	PORTC_set_pin_dir(1, dir);
}

static inline void MOTION_set_dir(const enum port_dir dir)
{
	PORTA_set_pin_dir(1, dir);
}

static inline void MOTION_set_pull(const enum port_dir pull)
{
	PORTA_set_pin_pull_mode(1, pull);
}

static inline void TPA1_set_pull_mode(const enum port_pull_mode pull_mode)
{
	PORTA_set_pin_pull_mode(1, pull_mode);
}
static inline void TPA3_set_pull_mode(const enum port_pull_mode pull_mode)
{
	PORTA_set_pin_pull_mode(3, pull_mode);
}
static inline void TPA4_set_pull_mode(const enum port_pull_mode pull_mode)
{
	PORTA_set_pin_pull_mode(4, pull_mode);
}
static inline void TPA6_set_pull_mode(const enum port_pull_mode pull_mode)
{
	PORTA_set_pin_pull_mode(6, pull_mode);
}
static inline void TPA5_set_pull_mode(const enum port_pull_mode pull_mode)
{
	PORTA_set_pin_pull_mode(5, pull_mode);
}

static inline void TPA2_set_pull_mode(const enum port_pull_mode pull_mode)
{
	PORTA_set_pin_pull_mode(2, pull_mode);
}

static inline void TPB5_set_pull_mode(const enum port_pull_mode pull_mode)
{
	PORTB_set_pin_pull_mode(5, pull_mode);
}
static inline void TPB6_set_pull_mode(const enum port_pull_mode pull_mode)
{
	PORTB_set_pin_pull_mode(6, pull_mode);
}
static inline void TPB4_set_pull_mode(const enum port_pull_mode pull_mode)
{
	PORTB_set_pin_pull_mode(4, pull_mode);
}
static inline void TPC1_set_pull_mode(const enum port_pull_mode pull_mode)
{
	PORTC_set_pin_pull_mode(1, pull_mode);
}
static inline void TPC2_set_pull_mode(const enum port_pull_mode pull_mode)
{
	PORTC_set_pin_pull_mode(2, pull_mode);
}
static inline void TPB1_set_pull_mode(const enum port_pull_mode pull_mode)
{
	PORTB_set_pin_pull_mode(1, pull_mode);
}

static inline void TPB2_set_pull_mode(const enum port_pull_mode pull_mode)
{
	PORTB_set_pin_pull_mode(2, pull_mode);
}

static inline bool TPB6_get_level()
{
	return PORTB_get_pin_level(6);
}

static inline void TPB5_set_level(const bool level)
{
	PORTB_set_pin_level(5, level);
}
static inline void TPB7_set_level(const bool level)
{
	PORTB_set_pin_level(7, level);
}

static inline void TPA1_set_level(const bool level)
{
	PORTA_set_pin_level(1, level);
}
static inline void TPA2_set_level(const bool level)
{
	PORTA_set_pin_level(2, level);
}
static inline void TPA4_set_level(const bool level)
{
	PORTA_set_pin_level(4, level);
}
static inline void TPA5_set_level(const bool level)
{
	PORTA_set_pin_level(5, level);
}


static inline void TPB6_set_isc(const PORT_ISC_t isc)
{
	PORTB_pin_set_isc(6, isc);
}
static inline void TPA1_set_isc(const PORT_ISC_t isc)
{
	PORTA_pin_set_isc(1, isc);
}
static inline void TPA5_set_isc(const PORT_ISC_t isc)
{
	PORTA_pin_set_isc(5, isc);
}
static inline void TPA7_set_isc(const PORT_ISC_t isc)
{
	PORTA_pin_set_isc(7, isc);
}
static inline void TPC1_set_isc(const PORT_ISC_t isc)
{
	PORTC_pin_set_isc(1, isc);
}
static inline void TPC3_set_dir(const enum port_dir dir)
{
	PORTC_set_pin_dir(3, dir);
}
static inline void TPC0_set_dir(const enum port_dir dir)
{
	PORTC_set_pin_dir(0, dir);
}
static inline void TPC4_set_dir(const enum port_dir dir)
{
	PORTC_set_pin_dir(4, dir);
}
static inline void TPB0_set_dir(const enum port_dir dir)
{
	PORTB_set_pin_dir(0, dir);
}
static inline void TPC3_set_pull_mode(const enum port_pull_mode pull_mode)
{
	PORTC_set_pin_pull_mode(3, pull_mode);
}
static inline void TPC4_set_pull_mode(const enum port_pull_mode pull_mode)
{
	PORTC_set_pin_pull_mode(4, pull_mode);
}
static inline void TPC0_set_pull_mode(const enum port_pull_mode pull_mode)
{
	PORTC_set_pin_pull_mode(0, pull_mode);
}
static inline void TPC3_set_isc(const PORT_ISC_t isc)
{
	PORTC_pin_set_isc(3, isc);
}
static inline void TPB0_set_pull_mode(const enum port_pull_mode pull_mode)
{
	PORTB_set_pin_pull_mode(0, pull_mode);
}
static inline void TPB3_set_dir(const enum port_dir dir)
{
	PORTB_set_pin_dir(3, dir);
}
static inline void TPB4_set_dir(const enum port_dir dir)
{
	PORTB_set_pin_dir(4, dir);
}
static inline void TPB3_set_pull_mode(const enum port_pull_mode pull_mode)
{
	PORTB_set_pin_pull_mode(3, pull_mode);
}

static inline void TPB0_set_level(const bool level)
{
	PORTB_set_pin_level(0, level);
}
static inline void TPB1_set_level(const bool level)
{
	PORTB_set_pin_level(1, level);
}
static inline void TPB2_set_level(const bool level)
{
	PORTB_set_pin_level(2, level);
}
static inline void TPA7_set_pull_mode(const enum port_pull_mode pull_mode)
{
	PORTA_set_pin_pull_mode(7, pull_mode);
}
static inline void TPB7_set_pull_mode(const enum port_pull_mode pull_mode)
{
	PORTB_set_pin_pull_mode(7, pull_mode);
}
static inline void TPC5_set_pull_mode(const enum port_pull_mode pull_mode)
{
	PORTC_set_pin_pull_mode(5, pull_mode);
}
static inline void TPA0_set_pull_mode(const enum port_pull_mode pull_mode)
{
	PORTA_set_pin_pull_mode(0, pull_mode);
}
static inline void TPA0_set_dir(const enum port_dir dir)
{
	PORTA_set_pin_dir(0, dir);
}

#define sleep_enable()             \
do {                               \
  _SLEEP_CONTROL_REG |= (uint8_t)_SLEEP_ENABLE_MASK;   \
} while(0)

#define sleep_cpu()                              \
do {                                             \
	__asm__ __volatile__ ( "sleep" "\n\t" :: );    \
} while(0)

    #define _SLEEP_CONTROL_REG  SLPCTRL_CTRLA
    #define _SLEEP_ENABLE_MASK  SLPCTRL_SEN_bm
    #define _SLEEP_SMODE_GROUP_MASK  SLPCTRL_SMODE_gm

#endif /* INITTINYBOARD_H_ */