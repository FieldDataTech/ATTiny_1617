/*
 * initTinyBoard.h
 *
 * Created: 2/17/2018 9:41:35 PM
 *  Author: doug
 */ 


#ifndef INITTINYBOARD_H_
#define INITTINYBOARD_H_

#include <compiler.h>
#include "initTinyBoard.h"

enum port_pull_mode {
	PORT_PULL_OFF,
	PORT_PULL_UP,
};

enum port_dir {
	PORT_DIR_IN,
	PORT_DIR_OUT,
	PORT_DIR_OFF,
};




#endif /* INITTINYBOARD_H_ */