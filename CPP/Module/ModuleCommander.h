/*
 * ModuleCommander.h
 *
 *  Created on: Aug 4, 2020
 *      Author: cjb88
 */

#ifndef MODULE_MODULECOMMANDER_H_
#define MODULE_MODULECOMMANDER_H_

#include "main.h"
#include "Usec.h"
#include "MsgBus/MsgType.hpp"
#include "MsgBus/MsgBus.hpp"
#include "queue.h"

namespace FC{

enum class Command{
	ModeChangeAttitude,
	ModeChangePosition,
	ModeChangeWaypoint,
	ModeChangeRTL,
	ModeChangeTakeoff,
	ModeChangeLand,

	ArmCommand,
	DisArmCommand
};

enum class CmdResult{
	Denied,
	Changed
};

typedef uint8_t command_t;

class ModuleCommander{
public:
	ModuleCommander();
	~ModuleCommander();
	void init();
	void main();

	bool sendCommand(command_t cmd);
private:
	struct ModeFlag modeFlagPub{FlightMode::ControlAttitude, ArmMode::DisArm};
	QueueHandle_t commandQueue;

	CmdResult commandHandler(command_t cmd);
	CmdResult toAttitude();
	CmdResult toPosition();
	CmdResult toArm();
	CmdResult toDisArm();
};

ModuleCommander moduleCommander;

ModuleCommander::ModuleCommander(){
	commandQueue = xQueueCreate(3, sizeof(command_t));
}
ModuleCommander::~ModuleCommander(){
	vQueueDelete(commandQueue);
}

void ModuleCommander::main(){
	command_t rcvCommand;
	if(xQueueReceive(commandQueue, &rcvCommand, portMAX_DELAY)){
		commandHandler(rcvCommand);
	}
}

bool ModuleCommander::sendCommand(command_t cmd){
	if(xQueueSendToBack(commandQueue, &cmd, 0) != pdPASS) return false;

	return true;
}

CmdResult ModuleCommander::commandHandler(rcvCommand){
	switch(rcvCommand){
	case Command::ModeChangeAttitude:
		return toAttitude();
		break;
	case Command::ModeChangePosition:
		return toPosition();
		break;
	case Command::ModeChangeWaypoint:
		break;
	case Command::ModeChangeRTL:
		break;
	case Command::ModeChangeTakeoff:
		break;
	case Command::ModeChangeLand:
		break;

	case Command::ArmCommand:
		return toArm();
		break;
	case Command::DisArmCommand:
		return toDisArm();
		break;
	}
}

CmdResult ModuleCommander::toAttitude(){
//	msgBus.setFlightMode(FlightMode::ControlAttitude);
}

CmdResult ModuleCommander::toArm(){
	struct ArmMode armMode;
	armMode.timestamp = microsecond();
	armMode.armModeType = ArmModeType::Arm;
	msgBus.setArmMode(armMode);
}

CmdResult ModuleCommander::toDisArm(){
	struct ArmMode armMode;
	armMode.timestamp = microsecond();
	armMode.armModeType = ArmModeType::DisArm;
	msgBus.setArmMode(armMode);
}

}



#endif /* MODULE_MODULECOMMANDER_H_ */
