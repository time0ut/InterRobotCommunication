#ifndef _TTRK_FORMATION_COMPONENT_H_
#define _TTRK_FORMATION_COMPONENT_H_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/OperationCaller.hpp>
#include <list>

#include "State.h"

namespace TTRK {

#define COMPONENT_NAME "formation"
#define VERSION 0.1
#define DEBUG this->_debugNbPeriods!=0

	enum TypeRobotRole {
		LEADER,		// Leader of the current mission => acts normally & broadcasts its position
		FOLLOWER, 	// Following the leader => listens to leader's position and tries to reach it
		NORMAL  	// Not taking part into any mission => component inactive
	};

	enum MissionPhase {
		INITIALIZATION,
		FORMATION
	};

	class formation : public RTT::TaskContext
	{
	public:

		formation(const std::string& name);
		void updateHook();


	protected:
		void init();
		bool bye();
		void ivyLoop();

		RTT::OperationCaller<bool(double,double,char) > c_cmdLawMoveTo;
		RTT::OperationCaller<bool(double) > c_cmdLawRotate;
		RTT::OperationCaller<bool() > c_cmdLawIsRunning;
		RTT::InputPort<PositionLocale> ip_relativePosition;

		int  _debugNbPeriods;
		PositionLocale _relative_position;
		double _delta_x, _delta_y;
		MissionPhase _phase;
	};

}

#endif
