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
#define perforperiodnum 100//number of periods during which we store the time of broadcast and reception of messages

	/**
	 * Current role of the robot
	 *
	 * This types allows to specify the role currently played by the robot.
	 */
	enum TypeRobotRole {
		LEADER,		/**< the robot is the leader of the current mission. */
		FOLLOWER, 	/**< the robot is involved in a mission. It is following a leader. */
		NORMAL  	/**< the robot doesn't take part into any mission. */
	};

	/**
	 * State of the mission
	 *
	 * This types allows to indicate the current phase of the formation component.
	 */
	enum MissionPhase {
		INITIALIZATION,	/**< the component is in the initialization phase (configuration of the formation). */
		FORMATION		/**< the component is in the formation phase (the formation is deployed). */
	};


	//! Formation class

	/**
	 * The formation class is an Orocos component allowing the deployment of a formation of robots
	 * (N "followers" following a "leader" robot).
	 * The communications between robots are based on Ivy software bus.
	 */
	class formation : public RTT::TaskContext
	{
	public:
		/**
		 *
		 */
		formation(const std::string& name);

		/**
		 *
		 */
		void updateHook();

		/**
		 *
		 */
		bool configureHook();


	protected:
		void init();
		bool bye();
		void ivyLoop();

		// Operations
		RTT::OperationCaller<bool(double,double,char) > c_cmdLawMoveTo;
		RTT::OperationCaller<bool(double) > c_cmdLawRotate;
		RTT::OperationCaller<bool() > c_cmdLawIsRunning;

		// Ports
		RTT::InputPort<PositionLocale> ip_relativePosition;

		// Properties
		int p_identifier; // robot identifier
		char p_role;
		int _debugNbPeriods;

		// Member data
		PositionLocale _relative_position;
		double _delta_x, _delta_y;
		MissionPhase _phase;

		//for network performance evaluation
		uint64_t time[perforperiodnum][2];
		int seq=0;//sequence number of the message
		int echoid=0;

	};

}

#endif
