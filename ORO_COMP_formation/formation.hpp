#ifndef _TTRK_FORMATION_COMPONENT_H_
#define _TTRK_FORMATION_COMPONENT_H_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/OperationCaller.hpp>
#include <list>

#include "State.h"
#include "MavDataTypes.h"

namespace TTRK {

#define COMPONENT_NAME "formation"
#define VERSION 0.1
#define DEBUG this->_debugNbPeriods!=0
#define perforperiodnum 200
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
		FORMATION,		/**< the component is in the formation phase (the formation is deployed). */
	//	NONE
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
		 * Formation component constructor.
		 *
		 * @param name is the name of this component instance (given in the Orocos deployer)
		 */
		formation(const std::string& name);

		/**
		 * Updates component state.
		 *
		 * Basically:
		 * - The leader broadcasts it's position on the Ivy bus.
		 * - The followers send leader position to their command law so as to copy its movements.
		 * - The robots that are neither leader nor followers don't do anything.
		 */
		void updateHook();

		/**
		 * Configures the component.
		 * The robot starts listening on Ivy bus to messages telling it that
		 * it is now the leader of a formation, or requesting it as a follower.
		 * @return true always
		 */
		bool configureHook();


	protected:
		/**
		 * Formation component initialization function.
		 *
		 * It is called from the constructor, but is empty for now.
		 */
		void init();

		/**
		 * Formation component stop function.
		 *
		 * This function has to be called when the component isn't needed anymore.
		 * It closes the connexion to the Ivy bus.
		 */
		bool bye();

		/**
		 * Call this to start the Ivy bus (necessary for the robot to take part
		 * into the communications with the other robots).
		 * Contains an infinite loop (doesn't return).
		 */
		void ivyLoop();


		/**
		 * Calls the MoveTo function of the command law.
		 * Format: MoveTo(x, y, repere) where repere = 'L' (local) or 'G' (GPS)
		 */
		RTT::OperationCaller<bool(double,double,char) > c_cmdLawMoveTo;

		/**
		 * Calls the Rotate function of the command law.
		 * Format: Rotate (angle)
		 */
		RTT::OperationCaller<bool(double) > c_cmdLawRotate;

		/**
		 * Calls the IsRunning function of the command law.
		 *  Allows to know if it is currently working or ready to be used.
		 */
		RTT::OperationCaller<bool() > c_cmdLawIsRunning;

		// Ports
		/**
		 * Relative position read from the output of the SLAM.
		 */
		RTT::InputPort<PositionLocale> ip_relativePosition;

		/**
		 * Joystick format position sent to the command law.
		 */
		RTT::OutputPort<TypeInfosJoystickMavLink> op_joystick;

		// Properties
		/**
		 * Robot ID number. This has to be set in the deployer file.
		 */
		int p_identifier;

		/**
		 * For debug
		 * @see other components.
		 */
		int _debugNbPeriods;

		// Member data
		/**
		 * Current local position of the robot.
		 */
		PositionLocale _relative_position;
//		double _delta_x, _delta_y;
		//for network performance evaluation
		uint64_t time[perforperiodnum][2];
		int seq=0;//sequence number of the message
		int echoid=0;
	};

}

#endif
