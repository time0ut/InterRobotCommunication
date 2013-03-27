#ifndef _TTRK_FORMATION_COMPONENT_H_
#define _TTRK_FORMATION_COMPONENT_H_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/OperationCaller.hpp>
#include <list>
#include <string>
#include "timeMeasurement.hpp"
#include "State.h"
#include "MavDataTypes.h"
#include "CICAS_UGV.h"


namespace TTRK {

#define COMPONENT_NAME "formation"
#define VERSION 0.1

TimeMeasurement 	*_timeMeasurement;
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
		TEST          /**< the component is in the performance test phase*/

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
		 * - The leader broadcasts the commands sent to its CICAS calculator (CICAS_UGV_TC parameters) on the Ivy bus.
		 * - The followers send these values to their CICAS calculator so as to copy its movements.
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
		 * It closes the connection to the Ivy bus.
		 */
		bool bye();

		/**
		 * Call this to start the Ivy bus (necessary for the robot to take part
		 * into the communications with the other robots).
		 * Contains an infinite loop (doesn't return).
		 */
		void ivyLoop();

		/**
		 * Call this to broadcast a message through the Ivy bus.
		 *
		 * @param msg is the message to send through the Ivy bus
		 */
		void ivySendMsg( std::string msg );

		////////////////////////// Ports
		/**
		 * TC values sent by the command law to the CICAS calculator (in the leader environment)
		 */
		RTT::InputPort<CICAS_UGV_TC> ip_cicas_tc;

		/**
		 * TC sent to the CICAS calculator (in the follower environment)
		 */
		RTT::OutputPort<CICAS_UGV_TC> op_cicas_tc;

		////////////////////////// Properties
		/**
		 * Robot ID number. This has to be set in the deployer file.
		 */
		int p_idnumber;

		/**
		 * The leader broadcasts on Ivy bus the TC sent by its command law to its CICAS every
		 * p_refresh_period calls to updateHook.
		 */
		int p_refresh_period;

		/**
		 * desired number of followers in the formation.
		 */
		int p_nb_followers;

		////////////////////////// Data member
		/**
		 * TC values, CICAS format
		 */
		CICAS_UGV_TC	_cicas_tc;

	};

}

#endif
