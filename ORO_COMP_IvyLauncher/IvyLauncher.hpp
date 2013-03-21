#ifndef _TTRK_IVY_LAUNCHER_COMPONENT_H_
#define _TTRK_IVY_LAUNCHER_COMPONENT_H_

#include <rtt/TaskContext.hpp>
#include <rtt/OperationCaller.hpp>
#include <rtt/Component.hpp>


namespace TTRK {

#define COMPONENT_NAME "ivylauncher"
#define VERSION 0.1
#define DEBUG this->_debugNbPeriods!=0

	//! IvyLauncher class

	/**
	 *
	 */
	class IvyLauncher : public RTT::TaskContext
	{
	public:
		/**
		 * IvyLauncher component constructor.
		 *
		 * @param name is the name of this component instance (given in the Orocos deployer)
		 */
		IvyLauncher(const std::string& name);

		/**
		 * Updates component state.
		 *
		 */
		void updateHook();

		/**
		 * Configures the component.
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
		 * Calls the IvyLoop service of the Formation component (start Ivy bus).
		 * This function doesn't return (infinite loop).
		 */
		RTT::OperationCaller<void() > c_ivyStart;

	};

}

#endif
