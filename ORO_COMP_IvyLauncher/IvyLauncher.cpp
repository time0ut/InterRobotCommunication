
#include "IvyLauncher.hpp"


namespace TTRK {

using namespace std;
using namespace RTT;


//constructor
IvyLauncher::IvyLauncher(const std::string& name) :
						TaskContext (name, PreOperational),
						c_ivyStart ( "IvyLoop" )

{
	cout<<"component "<< COMPONENT_NAME <<" build version:"<< VERSION << "on "<< __DATE__ <<" at "<< __TIME__<<endl;

	// Service requester
	this->requires()->addOperationCaller(this->c_ivyStart);

	this->init();
}

void IvyLauncher::updateHook()
{
	c_ivyStart();
}


bool IvyLauncher::configureHook()
{
	return true;
}

void IvyLauncher::init()
{
}

} // namespace

// Define component as deployable in the library
ORO_CREATE_COMPONENT_TYPE()
ORO_LIST_COMPONENT_TYPE(TTRK::IvyLauncher)
