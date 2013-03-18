#include <rtt/Component.hpp>

#include <Ivy/ivy.h>
#include <Ivy/ivyloop.h>

#include <map>
#include <sstream>

#include "formation.hpp"

#define  REFRESH_PERIOD 10 // Send position every 10 calls to "updateHook"


namespace TTRK {

using namespace std;
using namespace RTT;


void newWPCallback (IvyClientPtr app, void *data, int argc, char **argv);
void endOfFormationCallback (IvyClientPtr app, void *data, int argc, char **argv);
void followMeCallback (IvyClientPtr app, void *data, int cargc, char **argv);
void followReqCallback (IvyClientPtr app, void *data, int cargc, char **argv);
void ignoreReqCallback (IvyClientPtr app, void *data, int cargc, char **argv);
void followYesCallback (IvyClientPtr app, void *data, int cargc, char **argv);
void gogogoCallback (IvyClientPtr app, void *data, int cargc, char **argv);

std::map <string, MsgRcvPtr> filters;
std::list <PositionLocale> steps;
std::list <string> followers;
TypeRobotRole _current_role = FOLLOWER;

//constructor
formation::formation(const std::string& name) :
						TaskContext (name, PreOperational),
						_debugNbPeriods(0),
						c_cmdLawMoveTo ( "MoveTo" ),
						c_cmdLawRotate ( "Rotate" ),
						c_cmdLawIsRunning ( "IsCommandRunning" ),
						ip_relativePosition ( "position_local" ),
						_phase ( INITIALIZATION )

{
	cout<<"component "<< COMPONENT_NAME <<" build version:"<< VERSION << "on "<< __DATE__ <<" at "<< __TIME__<<endl;

	char *domain = getenv ("IVYBUS");
	IvyInit ( name.c_str(), NULL, 0, 0, 0, 0);
	IvyStart ( domain );

	if ( _current_role == FOLLOWER )
		filters.insert( pair<string, MsgRcvPtr> ( "follow_req",
			IvyBindMsg ( followReqCallback, 0, "^FOLLOW_REQ(.*)" )));

	// Ports
	this->addPort( ip_relativePosition ).doc("Local position input");

	// Properties

	// Operations
	this->provides()->addOperation("IvyLoop", &formation::ivyLoop, this);
	this->provides()->addOperation("bye", &formation::bye, this);

	// Service requester
	this->requires()->addOperationCaller(this->c_cmdLawMoveTo);
	this->requires()->addOperationCaller(this->c_cmdLawRotate);
	this->requires()->addOperationCaller(this->c_cmdLawIsRunning);

	// TMP debug
	_relative_position.x = 3.;

	this->init();
}

void formation::updateHook()
{
	static int countdown = REFRESH_PERIOD;

	// Update relative position
	ip_relativePosition.read( this->_relative_position );


	if ( _current_role == LEADER && countdown -- <= 0 )  // Time to refresh
	{
		if ( _phase == INITIALIZATION )
		{
			// TODO: add formation initialization (send messages to all the followers)
		}
		else if ( _phase == FORMATION )
		{
			// Send to ivy bus _longitude & _latitude & orientation (no altitude for now)
			IvySendMsg("CONTROL %lf %lf %lf %lf", this->_relative_position.x,
						this->_relative_position.y, 0., this->_relative_position.cap );
		}
		countdown = REFRESH_PERIOD; // Restart timer
	}
	else if ( _current_role == FOLLOWER )
	{
			if ( ! c_cmdLawIsRunning () ) // If the robot stopped moving
			{
				if ( ! steps.empty () ) // If there is a next step
				{
				//	IvySendMsg ("I have a step!!");
					PositionLocale wp = (PositionLocale) steps.front ();
					steps.pop_front();

					// Send command law if necessary
					if ( wp.x != this->_relative_position.x || wp.y != this->_relative_position.y )
					{
						IvySendMsg ("Calling cmd law");
						c_cmdLawMoveTo ( double (wp.x + _delta_x), double (wp.y + _delta_y), 'L');
					//	c_cmdLawMoveTo ( 1, 1, 'L');
						// Wait for movement to be executed
						while ( c_cmdLawIsRunning () );
					//	IvySendMsg ("Pos: %lf  %lf", this->_relative_position.x, this->_relative_position.y );
					}

					if ( wp.cap != _relative_position.cap )
					{
						c_cmdLawRotate ( wp.cap );
					}
				}
			}
	}
	// If current role is 'NORMAL', don't do anything

}

void formation::ivyLoop()
{
	IvyMainLoop();
}

void formation::init()
{

}

bool formation::bye(){
	cout << "johnnnnnnnnnnnnnnn";
	IvyStop();
	// TODO: remove ports and provided operations
	return true;
}

/******************************************************************************/
/************************ INITIALIZATION PHASE CALLBACKS **********************/
/******************************************************************************/
void followReqCallback (IvyClientPtr app, void *data, int cargc, char **argv)
{
//	if ( _current_role == NORMAL ) // If the robot isn't already in a formation
//	{
		IvySendMsg ( "FOLLOW_YES blablabla" ); // I'm in!

		// Desactivate follow requests
		IvyUnbindMsg( filters.find ( "follow_req" )->second );
		filters.erase( filters.find("follow_req") );


		// Wait for a confirmation or a cancellation
		filters.insert( pair<string, MsgRcvPtr> ( "follow_me",
				IvyBindMsg ( followMeCallback, 0, "^FOLLOW_ME(.*)" )));
		filters.insert( pair<string, MsgRcvPtr> ( "ignore_req",
				IvyBindMsg ( ignoreReqCallback, 0, "^IGNORE_REQ$" )));
//	}
}

void followYesCallback (IvyClientPtr app, void *data, int cargc, char **argv)
{
	// TODO: Add follower to the list
}

void followMeCallback (IvyClientPtr app, void *data, int cargc, char **argv)
{
	// TODO:  Read fields (delta, start pos) + call command law

	// Now wait for a start notification
	filters.insert( pair<string, MsgRcvPtr> ( "gogogo",
			IvyBindMsg ( gogogoCallback, 0, "^GOGOGO$" )));

	// Ignore next follow requests or cancellations
	IvyUnbindMsg( filters.find ( "follow_me" )->second );
	IvyUnbindMsg( filters.find ( "ignore_req" )->second );
	filters.erase( filters.find("follow_me") );
	filters.erase( filters.find("ignore_req") );

	_current_role = FOLLOWER;
}

void ignoreReqCallback (IvyClientPtr app, void *data, int cargc, char **argv)
{
	IvyUnbindMsg( filters.find ( "follow_me" )->second );
	IvyUnbindMsg( filters.find ( "ignore_req" )->second );
	filters.erase( filters.find("follow_me") );
	filters.erase( filters.find("ignore_req") );

	// Start again to listen to follow requests
	filters.insert( pair<string, MsgRcvPtr> ( "follow_req",
			IvyBindMsg ( followReqCallback, 0, "^FOLLOW_REQ(.*)" )));

}

void gogogoCallback (IvyClientPtr app, void *data, int cargc, char **argv)
{
	IvyUnbindMsg( filters.find ( "gogogo" )->second );
	filters.erase ( filters.find ("gogogo" ));

	// Set filters in order to receive control messages and the end of formation
	// notification
	filters.insert( pair<string, MsgRcvPtr> ( "control",
			IvyBindMsg ( newWPCallback, 0, "^CONTROL (.*)" )));
	filters.insert( pair<string, MsgRcvPtr> ( "leave_me_alone",
			IvyBindMsg ( endOfFormationCallback, 0, "^LEAVE_ME_ALONE$" )));
}

/******************************************************************************/
/************************ FORMATION PHASE CALLBACKS ***************************/
/******************************************************************************/

//void formation::newWPCallback (IvyApplication *app, void *data, int argc, char **argv)
void newWPCallback (IvyClientPtr app, void *data, int cargc, char **argv)
{
	char *args;
	char *token;
    char *saveptr;
    char delim = ' ';
    PositionLocale goal;

    args = argv[0];

    token = strtok_r(args, &delim, &saveptr);
    if (token != NULL) goal.x = ::atof(token);
    token = strtok_r(NULL, &delim, &saveptr);
    if (token != NULL) goal.y = ::atof(token);
    token = strtok_r(NULL, &delim, &saveptr);
    // For now, just ignore altitude...
    //if (token != NULL) XXX = ::atof(token);
    token = strtok_r(NULL, &delim, &saveptr);
    if (token != NULL) goal.cap = ::atof(token);
    steps.push_back( goal );
    // TMP (debug)
	IvySendMsg ("Position to reach: %lf %lf, cap: %lf", goal.x, goal.y, goal.cap);
}

//void formation::endOfFormationCallback (IvyApplication *app, void *data, int argc, char **argv)
void endOfFormationCallback (IvyClientPtr app, void *data, int argc, char **argv)
{
	// Unsubscribe (desactivate the filters)
	IvyUnbindMsg( filters.find ( "control" )->second );
	IvyUnbindMsg( filters.find ( "leave_me_alone" )->second );
	filters.erase ( filters.find ("control" ));
	filters.erase ( filters.find ("leave_me_alone" ));

	// Start again listening to follow request from leaders
	filters.insert( pair<string, MsgRcvPtr> ( "follow_req",
			IvyBindMsg ( followReqCallback, 0, "^FOLLOW_REQ(.*)" )));

	// Back to normal mode
	_current_role = NORMAL;
}

} // namespace

// Define component as deployable in the library
ORO_CREATE_COMPONENT_TYPE()
ORO_LIST_COMPONENT_TYPE(TTRK::formation)
