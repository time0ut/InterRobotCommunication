#include <rtt/Component.hpp>

#include <Ivy/ivy.h>
#include <Ivy/ivyloop.h>

#include <map>
#include <sstream>

#include "formation.hpp"

// Formation constants :
 // Send position every 10 calls to "updateHook"
#define  REFRESH_PERIOD 100
 // Number of followers needed by the leader to pilot the formation
#define  NB_FOLLOWERS 1		// For now, simulate with just 1 follower

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
void doDemoCallback (IvyClientPtr app, void *data, int cargc, char **argv);
void stopDemoCallback (IvyClientPtr app, void *data, int cargc, char **argv);

std::map <string, MsgRcvPtr> filters;
std::list <PositionLocale> steps;
std::list <int> followers;
TypeRobotRole role = FOLLOWER;
int id;

//constructor
formation::formation(const std::string& name) :
						TaskContext (name, PreOperational),
						_debugNbPeriods(0),
						c_cmdLawMoveTo ( "MoveTo" ),
						c_cmdLawRotate ( "Rotate" ),
						c_cmdLawIsRunning ( "IsCommandRunning" ),
						ip_relativePosition ( "position_local" ),
						_phase ( INITIALIZATION ),
						p_identifier ( 0 ),
						p_role ( 'N' )

{
	cout<<"component "<< COMPONENT_NAME <<" build version:"<< VERSION << "on "<< __DATE__ <<" at "<< __TIME__<<endl;

	char *domain = getenv ("IVYBUS");
	IvyInit ( name.c_str(), NULL, 0, 0, 0, 0);
	IvyStart ( domain );

	// Ports
	this->addPort( ip_relativePosition ).doc("Local position input");

	// Properties
	this->addProperty( "identifier", this->p_identifier ).doc ( "robot identifier" );
	this->addProperty( "role", this->p_role ).doc ( "robot role" );

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

	if ( role == LEADER && countdown -- <= 0 )  // Time to refresh
	{
		if ( _phase == INITIALIZATION )
		{
			if ( followers.size() >= NB_FOLLOWERS )
			{ // Sending a personal confirmation message to all chosen robots
				// Stop listening to candidates
				IvyUnbindMsg( filters.find ( "follow_yes" )->second );
				filters.erase( filters.find("follow_yes") );

				int msg_sent ( 0 );
				while ( ! followers.empty() && msg_sent < NB_FOLLOWERS )
				{
					IvySendMsg ( "FOLLOW_ME %d", followers.front() );
					followers.pop_front();
					msg_sent ++;
				}

				IvySendMsg("GOGOGO");
				// Now let's start the second phase
				_phase = FORMATION;
			}
		}
		else if ( _phase == FORMATION )
		{
			// Once formation has started, the leader can send a cancellation
			// message to the followers that were not chosen
			if ( ! followers.empty() )
			{
				while ( ! followers.empty () )
				{
					IvySendMsg ( "IGNORE_REQ %d", followers.front() );
					followers.pop_front();
				}
			}
			// Send to ivy bus _longitude & _latitude & orientation (no altitude for now)
			IvySendMsg("CONTROL %lf %lf %lf %lf", this->_relative_position.x,
						this->_relative_position.y, 0., this->_relative_position.cap );
		}
		countdown = REFRESH_PERIOD; // Restart timer
	}
	else if ( role == FOLLOWER )
	{
		p_role = 'F';
			if ( ! c_cmdLawIsRunning () ) // If the robot stopped moving
			{
				if ( ! steps.empty () ) // If there is a next step
				{
					PositionLocale wp = (PositionLocale) steps.front ();
					steps.pop_front();

					if ( wp.cap != _relative_position.cap )
					{
						c_cmdLawRotate ( wp.cap - _relative_position.cap );

						// Wait for movement to be executed:
						while ( c_cmdLawIsRunning () );
					}

					// Send command law if necessary
					if ( wp.x != this->_relative_position.x || wp.y != this->_relative_position.y )
					{
						IvySendMsg ("Calling cmd law");
						c_cmdLawMoveTo ( double (wp.x + _delta_x), double (wp.y + _delta_y), 'L');

					//	IvySendMsg ("Pos: %lf  %lf", this->_relative_position.x, this->_relative_position.y );
					}


				}
			}
	}
	// If current role is 'NORMAL', don't do anything
}


bool formation::configureHook()
{
	if ( p_role == 'L' )
	{
		role = LEADER;
	}
	else if ( p_role == 'F' )
	{
		role = FOLLOWER;
	}
	else if ( p_role == 'N' )
	{
		role = NORMAL;
	}


	if ( role == NORMAL )
	{
		filters.insert( pair<string, MsgRcvPtr> ( "follow_req",
			IvyBindMsg ( followReqCallback, 0, "^FOLLOW_REQ(.*)" )));
	}
	else if ( role == LEADER )
	{ // The very first message the leader with receive is a DO_DEMO (see specs)
		filters.insert( pair<string, MsgRcvPtr> ( "do_demo",
				IvyBindMsg ( doDemoCallback, 0, "^DO_DEMO$" )));
	}


	id = p_identifier;
	return true;
}

void formation::ivyLoop()
{
	IvyMainLoop();
}

void formation::init()
{
}

bool formation::bye(){
	IvyStop();

	return true;
}

/******************************************************************************/
/************************ INITIALIZATION PHASE CALLBACKS **********************/
/******************************************************************************/
void doDemoCallback (IvyClientPtr app, void *data, int cargc, char **argv)
{
	// Desactivate do demo request
	IvyUnbindMsg( filters.find ( "do_demo" )->second );
	filters.erase( filters.find("do_demo") );

	filters.insert( pair<string, MsgRcvPtr> ( "follow_yes",
			IvyBindMsg ( followYesCallback, 0, "^FOLLOW_YES (.*)" )));

	filters.insert( pair<string, MsgRcvPtr> ( "stop_demo",
			IvyBindMsg ( stopDemoCallback, 0, "^STOP_DEMO$" )));

	IvySendMsg("FOLLOW_REQ");

}

void followReqCallback (IvyClientPtr app, void *data, int cargc, char **argv)
{
	if ( role == NORMAL ) // If the robot isn't already in a formation
	{
		IvySendMsg ( "FOLLOW_YES %d", id ); // I'm in!

		// Desactivate follow requests
		IvyUnbindMsg( filters.find ( "follow_req" )->second );
		filters.erase( filters.find("follow_req") );


		// Wait for a confirmation or a cancellation
		filters.insert( pair<string, MsgRcvPtr> ( "follow_me",
				IvyBindMsg ( followMeCallback, 0, "^FOLLOW_ME (.*)" )));
		filters.insert( pair<string, MsgRcvPtr> ( "ignore_req",
				IvyBindMsg ( ignoreReqCallback, 0, "^IGNORE_REQ (.*)" )));
	}
}

void followYesCallback (IvyClientPtr app, void *data, int cargc, char **argv)
{
	char *args;
	char *token;
    char *saveptr;
    char delim = ' ';
    int f_id;

    args = argv[0];

    token = strtok_r( args, &delim, &saveptr );
    if (token != NULL) f_id = ::atoi(token);

    followers.push_back( f_id );
}

void followMeCallback (IvyClientPtr app, void *data, int cargc, char **argv)
{
	char *args;
	char *token;
    char *saveptr;
    char delim = ' ';
    int f_id;

    args = argv[0];

    // Read identifier of the addressee
    token = strtok_r( args, &delim, &saveptr );
    if (token != NULL) f_id = ::atoi(token);

    IvySendMsg("f_id: %d     id:  %d     role: %c", f_id, id, role);
    // Am I concerned by this message ?
    if ( f_id == id )
    {
    	IvySendMsg("I'm a follower %d", id);
		// TODO:  Read fields (delta, start pos) + call command law

		// Now wait for a start notification
		filters.insert( pair<string, MsgRcvPtr> ( "gogogo",
				IvyBindMsg ( gogogoCallback, 0, "^GOGOGO$" )));

		// Ignore next follow requests or cancellations
		IvyUnbindMsg( filters.find ( "follow_me" )->second );
		IvyUnbindMsg( filters.find ( "ignore_req" )->second );
		filters.erase( filters.find("follow_me") );
		filters.erase( filters.find("ignore_req") );

		role = FOLLOWER;
    }
}

void ignoreReqCallback (IvyClientPtr app, void *data, int cargc, char **argv)
{
	char *args;
	char *token;
    char *saveptr;
    char delim = ' ';
    int f_id;

    args = argv[0];

    token = strtok_r( args, &delim, &saveptr );
    if (token != NULL) f_id = ::atoi(token);

    // Am I concerned by this message ?
    if ( f_id == id )
    {

		IvyUnbindMsg( filters.find ( "follow_me" )->second );
		IvyUnbindMsg( filters.find ( "ignore_req" )->second );
		filters.erase( filters.find("follow_me") );
		filters.erase( filters.find("ignore_req") );

		// Start again to listen to follow requests
		filters.insert( pair<string, MsgRcvPtr> ( "follow_req",
				IvyBindMsg ( followReqCallback, 0, "^FOLLOW_REQ(.*)" )));
    }

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
	role = NORMAL;
}

void stopDemoCallback (IvyClientPtr app, void *data, int argc, char **argv)
{
	IvyUnbindMsg( filters.find ( "stop_demo" )->second );
	filters.erase ( filters.find ("stop_demo" ));
	IvySendMsg ("LEAVE_ME_ALONE");
}

} // namespace

// Define component as deployable in the library
ORO_CREATE_COMPONENT_TYPE()
ORO_LIST_COMPONENT_TYPE(TTRK::formation)
