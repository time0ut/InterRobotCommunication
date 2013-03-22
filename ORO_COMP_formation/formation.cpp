#include <rtt/Component.hpp>

#include <Ivy/ivy.h>
#include <Ivy/ivyloop.h>

#include <map>
#include <sstream>
#include <string>

#include "formation.hpp"

// Formation constants :
 // Send position every 10 calls to "updateHook"
#define  REFRESH_PERIOD 10

 // Number of followers needed by the leader to pilot the formation
#define  NB_FOLLOWERS 1		// For now, simulate with just 1 follower

// The follower own't update its orientation if it is already close from
// the desired (leader) orientation. The threshold is in degrees.
#define CAP_TRESHOLD 0.17
#define X_TRESHOLD 0.3
#define Y_TRESHOLD 0.3


namespace TTRK {

using namespace std;
using namespace RTT;

// Declaration of Ivy messages callback functions
void newWPCallback (IvyClientPtr app, void *data, int argc, char **argv);
void endOfFormationCallback (IvyClientPtr app, void *data, int argc, char **argv);
void followMeCallback (IvyClientPtr app, void *data, int cargc, char **argv);
void followReqCallback (IvyClientPtr app, void *data, int cargc, char **argv);
void ignoreReqCallback (IvyClientPtr app, void *data, int cargc, char **argv);
void followYesCallback (IvyClientPtr app, void *data, int cargc, char **argv);
void gogogoCallback (IvyClientPtr app, void *data, int cargc, char **argv);
void doDemoCallback (IvyClientPtr app, void *data, int cargc, char **argv);
void stopDemoCallback (IvyClientPtr app, void *data, int cargc, char **argv);
void inPositionCallback (IvyClientPtr app, void *data, int cargc, char **argv);
void leaderCallback (IvyClientPtr app, void *data, int cargc, char **argv);

// Desired position sent by the followers to the command law (via output port Formation::op_joystick)
TypeInfosJoystickMavLink pos;

// Ivy filters currently used by the robot (kept up-to-date in real time)
std::map <string, MsgRcvPtr> filters;

// List of successive positions the follower has to reach to follow the leader
//std::list <PositionLocale> steps;

std::list <CICAS_UGV_TC> steps;

// List of candidates to enter the formation as followers
std::list <int> candidates;

// List of followers
std::list <int> followers;

// Robot role
TypeRobotRole role = NORMAL;
// Robot ID number
int id = 0;
// Mission phase
MissionPhase phase;

// Number of followers ready to move
int nb_answers = 0;


CICAS_UGV_TC wp;

//constructor
formation::formation(const std::string& name) :
						TaskContext (name, PreOperational),
						_debugNbPeriods(0),
						c_cmdLawMoveTo ( "MoveTo" ),
						c_cmdLawRotate ( "Rotate" ),
						c_cmdLawIsRunning ( "IsCommandRunning" ),
						c_sendCommandToCICAS("sendCommand"),
						ip_relativePosition ( "position_local" ),
						ip_systemState ("state"),
						ip_cicas_tc ("cicasTC"),
						op_joystick ( "infosTcMavlink" ),
//						_phase ( INITIALIZATION ),
						p_identifier ( 0 ),
						p_refresh_period (5)

{
	cout<<"component "<< COMPONENT_NAME <<" build version:"<< VERSION << "on "<< __DATE__ <<" at "<< __TIME__<<endl;

	char *domain = getenv ("IVYBUS");
	IvyInit ( name.c_str(), NULL, 0, 0, 0, 0);
	IvyStart ( domain );

	// Ports
	this->addPort( ip_relativePosition ).doc("Local position input");
	this->addPort( ip_systemState ).doc("System State input");
	this->addPort( ip_cicas_tc ).doc("CICAS TC coming from command law");
	this->addPort( op_joystick ).doc("Output in joystick format for command law");

	// Properties
	this->addProperty( "identifier", this->p_identifier ).doc ( "robot identifier" );
	this->addProperty( "refresh_period", this->p_refresh_period ).doc ( "leader refresh period" );

	// Operations
	this->provides()->addOperation("IvyLoop", &formation::ivyLoop, this);
	this->provides()->addOperation("bye", &formation::bye, this);

	// Service requester
	this->requires()->addOperationCaller(this->c_cmdLawMoveTo);
	this->requires()->addOperationCaller(this->c_cmdLawRotate);
	this->requires()->addOperationCaller(this->c_cmdLawIsRunning);
	this->requires("ttrk")->addOperationCaller(this->c_sendCommandToCICAS);

	// TMP debug
	this->init();
}

void formation::updateHook()
{
	static int countdown = REFRESH_PERIOD;

	// Update relative position
	ip_relativePosition.read( this->_relative_position );
	ip_systemState.read ( this->_system_state );
	ip_cicas_tc.read ( this->_cicas_tc );

	if ( role == LEADER && countdown -- <= 0 )  // Time to refresh
	{
		if ( phase == INITIALIZATION )
		{
			if ( candidates.size() >= NB_FOLLOWERS )
			{ // Sending a personal confirmation message to all chosen robots
				// Stop listening to candidates
				IvyUnbindMsg( filters.find ( "follow_yes" )->second );
				filters.erase( filters.find("follow_yes") );

				filters.insert( pair<string, MsgRcvPtr> ( "in_position",
						IvyBindMsg ( inPositionCallback, 0, "^IN_POSITION$" )));

				int msg_sent ( 0 );
				while ( ! candidates.empty() && msg_sent < NB_FOLLOWERS )
				{
					IvySendMsg ( "FOLLOW_ME %d", candidates.front() );
					candidates.pop_front();
					msg_sent ++;
				}
				// Once formation has started, the leader can send a cancellation
				// message to the followers that were not chosen
				if ( ! candidates.empty() )
				{
					while ( ! candidates.empty () )
					{
						IvySendMsg ( "IGNORE_REQ %d", candidates.front() );
						candidates.pop_front();
					}
				}
				// Now let's start the second phase
				phase = FORMATION;
			}
		}
		else if ( phase == FORMATION )
		{
			// Send to ivy bus _longitude & _latitude & orientation (no altitude for now)
		//	IvySendMsg("CONTROL %.2f %.2f %.2f %.2f", this->_relative_position.x,
		//				this->_relative_position.y, 0., this->_system_state.mPsi );//this->_relative_position.cap );

			string cicasArgs = "NNCONTROL";
			for (int i = 0; i < NB_TC_CICAS_UGV; i++ )
			{
				cicasArgs += (" " + this->_cicas_tc.commands[i] );
			}
		//	IvySendMsg ( cicasArgs.c_str() );
			IvySendMsg("CONTROL %d %d %d %d %d %d %d %d %d", _cicas_tc.commands[0], _cicas_tc.commands[1], _cicas_tc.commands[2], _cicas_tc.commands[3],
					_cicas_tc.commands[4], _cicas_tc.commands[5], _cicas_tc.commands[6], _cicas_tc.commands[7], _cicas_tc.commands[8]);

		}
		countdown = p_refresh_period;//REFRESH_PERIOD; // Restart timer
	}
	else if ( role == FOLLOWER )
	{
		if ( ! c_sendCommandToCICAS.ready () ) // If the robot stopped moving
		{
			if ( ! steps.empty () ) // If there is a next step
			{
			//		IvySendMsg("New step");
					wp = (CICAS_UGV_TC) steps.front ();
					// call CICAS service
					if ( this->c_sendCommandToCICAS( &wp ) <= 0 )
					{	// We'll try again at next updateHook. Ivy debug msg.
					//	IvySendMsg ("Unable to send command to cicas");
					}
					else // If everything is ok, delete current step to execute the next one
					{
						steps.pop_front();
						IvySendMsg("OK");
					}
			}
		}
		else
		{
		//	IvySendMsg("Unavailable CICAS");
		}
	}

	// If current role is 'NORMAL', don't do anything
}


bool formation::configureHook()
{
	// Start listening to leader and follower promotions
	filters.insert( pair<string, MsgRcvPtr> ( "follow_req",
			IvyBindMsg ( followReqCallback, 0, "^FOLLOW_REQ(.*)" )));

	filters.insert( pair<string, MsgRcvPtr> ( "leader",
			IvyBindMsg ( leaderCallback, 0, "^LEADER (.*)" )));

	// Retrieve robot ID
	id = p_identifier;

	return true;
}

void formation::ivyLoop()
{
	// Start Ivy infinite loop
	IvyMainLoop();
}

void formation::init()
{
}

bool formation::bye(){
	// Stop Ivy bus (interrupts Ivy infinite loop)
	IvyStop();

	return true;
}

/******************************************************************************/
/************************ INITIALIZATION PHASE CALLBACKS **********************/
/******************************************************************************/

void leaderCallback (IvyClientPtr app, void *data, int cargc, char **argv)
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

    // Am I concerned by this message ?
    if ( f_id == id )
    {
    	// Stop listening to follower and leader promotions
		IvyUnbindMsg( filters.find ( "leader" )->second );
		IvyUnbindMsg( filters.find ( "follow_req" )->second );
		filters.erase( filters.find("leader") );
		filters.erase( filters.find("follow_req") );

		// The very first message the leader will receive is a DO_DEMO (see specs)
			filters.insert( pair<string, MsgRcvPtr> ( "do_demo",
					IvyBindMsg ( doDemoCallback, 0, "^DO_DEMO$" )));

		// I'm the new leader
		role = LEADER;
    }
}

void doDemoCallback (IvyClientPtr app, void *data, int cargc, char **argv)
{
	// Desactivate do demo request
	IvyUnbindMsg( filters.find ( "do_demo" )->second );
	filters.erase( filters.find("do_demo") );

	filters.insert( pair<string, MsgRcvPtr> ( "follow_yes",
			IvyBindMsg ( followYesCallback, 0, "^FOLLOW_YES (.*)" )));

	filters.insert( pair<string, MsgRcvPtr> ( "stop_demo",
			IvyBindMsg ( stopDemoCallback, 0, "^STOP_DEMO$" )));

	// Ask for candidate followers
	IvySendMsg("FOLLOW_REQ");
}

void followReqCallback (IvyClientPtr app, void *data, int cargc, char **argv)
{
	if ( role == NORMAL ) // If the robot isn't already in a formation
	{
		// Stop listening to follower and leader promotions
		IvyUnbindMsg( filters.find ( "leader" )->second );
		IvyUnbindMsg( filters.find ( "follow_req" )->second );
		filters.erase( filters.find("follow_req") );
		filters.erase( filters.find("leader") );

		// Wait for a confirmation or a cancellation
		filters.insert( pair<string, MsgRcvPtr> ( "follow_me",
				IvyBindMsg ( followMeCallback, 0, "^FOLLOW_ME (.*)" )));
		filters.insert( pair<string, MsgRcvPtr> ( "ignore_req",
				IvyBindMsg ( ignoreReqCallback, 0, "^IGNORE_REQ (.*)" )));

		IvySendMsg ( "FOLLOW_YES %d", id ); // I'm in!
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

    candidates.push_back( f_id );
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

    // Am I concerned by this message ?
    if ( f_id == id )
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

		role = FOLLOWER;
		IvySendMsg ("IN_POSITION");
    }
}

void inPositionCallback (IvyClientPtr app, void *data, int cargc, char **argv)
{
	nb_answers ++;

	// If all the followers are now in position (send GOGOGO once everyone has replied)
	if ( nb_answers == NB_FOLLOWERS )
	{
		IvyUnbindMsg( filters.find ( "in_position" )->second );
		filters.erase( filters.find("in_position") );

		IvySendMsg ( "GOGOGO" );
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
 //   PositionLocale goal;
    CICAS_UGV_TC tc;

    args = argv[0];

    for ( int i = 0; i < NB_TC_CICAS_UGV; i ++ )
    {
    	token = strtok_r(args, &delim, &saveptr);
    	if (token != NULL) tc.commands[i] = ::atoi(token);
    	args = NULL;
    }
    steps.push_back( tc );
	IvySendMsg("FOLLOWER: 40 -40 %d %d %d %d %d %d %d", /*tc.commands[0], tc.commands[1],*/ tc.commands[2], tc.commands[3],
			tc.commands[4], tc.commands[5], tc.commands[6], tc.commands[7], tc.commands[8]);
}

void endOfFormationCallback (IvyClientPtr app, void *data, int argc, char **argv)
{
	// Unsubscribe from control orders or end of mission notification
	IvyUnbindMsg( filters.find ( "control" )->second );
	IvyUnbindMsg( filters.find ( "leave_me_alone" )->second );
	filters.erase ( filters.find ("control" ));
	filters.erase ( filters.find ("leave_me_alone" ));

	// Back to normal mode
	role = NORMAL;
	// Back to first phase
	phase = INITIALIZATION;

	// Start again listening to follow request from leaders or to leader promotions
	filters.insert( pair<string, MsgRcvPtr> ( "follow_req",
			IvyBindMsg ( followReqCallback, 0, "^FOLLOW_REQ(.*)" )));
	filters.insert( pair<string, MsgRcvPtr> ( "leader",
			IvyBindMsg ( leaderCallback, 0, "^LEADER (.*)" )));
}

void stopDemoCallback (IvyClientPtr app, void *data, int argc, char **argv)
{
	// Unsubscribe from end of formation order
	IvyUnbindMsg( filters.find ( "stop_demo" )->second );
	filters.erase ( filters.find ("stop_demo" ));

	// Back to normal mode
	role = NORMAL;
	// Back to first phase
	phase = INITIALIZATION;

	// Start again listening to follow request from leaders or to leader promotions
	filters.insert( pair<string, MsgRcvPtr> ( "follow_req",
			IvyBindMsg ( followReqCallback, 0, "^FOLLOW_REQ(.*)" )));
	filters.insert( pair<string, MsgRcvPtr> ( "leader",
			IvyBindMsg ( leaderCallback, 0, "^LEADER (.*)" )));

	IvySendMsg ("LEAVE_ME_ALONE");
}

} // namespace

// Define component as deployable in the library
ORO_CREATE_COMPONENT_TYPE()
ORO_LIST_COMPONENT_TYPE(TTRK::formation)
