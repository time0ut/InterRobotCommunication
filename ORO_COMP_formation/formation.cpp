#include <rtt/Component.hpp>

#include <Ivy/ivy.h>
#include <Ivy/ivyloop.h>
#include <cstdio>

#include <map>
#include <sstream>

#include "formation.hpp"

namespace TTRK {

using namespace std;
using namespace RTT;

///////////// Declaration of Ivy messages callback functions:
// Called in the followers on reception of a CONTROL message
void newWPCallback (IvyClientPtr app, void *data, int argc, char **argv);
// Called in the followers on reception of a FOLLOW_ME message
void followMeCallback (IvyClientPtr app, void *data, int cargc, char **argv);
// Called in the followers on reception of a FOLLOW_REQ message
void followReqCallback (IvyClientPtr app, void *data, int cargc, char **argv);
// Called in the followers on reception of a IGNORE_REQ message
void ignoreReqCallback (IvyClientPtr app, void *data, int cargc, char **argv);
// Called in the leader on reception of a FOLLOW_YES message
void followYesCallback (IvyClientPtr app, void *data, int cargc, char **argv);
// Called in the followers on reception of a GOGOGO message
void gogogoCallback (IvyClientPtr app, void *data, int cargc, char **argv);
// Called in the leader on reception of a DO_DEMO message
void doDemoCallback (IvyClientPtr app, void *data, int cargc, char **argv);
// Called in all the robots on reception of a STOP_DEMO message
void stopDemoCallback (IvyClientPtr app, void *data, int cargc, char **argv);
// Called in the leader on reception of a IN_POSITION message
void inPositionCallback (IvyClientPtr app, void *data, int cargc, char **argv);
// Called in all the robots in NORMAL status on reception of a LEADER message
void leaderCallback (IvyClientPtr app, void *data, int cargc, char **argv);
// Perf tests callbacks
void doTestCallback (IvyClientPtr app, void *data, int cargc, char **argv);
void echoCallback(IvyClientPtr app, void *data, int cargc, char **argv);
void testCallback(IvyClientPtr app, void *data, int cargc, char **argv);

// Ivy filters currently used by the robot (kept up-to-date in real time)
std::map <string, MsgRcvPtr> filters;

// List of the successive commands the followers have to send to the CICAS calculator
// in order to reproduce the movements of the leader of the formation
std::list <CICAS_UGV_TC> steps;

// List of candidates to enter the formation as followers
std::list <int> candidates;

// List of followers
std::list <int> followers;
list <int>::const_iterator followeri;

// Robot role
TypeRobotRole role = NORMAL;

// Robot ID number
int id = 0;

// Number of followers in the formation
int nb_followers;

// Mission phase (initialization, formation ?)
MissionPhase phase;

// Number of followers ready to move
int nb_answers = 0;


CICAS_UGV_TC wp;

//constructor
formation::formation(const std::string& name) :
						TaskContext (name, PreOperational),
						ip_cicas_tc ("CICAS_TC_LEADER"),
						op_cicas_tc ("CICAS_TC_FOLLOWER"),
						p_idnumber ( 0 ),
						p_refresh_period ( 5 ),
						p_nb_followers ( 1 )

{
	cout<<"component "<< COMPONENT_NAME <<" build version:"<< VERSION << "on "<< __DATE__ <<" at "<< __TIME__<<endl;

	char *domain = getenv ("IVYBUS");
	IvyInit ( name.c_str(), NULL, 0, 0, 0, 0);
	IvyStart ( domain );

	// Ports
	this->addPort( ip_cicas_tc ).doc("CICAS TC coming from command law");
	this->addPort( op_cicas_tc ).doc("CICAS TC going to cicas");

	// Properties
	this->addProperty( "idnumber", this->p_idnumber ).doc ( "robot id number" );
	this->addProperty( "refresh_period", this->p_refresh_period ).doc ( "leader refresh period" );
	this->addProperty( "nbfollowers", this->p_nb_followers ).doc ( "number of followers in the formation" );

	// Operations
	this->provides()->addOperation("IvyLoop", &formation::ivyLoop, this);
	this->provides()->addOperation("bye", &formation::bye, this);
	this->provides()->addOperation("IvySend", &formation::ivySendMsg, this);

	this->init();
}

void formation::updateHook()
{
	static int countdown = p_refresh_period;

	// Update relative position
	ip_cicas_tc.read ( this->_cicas_tc );

	if ( role == LEADER && countdown -- <= 0 )  // Time to refresh
	{
		if ( phase == INITIALIZATION )
		{
			if ( candidates.size() >= p_nb_followers )
			{ // Sending a personal confirmation message to all chosen robots
				// Stop listening to candidates
				IvyUnbindMsg( filters.find ( "follow_yes" )->second );
				filters.erase( filters.find("follow_yes") );

				filters.insert( pair<string, MsgRcvPtr> ( "in_position",
						IvyBindMsg ( inPositionCallback, 0, "^IN_POSITION$" )));

				int msg_sent ( 0 );
				while ( ! candidates.empty() && msg_sent < p_nb_followers )
				{
					IvySendMsg ( "FOLLOW_ME %d", candidates.front() );
					followers.push_back(candidates.front());
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
				followeri=followers.begin();

				// Start listening to echo
				filters.insert( pair<string, MsgRcvPtr> ( "echo",
				IvyBindMsg ( echoCallback, 0, "^ECHO " )));


			}
		}
		else if ( phase == FORMATION )
		{
			int16_t *tmp = this->_cicas_tc.commands;
			IvySendMsg ( "CONTROL %d %d %d %d %d %d %d %d %d",
					tmp[0], tmp[1], tmp[2], tmp[3], tmp[4], tmp[5],
					tmp[6], tmp[7], tmp[8]);

		}
		else if ( phase == TEST )
		{
			static int seq (0);
			_timeMeasurement->startPoint();
			
			// Send to ivy bus the TEST message
			IvySendMsg("TEST %d %d 12345678901234567890", seq++,*followeri);//?followeri indicate which follower should send the ech
			followeri++;
			if(followeri==followers.end())
				followeri=followers.begin();

		}
		countdown = p_refresh_period;//REFRESH_PERIOD; // Restart timer
	}
	else if ( role == FOLLOWER )
	{
		if ( ! steps.empty () ) // If there is a next step
		{
			// retrieve next action and send it to the CICAS calculator
			CICAS_UGV_TC cmd = (CICAS_UGV_TC) steps.front ();
			this->op_cicas_tc.write( cmd );
			// remove it from the list
			steps.pop_front();
		}
	}
	// If current role is 'NORMAL', don't do anything
}


bool formation::configureHook()
{
	_timeMeasurement = new  TimeMeasurement(COMPONENT_NAME, 1);
	// Start listening to leader and follower promotions
	filters.insert( pair<string, MsgRcvPtr> ( "follow_req",
			IvyBindMsg ( followReqCallback, 0, "^FOLLOW_REQ(.*)" )));

	filters.insert( pair<string, MsgRcvPtr> ( "leader",
			IvyBindMsg ( leaderCallback, 0, "^LEADER (.*)" )));
	
	filters.insert( pair<string, MsgRcvPtr> ( "stop_demo",
			IvyBindMsg ( stopDemoCallback, 0, "^STOP_DEMO$" )));

	// Retrieve robot ID and number of followers in the formation
	id = p_idnumber;
	nb_followers = p_nb_followers;

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

void formation::ivySendMsg( std::string msg )
{
	IvySendMsg("%s", msg.c_str());
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
void doTestCallback (IvyClientPtr app, void *data, int cargc, char **argv)
{
	// Desactivate do demo request

	IvyUnbindMsg( filters.find ( "do_test" )->second );
	filters.erase( filters.find("do_test") );

	phase = TEST;
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

void echoCallback(IvyClientPtr app, void *data, int cargc, char **argv){
	_timeMeasurement->endpoint();
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
	if ( nb_answers == nb_followers )
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

		// Start listening again to follow requests
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
	filters.insert( pair<string, MsgRcvPtr> ( "test_perf",
			IvyBindMsg ( testCallback, 0, "^TEST (.*)" )));
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
    CICAS_UGV_TC tc;

    args = argv[0];

	_timeMeasurement->endpoint();
	_timeMeasurement->startPoint();

    for ( int i = 0; i < NB_TC_CICAS_UGV; i ++ )
    {
    	token = strtok_r(args, &delim, &saveptr);
    	if (token != NULL) tc.commands[i] = ::atoi(token);
    	args = NULL;
    }
    steps.push_back( tc );
}

//send the echo when receive the test message
void testCallback(IvyClientPtr app, void *data, int cargc, char **argv)
{
char *args;
	char *saveptr;
	char delim = ' ';
	char *seq;
	char *followerid;
	args = argv[0];
	seq = strtok_r(args, &delim, &saveptr);
	followerid= strtok_r(NULL, &delim, &saveptr);
	if(id==atoi(followerid))
		IvySendMsg("ECHO %s %s 12345678901234567890",seq,followerid);

}

void stopDemoCallback (IvyClientPtr app, void *data, int argc, char **argv)
{
	// Unsubscribe from end of formation order
	IvyUnbindMsg( filters.find ( "stop_demo" )->second );
	filters.erase ( filters.find ("stop_demo" ));

	// Start again listening to follow request from leaders or to leader promotions
	filters.insert( pair<string, MsgRcvPtr> ( "follow_req",
			IvyBindMsg ( followReqCallback, 0, "^FOLLOW_REQ(.*)" )));
	filters.insert( pair<string, MsgRcvPtr> ( "leader",
			IvyBindMsg ( leaderCallback, 0, "^LEADER (.*)" )));

	if ( role == FOLLOWER)
	{
		// Unsubscribe from control orders or end of mission notification
		if ( filters.find ( "control" ) != filters.end() )
		{
			IvyUnbindMsg( filters.find ( "control" )->second );
			filters.erase ( filters.find ("control" ));
		}
		if ( filters.find ( "leave_me_alone" ) != filters.end() )
		{
			IvyUnbindMsg( filters.find ( "leave_me_alone" )->second );
			filters.erase ( filters.find ("leave_me_alone" ));
		}
		if ( filters.find ( "test_perf" ) != filters.end() )
		{
			IvyUnbindMsg( filters.find ( "test_perf" )->second );
			filters.erase ( filters.find ("test_perf" ));
		}
	}

	// Back to normal mode
	role = NORMAL;
	// Back to first phase
	phase = INITIALIZATION;

}

} // namespace

// Define component as deployable in the library
ORO_CREATE_COMPONENT_TYPE()
ORO_LIST_COMPONENT_TYPE(TTRK::formation)
