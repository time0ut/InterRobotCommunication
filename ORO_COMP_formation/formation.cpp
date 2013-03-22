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

// The follower own't update its orientation if it is already close from
// the desired (leader) orientation. The threshold is in degrees.
#define CAP_TRESHOLD 0.15
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
		void echoCallback(IvyClientPtr app, void *data, int cargc, char **argv);
		void testCallback(IvyClientPtr app, void *data, int cargc, char **argv);
		// Desired position sent by the followers to the command law (via output port Formation::op_joystick)
TypeInfosJoystickMavLink pos;

// Ivy filters currently used by the robot (kept up-to-date in real time)
std::map <string, MsgRcvPtr> filters;

// List of successive positions the follower has to reach to follow the leader
std::list <PositionLocale> steps;

// List of candidates to enter the formation as followers
std::list <int> candidates;

// List of followers
std::list <int> followers;
list <int>::const_iterator followeri;

// Robot role
TypeRobotRole role = NORMAL;
// Robot ID number
int id = 0;
// Mission phase
MissionPhase phase;

// Number of followers ready to move
int nb_answers = 0;
//for network performance evaluation
//uint64_t timeNow;

//constructor
formation::formation(const std::string& name) :
						TaskContext (name, PreOperational),
						_debugNbPeriods(0),
						c_cmdLawMoveTo ( "MoveTo" ),
						c_cmdLawRotate ( "Rotate" ),
						c_cmdLawIsRunning ( "IsCommandRunning" ),
						ip_relativePosition ( "position_local" ),
						ip_systemState ("state"),
						op_joystick ( "infosTcMavlink" ),
//						_phase ( INITIALIZATION ),
						p_identifier ( 0 ),
						p_refresh_period (5),
						seq(0)

{
	cout<<"component "<< COMPONENT_NAME <<" build version:"<< VERSION << "on "<< __DATE__ <<" at "<< __TIME__<<endl;

	char *domain = getenv ("IVYBUS");
	IvyInit ( name.c_str(), NULL, 0, 0, 0, 0);
	IvyStart ( domain );

	// Ports
	this->addPort( ip_relativePosition ).doc("Local position input");
	this->addPort( ip_systemState ).doc("System State input");
	this->addPort( op_joystick ).doc("Output in joystick format for command law");

	// Properties
	this->addProperty( "identifier", this->p_identifier ).doc ( "robot identifier" );
	this->addProperty( "refresh_period", this->p_refresh_period ).doc ( "leader refresh period" );
	this->addProperty("timeMesNbPeriods",_timeMesNbPeriods).doc("Time measurement results are written to a  file every n activation periods  -- 0 if no measurement");
	_timeMesNbPeriods=1;

	// Operations
	this->provides()->addOperation("IvyLoop", &formation::ivyLoop, this);
	this->provides()->addOperation("bye", &formation::bye, this);

	// Service requester
	this->requires()->addOperationCaller(this->c_cmdLawMoveTo);
	this->requires()->addOperationCaller(this->c_cmdLawRotate);
	this->requires()->addOperationCaller(this->c_cmdLawIsRunning);

	// TMP debug
	this->init();
}

void formation::updateHook()
{
	static int countdown = REFRESH_PERIOD;
	//static std::ofstream outfile ("roundtripdelay.txt");//for performance evaluation, the file where we store the time of broadcast and reception of messages


	// Update relative position
	ip_relativePosition.read( this->_relative_position );
	ip_systemState.read ( this->_system_state );

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
			}
		}
		else if ( phase == FORMATION )
		{
			_timeMeasurement->startPoint();
			//timeNow = os::TimeService::Instance()->getNSecs()/1000;
			// Send to ivy bus the TEST message
			IvySendMsg("TEST %d %d 12345678901234567890", seq,*followeri);//?followeri indicate which follower should send the echo
			seq++;
			followeri++;
			if(followeri==followers.end())
				followeri=followers.begin();
		}
		countdown = p_refresh_period;//REFRESH_PERIOD; // Restart timer
	}
	else if ( role == FOLLOWER )
	{
		if ( ! c_cmdLawIsRunning () ) // If the robot stopped moving
		{
			if ( ! steps.empty () ) // If there is a next step
			{
					PositionLocale wp = (PositionLocale) steps.front ();
					steps.pop_front();

					if (abs(wp.cap - this->_system_state.mPsi ) > CAP_TRESHOLD*180/3.14){

						pos.roll = ((this->_system_state.mPsi) < (wp.cap)) ? -0.15 : 0.15;

					IvySendMsg ( "Heading: from %f to %f; r= %f",this->_relative_position.cap, wp.cap, pos.roll );
					op_joystick.write(pos);
					pos.roll = 0;
				}
						else if(abs(wp.x - this->_relative_position.x) > X_TRESHOLD ||
             					abs(wp.y - this->_relative_position.y) > Y_TRESHOLD){

							pos.pitch = 0.05;
							IvySendMsg ( "I want to move: pitch= %f, roll= %f", pos.pitch, pos.roll );
							op_joystick.write(pos);
							pos.pitch = 0;
						}

			}
			}
		}

	// If current role is 'NORMAL', don't do anything
}


bool formation::configureHook()
{
	_timeMeasurement =new  TimeMeasurement(COMPONENT_NAME,_timeMesNbPeriods);
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
	//for network performance evaluation
	filters.insert( pair<string, MsgRcvPtr> ( "echo",
				IvyBindMsg ( echoCallback, 0, "^ECHO (.*)" )));

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

void echoCallback(IvyClientPtr app, void *data, int cargc, char **argv){
	_timeMeasurement->endpoint();
	/*char *args;
	char *seq;
	char *saveptr;
	char delim = ' ';

	timeNow = os::TimeService::Instance()->getNSecs()/1000;
	args = argv[0];
	seq= strtok_r( args, &delim, &saveptr );
	time[seq][1]=timeNow;*/
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
	filters.insert( pair<string, MsgRcvPtr> ( "test",
			IvyBindMsg ( testCallback, 0, "^TEST (.*)" )));
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
}
//send the echo when receive the test message
void testCallback(IvyClientPtr app, void *data, int cargc, char **argv){
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

void endOfFormationCallback (IvyClientPtr app, void *data, int argc, char **argv)
{
	// Unsubscribe from control orders or end of mission notification
	IvyUnbindMsg( filters.find ( "test" )->second );
	IvyUnbindMsg( filters.find ( "leave_me_alone" )->second );
	filters.erase ( filters.find ("test" ));
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
