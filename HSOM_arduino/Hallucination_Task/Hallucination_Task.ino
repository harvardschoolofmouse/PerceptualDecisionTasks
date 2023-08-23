/*********************************************************************
	Arduino state machine code for Hallucination_Task (mice)
	
	Training Paradigm and Architecture    - Allison Hamilos (ahamilos@g.harvard.edu)
	Optogenetics Control System 		  - Allison Hamilos (ahamilos@g.harvard.edu)
	Matlab Serial Communication Interface - Ofer Mazor
	State System Architecture             - Lingfeng Hou (lingfenghou@g.harvard.edu)

	Created       12/08/22 - ahamilos
	Last Modified 01/11/22 - ljacobs-skolik	"JAN 11 2023 17h"

	static String versionCode        = "JAN 11 2023 17h";
	
	
/*	New to THIS version:
	- Initial version updated by LJS, August 2023
	
	------------------------------------------------------------------
	COMPATIBILITY REPORT:
		Matlab HOST: Matlab 2016a - FileName = MouseBehaviorInterface.m (depends on ArduinoConnection.m)
		Arduino:
			Default: TEENSY3.6
			Others:  UNO, TEENSY3.2, DUE, MEGA
	------------------------------------------------------------------
	Reserved:
		
		Event Markers: 0-##
		States:        0-##
		Result Codes:  0-##
		Parameters:    0-##
	------------------------------------------------------------------
	Task Architecture: Pavlovian-Operant

	Init Trial                (event marker = ????)
		-  Decide if S2 will be played and if so, when
	Trial Body
		-  Decide what to do.... (event marker = ????)
	End Trial                 (event marker = ???)    
		-  Decide what to do....

	Behavioral Events:
		-  Lick                 (event marker = 8)
		-  Reward dispensed     (event marker = 9)
		-  Quinine dispensed    (event marker = 10)
		-  Waiting for ITI      (event marker = 11)   - enters this state if trial aborted by behavioral error, House lamps ON
		-  Correct Lick         (event marker = 12)   - first correct lick in the window
    -  1st Lick             (event marker = 16)   - first relevant lick in trial

	Trial Type Markers:
		-  
	--------------------------------------------------------------------
	States:
		0: _INIT                (private) 1st state in init loop, sets up communication to Matlab HOST
		1: IDLE_STATE           Awaiting command from Matlab HOST to begin experiment
  		2: INIT_TRIAL		Determine time of S (if it occurs)
		2: CUE_TRIAL           	Play white noise (WN) and S
		3: NOISE_TRIAL          (+/-) Enforced no lick before response window opens
		4: RESPONSE_WINDOW      Window over which animal can choose L or R
		5: HIT              	Reward state for correctly identifying S
		6: CORRECT_REJECT	Reward state for correctly saying No S present
		7: MISS,		Unrewarded state for missing stimulus S
		8: FALSE_ALARM		Unrewarded state for picking side of S2, but there was no S2
    		9: NO_RESPONSE		
		8: INTERTRIAL           House lamps ON (if not already), write data to HOST and DISK
  

	---------------------------------------------------------------------
	Result Codes:
		0: CODE_CORRECT         First lick within response window               
		1: CODE_EARLY_LICK      Early lick -> Abort (Enforced No-Lick Only)
		2: CODE_LATE_LICK       Late Lick  -> Abort (Operant Only)
		3: CODE_NO_LICK         No Response -> Time Out
	---------------------------------------------------------------------
	Parameters:
		
	---------------------------------------------------------------------
		Incoming Message Syntax: (received from Matlab HOST)
			"(character)#"        -- a command
			"(character int1 int2)# -- update parameter (int1) to new value (int2)"
			Command characters:
				P  -- parameter
				O# -- HOST has received updated paramters, may resume trial
				Q# -- quit and go to IDLE_STATE
				G# -- begin trial (from IDLE_STATE)
	---------------------------------------------------------------------
		Outgoing Message Syntax: (delivered to Matlab HOST)
			ONLINE:  
				"~"                           Tells Matlab HOST arduino is running
			STATES:
				"@ (enum num) stateName"      Defines state names for Matlab HOST
				"$(enum num) num num"         State-number, parameter, value
 -                                          param = 0: current time
 -                                          param = 1: result code (enum num)
			EVENT MARKERS:
				"+(enum num) eventMarker"     Defines event markers with string
				"&(enum num) timestamp"       Event Marker with timestamp

			RESULT CODES:
			"* (enum num) result_code_name" Defines result code names with str 
			"` (enum num of result_code)"   Send result code for trial to Matlab HOST

			MESSAGE:
				"string"                      String to Matlab HOST serial monitor (debugging) 
	---------------------------------------------------------------------
	STATE MACHINE
		- States are written as individual functions
		- The main loop calls the appropriate state function depending on current state.
		- A state function consists of two parts
		 - Action: executed once when first entering this state.
		 - Transitions: evaluated on each loop and determines what the next state should be.
*********************************************************************/

/*****************************************************
                  Arduino Libraries
 *****************************************************/ 

/*****************************************************
	              Global Variables
*****************************************************/

/*****************************************************
Arduino Pin Outs (Mode: TEENSY3.6)

Available pins for 3.4: 4-11 20-23
*****************************************************/


// PWM OUT --- n.b! 2 tones can't play at the same time!
#define PIN_SPEAKER        A21   	// Speaker Pin            (DUE =  2)  (MEGA =  8)  (UNO =  9) (TEENSY = 5)	(TEENSY3.4 5)
#define PIN_NOISE          30   	// Noise Pin              (DUE =  3)  (MEGA =  9)  (UNO = 10) (TEENSY = 6)	(TEENSY3.4 6)
#define PIN_LEFT_REWARD    36   // Left spout solenoid
#define PIN_RIGHT_REWARD   35   // Right spout solenoid
#define PIN_LED_CUE        34   // Cue LED - Tells subject it is time to respond
#define PIN_HOUSE_LAMP     33   // House light
// Digital IN
#define PIN_LEFT_LICK      30    // Left lick sensor
#define PIN_RIGHT_LICK     32    // Right lick sensor
/*****************************************************
Enums - DEFINE States
*****************************************************/
// All the states
enum State
{
    _INIT,                // (Private) Initial state used on first loop.
    IDLE_STATE,           // Idle state. Wait for go signal from host.
    INIT_TRIAL,           // Determine time of S2 (if it occurs)
    CUE_TRIAL,            // Play white noise and S2
    NOISE_TRIAL,          // Play white noise only
    RESPONSE_WINDOW,	  // Window over which animal can choose L or R
    HIT,               	  // Reward state for correctly identifying S2
    CORRECT_REJECT,       // Reward state for correctly saying No S2 present
    MISS,		// Unrewarded state for missing stimulus S2
    FALSE_ALARM,	// Unrewarded state for picking side of S2, but there was no S2
    NO_RESPONSE,	// WHAT HAPPENS HERE?
    INTERTRIAL,           // WHAT HAPPENS HERE?
    _NUM_STATES           // (Private) Used to count number of states
};

// State names stored as strings, will be sent to host
// Names cannot contain spaces!!!
static const char *_stateNames[] =
        {
                "_INIT",
                "IDLE_STATE",
                "INIT_TRIAL",
                "CUE_TRIAL",
                "NOISE_TRIAL",
                "RESPONSE_WINDOW",
                "HIT",
                "CORRECT_REJECT",
                "MISS",
                "FALSE_ALARM",
                "NO_RESPONSE",
                "INTERTRIAL"
        };

// Define which states allow param update
static const int _stateCanUpdateParams[] = {0,1,0,0,0,0,0,0,0,0,0,1};
// Defined to allow Parameter upload from host during IDLE_STATE and INTERTRIAL

// Define and label duration in milliseconds of each state
static const int _stateDurationsMS[] = {1000, // _INIT
                                        1000, // IDLE_STATE
                                        1000, // INIT_TRIAL
                                        1000, // CUE_TRIAL
                                        1000, // NOISE_TRIAL
                                        1000, // RESPONSE_WINDOW
                                        1000, // HIT
                                        1000, // CORRECT_REJECT
                                        1000, // MISS
                                        1000, // FALSE_ALARM
                                        5000, // NO_RESPONSE
                                        2000 // INTERTRIAL
                                       };


/*****************************************************
Event Markers
*****************************************************/
enum EventMarkers
/* You may define as many event markers as you like.
		Assign event markers to any IN/OUT event
		Times and trials will be defined by global time,
		which can be parsed later to validate time measurements */
{
    EVENT_TRIAL_INIT,       // New trial initiated
    EVENT_STIMULUS_WINDOW,       // New trial initiated
    EVENT_RESPONSE_WINDOW,       // New trial initiated
    EVENT_HIT,       			 // New trial initiated
    EVENT_CORRECT_REJECT,        // New trial initiated
    EVENT_MISS,       			 // New trial initiated
    EVENT_FALSE_ALARM,       	 // New trial initiated
    EVENT_NO_RESPONSE,       	 // New trial initiated
    EVENT_NOISE_TRIAL,       	 // Noise trial started
    EVENT_CUE_TRIAL,       		 // Cue trial started
    EVENT_INTERTRIAL,             // New trial initiated
    EVENT_WHITE_NOISE_ON,      	  // New trial initiated
    EVENT_S2,       			  // New trial initiated
    EVENT_LICK_LEFT,      		  // New trial initiated
    EVENT_LICK_RIGHT,       	  // New trial initiated
    EVENT_REWARD_LEFT,            // Reward dispensed
    EVENT_REWARD_RIGHT,           // Reward dispensed
    _NUM_OF_EVENT_MARKERS
};

static const char *_eventMarkerNames[] =    // * to define array of strings
        {
                "TRIAL_INIT",
                "STIMULUS_WINDOW",
                "RESPONSE_WINDOW",
                "HIT",
                "CORRECT_REJECT",
                "MISS",
                "FALSE_ALARM",
                "NO_RESPONSE",
                "NOISE_TRIAL",
                "CUE_TRIAL",
                "INTERTRIAL",
                "WHITE_NOISE_ON",
                "S2",
                "LICK_LEFT",
                "LICK_RIGHT",
                "REWARD_LEFT",
                "REWARD_RIGHT"
        };

/*****************************************************
Result codes
*****************************************************/
enum ResultCode
{
    CODE_HIT,                           	   // Correctly responded there was an S2
    CODE_MISS,                            	   // There was S2 but responded that there wasn't
    CODE_CORRECT_REJECT,                       // Responded no S2 when there wasn't S2
    CODE_FALSE_ALARM,                    	   // False positive (responded that there was a S2 when there wasn't)
    CODE_NO_RESPONSE,                    	   // No response
    _NUM_RESULT_CODES                          // (Private) Used to count how many codes there are.
};

// We'll send result code translations to MATLAB at startup
static const char *_resultCodeNames[] =
        {
                "CODE_HIT",
                "CODE_MISS",
                "CODE_CORRECT_REJECT",
                "CODE_FALSE_ALARM",
                "CODE_NO_RESPONSE"
        };


/*****************************************************
Audio cue frequencies
*****************************************************/
enum SoundEventFrequencyEnum
{
    TONE_REWARD  = 17000,            // What is this?
    TONE_FAIL    = 10000,            // What is this?
    TONE_S2      = 2000             // What is this?
};

/*****************************************************
Parameters that can be updated by HOST
*****************************************************/
// Storing everything in array _params[]. Using enum ParamID as array indices so it's easier to add/remove parameters.
enum ParamID
{
    _DEBUG,                         // (Private) 1 to enable debug messages from HOST. Default 0.
    CALIBRATION,                    // 1 to enable calibration mode. Default 0.
    PERCENT_S2,					    // percent of trials where S2 should be present
    S2_SPOUT,                       // 1 if the Left side is S2 spout, 2 if right side is S2 spout
    NOISE_SPOUT,                    // 1 if the Left side is noise spout, 2 if right side is noise spout
    REWARD_DURATION_MS,             // Reward duration in ms
    WHITE_NOISE_DURATION_MS,        // Time of the white noise cue
    S2_DURATION_MS,                 // Time of S2 being on
    SNR_PERCENT,                    // Signal to noise ratio
    SNR_STEP,                       // Signal to noise ratio step size
    RESPONSE_WINDOW_MS,             // How long animal has to decide
    ITI_DURATION_MS,                // How long is ITI state
    PENALTY_DURATION_MS,            // How long is the penalty state
    TIME_NOISE_START,				// how long after entering NOISE_TRIAL or CUE_TRIAL white noise should start playing in ms
    MOVING_AVG_WINDOW,              // How many trials to use for moving average
    _NUM_PARAMS                     // (Private) Used to count how many parameters there are so we can initialize the param array with the correct size. Insert additional parameters before this.
}; //**** BE SURE TO ADD NEW PARAMS TO THE NAMES LIST BELOW!*****//

// Store parameter names as strings, will be sent to host
// Names cannot contain spaces!!!
static const char *_paramNames[] =
        {
                "_DEBUG",
                "CALIBRATION",
                "PERCENT_S2",
                "S2_SPOUT",
                "NOISE_SPOUT",
                "REWARD_DURATION_MS",
                "WHITE_NOISE_DURATION_MS",
                "S2_DURATION_MS",
                "SNR_PERCENT",
                "SNR_STEP",
                "RESPONSE_WINDOW_MS",
                "ITI_DURATION_MS",
                "PENALTY_DURATION_MS",
                "TIME_NOISE_START",
                "MOVING_AVG_WINDOW"
        }; //**** BE SURE TO INIT NEW PARAM VALUES BELOW!*****//

// Initialize parameters
int _params[_NUM_PARAMS] =
        {
                0,                              // _DEBUG
                0,                              // CALIBRATION
                50,								// PERCENT_S2
                1,                              // S2_SPOUT
                2,                              // NOISE_SPOUT
                40,                             // REWARD_DURATION_MS
                3000,							// WHITE_NOISE_DURATION_MS
                250,                            // S2_DURATION_MS
                50,                             // SNR_PERCENT
                5,                              // SNR_STEP
                3000,                           // RESPONSE_WINDOW_MS
                5000,                           // ITI_DURATION_MS
                5000,                           // PENALTY_DURATION_MS
                50, 						    // TIME_NOISE_START
                6                               // MOVING_AVG_WINDOW
        };

/*****************************************************
Other Global Variables
*****************************************************/
// Variables declared here can be carried to the next loop, AND read/written in function scope as well as main scope
// (previously defined):
static State _state                  = _INIT;    // This variable (current _state) get passed into a _state function, which determines what the next _state should be, and updates it to the next _state.
static State _prevState              = _INIT;    // Remembers the previous _state from the last loop (actions should only be executed when you enter a _state for the first time, comparing currentState vs _prevState helps us keep track of that).
static char _command                 = ' ';      // Command char received from host, resets on each loop
static int  _arguments[2]     		 = {0};      // Two integers received from host , resets on each loop ** changed from int to this so that we can collect negative inputs
static long _resultCode              = -1;       // Result code number. -1 if there is no result.

static long _eventMarkerTimer        = 0;
static long _trialTimer              = 0;
static long _time_init_trial 		 = 50;	     // time in ms in the init_trial state
static long _time_S2_start			 = 0;		 // time in ms when S2 should begin
static long _random_delay_timer      = 0;        // Random delay timer
static long _single_loop_timer       = 0;        // Timer


// state timers - track how long controller has been in each state
static long _timer_init_trial        = 0;        // Tracks time in init_trial state
static long _timer_cue_trial         = 0;        // Tracks time in cue_trial state
static long _timer_noise_trial       = 0;        // Tracks time in noise_trial state
static long _timer_response_window   = 0;        // Tracks time in response_window state
static long _timer_hit               = 0;        // Tracks time in hit state
static long _timer_miss              = 0;        // Tracks time in miss state
static long _timer_correct_reject    = 0;        // Tracks time in correct_reject state
static long _timer_false_alarm       = 0;        // Tracks time in false_alarm state
static long _timer_no_response       = 0;        // Tracks time in no_response state
static long _timer_intertrial        = 0;        // Tracks time in intertrial state

// number trackers for trials completed
static long _num_trials              = 0;        // Tracks number of trials completed
static long _num_left                = 0;        // Tracks number of trials where animal licked left
static long _num_right               = 0;        // Tracks number of trials where animal licked right
static long _num_correct             = 0;        // Tracks number of correct trials
static long _num_incorrect           = 0;        // Tracks number of incorrect trials
static long _num_hits                = 0;        // Tracks number of hits
static long _num_misses              = 0;        // Tracks number of misses
static long _num_correct_rejects     = 0;        // Tracks number of correct rejects
static long _num_false_alarms        = 0;        // Tracks number of false alarms
static long _num_no_responses        = 0;        // Tracks number of no responses

// percent trackers for animal performance
static long _percent_left            = 0;        // Tracks percent of trials that the animal chose the left side
static long _percent_right           = 0;        // Tracks percent of trials that the animal chose the right side
static long _percent_correct         = 0;        // Tracks percent of trials that the animal chose the correct side
static long _percent_incorrect       = 0;        // Tracks percent of trials that the animal chose the incorrect side
static long _percent_hit             = 0;        // Tracks percent of trials that the animal got a hit
static long _percent_miss            = 0;        // Tracks percent of trials that the animal missed
static long _percent_correct_reject  = 0;        // Tracks percent of trials that the animal correctly rejected
static long _percent_false_alarm     = 0;        // Tracks percent of trials that the animal had a false alarm
static long _percent_no_response     = 0;        // Tracks percent of trials that the animal did not respond in time

// moving average trackers for animal performance
static long _recentHitRate           = 0;        // Tracks the hit rate over the last 6 trials
static long _recentMissRate          = 0;        // Tracks the miss rate over the last 6 trials
static long _recentCorrectRejectRate = 0;        // Tracks the correct reject rate over the last 6 trials
static long _recentFalseAlarmRate    = 0;        // Tracks the false alarm rate over the last 6 trials
static long _recentNoResponseRate    = 0;        // Tracks the no response rate over the last 6 trials
static long _lastSNR                 = 50;        // Tracks the last SNR used
static State _recentOutcomes[6]; // Tracks the outcomes of the last 6 trials

static bool _leftLick         = false;    // True when left lick detected, False when no lick
static bool _rightLick        = false;    // True when right lick detected, False when no lick
static bool _prevLeftLick     = false;    // lags behind _leftLick by one loop, used to detect when new lick begins
static bool _prevRightLick    = false;    // lags behind _rightLick by one loop, used to detect when new lick begins
static bool _toneSpoutLick    = false;    // True when tone spout lick detected, False when no lick
static bool _noiseSpoutLick   = false;    // True when noise spout lick detected, False when no lick
static bool _is_S2			  = false;	  // Set during INIT_TRIAL state, indicates whether trial will have S2 present or be noise only

static long _exp_timer               = 0;        // Experiment timer, reset to signedMillis() at every soft reset
static long _stimulus_timer          = 0;        // Tracks time cue has been displayed for
static long _reward_timer            = 0;        // Tracks time in reward state

static long _dice_roll = 0; // random number generator for determining whether S2 is present or not

static char versionCode = "1.0.0";


/*****************************************************
	INITIALIZATION LOOP
*****************************************************/
void setup()
{
    //--------------------I/O initialization------------------//
    // OUTPUTS
    pinMode(PIN_SPEAKER, OUTPUT);            // write your outputs here
    pinMode(PIN_LEFT_REWARD, OUTPUT);
    pinMode(PIN_RIGHT_REWARD, OUTPUT);
    pinMode(PIN_LED_CUE, OUTPUT);
    pinMode(PIN_HOUSE_LAMP, OUTPUT);
    pinMode(PIN_NOISE, OUTPUT);

    // INPUTS
    pinMode(PIN_LEFT_LICK, INPUT);                   // Lick detector
    pinMode(PIN_RIGHT_LICK, INPUT);                   // Lick detector

    //--------------------------------------------------------//


    //------------------------Serial Comms--------------------//
    Serial.begin(115200);                       // Set up USB communication at 115200 baud

    //---------------------- Potentiometer --------------------//
    
} // End Initialization Loop -----------------------------------------------------------------------------------------------------


/*****************************************************
	MAIN LOOP
*****************************************************/
void loop()
{
    // Initialization
    mySetup();

    // Main loop (R# resets it)
    while (true)
    {
        /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            Step 1: Read USB MESSAGE from HOST (if available)
        ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
        // 1) Check USB for MESSAGE from HOST, if available. String is read byte by byte. (Each character is a byte, so reads e/a character)
        static String usbMessage  = "";             // Initialize usbMessage to empty string, only happens once on first loop (thanks to static!)
        _command = ' ';                              // Initialize _command to a SPACE
        _arguments[0] = 0;                           // Initialize 1st integer argument
        _arguments[1] = 0;                           // Initialize 2nd integer argument

        if (Serial.available() > 0)  {              // If there's something in the SERIAL INPUT BUFFER (i.e., if another character from host is waiting in the queue to be read)
            char inByte = Serial.read();                  // Read next character

            // The pound sign ('#') indicates a complete message!------------------------
            if (inByte == '#')  {                         // If # received, terminate the message
                // Parse the string, and updates `_command`, and `_arguments`
                _command = getCommand(usbMessage);               // getCommand pulls out the character from the message for the _command
                getArguments(usbMessage, _arguments);            // getArguments pulls out the integer values from the usbMessage
                usbMessage = "";                                // Clear message buffer (resets to prepare for next message)
                if (_command == 'R') {
                    break;
                }
            }
            else {
                // append character to message buffer
                usbMessage = usbMessage + inByte;       // Appends the next character from the queue to the usbMessage string
            }
        }

        /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            Step 2: Update the State Machine
        ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
        // Depending on what _state we're in , call the appropriate _state function, which will evaluate the transition conditions, and update `_state` to what the next _state should be
        switch (_state) {
            case _INIT:
                idle_state();
                break;

            case IDLE_STATE:
                idle_state();
                break;

            case INIT_TRIAL:
                init_trial();
                break;

            case NOISE_TRIAL:
                noise_trial();
                break;

            case CUE_TRIAL:
                cue_trial();
                break;

            case RESPONSE_WINDOW:
                response_window();
                break;

            case HIT:
                hit();
                break;

            case MISS:
                miss();
                break;

            case FALSE_ALARM:
                false_alarm();
                break;

            case CORRECT_REJECT:
                correct_reject();
                break;

            case NO_RESPONSE:
                no_response();
                break;

            case INTERTRIAL:
                intertrial();
                break;
        } // End switch statement--------------------------
    }
} // End main loop-------------------------------------------------------------------------------------------------------------



void mySetup()
{

    //--------------Set initial OUTPUTS----------------//

    // Set the initial state of all your input and outputs. Examples:
    // setHouseLamp(true);                          // House Lamp ON
    // setCueLED(false);                            // Cue LED OFF


    //---------------------------Reset a bunch of variables---------------------------//

    _command                	= ' ';      // Command char received from host, resets on each loop
    _arguments[0]           	= 0;        // Two integers received from host , resets on each loop
    _arguments[1]           	= 0;        // Two integers received from host , resets on each loop
    _resultCode             	= -1;


    _eventMarkerTimer        = 0;
    _timer_init_trial		 = 0;
    _trialTimer              = 0;
    _random_delay_timer      = 0;        // Random delay timer
    _single_loop_timer       = 0;        // Timer

    _leftLick         = false;    // True when lick detected, False when no lick
    _rightLick        = false;    // True when lick detected, False when no lick

    _exp_timer               = 0;        // Experiment timer, reset to signedMillis() at every soft reset
    _stimulus_timer          = 0;        // Tracks time cue has been displayed for
    _reward_timer            = 0;        // Tracks time in reward state

    // Tell PC that we're running by sending '~' message:
    hostInit();                         // Sends all parameters, states and error codes to Matlab (LF Function)
}

/*****************************************************
	States for the State Machine
*****************************************************/
/* New states are initialized by the ACTION LIST
 In the main loop after state runs, Arduino checks for new parameters and switches the state */


/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	IDLE STATE - awaiting start cue from Matlab HOST
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void idle_state() {
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        ACTION LIST -- initialize the new state
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    if (_state != _prevState) {                      // If ENTERING IDLE_STATE:
        // Display the version code:
        sendMessage("-");
        sendMessage("Hallucination_Task");
        sendMessage("Version Code: " + versionCode);
        _prevState = _state;                             // Assign _prevState to idle _state
        sendMessage("$" + String(_state));               // Send a message to host upon _state entry -- $1 (Idle State)

        // kill any digital/analog outs here, for example
        // setHouseLamp(true);                              // Turn House Lamp ON

        // Reset state variables, for example
        // _pre_window_elapsed = false;                 	 // Reset pre_window time tracker



        //------------------------DEBUG MODE--------------------------//
        if (_params[_DEBUG]) {
            sendMessage("Idle.");
        }
        //----------------------end DEBUG MODE------------------------//
    }

    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        TRANSITION LIST -- checks conditions, moves to next state
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/


    if (_command == 'G') {                           // If Received GO signal from HOST ---transition to---> READY
        // Send Trigger Signal to the CED/NiDAQ:
        // Note, in new version this turns IR LED to OFF state
        // playSound(TONE_TRIGGER);                                                                                                                         ////////////////////////////////////////////////////////////////////////////////trigger///////////////////////////
        _state = INIT_TRIAL;                              // State set to INIT_TRIAL
        return;                                           // Exit function
    }


    if (_command == 'P') {                           // Received new param from host: format "P _paramID _newValue" ('P' for Parameters)
        //----------------------DEBUG MODE------------------------//
        if (_params[_DEBUG]) {sendMessage("Parameter " + String(_arguments[0]) + " changed to " + String(_arguments[1]));
        }
        //-------------------end DEBUG MODE--- -------------------//

        _params[_arguments[0]] = _arguments[1];           // Update parameter. Serial input "P 0 1000" changes the 1st parameter to 1000.
        _state = IDLE_STATE;                              // State returns to IDLE_STATE
        return;                                           // Exit function
    }

    _state = IDLE_STATE;                             // Return to IDLE_STATE
} // End IDLE_STATE ------------------------------------------------------------------------------------------------------------------------------------------



/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	INIT_TRIAL - Trial started.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void init_trial() {
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        ACTION LIST -- initialize the new state
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    if (_state != _prevState) {                             // If ENTERING INIT_TRIAL STATE:
        hostInit();
        _prevState = _state;                                // Assign _prevState to INIT_TRIAL _state
        sendMessage("$" + String(_state));                  // Send  HOST _state entry -- $2 (INIT_TRIAL State)
        // Send event marker (house_lamp_off) to HOST with timestamp
        sendMessage("&" + String(EVENT_TRIAL_INIT) + " " + String(signedMillis() - _exp_timer));
        _timer_init_trial = signedMillis();                     // Start _random_delay_timer
        if (_params[_DEBUG]) {sendMessage("Trial Started.");}
        _num_trials++; // increment trial counter
        // Do the initial business of the state -- something you only want to do once when you enter
        // choose whether trial will be S2 or noise
        _dice_roll = random(1,100);
        if (_dice_roll <= _params[PERCENT_S2]) {
            _is_S2 = true;
            // if debug, send message to host that trial will be tone trial and include the value of _dice_roll
            if (_params[_DEBUG]) {
                sendMessage("Trial will be S2. Dice roll: " + String(_dice_roll));
            }
        } else {
            _is_S2 = false;
            // if debug, send message to host that trial will be noise trial and include the value of _dice_roll
            if (_params[_DEBUG]) {
                sendMessage("Trial will be noise. Dice roll: " + String(_dice_roll));
            }
        }
        // if trial will be S2, choose when S2 will play
        if (_is_S2) {
            _time_S2_start = random(1,_params[WHITE_NOISE_DURATION_MS] - _params[S2_DURATION_MS]); // select when s2 will play by generating random number between 1ms and latest possible that still overlaps with white noise
        }
    }


    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        TRANSITION LIST -- checks conditions, moves to next state
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    if (checkQuit()) {return;} // always have this

    // check any conditions we care about, ie is the animal licking?
    checkLick(); // YOU WANT A LICK R and L function


    // define condition for leaving this state
    if (signedMillis() - _timer_init_trial >= _stateDurationsMS[_state]) {// Init_trial time allotted has elapsed -> White noise stimulus window
        if (_params[_DEBUG]) {sendMessage("Pre-cue delay successfully completed.");}
        // if noise trial _state = NOISE_TRIAL
        if (_is_S2) {
            _state = CUE_TRIAL; 						// Move to CUE_TRIAL state
        } else {
            _state = NOISE_TRIAL;						// Move to NOISE_TRIAL state
        }
        return;                                         // Exit Fx
    }

    _state = INIT_TRIAL;                            // No Command --> Cycle back to INIT_TRIAL
} // End INIT_TRIAL STATE ------------------------------------------------------------------------------------------------------------------------------------------


/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	NOISE_TRIAL - Noise Trial started.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void noise_trial() {
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        ACTION LIST -- initialize the new state
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    if (_state != _prevState) {                   		    // If ENTERING NOISE_TRIAL STATE:
        _prevState = _state;                                // Assign _prevState to NOISE_TRIAL _state
        sendMessage("$" + String(_state));                  // Send  HOST _state entry -- $2 (NOISE_TRIAL State)
        // Send event marker (house_lamp_off) to HOST with timestamp
        sendMessage("&" + String(EVENT_NOISE_TRIAL) + " " + String(signedMillis() - _exp_timer));
        _timer_noise_trial = signedMillis();                     // mark time when state was entered
        if (_params[_DEBUG]) {sendMessage("Noise Trial Started.");}
        // Do the initial business of the state -- something you only want to do once when you enter


    }

    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        TRANSITION LIST -- checks conditions, moves to next state
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    if (checkQuit()) {return;} // always have this

    // check any conditions we care about, ie is the animal licking?
    checkLick(); // YOU WANT A LICK R and L function

    // check if time has reached noise start, if so play noise until end of state time out
    long time_elapsed = signedMillis() - _timer_noise_trial;
    bool isWithinNoiseWindow = (time_elapsed >= _params[TIME_NOISE_START]) && (time_elapsed <= (_params[TIME_NOISE_START] + _params[WHITE_NOISE_DURATION_MS]));		// boolean to denote whether it is time to play the noise
    if (isWithinNoiseWindow) {
        playSound(true,false); // play noise alone
    }

    // define condition for leaving this state
    if (time_elapsed >= (_params[TIME_NOISE_START] + _params[WHITE_NOISE_DURATION_MS])) {// noise_trial time allotted has elapsed -> response window
        if (_params[_DEBUG]) {sendMessage("white noise trial successfully completed.");}
        _state = RESPONSE_WINDOW;                         // Move to RESPONSE_WINDOW state
        return;                                           // Exit Fx
    }

    _state = NOISE_TRIAL;                            // No Command --> Cycle back to INIT_TRIAL
} // End NOISE_TRIAL STATE ------------------------------------------------------------------------------------------------------------------------------------------


/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	CUE_TRIAL - Cue Trial started.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void cue_trial() {
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        ACTION LIST -- initialize the new state
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    if (_state != _prevState) {                   		    // If ENTERING CUE_TRIAL STATE:
        _prevState = _state;                                // Assign _prevState to CUE_TRIAL _state
        sendMessage("$" + String(_state));                  // Send  HOST _state entry -- $2 (CUE_TRIAL State)
        // Send event marker (house_lamp_off) to HOST with timestamp
        sendMessage("&" + String(EVENT_CUE_TRIAL) + " " + String(signedMillis() - _exp_timer));
        _timer_cue_trial = signedMillis();                     // mark time when state was entered
        if (_params[_DEBUG]) {sendMessage("Trial Started.");}
        // Do the initial business of the state -- something you only want to do once when you enter
    }


    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        TRANSITION LIST -- checks conditions, moves to next state
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    if (checkQuit()) {return;} // always have this

    // check any conditions we care about, ie is the animal licking?
    checkLick(); // YOU WANT A LICK R and L function

    // check if time has reached noise start, if so play noise until end of state time out
    long time_elapsed = signedMillis() - _timer_cue_trial;
    bool isWithinNoiseWindow = (time_elapsed >= _params[TIME_NOISE_START]) && (time_elapsed <= (_params[TIME_NOISE_START] + _params[WHITE_NOISE_DURATION_MS]));		// boolean to denote whether it is time to play the noise
    bool isWithinS2Window = (time_elapsed >= _time_S2_start) && (time_elapsed <= (_time_S2_start + _params[S2_DURATION_MS]));		// boolean to denote whether it is time to play the tone (S2

    if (isWithinNoiseWindow) { // if it is within time window to play noise
        if(isWithinS2Window) { // if it is within time window to play S2
            playSound(true,true); // play tone and noise
        }
        else {
            playSound(true,false); // play noise alone
        }
    }

    // define condition for leaving this state
    if (time_elapsed >= (_params[TIME_NOISE_START] + _params[WHITE_NOISE_DURATION_MS])) { // cue_trial time allotted has elapsed -> response window
        if (_params[_DEBUG]) {sendMessage("cue trial successfully completed.");}
        _state = RESPONSE_WINDOW;                         // Move to RESPONSE_WINDOW state
        return;                                           // Exit Fx
    }

    _state = CUE_TRIAL;                            // No Command --> Cycle back to CUE_TRIAL
} // End CUE_TRIAL STATE ------------------------------------------------------------------------------------------------------------------------------------------


/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	RESPONSE_WINDOW - response window started.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void response_window() {
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        ACTION LIST -- initialize the new state
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    if (_state != _prevState) {                   		    // If ENTERING RESPONSE_WINDOW STATE:

        _prevState = _state;                                // Assign _prevState to RESPONSE_WINDOW _state
        sendMessage("$" + String(_state));                  // Send  HOST _state entry -- $2 (CUE_TRIAL State)
        // Send event marker (house_lamp_off) to HOST with timestamp
        sendMessage("&" + String(EVENT_RESPONSE_WINDOW) + " " + String(signedMillis() - _exp_timer));
        _timer_response_window = signedMillis();                     // mark time when state was entered
        if (_params[_DEBUG]) {sendMessage("Trial Started.)");}

        setCueLED(true); // turn on cue LED to tell animal it is time to lick
        _toneSpoutLick = false; // reset lick variables
        _noiseSpoutLick = false; // reset lick variables
        // Do the initial business of the state -- something you only want to do once when you enter
    }


    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        TRANSITION LIST -- checks conditions, moves to next state
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    if (checkQuit()) {return;} // always have this
    checkLick(); // updates _leftLick and _rightLick global variables

    // update left and right lick counters and rates
    if (_leftLick) {
        _num_left++;
        _percent_left = _num_left / _num_trials;

    }
    if (_rightLick) {
        _num_right++;
        _percent_right = _num_right / _num_trials;
    }
    // check if the tone spout or noise spout has been licked
    if ((_leftLick && _params[S2_SPOUT] == 1) || (_rightLick && _params[S2_SPOUT] == 2)) { // if the animal licked the S2 spout
        _toneSpoutLick = true;
        _noiseSpoutLick = false;
        // if debugging, send message to HOST that the animal licked the tone spout
        if (_params[_DEBUG]) {sendMessage("Animal licked the tone spout.");}
    }
    if ((_leftLick && _params[S2_SPOUT] == 2) || (_rightLick && _params[S2_SPOUT] == 1)) { // if the animal licked the noise spout
        _toneSpoutLick = false;
        _noiseSpoutLick = true;
        // if debugging, send message to HOST that the animal licked the noise spout
        if (_params[_DEBUG]) {sendMessage("Animal licked the noise spout.");}
    }
    // define condition for leaving this state
    if (_is_S2 && _toneSpoutLick) { // if S2 was played and the animal licked the S2 spout
        _state = HIT; // send to HIT state
        setCueLED(false); // turn off cue LED
        return;
    }
    if (_is_S2 && _noiseSpoutLick) { // if S2 was played and the animal licked the noise spout
        _state = MISS; // send to MISS state
        setCueLED(false); // turn off cue LED
        return;
    }
    if (!_is_S2 && _toneSpoutLick) { // if S2 was not played and the animal licked the S2 spout
        _state = FALSE_ALARM; // send to FALSE_ALARM state
        setCueLED(false); // turn off cue LED
        return;
    }
    if (!_is_S2 && _noiseSpoutLick) { // if S2 was not played and the animal licked the noise spout
        _state = CORRECT_REJECT; // send to CORRECT_REJECT state
        setCueLED(false); // turn off cue LED
        return;
    }
    if (signedMillis() - _timer_response_window >= _params[RESPONSE_WINDOW_MS]) {// response window time has been exceeded
        if (_params[_DEBUG]) {sendMessage("mouse did not lick within response window.");}
        setCueLED(false); // turn off cue LED
        _state = NO_RESPONSE;                         // Move to NO_RESPONSE state
        return;
    }

    _state = RESPONSE_WINDOW;                            // No Command --> Cycle back to RESPONSE_WINDOW
} // End RESPONSE_WINDOW STATE ------------------------------------------------------------------------------------------------------------------------------------------

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    HIT - animal correctly detected S2.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void hit() {
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        ACTION LIST -- reward animal, update hit rate
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    if (_state != _prevState) {                          // If ENTERING HIT STATE:
        _prevState = _state;                            // Assign _prevState to HIT _state
        _resultCode = CODE_HIT;                         // Set _resultCode to CODE_HIT
        sendMessage("$" + String(_state));              // Send  HOST _state entry -- $2 (CUE_TRIAL State)
        // Send event marker (house_lamp_off) to HOST with timestamp
        sendMessage("&" + String(EVENT_HIT) + " " + String(signedMillis() - _exp_timer));
        _timer_hit = signedMillis();                     // mark time when state was entered
        if (_params[_DEBUG]) {sendMessage("mouse hit.)");}
        setCueLED(false); // turn off cue LED
        giveReward(_params[S2_SPOUT]); // dispense juice on the tone spout
        // send event with timestamp to host that reward was given on tone spout, accounting for if tone spout is the right or left
        if (_params[S2_SPOUT] == 1) {
            sendMessage("&" + String(EVENT_REWARD_LEFT) + " " + String(signedMillis() - _exp_timer));
        }
        if (_params[S2_SPOUT] == 2) {
            sendMessage("&" + String(EVENT_REWARD_RIGHT) + " " + String(signedMillis() - _exp_timer));
        }
        sendMessage("reward dispensed on tone spout"); // send message to host
        // if calibration mode is on, decrement SNR_PERCENT by SNR_STEP
        if (_params[CALIBRATION]) {
            _params[SNR_PERCENT] -= _params[SNR_STEP];
            if (_params[SNR_PERCENT] < 0) {_params[SNR_PERCENT] = 0;}
        }
    }
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      TRANSITION LIST -- checks conditions, moves to next state
     ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    if (checkQuit()) {return;} // always have this

    _state = INTERTRIAL;                            // go to intertrial state
    return;
} // End HIT STATE ------------------------------------------------------------------------------------------------------------------------------------------

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    MISS - animal did not detect S2.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void miss() {
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        ACTION LIST -- update miss rate
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    if (_state != _prevState) {                          // If ENTERING MISS STATE:
        _prevState = _state;                            // Assign _prevState to MISS _state
        _resultCode = CODE_MISS;                         // Set _resultCode to CODE_MISS
        sendMessage("$" + String(_state));              // Send  HOST _state entry -- $2 (CUE_TRIAL State)
        // Send event marker (house_lamp_off) to HOST with timestamp
        sendMessage("&" + String(EVENT_MISS) + " " + String(signedMillis() - _exp_timer));
        _timer_miss = signedMillis();                     // mark time when state was entered
        if (_params[_DEBUG]) {sendMessage("mouse missed.)");}

        setCueLED(false); // turn off cue LED
        
        // if calibration mode is on, increment SNR_PERCENT by SNR_STEP
        if (_params[CALIBRATION]) {
            _params[SNR_PERCENT] += _params[SNR_STEP];
            if (_params[SNR_PERCENT] > 100) {_params[SNR_PERCENT] = 100;}
        }
    }
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      TRANSITION LIST -- checks conditions, moves to next state
     ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    if (checkQuit()) {return;} // always have this

    _state = INTERTRIAL;                            // go to intertrial state
    return;
} // End MISS STATE ------------------------------------------------------------------------------------------------------------------------------------------

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    FALSE_ALARM - animal detected S2 when it was not played.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void false_alarm() {
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        ACTION LIST -- update false alarm rate
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    if (_state != _prevState) {                         // If ENTERING FALSE_ALARM STATE:
        _prevState = _state;                            // Assign _prevState to FALSE_ALARM _state
        _resultCode = CODE_FALSE_ALARM;                 // Set _resultCode to CODE_FALSE_ALARM
        sendMessage("$" + String(_state));              // Send  HOST _state entry -- $2 (CUE_TRIAL State)
        // Send event marker (house_lamp_off) to HOST with timestamp
        sendMessage("&" + String(EVENT_FALSE_ALARM) + " " + String(signedMillis() - _exp_timer));
        _timer_false_alarm = signedMillis();                     // mark time when state was entered
        if (_params[_DEBUG]) {sendMessage("mouse false alarmed.)");}

        setCueLED(false); // turn off cue LED
        
        // Do the initial business of the state -- something you only want to do once when you enter
    }
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      TRANSITION LIST -- checks conditions, moves to next state
     ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    if (checkQuit()) {return;} // always have this

    _state = INTERTRIAL;                            // go to intertrial state
    return;
} // End FALSE_ALARM STATE ------------------------------------------------------------------------------------------------------------------------------------------

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    CORRECT_REJECT - animal did not detect S2 when it was not played.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void correct_reject() {
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        ACTION LIST -- update correct reject rate
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    if (_state != _prevState) {                          // If ENTERING CORRECT_REJECT STATE:
        _prevState = _state;                            // Assign _prevState to CORRECT_REJECT _state
        _resultCode = CODE_CORRECT_REJECT;                         // Set _resultCode to CODE_CORRECT_REJECT
        sendMessage("$" + String(_state));              // Send  HOST _state entry -- $2 (CUE_TRIAL State)
        // Send event marker (house_lamp_off) to HOST with timestamp
        sendMessage("&" + String(EVENT_CORRECT_REJECT) + " " + String(signedMillis() - _exp_timer));
        _timer_correct_reject = signedMillis();                     // mark time when state was entered
        if (_params[_DEBUG]) {sendMessage("mouse correctly rejected.)");}

        setCueLED(false); // turn off cue LED
        
        // Do the initial business of the state -- something you only want to do once when you enter
        giveReward(_params[NOISE_SPOUT]); // dispense juice on the noise spout
        sendMessage("reward dispensed on noise spout"); // send message to host
    }
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      TRANSITION LIST -- checks conditions, moves to next state
     ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    if (checkQuit()) {return;} // always have this

    _state = INTERTRIAL;                            // go to intertrial state
    return;
} // End CORRECT_REJECT STATE ------------------------------------------------------------------------------------------------------------------------------------------

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    NO_RESPONSE - animal did not lick either spout.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void no_response() {
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        ACTION LIST -- update no response rate
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    if (_state != _prevState) {                         // If ENTERING NO_RESPONSE STATE:
        _prevState = _state;                            // Assign _prevState to NO_RESPONSE _state
        _resultCode = CODE_NO_RESPONSE;                         // Set _resultCode to CODE_NO_RESPONSE
        sendMessage("$" + String(_state));              // Send  HOST _state entry -- $2 (CUE_TRIAL State)
        // Send event marker (house_lamp_off) to HOST with timestamp
        sendMessage("&" + String(EVENT_NO_RESPONSE) + " " + String(signedMillis() - _exp_timer));
        _timer_no_response = signedMillis();                     // mark time when state was entered
        if (_params[_DEBUG]) {sendMessage("mouse did not respond.)");}

        setCueLED(false); // turn off cue LED
        setHouseLamp(false); // turn off house lamp

        
        // Do the initial business of the state -- something you only want to do once when you enter
    }
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      TRANSITION LIST -- checks conditions, moves to next state
     ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    if (checkQuit()) {return;} // always have this
    if (signedMillis() - _timer_no_response > _params[PENALTY_DURATION_MS]) { // if penalty time has elapsed
        _state = INTERTRIAL;                            // go to intertrial state
        return;
    }
    _state = NO_RESPONSE;                            // cycle back to NO_RESPONSE state
    return;
} // End NO_RESPONSE STATE ------------------------------------------------------------------------------------------------------------------------------------------
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	INTERTRIAL - Enforced ITI with Data Writing and Initialization of new parameters delivered from host
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    void intertrial() {
        /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            ACTION LIST -- initialize the new state
        ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
        static bool isParamsUpdateStarted;              // Initialize tracker of new param reception from HOST - true when new params received
        static bool isParamsUpdateDone;                 // Set to true upon receiving confirmation signal from HOST ("Over")
        if (_state != _prevState) {                     // If ENTERTING ITI:
            _timer_intertrial = signedMillis();              // Start ITI timer
            setHouseLamp(true);                              // House Lamp ON (if not already)
            setCueLED(false);                                // Cue LED OFF
            _prevState = _state;                             // Assign _prevState to ITI _state
            sendMessage("$" + String(_state));               // Send HOST $7 (ITI State)
            // Send event marker (ITI) to HOST with timestamp
            sendMessage("&" + String(EVENT_INTERTRIAL) + " " + String(signedMillis() - _exp_timer));

            // compute behavior rates

            _percent_left = _num_left / _num_trials;
            _percent_right = _num_right / _num_trials;
            _percent_hit = _num_hits / _num_trials;
            _percent_miss = _num_misses / _num_trials;
            _percent_false_alarm = _num_false_alarms / _num_trials;
            _percent_correct_reject = _num_correct_rejects / _num_trials;
            _percent_correct = (_num_hits + _num_correct_rejects) / _num_trials;
            _percent_incorrect = (_num_misses + _num_false_alarms) / _num_trials;

            // Reset state variables
            _leftLick = false;
            _rightLick = false;

            //=================== INIT HOST COMMUNICATION=================//
            isParamsUpdateStarted = false;                      // Initialize HOST param message monitor Start
            isParamsUpdateDone = false;                         // Initialize HOST param message monitor End

            //=================== SEND RESULT CODE=================//
            if (_resultCode > -1) {                       // If result code exists...
                sendMessage("`" + String(_resultCode));           // Send result to HOST
                _resultCode = -1;                                 // Reset result code to null state
            }

            //------------------------DEBUG MODE--------------------------//
            if (_params[_DEBUG]) {sendMessage("Intertrial.");}
            //----------------------end DEBUG MODE------------------------//
        }


        /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            TRANSITION LIST -- checks conditions, moves to next state
        ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
        if (checkQuit()) {return;}
        checkLick();

        // this is reserved code for changing params, dont touch ----------
        if (_command == 'P') {                          // Received new param from HOST: format "P _paramID _newValue" ('P' for Parameters)
            isParamsUpdateStarted = true;                   // Mark transmission start. Don't start next trial until we've finished.
            _params[_arguments[0]] = _arguments[1];         // Update parameter. Serial input "P 0 1000" changes the 1st parameter to 1000.
            _state = INTERTRIAL;                            // Return -> ITI
            if (_params[_DEBUG]) {
                sendMessage("Parameter " + String(_arguments[0]) + " changed to " + String(_arguments[1]));
            }
            return;                                         // Exit Fx
        }

        if (_command == 'O') {                          // HOST transmission complete: HOST sends 'O' for Over.
            isParamsUpdateDone = true;                      // Mark transmission complete.
            _state = INTERTRIAL;                            // Return -> ITI
            return;                                         // Exit Fx
        }
        // ----------

        if (signedMillis() - _timer_intertrial >= _params[ITI_DURATION_MS] && (isParamsUpdateDone || !isParamsUpdateStarted))  { // End when ITI ends. If param update initiated, should also wait for update completion signal from HOST ('O' for Over).
            _state = INIT_TRIAL;                                 // Move -> READY state
            return;                                         // Exit Fx
        }

        _state = INTERTRIAL;                            // No Command -> Cycle back to ITI
    } // End ITI---------------------------------------------------------------------------------------------------------------------




/*****************************************************
	DEFINE TRANSITION-STATE REDUNDANCY FXS
*****************************************************/

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	Check for Quit Command
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    bool checkQuit() {
        if (_command == 'Q')  {                          // HOST: "QUIT" -> IDLE_STATE
            _state = IDLE_STATE;                             // Set IDLE_STATE
            return true;                                          // Exit Fx
        }
        else {
            return false;
        }
    } // end checkQuit---------------------------------------------------------------------------------------------------------------------


/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	Check for Lick Actions
	in previous code, if there's a state-changing lick, checkLick returns true, otherwise returns false
    currently check lick runs getLickState and always returns false
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    void checkLick() {
        _prevLeftLick = _leftLick;
        _prevRightLick = _rightLick;
        getLickState();                                 // Get lick state, updating _leftLick and _rightLick
        // check to see if getLickState returns a different result on a second execution, indicating a new lick
        if (_leftLick && !_prevLeftLick) {            // If new left lick
            // send event marker (left lick) to HOST with timestamp
            sendMessage("&" + String(EVENT_LICK_LEFT) + " " + String(signedMillis() - _exp_timer));
        }
        if (_rightLick && !_prevRightLick) {          // If new right lick
            // send event marker (right lick) to HOST with timestamp
            sendMessage("&" + String(EVENT_LICK_RIGHT) + " " + String(signedMillis() - _exp_timer));
        }
        // go through what should happen if the mouse licks in each state
        
    } // end checkLick---------------------------------------------------------------------------------------------------------------------
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   Update recent outcomes and calculate performance
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    void updatePerformance() {

        // shift recent outcomes array to the left
        for (int i = 0; i < _params[MOVING_AVG_WINDOW] - 1; i++) {
            _recentOutcomes[i] = _recentOutcomes[i + 1];
        }
        _recentOutcomes[_params[MOVING_AVG_WINDOW] - 1] = _state; // add current state to end of array

        // calculate what % of _recentOutcomes are hits
        int numHits = 0;
        for (int i = 0; i < _params[MOVING_AVG_WINDOW]; i++) {
            if (_recentOutcomes[i] == HIT) {
                numHits++;
            }
        }
        _recentHitRate = numHits / _params[MOVING_AVG_WINDOW];

        // calculate what % of _recentOutcomes are misses
        int numMisses = 0;
        for (int i = 0; i < _params[MOVING_AVG_WINDOW]; i++) {
            if (_recentOutcomes[i] == MISS) {
                numMisses++;
            }
        }
        _recentMissRate = numMisses / _params[MOVING_AVG_WINDOW];

        // calculate what % of _recentOutcomes are false alarms
        int numFalseAlarms = 0;
        for (int i = 0; i < _params[MOVING_AVG_WINDOW]; i++) {
            if (_recentOutcomes[i] == FALSE_ALARM) {
                numFalseAlarms++;
            }
        }
        _recentFalseAlarmRate = numFalseAlarms / _params[MOVING_AVG_WINDOW];

        // calculate what % of _recentOutcomes are correct rejects
        int numCorrectRejects = 0;
        for (int i = 0; i < _params[MOVING_AVG_WINDOW]; i++) {
            if (_recentOutcomes[i] == CORRECT_REJECT) {
                numCorrectRejects++;
            }
        }
        _recentCorrectRejectRate = numCorrectRejects / _params[MOVING_AVG_WINDOW];

        // calculate what % of _recentOutcomes are no responses
        int numNoResponses = 0;
        for (int i = 0; i < _params[MOVING_AVG_WINDOW]; i++) {
            if (_recentOutcomes[i] == NO_RESPONSE) {
                numNoResponses++;
            }
        }
        _recentNoResponseRate = numNoResponses / _params[MOVING_AVG_WINDOW];


        // use _state variable to update global performance trackers
        _num_trials++; // increment number of trials
        if (_state == HIT) { // animal licked tone spout when tone was played
            _num_hits++;
            _num_correct++;
            if (_params[S2_SPOUT] == 1) { // if tone spout is on the left
                _num_left++; // mark that the animal licked the tone spout (left)
            }
            else if (_params[S2_SPOUT] == 2) { // if tone spout is on the right
                _num_right++; // mark that the animal licked the tone spout (right)
            }
        }
        else if (_state == MISS) { // animal licked noise spout when tone was played
            _num_misses++;
            _num_incorrect++;
            if (_params[S2_SPOUT] == 1) { // if tone spout is on the left
                _num_right++; // mark that the animal licked the noise spout (right)
            }
            else if (_params[S2_SPOUT] == 2) { // if tone spout is on the right
                _num_left++; // mark that the animal licked the noise spout (left)
            }
        }
        else if (_state == FALSE_ALARM) { // animal licked tone spout when noise was played
            _num_false_alarms++;
            _num_incorrect++; 
            if (_params[S2_SPOUT] == 1) { // if tone spout is on the left
                _num_left++; // mark that the animal licked the tone spout (left)
            }
            else if (_params[S2_SPOUT] == 2) { // if tone spout is on the right
                _num_right++; // mark that the animal licked the tone spout (right)
            }
        }
        else if (_state == CORRECT_REJECT) { // animal licked noise spout when noise was played
            _num_correct_rejects++;
            _num_correct++;
            if (_params[S2_SPOUT] == 1) { // if tone spout is on the left
                _num_right++; // mark that the animal licked the noise spout (right)
            }
            else if (_params[S2_SPOUT] == 2) { // if tone spout is on the right
                _num_left++; // mark that the animal licked the noise spout (left)
            }
        }
        // calculate performance percentages
        _percent_left = 100 * _num_left / _num_trials;
        _percent_right = 100 * _num_right / _num_trials;
        _percent_correct = 100 * _num_correct / _num_trials;
        _percent_incorrect = 100 * _num_incorrect / _num_trials;
        _percent_hit = 100 * _num_hits / _num_trials;
        _percent_miss = 100 * _num_misses / _num_trials;
        _percent_correct_reject = 100 * _num_correct_rejects / _num_trials;
        _percent_false_alarm = 100 * _num_false_alarms / _num_trials;
        _percent_no_response = 100 * _num_no_responses / _num_trials;


    }



/*****************************************************
	HARDWARE CONTROLS
*****************************************************/

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Dispense Reward on chosen spout
 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    void giveReward(int spout) {
        if (spout == 1) { // If left spout
            digitalWrite(PIN_LEFT_REWARD, HIGH);
            delay(_params[REWARD_DURATION_MS]);
            digitalWrite(PIN_LEFT_REWARD, LOW);
        }
        else if (spout == 2) { // If right spout
            digitalWrite(PIN_RIGHT_REWARD, HIGH);
            delay(_params[REWARD_DURATION_MS]);
            digitalWrite(PIN_RIGHT_REWARD, LOW);
        }
    } // end giveReward---------------------------------------------------------------------------------------------------------------------


/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	Set House Lamp (ON/OFF)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    void setHouseLamp(bool turnOn) {
        if (turnOn) {
            digitalWrite(PIN_HOUSE_LAMP, HIGH);
        }
        else {
            digitalWrite(PIN_HOUSE_LAMP, LOW);
        }
    } // end Set House Lamp---------------------------------------------------------------------------------------------------------------------

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	Set Cue LED (ON/OFF)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    void setCueLED(bool turnOn) {
        if (turnOn) {
            digitalWrite(PIN_LED_CUE, HIGH);
        }
        else {
            digitalWrite(PIN_LED_CUE, LOW);
        }
    } // end Set Cue LED---------------------------------------------------------------------------------------------------------------------


/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	GET LICK STATE (True/False - Boolean)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    void getLickState() {
        if (digitalRead(PIN_LEFT_LICK) == HIGH) {_leftLick = true;} else {_leftLick = false;} // update left lick global variable
        if (digitalRead(PIN_RIGHT_LICK) == HIGH) {_rightLick = true;} else {_rightLick = false;} // update right lick global variable
    } // end Get Lick State---------------------------------------------------------------------------------------------------------------------

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	PLAY SOUND (Choose event from Enum list and input the frequency for that event)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    void playSound(bool playNoise, bool playTone) {
        float SNR = _params[SNR_PERCENT] / 100.0; // get the SNR value from the parameter list
        float noiseVal = playNoise * (1 - SNR) * random(0, 255); // random value between 0 and 4095 to create white noise
        float toneVal = playTone * SNR * 127.0 * sin(2.0 * 3.14159 * TONE_S2 * (micros()) / 1000000.0) + 127.0; // calculate the value of the tone
        int audioVal = (int)noiseVal + (int)toneVal; // calculate the value of the audio signal
        analogWrite(PIN_SPEAKER, audioVal); // output the audio signal to the speaker

    } // end Play Sound---------------------------------------------------------------------------------------------------------------------

/*****************************************************
	SERIAL COMMUNICATION TO HOST
*****************************************************/

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	SEND MESSAGE to HOST
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    void sendMessage(String message)   // Capital (String) because is defining message as an object of type String from arduino library
    {
        Serial.println(message);
    } // end Send Message---------------------------------------------------------------------------------------------------------------------

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	GET COMMAND FROM HOST (single character)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    char getCommand(String message)
    {
        message.trim();                 // Remove leading and trailing white space
        return message[0];              // Parse message string for 1st character (the command)
    } // end Get Command---------------------------------------------------------------------------------------------------------------------

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	GET ARGUMENTS (of the command) from HOST (2 int array)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    void getArguments(String message, int *_arguments)  // * to initialize array of strings(?)
    {
        _arguments[0] = 0;              // Init Arg 0 to 0 (reset)
        _arguments[1] = 0;              // Init Arg 1 to 0 (reset)

        message.trim();                 // Remove leading and trailing white space from MESSAGE

        //----Remove command (first character) from string:-----//
        String parameters = message;    // The remaining part of message is now "parameters"
        parameters.remove(0,1);         // Remove the command character and # (e.g., "P#")
        parameters.trim();              // Remove any spaces before next char

        //----Parse first (optional) integer argument-----------//
        String intString = "";          // init intString as a String object. intString concatenates the arguments as a string
        while ((parameters.length() > 0) && (isDigit(parameters[0])))
        {                               // while the first argument in parameters has digits left in it unparced...
            intString += parameters[0];       // concatenate the next digit to intString
            parameters.remove(0,1);           // delete the added digit from "parameters"
        }
        _arguments[0] = intString.toInt();  // transform the intString into integer and assign to first argument (Arg 0)


        //----Parse second (optional) integer argument----------//
        parameters.trim();              // trim the space off of parameters
        intString = "";                 // reinitialize intString
        while ((parameters.length() > 0) && (isDigit(parameters[0])))
        {                               // while the second argument in parameters has digits left in it unparced...
            intString += parameters[0];       // concatenate the next digit to intString
            parameters.remove(0,1);           // delete the added digit from "parameters"
        }
        _arguments[1] = intString.toInt(); // WAS PREVIOUSLY: intString.toInt();  // transform the intString into integer and assign to second argument (Arg 1)
    } // end Get Arguments---------------------------------------------------------------------------------------------------------------------


/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	INIT HOST (send States, Names/Value of Parameters to HOST)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    void hostInit()
    {
        //------Send state names and which states allow parameter update-------//
        for (int iState = 0; iState < _NUM_STATES; iState++)
        {// For each state, send "@ (number of state) (state name) (0/1 can update params)"
            sendMessage("@ " + String(iState) + " " + _stateNames[iState] + " " + String(_stateCanUpdateParams[iState]));
        }

        //-------Send event marker codes---------------------------------------//
        /* Note: "&" reserved for uploading new event marker and timestamp. "+" is reserved for initially sending event marker names */
        for (int iCode = 0; iCode < _NUM_OF_EVENT_MARKERS; iCode++)
        {// For each state, send "+ (number of event marker) (event marker name)"
            sendMessage("+ " + String(iCode) + " " + _eventMarkerNames[iCode]); // Matlab adds 1 to each code # to index from 1-n rather than 0-n
        }

        //-------Send param names and default values---------------------------//
        for (int iParam = 0; iParam < _NUM_PARAMS; iParam++)
        {// For each param, send "# (number of param) (param names) (param init value)"
            sendMessage("# " + String(iParam) + " " + _paramNames[iParam] + " " + String(_params[iParam]));
        }
        //--------Send result code interpretations.-----------------------------//
        for (int iCode = 0; iCode < _NUM_RESULT_CODES; iCode++)
        {// For each result code, send "* (number of result code) (result code name)"
            sendMessage("* " + String(iCode) + " " + _resultCodeNames[iCode]);
        }
        sendMessage("~");                           // Tells PC that Arduino is on (Send Message is a LF Function)
    }

    long signedMillis()
    {
        long time = (long)(millis());
        return time;
    }
