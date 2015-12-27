/*******************************************************************
   MAINSTREAM_DIVERTER_V4xx.C   Colombo Semi-Automatic Pneumatic Tube Transfer System

   Purpose:    Remote station control program.  Processes local i/o
               and main_station control commands.

   Target:     Z-World BL2500 Rabbit

   Language:   Dynamic C v9.x

   Created:    Aug 16, 2008 by Mike Schwedt (C) MS Technology Solutions

   History:
   12-Jun-06   v301 Original version created from RMOT235.c
   21-Sep-06   v302 skipped version to sync with main
   21-Sep-06   v303 Invert logic state of read-bank inputs
   24-Sep-06   v304 Further refinement of I/O, add InUse/CIC/Flashing
   22-Oct-06   v307 Sync version with Main.  Integrate 4 open-door alerts.
   04-Jan-07   v311 Sync version with Main.  Extend command to 5 data bytes.
   08-Jan-07   v312 Do not invert requestToSend inputs
   10-Jan-07   v313 Setup for using ALL_DEVICES = 0xFF in .devAddr
   14-Jan-07   v314 Integrate turnaround head diverter
   19-Jan-07   v316 Diverter map missing [8] and causing out-of-bounds crash
   24-Jan-07   v318 Continued improvements
   25-Jan-07   v319 Enable TX control a bit sooner to improve communications
                    HD & slave both known at station 8, so ready/not-ready commands now return devAddr
   27-Jan-07   v320 fix bug returning response with device address (THIS_LPLC) -> (THIS_LPLC-1)
                    Clean up implentation & reporting of latched arrival interrupt
   28-Jan-07   v321 Delay response commands to give main time to be ready
   01-Feb-07   v322 Switch from standard blower to heavy duty APU
   03-Feb-07   v323 Initialize blower when starting up head diverter
   11-Feb-07   v326 Handle failure of readDigBank when rn1100 is failed/missing
                    Fix bug in sending arrival status to main
   27-Mar-07   v342 Invert logic state of carrier in chamber and arrival optics
   31-Mar-07   v343 Revert to original logic of CIC and arrival optics
                    Set digital inputs low upon loss of connection to RN1100
    10-Apr-07   v346 Set diverter addr-2 passthrough to port 4 instead of 1
   03-May-07   v347 Delay clearing of arrival latch for 1 second
	04-May-07   v348 Fix handling of latched arrival
   24-May-07   v349 Fix bug in CIC lights not working, no IO Shift
   05-Jun-07   v350 Don't clear latchedCarrierArrival until transaction is complete (remove 1 second clear)
   15-Jul-07   v351 Improve exercise_io so that arrival alert and inuse light up
   					  Report latched arrival only when state is wait_for_remote_arrive
                    Implement feature for remote alternate address request-to-send, to send to master or slave
   23-Jul-07   v352 Implement compile-time selectable blower interface
   25-Jul-07   v353 Bug fix for standard blower in head diverter setup (blwrConfig must equal 1)
   27-Jul-07   v354 Shore up possible weakness in head diverter optic detection
   					  Head diverter arrival optic is on input 0
   31-Jul-07   v355 Don't use interrupt for head diverter, raw arrival input only
   29-Aug-07   v356 DEVELOPMENT VERSION
   29-Dec-07   v365 Released Version - sync w/ main
   					  Get blower selection by main menu setting
                    Get diverter port mapping configuration by main menu setting
                    Support for point to point configuration, remote with one station, no diverter, re-mapped I/O
   13-Apr-08   v366 Support for reporting blower errors
                    Reduce blower timeout to 12 seconds so it happens before carrier lift timeout
	16-Apr-08   v367 Fix mistake in initBlower:   blowerPosition==0 --> blowerPosition()==0
                    Return blowerPosition to the main station for head diverters with APU
   16-Aug-08   v400 Created from MAINSTREAM_REMOTE_V367.C
   16-Nov-08   v403 Added diverter configuration 8, two position diverter, operates station 2, passes 3-7
   06-Dec-08   v404 Added handling of secure transaction
   21-Dec-08   v405 Misc improvements to secure handling
   03-Mar-09   v409 Bump version to 409
   					  Add communication of secure card parameters
   10-Apr-10   v410 Create new diverter configurations
   						Head diverter using 3 ports: 1 for main, 2 for two sub-stations
                     Standard diverter with no ports and two stations used with new head diverter config
   18-Apr-10   v411 Fixes to allow for non-tcpip configurations
   11-May-10   v412 Fix bug in using uninitialized msg_address in get_command - introduced in remote v3.19, jan 2007
                    Allow the use of diverter_map 0 when calling set_diverter
   26-Mar-11	v413 Pass Head Diverter arrival state always, not just in-process to allow for blocked optic msg
                    When notified of purge mode, flash in-use and cic alternating
   02-May-11   v414 Seems that as TRUE == 0x01 instead of 0xFF then passing latchCarrierArrival as TRUE does not satisfy
                    the expected logic at the main system controller (needs to be a specific station)
   06-Jun-11   v415 Fixed CIC outputs.  Missed the shifting of CIC outputs when it got reworked.
   18-Aug-14   v415a Added diverter configuration 14 for a virtual station
   31-Oct-15   v415b Point 2 Point; Delay enable of arrival interrupt for 2 seconds

***************************************************************/
#memmap xmem  // Required to reduce root memory usage
#class auto
#define FIRMWARE_VERSION "DIVERTER V4.15"
#define VERSION                       15

// compile as "STANDARD" or "POINT2POINT" or "DIVERTERONLY" remote
// STANDARD are directly connected where DIVERTERONLY are using dedicated remotes (TCPIP)
#define RT_STANDARD     1
#define RT_POINT2POINT  2
#define RT_DIVERTERONLY 3
#define REMOTE_TYPE RT_STANDARD
//#define REMOTE_TYPE RT_DIVERTERONLY
//#define REMOTE_TYPE RT_POINT2POINT

//int lastcommand; // for debug
//#define USE_TCPIP 1

// define PRINT_ON for additional debug printing via printf
// #define PRINT_ON 1
#use "bl25xx.lib"          Controller library
#use "rn_cfg_bl25.lib"     Configuration library
#use "rnet.lib"            RabbitNet library
#use "rnet_driver.lib"     RabbitNet library
#use "rnet_keyif.lib"      RN1600 keypad library
#use "rnet_lcdif.lib"      RN1600 LCD library
#define USE_BLOWER
#use "MAINSTREAM.lib"      Mainstream v4xx function library

// RabbitNet RN1600 Setup
//int DevRN1600;                    // Rabbit Net Device Number Keypad/LCD
int DevRN1100;                   // Rabbit Net Device Number Digital I/O
//configure to sinking safe state
#define OUTCONFIG 0xFFFF
#define SHOW_CYCLE_TIME

// Setup interrupt handling for carrier arrival input
// Set equal to 1 to use fast interrupt in I&D space
#define FAST_INTERRUPT 0
char latchCarrierArrival;
void my_isr1();
void arrivalEnable(char how);
unsigned long arrivalEnableDelay;  // to delay enabling of arrival signal

// Communications data structures
#define NUMDATA   5
struct iomessage     /* Communications command structure */
{
   char device;
   char command;
   char station;
   char data[NUMDATA];
};
#define NUMUDPDATA 15
struct UDPmessageType  // UDP based messages
{
	char device;
   char devType;  // M for main station
   char command;  // can be heartbeat, event, transaction
   char station;
   unsigned long timestamp;
   char data[NUMUDPDATA];
};
// Define communications using TCP/IP (UDP)
#ifdef USE_TCPIP
//	#define UDP_DEBUG
// #define DCRTCP_VERBOSE
	#define TCPCONFIG 106         // need to use 100+ to use static IP defined here
	#define MAX_UDP_SOCKET_BUFFERS 1
	#define DISABLE_TCP		// Not using TCP
   #define MY_STATIC_IP "192.168.0.10"
   #define MY_BASE_IP "192.168.0.2%d"
   char myIPaddress[18];  // string to hold runtime IP address
	#define MY_IP_NETMASK "255.255.255.0"
	#define LOCAL_PORT   1234      // for inbound messages
//	#define REMOTE_IP    "255.255.255.255" //255.255.255.255" /*broadcast*/
	#define REMOTE_PORT  1235      // for outbound messages
	#use "dcrtcp.lib"
	udp_Socket sock;
#endif
// still define the function prototypes
	int sendUDPHeartbeat(char sample);
	int sendUDPcommand(struct UDPmessageType message);
	int getUDPcommand(struct UDPmessageType *message);
	void processUDPcommand(struct UDPmessageType message);
	void processUDPio(void);
	void monitorUDPactivity(char deviceID);
   void sendUDPCardParameters(void);


// Including parameter block structure
// Parameters are NOT read to / written from FLASH
struct par_block_type
{  // These parameters are defined on the fly at run-time
   char opMode;      // STD_REMOTE or HEAD_DIVERTER
   char subStaAddressing; // to indicate if substation can select slave for destination
   char blowerType;
   char portMapping;
   char cardL3Digits;     // Constant left 3 digits of valid secure cards
	unsigned long cardR9Min;  // Secure card ID minimum value for the right 9 digits
   unsigned long cardR9Max;  // Secure card ID maximum value for the right 9 digits
} param;
#define STD_REMOTE    0
#define HEAD_DIVERTER 1
#define HEAD_DIVERTER2 2
char mainStation;
#define SLAVE   0x08      // Station number of the slave
#define SLAVE_b 0x80      // Bit representation

#define NUM_STATIONS 8
char THIS_DEVICE;         // board address for communications and setup
char MY_STATIONS;       // to mask supported stations
char IO_SHIFT;          // to map expansion board i/o
char diverter_map[9];   // to assign diverter positions to stations
char activeStations;    // to track who is out there and active

// Global variables
unsigned long thistime, lasttime, lost;
char securePIN_hb, securePIN_lb;
struct {
	unsigned long id;
   char time;
} secureCard[10];
char secureAck;
char nextSecureStation(void);
//unsigned long secureCardID[10];
/* Declare general function prototypes */
void msDelay(unsigned int msec);
//char bit2station(char station_b);// station_b refers to bit equivalent: 0100
//char station2bit(char station);  // station refers to decimal value: 3 --^
void init_counter(void);
void process_local_io(void);
void processCycleTime(void);
void set_remote_io(char *remotedata);
void init_io(void);
void exercise_io(void);
void send_response(struct iomessage message);
char get_command(struct iomessage *message);
void process_command(struct iomessage message);
void enable_commands(void);
void arrival_alert(char code, char station);
char isMyStation(char station);
void toggleLED(char LEDDev);
void maintenance(void);
char Timeout(unsigned long start_time, unsigned long duration);
int  readParameterSettings(void);
int  writeParameterSettings(void);

/* Declare local and expansion bus input prototypes */
char carrierInChamber(char station_mask);
char doorClosed(char station_mask);
char carrierArrival(char station_mask);
char requestToSend(char station_mask);
char requestToSend2(char station_mask);
//char diverter_pos(void);
// Declare global equivalents for inputs coming from dedicated remotes (UDP)
char g_carrierInChamber;
char g_doorClosed;
char g_carrierArrival;
char g_requestToSend;
char g_requestToSend2;

/* Declare local and expansion bus output prototypes */
char rts_latch;      // Used for latching requestToSend
char rts2_latch;     // Used for latching 2nd requestToSend (to slave)
void inUse(char how, char station_b);
void alert(char how, char station_b);
void alarm(char how);
// void diverter(char how);
void setDiverter(char station);
void processDiverter(void);
void setDiverterMap(char portMap);

// Declare watchdog and powerlow handlers
char watchdogCount(void);
char powerlowCount(void);
void incrementWDCount(void);
void incrementPLCount(void);
void loadResetCounters(void);

// Digital I/O definitions
char outputvalue [6];   // buffer for some outputs
char remoteoutput [5];  // buffer for main commanded outputs
// INPUTS
int readDigInput(char channel);
int readDigBank(char bank);  // banks 0,1 on Coyote; banks 2,3 on RN1100
// dio_ON|OFF must be used ONLY in primatives readDigInput and setDigOutput
// they reflect the real state of the logic inputs and outputs
#define dio_ON  0
#define dio_OFF 1
#define di_HDArrival					  readDigInput(0)
// Check for and setup point to point configuration
#if REMOTE_TYPE == RT_POINT2POINT
	#warnt "Compiling as point-to-point configuration"
	#define di_carrierArrival          readDigInput(0)
	#define di_requestToSend           readDigInput(1)
	#define di_requestToSend2          0
	#define di_doorClosed              readDigInput(2)
	#define di_carrierInChamber        readDigInput(3)
#endif
#if REMOTE_TYPE == RT_STANDARD
	#define di_carrierArrival          (((readDigBank(0)) & 0x1E) >> 1)
	#define di_carrierInChamber        (((readDigBank(1)) & 0xF0) >> 4)
	#define di_requestToSend           (readDigBank(2) & 0x0F)
	#define di_requestToSend2          (readDigBank(3) & 0x0F)
	#define di_doorClosed              ((readDigBank(2) & 0xF0) >> 4)
#endif
#if REMOTE_TYPE == RT_DIVERTERONLY
	// define di_carrierArrival only to support definition in my_isr1
	#define di_carrierArrival          readDigInput(0)
#endif
#define di_remoteAddress           ((readDigBank(0) & 0xE0) >> 5)
#define di_diverterPos             (readDigBank(1) & 0x0F)
#define di_shifterPosIdle          readDigInput(12)
#define di_shifterPosPrs           readDigInput(13)
#define di_shifterPosVac           readDigInput(14)

// OUTPUTS
void setDigOutput(int channel, int value);
#define do_shift 0
// Check for and setup point to point configuration
#if REMOTE_TYPE == RT_POINT2POINT
	#define do_doorAlert(which,value)    setDigOutput(0,value)
	#define do_arrivalAlert(which,value) setDigOutput(1,value)
	#define do_CICLight(which,value)     setDigOutput(2,value)
	#define do_inUseLight(which,value)   setDigOutput(3,value)
   // inUseLight on output 3,4,7 causes RS485 transmitter to enable (PA4)
   // Root cause was use of static CHAR elements in maintenance() instead of static INT
#endif
#if REMOTE_TYPE == RT_STANDARD
	#define do_doorAlert(which,value)    setDigOutput(0+do_shift+which,value)
	#define do_arrivalAlert(which,value) setDigOutput(8+do_shift+which,value)
	#define do_inUseLight(which,value)   setDigOutput(12+do_shift+which,value)
	#define do_inUse2Light(which,value)  setDigOutput(20+do_shift+which,value)
	#define do_CICLight(which,value)     setDigOutput(16+do_shift+which,value)
#endif
#define do_diverter(value)           setDigOutput(4+do_shift,value)
//#define do_alarm(value)              setDigOutput(xx+do_shift,value)
#define do_blower(value)             setDigOutput(5+do_shift,value)
#define do_blowerVac(value)          setDigOutput(6+do_shift,value)
#define do_blowerPrs(value)          setDigOutput(7+do_shift,value)


#define ALARM_MASK           0x20
// #define beep(value)                rn_keyBuzzerAct(DevRN1600, value, 0)
void inUse(char how, char station_b);
void inUse2(char how, char station_b);
void alarm(char how);

/* "how" types & values and other constants */
#define ON           0xFF
#define OFF          0x00
#define FLASH        0x01
#define READY        0x01
#define NOT_READY    0xFF
//#define TRUE         0xFF
//#define FALSE        0x00
#define aaRESET      0x00
#define aaSET        0x01
#define aaTEST       0x02
#define ToREMOTE     0x01
#define ToMAIN       0x02
#define ALL_DEVICES  0xFF

/* Serial communications send and receive commands */
#define NAK 0x15
#define ACK 0x06
#define STX 0x02
#define ETX 0x03
#define DINBUFSIZE 63
#define DOUTBUFSIZE 63
#define SET_DIVERTER       'A'
#define DIVERTER_STATUS    'B'
#define REMOTE_PAYLOAD     'C'
#define SECURE_REMOVAL     'C'
#define ACK_SECURE_REMOVAL 'D'
#define RETURN_INPUTS      'E'
#define INPUTS_ARE         'E'
#define SET_OUTPUTS        'G'
#define CLEAR_OUTPUTS      'H'
#define SET_OPMODE         'I'
#define ARE_YOU_READY      'J'
#define RETURN_EXTENDED    'K'
#define SET_TRANS_COUNT    'L'
#define SET_DATE_TIME      'M'
#define SET_STATION_NAME   'N'
#define CANCEL_PENDING     'O'
#define TAKE_COMMAND       'P'
#define DISPLAY_MESSAGE    'Q'
#define SET_PHONE_NUMS     'R'
#define SET_PARAMETERS     'S'
#define SET_CARD_PARAMS    'T'
#define TRANS_COMPLETE     'X'
#define RESET              'Y'
#define MALFUNCTION        'Z'

// Define all system states - MUST BE IDENTICAL TO MAIN
#define  IDLE_STATE             0x01
#define  PREPARE_SEND           0x02
#define  WAIT_FOR_DIVERTERS     0x03
#define  BEGIN_SEND             0x04
#define  WAIT_FOR_MAIN_DEPART   0x05
#define  WAIT_FOR_REM_ARRIVE    0x06
#define  HOLD_TRANSACTION       0x07
#define  WAIT_FOR_REM_DEPART    0x08
#define  WAIT_FOR_MAIN_ARRIVE   0x09
#define  WAIT_FOR_TURNAROUND    0x0A
#define  SHIFT_TURNAROUND       0x0B
#define  CANCEL_STATE           0x0C
#define  MALFUNCTION_STATE      0x0D
#define  FINAL_COMMAND          0x0E

// Define transaction direction constants
#define  DIR_SEND             1
#define  DIR_RETURN           2

/* Declare miscellaneous timeouts */
#define DIVERTER_TIMEOUT  30000        // How long to set diverter
                   // May be longer than mainsta timeout
/******************************************************************/
char arrival_from;          // indicates if main is ready for receive or in arrival alert
char purge_mode;            // indicates if main is in purge mode (flash lights differently)

/* Array index values for remote data, returned by INPUTS_ARE command. */
#define REMOTE_CIC      0
#define REMOTE_DOOR     1
#define REMOTE_ARRIVE   2
#define REMOTE_RTS      3
#define REMOTE_RTS2     4

main()
{  int i;
   unsigned long timeout;
   struct iomessage message;
   unsigned long heartbeat_timer;
   unsigned long heartbeat_interval;
   struct UDPmessageType UDPmessage;
   char key, simonsays;
   auto rn_search newdev;
   int status;

   /* Initialize i/o */
   loadResetCounters();  // load counters for watchdog and powerlow resets
   // Initialize the controller
   brdInit();              // Initialize the controller
   rn_init(RN_PORTS, 1);   // Initialize controller RN ports

   // Check if Rabbitnet boards are connected
   newdev.flags = RN_MATCH_PRDID;
   newdev.productid = RN1100;
   if ((DevRN1100 = rn_find(&newdev)) == -1)
   {
      printf("\n no RN1100 found\n");
   }
   else status = rn_digOutConfig(DevRN1100, OUTCONFIG);  //configure safe state

   // check if last reset was due to watchdog
   //if (wderror()) incrementWDCount();

   hitwd();
   init_io();
   arrival_alert(aaRESET, 0);
   toggleLED(255);  // initialize LED outputs
	readParameterSettings();

   /* read board address and set configuration i/o mappings */
   THIS_DEVICE = (di_remoteAddress);
   //if (PRINT_ON) printf("\nThis is remote plc # %d", THIS_DEVICE);

   // Check for and setup point to point configuration
   #if REMOTE_TYPE == RT_POINT2POINT
		THIS_DEVICE = 1;
   #endif

   // flash the plc number
   //flasher(0);
   for (i=0; i<THIS_DEVICE; i++)
   {  msDelay(200);
      ledOut(0,1);
      msDelay(150);
      ledOut(0,0);
      hitwd();
   }
   // Set the diverter mapping based on the Remote ID#
   setDiverterMap(param.portMapping);

   // setup interrupt handler for arrival optics
   #if __SEPARATE_INST_DATA__ && FAST_INTERRUPT
      interrupt_vector ext1_intvec my_isr1;
   #else
      SetVectExtern3000(1, my_isr1);
      // re-setup ISR's to show example of retrieving ISR address using GetVectExtern3000
      //SetVectExtern3000(1, GetVectExtern3000(1));
   #endif
   // WrPortI(I1CR, &I1CRShadow, 0x09);      // enable external INT1 on PE1, rising edge, priority 1
   arrivalEnable(FALSE);  // Disable for now

#ifdef USE_TCPIP
	// setup default static IP using THIS_DEVICE (myIPaddress = MY_BASE_IP + THIS_DEVICE)
	sprintf(myIPaddress, MY_BASE_IP, THIS_DEVICE);
   printf("My IP address: %s\n", myIPaddress);
   // configure the interface
   if(sock_init()) printf("IP sock_init() failed\n");	// Initialize UDP communications
	ifconfig(IF_ETH0,
				IFS_DHCP, 1,
				IFS_IPADDR,aton(myIPaddress),
				IFS_NETMASK,aton(MY_IP_NETMASK),
				IFS_DHCP_FALLBACK, 1,
	         IFS_UP,
	         IFS_END);

   if(!udp_open(&sock, LOCAL_PORT, -1/*resolve(REMOTE_IP)*/, REMOTE_PORT, NULL))
   {  printf("udp_open failed!\n");
      //lcd_print(1, 0, "No IP Communications");
      msDelay(1000);
   }
#endif

   // if (PRINT_ON) printf("\nWD and PL Reset Counters %d  %d", watchdogCount(), powerlowCount());
   exercise_io();

   /* Initialize serial port 1 */
   enable_commands();

   thistime=MS_TIMER;
   lost=0;
  	heartbeat_interval=500;
   simonsays=TRUE;
   while(simonsays)
   {  // loop forever

      maintenance();  // Hit WD, flash LEDs, write outputs

      /* check for and process incoming commands */
      if (get_command(&message)) process_command(message);
      if (getUDPcommand(&UDPmessage) > 0) processUDPcommand(UDPmessage);

      /* check for and process remote stations with dedicated processors*/
      //processUDPio();

      /* check for and process local inputs */
      process_local_io();
      arrival_alert(aaTEST, 0);

      if ((MS_TIMER - heartbeat_timer) > heartbeat_interval)
      {  // Send a UDP heartbeat
	      sendUDPHeartbeat(1);
         heartbeat_timer=MS_TIMER;
		   monitorUDPactivity(0);  // force a refresh of station activity in case all are off
      }
#ifdef SHOW_CYCLE_TIME
	   processCycleTime();
#endif

   } /* end while */
}
/********************************************************/
char transStation, diverterStation, transFlags;
char carrier_attn, carrier_attn2;
char system_state, systemDirection;
//char remote_stat_alert;
//unsigned long arrival_time;
/********************************************************/
char whichCA;
nodebug root interrupt void my_isr1()
{
   latchCarrierArrival=activeStations; // can't tell which station so send via all
   whichCA=di_carrierArrival; //carrierArrival(255);
   WrPortI(I1CR, &I1CRShadow, 0x00);      // disble external INT1 on PE1
}
void arrivalEnable(char how)
{
   // should call this routine once to enable interrupt and once to disable

   #GLOBAL_INIT
   {
      latchCarrierArrival=FALSE;
   }

   latchCarrierArrival=FALSE;    // clear existing latch
   if (how)
   {   //outport(ITC,(inport(ITC))|0x02);      // enable  INT1
      //WrPortI(I1CR, &I1CRShadow, 0x09);      // enable external INT1 on PE1, rising edge, priority 1
   	WrPortI(I1CR, &I1CRShadow, 0x0A);		// enable external INT1 on PE1, rising and falling edge, priority 2
   } else
   {   //outport(ITC,(inport(ITC))&~0x02);     // disable INT1
      WrPortI(I1CR, &I1CRShadow, 0x00);      // disble external INT1 on PE1
   }
}

void setDiverterMap(char portMap)
{	// Sets up the diverter port mapping based on the supplied parameter
	// Also sets the parameter opMode to standard or head-diverter
   // Entry in diverter_map is the binary diverter port number (1,2,4,8)
   param.opMode = STD_REMOTE;
   switch (portMap)
   {
      case 0:
         break;
      case 1:
         MY_STATIONS = 0x0F;     // mask for stations used by this device
         IO_SHIFT = 0;           // 4 for stations 5-7, 0 for stations 1-3
         diverter_map[0] = 0; diverter_map[1] = 1;
         diverter_map[2] = 2; diverter_map[3] = 4;
         diverter_map[4] = 8; diverter_map[5] = 0;
         diverter_map[6] = 0; diverter_map[7] = 0;
         diverter_map[8] = 0; // for main to main
	      // Check for and setup point to point configuration
	      #if REMOTE_TYPE == RT_POINT2POINT
	         MY_STATIONS = 0x01;     // mask for stations used by this device
	         diverter_map[2] = 0; diverter_map[3] = 0; diverter_map[4] = 0;
         #endif
         break;
      case 2:
         MY_STATIONS = 0x70;     // mask for stations used by this device
         IO_SHIFT = 4;           // 4 for stations 5-7, 0 for stations 1-3
         diverter_map[0] = 0; diverter_map[1] = 8;
         diverter_map[2] = 8; diverter_map[3] = 8;
         diverter_map[4] = 8; diverter_map[5] = 1;
         diverter_map[6] = 2; diverter_map[7] = 4;
         diverter_map[8] = 0; // for main to main
         break;
      case 3:
         MY_STATIONS = 0x03;     // mask for stations used by this device
         IO_SHIFT = 0;           // maps i/o 1-4 to stations x-x
         diverter_map[0] = 0; diverter_map[1] = 1;
         diverter_map[2] = 2; diverter_map[3] = 4;
         diverter_map[4] = 4; diverter_map[5] = 8;
         diverter_map[6] = 8; diverter_map[7] = 8;
         diverter_map[8] = 0; // for main to main
         break;
      case 4:
         MY_STATIONS = 0x0C;     // mask for stations used by this device
         IO_SHIFT = 2;           // maps i/o 1-4 to stations x-x
         diverter_map[0] = 0; diverter_map[1] = 0;
         diverter_map[2] = 0; diverter_map[3] = 1;
         diverter_map[4] = 2; diverter_map[5] = 0;
         diverter_map[6] = 0; diverter_map[7] = 0;
         diverter_map[8] = 0; // for main to main
         break;
      case 5:
         MY_STATIONS = 0x70;     // mask for stations used by this device
         IO_SHIFT = 4;           // maps i/o 1-4 to stations x-x
         diverter_map[0] = 0; diverter_map[1] = 0;
         diverter_map[2] = 0; diverter_map[3] = 0;
         diverter_map[4] = 0; diverter_map[5] = 1;
         diverter_map[6] = 2; diverter_map[7] = 4;
         diverter_map[8] = 0; // for main to main
         break;
      case 6:
         MY_STATIONS = 0x01;     // mask for stations used by this device
         IO_SHIFT = 0;           // maps i/o 1-4 to stations x-x
         diverter_map[0] = 0; diverter_map[1] = 1;
         diverter_map[2] = 2; diverter_map[3] = 2;
         diverter_map[4] = 2; diverter_map[5] = 2;
         diverter_map[6] = 2; diverter_map[7] = 2;
         diverter_map[8] = 0; // for main to main
         break;
      case 7:  // HEAD DIVERTER USE ONLY
         param.opMode = HEAD_DIVERTER;
         MY_STATIONS = 0x80;     // mask for stations used by this device
         IO_SHIFT = 0;           // maps i/o 1-4 to stations x-x
         diverter_map[0] = 0; diverter_map[1] = 1;
         diverter_map[2] = 2; diverter_map[3] = 4;
         diverter_map[4] = 0; diverter_map[5] = 0;
         diverter_map[6] = 0; diverter_map[7] = 0;
         diverter_map[8] = 0; // for main to main
         initBlower();  // only head diverter uses a blower
         break;
      case 8: // Operate station 2, ignore station 1, pass through stations 3-7
         MY_STATIONS = 0x02;     // mask for stations used by this device
         IO_SHIFT = 1;           // maps i/o 1-4 to stations x-x
         diverter_map[0] = 0; diverter_map[1] = 0;
         diverter_map[2] = 1; diverter_map[3] = 2;
         diverter_map[4] = 2; diverter_map[5] = 2;
         diverter_map[6] = 2; diverter_map[7] = 2;
         diverter_map[8] = 0; // for main to main
         break;
      case 9:  // HEAD DIVERTER USE ONLY - ALTERNATE CONFIGURATION
         param.opMode = HEAD_DIVERTER2;
         MY_STATIONS = 0x80;     // mask for stations used by this device
         IO_SHIFT = 0;           // maps i/o 1-4 to stations x-x
         diverter_map[0] = 1; diverter_map[1] = 2;
         diverter_map[2] = 4; diverter_map[3] = 0;
         diverter_map[4] = 0; diverter_map[5] = 0;
         diverter_map[6] = 0; diverter_map[7] = 0;
         diverter_map[8] = 0; // for main to main
         initBlower();  // only head diverter uses a blower
         break;
      case 10:  // for use with head diverter configuration #9
         // controls station 1 & 2 but no actual diverter
         MY_STATIONS = 0x03;     // mask for stations used by this device
         IO_SHIFT = 0;           // maps i/o 1-4 to stations x-x
         diverter_map[0] = 0; diverter_map[1] = 0;
         diverter_map[2] = 0; diverter_map[3] = 0;
         diverter_map[4] = 0; diverter_map[5] = 0;
         diverter_map[6] = 0; diverter_map[7] = 0;
         diverter_map[8] = 0; // for main to main
         break;
         // 11-13 implemented in a later version
      case 14:
         MY_STATIONS = 0x0F;     // mask for stations used by this device
         IO_SHIFT = 0;           // maps i/o 1-4 to stations x-x
         diverter_map[0] = 0; diverter_map[1] = 4;
         diverter_map[2] = 2; diverter_map[3] = 4;
         diverter_map[4] = 8; diverter_map[5] = 0;
         diverter_map[6] = 0; diverter_map[7] = 0;
         diverter_map[8] = 0; // for main to main
         break;
      default:   // any other undefined selection
         MY_STATIONS = 0x00;     // mask for stations used by this device
         IO_SHIFT = 0;           // maps i/o 1-4 to stations x-x
         diverter_map[0] = 0; diverter_map[1] = 0;
         diverter_map[2] = 0; diverter_map[3] = 0;
         diverter_map[4] = 0; diverter_map[5] = 0;
         diverter_map[6] = 0; diverter_map[7] = 0;
         diverter_map[8] = 0; // for main to main
         break;
   }
#ifndef USE_TCPIP
	activeStations = MY_STATIONS;
#endif
}
#ifdef SHOW_CYCLE_TIME
void processCycleTime()
{  // calculate and display cycle time statistics
#define CYCLETIME_UPDATE 2000
   static int ct_min, ct_avg, ct_max;
   static int ct_loops;
   static unsigned long cycle_start;
   static unsigned long ct_last;
   unsigned long cycle_time;

   if (MS_TIMER - cycle_start >= CYCLETIME_UPDATE)
   {  if (ct_loops > 0) ct_avg = (int)((MS_TIMER - cycle_start) / ct_loops);
      printf("\n n=%d min=%d avg=%d max=%d", ct_loops, ct_min, ct_avg, ct_max);
      ct_max=0;
      ct_min=32767;
      ct_loops=0;
      cycle_start=MS_TIMER;
   }
   else
   {
      cycle_time = MS_TIMER - ct_last;
      ct_loops = ct_loops + 1;
      if (cycle_time > ct_max) ct_max = (int)cycle_time;
      if (cycle_time < ct_min) ct_min = (int)cycle_time;
   }
   ct_last = MS_TIMER;
}
#endif

void init_io()
{
   char i;
   for (i=0; i<4; i++)          // four or five output ports
   {
      outputvalue[i] = 0;
      remoteoutput[i] = 0;
   }
   blower(blwrOFF);
   alarm(OFF);
   secureAck=0;
   for (i=0; i<10; i++) secureCard[i].id=0;
   purge_mode=0;
}
void exercise_io()
{
//   if (!wderror())
//   {
      alert(ON, MY_STATIONS);
      inUse(ON, MY_STATIONS);
      inUse2(ON, MY_STATIONS);
      maintenance(); msDelay(1000); maintenance(); msDelay(1000);
      //inUse(ON, MY_STATIONS); maintenance(); msDelay(200);
      inUse(OFF, MY_STATIONS); inUse2(OFF, MY_STATIONS); alert(OFF, MY_STATIONS);
      maintenance(); msDelay(200);

      inUse(ON, MY_STATIONS); inUse2(ON, MY_STATIONS); alert(ON, MY_STATIONS);
      maintenance(); msDelay(200);
//   }
   inUse(OFF, MY_STATIONS);
   inUse2(OFF, MY_STATIONS);
   alert(OFF, MY_STATIONS);
	maintenance();
   hitwd();       // hit the watchdog

}
/********************************************************/
void process_local_io()
{
   char rts_data, rts2_data, cic_data;  /* bitwise station bytes */
   char door_closed;
   char on_bits, flash_bits;  /* which ones on and flash */
   char on_bits2, flash_bits2;  /* which ones on and flash for 2nd in-use light */
   char s, sb;                /* work vars for station & stationbit */
   char i;
   //static unsigned long LA_Timer;  // to latch the arrival signal

   #GLOBAL_INIT
   {
      rts_latch=0;
      rts2_latch=0;
      transStation=0;
      diverterStation=0;
      carrier_attn=0;
      carrier_attn2=0;
      //arrival_time=0;
      arrival_from=0;
      //LA_Timer=0;
      mainStation=0;
      param.subStaAddressing=FALSE;
	   g_carrierInChamber=0;
	   g_doorClosed=0;
	   g_carrierArrival=0;
	   g_requestToSend=0;
	   g_requestToSend2=0;
      arrivalEnableDelay=0;
   }

   // Check for delayed arrivalEnable
   if ((arrivalEnableDelay > 0) && (MS_TIMER - arrivalEnableDelay > 2000))
   {  arrivalEnable(TRUE);
   	arrivalEnableDelay = 0;
   }

   /* Check for diverter and blower attention */
   processDiverter();
   processBlower();

   /* Use local buffers for some inputs */
   if (param.opMode == STD_REMOTE)
   {  // stuff for standard remote inputs
      rts_data=requestToSend(MY_STATIONS);
      rts2_data=requestToSend2(MY_STATIONS);
      cic_data=carrierInChamber(MY_STATIONS);
      door_closed=doorClosed(MY_STATIONS);

      // handle carrier in chamber light
      // Need to shift CIC input back to (1..4) range for correct output
      // don't latch inputs if in purge mode
      if (purge_mode==0)
      {
	      /* Latch request_to_send for new requests */
	      rts_latch |= (rts_data & cic_data & door_closed) & ~carrier_attn & ~rts2_data;  // de-latch when rts2 pressed
	      rts2_latch |= (rts2_data & cic_data & door_closed) & ~carrier_attn2 & ~rts_data;  // de-latch when rts pressed
      }

      /* Don't latch requests from station in current transaction */
      rts_latch &= ~station2bit(transStation);
      rts2_latch &= ~station2bit(transStation);

      /* Turn off those who have lost their carrier or opened door */
      rts_latch &= cic_data;
      rts2_latch &= cic_data;
      rts_latch &= door_closed;
      rts2_latch &= door_closed;

      // Setup in-use lights for primary and optional secondary in-use lights
      on_bits    = station2bit(transStation);
      flash_bits = rts_latch | rts_data;
      flash_bits |= ~door_closed;  // flash open doors
      flash_bits |= carrier_attn; // flash stations needing attention  (unremoved delivery)
      on_bits2    = station2bit(transStation);
      flash_bits2 = rts2_latch | rts2_data;
      flash_bits2 |= ~door_closed;  // flash open doors
      flash_bits2 |= carrier_attn2; // flash stations needing attention  (unremoved delivery)
      if ((mainStation==SLAVE) && (param.subStaAddressing==TRUE))
      		flash_bits2 |= station2bit(diverterStation);
      else  flash_bits |= station2bit(diverterStation);

// NEED TO LOOK INTO HOW ARRIVAL_FROM WORKS
      // if main arrival active, flash inuse at who sent to main
      if (arrival_from)
      {  on_bits &= ~arrival_from;   // Not on solid
         flash_bits |= arrival_from; // On flash instead
      }

      // set the in use lights as necessary
      inUse(OFF, ~on_bits & ~flash_bits & MY_STATIONS & ~station2bit(transStation));
      inUse(ON, on_bits);
      inUse(FLASH, flash_bits & ~station2bit(transStation));
      inUse2(OFF, ~on_bits2 & ~flash_bits2 & MY_STATIONS & ~station2bit(transStation));
      inUse2(ON, on_bits2);
      inUse2(FLASH, flash_bits2 & ~station2bit(transStation));

	   // latch the arrival signal
      if (arrivalEnableDelay == 0)
      {  // only if no delay is active
      i = carrierArrival(MY_STATIONS);
	   if (i) latchCarrierArrival=i;     // latch this value
      }

   } else
   {  // stuff for head diverter
   	//i = di_HDArrival;
      // && (system_state==WAIT_FOR_TURNAROUND)
		if (di_HDArrival && (system_state==WAIT_FOR_TURNAROUND)) latchCarrierArrival=TRUE;     // latch this value
      rts_data=0;   // No push to send buttons
      cic_data=0;   // No carrier in chamber optics
      door_closed=0;  // No door switches
   }



}

/********************************************************/
void process_command(struct iomessage message)
{
   char wcmd, wdata;
   char ok;
   char i;
   char station;
   char * sid;
   //static char system_state, new_state;
   static char new_state;
   struct iomessage response;
	struct UDPmessageType UDPmessage;
   static unsigned long lastSecureRemoveMsg;

   #GLOBAL_INIT
   {
      system_state=IDLE_STATE;
      new_state=system_state;
      lastSecureRemoveMsg=0;
   }

   response.command=0;
   response.station=message.station;   // set initial default
   /* determine what type of command */
   switch(message.command)
   {
      case ARE_YOU_READY:
// debug to see what may be causing loss of communication
//lastcommand=message.command;
         // enable arrival interrupt / latch
         // Delay arrivalEnable some seconds to avoid early termination signal (v419)
         ////if (param.opMode == STD_REMOTE) arrivalEnable(TRUE);  // don't enable interrupt for head diverter
         if (param.opMode == STD_REMOTE) arrivalEnableDelay = MS_TIMER;
         response.command=ARE_YOU_READY;
         // assume ready for now
         response.data[0]=1<<(THIS_DEVICE-1);
         transStation=message.station;
         systemDirection=message.data[0];
         mainStation=message.data[1];  // get the main station to be used
         //transFlags=message.data[3];   // transaction flags such as STAT & Secure
         // Negate assumption if not ready
         if (isMyStation(message.station) && (param.opMode == STD_REMOTE))
         {  // stuff for standard remote
            rts_latch &= ~station2bit(message.station); // clear latch
            rts2_latch &= ~station2bit(message.station); // clear latch
            /* only if door closed, and (CIC or send-to-here) */
            if  ( doorClosed(station2bit(message.station))
             && ( carrierInChamber(station2bit(message.station)) || message.data[0]==DIR_SEND) )
            {  // ready
            	if (param.subStaAddressing && mainStation==SLAVE)
               {  inUse2(ON, station2bit(message.station));
               }else
               {  inUse(ON, station2bit(message.station));
            	}
            }
            else
            {  response.data[0]=0;                     // not ready
            	if (param.subStaAddressing && mainStation==SLAVE)
               		inUse2(OFF, station2bit(message.station));
	            else  inUse(OFF, station2bit(message.station));
               alert(ON, station2bit(message.station));
            }
         }
         break;

      case SET_DIVERTER:
// debug to see what may be causing loss of communication
//lastcommand=message.command;
			switch (param.opMode)
         {
	         case STD_REMOTE:
	            // stuff for standard remote
	            if (param.subStaAddressing && message.data[1]==SLAVE)
	               inUse2(FLASH, station2bit(message.station));
	            else
	               inUse(FLASH, station2bit(message.station));
	            setDiverter(message.station);
	            break;
	         case HEAD_DIVERTER:
	            // stuff for standard head diverter
	            setDiverter(message.data[0]);  // head diverter comes in different byte
	            break;
	         case HEAD_DIVERTER2:
	            // alternate head diverter
	            // main on port 1, station 1 on port 2, station 2 on port 3
	            if (message.data[0]==1) // align to main on port 1
	               setDiverter(0);  // align main
	            else
	               setDiverter(message.station); // align to station
	            break;
         }
         mainStation=message.data[1];  // get the main station to be used
         param.subStaAddressing=message.data[2];
         param.blowerType = message.data[3];
         transFlags = message.data[4];    // transaction flags such as STAT & Secure

         break;

      case DIVERTER_STATUS:      /* return logic of correct position */
// debug to see what may be causing loss of communication
//lastcommand=message.command;
			// capture the securePIN
         securePIN_hb = message.data[2];
         securePIN_lb = message.data[3];
         // return response
         response.command=DIVERTER_STATUS;
         // which port dependent on opMode
			switch (param.opMode)
         {
	         case STD_REMOTE:
	            i = message.station;
	            break;
	         case HEAD_DIVERTER:
         		i = message.data[0];				// main, remote, or slave
	            break;
	         case HEAD_DIVERTER2:
	            if (message.data[0]==1) i=0;  // main
	            else i = message.station;     // sub station for alternate config
            	break;
         }
         // is that port aligned? or not used?
         if ( (di_diverterPos == diverter_map[i]) || (diverter_map[i] == 0) )
         {  response.data[0]=1<<(THIS_DEVICE-1); // DIVERTER_READY;
         } else
         { response.data[0]=0;            // DIVERTER_NOT_READY;
         }
         break;

      case RETURN_INPUTS:        /* return general status */
// debug to see what may be causing loss of communication
//lastcommand=message.command;
         // main includes some status info here ... pull it out
         arrival_from=message.data[0];
         new_state=message.data[1];
         // NOT USING IT: system_direction=message.data[2];

         // tell main how I am doing
         response.command=INPUTS_ARE;
         response.station=activeStations; // MY_STATIONS;  // return this to enlighten main

         // fill in the rest of the response based on who I am
         if (param.opMode == STD_REMOTE)
         {  // stuff for standard remote
            // are there any Secure Card ID's to pass along?
            station = nextSecureStation();
				if (station && ((MS_TIMER - lastSecureRemoveMsg)>500))
            {  // instead of INPUTS_ARE need to send SECURE_REMOVAL
               lastSecureRemoveMsg = MS_TIMER;
		         response.command=SECURE_REMOVAL;
               response.station=station;
				   sid = (char *)&secureCard[station].id;
	            response.data[0] = *sid++; // secureCardID byte 3
	            response.data[1] = *sid++; // secureCardID byte 2
	            response.data[2] = *sid++; // secureCardID byte 1
	            response.data[3] = *sid++; // secureCardID byte 0
               response.data[4] = secureCard[station].time;
            } else
				{
	            // Return latched carrier arrival if state is wait_for_remote_arrival
	            //if (latchCarrierArrival && system_state==WAIT_FOR_REM_ARRIVE) ok=TRUE;
	            //else ok=FALSE;
	            // Return the arrival signal directly
	            response.data[REMOTE_ARRIVE]=carrierArrival(MY_STATIONS);
	            // but if it is none then send latched arrival under some conditions
	            if ((response.data[REMOTE_ARRIVE]==0) && (latchCarrierArrival)
	                && (system_state==WAIT_FOR_REM_ARRIVE))   response.data[REMOTE_ARRIVE]=latchCarrierArrival;
	            response.data[REMOTE_CIC]=carrierInChamber(MY_STATIONS);
	            response.data[REMOTE_DOOR]=doorClosed(MY_STATIONS);
	            response.data[REMOTE_RTS]=rts_latch;
	            response.data[REMOTE_RTS2]=rts2_latch;
            }
         } else
         {  // stuff for head diverter
	         // Return latched carrier arrival if state is wait_for_turnaround
	         if ((latchCarrierArrival || di_HDArrival) && (system_state==WAIT_FOR_TURNAROUND)) ok=TRUE;
	         else ok=FALSE;
            // Return the arrival signal, latched in process or steady state
            if (ok || di_HDArrival) response.data[REMOTE_ARRIVE]=MY_STATIONS;
            else response.data[REMOTE_ARRIVE]=0;
            response.data[REMOTE_CIC]=0;
            response.data[REMOTE_DOOR]=0; // HD has no stations  MY_STATIONS;
            response.data[REMOTE_RTS]=0;
            if (blwrError==FALSE) response.data[REMOTE_RTS2]=blowerPosition();
            else response.data[REMOTE_RTS2]=0xFF; // Blower has an error
         }
         break;

		case ACK_SECURE_REMOVAL:
         // Acknowledge of secure card id from station n
			// secureAckStation = message.station
         // message.data[0] contains bit-wise stations to acknowledge
         for (station=1; station<=NUM_STATIONS; station++)
            if (message.data[0] & (station2bit(station))) secureCard[station].id=0;
      	break;

      case RETURN_EXTENDED:     /* return extended data */
// debug to see what may be causing loss of communication
//lastcommand=message.command;
          // return watchdog and powerlow counters
         response.command=RETURN_EXTENDED;
         response.data[0]=0;//watchdogCount();
         response.data[1]=0;//powerlowCount();
         response.data[2]=VERSION;
         break;

      case SET_OUTPUTS:
// debug to see what may be causing loss of communication
//lastcommand=message.command;
         if (param.opMode == STD_REMOTE)
         {  // stuff for standard remote
            /* set outputs from mainsta */
            set_remote_io(&message.data[0]);
         }
         break;

      case CLEAR_OUTPUTS:
// debug to see what may be causing loss of communication
//lastcommand=message.command;
         break;

      case SET_OPMODE:
// debug to see what may be causing loss of communication
//lastcommand=message.command;
         break;

      case SET_PARAMETERS:  // get the blower type and other parameters
// debug to see what may be causing loss of communication
//lastcommand=message.command;
			param.portMapping = message.data[0];
      	param.blowerType = message.data[1];
			writeParameterSettings();
         setDiverterMap(param.portMapping);

      	break;

      case SET_CARD_PARAMS:  // Setup the secure card parameters
      	if (message.data[0]==0)
			{
         	param.cardL3Digits = message.data[1];
         }
      	else if (message.data[0]==1)
         {
         	param.cardR9Min = *(unsigned long *)&message.data[1];
         }
      	else if (message.data[0]==2)
         {
         	param.cardR9Max = *(unsigned long *)&message.data[1];
            // last one so save and send the data to remotes
				// writeParameterSettings();  SAVE WILL FOLLOW IMMEDIATELY FROM SET_PARAMETERS
				sendUDPCardParameters();
         }
         break;

      case TRANS_COMPLETE:
// debug to see what may be causing loss of communication
//lastcommand=message.command;
         arrivalEnable(FALSE);  // Disable arrival interrupt
         transStation=0;

         if (param.opMode == STD_REMOTE)
         {  // stuff for standard remote
            alert(OFF, station2bit(message.station));
            // set arrival alert if the transaction is to here
            if (message.data[0] == DIR_SEND && isMyStation(message.station))
            {  if (message.data[1]==1)  // and Main says to set arrival alert
               { // OK to set alert
                  // REDUNDANT inUse(FLASH, station2bit(message.station));  // until no cic
                  if ((mainStation==SLAVE) && (param.subStaAddressing==TRUE))
		                  carrier_attn2 |= station2bit(message.station);  // to flash inuse
						else  carrier_attn |= station2bit(message.station);  // to flash inuse
                  arrival_alert(aaSET, message.station);
               }
            }
            else if (message.data[0] == DIR_RETURN && isMyStation(message.station))
            {  // capture the main arrival flag
            }
         } // nothing else for head diverter
         // pass this along to the remote stations
         UDPmessage.command = TRANS_COMPLETE;
         UDPmessage.station = message.station;
         UDPmessage.data[0] = message.data[0];
         UDPmessage.data[1] = message.data[1];
         UDPmessage.data[2] = message.data[2];
		   sendUDPcommand(UDPmessage); // ignore return value, will send again periodically

         break;

      case MALFUNCTION:  // may be contributing to funky flashing
// debug to see what may be causing loss of communication
//lastcommand=message.command;
         if (param.opMode == STD_REMOTE)
         {  // stuff for standard remote only
            // turn on alarm output only
            inUse(OFF, MY_STATIONS);           // clear local i/o
            inUse2(OFF, MY_STATIONS);           // clear local i/o
            alert(OFF, MY_STATIONS);
            alarm(ON);
            arrival_alert(aaRESET, 0);
         }
         arrivalEnable(FALSE);  // Disable arrival interrupt
         rts_latch=0;
         rts2_latch=0;
         transStation=0;
         diverterStation=0;
         response.data[0]=0;                 // clear remote i/o
         response.data[1]=0;
         response.data[2]=0;
         response.data[3]=0;
         set_remote_io(&response.data[0]);
         break;

      case RESET:
// debug to see what may be causing loss of communication
//lastcommand=message.command;
         if (param.opMode == STD_REMOTE)
         {  // stuff for standard remote only
            // Turn off all outputs
            inUse(OFF, MY_STATIONS);           // clear local i/o
            inUse2(OFF, MY_STATIONS);           // clear local i/o
            alert(OFF, MY_STATIONS);
            alarm(OFF);
            arrival_alert(aaRESET, 0);
         }
         blwrError=FALSE;       // reset blower error
         arrivalEnable(FALSE);  // Disable arrival interrupt
         /// rts_latch=0;  Try to not clear latch on reset
         transStation=0;
         diverterStation=0;
         response.data[0]=0;                 // clear remote i/o
         response.data[1]=0;
         response.data[2]=0;
         response.data[3]=0;
         set_remote_io(&response.data[0]);
         break;
   }

   /* send the response message if any AND the query was only to me */
   if (response.command && (message.device==THIS_DEVICE))
   {  msDelay(2); // hold off response just a bit
      //response.lplc=THIS_DEVICE;
      send_response(response);
   }

   // process state change
   if (new_state != system_state)
   {  // set blower for new state
      system_state=new_state;
      blower(blowerStateTable[system_state][blwrConfig]);
   }

   return;
}
void processUDPcommand(struct UDPmessageType UDPmessage)
{
#ifdef USE_TCPIP
	// Handle any incoming UDP packets for me
   int b_station;
   static unsigned long print_timer;
   unsigned long secureID;
   static int msg_count;
   #GLOBAL_INIT { msg_count=0; }

   // track and print UDP stats
   msg_count++;
  	if (MS_TIMER - print_timer > 1000)
   {  print_timer=MS_TIMER;
      printf("\n Got %d messages", msg_count);
      msg_count=0;
   	//printf("\n  Got UDP message %c at %ld", UDPmessage.command, UDPmessage.timestamp);
   }
   /* determine what type of command */
   switch(UDPmessage.command)
   {
      case INPUTS_ARE:
			// place returned values into local variables
         // Note: need to shift input bit positions according to station number
         b_station = station2bit(UDPmessage.station);
         if (UDPmessage.data[0]) g_carrierInChamber |= b_station;   // bit on
         else                    g_carrierInChamber &= ~b_station;  // bit off
         if (UDPmessage.data[1]) g_doorClosed |= b_station;   // bit on
         else                    g_doorClosed &= ~b_station;  // bit off
         if (UDPmessage.data[2]) g_carrierArrival |= b_station;   // bit on
         else                    g_carrierArrival &= ~b_station;  // bit off
         if (UDPmessage.data[3]) g_requestToSend |= b_station;   // bit on
         else                    g_requestToSend &= ~b_station;  // bit off
         if (UDPmessage.data[4]) g_requestToSend2 |= b_station;   // bit on
         else                    g_requestToSend2 &= ~b_station;  // bit off

         // save version number

         // Secure Card ID starts at .data[9]
         secureID = *(unsigned long *)&UDPmessage.data[9];
         if (secureID)
         {  // non-zero ID to be cached
	         secureCard[UDPmessage.station].id = secureID;
	         secureCard[UDPmessage.station].time = UDPmessage.data[13];
	         // tell remote we got the card ID
            secureAck = UDPmessage.station;
//	         if (secureCard[UDPmessage.station].id)
//	            secureAck = UDPmessage.station;
//	         else
//	            secureAck = 0;
         }

      break;
   }
   // track the active stations to report when a station stops communicating
   monitorUDPactivity(UDPmessage.device);

   // do we need to send any UDP packets?
   // handle them
#endif
}
void monitorUDPactivity(char deviceID)
{  // track the active stations to report when a station stops communicating
   // call with deviceID = 0 to refresh statistics
#ifdef USE_TCPIP
	static int recentDevices[NUM_STATIONS+1];
   static unsigned long refreshTimer;
   static int idx;

   #GLOBAL_INIT
   {	for (idx=0; idx<=NUM_STATIONS; idx++) recentDevices[idx]=0;
      refreshTimer=0;
      activeStations=0;
   }

   // make sure this device is within range
   if (deviceID < 10)
   {  // capture this device
	   recentDevices[deviceID]++;
   }
   // update and report new statistic every 1 secon
   if (MS_TIMER - refreshTimer >1000)
   {  // report the new statistic
      refreshTimer = MS_TIMER;
      activeStations = 0;
      for (idx=NUM_STATIONS; idx>0; idx--)  // count down
      {  activeStations = (activeStations << 1);            // shift
      	activeStations |= (recentDevices[idx]>0) ? 1 : 0;  // include bit
		   // clear the history buffer
		   recentDevices[idx]=0;
      }
   }
#endif
}

char nextSecureStation(void)
{  // if a secureCardID is in the buffer return the station number
	char i;
   for (i=1; i<9; i++)
   	if (secureCard[i].id) return i;

   return 0;
}

unsigned long alert_timer[8];   // for stations 1..7     ...  [0] is not used.
void arrival_alert(char func, char station)
{
   int i;

   switch (func)
   {
      case aaRESET:
       for (i=1; i<=7; i++)
       {
          alert_timer[i]=0;
          alert(OFF, station2bit(i));
       }
       break;

      case aaSET:
       if (station>=1 && station<=7)       // make sure s# is correct
       {
          alert_timer[ station ] = SEC_TIMER;
       }
       break;

      case aaTEST:
       for (i=1; i<=7; i++)     // Check each station
       {
          if (alert_timer[i])
          {
             if ( carrierInChamber( station2bit(i) ))    // || (remote_stat_alert==i) )
             {
               // Flash as necessary
               if ( (SEC_TIMER - alert_timer[i]) %20 >= 10 )
                 alert(ON, station2bit(i));
               else
                 alert(OFF, station2bit(i));
             }
             //else if (clock() = alert_timer[i] > 1)
             else if (!doorClosed( station2bit(i) ) || i==transStation)
             {  // no carrier, door open
               // or no carrier, in transit --> reset
               alert_timer[i]=0;
               alert(OFF, station2bit(i));
               carrier_attn &= ~station2bit(i); // clear attn flag
               carrier_attn2 &= ~station2bit(i); // clear attn flag

             }
          } else
          {  carrier_attn &= ~station2bit(i); // no timer, so no flag
          	 carrier_attn2 &= ~station2bit(i); // no timer, so no flag
          }

       }
       break;
   }
}

/************************************************************/
/**************     DIGITAL I/O     *************************/
/************************************************************/
#define devIUS   0
#define devIUF   1
#define devAlert 2
#define devAlarm 3
#define dev2IUS  4
#define dev2IUF  5
// #define devDoorAlert 4

int readDigInput(char channel)
{  // returns the state of digital input 0-39
   // function supports inverted logic by assigning dio_ON and dio_OFF

   char rtnval;

   if (channel < 16)
   {  // on-board inputs
      rtnval = digIn(channel);
   }
   else
   {  // rn1100 inputs
      if (DevRN1100 != -1)
      {  if (rn_digIn(DevRN1100, channel-16, &rtnval, 0) == -1) rtnval=1; } // 1 is off
      else rtnval = 0;
   }
   // deal with logic inversion
   if (rtnval) rtnval = dio_ON;
   else        rtnval = dio_OFF;
   return rtnval;
}
int readDigBank(char bank)
{  // returns the state of digital input banks 0-3
   // banks 0-1 on Coyote; banks 2-3 on RN1100
   // function supports inverted logic by assigning dio_ON and dio_OFF

   static char rtnval[4];  // to cache previous inputs
   char returnVal;
   char temp;

   #GLOBAL_INIT
   {  rtnval[0]=0; rtnval[1]=0; rtnval[2]=0; rtnval[3]=0;
   }

   if (bank < 2)
   {  // on-board inputs
      rtnval[bank] = digBankIn(bank);
   }
   else
   {  // rn1100 inputs
      if (DevRN1100 != -1)
      {
         if (rn_digBankIn(DevRN1100, bank-1, &temp, 0) == -1) rtnval[bank]=0xFF;  // 1 is off
         else rtnval[bank]=temp;
      }
   }
   returnVal = rtnval[bank];
   // deal with logic inversion
   if (dio_OFF) returnVal ^= 0xFF;
   return returnVal;
}

void setDigOutput(int channel, int value)
{  // sets the state of digital output 0-23
   // call with logic value (0=OFF, 1=ON)
   // function is adapted to support inverted logic by assigning dio_ON and dio_OFF

   int outval;
   // check for logic inversion
   if (value) outval = dio_ON;
   else       outval = dio_OFF;

   if (channel < 8)
   {  // on-board outputs
      digOut(channel, outval);
   }
   else
   {  // rn1100 outputs
      if (DevRN1100 != -1)
         rn_digOut(DevRN1100, channel-8, outval, 0);
   }
}

/*************************************************************/
void set_remote_io(char *remotedata)
{
   /* save state of remote output request */
   remoteoutput[0]=remotedata[0];
   remoteoutput[1]=remotedata[1];
//   remoteoutput[2]=remotedata[2];  // THIS IS devAlert - DON'T TAKE FROM MAIN
   remoteoutput[2]=remotedata[2];   // Instead use remotedata[2] to drive the door alert
//   remoteoutput[3]=remotedata[3];
   purge_mode=remotedata[3];
   /* write out all outputs .. some may need to be shifted */
//   write4data(addrIUS, outputvalue[devIUS] | remoteoutput[devIUS]);
//   write4data(addrIUF, outputvalue[devIUF] | remoteoutput[devIUF]);
//   outval = ((outputvalue[devAlert] | remoteoutput[devAlert]) >> IO_SHIFT);
//   write4data(addrALT, outval);
//   write4data(addrALM, outputvalue[devAlarm] | remoteoutput[devAlarm]);

   // Write remotedata[devAlert] to output ports O5-O8
//   hv_outval &= ~DOOR_ALERT_MASK;    // turn off door alert bits
//   hv_outval = (remotedata[devAlert] & MY_STATIONS) >> IO_SHIFT;
//   hv_wr(hv_outval);

}
/******************************************************************/
char carrierInChamber(char stamask)
{
#if REMOTE_TYPE == RT_DIVERTERONLY
  return (g_carrierInChamber) & stamask;
#else
  return (di_carrierInChamber << IO_SHIFT) & stamask;
#endif
}
/******************************************************************/
char doorClosed(char stamask)
{  // Note: input is normally high on closed.
   char indata;
#if REMOTE_TYPE == RT_DIVERTERONLY
   return (g_doorClosed & stamask);
#else
   indata = di_doorClosed;
   return ((indata << IO_SHIFT) & stamask);
#endif
}
/******************************************************************/
char carrierArrival(char stamask)
{
#if REMOTE_TYPE == RT_DIVERTERONLY
	return g_carrierArrival & stamask;
#else
	return (di_carrierArrival << IO_SHIFT) & stamask;
#endif
}

/******************************************************************/
char requestToSend(char stamask)
{
   char indata;
#if REMOTE_TYPE == RT_DIVERTERONLY
   return (g_requestToSend & stamask);
#else
   indata = di_requestToSend;
   return ((indata << IO_SHIFT) & stamask);
#endif
}
char requestToSend2(char stamask)
{
   char indata;
#if REMOTE_TYPE == RT_DIVERTERONLY
   return (g_requestToSend2 & stamask);
#else
   indata = di_requestToSend2;
   return ((indata << IO_SHIFT) & stamask);
#endif
}
/******************************************************************/
void inUse(char how, char station_b)
{
   if (how == ON)
   {  /* solid on, flash off */
      outputvalue[devIUS] |= station_b;
      outputvalue[devIUF] &= ~station_b;
   }

   if (how == OFF)
   {  /* solid off, flash off */
      outputvalue[devIUS] &= ~station_b;
      outputvalue[devIUF] &= ~station_b;
   }

   if (how == FLASH)
   {  /* solid off, flash on */
      outputvalue[devIUS] &= ~station_b;
      outputvalue[devIUF] |= station_b;
   }

   return;
}
void inUse2(char how, char station_b)
{
   if (how == ON)
   {  /* solid on, flash off */
      outputvalue[dev2IUS] |= station_b;
      outputvalue[dev2IUF] &= ~station_b;
   }

   if (how == OFF)
   {  /* solid off, flash off */
      outputvalue[dev2IUS] &= ~station_b;
      outputvalue[dev2IUF] &= ~station_b;
   }

   if (how == FLASH)
   {  /* solid off, flash on */
      outputvalue[dev2IUS] &= ~station_b;
      outputvalue[dev2IUF] |= station_b;
   }

   return;
}
/******************************************************************/
void alert(char how, char station_b)
{
   if (how == ON) outputvalue[devAlert] |= station_b;
   if (how == OFF) outputvalue[devAlert] &= ~station_b;

   //outval = ((outputvalue[devAlert] | remoteoutput[devAlert]) >> IO_SHIFT);
   //write4data(addrALT, outval);
   //do_alert(outval, how);
   // OUTPUTS WRITTEN IN SYSTEM LOOP CALL TO updateOutputs();

   return;
}
/******************************************************************/
void alarm(char how)
{
   // outputvalue is checked in the inUse function to turn alarm on/off
   outputvalue[devAlarm] = how;

   return;
}
/******************************************************************/
unsigned long diverter_start_time;     // these used primarily by diverter functions
char diverter_setting;
char diverter_attention;
void setDiverter(char station)
{  /* Controls the setting of diverter positions
      Return from this routine is immediate.  If diverter is not
      in position, it is turned on and a timer started.

      You MUST repeatedly call processDiverter() to complete processing
      of the diverter control position
   */

   /* use mapping from station to diverter position */

   if ((station>=0) && (station<9))     // Valid station #
   {
      diverter_setting = diverter_map[station];    // mapped setting

      // if position<>ANY (any=0) and not in position
      if ((diverter_setting>0) && (diverter_setting!=di_diverterPos))
      {
         /* turn on diverter and start timer */
         do_diverter(ON);

         diverter_start_time = MS_TIMER;
         diverter_attention = TRUE;
         diverterStation=station;
      }
   }
   return;
}
/******************************************************************/
void processDiverter()
{  /* Poll type processing of diverter control system */
   /* Allows other processes to be acknowleged when setting diverter */

   if (diverter_attention)
   {
      if ((diverter_setting == di_diverterPos) || Timeout(diverter_start_time, DIVERTER_TIMEOUT))
      {
         // turn off diverter and clear status
         do_diverter(OFF);
         diverter_attention = FALSE;
         diverterStation=0;
      }
   }
}
void showactivity()
{
   // update active processor signal
   ledOut(0, (MS_TIMER %250) < 125);  // on fixed frequency
   //ledOut(1, (MS_TIMER %500) > 250);  // off fixed frequency
   toggleLED(2);                      // toggle each call
}
void toggleLED(char LEDDev)
{
   static char LEDState[4];

   // if LEDDev not 0..3 then initialize LED states off
   if (LEDDev > 3)
   {  LEDState[0]=0; LEDState[1]=0; LEDState[2]=0; LEDState[3]=0;
   }
   else
   {
      LEDState[LEDDev] ^= 1;               // toggle LED state on/off
      ledOut(LEDDev, LEDState[LEDDev]);    // send LED state
   }
}

void maintenance(void)
{  // handle all regularly scheduled calls
	// use of static CHAR was causing PA4 RS485 transmitter to enable without reason ... static INT works ok.
   static int out_alert, out_inuse, out_inuse2, out_doorAlert;
   char i, cic_data, pflash;

   hitwd();                // hit the watchdog timer
   showactivity();
   //rn_keyProcess(DevRN1600, 0);  // process keypad device

   // process command communications
#ifdef USE_TCPIP
   tcp_tick(NULL);
#endif
   //receive_command();

   // process flashing inUse lights
   // out_alert = ((outputvalue[devAlert] | remoteoutput[devAlert]) >> IO_SHIFT);
   out_alert = ((outputvalue[devAlert]) >> IO_SHIFT);
   out_doorAlert = (remoteoutput[devAlert] >> IO_SHIFT);

   // flash on
   if ((MS_TIMER %500) < 300)
   {  out_inuse = (outputvalue[devIUF] | outputvalue[devIUS]) >> IO_SHIFT;
		out_inuse2 = (outputvalue[dev2IUF] | outputvalue[dev2IUS]) >> IO_SHIFT;
   }
	// or flash off
   else
   {	out_inuse = outputvalue[devIUS] >> IO_SHIFT;
   	out_inuse2 = outputvalue[dev2IUS] >> IO_SHIFT;
   }

   // get carrier in chamber data
   cic_data=carrierInChamber(MY_STATIONS) >> IO_SHIFT;

   // if purge mode handle lights different
   if (purge_mode==1)
   {
      if ((MS_TIMER%300) < 150)
      {  out_inuse=0;
         out_inuse2=0;
         cic_data=MY_STATIONS >> IO_SHIFT;
      } else
      {  out_inuse=MY_STATIONS >> IO_SHIFT;
   		out_inuse2=MY_STATIONS >> IO_SHIFT;
         cic_data=0;
      }
   }

   // Write out alerts and inUse light and CIC
   // Check for and setup point to point configuration
   #if REMOTE_TYPE == RT_POINT2POINT
	   do_arrivalAlert(0, out_alert & 1);
	   do_doorAlert(0, out_doorAlert & 1);
		do_inUseLight(0, out_inuse & 1);
		do_CICLight(0, cic_data & 1);
   #endif
	#if REMOTE_TYPE == RT_STANDARD
	   for (i=0; i<4; i++)
	   {  do_arrivalAlert(i, out_alert & (1<<i));
	      do_doorAlert(i, out_doorAlert & (1<<i));
	      do_inUseLight(i, out_inuse & (1<<i));
	      do_inUse2Light(i, out_inuse2 & (1<<i));
         do_CICLight(i, cic_data & (1<<i));
	   }
   #endif


   // Process alarm output
   //do_alarm(outputvalue[devAlarm]);

}

/******************************************************************/
// Blower Processing Routines - supports both standard and APU blowers
/******************************************************************/
// Interface definition
// char blwrConfig;     // to become a parameter from eeprom
// blwrConfig 0 = Standard blower
// blwrConfig 1 = High capacity APU blower
// #define blwrOFF  0
// #define blwrIDLE 1
// #define blwrVAC  2
// #define blwrPRS  3
// void initBlower(void);
// void blower(char blowerOperatingValue);  // use blwrOFF, blwrIDLE, blwrVAC, blwrPRS
// char processBlower(void);
// char blowerPosition(void);
/*
char blower_mode, blower_limbo, last_shifter_pos, lastDir;
unsigned long blower_timer;

void initBlower()
{   // trys to find any shifter position, then goes to idle
    unsigned long mytimer;

    blwrConfig = 1; // Turnaround configuration

    // Clear global blower variables
    last_shifter_pos=0;
    lastDir=0;
    blower_limbo=FALSE;
	 blwrError=FALSE;

    // turn off all outputs
    do_blower(OFF);
    do_blowerVac(OFF);
    do_blowerPrs(OFF);

    // remainder of initialization is for APU blowers only
    if (param.blowerType == blwrType_APU)
    {
	    // if not in any position, try to find any position
	    if (blowerPosition() == 0)
	    {  // hunt for any position
	       do_blowerPrs(ON);
	       mytimer=MS_TIMER;
	       while (Timeout(mytimer, 5000)==FALSE && blowerPosition()==0) hitwd();
	       if (blowerPosition()==0)
	       {  // still not found...so go the other way
	          do_blowerPrs(OFF);
	          do_blowerVac(ON);
	          mytimer=MS_TIMER;
	          while (Timeout(mytimer, 5000)==FALSE && blowerPosition()==0) hitwd();
	          do_blowerVac(OFF);
             if (blowerPosition()==0) blwrError = TRUE;  // capture unknown position error
	       }
	    }

	    // after all that, now command to goto idle
	    blower(blwrIDLE);
	 }
}
void blower(char request)
{  // operates both standard blowers and APU blowers
   // sets the direction of the blower shifter
   // use  blwrVAC, blwrPRS, blwrIDLE, or OFF
   char hv_save;
   char how;
   static char lastHow;

   #GLOBAL_INIT
   {  blwrConfig = 1; // Turnaround configuration
      lastHow = 0;
   }

   // Handling of standard blower type
   if (param.blowerType == blwrType_STD)
   {  // No such state as idle in standard blowers so map to OFF
   	if (request==blwrIDLE) request=blwrOFF;
	   how = request;

	   // make sure correct parameter is used.  anything but VAC and PRS is OFF
	   if ((how != blwrVAC) && (how != blwrPRS)) how = blwrOFF;

	   // if going from one on-state to the other on-state then turn off first
	   if ((how != blwrOFF) && (lastHow != blwrOFF) && (how != lastHow))
	   {  // can't run in both directions at the same time
	      do_blowerPrs(OFF);
	      do_blowerVac(OFF);
	      msDelay(100);                   // wait a little while
	   }

	   // turn on appropriate bit
	   if (how == blwrPRS)      do_blowerPrs(ON);
	   else if (how == blwrVAC) do_blowerVac(ON);
	   else { do_blowerPrs(OFF);
	          do_blowerVac(OFF);
	        }
	   // Add alternate blower on/off control per Joe request on 10-Jul-07
   	if (how != blwrOFF) do_blower(ON);
		else do_blower(OFF);

	   // remember last setting
	   lastHow = how;
   } // end of standard blower

   // Handling of APU blower type
   else if ((param.blowerType == blwrType_APU) && (request != lastHow))
   {
	   // store in local work space
	   blower_mode=request;
	   blower_limbo=FALSE;
      lastHow=request;

	   // clear previous state
	   // hv_save=output_buffer;
	   // turn off shifters
	   do_blowerPrs(OFF);
	   do_blowerVac(OFF);

	   // CW for Pressure to Idle or Any to Vacuum
	   // CCW for Vacuum to Idle or Any to Pressure
	   if (blower_mode == blwrVAC) { do_blowerVac(ON); lastDir=blwrVAC; }
	   else if (blower_mode == blwrPRS) { do_blowerPrs(ON); lastDir=blwrPRS; }
	   else if (blower_mode == blwrIDLE)
	   {  if (blowerPosition() == blwrPRS) { do_blowerVac(ON); lastDir=blwrVAC; }
	      else if (blowerPosition() == blwrVAC) { do_blowerPrs(ON); lastDir=blwrPRS; }
	      else if (blowerPosition() == 0)
	      {
	          // go to idle but don't know which way
	          if (lastDir==blwrVAC) { do_blowerVac(ON); blower_limbo=TRUE; lastDir=blwrVAC; }
	          else if (lastDir==blwrPRS) { do_blowerPrs(ON); blower_limbo=TRUE; lastDir=blwrPRS; }
	          else
	          {  // use last position to know which way to go
	             blower_limbo = TRUE;       // incomplete operation
	             if (last_shifter_pos==blwrPRS) { do_blowerVac(ON); lastDir=blwrVAC; }
	             else if (last_shifter_pos==blwrVAC) { do_blowerPrs(ON); lastDir=blwrPRS; }
	             else blower_limbo=FALSE;  // idle to idle is ok to leave alone
	          }
	      } // else go-idle and already in idle position
	   } else if (blower_mode == OFF) do_blower(OFF);  // power off

	   blower_timer = MS_TIMER;
   }

   return;
}
char processBlower()
{   // call repeatedly to handle process states of the blower
    // returns non-zero when shifter or blower error
    char rtnval;

    rtnval=0;  // assume all is good

    // Remainder only for APU blowers
    if (param.blowerType == blwrType_APU)
    {

	    if (blower_mode==OFF) return rtnval;  // nothing happening, go home.
	    if (blower_mode==ON) return rtnval;   // running ... nothing to do

	    // check for idled blower timeout
	    if ((blower_mode==blwrIDLE) && (last_shifter_pos == blwrIDLE) && (lastDir==0) )
	    {  // turn off idle'd blower after 2 minutes
	       if (MS_TIMER - blower_timer > 120000)
	       {  // turn off all blower outputs
	          do_blowerPrs(OFF);
	          do_blowerVac(OFF);
	          do_blower(OFF);
	          blower_mode=OFF;
	          lastDir=0;
	       }
	       return rtnval;
	    }

	    // check for incomplete change of direction and reset shifter outputs.
	    // if was in limbo, and now in a known position, reset shifter
	    if (blowerPosition() && blower_limbo) blower(blower_mode);

	    // if in position  .. but only the first time FIX.
	    if (blower_mode == blowerPosition())
	    {  // turn off shifter, turn on blower
	       do_blowerPrs(OFF);
	       do_blowerVac(OFF);
	       lastDir=0;

	        // if going for idle position, mode is not ON ... just idle
	        if (blower_mode != blwrIDLE)
	        {  blower_mode = ON;          // generic ON state
	           do_blower(ON);
	        }
	        blower_timer=MS_TIMER;  // use also for blower idle time
	    }
	    else if (MS_TIMER - blower_timer > 12000)
	    {  // timeout ... shut off all outputs
	       do_blowerPrs(OFF);
	       do_blowerVac(OFF);
	       lastDir=0;
	       blower_mode=OFF;
	       rtnval=1;          // set error code
          blwrError=TRUE;
	    }
    }

    return rtnval;

}
char blowerPosition()
{  // returns the current position of the blower shifter
   // also keeps track of the last valid shifter position
   char rtnval, inval;
   rtnval=0;
   if (di_shifterPosVac) rtnval=blwrVAC;
   if (di_shifterPosPrs) rtnval=blwrPRS;
   if (di_shifterPosIdle) rtnval=blwrIDLE;
   if (rtnval) last_shifter_pos=rtnval;
   return rtnval;
}
/*

/********************************************************
   IP Communication drivers
*********************************************************/
int getUDPcommand(struct UDPmessageType *UDPmessage)
{
#ifdef USE_TCPIP
	// be sure to allocate enough space in buffer to handle possible incoming messages (up to 128 bytes for now)
   auto char   buf[128];
   auto int    length, retval;
   int idx;
   char * UDPbuffer;  // to index into UDPmessage byte by byte

   length = sizeof(buf);
   retval = udp_recv(&sock, buf, length);
   if (retval < -1) {
      printf("Error %d receiving datagram!  Closing and reopening socket...\n", retval);
      sock_close(&sock);
      if(!udp_open(&sock, LOCAL_PORT, -1/*resolve(REMOTE_IP)*/, REMOTE_PORT, NULL)) {
         printf("udp_open failed!\n");
      }
   }
   else
   {
   	// is the command for me?
		// map it into the UDPmessage structure
      UDPbuffer = (char *) UDPmessage;
      for (idx = 0; idx < sizeof(*UDPmessage); idx++)
      	UDPbuffer[idx] = buf[idx];
      // Return it
	}
   return retval;
#endif
}
int sendUDPcommand(struct UDPmessageType UDPmessage)
{
#ifdef USE_TCPIP
	// Send the supplied command over UDP
   // Appends some info to the standard message
   //   .device
   //   .timestamp

   auto int length, retval;
   char * UDPbuffer;  // to index into UDPmessage byte by byte

   // fill the packet with additional data
   UDPmessage.device = THIS_DEVICE;  // system id
   UDPmessage.devType = 2; // diverter
   UDPmessage.timestamp = MS_TIMER;

   length = sizeof(UDPmessage); // how many bytes to send
   UDPbuffer = (char *) &UDPmessage;

   /* send the packet */
   retval = udp_send(&sock, UDPbuffer, length);
   if (retval < 0) {
      printf("Error sending datagram!  Closing and reopening socket...\n");
      sock_close(&sock);
      if(!udp_open(&sock, LOCAL_PORT, -1/*resolve(REMOTE_IP)*/, REMOTE_PORT, NULL)) {
         printf("udp_open failed!\n");
      }
   }
#endif
}
int sendUDPHeartbeat(char sample)
{  // Sends a periodic message to the remote stations
#ifdef USE_TCPIP
	struct UDPmessageType HBmessage;

   HBmessage.command = REMOTE_PAYLOAD;
   HBmessage.station = MY_STATIONS;
   HBmessage.data[0] = arrival_from; //
   HBmessage.data[1] = system_state; //
   HBmessage.data[2] = 0; // remote alert - not used
   HBmessage.data[3] = transStation; //
   HBmessage.data[4] = mainStation; //
   HBmessage.data[5] = diverterStation; //
   HBmessage.data[6] = param.subStaAddressing; //
   HBmessage.data[7] = transFlags; //
   HBmessage.data[8] = secureAck; // secure ACK
   secureAck = 0; // clear Ack after sending
   HBmessage.data[9] = securePIN_hb; // secure PIN high byte
   HBmessage.data[10] = securePIN_lb; // secure PIN low byte
   HBmessage.data[11] = systemDirection; // transaction direction
   HBmessage.data[12] = purge_mode;
   HBmessage.data[13] = 0; // nothing
   sendUDPcommand(HBmessage); // ignore return value, will send again periodically
#endif
}
void sendUDPCardParameters(void)
{
// Sends secure card parameters the remote stations
#ifdef USE_TCPIP
	struct UDPmessageType HBmessage;

   HBmessage.command = SET_CARD_PARAMS;
   HBmessage.station = MY_STATIONS;
   HBmessage.data[0] = param.cardL3Digits; //
   *(unsigned long*)&HBmessage.data[1] = param.cardR9Min; //
   *(unsigned long*)&HBmessage.data[5] = param.cardR9Max; //
   sendUDPcommand(HBmessage); // ignore return value, will send again periodically
#endif
}

/********************************************************
   Communication I/O drivers
*********************************************************/
/* Initialize buffers and counters */
#define BAUD_RATE 19200
#define CMD_LENGTH   11
#define RCV_TIMEOUT  2
#define rbuflen      30
#define sbuflen      5
char rbuf[rbuflen], sbuf[sbuflen];
char bufstart; //rcc

void disable485whenDone(void)
{
   while (serDwrUsed());          // wait for all bytes to be transmitted
   while (RdPortI(SDSR) & 0x0C);  // wait for last byte to complete
   serDrdFlush();                 // flush the read buffer
   ser485Rx();                    // disable transmitter
}
/********************************************************/
void enable_commands()
{  // open serial port D
   serDopen(BAUD_RATE);
   ser485Rx();       // enable the receiver
   serDrdFlush();    // flush the read buffer
}
/********************************************************/
void send_response(struct iomessage message)
{  /* Transmits a message/command to the main system.
      Command string defined as:
      <STX> <device> <Command> <Station> <Data0..3> <ETX> <CHKSUM> */

   char i;
   char cmdstr[CMD_LENGTH];
   char cmdlen;
   char rcvbuf[CMD_LENGTH];
   char rcvlen;
   unsigned long timeout, cmdsent;
   cmdlen=CMD_LENGTH;

   // enable transmitter and send
   ser485Tx();

   /* formulate the full command string */
   cmdstr[0] = STX;
   cmdstr[1] = THIS_DEVICE;
   cmdstr[2] = message.command;
   cmdstr[3] = message.station;
   for (i=0; i<NUMDATA; i++) cmdstr[i+4]=message.data[i];
   cmdstr[CMD_LENGTH-2] = ETX;
   cmdstr[CMD_LENGTH-1] = 0;
   for (i=0; i<CMD_LENGTH-1; i++)
      cmdstr[CMD_LENGTH-1] += cmdstr[i];  /* calculate checksum */

   // send it
   cmdlen = serDwrite(cmdstr, CMD_LENGTH);
   if (cmdlen != CMD_LENGTH) printf("Send command failed, only sent %d chars \n", cmdlen);

   disable485whenDone();

}


/********************************************************/
char get_command(struct iomessage *message)
{
/* Repeatedly call this function to process incoming commands
   efficiently.
*/
   int charsinbuf;
   char msg_address;  //worklen,
   char rtnval, chksum, i;
   unsigned long t1, t2;
   char tossed[50];


   // charsinbuf = rbuflen-rcc;
   // worklen = charsinbuf-bufstart;
   rtnval=FALSE;     /* assume none for now */

   // align frame to <STX>
   i=0;
   t1=MS_TIMER;
   while ((serDrdUsed() > 0) && (serDpeek() != STX)) { tossed[i]=serDgetc(); i++; }
   t2=MS_TIMER;
   if (i>0)
   {  //printf("%ld %ld tossed %d: ",t1, t2, i);
      //while (i>0) { i--; printf(" %d", tossed[i]); }
      //printf("\n");
   }

   // if at least 1 command in buffer then get them
   charsinbuf=0;
   if (serDrdUsed() >= CMD_LENGTH)
      charsinbuf = serDread(rbuf, CMD_LENGTH, RCV_TIMEOUT);

   bufstart = 0;
   if (charsinbuf==CMD_LENGTH)  /* all characters received */
   {
      /* verify STX, ETX, checksum then respond */
      if ((rbuf[bufstart] == STX) && (rbuf[bufstart+CMD_LENGTH-2] == ETX))
      {
         // enable transmitter if message is addressed to me only
         // set msg_address here since i'm using it next
         msg_address = rbuf[bufstart+1];
         if (msg_address==THIS_DEVICE) ser485Tx();
//for (i=0; i<CMD_LENGTH-1; i++) printf(" %d", rbuf[bufstart+i]);

         /* check checksum */
         chksum=0;
         for (i=0; i<CMD_LENGTH-1; i++) chksum+=rbuf[bufstart+i];
         if (chksum != rbuf[bufstart+CMD_LENGTH-1])
         {  // no good, send NAK
            sbuf[0]=NAK;
            rtnval=FALSE;
         }
         else
         {  /* looks good, send ACK */
            sbuf[0]=ACK;
            rtnval=TRUE;
         }
         // don't set msg_address here.  need to use it up above
         //msg_address = rbuf[bufstart+1];
         // send response if message is addressed to me only
         if (msg_address==THIS_DEVICE)
         {
            // enable transmitter and send
            ser485Tx();
            serDwrite(sbuf, 1);
            disable485whenDone();
         }

         // If this message for me OR everyone then process it.
         if ((msg_address==THIS_DEVICE) || (msg_address==ALL_DEVICES))
         {  // message for me (and possibly others)
            /* return the command for processing */
            message->device=msg_address;
            message->command=rbuf[bufstart+2];
            message->station=rbuf[bufstart+3];
            for (i=0; i<NUMDATA; i++) message->data[i]=rbuf[i+bufstart+4];

         } else rtnval=FALSE;  // command not for me!

         // Maybe shouldn't flush the buffer
         // serDrdFlush(); // flush the read buffer
      }
   }
   //else
   //{ printf("missing chars %d\n", charsinbuf); serDrdFlush(); }
   return rtnval;
}
/******************************************************************/
char isMyStation(char station)
{       return (station2bit(station) & MY_STATIONS);
}
/******************************************************************/
void msDelay(unsigned int delay)
{
   auto unsigned long start_time;
   start_time = MS_TIMER;
   if (delay < 500) while( (MS_TIMER - start_time) <= delay );
   else while( (MS_TIMER - start_time) <= delay ) hitwd();
}

char Timeout(unsigned long start_time, unsigned long duration)
{
   return ((MS_TIMER - start_time) < duration) ? FALSE : TRUE;
}

/*******************************************************/
// WATCHDOG AND POWER-LOW RESET COUNTERS
/*******************************************************/
char valid_resets;     // at myxdata+0
char watchdog_resets;  // at myxdata+1
char powerlow_resets;  // at myxdata+2
//xdata myxdata[10];

/*******************************************************/
char watchdogCount() { return watchdog_resets; }
char powerlowCount() { return powerlow_resets; }
/*******************************************************/
void incrementWDCount()
{
   watchdog_resets++;
   //root2xmem( &watchdog_resets, myxdata+1, 1);
}
/*******************************************************/
void incrementPLCount()
{
   powerlow_resets++;
   //root2xmem( &powerlow_resets, myxdata+2, 1);
}

/*******************************************************/
void loadResetCounters()
{
   // read valid_resets and check
   //xmem2root( myxdata, &valid_resets, 1);
   if (valid_resets != 69)
   {  // set to zero and mark valid
      watchdog_resets=0;
      //root2xmem( &watchdog_resets, myxdata+1, 1);
      powerlow_resets=0;
      //root2xmem( &powerlow_resets, myxdata+2, 1);
      valid_resets=69;
      //root2xmem( &valid_resets, myxdata, 1);
   } else
   {
      // Read watchdog and power-low counters
      //xmem2root( myxdata+1, &watchdog_resets, 1);
      //xmem2root( myxdata+2, &powerlow_resets, 1);
   }
}
int readParameterSettings(void)
{  // Read the parameter block param
   int rtn_code;
   rtn_code = readUserBlock(&param, 0, sizeof(param));
   return rtn_code;
}
int writeParameterSettings(void)
{  // Write the parameter block param
   int rtn_code;
   rtn_code = writeUserBlock(0, &param, sizeof(param));
   return rtn_code;
}

