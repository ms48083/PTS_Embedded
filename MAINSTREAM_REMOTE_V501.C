/*******************************************************************
   MAINSTREAM_REMOTE_V4xx.C   Colombo Semi-Automatic Pneumatic Tube Transfer System

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
   			        Now handles only one dedicated station
   19-Oct-08   v402 Clear request-to-send latch on setting diverter
   16-Nov-08   v403 Fix up to operate point to point over RS485
   06-Dec-08   v404 Added handling of secure transaction
   24-Dec-08   v405 Improve handling of secure transaction
   30-Dec-08   v406 Improve robustness of secure transaction
   05-Jan-09   v407 Fix handling of alert for door open during transaction
   03-Mar-09   v408 Bump version to 409
   					  Implement card parameters
   19-Apr-11   v410 When notified of purge mode, flash in-use and cic alternating
   01-Dec-11   v500 First adaptation for full ethernet communication
   05-Feb-12   v501 Lock in v500 as stable development version

***************************************************************/
#memmap xmem  // Required to reduce root memory usage
#class auto
#define FIRMWARE_VERSION "REMOTE V5.01"
#define VERSION                   5
#define SUBVERSION                  01

// compile as "STANDARD" or "POINT2POINT" remote
#define RT_STANDARD    1
#define REMOTE_TYPE RT_STANDARD
//#define REMOTE_TYPE RT_POINT2POINT

//int lastcommand; // for debug
#define USE_TCPIP 1

// define PRINT_ON for additional debug printing via printf
// #define PRINT_ON 1
#use "bl25xx.lib"          Controller library
#use "rn_cfg_bl25.lib"     Configuration library
#use "rnet.lib"            RabbitNet library
#use "rnet_driver.lib"     RabbitNet library
#use "rnet_keyif.lib"      RN1600 keypad library
#use "rnet_lcdif.lib"      RN1600 LCD library
// #define USE_BLOWER
#use "MAINSTREAM.lib"      Mainstream v4xx function library
#define WIEGAND_DEBUG
#define WIEGAND_VERBOSE
#use "wiegand_v2.lib"         RFID card reader interface
// RabbitNet RN1600 Setup
//int DevRN1600;                    // Rabbit Net Device Number Keypad/LCD
int DevRN1100;                   // Rabbit Net Device Number Digital I/O
//configure to sinking safe state
#define OUTCONFIG 0xFFFF

// Setup interrupt handling for carrier arrival input
// Set equal to 1 to use fast interrupt in I&D space
#define FAST_INTERRUPT 0
int latchCarrierArrival;
void my_isr1();
void arrivalEnable(char how);


// Communications data structures
#define NUMDATA   5
struct iomessage     /* Communications command structure */
{
   char device;
   char command;
   char station;
   char data[NUMDATA];
};
#define NUMUDPDATA 20
struct UDPmessageType  // UDP based messages
{
   char system;
	char device;
   char devType;  // M for main station
   char command;  // can be heartbeat, event, transaction
   char station;
   char sequence; // rolling packet sequence number
   unsigned long timestamp;
   char data[NUMUDPDATA];
};
// Define communications using TCP/IP (UDP)
#ifdef USE_TCPIP
	#define TCPCONFIG 106         // 106 is an empty config for run-time setup
   //#define USE_DHCP
   //#define USE_ETHERNET 1
	#define MAX_UDP_SOCKET_BUFFERS 1
	#define DISABLE_TCP		// Not using TCP
   #define MY_STATIC_IP "192.168.0.11"
   #define MY_BASE_IP "192.168.%d.3%d"
   char myIPaddress[18];  // string to hold runtime IP address
	#define MY_IP_NETMASK "255.255.255.0"
	#define LOCAL_PORT   1235      // for inbound messages
//	#define REMOTE_IP    "255.255.255.255" //255.255.255.255" /*broadcast*/
	#define REMOTE_PORT  1234      // for outbound messages
	#use "dcrtcp.lib"
	udp_Socket sock;
	int sendUDPHeartbeat(char sample);
	int sendUDPcommand(struct UDPmessageType message, long ipAddr);
	int getUDPcommand(struct UDPmessageType *message);
	void processUDPcommand(struct UDPmessageType message);
	void sendDeviceInfo(void);
   void sendCardScan(unsigned long cardID, unsigned long cardSite, char authorized, char action);
   long payloadIPaddr; // last payload ip address (my main)
   long lastIPaddr;    // last incoming ip address (could be any main, eg broadcast)
#endif
// Including parameter block structure
// Parameters are NOT read to / written from FLASH
struct par_block_type
{  // These parameters are defined on the fly at run-time
   char opMode;      // STD_REMOTE or HEAD_DIVERTER
   char subStaAddressing; // to indicate if substation can select slave for destination
   char blowerType;
   char portMapping;
   char system;
   char cardL3Digits;     // Constant left 3 digits of valid secure cards
	unsigned long cardR9Min;  // Secure card ID minimum value for the right 9 digits
   unsigned long cardR9Max;  // Secure card ID maximum value for the right 9 digits
   char cardNumBits;
   char cardSiteStart;
   char cardSiteLength;
   char cardIdStart;
   char cardIdLength;
   char disableLocks;
   int cardSiteCode;  // required card site code
} param;
#define STD_REMOTE    0
#define HEAD_DIVERTER 1
#define SLAVE   0x08      // Station number of the slave
#define SLAVE_b 0x80      // Bit representation

char THIS_DEVICE;       // board address for communications and setup
char THIS_DEVICE_b;
char MY_STATION;       // to mask supported stations
#define IO_STATION  1  // always output on first station i/o
char IO_SHIFT;          // to map expansion board i/o
char diverter_map[9];   // to assign diverter positions to stations

// Global variables
unsigned long thistime, lasttime, lost;

/* Declare general function prototypes */
void msDelay(unsigned int msec);
//char bit2station(char station_b);// station_b refers to bit equivalent: 0100
//char station2bit(char station);  // station refers to decimal value: 3 --^
void init_counter(void);
void process_local_io(void);
void set_remote_io(char *remotedata);
void init_io(void);
void exercise_io(void);
void processCardReads(void);
void arrival_alert(char code, char station);
char isMyStation(char station);
void toggleLED(char LEDDev);
void maintenance(void);
char Timeout(unsigned long start_time, unsigned long duration);
int  readParameterSettings(void);
int  writeParameterSettings(void);
char initCardReader(void);

/* Declare local and expansion bus input prototypes */
char carrierInChamber(char station_mask);
char doorClosed(char station_mask);
char carrierArrival(char station_mask);
char requestToSend(char station_mask);
char requestToSend2(char station_mask);
//char diverter_pos(void);

/* Declare local and expansion bus output prototypes */
char rts_latch;      // Used for latching requestToSend
char rts2_latch;     // Used for latching 2nd requestToSend (to slave)
void inUse(char how, char station_b);
void alert(char how, char station_b);
void alarm(char how);
void doorAlert(char how);
void unlockDoor(void);
// void diverter(char how);
//void setDiverter(char station);
//void processDiverter(void);
void setDiverterMap(char portMap);

// Declare watchdog and powerlow handlers
char watchdogCount(void);
char powerlowCount(void);
void incrementWDCount(void);
void incrementPLCount(void);
void loadResetCounters(void);
void checkMalfunction(void);

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
#define di_carrierArrival          readDigInput(0)
#define di_requestToSend           readDigInput(1)
#define di_requestToSend2          readDigInput(8)
#define di_doorClosed              readDigInput(2)
#define di_carrierInChamber        readDigInput(3)
#define di_remoteAddress           ((readDigBank(0) & 0xF0) >> 4)
/* NO BLOWER OR DIVERTER IN THIS VERSION
#define USE_DIVERTER
#define USE_BLOWER
#define di_diverterPos             (readDigBank(1) & 0x0F)
#define di_shifterPosIdle          readDigInput(12)
#define di_shifterPosPrs           readDigInput(13)
#define di_shifterPosVac           readDigInput(14)
*/

// OUTPUTS
void setDigOutput(int channel, int value);
#define do_shift 0
#define do_doorAlert(which,value)    setDigOutput(0,value)
#define do_arrivalAlert(which,value) setDigOutput(1,value)
#define do_CICLight(which,value)     setDigOutput(2,value)
#define do_inUseLight(which,value)   setDigOutput(3,value)
#define do_doorUnLock(value)   		 setDigOutput(4,value)
#define do_inUseLight2(which,value)  setDigOutput(5,value)
#define do_secureTransLight(value)   setDigOutput(7,value)
// inUseLight on output 3,4,7 causes RS485 transmitter to enable (PA4)
// Root cause was use of static CHAR elements in maintenance() instead of static INT
//#define do_alarm(value)              setDigOutput(xx+do_shift,value)
/*  NO BLOWER OR DIVERTER IN THIS VERSION
#define do_diverter(value)           setDigOutput(4+do_shift,value)
#define do_blower(value)             setDigOutput(5+do_shift,value)
#define do_blowerVac(value)          setDigOutput(6+do_shift,value)
#define do_blowerPrs(value)          setDigOutput(7+do_shift,value)
*/

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
#define DEVICE_INFO        'I'
#define ARE_YOU_READY      'J'
//#define RETURN_EXTENDED    'K'
#define CARD_SCAN          'K'
#define SET_TRANS_COUNT    'L'
#define SET_DATE_TIME      'M'
#define SET_STATION_NAME   'N'
#define CANCEL_PENDING     'O'
#define TAKE_COMMAND       'P'
#define DISPLAY_MESSAGE    'Q'
#define SET_PHONE_NUMS     'R'
#define SET_PARAMETERS     'S'
#define SET_CARD_PARAMS    'T'
#define REGISTER_DEVICE    'U'
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
// Define various globals
char mainStation;
char arrival_from;          // indicates if main is ready for receive or in arrival alert
char transStation, diverterStation, headDivStation, ready4Trans;
char ackEvent, ackCommand;
char malfunctionActive;
char carrier_attn, carrier_attn2;
char system_state, systemDirection;
char remoteAlert;
#define FLAG_SECURE      0x20
char transFlags;
char handleSecure;
char gotCardRead;
char gotPinRead;
int securePIN;
unsigned long cardSiteNum, cardIDNum;
#define CARDHBOFFSET  119
#define CARDLBOFFSET  3676144640
char purge_mode;            // indicates if main is in purge mode (flash lights differently)
char msgSequence;
//unsigned long startupTimestamp;
struct tm startupTimestamp;

//unsigned long secureCardID;
//unsigned long secureCardTime;
struct {
	unsigned long id;
   unsigned long time;
} secureCard;
unsigned long arrivalTime;
// offset to get printed card id number
#define CARDBASE 0xE8777C00

/* Array index values for remote data, returned by INPUTS_ARE command. */
#define REMOTE_CIC      0
#define REMOTE_DOOR     1
#define REMOTE_ARRIVE   2
#define REMOTE_RTS      3
#define REMOTE_RTS2     4

main()
{  int i;
   unsigned long timeout;
   unsigned long heartbeat_timer;
   unsigned long heartbeat_interval;
   struct UDPmessageType UDPmessage;
   char key, simonsays;
   auto rn_search newdev;
   int status;

   // Initialize the controller
   brdInit();              // Initialize the controller
   rn_init(RN_PORTS, 1);   // Initialize controller RN ports

	// time of startup
	tm_rd(&startupTimestamp); // = SEC_TIMER;

   /* Initialize i/o */
   loadResetCounters();  // load counters for watchdog and powerlow resets

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
   // THIS_DEVICE = (di_remoteAddress);
   THIS_DEVICE = 0;  // startup with no device number
   THIS_DEVICE_b = 0; // station2bit(THIS_DEVICE);
   // Set the diverter mapping based on the Remote ID# .. also sets MY_STATION
   setDiverterMap(param.portMapping);

   // flash the plc number
   //flasher(0);
   for (i=0; i<MY_STATION; i++)
   {  msDelay(200);
      ledOut(0,1);
      msDelay(150);
      ledOut(0,0);
      hitwd();
   }

   // setup interrupt handler for arrival optics
   #if __SEPARATE_INST_DATA__ && FAST_INTERRUPT
      interrupt_vector ext1_intvec my_isr1;
   #else
      SetVectExtern3000(1, my_isr1);
   #endif
   arrivalEnable(FALSE);  // Disable for now

#ifdef USE_TCPIP
	// setup default static IP using THIS_DEVICE (myIPaddress = MY_BASE_IP + THIS_DEVICE)
	sprintf(myIPaddress, MY_BASE_IP, param.system, THIS_DEVICE);
   printf("My default IP address: %s\n", myIPaddress);
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

   // flash output lamps
   exercise_io();

   // Initialize card reader
   // Input capture 0, zero bit on port F:3, one bit port F:5, 26 bits
   //numbits=52;
   initCardReader();
	sendDeviceInfo(); // send mac & ip to main

   thistime=MS_TIMER;
   lost=0;
	heartbeat_interval=250;
   simonsays=TRUE;
   while(simonsays)
   {  // loop forever

      maintenance();  // Hit WD, flash LEDs, write outputs, tcp tick

      /* check for and process incoming commands */
      //if (get_command(&message)) process_command(message);
      if (getUDPcommand(&UDPmessage) > 0) processUDPcommand(UDPmessage);

      /* check for and process local inputs */
      process_local_io();
      arrival_alert(aaTEST, 0);

      if ((MS_TIMER - heartbeat_timer) > heartbeat_interval)
      {  // Send a UDP heartbeat
	      sendUDPHeartbeat(1);
         heartbeat_timer=MS_TIMER;
      }

   } /* end while */
}
/********************************************************/
//unsigned long arrival_time;
//char whichCA;
unsigned long arrivalDuration;
char arrivalISRstate;
#define AISR_RISING 1
#define AISR_FALLING 2
nodebug root interrupt void my_isr1()
{
/*
   latchCarrierArrival=TRUE;
   //whichCA=di_carrierArrival; //carrierArrival(255);
   WrPortI(I1CR, &I1CRShadow, 0x00);      // disble external INT1 on PE1
*/
	// New concept ...
   // arrivalEnable sets up for falling edge interrupt
   // Detect that here and then reverse setup to detect rising edge
   // Once rising edge detected then validate duration
   // If duration is good (> x ms) then latch and disable interrupt
   // Otherwise reset interrupt for rising edge since it was previously active
	if (arrivalISRstate == AISR_FALLING)  // Carrier has entered optic zone
   {  // Setup to detect rising edge
		WrPortI(I1CR, &I1CRShadow, 0x0A);
      arrivalDuration = MS_TIMER;
      arrivalISRstate = AISR_RISING;
   } else if (arrivalISRstate == AISR_RISING) // Carrier has left optic zone
   {  //
		WrPortI(I1CR, &I1CRShadow, 0x00);  // disable interrupt
      arrivalDuration = MS_TIMER - arrivalDuration;
      arrivalISRstate = 0;  // No more activity
      if (arrivalDuration > 10) latchCarrierArrival=TRUE;  // Got a good arrival
      // note: 10 ms pulse for a 12 inch carrier is 100 FPS or 61.4 MPH
      else
      {  WrPortI(I1CR, &I1CRShadow, 0x06);  // Too short, reset for another try
         arrivalISRstate = AISR_FALLING;
      }
   }
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
   {	// enable  INT1
   	WrPortI(I1CR, &I1CRShadow, 0x06);		// enable external INT1 on PE1, falling edge, priority 2
      arrivalISRstate = AISR_FALLING;
   } else
   {  // disable INT1
      WrPortI(I1CR, &I1CRShadow, 0x00);      // disble external INT1 on PE1
      arrivalISRstate = 0;
   }
}

void setDiverterMap(char portMap)
{	// Sets up the diverter port mapping based on the supplied parameter
	// Also sets the parameter opMode to standard or head-diverter
   // Entry in diverter_map is the binary diverter port number (1,2,4,8)

	param.opMode = STD_REMOTE;
	// MY_STATION = THIS_DEVICE; Station not set by THIS_DEVICE, instead by param.portMapping
   MY_STATION = portMap;
	IO_SHIFT = 0;
	diverter_map[0] = 0; diverter_map[1] = 0;
	diverter_map[2] = 0; diverter_map[3] = 0;
	diverter_map[4] = 0; diverter_map[5] = 0;
	diverter_map[6] = 0; diverter_map[7] = 0;
	diverter_map[8] = 0; // for main to main
}

void init_io()
{
   char i;
   for (i=0; i<4; i++)          // four or five output ports
   {
      outputvalue[i] = 0;
      remoteoutput[i] = 0;
   }
#ifdef USE_BLOWER
   blower(blwrOFF);
#endif
   alarm(OFF);
   do_doorUnLock(OFF);
   secureCard.id=0;
   secureCard.time=0;
   handleSecure=FALSE;
   arrivalTime=0;
   purge_mode=0;
}
void exercise_io()
{
//   if (!wderror())
//   {
      alert(ON, IO_STATION);
      inUse(ON, IO_STATION);
      inUse2(ON, IO_STATION);
      maintenance(); msDelay(1000); //maintenance(); msDelay(1000);
      //inUse(ON, MY_STATION); maintenance(); msDelay(200);
      inUse(OFF, IO_STATION); inUse2(OFF, IO_STATION); alert(OFF, IO_STATION);
      maintenance(); msDelay(200);

      inUse(ON, IO_STATION); inUse2(ON, IO_STATION); alert(ON, IO_STATION);
      maintenance(); msDelay(200);
//   }
   inUse(OFF, IO_STATION);
   inUse2(OFF, IO_STATION);
   alert(OFF, IO_STATION);
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
   	msgSequence=0; // rolling packet sequence number
      rts_latch=0;
      rts2_latch=0;
      transStation=0;
      diverterStation=0;
      headDivStation=0;
      ready4Trans=0;
      ackEvent=0;
      carrier_attn=0;
      carrier_attn2=0;
      //arrival_time=0;
      arrival_from=0;
      //LA_Timer=0;
      mainStation=0;
      param.subStaAddressing=FALSE;
   }

   /* Check for diverter and blower attention */
#ifdef USE_DIVERTER
   processDiverter();
#endif
#ifdef USE_BLOWER
   processBlower();
#endif
   /* Use local buffers for some inputs */
   if (param.opMode == STD_REMOTE)
   {  // stuff for standard remote inputs
      rts_data=requestToSend(MY_STATION);
      rts2_data=requestToSend2(MY_STATION);
      cic_data=carrierInChamber(MY_STATION);
      door_closed=doorClosed(MY_STATION);

      /* Latch request_to_send for new requests */
      //rts_latch |= (rts_data & cic_data & door_closed) & ~carrier_attn & ~rts2_data;  // de-latch when rts2 pressed
      //rts2_latch |= (rts2_data & cic_data & door_closed) & ~carrier_attn2 & ~rts_data;  // de-latch when rts pressed
      // don't latch inputs if in purge mode
      if (purge_mode==0)
      {
	      rts_latch |= (rts_data) & ~carrier_attn & ~rts2_data;  // de-latch when rts2 pressed
   	   rts2_latch |= (rts2_data) & ~carrier_attn2 & ~rts_data;  // de-latch when rts pressed
      }
      /* Clear latch requests from station in current transaction */
      rts_latch &= (isMyStation(transStation) | isMyStation(diverterStation)) ? 0 : 1;
      rts2_latch &= (isMyStation(transStation) | isMyStation(diverterStation)) ? 0 : 1;

      /* Turn off those who have lost their carrier or opened door */
      rts_latch &= cic_data;
      rts2_latch &= cic_data;
      rts_latch &= door_closed;
      rts2_latch &= door_closed;

      // Setup in-use lights for primary and optional secondary in-use lights
      on_bits    = isMyStation(transStation) ? 1 : 0;
      flash_bits = rts_latch | rts_data;
      flash_bits |= ~door_closed;  // flash open doors
      flash_bits |= carrier_attn; // flash stations needing attention  (unremoved delivery)
      on_bits2    = isMyStation(transStation) ? 1 : 0;
      flash_bits2 = rts2_latch | rts2_data;
      flash_bits2 |= ~door_closed;  // flash open doors
      flash_bits2 |= carrier_attn2; // flash stations needing attention  (unremoved delivery)
      if ((mainStation==SLAVE) && (param.subStaAddressing==TRUE))
      		flash_bits2 |= isMyStation(diverterStation) ? 1 : 0;
      else  flash_bits |= isMyStation(diverterStation) ? 1 : 0;

      // if main arrival active, flash inuse at who sent to main
      if (arrival_from & THIS_DEVICE_b)
      {  on_bits =0; //&= ~arrival_from;   // Not on solid
         flash_bits =1; //|= arrival_from; // On flash instead
      }
      // Now ensure on bits overrides flash bits
      flash_bits &= ~on_bits;
      flash_bits2 &= ~on_bits2;

      // set the in use lights as necessary
      //inUse(OFF, ~on_bits & ~flash_bits & MY_STATION & ~station2bit(transStation));
      //inUse(FLASH, flash_bits & ~station2bit(transStation));
      inUse(OFF, ~on_bits & ~flash_bits);
      inUse(ON, on_bits);
      inUse(FLASH, flash_bits);
      //inUse2(OFF, ~on_bits2 & ~flash_bits2 & MY_STATION & ~station2bit(transStation));
      //inUse2(FLASH, flash_bits2 & ~station2bit(transStation));
      inUse2(OFF, ~on_bits2 & ~flash_bits2);
      inUse2(ON, on_bits2);
      inUse2(FLASH, flash_bits2);

	   // latch the arrival signal
////	   if (carrierArrival(MY_STATION)) latchCarrierArrival=TRUE;     // latch this value

      // locally handle the alert for door open during transaction
      if (door_closed) doorAlert(OFF);
      else if (isMyStation(transStation)) doorAlert(ON);

      // Check for and process door unlock
      processCardReads();

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
char initCardReader(void)
{
   if (wiegand_init(3, PFDR, 3, PFDR, 5, param.cardNumBits))
   	printf("Card reader init failed\n");
}
void processCardReads(void)
{  // check for card or pin read and latch in gotCardRead or gotPinRead

	unsigned long result[2];
   unsigned long rawResult[2];
	int wr, PIN;

   // check for card read
	wr = wiegand_result(3, rawResult, result);
	if ((wr == WIEGAND_OK) || (wr == WIEGAND_PARITY_ERROR))
   {  // got a card read
   	// check if this is a non-standard format like 35 bit Corporate 1000
      if (param.cardNumBits == 35)
      {  // check for HID Corporate 1000 format
         cardSiteNum = rawResult[1] >> 3 & 0x0FFF;
         cardIDNum = (rawResult[1] << 17 & 0x07) | (rawResult[0] >> 1 & 0x1FFFF);
	      printf(" Got Card %lX %lX hex; %ld %ld decimal;\n", cardSiteNum, cardIDNum, cardSiteNum, cardIDNum);
		} else
      {  // check for standard 26 bit (original WCMC format)
	      cardIDNum = result[0] | (result[1] << 19); // first 32 bits
	      cardSiteNum = result[1] >> 13;
	      printf(" Got Card %lX %lX -> %ld with raw data %lX %lX\n", cardSiteNum, cardIDNum, (cardIDNum - CARDBASE), result[0], result[1]);
      }
      // validate if the card is ok
      ///if (((cardSiteNum+CARDHBOFFSET) == param.cardL3Digits)
      /// && ((cardIDNum-CARDLBOFFSET) >= param.cardR9Min)
      /// && ((cardIDNum-CARDLBOFFSET) <= param.cardR9Max))
      if ((param.cardSiteCode==0) || (cardSiteNum == param.cardSiteCode)) // accept any card from this site or any card if par=0
      {  printf(" Card is valid\n");
	      gotCardRead = TRUE;
      }
      else sendCardScan(cardIDNum, cardSiteNum, 0, 0); // invalid card

	}

   // check for PIN read if in a secure transaction
   if (handleSecure)
   {  do_secureTransLight(ON);
	   wr = wiegand_PIN_result(&PIN);
	   if (wr == WIEGAND_OK)
	   {  if (PIN == securePIN)
	   //{  if (PIN == 2321)
	      {  // PIN matches securePIN
	         printf(" Got PIN %d\n", PIN);
	         gotPinRead = TRUE;
	      }
	   }
   } else do_secureTransLight(OFF);

   // what to do
   if (gotCardRead)
   {
      if (isMyStation(transStation))
      {  // we're busy so ignore card read
         gotCardRead=FALSE;
         // if there is a returning transaction from me
         // then may be due to an auto or manual return
         // therefore clear the handling of secure transactions
         if (systemDirection == DIR_RETURN) handleSecure=FALSE;
         // send card read, but busy and no action
         sendCardScan(cardIDNum, cardSiteNum, 1, 0);
      }
      else
      {  // no active transaction, so what next
	   	if (handleSecure)
         {  // we need to deal with secure, did we get a pin also
            if (gotPinRead)
	         {  // Got a valid PIN number on a secure transaction
	            unlockDoor();   // Unlatch door
	            // Return card number to host (in return inputs x2)
	            // Reset operations
	            handleSecure=FALSE;
	            gotPinRead=FALSE;
	            gotCardRead=FALSE;
			      secureCard.id = cardIDNum;
               secureCard.time = SEC_TIMER - arrivalTime;
		         // send card scan via secure transaction process & message
	         }
         }
         else
         {  // not doing anything else so just open door
         	unlockDoor();
            gotCardRead=FALSE;
	         // send card read, opened door
            sendCardScan(cardIDNum, cardSiteNum, 1, 1);
         }
      }
   }

}
void checkMalfunction()
{  // take care of business when malfunction changes state
	// uses malfunctionActive
   static char lastMalfunction;   // to keep track of when malfunctionActive changes

   if (lastMalfunction != malfunctionActive)
   {  // malf activated or reset?
      if (malfunctionActive)
      {  // malfunction
         alarm(ON);
         arrival_alert(aaRESET, 0);
      } else
      {  // reset
         alarm(OFF);
         #ifdef USE_BLOWER
            blwrError=FALSE;       // reset blower error
         #endif
         arrivalEnable(FALSE);  // Disable arrival interrupt
         // Resetting of handleSecure may be a problem because it could lead to
         // the ability to remove a secure substance when the main station resets
         // handleSecure = FALSE;
      }
      lastMalfunction = malfunctionActive;
   }

}
/********************************************************/
void processSubCommand(char command)
{
   char ok;
   char i;
   char *sid;
   //static char system_state, new_state;
   static unsigned long lastSecureRemoveMsg;

   #GLOBAL_INIT
   {
      lastSecureRemoveMsg=0;
   }

   /* determine what type of command */
   switch(command)
   {
      case ARE_YOU_READY:
         break;

      case SET_DIVERTER:
         break;


      case DIVERTER_STATUS:      // return logic of correct position
         break;

      case RETURN_INPUTS:        /* return general status */
/*
         // main includes some status info here ... pull it out
         arrival_from=message.data[0];
         //new_state=message.data[1];
         // NOT USING IT: system_direction=message.data[2];

         // fill in the rest of the response based on who I am
         if (param.opMode == STD_REMOTE)
         {  // stuff for standard remote
            // is there a Secure Card ID to pass along?
				if (secureCard.id && ((MS_TIMER - lastSecureRemoveMsg)>500))
            {  // instead of INPUTS_ARE need to send SECURE_REMOVAL
               lastSecureRemoveMsg = MS_TIMER;
		         response.command=SECURE_REMOVAL;
				   sid = (char *)&secureCard.id;
	            response.data[0] = *sid++; // secureCardID byte 3
	            response.data[1] = *sid++; // secureCardID byte 2
	            response.data[2] = *sid++; // secureCardID byte 1
	            response.data[3] = *sid++; // secureCardID byte 0
               response.data[4] = (char) (secureCard.time/6);  // 6 second resolution
            } else
				{
	            // Return latched carrier arrival if state is wait_for_remote_arrival
	            if (latchCarrierArrival && system_state==WAIT_FOR_REM_ARRIVE)
	            {  ok=TRUE;
	               gotPinRead=FALSE;  // Reset any PIN and card reads
	               gotCardRead=FALSE;
	            }
	            else ok=FALSE;
	            // Return the arrival signal
	            response.data[REMOTE_ARRIVE]=carrierArrival(MY_STATION);
	            // but if it is none then send latched arrival under some conditions
	            if ((response.data[REMOTE_ARRIVE]==0) && (latchCarrierArrival)
	                && (system_state==WAIT_FOR_REM_ARRIVE))   response.data[REMOTE_ARRIVE]=latchCarrierArrival;
	            response.data[REMOTE_CIC]=carrierInChamber(MY_STATION);
	            response.data[REMOTE_DOOR]=doorClosed(MY_STATION);
	            response.data[REMOTE_RTS]=rts_latch;
	            response.data[REMOTE_RTS2]=rts2_latch;
            }
         }
         */
         break;

		case ACK_SECURE_REMOVAL:
         // Acknowledge of secure card id from station n
			//if (isMyStation(message.station))
         // message.data[0] contains bit-wise stations to acknowledge
 ///        if (message.data[0] & THIS_DEVICE_b) secureCard.id=0;

      	break;

      case SET_OUTPUTS:
         break;

      case CLEAR_OUTPUTS:
         break;

      case DEVICE_INFO:
      	sendDeviceInfo();
         ackEvent=command;  // feedback to main that we got it
         break;

      case SET_PARAMETERS:  // get the blower type and other parameters
      	break;

      case SET_CARD_PARAMS:  // Setup the secure card parameters
/*      	if (message.data[0]==0)
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
         }
 */        break;

      case TRANS_COMPLETE:
         // handle completed transaction
         arrivalEnable(FALSE);  // Disable arrival interrupt - probably redundant
         transStation=0;
         alert(OFF, IO_STATION);  // in case it was on??
         doorAlert(OFF);           // ditto
         // set arrival alert if the transaction is to here
         if (systemDirection == DIR_SEND && isMyStation(transStation))
         {  if (remoteAlert==1)  // and Main says to set arrival alert
            { // OK to set alert
               if ((mainStation==SLAVE) && (param.subStaAddressing==TRUE))
                     carrier_attn2 = 1; //|= station2bit(UDPmessage.station);  // to flash inuse
               else  carrier_attn = 1; //|= station2bit(UDPmessage.station);  // to flash inuse
               arrival_alert(aaSET, 1); //UDPmessage.station);
            }
	         gotPinRead=FALSE;  // Reset any PIN and card reads
	         gotCardRead=FALSE;
            arrivalTime=SEC_TIMER;  // used with secureRemovalTime
         }
         ackEvent=command;  // feedback to main that we got it
         break;

      case MALFUNCTION:
         break;

      case RESET:
         if (param.opMode == STD_REMOTE)
         {  // stuff for standard remote only
            // Turn off all outputs
            inUse(OFF, IO_STATION);           // clear local i/o
            inUse2(OFF, IO_STATION);           // clear local i/o
            alert(OFF, IO_STATION);
            alarm(OFF);
            arrival_alert(aaRESET, 0);
         }
#ifdef USE_BLOWER
         blwrError=FALSE;       // reset blower error
#endif
         arrivalEnable(FALSE);  // Disable arrival interrupt
         /// rts_latch=0;  Try to not clear latch on reset
         //transStation=0;
         //diverterStation=0;
         ready4Trans=0;
         ackEvent=command;  // feedback to main that we got it
         break;
   }

   /* send the response message if any AND the query was only to me */
   //if (response.command && (message.device==THIS_DEVICE))
   //{  msDelay(2); // hold off response just a bit
   //   //response.device=THIS_DEVICE;
   //   send_response(response);
   //}


   return;
}

void processUDPcommand(struct UDPmessageType UDPmessage)
{   // Handles incoming requests from the diverter controller

	static char lastTransStation;  // to keep track of when transStation changes
   static char lastState;

   #GLOBAL_INIT
   {
      lastState=0;
   }

  	//printf("\n  Got UDP message %c at %ld, %d, %d, %ld", UDPmessage.command, UDPmessage.timestamp, latchCarrierArrival, arrivalISRstate, arrivalDuration);
   /* determine what type of command */
   ackCommand=UDPmessage.command;  // always ack whatever command
   switch(UDPmessage.command)
   {
      case REMOTE_PAYLOAD:  // standard data distribution message
	      if (UDPmessage.system == param.system)  // is this for me?
	      {
	         payloadIPaddr = lastIPaddr;  // capture my main host because this is never a broadcast message
	         arrival_from = UDPmessage.data[0];
	         system_state = UDPmessage.data[1];
	         remoteAlert = UDPmessage.data[2];
	         transStation = UDPmessage.data[3];
	         mainStation = UDPmessage.data[4];
	         diverterStation = UDPmessage.data[5];
	         headDivStation = UDPmessage.data[14];
	         param.subStaAddressing = UDPmessage.data[6];
	         transFlags = UDPmessage.data[7];
	         systemDirection = UDPmessage.data[11];
	         purge_mode = UDPmessage.data[12];

	         // handle capture of secure transaction
	         if ((transFlags & FLAG_SECURE) && isMyStation(transStation))
	         {  handleSecure = TRUE;
	            securePIN = (UDPmessage.data[10]<<8) + UDPmessage.data[9];
	            secureCard.id=0;
	            secureCard.time=0;
	         }
	         // handle ack and/or reset of secure transaction
	         if ((system_state == CANCEL_STATE) && isMyStation(transStation)
	              || isMyStation(UDPmessage.data[8]))
	         {  handleSecure = FALSE;
	            secureCard.id=0;
	            secureCard.time=0;
	         }

	         // handle some state changes
	         if (transStation != lastTransStation)
	         {  // is transaction starting or stopping?
	            if (isMyStation(transStation))
	            {  arrivalEnable(TRUE);  // ready latch
	            }
	            else arrivalEnable(FALSE);   // clear the latched arrival

	            // signal if ready (ready4Trans)
	            ready4Trans=transStation; // Negate this assumption if not ready
	            if (transStation > 0)
	            {  // treat as ARE_YOU_READY signal
	               if (isMyStation(transStation) && (param.opMode == STD_REMOTE))
	               {  // stuff for standard remote
	                  ///rts_latch &= ~station2bit(transStation); // clear latch
	                  ///rts2_latch &= ~station2bit(transStation); // clear latch
	                  /* only if door closed, and (CIC or send-to-here) */
	                  if  ( doorClosed(station2bit(MY_STATION))
	                   && ( carrierInChamber(station2bit(MY_STATION)) || systemDirection==DIR_SEND) )
	                  {  // ready
	                     if (param.subStaAddressing && mainStation==SLAVE)
	                     {  inUse2(ON, station2bit(IO_STATION));
	                     }else
	                     {  inUse(ON, station2bit(IO_STATION));
	                     }
	                  }
	                  else
	                  {  ready4Trans=0;                     // not ready
	                     if (param.subStaAddressing && mainStation==SLAVE)
	                           inUse2(OFF, station2bit(IO_STATION));
	                     else  inUse(OFF, station2bit(IO_STATION));
	                     alert(ON, station2bit(IO_STATION));
	                  }
	               }
	            }

	            // reset state change
	            lastTransStation = transStation;
	         }
	         // handle state change of malfunction
	         malfunctionActive = (system_state == MALFUNCTION_STATE);
	         checkMalfunction();

	         // process ackEvents; could happen repeatedly for the same command
	         if (UDPmessage.data[15] > 0) processSubCommand(UDPmessage.data[15]);
	         else ackEvent=0;

	         // process blower using state change
	         if (lastState != system_state)
	         {  // set blower for new state
	            lastState=system_state;
#ifdef USE_BLOWER
	            blower(blowerStateTable[system_state][blwrConfig]);
#endif
	         }
	      }
         break;

      case SET_PARAMETERS:
	      if (UDPmessage.system == param.system)  // is this for me?
	      {
	         param.system = UDPmessage.system;
	         param.blowerType = UDPmessage.data[0];
	         //param.portMapping = UDPmessage.data[MY_STATION];
	         if (param.cardNumBits != UDPmessage.data[10])  // did numbits change
	         {  param.cardNumBits = UDPmessage.data[10];
	            initCardReader();                           // re-init card reader
	         }
	         param.cardSiteStart = UDPmessage.data[11];
	         param.cardSiteLength = UDPmessage.data[12];
	         param.cardIdStart = UDPmessage.data[13];
	         param.cardIdLength = UDPmessage.data[14];
	         if (param.disableLocks != UDPmessage.data[15]);
	         {  // enable or disable locks
	            param.disableLocks = UDPmessage.data[15];
	            unlockDoor();
	         }
	         param.cardSiteCode = *(int *)&UDPmessage.data[16];

	         // Get card parameters too
	         writeParameterSettings();
	         setDiverterMap(param.portMapping);
         }
      	break;

      case SET_CARD_PARAMS:
	      if (UDPmessage.system == param.system)  // is this for me?
	      {
	         param.cardL3Digits = UDPmessage.data[0];
	         param.cardR9Min = *(unsigned long *)&UDPmessage.data[1];
	         param.cardR9Max = *(unsigned long *)&UDPmessage.data[5];
	//??//         param.cardSiteCode = *(int *)&UDPmessage.data[9];
	         writeParameterSettings();
			}
      	break;

      case DEVICE_INFO:
      	sendDeviceInfo();
         break;

      case REGISTER_DEVICE:
      	// take on the device profile
         if (UDPmessage.data[0] > 0) param.portMapping = UDPmessage.data[0];
         param.system = UDPmessage.system;
		   MY_STATION = param.portMapping;
         THIS_DEVICE = UDPmessage.data[1];
		   THIS_DEVICE_b = station2bit(THIS_DEVICE);
         printf("Registering as sys %d, sta %d, dev %d\n", param.system, MY_STATION, THIS_DEVICE);
         writeParameterSettings();

         break;
   }
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
             //else if (!doorClosed( station2bit(i) ) || i==transStation)
             // will only use station 1 in the alert array but transStation is the real station number
             else if (!doorClosed( station2bit(i) ) || isMyStation(transStation))
             {  // no carrier, door open
               // or no carrier, in transit --> reset
               alert_timer[i]=0;
               alert(OFF, station2bit(i));
               carrier_attn &= ~station2bit(i); // clear attn flag
               carrier_attn2 &= ~station2bit(i); // clear attn flag
               // This should only happen with this_station
               handleSecure = FALSE;

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
   remoteoutput[3]=remotedata[3];

   /* write out all outputs .. some may need to be shifted */
//   write4data(addrIUS, outputvalue[devIUS] | remoteoutput[devIUS]);
//   write4data(addrIUF, outputvalue[devIUF] | remoteoutput[devIUF]);
//   outval = ((outputvalue[devAlert] | remoteoutput[devAlert]) >> IO_SHIFT);
//   write4data(addrALT, outval);
//   write4data(addrALM, outputvalue[devAlarm] | remoteoutput[devAlarm]);

   // Write remotedata[devAlert] to output ports O5-O8
//   hv_outval &= ~DOOR_ALERT_MASK;    // turn off door alert bits
//   hv_outval = (remotedata[devAlert] & MY_STATION) >> IO_SHIFT;
//   hv_wr(hv_outval);

}
/******************************************************************/
char carrierInChamber(char stamask)
{  //return (di_carrierInChamber << IO_SHIFT) & stamask;
   return (di_carrierInChamber);
}
/******************************************************************/
char doorClosed(char stamask)
{  // Note: input is normally high on closed.
   //char indata;
   //indata = di_doorClosed;
   //return ((indata << IO_SHIFT) & stamask);
   return (di_doorClosed);
}
/******************************************************************/
char carrierArrival(char stamask)
{  //return (di_carrierArrival << IO_SHIFT) & stamask;
	return (di_carrierArrival);
}

/******************************************************************/
char requestToSend(char stamask)
{
   //char indata;
   //indata = di_requestToSend;
   //return ((indata << IO_SHIFT) & stamask);
   return di_requestToSend;
}
char requestToSend2(char stamask)
{
   //char indata;
   //indata = di_requestToSend2;
   //return ((indata << IO_SHIFT) & stamask);
   return di_requestToSend2;
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
void doorAlert(char how)
{  // still using remoteoutput[devAlert] because thats how it was handled before
	remoteoutput[devAlert] = how;
	return;
}

unsigned long doorUnLockTimer;
void unlockDoor(void)
{  // trigger door output timer
   // if param.disableLocks = 0 then timer to toggle
   // if param.disableLocks <> 0 then no timer, remain unlocked
	#GLOBAL_INIT {doorUnLockTimer=0;}
   if (param.disableLocks) doorUnLockTimer = 0;
	else doorUnLockTimer = MS_TIMER;
   do_doorUnLock(ON);
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
   // outputvalue is checked in the maintenance function to turn alarm on/off
   outputvalue[devAlarm] = how;

   return;
}
/******************************************************************/
#ifdef USE_DIVERTER
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

	   if ((station>0) && (station<9))     // Valid station #
	   {
	      diverter_setting = diverter_map[station];    // mapped setting

	      // if position<>ANY (any=0) and not in position
	      if ((diverter_setting>0) && (diverter_setting!=di_diverterPos))
	      {
	         /* turn on diverter and start timer */
	         do_diverter(ON);

	         diverter_start_time = MS_TIMER;
	         diverter_attention = TRUE;
	         //diverterStation=station;
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
	         //diverterStation=0;
	      }
	   }
	}
#endif
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
#ifdef USE_TCPIP
//long lastIPaddr;
//int  lastPort;
int getUDPcommand(struct UDPmessageType *UDPmessage)
{
	// be sure to allocate enough space in buffer to handle possible incoming messages (up to 128 bytes for now)
   auto char   buf[128];
   auto int    length, retval;
   int idx;
   int lastPort;
   char * UDPbuffer;  // to index into UDPmessage byte by byte

   #GLOBAL_INIT
   {  lastIPaddr = -1; // default to broadcast
   	payloadIPaddr = -1;
	}

   length = sizeof(buf);
   retval = udp_recvfrom(&sock, buf, length, &(longword)lastIPaddr, &(word)lastPort);
   if (retval < -1) {
      printf("Error %d receiving datagram!  Closing and reopening socket...\n", retval);
      sock_close(&sock);
      if(!udp_open(&sock, LOCAL_PORT, -1/*resolve(REMOTE_IP)*/, REMOTE_PORT, NULL)) {
         printf("udp_open failed!\n");
      }
   }
   else if (retval > 0)
   {  // is the command for me?
      // map it into the UDPmessage structure
      UDPbuffer = (char *) UDPmessage;
      for (idx = 0; idx < retval; idx++)
         UDPbuffer[idx] = buf[idx];
      if (retval > sizeof(buf)) printf("RECEIVED LARGE MESSAGE -> INPUT BUFFER TRUNCATED!!\n");
   }
   return retval;
}
int sendUDPcommand(struct UDPmessageType UDPmessage, long ipAddr)
{
	// Send the supplied command over UDP
   // Appends some info to the standard message
   //   .device
   //   .timestamp

   auto int    length, retval;
   //long ipAddr;
   //int port;
   char * UDPbuffer;  // to index into UDPmessage byte by byte

   // fill the packet with additional data
   UDPmessage.system = param.system;  // system id
   UDPmessage.device = THIS_DEVICE;  // device id
   UDPmessage.devType = 2; // remote
   UDPmessage.sequence = ++msgSequence;
   UDPmessage.timestamp = MS_TIMER;

   length = sizeof(UDPmessage); // how many bytes to send
   UDPbuffer = (char *) &UDPmessage;

   /* send the packet */
   retval = udp_sendto(&sock, UDPbuffer, length, ipAddr, REMOTE_PORT);
   if (retval < 0) {
      printf("Error sending datagram!  Closing and reopening socket...\n");
      sock_close(&sock);
      if(!udp_open(&sock, LOCAL_PORT, -1/*resolve(REMOTE_IP)*/, REMOTE_PORT, NULL)) {
         printf("udp_open failed!\n");
      }
   }
}
int sendUDPHeartbeat(char sample)
{  // Sends an INPUTS_ARE message to the host

	struct UDPmessageType HBmessage;
   char * sid;

   HBmessage.command = INPUTS_ARE;
   HBmessage.station = MY_STATION;  // provide the real station number
   HBmessage.data[0] = carrierInChamber(MY_STATION); // CIC
   HBmessage.data[1] = doorClosed(MY_STATION); // Door closed
   HBmessage.data[2] = carrierArrival(MY_STATION); // Arrival
   // but if Arrival is none then send latched arrival under some conditions
   if ((HBmessage.data[2]==0) && (latchCarrierArrival) && (system_state==WAIT_FOR_REM_ARRIVE))
	   HBmessage.data[2] = latchCarrierArrival; // Arrival
   HBmessage.data[3] = rts_latch;  // RTS latch
   HBmessage.data[4] = rts2_latch; // RTS latch - 2nd destination
   HBmessage.data[5] = VERSION; // Version
   HBmessage.data[6] = SUBVERSION; // Sub-version
   HBmessage.data[7] = 0; // nothing
   HBmessage.data[8] = 0; // nothing
   sid = (char *)&secureCard.id;
   HBmessage.data[9] = *sid++; // secureCardID[0]
   HBmessage.data[10] = *sid++; // secureCardID[1]
   HBmessage.data[11] = *sid++; // secureCardID[2]
   HBmessage.data[12] = *sid++; // secureCardID[3]
   HBmessage.data[13] = (char) (secureCard.time/6);
   HBmessage.data[14]=diverterStation;
   HBmessage.data[15]=headDivStation;
   HBmessage.data[17]=ackEvent;
   HBmessage.data[18]=ackCommand;
   HBmessage.data[19]=ready4Trans;
   sendUDPcommand(HBmessage, payloadIPaddr); // ignore return value, will send again periodically

}
void sendDeviceInfo(void)
{  // sends the device information to the host who sent the last message
	struct UDPmessageType message;
	struct tm time;

   char macbuf[6], ipbuf[4];
   if (!ifconfig(IF_ETH0, IFG_HWA, macbuf, IFG_IPADDR, ipbuf, IFS_END))
      printf("  Hardware address is %02x:%02x:%02x:%02x:%02x:%02x at IP %d.%d.%d.%d\n",
         macbuf[0],macbuf[1],macbuf[2],macbuf[3],macbuf[4],macbuf[5],
         ipbuf[3],ipbuf[2],ipbuf[1],ipbuf[0]);
   else
      printf("  No hardware address for this interface.\n");

//   printf("\nCalling ip_print_ifs()...\n\n");
//   ip_print_ifs();

   message.command = DEVICE_INFO;
   message.station = MY_STATION;  // provide the real station number
   message.data[0] = macbuf[0]; // MAC
   message.data[1] = macbuf[1]; // MAC
   message.data[2] = macbuf[2]; // MAC
   message.data[3] = macbuf[3]; // MAC
   message.data[4] = macbuf[4]; // MAC
   message.data[5] = macbuf[5]; // MAC
   message.data[6] = ipbuf[0]; // IP
   message.data[7] = ipbuf[1]; // IP
   message.data[8] = ipbuf[2]; // IP
   message.data[9] = ipbuf[3]; // IP
   message.data[10] = startupTimestamp.tm_year;
   message.data[11] = startupTimestamp.tm_mon;
   message.data[12] = startupTimestamp.tm_mday;
   message.data[13] = startupTimestamp.tm_hour;
   message.data[14] = startupTimestamp.tm_min;
   message.data[15] = startupTimestamp.tm_sec;
   message.data[16] = MY_STATION;

   sendUDPcommand(message, lastIPaddr); // ignore return value, will send again periodically
}
void sendCardScan(unsigned long cardID, unsigned long cardSite, char authorized, char action)
{  // sends the card scan data to the main
	struct UDPmessageType message;

   message.command = CARD_SCAN;
   message.station = MY_STATION;  // provide the real station number
   *(unsigned long *)&message.data[0] = cardID;
   *(unsigned long *)&message.data[4] = cardSite;
   message.data[8] = authorized;
   message.data[9] = action;

   sendUDPcommand(message, payloadIPaddr); // ignore return value, will send again periodically
}

#endif
void maintenance(void)
{  // handle all regularly scheduled calls
	// use of static CHAR was causing PA4 RS485 transmitter to enable without reason ... static INT works ok.
   static int out_alert, out_inuse, out_inuse2, out_doorAlert, out_cic;
   char i;

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

   // handle carrier in chamber light
   // Need to shift CIC input back to (1..4) range for correct output
   out_cic = carrierInChamber(MY_STATION)>>IO_SHIFT;

   // Process alarm output or purge mode flashing
   if (outputvalue[devAlarm] || purge_mode==1)
   {  // override use of inuse and carrier-in-chamber
   	if ((MS_TIMER %300) < 150)
      {  out_inuse=IO_STATION;
      	out_inuse2=IO_STATION;
         out_cic=0;
      } else
      {  out_inuse=0;
      	out_inuse2=0;
         out_cic=IO_STATION;
      }
   }

   // Write out alerts and inUse light
   do_arrivalAlert(0, out_alert & 1);
   do_inUseLight(0, out_inuse & 1);
   do_inUseLight2(0, out_inuse2 & 1);
   do_doorAlert(0, out_doorAlert & 1);
   do_CICLight(0, out_cic & 1);

   // handle door lock reset
   if (doorUnLockTimer && ((MS_TIMER - doorUnLockTimer) > 2500))
   {  // Reset door lock
   	doorUnLockTimer = 0;
	   do_doorUnLock(OFF);
	}
}


/******************************************************************/
char isMyStation(char station)
{  //return (station2bit(station) & MY_STATION);
   return (station2bit(station) & THIS_DEVICE_b);
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

