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
   07-Sep-13   v411 Additional card range block and parameters
	31-Dec-13   v412 Add individual user identification
   06-Jun-14   v413 Send every card scan
   15-Aug-14   v414 Improve com when sending UID's; Set limit to 1100 users
   15-Dec-14   v415
   24-Oct-15   v416 Include Point-to-Point support (card data over RS485)
   02-Jan-16   v417 Support for blower controlled by remote (vacuum pull)
   22-May-16   v418 Setup for card format parameter 26 or 40 bits
   					  cardNumbits = = param.cardFormat
                    Changed lib\wiegand_v2.lib to accept 4-bit pin input
   26-Jan-17   v419 Update mapping of 26 bit to right justify site code w/ card # (one big number)
   05-Nov-17	v420 Mainstream_Touch_Remote first version to support Reach color touchscreen
   					  Including local logging and station selection by touch-screen
               v421 Support for landscape.  Was developed only for portrait.

***************************************************************/
#memmap xmem  // Required to reduce root memory usage
#class auto
#define FIRMWARE_VERSION "TOUCH REMOTE V4.21"
#define VERSION                         4
#define SUBVERSION                        21

// compile as "STANDARD" or "POINT2POINT" remote
#define RT_STANDARD    1
#define REMOTE_TYPE RT_STANDARD
//#define REMOTE_TYPE RT_POINT2POINT
// For Point2Point do not use TCPIP.  Could fix this though (activeStations)

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
#define USE_BLOWER
#use "MAINSTREAM.lib"      Mainstream v4xx function library
//#define WIEGAND_DEBUG
//#define WIEGAND_VERBOSE
#use "wiegand_v2.lib"         RFID card reader interface
// RabbitNet RN1600 Setup
//int DevRN1600;                    // Rabbit Net Device Number Keypad/LCD
int DevRN1100;                   // Rabbit Net Device Number Digital I/O
//configure to sinking safe state
#define OUTCONFIG 0xFFFF

// setup for Reach color LCD
#define	LCD_USE_PORTE				// tell Reach library which serial port to use
//#define REACH_LIB_DEBUG debug
#use Reach101.lib

// Setup interrupt handling for carrier arrival input
// Set equal to 1 to use fast interrupt in I&D space
#define FAST_INTERRUPT 0
int latchCarrierArrival;
void my_isr1();
void arrivalEnable(char how);


// Communications data structures
#define NUMDATA   6
struct iomessage     /* Communications command structure */
{
   char device;
   char command;
   char station;
   char data[NUMDATA];
};
struct stats_type               // transaction summary statistics
{
   long trans_in;               // incoming transactions
   long trans_out;              // outgoing transactions
   int  deliv_alarm;            // incomplete delivery timeout alarm
   int  divert_alarm;           // diverter timeout alarm
   int  cic_lift_alarm;         // carrier lift timeout alarm
   int  door_alarm;             // door open during transaction
};
struct trans_log_type           // transaction logging info
{
   unsigned long trans_num;     // transaction number
   unsigned long start_tm;      // time of departure
   unsigned int duration;       // duration of event in seconds // time of arrival (or alarm)
   char  source_sta;            // where it came from
   char  dest_sta;              // where it went to
   char  status;                // transaction status (0-63) or other event (64-255)
   									  // 0 = no malfunction or other transaction status
                                // 1 = diverter timeout
                                // 2 = carrier exit (lift) timeout
                                // 3 = delivery overdue timeout
                                // 4 = blower timeout
                                // 5 = station not ready after setting diverter or cancel dispatch
                                // 6 = transaction cancelled/aborted (future)
                                // 64 = Door opened event
   char  flags;                 // transaction flags
                                // xxxx xxx1 = Stat Transaction
                                // xxxx xx1x = Carrier return function
                                // xxxx x1xx = Auto return function
                                // xxxx 1xxx = Door opened in transaction
                                // other event flags defined elsewhere
         // IF TRANSLOG GROWS MAKE SURE UDP COMMAND CAN HOLD IT ALL (NUMUDPDATA)
};
// define transaction and event logging status and flags
#define STS_DIVERTER_TOUT  1
#define STS_DEPART_TOUT    2
#define STS_ARRIVE_TOUT    3
#define STS_BLOWER_TOUT    4
#define STS_TRANS_CANCEL   5
#define STS_CANCEL_BY_SEND 6
#define STS_CANCEL_BY_CIC  7
#define STS_CANCEL_BY_REMOTE 8
#define STS_CANCEL_BY_STACK 9
#define LAST_TRANS_EVENT  31
#define FLAG_STAT        0x01
#define FLAG_CRETURN     0x02
#define FLAG_ARETURN     0x04
#define FLAG_DOOROPEN    0x08
#define FLAG_ARETURNING  0x10
#define FLAG_SECURE      0x20
// define event logging status and flags
#define ESTS_DOOROPEN      64
#define EFLAG_MAINDOOR   0x01
#define EFLAG_SUBSDOOR   0x02
#define EFLAG_EXTRA_CIC  0x04
#define ESTS_MANPURGE      65
#define ESTS_AUTOPURGE     66
#define ESTS_SECURE_REMOVAL 32
#define ESTS_CARD_SCAN     33
#define ESTS_UNAUTH_CARD   34
#define ESTS_BLANK        255

#define NUMUDPDATA 18
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
	#define TCPCONFIG 106         // 106 is an empty config for run-time setup
   //#define USE_DHCP
   //#define USE_ETHERNET 1
	#define MAX_UDP_SOCKET_BUFFERS 1
	#define DISABLE_TCP		// Not using TCP
   #define MY_STATIC_IP "192.168.0.11"
   #define MY_BASE_IP "192.168.0.3%d"
   char myIPaddress[18];  // string to hold runtime IP address
	#define MY_IP_NETMASK "255.255.255.0"
	#define LOCAL_PORT   1235      // for inbound messages
//	#define REMOTE_IP    "255.255.255.255" //255.255.255.255" /*broadcast*/
	#define REMOTE_PORT  1234      // for outbound messages
	#use "dcrtcp.lib"
	udp_Socket sock;
	int sendUDPHeartbeat(char sample);
	int sendUDPcommand(struct UDPmessageType message);
	int getUDPcommand(struct UDPmessageType *message);
	void processUDPcommand(struct UDPmessageType message);
#endif
// Including parameter block structure
// Parameters are NOT read to / written from FLASH
#define UIDSize 1000
// UIDLen more than 5 is not supported by UID_Send due to command length limit
#define UIDLen  5

struct par_block_type
{  // These parameters are defined on the fly at run-time
   char opMode;      // STD_REMOTE or HEAD_DIVERTER
   char subStaAddressing; // to indicate if substation can select slave for destination
   char blowerType;
   char portMapping;
   //char cardL3Digits;     // Constant left 3 digits of valid secure cards
	//unsigned long cardR9Min;  // Secure card ID minimum value for the right 9 digits
   //unsigned long cardR9Max;  // Secure card ID maximum value for the right 9 digits
   //char card2L3Digits;     // Constant left 3 digits of valid secure cards
	//unsigned long card2R9Min;  // Secure card ID minimum value for the right 9 digits
   //unsigned long card2R9Max;  // Secure card ID maximum value for the right 9 digits
	char cardCheckEnabled;  // enable individual card id checking
   char cardFormat;        // defines type of card (26 or 40 bits)
   char stationNum;
   char stationName[12];
   char mainName[12];
   char remoteAudibleAlert;
   char orientLandscape;  // which screen orientation
   char reserved[71];
   unsigned long UIDSync; // timestamp (seconds) since last good block
   char UID[UIDSize][UIDLen];
} param;
#define STD_REMOTE    0
#define HEAD_DIVERTER 1
#define SLAVE   0x08      // Station number of the slave
#define SLAVE_b 0x80      // Bit representation

char THIS_DEVICE;       // board address for communications and setup
char THIS_DEVICE_b;
char MY_STATIONS;       // to mask supported stations
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
void send_response(struct iomessage message);
char get_command(struct iomessage *message);
void process_command(struct iomessage message);
void processCardReads(void);
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
char secTimeout(unsigned long ttime);

// TRANSACTION LOG DEFINITIONS
void initTransLog(void);
void addTransaction(struct trans_log_type trans, char xtra);
//void addSecureTransaction(struct secure_log_type);
int getTransaction(long entry, struct trans_log_type *trans);
long findTransaction(unsigned long transNum);
long sizeOfTransLog(void);
void checkSerialFlash(void);
void analyzeEventLog(void);
struct trans_log_type translog;  // can this be encapsulated?
unsigned long transactionCount(void);
void loadTransactionCount(void);
void resetTransactionCount(unsigned long value);
void resetStatistics(void);

// TOUCHSCREEN DEFINITIONS
void lcd_splashScreen(char view);
void lcd_drawScreen(char whichScreen, char *title);
char systemStateMsg;
int lastBkColor;
void lcd_resetMsgBackColor(void);
int lcd_Center(char * string, char font);
char lcd_ShowMenu(char menuLevel, char page);
int lcd_RefreshScreen(char refresh);
void lcd_ProcessButtons(void);
char lcd_SendButton(void);
int lcd_SelectChoice(char *choice, char *label, char *optZero, char *optOne);
char lcd_GetNumberEntry(char *description, unsigned long * par_value,
         unsigned long par_min, unsigned long par_max, char * par_units, char max_digits, char nc_as_ok);
void lcd_ShowKeypad(char opts);
void lcd_showPage(long thisPage, long lastPage);
void lcd_showTransPage(long page, long lastPage, char format);
void lcd_showTransactions(char format);
void lcd_showTransSummary(void);
char lcd_enableAdvancedFeatures(char menu_level, char * description);
char lcd_getPin(char *PIN, char *description);
void lcd_enterSetupMode(char operatorLevel);
char lcd_ProcessMenuItem(char menu_idx, char *menu_level);
void lcd_helpScreen(char view);
void lcd_show_inputs(void);
unsigned long menu_timeout;
void set_lcd_limits(void);
int lcd_max_x;  // to manage orientation
int lcd_max_y;
char echoLcdToTouchscreen;  // to control echo of lcd messages to the touch screen
#define  lcd_NO_BUTTONS        ""
#define  lcd_WITH_BUTTONS      " "
#define  LCD_DIM               0
#define  LCD_BRIGHT            4
#define	BMP_COLOMBO_LOGO		 1
#define	BMP_green_light		 2
#define	BMP_red_light			 3
#define 	BMP_check_box			 4
#define	BMP_check_box_click	 5
#define	BMP_button_on         6
#define	BMP_button_off        7
#define	BMP_button_up         8
#define	BMP_button_dn         9
#define	BMP_input_box			 10
#define	BMP_med_button        11
#define	BMP_med_button_dn     12
#define	BMP_long_button       13
#define	BMP_long_button_dn    14
#define	BMP_92x32_button      15
#define	BMP_92x32_button_dn   16
#define	BMP_med_red_button    17
#define	BMP_med_red_button_dn 18
#define	BMP_med_grn_button    19
#define	BMP_med_grn_button_dn 20
#define	BMP_92x40_button      21
#define	BMP_92x40_button_dn   22
#define	BMP_med_yel_button    23
#define	BMP_med_yel_button_dn 24
#define	BMP_ltgrey_light		 25

#define  BTN_MENU              1
#define  BTN_DIRECTORY         2
//#define  BTN_HELP              3
#define  BTN_STAT              4
#define  BTN_CRETURN           5
#define  BTN_ARETURN           6
#define  BTN_SECURE            7
#define  BTN_FUTURE            8
#define  BTN_OK                10
#define  BTN_CANCEL            11
#define  BTN_SAVE              12
#define  BTN_PREV              13
#define  BTN_NEXT              14
#define  BTN_HELP              15
#define  BTN_YES_ON            16
#define  BTN_NO_OFF            17
#define  BTN_DEL               18
#define  BTN_EXIT              19
#define  BTN_MNU_FIRST         21
#define  BTN_SEND              22

// USER ID MANAGEMENT DEFINITIONS
int UID_Add(char *ID);
int UID_Del(char *ID);
int UID_Search(char *ID);
int UID_Get(int UIndex);
unsigned int UID_Cksum();
int UID_Decode(char *UID, char *Result);
int UID_Encode(char *UID, char *Result);
int UID_Not_Zero(char *UID);
void UID_Clear(char *UID);

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
/* BLOWER BUT NO DIVERTER IN THIS VERSION
#define USE_DIVERTER
#define USE_BLOWER
#define di_diverterPos             (readDigBank(1) & 0x0F)
*/
#define di_shifterPosIdle          1
#define di_shifterPosPrs           0
#define di_shifterPosVac           0


// OUTPUTS
void setDigOutput(int channel, int value);
#define do_shift 0
#define do_doorAlert(which,value)    setDigOutput(0,value)
#define do_arrivalAlert(which,value) setDigOutput(1,value)
#define do_CICLight(which,value)     setDigOutput(2,value)
#define do_inUseLight(which,value)   setDigOutput(3,value)
#define do_doorUnLock(value)   		 setDigOutput(4,value)
//#define do_inUseLight2(which,value)  setDigOutput(5,value)
// for remotes, vac and prs are reversed (prs=O5; vac=O6)
#define do_blowerVac(value)          setDigOutput(5,value)
#define do_blowerPrs(value)          setDigOutput(6,value)
#define do_blower(value)
#define do_secureTransLight(value)   setDigOutput(7,value)
// inUseLight on output 3,4,7 causes RS485 transmitter to enable (PA4)
// Root cause was use of static CHAR elements in maintenance() instead of static INT
//#define do_alarm(value)              setDigOutput(xx+do_shift,value)
/*  BLOWER BUT NO DIVERTER IN THIS VERSION
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
#define UID_ADD            'U'
#define UID_DEL            'u'
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
char transStation, diverterStation;
char malfunctionActive;
char carrier_attn, carrier_attn2;
char system_state, transDirection;
char doorWasOpen;
#define FLAG_SECURE      0x20
char transFlags;
char handleSecure;
char gotCardRead;
char gotPinRead;
int securePIN;
unsigned long cardID_hb, cardID_lb;
int cardNumbits;
void cardReaderInit();
//#define CARDHBOFFSET  119
//#define CARDLBOFFSET  3676144640
// offset to get printed card id number
//#define CARDBASE 0xE8777C00
char purge_mode;            // indicates if main is in purge mode (flash lights differently)

//unsigned long secureCardID;
//unsigned long secureCardTime;
struct {
	//unsigned long id;
   char id[UIDLen];  // same as UIDLen
   unsigned long time;
} secureCard, stdCard, unauthCard;
unsigned long arrivalTime;
static struct stats_type statistics;

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
   struct iomessage message;
   struct UDPmessageType UDPmessage;
   char key, simonsays;
   auto rn_search newdev;
   int status;
	char UID[15];
	char txt[6];
//	unsigned long R[2];
//	unsigned short fac, id;
//	int wr;

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

   // initialize display and show splash screen
   // back door way to toggle screen orientation
   if (di_requestToSend==1) param.orientLandscape = !param.orientLandscape;
   set_lcd_limits();
   lcd_splashScreen(0);

	// allocate xmem for transaction log
	initTransLog();
   loadTransactionCount();   // Read transaction counter from EEPROM

   hitwd();
   init_io();
   arrival_alert(aaRESET, 0);
   toggleLED(255);  // initialize LED outputs
	readParameterSettings();

   /* read board address and set configuration i/o mappings */
   //THIS_DEVICE = (di_remoteAddress);
   THIS_DEVICE = param.stationNum; // get device/station from parameter instead of digital inputs
   if ((THIS_DEVICE < 1) || (THIS_DEVICE > 7)) THIS_DEVICE = 1;  // default to 1 when out of range
   THIS_DEVICE_b = station2bit(THIS_DEVICE);
   lcd_DispText("Device/Station: ",0,0,MODE_NORMAL);
   sprintf(txt, "%d\n", THIS_DEVICE);
   lcd_DispText(txt,0,0,MODE_NORMAL);

   // flash the plc number
   //flasher(0);
   for (i=0; i<THIS_DEVICE; i++)
   {  msDelay(200);
      ledOut(0,1);
      msDelay(150);
      ledOut(0,0);
      hitwd();
   }
   // if (PRINT_ON) printf("\nWD and PL Reset Counters %d  %d", watchdogCount(), powerlowCount());
   // Set the diverter mapping based on the Remote ID#
   setDiverterMap(param.portMapping);

   // setup interrupt handler for arrival optics
   #if __SEPARATE_INST_DATA__ && FAST_INTERRUPT
      interrupt_vector ext1_intvec my_isr1;
   #else
      SetVectExtern3000(1, my_isr1);
   #endif
   arrivalEnable(FALSE);  // Disable for now

#ifdef USE_TCPIP
	// setup default static IP using THIS_DEVICE (myIPaddress = MY_BASE_IP + THIS_DEVICE)
	sprintf(myIPaddress, MY_BASE_IP, THIS_DEVICE);
   //printf("My IP address: %s\n", myIPaddress);
   lcd_DispText("IP address: ",0,0,MODE_NORMAL);
   lcd_DispText(myIPaddress,0,0,MODE_NORMAL);
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

   // Initialize serial port 1
   enable_commands();

   // Initialize card reader
   cardReaderInit();

   // setup screen
	lcd_drawScreen(1, "TEST");
   systemStateMsg=4; // INITIAL systemStateMsg PAUSED
	lcd_RefreshScreen(1); // force refresh of messages

   thistime=MS_TIMER;
   lost=0;
	heartbeat_interval=100;
   simonsays=TRUE;
   while(simonsays)
   {  // loop forever
      maintenance();  // Hit WD, flash LEDs, write outputs, tcp tick
		lcd_RefreshScreen(0);
      lcd_ProcessButtons();

      /* check for and process incoming commands */
      if (get_command(&message)) process_command(message);
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
void cardReaderInit()
{
   // Initialize card reader
   // Input capture 0, zero bit on port F:3, one bit port F:5, 26 bits
   cardNumbits=param.cardFormat;
   if (wiegand_init(3, PFDR, 3, PFDR, 5, cardNumbits))
   	printf("Card reader init failed\n");
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

	// At the moment, station number is tied to device number
	param.opMode = STD_REMOTE;
	MY_STATIONS = 1; //THIS_DEVICE;
	IO_SHIFT = 0;
	diverter_map[0] = 0; diverter_map[1] = 0;
	diverter_map[2] = 0; diverter_map[3] = 0;
	diverter_map[4] = 0; diverter_map[5] = 0;
	diverter_map[6] = 0; diverter_map[7] = 0;
	diverter_map[8] = 0; // for main to main
   initBlower();  // put this here to be similar to diverter software

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
   //secureCard.id=0;
   UID_Clear(secureCard.id);
   UID_Clear(stdCard.id);
   UID_Clear(unauthCard.id);
   secureCard.time=0;
   stdCard.time=0;
   unauthCard.time=0;
   handleSecure=FALSE;
   arrivalTime=0;
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
      doorWasOpen=0;
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
      rts_data=requestToSend(MY_STATIONS);  // check for digital input send button
      //rts2_data=requestToSend2(MY_STATIONS);
      if (rts_data==0) rts_data=lcd_SendButton();  // if no digital button then try screen button
      rts2_data=0; //requestToSend2(MY_STATIONS);
      cic_data=carrierInChamber(MY_STATIONS);
      door_closed=doorClosed(MY_STATIONS);

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
      //inUse(OFF, ~on_bits & ~flash_bits & MY_STATIONS & ~station2bit(transStation));
      //inUse(FLASH, flash_bits & ~station2bit(transStation));
      inUse(OFF, ~on_bits & ~flash_bits);
      inUse(ON, on_bits);
      inUse(FLASH, flash_bits);
      //inUse2(OFF, ~on_bits2 & ~flash_bits2 & MY_STATIONS & ~station2bit(transStation));
      //inUse2(FLASH, flash_bits2 & ~station2bit(transStation));
      inUse2(OFF, ~on_bits2 & ~flash_bits2);
      inUse2(ON, on_bits2);
      inUse2(FLASH, flash_bits2);

	   // latch the arrival signal
////	   if (carrierArrival(MY_STATIONS)) latchCarrierArrival=TRUE;     // latch this value

      // locally handle the alert for door open during transaction
      if (door_closed) doorAlert(OFF);
      else
      {  if (isMyStation(transStation))
      	{	doorAlert(ON);
            doorWasOpen=TRUE;
         }
      }


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
void processCardReads(void)
{  // check for card or pin read and latch in gotCardRead or gotPinRead

	unsigned long R[2];
	int wr, PIN;
	unsigned long test;
	unsigned long test2;
	char Buf[14];
	static char Uid[5];

   // check for card read
	wr = wiegand_result(3, NULL, R);
	if (wr == WIEGAND_OK)
   {  // got a card read
   	if (cardNumbits == 40)
      {	cardID_lb = R[0] | (R[1] << 19); // first 32 bits
	      cardID_hb = R[1] >> 13;
      } else if (cardNumbits == 26)
      {  cardID_lb = (R[0] | (R[1] << 12)) & 0xFFFFFF;  // card number (16 bits) w/ site code
      	cardID_hb = 0;//(R[1] >> 4);                      // site code (8 bits)
      } else // unknown format
      {  cardID_lb = 0;
         cardID_hb = 0;
      }
      // stuff lb,hb into Uid
	   Uid[0] = (char)cardID_hb;
	   Uid[1] = (char)((cardID_lb >> 24) & 0xFF);
	   Uid[2] = (char)((cardID_lb >> 16) & 0xFF);
	   Uid[3] = (char)((cardID_lb >>  8) & 0xFF);
	   Uid[4] = (char)((cardID_lb)       & 0xFF);
		UID_Decode(Uid, Buf);
      printf(" Got Card %lX %lX -> %s\n", cardID_hb, cardID_lb, Buf);
      // check against card block 1 or 2
///      if ( (((cardID_hb+CARDHBOFFSET) == param.cardL3Digits)
///         && ((cardID_lb-CARDLBOFFSET) >= param.cardR9Min)
///         && ((cardID_lb-CARDLBOFFSET) <= param.cardR9Max))
///        || (((cardID_hb+CARDHBOFFSET) == param.card2L3Digits)
///         && ((cardID_lb) >= param.card2R9Min)
///         && ((cardID_lb) <= param.card2R9Max)) )

		///if ((param.cardCheckEnabled == FALSE) || (UID_Search(Uid) >= 0))
      ///{  printf(" Card is valid\n");
	   ///   gotCardRead = TRUE;
      ///}
      gotCardRead = TRUE; // Always accept a card read regardless of param and search

	} //else if (gotCardRead==FALSE) secureCardID = 0; // only clear if not pending xmit

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
         if (transDirection == DIR_RETURN) handleSecure=FALSE;
      }
      else
      {  // no active transaction, so what next
         // 3 choices
         //   Card check not enabled
         //   Card check enabled and card found
         //   Card check enabled and card not found
         // Choice 1 & 2
         if ((param.cardCheckEnabled == FALSE) || (UID_Search(Uid) >= 0))
         {  // Either card check not enabled or is enabled and card found
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
	               //secureCard.id = cardID_lb;
	               memcpy(secureCard.id, Uid, sizeof(Uid));
	               secureCard.time = SEC_TIMER - arrivalTime;
	            }
	         }
	         else
	         {  // not doing anything else so just open door
	            unlockDoor();
	            gotCardRead=FALSE;
	            // need to send non-secure card reads as well
	            memcpy(stdCard.id, Uid, sizeof(Uid));
	            stdCard.time = SEC_TIMER;

	         }
         } else
         {  // Choice 3 card check enabled and card not found
            // don't unlock door, but do send card data to host
            gotCardRead=FALSE;
            memcpy(unauthCard.id, Uid, sizeof(Uid));
            unauthCard.time = SEC_TIMER;
         }
      }
   }

}
/********************************************************/
void process_command(struct iomessage message)
{  // from RS485 bus
   char wcmd, wdata;
   char ok;
   char i;
   char *sid;
   char scanType;
   //static char system_state, new_state;
   static char new_state;
   struct iomessage response;
   static unsigned long lastSecureRemoveMsg;

	char UID[15];
   char bUID[7];

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
		case UID_ADD:
      case UID_DEL:
         // works for both
      	for (i=0; i<UIDLen; i++) bUID[i] = message.data[i];
         UID_Decode(bUID, UID);
         if (message.command == UID_ADD) UID_Add(UID);
         else if (bUID[0]=='d') UID_Del("ALL"); // clear whole list
         	  else UID_Del(UID);             // delete just this one
         UID[14]=0; // null term for printf
         printf("%s UID %s\n", (message.command == UID_ADD) ? "ADD" : "DEL", UID);

      	break;

      case ARE_YOU_READY:
// debug to see what may be causing loss of communication
//lastcommand=message.command;
         // enable arrival interrupt / latch
         if (param.opMode == STD_REMOTE) arrivalEnable(TRUE);  // don't enable interrupt for head diverter
         response.command=ARE_YOU_READY;
         // assume ready for now
//#warnt "NEED TO FIX LOGIC OF HANDLING DEVICE > 7"
         response.data[0]=THIS_DEVICE_b;   // CONSIDER TO SEND DECIMAL NUMBER AND LET HOST ASSEMBLE THE COLLECTIVE RESPONSE
         transStation=message.station;
         transDirection=message.data[0];
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

         if (param.opMode == STD_REMOTE)
         {  // stuff for standard remote
            if (param.subStaAddressing && message.data[1]==SLAVE)
               inUse2(FLASH, station2bit(message.station));
            else
	            inUse(FLASH, station2bit(message.station));
#ifdef USE_DIVERTER
            setDiverter(message.station);
#endif
         } else
         {  // stuff for head diverter
#ifdef USE_DIVERTER
            setDiverter(message.data[0]);  // head diverter comes in different byte
#endif
         }
         mainStation=message.data[1];  // get the main station to be used
         param.subStaAddressing=message.data[2];
         param.blowerType = message.data[3];
         transFlags = message.data[4];    // transaction flags such as STAT & Secure
			if (transFlags & FLAG_SECURE) handleSecure = TRUE;
         break;


      case DIVERTER_STATUS:      // return logic of correct position
// debug to see what may be causing loss of communication
//lastcommand=message.command;
			// capture the securePIN
         if (isMyStation(message.station)) securePIN = (message.data[2]<<8) + message.data[3];
         // return response
         response.command=DIVERTER_STATUS;
         // which port dependent on opMode
         if (param.opMode == STD_REMOTE) i = message.station;
         else i = message.data[0];
         // is that port aligned? or not used?
			response.data[0]=THIS_DEVICE_b; // DIVERTER_READY;
/*
         if ( (di_diverterPos == diverter_map[i]) || (diverter_map[i] == 0) )
         {  response.data[0]=1<<(THIS_DEVICE-1); // DIVERTER_READY;
         } else
         { response.data[0]=0;            // DIVERTER_NOT_READY;
         }
*/
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
         response.station=MY_STATIONS;  // return this to enlighten main

         // fill in the rest of the response based on who I am
         if (param.opMode == STD_REMOTE)
         {  // stuff for standard remote
            // is there a Secure Card or Std Card ID to pass along?


				// determine if there is a secure or standard card swipe
	         sid = secureCard.id;   // maybe nothing maybe something
	         if (UID_Not_Zero(secureCard.id))
	         {  //sid = secureCard.id; already set
	            scanType = 1; // secure ID
	         }
	         else if (UID_Not_Zero(stdCard.id))
	         {  sid = stdCard.id;
	            scanType = 2; // standard ID
				}
            else if (UID_Not_Zero(unauthCard.id))
            {  sid = unauthCard.id;
            	scanType = 3; // unauthorized ID
            }

				if (UID_Not_Zero(sid) && ((MS_TIMER - lastSecureRemoveMsg)>500))
            {  // instead of INPUTS_ARE need to send SECURE_REMOVAL
               lastSecureRemoveMsg = MS_TIMER;
		         response.command=SECURE_REMOVAL;
	            response.data[0] = *sid++; // secureCardID byte 3
	            response.data[1] = *sid++; // secureCardID byte 2
	            response.data[2] = *sid++; // secureCardID byte 1
	            response.data[3] = *sid++; // secureCardID byte 0
               //response.data[4] = (char) (secureCard.time/6);  // 6 second resolution
               response.data[4] = *sid++; // 5 bytes and time is calculated
               response.data[5] = scanType; // scan type Secure or Std
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
            // Return the arrival signal
            if (ok) response.data[REMOTE_ARRIVE]=MY_STATIONS;
            else response.data[REMOTE_ARRIVE]=0;
            response.data[REMOTE_CIC]=0;
            response.data[REMOTE_DOOR]=0; // HD has no stations  MY_STATIONS;
            response.data[REMOTE_RTS]=0;
#ifdef USE_BLOWER
            if (blwrError==FALSE) response.data[REMOTE_RTS2]=blowerPosition();
            else response.data[REMOTE_RTS2]=0xFF; // Blower has an error
#else
				response.data[REMOTE_RTS2]=0; // No blower here
#endif
         }
         break;

		case ACK_SECURE_REMOVAL:
         // Acknowledge of secure card id from station n
			//if (isMyStation(message.station))
         // message.data[0] contains bit-wise stations to acknowledge
         if (message.data[0] & THIS_DEVICE_b)
         {
         	UID_Clear(secureCard.id);
				UID_Clear(stdCard.id);
				UID_Clear(unauthCard.id);
         }

      	break;

      case RETURN_EXTENDED:     /* return extended data */
// debug to see what may be causing loss of communication
//lastcommand=message.command;
          // return watchdog and powerlow counters
         response.command=RETURN_EXTENDED;
         response.data[0]=0;//watchdogCount();
         response.data[1]=0;//powerlowCount();
         response.data[2]=SUBVERSION;
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
			switch (message.data[0])
         {
         case 0:
         	param.cardFormat = message.data[1];
			   cardReaderInit();
	      	break;
      	case 1:
///         	param.cardR9Min = *(unsigned long *)&message.data[1];
	      	break;
         case 2:
///         	param.cardR9Max = *(unsigned long *)&message.data[1];
	      	break;
         case 3:
///         	param.card2L3Digits = message.data[1];
	      	break;
      	case 4:
///         	param.card2R9Min = *(unsigned long *)&message.data[1];
	      	break;
         case 5:
///         	param.card2R9Max = *(unsigned long *)&message.data[1];
				break;
            // last one so save and send the data to remotes
				// writeParameterSettings();  SAVE WILL FOLLOW IMMEDIATELY FROM SET_PARAMETERS
         case 6: // others above probably not used except 0
         	param.cardCheckEnabled = message.data[1];
	      	break;
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
               arrivalTime=SEC_TIMER;  // used with secureRemovalTime
            }
            else if (message.data[0] == DIR_RETURN && isMyStation(message.station))
            {  // capture the main arrival flag
            }
         } // nothing else for head diverter

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
#ifdef USE_BLOWER
         blwrError=FALSE;       // reset blower error
#endif
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
      //response.device=THIS_DEVICE;
      send_response(response);
   }

   // process state change
   if (new_state != system_state)
   {  // set blower for new state
      system_state=new_state;
#ifdef USE_BLOWER
      blower(blowerStateTable[system_state][0]); // remote always uses blwrconfig 0
#endif
   }

   // state change to/from malfunction state
   malfunctionActive = (system_state == MALFUNCTION_STATE);
   // handle state change of malfunction
   checkMalfunction();

   return;
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
void processUDPcommand(struct UDPmessageType UDPmessage)
{   // Handles incoming requests from the diverter controller

	static char lastTransStation;  // to keep track of when transStation changes
	char UID[15];
   char bUID[7];
   int i, j;
   struct tm time;
   static char new_state;

   #GLOBAL_INIT
   {
      new_state=0;
   }

  	//printf("\n  Got UDP message %c at %ld, %d, %d, %ld", UDPmessage.command, UDPmessage.timestamp, latchCarrierArrival, arrivalISRstate, arrivalDuration);
   /* determine what type of command */
   switch(UDPmessage.command)
   {
   	case UID_ADD:
      case UID_DEL:
      	for (i=0; i<UIDLen; i++) bUID[i] = UDPmessage.data[i];
         UID_Decode(bUID, UID);
         if (UDPmessage.command == UID_ADD) UID_Add(UID);
         else if (bUID[0]=='d') UID_Del("ALL"); // clear whole list
         	  else UID_Del(UID);             // delete just this one
         UID[14]=0; // null term for printf
         printf("%s UID %s\n", (UDPmessage.command == UID_ADD) ? "ADD" : "DEL", UID);
      	break;

   	case TRANS_COMPLETE:
         // handle completed transaction
         arrivalEnable(FALSE);  // Disable arrival interrupt - probably redundant
         transStation=0;
         alert(OFF, MY_STATIONS);  // in case it was on??
         doorAlert(OFF);           // ditto
         // set arrival alert if the transaction is to here
         if (isMyStation(UDPmessage.station))
         {
	         if (UDPmessage.data[0] == DIR_SEND)
	         {
            	//if (UDPmessage.data[1]==1)  // and Main says to set arrival alert
               if (param.remoteAudibleAlert)  // and WE say to set arrival alert
	            { // OK to set alert
	               if ((mainStation==SLAVE) && (param.subStaAddressing==TRUE))
	                     carrier_attn2 = 1; //|= station2bit(UDPmessage.station);  // to flash inuse
	               else  carrier_attn = 1; //|= station2bit(UDPmessage.station);  // to flash inuse
	               arrival_alert(aaSET, 1); //UDPmessage.station);
	            }
	            gotPinRead=FALSE;  // Reset any PIN and card reads
	            gotCardRead=FALSE;
	            arrivalTime=SEC_TIMER;  // used with secureRemovalTime
               statistics.trans_in++;
	         } else {
            	statistics.trans_out++;
            }
	         // record in the log

            translog.source_sta=UDPmessage.station;
            translog.status |= UDPmessage.data[3]; // local or global status
            translog.flags = UDPmessage.data[4]; // local or global flags
	         translog.duration=(int)(SEC_TIMER-translog.start_tm);
	         addTransaction( translog, 0 );
            // update alarm statistics
            if (translog.status==3) statistics.deliv_alarm++;            // incomplete delivery timeout alarm
				else if (translog.status==2) statistics.cic_lift_alarm++;
            if (doorWasOpen)
            {	statistics.door_alarm++;
            	doorWasOpen=0;
            }
         }
         // refresh screen on trans complete
        	lcd_RefreshScreen(1); // force refresh of messages

      	break;

      case SET_DIVERTER:
// debug to see what may be causing loss of communication
//lastcommand=message.command;

			if (isMyStation(UDPmessage.station))
         {
	         if (param.opMode == STD_REMOTE)
	         {  // stuff for standard remote
	            if (param.subStaAddressing && UDPmessage.data[1]==SLAVE)
	               inUse2(FLASH, station2bit(UDPmessage.station));
	            else
	               inUse(FLASH, station2bit(UDPmessage.station));
#ifdef USE_DIVERTER
	            setDiverter(UDPmessage.station);
#endif
	         } else
	         {  // stuff for head diverter
#ifdef USE_DIVERTER
	            setDiverter(UDPmessage.data[0]);  // head diverter comes in different byte
#endif
	         }
	         mainStation=UDPmessage.data[1];  // get the main station to be used
	         param.subStaAddressing=UDPmessage.data[2];
	         param.blowerType = UDPmessage.data[3];
	         transFlags = UDPmessage.data[4];    // transaction flags such as STAT & Secure
	         if (transFlags & FLAG_SECURE) handleSecure = TRUE;

	         // starting a transaction?  let's assume
	         translog.trans_num = transactionCount()+1;
	         //eventlog.trans_num = translog.trans_num;
	         translog.start_tm = SEC_TIMER;
	         translog.duration = 0;
	         translog.source_sta = 0;
	         translog.dest_sta = 0;
	         translog.status = 0;
	         translog.flags=transFlags;
	      }



         break;

      case REMOTE_PAYLOAD:  // standard data distribution message
         arrival_from = UDPmessage.data[0];
			new_state = UDPmessage.data[1];
         transStation = UDPmessage.data[3];
         mainStation = UDPmessage.data[4];
         diverterStation = UDPmessage.data[5];
         param.subStaAddressing = UDPmessage.data[6];
         transFlags = UDPmessage.data[7];
         // do we have a secure ack?
         if (isMyStation(UDPmessage.data[8]))
         {  UID_Clear(secureCard.id);  // clear secureCardID
            UID_Clear(stdCard.id);  // and standard card
				UID_Clear(unauthCard.id);
         }
         malfunctionActive = (new_state == MALFUNCTION_STATE);
         transDirection = UDPmessage.data[11];
         purge_mode = UDPmessage.data[12];

         // handle capture of secure transaction
         if ((transFlags & FLAG_SECURE) && isMyStation(transStation))
         {  handleSecure = TRUE;
            securePIN = (UDPmessage.data[9]<<8) + UDPmessage.data[10];
            UID_Clear(secureCard.id);
         }
         // handle reset of secure transaction
         if ((new_state == CANCEL_STATE) && isMyStation(transStation))
         {  handleSecure = FALSE;
         	UID_Clear(secureCard.id);
            UID_Clear(stdCard.id);
				UID_Clear(unauthCard.id);
         }
         // handle some state changes
         if (transStation != lastTransStation)
         {  // is transaction starting or stopping?
         	if (isMyStation(transStation))
            {  arrivalEnable(TRUE);  // ready latch
            }
         	else arrivalEnable(FALSE);   // clear the latched arrival
            // reset state change
            lastTransStation = transStation;
         }
         // handle state change of malfunction
		   checkMalfunction();

      	break;

	   case SET_STATION_NAME:
	      if (UDPmessage.station==69)
	      {  // construct messages and save parameters to flash
	         //buildStationNames();
	         writeParameterSettings();
            printf("Set station name %s\n", param.stationName);
            printf("Set main name %s\n", param.mainName);
	         lcd_drawScreen(1, lcd_WITH_BUTTONS);      // redraw main screen
	         lcd_RefreshScreen(1);
	      } else
	      {  if (UDPmessage.station == param.stationNum)
         	{  // is my station
	         	// save chunk of name
		         i=UDPmessage.data[0];
	   	      for (j=0; j<4; j++) param.stationName[i+j] = UDPmessage.data[j+1];
            } else if (UDPmessage.station == 0)  // 0 is main; 9 is system
            {  // main station name
	         	// save chunk of name
		         i=UDPmessage.data[0];
	   	      for (j=0; j<4; j++) param.mainName[i+j] = UDPmessage.data[j+1];
            } // else not my station
	      }
	      break;

	   case SET_DATE_TIME:
	      tm_rd(&time);  // read all time data
	      time.tm_year=UDPmessage.data[0];
	      time.tm_mon=UDPmessage.data[1];
	      time.tm_mday=UDPmessage.data[2];
	      time.tm_hour=UDPmessage.data[3];
	      time.tm_min=UDPmessage.data[4];
         time.tm_sec=UDPmessage.station;
	      tm_wr(&time);  // write new time data
	      SEC_TIMER = mktime(&time);   // resync SEC_TIMER
	      break;

      case SET_CARD_PARAMS:
///         param.cardL3Digits = UDPmessage.data[0];
///         param.cardR9Min = *(unsigned long *)&UDPmessage.data[1];
///         param.cardR9Max = *(unsigned long *)&UDPmessage.data[5];
///         param.card2L3Digits = UDPmessage.data[9];
///         param.card2R9Min = *(unsigned long *)&UDPmessage.data[10];
///         param.card2R9Max = *(unsigned long *)&UDPmessage.data[14];
			param.cardCheckEnabled = UDPmessage.data[0];
         param.cardFormat = UDPmessage.data[1];
         writeParameterSettings();
		   cardReaderInit();

      	break;
   }
      // process state change
   if (new_state != system_state)
   {  // set blower for new state
      system_state=new_state;
#ifdef USE_BLOWER
      blower(blowerStateTable[system_state][0]); // always config 0 (standard blower)
#endif
	}
   // update system state message EVERY TIME
   if (system_state==IDLE_STATE) systemStateMsg = 1; // READY
   else if (system_state==HOLD_TRANSACTION) systemStateMsg = 4;  // PAUSED
   else if (system_state==MALFUNCTION_STATE) systemStateMsg = 6; // ALARM
   else if (transStation!=THIS_DEVICE) systemStateMsg = 5; // BUSY
   else if (transDirection==DIR_RETURN) systemStateMsg = 3; // SENDING
   else systemStateMsg = 2; // RECEIVING
   if (isMyStation(bit2station(arrival_from))) systemStateMsg = 8; // waiting for removal at main
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
//   hv_outval = (remotedata[devAlert] & MY_STATIONS) >> IO_SHIFT;
//   hv_wr(hv_outval);

}
/******************************************************************/
char carrierInChamber(char stamask)
{  return (di_carrierInChamber << IO_SHIFT) & stamask; }
/******************************************************************/
char doorClosed(char stamask)
{  // Note: input is normally high on closed.
   char indata;
   indata = di_doorClosed;
   return ((indata << IO_SHIFT) & stamask);
}
/******************************************************************/
char carrierArrival(char stamask)
{  return (di_carrierArrival << IO_SHIFT) & stamask; }

/******************************************************************/
char requestToSend(char stamask)
{
   char indata;
   indata = di_requestToSend;
   return ((indata << IO_SHIFT) & stamask);
}
char requestToSend2(char stamask)
{
   char indata;
   indata = di_requestToSend2;
   return ((indata << IO_SHIFT) & stamask);
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
	#GLOBAL_INIT {doorUnLockTimer=0;}
	doorUnLockTimer = MS_TIMER;
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
int getUDPcommand(struct UDPmessageType *UDPmessage)
{
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
   {  // Is there any message
   	if (retval >1)
	   {  // is the command for me?
	      // map it into the UDPmessage structure
	      UDPbuffer = (char *) UDPmessage;
	      for (idx = 0; idx < sizeof(*UDPmessage); idx++)
	         UDPbuffer[idx] = buf[idx];
	      // Return it
      }
	}
   return retval;
}
int sendUDPcommand(struct UDPmessageType UDPmessage)
{
	// Send the supplied command over UDP
   // Appends some info to the standard message
   //   .device
   //   .timestamp

   auto int    length, retval;
   char * UDPbuffer;  // to index into UDPmessage byte by byte

   // fill the packet with additional data
   UDPmessage.device = THIS_DEVICE;  // system id
   UDPmessage.devType = 3; // remote
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
}
int sendUDPHeartbeat(char sample)
{  // Sends an INPUTS_ARE message to the host

	struct UDPmessageType HBmessage;
   char * sid;

   HBmessage.command = INPUTS_ARE;
   HBmessage.station = THIS_DEVICE;  // provide the real station number
   HBmessage.data[0] = carrierInChamber(MY_STATIONS); // CIC
   HBmessage.data[1] = doorClosed(MY_STATIONS); // Door closed
   HBmessage.data[2] = carrierArrival(MY_STATIONS); // Arrival
   // but if Arrival is none then send latched arrival under some conditions
   if ((HBmessage.data[2]==0) && (latchCarrierArrival) && (system_state==WAIT_FOR_REM_ARRIVE))
	   HBmessage.data[2] = latchCarrierArrival; // Arrival
   HBmessage.data[3] = rts_latch;  // RTS latch
   HBmessage.data[4] = rts2_latch; // RTS latch - 2nd destination
   HBmessage.data[5] = VERSION; // Version
   HBmessage.data[6] = SUBVERSION; // Sub-version
   HBmessage.data[7] = 0; // nothing
   HBmessage.data[8] = 0; // nothing initialize
   sid = secureCard.id;   // maybe nothing maybe something
   if (UID_Not_Zero(secureCard.id))
   {  //sid = secureCard.id; already set
		HBmessage.data[8] = 1; // secure ID
	   HBmessage.data[14] = (char) (secureCard.time/6);
   }
   else if (UID_Not_Zero(stdCard.id))
   {  sid = stdCard.id;
		HBmessage.data[8] = 2; // standard ID
	   HBmessage.data[14] = 0; // no timing
   }
   else if (UID_Not_Zero(unauthCard.id))
   {  sid = unauthCard.id;
		HBmessage.data[8] = 3; // unauthorized ID
	   HBmessage.data[14] = 0; // no timing
   }
   HBmessage.data[9] = *sid++; // secureCardID[0]
   HBmessage.data[10] = *sid++; // secureCardID[1]
   HBmessage.data[11] = *sid++; // secureCardID[2]
   HBmessage.data[12] = *sid++; // secureCardID[3]
   HBmessage.data[13] = *sid++; // secureCardID[4]
   sendUDPcommand(HBmessage); // ignore return value, will send again periodically

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
   out_cic = carrierInChamber(MY_STATIONS)>>IO_SHIFT;

   // Process alarm output or purge mode flashing
   if (outputvalue[devAlarm] || purge_mode==1)
   {  // override use of inuse and carrier-in-chamber
   	if ((MS_TIMER %300) < 150)
      {  out_inuse=MY_STATIONS;
      	out_inuse2=MY_STATIONS;
         out_cic=0;
      } else
      {  out_inuse=0;
      	out_inuse2=0;
         out_cic=MY_STATIONS;
      }
   }

   // Write out alerts and inUse light
   do_arrivalAlert(0, out_alert & 1);
   do_inUseLight(0, out_inuse & 1);
   // do_inUseLight2(0, out_inuse2 & 1); output reassigned v417 and later
   do_doorAlert(0, out_doorAlert & 1);
   do_CICLight(0, out_cic & 1);

   // handle door lock reset
   if (doorUnLockTimer && ((MS_TIMER - doorUnLockTimer) > 2500))
   {  // Reset door lock
   	doorUnLockTimer = 0;
	   do_doorUnLock(OFF);
	}
}


/********************************************************
   Communication I/O drivers
*********************************************************/
/* Initialize buffers and counters */
#define BAUD_RATE 19200
#define CMD_LENGTH   12
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
{  //return (station2bit(station) & MY_STATIONS);
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
nodebug char secTimeout(unsigned long ttime)
{
   return (ttime > SEC_TIMER) ? FALSE : TRUE;
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
//////////////////////////
// USER ID FUNCTIONS
//////////////////////////
int UID_Add(char *ID)
{  // add the given 12 byte UID to the list and return the position added (-1 if can't add)
	// first search to make sure ID doesn't already exist
   int i;
   char blank[UIDLen];
   char enc[UIDLen];

   if (UID_Encode(ID, enc) >= 0)
	{  // failed to encode
   	i = UID_Search(enc);
   	if (i == -1)
	   {  // not found so add the first blank entry
	      memset(blank, 0, UIDLen);  // make blank
	      for (i=0; i<UIDSize; i++)
	      {  //
	         if (memcmp(param.UID[i], blank, UIDLen) == 0)
	         {  // stick it here and return i
	            memcpy(param.UID[i], enc, UIDLen);
printf("Adding id %d\n", i);
	            return i;
	         }
	      }
	      // what? no blanks??
	   }
      else
      {  // already in there so return that index
         return i;
      }
   }
   return -1;

}
int UID_Del(char *ID)
{  // clear out one or all user UID's
	// ID is the 12 byte representation
   // return 0 if success, -1 if failed or not found

   char enc[UIDLen];
	int i;

	if (strncmp(ID,"ALL",3)==0)
   {  // erase all UID's
   	for (i=0; i<UIDSize; i++)
      {	memset(param.UID[i], 0, UIDLen);
      }
printf("Deleting All id\n", i);
      param.UIDSync = 0;  // reset sync timestamp
      return 0;
   }
   else
   {  if (UID_Encode(ID, enc) >= 0)
   	{
	      i = UID_Search(enc);
	      if (i >= 0)
         {  memset(param.UID[i], 0, UIDLen);
printf("Deleting id %d\n", i);
         	return 0;
         }
      }
   }
	return -1;
}
int UID_Search(char *ID)
{  // search for the given 5-byte UID and return the position
	// returns index of match or -1 if no match
	int i;
   for (i=0; i<UIDSize; i++)
   {	if (memcmp(param.UID[i], ID, UIDLen) == 0)
   	{
printf("Found id %d\n", i);
      	return i;
      }
   }
	return -1;
}
int UID_Get(int UIndex)
{  // return the (6 or 12 byte)? UID at position UIndex

}
unsigned int UID_Cksum()
{  // calculate checksum of all valid UIDs

}
int UID_Encode(char *UID, char *Result)
{  // convert 12 byte string into 5 byte UID
   // Result must be sized to handle 5 bytes
   // Return -1 if invalid UID or 0 if success

	int i;
   int hval;
   int carry;
   int uidlen;
   unsigned long lval;
   unsigned long llast;
   unsigned long incr;
   char hdig[4];  // high 3 digits
   char ldig[10]; // low 9 digits

   // deal with length of number up to 12 digits
	uidlen = strlen(UID);
   if (uidlen < 10)
   {  // simple math on 4 byte numbers
   	hval = 0;
      lval = atol(UID);
   } else
   { // split after 9th digit and get decimal equivalents
	   strncpy(hdig, UID, uidlen-9); hdig[3]=0;
	   strcpy(ldig, &UID[uidlen-9]);
	   hval = atoi(hdig);
   	lval = atol(ldig);
   }

   // do the math
   incr = 1000000000;
   llast = lval;
   carry=0;
	for (i=0; i<hval; i++)
	{  // long math loop
   	lval = lval + incr;
      if (lval<llast) carry++;
      llast = lval;
	}
   // transfer back into Result
   Result[0] = (char)carry;
	Result[1] = (char)((lval >> 24) & 0xFF) ;
	Result[2] = (char)((lval >> 16) & 0xFF) ;
	Result[3] = (char)((lval >> 8) & 0XFF);
	Result[4] = (char)((lval & 0XFF));

   return 0;
}
int UID_Decode(char *UID, char *Result)
{  // convert 5 byte UID into 12 byte string
   // Result must be sized to handle 12+1 bytes
   // Return -1 if invalid UID or 0 if success
   int i;
   unsigned long lval;
   unsigned long ba;
   unsigned long bm;
   unsigned long lo;
   int hi;
   int hval;

   // setup the math
   lval = (unsigned long)UID[1] << 24;
   lval = lval | (unsigned long)UID[2] << 16;
   lval = lval | (unsigned long)UID[3] << 8;
   lval = lval | (unsigned long)UID[4];
   hval = (int)(UID[0]<<4) + (int)((lval & 0xF0000000)>>28);  // how many times to loop
   lo = lval & 0x0FFFFFFF; // highest byte in hi
   hi=0;
   ba = 0x10000000; // add this
   bm = 1000000000; // mod this
   for (i=0; i<hval; i++)
   {  // do the long math
      lo = lo + ba;   // add
      hi += (int)(lo / bm);  // mod
      lo = lo % bm;   // div
   }
   // now put into string
   if (hi>0)
   {  // longer than 9 digits
	   snprintf(Result, 13, "%d%09lu", hi, lo);
      Result[12]=0;   // null term
   } else
   {  // less or equal to 9 digits
	   sprintf(Result,"%lu", lo);
   }
   return 0;
}
int UID_Not_Zero(char *UID)
{  // checks if UID is all zeros or not
   // returns 0 if all zeros
   // returns 1 if not all zeros
   int i;
   int result;

   result=0;
   for (i=0; i<UIDLen; i++)
   {  if (UID[i] > 0) result=1;
   }
   return result;
}
void UID_Clear(char *UID)
{  // Sets UID to all zeros
	memset(UID, 0, UIDLen);
}
void lcd_splashScreen(char view)
{  // view 0 for initial startup splash (no wait)
   // view 1 for help->about splash (wait for button)
	int loop_count, i;
   char sbuf[15];
   int xoff;

   if (view==0)
	{   for ( loop_count = 0; loop_count < 20; loop_count++)
	   {  if ( (i = lcd_Connect()) == lcd_SUCCESS ) break;
	      hitwd();          // "hit" the watchdog timer
	   }
	   //if ( i!= lcd_SUCCESS ) exit (1);     // exit if no comm with unit
	   lcd_BeepVolume ( 100 );             // default of 200 is TOO loud
	   lcd_Typematic ( 500, 330 );         // delay .5 secs, repeat 3/sec
   }

   // Logo splash screen
	lcd_BFcolorsB( lcd_BLACK, lcd_WHITE);
	lcd_ClearScreen ();
	lcd_Origin( 0, 0 );
	//lcd_screenSaver(LCD_BRIGHT);
   xoff = param.orientLandscape ? 0 : 40; // setup offset for orientation
	lcd_DispBitmap( BMP_COLOMBO_LOGO, 79-xoff, 30+xoff );
   lcd_Font("16");
   lcd_DispText("Colombo Pneumatic Tube Systems", 55-xoff, lcd_max_y-60, MODE_NORMAL);
   lcd_DispText("800-547-2820", 115-xoff, lcd_max_y-40, MODE_NORMAL);
   if (view)
   {  //lcd_DispText("Software version ", 70, 225, MODE_NORMAL);
      lcd_Font("13");
   	lcd_DispText(FIRMWARE_VERSION, 108-xoff, lcd_max_y-20, MODE_NORMAL);
      // show phone numbers for admin, maint, and colombo
      msDelay(4000);  // wait 4 seconds
   } else
   {
	   // Please wait message
	   msDelay(1000);
	   lcd_ClearScreen ();
	   lcd_Font("16B");
	   lcd_DispText("Colombo Pneumatic Tube Systems\n", 0, 0, MODE_NORMAL);
	   //lcd_DispText("Pneumatic Tube System\n", 0, 0, MODE_NORMAL);
	   //lcd_DispText("Software version ", 0, 0, MODE_NORMAL);
	   lcd_DispText(FIRMWARE_VERSION, 0, 0, MODE_NORMAL);
	   lcd_DispText(" ", 0, 0, MODE_NORMAL);
	   lcd_DispText(__DATE__, 0, 0, MODE_NORMAL);
	   lcd_Font("16");
	   lcd_DispText("\nby MS Technology Solutions, LLC", 0, 0, MODE_NORMAL);
      lcd_DispText("\nAvailable extended memory ",0,0,MODE_NORMAL);
      sprintf(sbuf, "%ld\n", xavail((long *)0));
      lcd_DispText(sbuf,0,0,MODE_NORMAL);
   }
   lcd_Font("16");

}
void set_lcd_limits()
{  // set max x,y based on orientation
	if (param.orientLandscape)
   {  lcd_max_x = 319;      // 0 - 319
   	lcd_max_y = 240;
   } else
   {  lcd_max_x = 239;		// 0 - 239
   	lcd_max_y = 320;
   }
}
void lcd_resetMsgBackColor(void) { lastBkColor = -1; }
void lcd_drawScreen(char whichScreen, char *title)
{
	// whichScreen:
   //  1: main screen with text boxes, unless title[0]=0 / ""
   //  2: menu header and bottom buttons
   //  3: menu header only
   //  4: menu header, keypad, OK, cancel
   //  5: header with exit button
   //  6: header keyboard, next, exit
   //  7: header clock up/downs, exit
   //  8: transaction log: pgup, pgdn, help, done
   //  9: header, keypad, next, exit
   // 10: header keyboard, OK, cancel
   int x, dx;
   int xpos, ypos;
	lcd_BFcolorsB( lcd_BLACK, lcd_WHITE);
	lcd_ClearScreen ();
	lcd_Origin( 0, 0 );
   lcd_PenWidth(1);
   lcd_Font("24");
	lcd_resetMsgBackColor();

   // draw title if needed
   if (whichScreen >=2)
   {
	   lcd_BFcolorsB( lcd_BLUE, lcd_WHITE );   // inside color
//	   lcd_Rectangle(5, 5, 235, 40, 1);        // inside paint
	   lcd_Rectangle(1, 1, lcd_max_x-1, 40, 1);        // inside paint
	   lcd_BFcolorsB( lcd_BLACK, lcd_WHITE);   // border color
//	   lcd_Rectangle(5, 5, 235, 40, 0);        // border paint
	   lcd_Rectangle(0, 0, lcd_max_x, 40, 0);        // border paint
	   lcd_Font("24B");
	   lcd_DispText(title, lcd_Center(title, 1), 14, MODE_TRANS);
   }
   // offset for centering
   dx = (param.orientLandscape) ? 40 : 0;

   // now handle screen specific stuff
   switch (whichScreen)
   {
   case 1:
	   if (title[0] != 0) // Buttons? (any) or not ("")
		{
	      {
	         // Draw boxes for bottom buttons
	         //lcd_BFcolorsB( lcd_LBLUE, lcd_LBLUE );   // inside color
            // for send button
            ypos = (param.orientLandscape) ? 237 : 270;
			   lcd_BFcolorsB( lcd_LBLUE, lcd_GREY);  // inside color
	         lcd_Rectangle(5, 179, lcd_max_x-4, ypos, 1);      // inside paint 158 -> 140
	         lcd_BFcolorsB( lcd_BLUE, lcd_WHITE);     // border color
	         lcd_Rectangle(5, 179, lcd_max_x-4, ypos, 0);      // border paint

	         // for menu/help
	         lcd_BFcolorsB( lcd_LBLUE, lcd_GREY );   // inside color
	         lcd_Rectangle(5, 270, lcd_max_x-4, 317, 1);      // inside paint
	         lcd_BFcolorsB( lcd_BLUE, lcd_WHITE);     // border color
	         lcd_Rectangle(5, 270, lcd_max_x-4, 317, 0);      // border paint

	         // Draw bottom buttons
	         lcd_BFcolorsD( lcd_BLACK, lcd_VLTGRAY_D );
	         lcd_Font("16B");
	         lcd_ButtonDef( BTN_MENU,
	            BTN_MOM,                      // momentary operation
	            BTN_TLXY, 13, lcd_max_y-40,            // starting x,y for buttons
	            BTN_TYPE, BUTTON_RELEASE,
	            BTN_MARGINS, 5, 300,         // set left and right margins
	            BTN_SEP, 63, 43,              // set up for button sep
	            BTN_TEXT, "Menu",
	            BTN_TXTOFFSET, 10, 9,
	            BTN_BMP, BMP_med_button, BMP_med_button_dn,
	            BTN_END );

	         lcd_ButtonDef( BTN_HELP,
	            BTN_TLXY, lcd_max_x-69, lcd_max_y-40,
	            BTN_TXTOFFSET, 12, 9,
	            BTN_TEXT, "Help",
	            BTN_BMP, BMP_med_button, BMP_med_button_dn,
	            BTN_END );

	         lcd_Font("32B");
	         lcd_BFcolorsB( lcd_WHITE, lcd_LGREY);     // border color
            ypos = (param.orientLandscape) ? 190 : 204;
            xpos = (param.orientLandscape) ? 117 : 77;
            // make sure to sync positionn with button highlight in lcd_refreshScreen
	         lcd_ButtonDef( BTN_SEND,
	            BTN_TLXY, xpos, ypos,
	            BTN_TEXT, "SEND",
	            BTN_TXTOFFSET, 7, 3,
	            BTN_BMP, BMP_92x40_button, BMP_92x40_button_dn,
	            BTN_END );

	      }
      }
	   // Draw LCD Message boxes
	   lcd_BFcolorsB( lcd_LGREY, lcd_LGREY);  // border color
	   lcd_Rectangle(5, 5, lcd_max_x-4, 60, 1);       // border paint
	   lcd_BFcolorsB( lcd_BLUE, lcd_LGREY);  // border color
	   lcd_Rectangle(5, 5, lcd_max_x-4, 60, 0);       // border paint
	   lcd_Rectangle(5, 60, lcd_max_x-4, 130, 0);     // border paint
	   lcd_Rectangle(5, 130, lcd_max_x-4, 179, 0);     // border paint

      // since this screen was just cleared, for a refresh of the lcd messages
///      reset_msg_timer(NOW);






      break;

   case 2:

	   // Draw bottom buttons
	   lcd_Font("16B");
	   lcd_ButtonDef( BTN_PREV,
	      BTN_MOM,                      // momentary operation
	      BTN_TLXY, 5, lcd_max_y-40,             // starting x,y for buttons
		   BTN_TYPE, BUTTON_RELEASE,
	      BTN_MARGINS, 5, 330,         // set left and right margins
	      BTN_SEP, 63, 43,              // set up for button sep
	      BTN_TEXT, "PgUp",
	      BTN_TXTOFFSET, 8, 9,
	      BTN_BMP, BMP_med_button, BMP_med_button_dn,
	      BTN_END );
	   lcd_ButtonDef( BTN_NEXT,
	      BTN_TEXT, "PgDn",
	      BTN_END );
/*	   lcd_ButtonDef( 1,
	      BTN_TEXT, "",
	      BTN_END );
	   lcd_ButtonDef( BTN_SAVE,
	      BTN_TEXT, "",
	      BTN_END );
*/
	   lcd_ButtonDef( BTN_CANCEL,
	      //BTN_TXTOFFSET, 5, 9,
	      BTN_TLXY, lcd_max_x-69, lcd_max_y-40,             // starting x,y for buttons
	      BTN_TEXT, "Done",
	      BTN_END );

   	break;
   case 3:
      break;
   case 4:

      lcd_ShowKeypad(0);

	   // Draw bottom buttons
	   lcd_Font("16B");
	   lcd_ButtonDef( BTN_OK,
	      BTN_MOM,                      // momentary operation
	      BTN_TLXY, 5, lcd_max_y-40,             // starting x,y for buttons
		   BTN_TYPE, BUTTON_RELEASE,
	      BTN_TEXT, "OK",
	      BTN_MARGINS, 5, 330,         // set left and right margins
	      BTN_TXTOFFSET, 14, 9,
	      BTN_BMP, BMP_med_button, BMP_med_button_dn,
	      BTN_END );
	   lcd_ButtonDef( BTN_CANCEL,
	      BTN_TXTOFFSET, 5, 9,
         BTN_TLXY, lcd_max_x-69, lcd_max_y-40,
	      BTN_TEXT, "Cancel",
	      BTN_END );
      break;
   case 5:

	   // Draw bottom buttons
	   lcd_Font("16B");
	   lcd_ButtonDef( BTN_EXIT,
	      BTN_MOM,                      // momentary operation
	      BTN_TLXY, lcd_max_x-69, lcd_max_y-40,
		   BTN_TYPE, BUTTON_RELEASE,
	      BTN_MARGINS, 5, 330,         // set left and right margins
	      BTN_SEP, 63, 43,              // set up for button sep
	      BTN_TEXT, "Exit",
	      BTN_TXTOFFSET, 15, 9,
	      BTN_BMP, BMP_med_button, BMP_med_button_dn,
	      BTN_END );
      break;
   case 6:  // keyboard entry
///      lcd_drawKeyboard();
	   // Draw bottom buttons
	   lcd_Font("16B");
	   lcd_ButtonDef( BTN_NEXT,
	      BTN_MOM,                      // momentary operation
	      BTN_TLXY, 5, 205,             // starting x,y for buttons
		   BTN_TYPE, BUTTON_RELEASE,
	      BTN_TEXT, "Next",
	      BTN_MARGINS, 5, 330,         // set left and right margins
	      BTN_TXTOFFSET, 8, 9,
	      BTN_BMP, BMP_med_button, BMP_med_button_dn,
	      BTN_END );
	   lcd_ButtonDef( BTN_CANCEL,
	      BTN_MOM,                      // momentary operation
	      BTN_TLXY, lcd_max_x-69, 205,             // starting x,y for buttons
		   BTN_TYPE, BUTTON_RELEASE,
	      BTN_MARGINS, 5, 330,         // set left and right margins
	      BTN_SEP, 63, 43,              // set up for button sep
	      BTN_TEXT, "Exit",
	      BTN_TXTOFFSET, 15, 9,
	      BTN_BMP, BMP_med_button, BMP_med_button_dn,
	      BTN_END );
      break;
   case 7:  // clock with up/downs
	   // Draw bottom buttons
	   lcd_Font("16B");
	   lcd_ButtonDef( BTN_SAVE,
	      BTN_MOM,                      // momentary operation
	      BTN_TLXY, 257, 205,             // starting x,y for buttons
		   BTN_TYPE, BUTTON_RELEASE,
	      BTN_MARGINS, 5, 330,         // set left and right margins
	      BTN_SEP, 63, 43,              // set up for button sep
	      BTN_TEXT, "Exit",
	      BTN_TXTOFFSET, 15, 9,
	      BTN_BMP, BMP_med_button, BMP_med_button_dn,
	      BTN_END );
      // print date/time message
      lcd_Font("16x32i");
///	   lcd_DispText(sys_message[MSG_DATE_TIME].msg, 30, 60, MODE_NORMAL);
      // Draw up/dn buttons
      lcd_ButtonDef( 0, // month +
	      BTN_MOM,                      // momentary operation
	      BTN_TLXY,    46, 100,             // starting x,y for buttons
	      BTN_MARGINS, 46, 264,         // set left and right margins
		   BTN_TYPE, BUTTON_RELEASE,
	      BTN_SEP, 48, 40,              // set up for button sep
	      BTN_TEXT, "+",
	      BTN_TXTOFFSET, 8, 1,
	      BTN_BMP, BMP_button_up, BMP_button_dn,
	      BTN_END );                                        // month
      lcd_ButtonDef( 2, BTN_MOM, BTN_TEXT, "+", BTN_END ); // day
      lcd_ButtonDef( 4, BTN_MOM, BTN_TEXT, "+", BTN_END ); // year
      lcd_ButtonDef( 6, BTN_MOM, BTN_TEXT, "+", BTN_END ); // hour
      lcd_ButtonDef( 8, BTN_MOM, BTN_TEXT, "+", BTN_END ); // min
      lcd_ButtonDef( 1, BTN_MOM, BTN_TEXT, "-", BTN_END );
      lcd_ButtonDef( 3, BTN_MOM, BTN_TEXT, "-", BTN_END );
      lcd_ButtonDef( 5, BTN_MOM, BTN_TEXT, "-", BTN_END );
      lcd_ButtonDef( 7, BTN_MOM, BTN_TEXT, "-", BTN_END );
      lcd_ButtonDef( 9, BTN_MOM, BTN_TEXT, "-", BTN_END );
      break;

   case 8:  // PgUp, PgDn, Help, Done
	   // Draw bottom buttons
	   lcd_Font("16B");
	   lcd_ButtonDef( BTN_PREV,
	      BTN_MOM,                      // momentary operation
	      BTN_TLXY, 1, lcd_max_y-34,             // starting x,y for buttons
		   BTN_TYPE, BUTTON_RELEASE,
	      BTN_MARGINS, 1, 330,         // set left and right margins
	      BTN_SEP, 60, 43,              // set up for button sep
	      BTN_TEXT, "PgUp",
	      BTN_TXTOFFSET, 8, 9,
	      BTN_BMP, BMP_med_button, BMP_med_button_dn,
	      BTN_END );
	   lcd_ButtonDef( BTN_NEXT,
	      BTN_TEXT, "PgDn",
	      BTN_END );
      lcd_ButtonDef( BTN_HELP,
         BTN_TXTOFFSET, 12, 9,
         BTN_TEXT, "Help",
         BTN_END );
	   lcd_ButtonDef( BTN_CANCEL,
	      BTN_TEXT, "Exit",
	      BTN_END );

   	break;
   case 9:

///      lcd_ShowKeypad(1);

	   // Draw bottom buttons
	   lcd_Font("16B");
	   lcd_ButtonDef( BTN_NEXT,
	      BTN_MOM,                      // momentary operation
	      BTN_TLXY, 5, 205,             // starting x,y for buttons
		   BTN_TYPE, BUTTON_RELEASE,
	      BTN_TEXT, "Next",
	      BTN_MARGINS, 5, 330,         // set left and right margins
	      BTN_TXTOFFSET, 8, 9,
	      BTN_BMP, BMP_med_button, BMP_med_button_dn,
	      BTN_END );
	   lcd_ButtonDef( BTN_CANCEL,
	      BTN_MOM,                      // momentary operation
	      BTN_TLXY, lcd_max_x-63, 205,             // starting x,y for buttons
		   BTN_TYPE, BUTTON_RELEASE,
	      BTN_MARGINS, 5, 330,         // set left and right margins
	      BTN_SEP, 63, 43,              // set up for button sep
	      BTN_TEXT, "Exit",
	      BTN_TXTOFFSET, 15, 9,
	      BTN_BMP, BMP_med_button, BMP_med_button_dn,
	      BTN_END );
      break;
   case 10:  // keyboard entry
///      lcd_drawKeyboard();
	   // Draw bottom buttons
	   lcd_Font("16B");
	   lcd_ButtonDef( BTN_OK,
	      BTN_MOM,                      // momentary operation
	      BTN_TLXY, 5, 205,             // starting x,y for buttons
		   BTN_TYPE, BUTTON_RELEASE,
	      BTN_TEXT, "OK",
	      BTN_MARGINS, 5, 330,         // set left and right margins
	      BTN_TXTOFFSET, 14, 9,
	      BTN_BMP, BMP_med_button, BMP_med_button_dn,
	      BTN_END );
	   lcd_ButtonDef( BTN_CANCEL,
	      BTN_TXTOFFSET, 5, 9,
	      BTN_TLXY, lcd_max_x-63, 205,           // starting x,y for buttons
	      BTN_TEXT, "Cancel",
	      BTN_END );
      break;
   }

}
int lcd_Center(char * string, char font)
{
	int temp;

   temp = param.orientLandscape ? 160 : 120;
   if (font==1)      temp = temp - (int)(strlen(string) * 7);  // "24"
   else if (font==2) temp = temp - (int)(strlen(string) * 4);  // "16"
   else if (font==3) temp = temp - (int)(strlen(string) * 9);  // "32"
//   if (temp < 6) temp = 6;
   if (temp < 2) temp = 2;
	return temp;
}
int lcd_RefreshScreen(char refresh)
{  // update system status and local status
   // refresh = 0: update screen only when status is changing
   // refresh else: rewrite all messages
   static char mydoor, mycic, myssm;
   char cicmsg;
   int ypos, dx;

   char buf[20];
   struct tm time;
   static unsigned long lastclock;
   char flashSend;
   static char lastFlashSend;
   int bgColor;

	lcd_Font("32B");
   dx = (param.orientLandscape) ? 40 : 0;
   // update system state message
   if ((systemStateMsg != myssm) || (refresh))
	{  // erase last message
	   lcd_BFcolorsB( lcd_LGREY, lcd_LGREY);  // color
	   lcd_Rectangle(10, 10, 234+dx, 55, 1);       // paint
      lcd_BFcolorsB( lcd_BLACK, lcd_LGREY);
      ypos=18;
      if (systemStateMsg == 8)
      {  lcd_Font("24B");
      	lcd_DispText("WAIT FOR REMOVAL", 15+dx, ypos-5, MODE_NORMAL);
         lcd_Font("24B");
         strcpy(buf, "AT ");
         strcat(buf, param.mainName);
         lcd_DispText(buf, lcd_Center(buf,1), ypos+15, MODE_NORMAL);
      }
   	else if (systemStateMsg == 1) lcd_DispText("READY", 75+dx, ypos, MODE_NORMAL);
   	else if (systemStateMsg == 2) lcd_DispText("RECEIVING", 45+dx, ypos, MODE_NORMAL);
   	else if (systemStateMsg == 3) lcd_DispText("SENDING", 59+dx, ypos, MODE_NORMAL);
   	else if (systemStateMsg == 4) lcd_DispText("PAUSED", 65+dx, ypos, MODE_NORMAL);
   	else if (systemStateMsg == 5) lcd_DispText("BUSY", 80+dx, ypos, MODE_NORMAL);
   	else if (systemStateMsg == 6) lcd_DispText("ALARM", 75+dx, ypos, MODE_NORMAL);
   	else if (systemStateMsg == 7) lcd_DispText("QUEUED", 65+dx, ypos, MODE_NORMAL);
      else lcd_DispText("UNKNOWN", 50+dx, 18, MODE_TRANS);
		//lcd_BFcolorsB( lcd_GREEN, lcd_LGREY);
      myssm = systemStateMsg;
   }

   // what is carrier status 1, 2, 3
   // WHAT ABOUT IN-ROUTE TO (FROM) MAIN
   if (isMyStation(transStation))
   { 	cicmsg = 5; bgColor = lcd_WHITE; // sending to/from main
   } else if (carrier_attn)
   {  cicmsg = 1; bgColor = lcd_YELLOW;
   } else if (rts_latch)
   {  cicmsg = 4; bgColor = lcd_MAGENTA;
   } else if (carrierInChamber(MY_STATIONS))
   {  cicmsg = 2; bgColor = lcd_GREEN;
   } else
   {  cicmsg = 3; bgColor = lcd_WHITE;    // no carrier nothing else active
   }

   // update carrier message
   if ((cicmsg != mycic) || refresh)
	{  // status has changed, update message
   	// draw colored box
		lcd_Font("32B");
	   lcd_BFcolorsB( bgColor, bgColor);
	   lcd_Rectangle(6, 61, 234+dx+dx, 129, 1);   // Fill the area
      lcd_BFcolorsB( lcd_BLACK, bgColor);
      if (cicmsg==3) // no carrier
      {  // different message style for this
			lcd_Font("24B");
	      lcd_DispText("INSERT CARRIER", 32+dx, 70, MODE_NORMAL);
	      lcd_DispText("AND PRESS SEND", 28+dx, 97, MODE_NORMAL);
		} else if (cicmsg==5)
      {  // in route to main
	   	if (transDirection==DIR_SEND) strcpy(buf, "IN ROUTE FROM");
         else strcpy(buf, "IN ROUTE TO");
         lcd_DispText(buf, lcd_Center(buf,3), 67, MODE_NORMAL);
         lcd_DispText(param.mainName, lcd_Center(param.mainName,3), 97, MODE_NORMAL);
         refresh=TRUE;  // Force update of door message
      } else
	   {  // 1st line "CARRIER"
      	lcd_DispText("CARRIER", 60+dx, 67, MODE_TRANS);
	      if (cicmsg==1)
	      {  // carrier arrival
	         //lcd_BFcolorsB( lcd_RED, lcd_WHITE);
	         lcd_DispText("ARRIVED ", 63+dx, 97, MODE_NORMAL);
	      } else if (cicmsg==2)
	      { // carrier in chamber
	         //lcd_BFcolorsB( lcd_GREEN, lcd_WHITE);
	         lcd_DispText(" READY   ", 63+dx, 97, MODE_NORMAL);
	      } else if (cicmsg==4)
	      { // carrier in chamber
	         //lcd_BFcolorsB( lcd_MAGENTA, lcd_WHITE);
	         lcd_DispText("QUEUED  ", 63+dx, 97, MODE_NORMAL);
	      }
      }
      mycic = cicmsg;
	}
   // update door message
   if ((doorClosed(MY_STATIONS) != mydoor) || refresh)
   {  // door status has changed or forced refresh
		lcd_Font("32B");
	   if (doorClosed(MY_STATIONS))
	   { // door is closed
         lcd_BFcolorsB( lcd_WHITE, lcd_WHITE);
         lcd_Rectangle(6, 131, 234+dx+dx, 178, 1);   // Fill the area
         lcd_BFcolorsB( lcd_BLACK, lcd_WHITE);
	      if (isMyStation(transStation))
         {  // busy, do not use
				lcd_Font("24B");
	         lcd_DispText("BUSY DO NOT USE", 20+dx, 140, MODE_TRANS);
         }
         else
         {  // normal
	         lcd_DispText("DOOR CLOSED", 20+dx, 140, MODE_TRANS);
         }
	   } else
	   { // door is opened
	      lcd_BFcolorsB( lcd_YELLOW, lcd_YELLOW);
		   lcd_Rectangle(6, 131, 234+dx+dx, 178, 1);   // Fill the area
	      lcd_BFcolorsB( lcd_BLACK, lcd_YELLOW);
		   lcd_DispText("DOOR OPEN", 40+dx, 140, MODE_TRANS);
	   }
      mydoor = doorClosed(MY_STATIONS);
   }

   // EMPHASIZE SEND BUTTON WHEN THE PLANETS ALIGN
   // CIC, DOOR CLOSED, SYSTEM READY)
   if ((cicmsg==2) && (doorClosed(MY_STATIONS)) && (systemStateMsg == 1))
   {  // highlight send
      flashSend=TRUE;
   } else
   {  // regular send
      flashSend=FALSE;
   }
   if ((flashSend != lastFlashSend) || refresh)
   {  // state changed
   	if (flashSend)
      {  lcd_BFcolorsB( lcd_BLACK, lcd_GREEN );
      } else
      {  lcd_BFcolorsB( lcd_WHITE, lcd_LGREY );
      }
      lcd_Font("32B");
      ypos = (param.orientLandscape) ? 193 : 207;  // should be button ypos + 3
      // make sure this highlight syncs with the button pos in lcd_drawScreen
      lcd_DispText("SEND", 84+dx, ypos, MODE_NORMAL);
   	lastFlashSend=flashSend;
   }

   // show station name and clock
   if (SEC_TIMER != lastclock)
   {  lastclock=SEC_TIMER;
	   lcd_Font("13B");
	   lcd_BFcolorsB( lcd_BLACK, lcd_LBLUE);
	   lcd_DispText(param.stationName, lcd_Center(param.stationName,2), 276, MODE_NORMAL);
	   tm_rd(&time);  // read all time data
	   sprintf(buf, "%02d/%02d/%02d", time.tm_mon, time.tm_mday, time.tm_year%100);
	   lcd_DispText(buf, 88+dx, 289, MODE_NORMAL);
	   sprintf(buf, "%02d:%02d:%02d", time.tm_hour, time.tm_min, time.tm_sec);
	   lcd_DispText(buf, 90+dx, 302, MODE_NORMAL);
   }

}
char lcd_stationButton;
void lcd_ProcessButtons(void)
{  char button;
	button = lcd_GetTouch(1);
   if (isMyStation(transStation))
   {  		// ignore buttons when busy
   } else { // ok to process buttons
	   if (button > 0)
	   {  if (button == BTN_SEND)
	      {  lcd_stationButton=1;
	      }
	      else if (button == BTN_MENU)
	      {  // enter setup menu
	         lcd_enterSetupMode(1);
	         lcd_drawScreen(1, lcd_WITH_BUTTONS);      // redraw main screen
	         lcd_RefreshScreen(1);
	      }
	      else if (button == BTN_HELP)
	      {
	         lcd_helpScreen(0);
	         lcd_drawScreen(1, lcd_WITH_BUTTONS);      // redraw main screen
	         lcd_RefreshScreen(1);
	      }
      }
   }

}
char lcd_SendButton(void)
{  // returns the lcd_stationButton
	char rtnVal;
   #GLOBAL_INIT
   { lcd_stationButton=0; }

   rtnVal = lcd_stationButton;
   lcd_stationButton=0; // reset whenever it is accesses
	return rtnVal;
}
/******************************************************************/
// MAKING DESIGN ASSUMPTION --> MNU_xxx value is the same as the index in menu[]
// define menu item numbers
enum MenuItems {
	   MNU_COMM_STAT,
	   MNU_SETPASSWORD,
      MNU_CLEARPASSWORDS,
	   MNU_DEF_STATIONS,
      MNU_RESET,
      MNU_ADMIN_FEATURES,
      MNU_MAINT_FEATURES,
      MNU_SYSTEM_CONFIG,
      MNU_VIEW_LOGS,
      MNU_VIEW_TLOG, // Transaction log
      MNU_VIEW_SLOG, // Summary log
      MNU_VIEW_ELOG, // Event log
      MNU_VIEW_ALOG, // Alarm log
      MNU_CHECK_SFLASH,
      MNU_ANALYZE_LOG,
      MNU_LCD_CALIBRATE,
      MNU_SHOW_IPADDR,
      MNU_SHOW_INPUTS,
      MNU_SET_STATION,
      MNU_RARRIVAL_BEEP,
      MNU_LCD_ORIENTATION,
	   MNU_LAST
};

// Define bitmasks for activation of menu items
#define MNU_ADV  0x0E
#define MNU_STD  0x01
#define MNU_ALRM 0x10
#define MNU_NONE 0x00
// Define parameter types
#define MNU_FLAG 01
#define MNU_VAL  02
#define MNU_TIME 03
#define MNU_CLK  04
#define MNU_OTHR 05
// define parameter menu
char NOch;       // dummy/temporary character place holder
const struct{
   char item;   // menu item
   //char usage;  // when available to the user
   char *msg1;  // pointer to message
   char partype;  // parameter type (flag, clock, integer value, other)
   char *parval;  // parameter value
   unsigned long min;  // parameter minimum
   unsigned long max;  // parameter maximum (or don't save flag changes [=0])
   char units[4];  // parameter units display
   } menu[] = {
 MNU_COMM_STAT,      "SHOW COMMUNICATIONS",MNU_OTHR, &NOch, 0, 0, "",
 MNU_SETPASSWORD,    "SET PASSWORD",       MNU_OTHR, &NOch, 0, 0, "",
 MNU_CLEARPASSWORDS, "CLEAR PASSWORDS",    MNU_OTHR, &NOch, 0, 0, "",
 MNU_DEF_STATIONS,   "SET ACTIVE STATIONS",MNU_OTHR, &NOch, 0, 0, "",
 MNU_RESET,          "RESTART SYSTEM",     MNU_OTHR, &NOch, 0, 0, "",
 MNU_ADMIN_FEATURES, "ADMIN FEATURES",     MNU_OTHR, &NOch, 0, 0, "",
 MNU_MAINT_FEATURES, "MAINT FEATURES",     MNU_OTHR, &NOch, 0, 0, "",
 MNU_SYSTEM_CONFIG,  "SYSTEM CONFIG",      MNU_OTHR, &NOch, 0, 0, "",
 MNU_VIEW_LOGS,      "VIEW LOGS",          MNU_OTHR, &NOch, 0, 0, "",
 MNU_VIEW_TLOG,      "VIEW TRANSACTIONS",  MNU_OTHR, &NOch, 0, 0, "",
 MNU_VIEW_SLOG,      "VIEW SUMMARY LOG",   MNU_OTHR, &NOch, 0, 0, "",
 MNU_VIEW_ELOG,      "VIEW EVENT LOG",     MNU_OTHR, &NOch, 0, 0, "",
 MNU_VIEW_ALOG,      "VIEW RECENT ALARMS", MNU_OTHR, &NOch, 0, 0, "",
 MNU_CHECK_SFLASH,   "CHECK SERIAL FLASH", MNU_OTHR, &NOch, 0, 0, "",
 MNU_ANALYZE_LOG,    "ANALYZE EVENT LOG",  MNU_OTHR, &NOch, 0, 0, "",
 MNU_LCD_CALIBRATE,  "CALIBRATE SCREEN",   MNU_OTHR, &NOch, 0, 0, "",
 MNU_SHOW_IPADDR,    "SHOW IP ADDRESS",    MNU_OTHR, &NOch, 0, 0, "",
 MNU_SHOW_INPUTS,    "SHOW INPUTS",        MNU_OTHR, &NOch, 0, 0, "",
 MNU_SET_STATION,    "SET STATION NUMBER", MNU_VAL,  (char *)&param.stationNum, 1, 7, "",
 MNU_RARRIVAL_BEEP,  "SET AUDIBLE ALERT",  MNU_FLAG, &param.remoteAudibleAlert, 0, 1, "",
 MNU_LCD_ORIENTATION,"ORIENT LANDSCAPE",   MNU_FLAG, &param.orientLandscape, 0, 1, "",
 MNU_LAST,           "END",                MNU_NONE, &NOch, 0, 0, "" };

char menuOrder[25];
void setupMenuOrder(char menuLevel)
{
	char i;
   i=0;
	// content of menu depends on if passwords are enabled or not
   switch (menuLevel)
   {
   case 0: // ALARM
   	break;
   case 1: // Basic User
      menuOrder[i++]=MNU_VIEW_LOGS;
      //menuOrder[i++]=MNU_MAINT_FEATURES;
		//menuOrder[i++]=MNU_ADMIN_FEATURES; // do allow admin for slave
      menuOrder[i++]=MNU_SYSTEM_CONFIG;
      menuOrder[i++]=MNU_RARRIVAL_BEEP;
   	break;
   case 2: // Administrator
   	break;
   case 3: // Maintenance
   	break;
   case 4: // System configuration (Colombo only)
      menuOrder[i++]=MNU_SET_STATION;
      //menuOrder[i++]=MNU_DEF_STATIONS;
      //menuOrder[i++]=MNU_SHOW_IPADDR;
      menuOrder[i++]=MNU_SHOW_INPUTS;
      menuOrder[i++]=MNU_SETPASSWORD;
      menuOrder[i++]=MNU_CLEARPASSWORDS;
      menuOrder[i++]=MNU_LCD_ORIENTATION;
      //menuOrder[i++]=MNU_CHECK_SFLASH;
      //menuOrder[i++]=MNU_ANALYZE_LOG;
      menuOrder[i++]=MNU_LCD_CALIBRATE;
      menuOrder[i++]=MNU_RESET;
   	break;
   case 5: // view logs
      menuOrder[i++]=MNU_VIEW_SLOG; // transaction summary
      menuOrder[i++]=MNU_VIEW_TLOG; // transaction log
      //menuOrder[i++]=MNU_VIEW_ELOG; // event log
      menuOrder[i++]=MNU_VIEW_ALOG; // alarm log
   	break;
   case 6: // secure card setup
      //menuOrder[i++]=MNU_SECURE_ENABLE;
		//menuOrder[i++]=MNU_CARDID_ENABLE;
      //menuOrder[i++]=MNU_SERVER_ADDRESS;
      //menuOrder[i++]=MNU_SYNC_INTERVAL;
      //menuOrder[i++]=MNU_RESYNC_USERS;
      //menuOrder[i++]=MNU_CARD_FORMAT;
      //menuOrder[i++]=MNU_SECURE_R9MAX;
      //menuOrder[i++]=MNU_SECURE2_L3DIGITS;
      //menuOrder[i++]=MNU_SECURE2_R9MIN;
      //menuOrder[i++]=MNU_SECURE2_R9MAX;
   	break;
	}
   menuOrder[i]=MNU_LAST;

}
//#define PAGE_SIZE 7
const char * const title[7] = { "ALARM MENU", "MENU", "ADMIN MENU", "MAINTENANCE MENU", "CONFIG MENU", "LOGS MENU","SECURE TRANS MENU" };
char lcd_ShowMenu(char menuLevel, char page)
{  // returns if a next page is available
	char i, j;
   char rtnval;
   int row, count, button;
   char ttl;
   char btnX;       // menu item x position
   char PAGE_SIZE;  // 5 for landscape, 7 for portrait
   if (param.orientLandscape)
   {  PAGE_SIZE=5;
   	btnX=40;
   } else
   {  PAGE_SIZE=7;
   	btnX=0;
   }
   rtnval=TRUE;

   setupMenuOrder(menuLevel);
   ttl = (menuLevel<7) ? menuLevel : 0;  // which title to show
	lcd_drawScreen(2,title[ttl]);         // show the screen & title

   // make sure we can handle the page size
   if (page >= 1)
   { 	i = page * PAGE_SIZE;
   	// count backwards and reduce i if needed
   	for (j=i; j>0; j--) if (menuOrder[j]==MNU_LAST) i=j-1;
	} else i=0;

   count=0;
   lcd_Font("16B");
   while ((menuOrder[i] != MNU_LAST) && (count < PAGE_SIZE))
   {  row = 47 + count*30;
   	// put up text
	   //lcd_DispText(sys_message[menu[menuOrder[i]].msg1].msg, row, 50, MODE_TRANS);
      // put up button
	   lcd_ButtonDef( BTN_MNU_FIRST+menuOrder[i],  // menu items at 21+
	      BTN_MOM,                      // momentary operation
	      BTN_TLXY, btnX, row,             // starting x,y for buttons
		   BTN_TYPE, BUTTON_RELEASE,
         BTN_MARGINS, 5, 315,
	      BTN_TEXT, menu[menuOrder[i]].msg1,
	      BTN_TXTOFFSET, 10, 6,
	      BTN_BMP, BMP_long_button, BMP_long_button_dn,
	      BTN_END );
      i++;
      count++;
   }
   // show page # at the bottom
	j=0; while (menuOrder[j] != MNU_LAST) j++;
   count = ((j-1) / PAGE_SIZE) + 1;
	lcd_showPage(page+1, count);

   // is there another page to show?
   //if ( (menuOrder[i] == MNU_LAST) || (menuOrder[i+1] == MNU_LAST) ) rtnval=FALSE;
   if ( (menuOrder[i] == MNU_LAST) ) rtnval=FALSE;
   return rtnval;

}
char lcd_ProcessMenuItem(char menu_idx, char *menu_level)
{  // can modify menu_level using advanced configuration menu item
   // returns true if a flash parameter changed
   unsigned long clock_in_seconds;
   char work;
   char maxDigits;
   unsigned long temp_ulong;
   static char parChanged;
   parChanged=FALSE;  // assume not

   switch (menu[menu_idx].partype)
   {
   case MNU_VAL:
   	//lcd_drawScreen(4);sys_message[menu[menuOrder[i]].msg1].msg
      if (menu[menu_idx].max > 999)
      {  // unsigned long data type
	      maxDigits = 9;  // 9 digits max
	      temp_ulong = *(unsigned long*)menu[menu_idx].parval; // type is ulong
      }
      else
      {  // character data type
	      maxDigits = 3;  // only 3 digits max
	      temp_ulong = *menu[menu_idx].parval;  // type is character
      }
      if (lcd_GetNumberEntry(menu[menu_idx].msg1, &temp_ulong,
             menu[menu_idx].min, menu[menu_idx].max, menu[menu_idx].units, maxDigits, 0))
      {
      	if (menu[menu_idx].max > 999)  // cast unsigned long
         	*(unsigned long*)menu[menu_idx].parval = temp_ulong;
			else                           // cast character
         	*menu[menu_idx].parval = (char)temp_ulong;
         parChanged=TRUE;
      }
		break;
   case MNU_FLAG:
		parChanged=lcd_SelectChoice(menu[menu_idx].parval, menu[menu_idx].msg1, "YES", "NO");
      // Some flags don't require parameters to be saved (when max=0);
      if (menu[menu_idx].max == 0) parChanged=0; // Don't save parameter change to flash
      //if (menu[menu_idx].item == MNU_ALARM_SOUND) alarm(ON); // update alarm if needed
      break;
   case MNU_OTHR:
      switch (menu[menu_idx].item)
      {
      case MNU_ADMIN_FEATURES:
      	// get pin for next menu level
         if (lcd_enableAdvancedFeatures(2, menu[menu_idx].msg1)) *menu_level=2;
         break;
      case MNU_MAINT_FEATURES:
      	// get pin for next menu level
         if (lcd_enableAdvancedFeatures(3, menu[menu_idx].msg1)) *menu_level=3;
         break;
      case MNU_SYSTEM_CONFIG:
      	// get pin for next menu level
         if (lcd_enableAdvancedFeatures(4, menu[menu_idx].msg1)) *menu_level=4;
         break;
      case MNU_VIEW_LOGS:
         *menu_level=5;  // Logs menu items
         break;
      case MNU_DEF_STATIONS:        // define active stations
         //parChanged=lcd_defineActiveStations();
         break;
      case MNU_SETPASSWORD:        // set new password
         // get the PIN for the given menu level
         //if (*menu_level==2) parChanged |= lcd_getPin(param.adminPassword, "SET ADMIN PASSWORD");
         //else if (*menu_level==3) parChanged |= lcd_getPin(param.maintPassword, "SET MAINT PASSWORD");
         //else if (*menu_level==4) parChanged |= lcd_getPin(param.cfgPassword, "SET CONFIG PASSWORD");
         break;
      case MNU_CLEARPASSWORDS:    // reset passwords
      	///param.adminPassword[0]=0;
         ///param.maintPassword[0]=0;
         parChanged=TRUE;
         break;
      case MNU_COMM_STAT:
         //show_comm_test(1); // wait for button press
         break;
      case MNU_VIEW_SLOG:
	      lcd_showTransSummary();
         break;
      case MNU_VIEW_TLOG:
         lcd_showTransactions(0);
      	break;
      case MNU_VIEW_ALOG:
         lcd_showTransactions(1);
      	break;
      case MNU_SHOW_IPADDR:
      	//show_ip_address();
      	break;
      case MNU_SHOW_INPUTS:
      	lcd_show_inputs();
         break;
      case MNU_RESET:
         // go into long loop to allow watchdog reset
         clock_in_seconds = SEC_TIMER;
         lcd_drawScreen(3, menu[menu_idx].msg1);
	      lcd_Font("24");
	      lcd_DispText("SYSTEM RESTARTING", 5, 85, MODE_NORMAL);
         #asm
            ipset 3          ; disable interrupts so that the periodic ISR doesn't hit the watchdog.
            ld a,0x53        ; set the WD timeout period to 250 ms
            ioi ld (WDTCR),a
         #endasm
         while (SEC_TIMER - clock_in_seconds < 5);
	      lcd_DispText("UNABLE TO RESET", 50, 115, MODE_NORMAL);
         msDelay(1000);
         break;
      }
	}
   return parChanged;
}
/******************************************************************/
char lcd_GetNumberEntry(char *description, unsigned long * par_value,
         unsigned long par_min, unsigned long par_max, char * par_units, char max_digits, char nc_as_ok)
{  // Allows numeric keypad entry using the touch screen
   // return value is 0 = *no change or cancel or timeout
   //                 1 = valid number entered and updated to par_value
   //                 * no change returns 1 if nc_as_ok is true

	int button;
   char keepLooping;
   unsigned long inval;
   char number[11];  // for display of numeric text
   char numDigits;
   char rtnval;
   int xref, yref;
   char i;
   inval=0;
   keepLooping=TRUE;
   numDigits=0;
   rtnval=0;  // assume no good

   // Initialize 60 second menu timeout
   menu_timeout = SEC_TIMER + 60;

   // draw screen with keypad
   lcd_drawScreen(4, "ENTER VALUE");

   // print title and place to print the number
   xref=80; // x pos of input box
   yref=76;
   lcd_Font("18BC");
   lcd_DispText(description, lcd_Center(description, 2)-20, 50, MODE_NORMAL); // hack position with -10
	lcd_DispBitmap( BMP_input_box, xref, yref );
   lcd_DispText(par_units, xref+67, yref+8, MODE_NORMAL);

   // show actual, min and max
   lcd_Font("16B");
   lcd_DispText("Min = ", 20, 110, MODE_NORMAL);
   lcd_DispText(ltoa((long)par_min, number), 0, 0, MODE_NORMAL);
   lcd_DispText("; Current = ", 0, 0, MODE_NORMAL);
   lcd_DispText(ltoa((long)*par_value, number), 0, 0, MODE_NORMAL);
   lcd_DispText("; Max = ", 0, 0, MODE_NORMAL);
   lcd_DispText(ltoa((long)par_max, number), 0, 0, MODE_NORMAL);
   // Setup for first character to display
	number[0]=0; number[1]=0;

   while ( (keepLooping) && !secTimeout(menu_timeout) )
   {
      maintenance();  // watchdog, led activity, UDP commands
      button = lcd_GetTouch(100);
      if (button != -1)
      {  // valid button pressed
      	menu_timeout = SEC_TIMER + 60;
	      if ((button >= '0') && (button <= '9') && (numDigits < max_digits))
         {  // add the digit to the string
         	number[numDigits]=button;
         	numDigits++;
         	number[numDigits]=0;  // trailing null
				lcd_DispText(number, xref+5, yref+7, MODE_TRANS); // show the number
	      } else if ((button == BTN_DEL) && (numDigits > 0))
         {  numDigits--;
         	number[numDigits]=0;  // trailing null
            // reprint the screen
	         lcd_DispBitmap( BMP_input_box, xref, 80 );
	         lcd_DispText(number, xref+5, yref+7, MODE_TRANS); // show the number
	      } else if (button == BTN_CANCEL)
	      {  keepLooping=FALSE;
	      } else if (button == BTN_OK)
	      {  // was anything entered?
         	if (numDigits==0)
            { // no entry
            	if (nc_as_ok)
               {  // return nc as ok
                  rtnval=1;
               }
	            else
               {  lcd_DispText("NO CHANGE MADE", 65, 265, MODE_REV);
                  msDelay(750);
               }
               keepLooping=FALSE;
            } else
            {
	            inval = (unsigned long)atol(number);
	            if ((inval >= par_min) && (inval <= par_max))
	            {  keepLooping=FALSE;
	               *par_value = inval;
	               rtnval=1;
	            } else
	            { lcd_DispText("OUTSIDE RANGE", 65, 265, MODE_REV);
	              msDelay(750);
	              numDigits=0;
	              number[0]=0; number[1]=0;
	              lcd_DispBitmap( BMP_input_box, xref, yref );
	              lcd_DispText(number, xref+5, yref+7, MODE_TRANS); // show the number
               }
            }
         }
      }
   }
   return rtnval;
}
void lcd_enterSetupMode(char operatorLevel)
{
	int button;
   char mych;
   char keepLooping;
   char menu_idx;
   char page;
   char anotherPage;
   char showPage;
   char newLevel;
   char changed;
   keepLooping=TRUE;
   page=0;
   changed=FALSE; // None yet

   // clear the button input buffer
   lcd_GetTouch(1);

	anotherPage = lcd_ShowMenu(operatorLevel, page);
   // Initialize 60 second menu timeout
   menu_timeout = SEC_TIMER + 60;

   while ( (keepLooping) && !secTimeout(menu_timeout) )
   {
      maintenance();  // watchdog, led activity, UDP commands
      // inKey = getKey();
      showPage=FALSE;
      button = lcd_GetTouch(100);
      if (button != -1)
      {  // valid button pressed
      	menu_timeout = SEC_TIMER + 60;
	      if ((button >= BTN_MNU_FIRST) && (button <= BTN_MNU_FIRST+MNU_LAST))
	      {  menu_idx = button-BTN_MNU_FIRST;
         	newLevel = operatorLevel;
	         changed |= lcd_ProcessMenuItem(menu_idx, &newLevel);
            // Did we advance to a new menu
            if (newLevel != operatorLevel)
            { operatorLevel = newLevel;
              page=0;
            }
            // continue to show menu?
            if (operatorLevel==99) keepLooping=FALSE;
            // update display of menu items?
            if (!secTimeout(menu_timeout)) showPage=TRUE;
	      }
	      else if (button == BTN_PREV)
	      {  if (page>0) { page--; showPage=TRUE;}
	      }
	      else if (button == BTN_NEXT)
	      {  if (anotherPage) { page++; showPage=TRUE;}
	      }
	      //else if (button == BTN_SAVE)
	      else if (button == BTN_CANCEL)  // DONE
         {
         	if (changed)
            {  // save changes if needed
	            lcd_BFcolorsB( lcd_BLACK, lcd_WHITE);
	            lcd_ClearScreen ();
	            lcd_Origin( 0, 0 );
	            lcd_Font("24");
	            lcd_DispText("SAVING CHANGES ...", 10, 50, MODE_NORMAL);
	            if (writeParameterSettings() == 0)
	               lcd_DispText("\nCHANGES SAVED", 0, 0, MODE_NORMAL);
	            else
	               lcd_DispText("\nERROR SAVING CHANGES", 0, 0, MODE_REV);
	            msDelay(800);
               //sendParametersToLogger(); // send parameters to data logger
            }
         	keepLooping = FALSE;
         }
	      //else if (button == BTN_CANCEL) keepLooping = FALSE;

         // show the same or next menu page if needed
         if (showPage && keepLooping) anotherPage = lcd_ShowMenu(operatorLevel, page);
      }

	}

}
nodebug char lcd_enableAdvancedFeatures(char menu_level, char * description)
{  // get password for the requested level
	char reqdPW[5];
   char altPW[5];  // for non-settable cfg (Colombo) PW
   char enteredPW[5];
   char rtnVal;

   strcpy(altPW, "xxxx");  // start with pin that can't be entered

   // determine which password to compare with
   switch (menu_level)
   {
//   case 2: 	strcpy(reqdPW, param.adminPassword); break;
//   case 3: 	strcpy(reqdPW, param.maintPassword); break;
//   case 4: 	strcpy(reqdPW, param.cfgPassword);
   default: reqdPW[0]=0;
   }
	strcpy(altPW, "2321");  // 2321 is always available

	rtnVal=FALSE;  // assume no good
   if (strlen(reqdPW)==0) rtnVal=TRUE;
   else
   {  // get the password/pin
   	if (lcd_getPin(enteredPW, description))
      {  // allow entry of settable or alternate/fixed pin
      	if ( (strcmp(enteredPW, reqdPW)==0) || (strcmp(enteredPW, altPW)==0) ) rtnVal=TRUE;
         else
         {  // sorry charlie
				lcd_BFcolorsB( lcd_WHITE, lcd_RED);
			   lcd_Font("24");
			   lcd_DispText(" INCORRECT PIN ", 17, 160, MODE_NORMAL);
            msDelay(2000);
         }
      }
   }
   return rtnVal;
}
nodebug char lcd_getPin(char *PIN, char *description)
{  // get a 4 digit pin from the touch screen
	int button;
   char keepLooping;
   char number[5];  // for storing entered digits
   char asteric[5]; // for display on screen
   char numDigits, maxDigits;
   char rtnval;
   char i;
   keepLooping=TRUE;
   numDigits=0;
   maxDigits=4;
   rtnval=0;  // assume no good

   // Initialize 60 second menu timeout
   menu_timeout = SEC_TIMER + 60;

   // draw screen with keypad
   lcd_drawScreen(4, description);

   // print title and place to print the number
   lcd_Font("18BC");
   lcd_DispText("ENTER 4 DIGIT PIN", 20, 50, MODE_NORMAL);
	lcd_DispBitmap( BMP_input_box, 45, 80 );

   // show actual, min and max
   lcd_Font("32B");
   // Setup for first character to display
	number[0]=0; number[1]=0;
   asteric[0]=0; asteric[1]=0;

   while ( (keepLooping) && !secTimeout(menu_timeout) )
   {
      maintenance();  // watchdog, led activity, UDP commands
      button = lcd_GetTouch(100);
      if (button != -1)
      {  // valid button pressed
      	menu_timeout = SEC_TIMER + 60;
	      if ((button >= '0') && (button <= '9') && (numDigits < maxDigits))
         {  // add the digit to the string
         	number[numDigits]=button;
            asteric[numDigits]='*';
         	numDigits++;
         	number[numDigits]=0;  // trailing null
            asteric[numDigits]=0;
				lcd_DispText(asteric, 50, 87, MODE_TRANS); // show the number
	      } else if ((button == BTN_DEL) && (numDigits > 0))
         {  numDigits--;
         	number[numDigits]=0;  // trailing null
            asteric[numDigits]=0;
            // reprint the screen
	         lcd_DispBitmap( BMP_input_box, 45, 80 );
	         lcd_DispText(asteric, 50, 87, MODE_TRANS); // show the number
	      } else if (button == BTN_CANCEL)
	      {  keepLooping=FALSE;
	      } else if (button == BTN_OK)
	      {  strcpy(PIN, number);
         	keepLooping=FALSE;
            rtnval=1;
         }
      }
   }
   return rtnval;

}
void lcd_show_inputs()
{
   // shows digital inputs on-screen for diagnosis
   char done;
   char i, j, col, msgbuf[25];
   char inKey;
   int button;

	lcd_drawScreen(5, "SHOW INPUTS");

   //strcpy(msgbuf, sys_message[MSG_BLANK].msg);  // buffer to show stations

   done=FALSE;
   // Loop until done or timeout
	while ((done==FALSE) && !secTimeout(menu_timeout))
   {
		maintenance();  // watchdog, led activity, UDP commands
	   menu_timeout = SEC_TIMER + 60; // never timeout like this
	   button = lcd_GetTouch(100);

	   // If BTN_SAVE, save changes
	   if (button == BTN_EXIT)  // Exit button
	   {
         done=TRUE;
      }
	   lcd_DispText("Arrival: ", 20, 50, MODE_NORMAL);
      lcd_DispText(di_carrierArrival ? "ON  " : "OFF", 0, 0, MODE_NORMAL);
	   lcd_DispText("\nIn Chamber: ", 0, 0, MODE_NORMAL);
      lcd_DispText(di_carrierInChamber ? "ON  " : "OFF", 0, 0, MODE_NORMAL);
	   lcd_DispText("\nDoor: ", 0, 0, MODE_NORMAL);
      lcd_DispText(di_doorClosed ? "ON  " : "OFF", 0, 0, MODE_NORMAL);
	   lcd_DispText("\nSend Button: ", 0, 0, MODE_NORMAL);
	   sprintf(msgbuf, "%X  ", di_requestToSend);
      lcd_DispText(msgbuf, 0, 0, MODE_NORMAL);
      lcd_DispText("\nDiverter Pos: ", 0, 0, MODE_NORMAL);
	   //sprintf(msgbuf, "%X  ", di_diverterPos);
      //lcd_DispText(msgbuf, 0, 0, MODE_NORMAL);
	   lcd_DispText("\nRaw Inputs: ", 0, 0, MODE_NORMAL);
      strcpy(msgbuf,""); // blank out
      j=readDigBank(0);
		for (i=0; i<8; i++) strcat(msgbuf, (j & 1<<i) ? "1" : "0");
      strcat(msgbuf, " ");
      j=readDigBank(1);
		for (i=0; i<8; i++) strcat(msgbuf, (j & 1<<i) ? "1" : "0");
      lcd_DispText(msgbuf, 0, 0, MODE_NORMAL);
	   lcd_DispText("\nIP address: ",0,0,MODE_NORMAL);
	   lcd_DispText(myIPaddress,0,0,MODE_NORMAL);
   }

}
void lcd_ShowKeypad(char opts)
{  // display 0..9 in a keypad arrangement on the screen
   // opts = 0 = no '-' button
   //        1 = use '-' button
	char i;
   char btn[2];
   int x1, x2, y1;

   i=1; // first digit 1
   btn[0]=49;
   btn[1]=0;

   if (param.orientLandscape)
   {  x1=200;
      x2=300;
      y1=55;
   } else
   {  x1=120;
      x2=190;
      y1=120;
   }
   // draw first button
   lcd_Font("16B");
   lcd_ButtonDef( '0'+i,
      BTN_MOM,                      // momentary operation
      BTN_TLXY, x1, y1,               // starting x,y for buttons
      BTN_TYPE, BUTTON_RELEASE,
      BTN_MARGINS, x1, x2,        // set left and right margins
      BTN_SEP, 35, 35,              // set up for button sep
      BTN_TEXT, btn,
      BTN_TXTOFFSET, 11, 9,
      BTN_BMP, BMP_button_up, BMP_button_dn,
      BTN_END );
   // draw 2-9
   for (i=2; i<10; i++)
   {  btn[0]++;
	   lcd_ButtonDef( '0'+i,
	      BTN_TEXT, btn,
	      BTN_END );
	}
   // draw last buttons
   if (opts==1) // use '-' button
   {  i='-';
      btn[0]='-';
	   lcd_ButtonDef( i,
	      BTN_TEXT, btn,
	      BTN_END );
   }
   // draw '0'
   i=0;
   btn[0]=48;
   lcd_ButtonDef( '0'+i,
      BTN_TEXT, btn,
      BTN_END );
   // draw 'Del'
   lcd_ButtonDef( BTN_DEL,
      BTN_TXTOFFSET, 5, 9,
   	BTN_TEXT, "Del",
      BTN_END );

}
void lcd_showPage(long thisPage, long lastPage)
{  // show page x of y
   char buf[25];
return;
   sprintf(buf, "Page %ld of %ld  ", thisPage, lastPage);
   lcd_Font("6x9");
   lcd_BFcolorsB( lcd_BLACK, lcd_WHITE);
   lcd_DispText(buf, 145, 218, MODE_NORMAL);
}
int lcd_SelectChoice(char *choice, char *label, char *optZero, char *optOne)
{
	// Show label and two choices and allow selection
   // choice is modified as TRUE (optZero) or FALSE (optOne) based on selection
   // function returns 0 if cancel or 1 if a choice is made

   int button;
   int x1;

   x1 = param.orientLandscape ? 150 : 110;
	lcd_drawScreen(3, "MENU");
   lcd_Font("24");
   lcd_DispText(label, lcd_Center(label, 1), 50, MODE_NORMAL);
   lcd_DispText(optZero, x1, 85, MODE_NORMAL);
   lcd_DispText(optOne, x1, 115, MODE_NORMAL);

   // show optZero choice
   x1 = x1 - 40;
   lcd_ButtonDef( BTN_YES_ON,
   BTN_LAT,                      // latching operation
   BTN_TLXY, x1, 80,             // starting x,y for buttons
   BTN_TYPE, (*choice) ? BUTTON_LAT_1 : BUTTON_LAT_0,
   BTN_BMP, BMP_check_box, BMP_check_box_click,
   BTN_END );
   // show optOne choice
   lcd_ButtonDef( BTN_NO_OFF,
   BTN_TLXY, x1, 110,             // starting x,y for buttons
   BTN_TYPE, (*choice) ? BUTTON_LAT_0 : BUTTON_LAT_1,
   BTN_BMP, BMP_check_box, BMP_check_box_click,
   BTN_END );
   // show Cancel button
   lcd_Font("16B");
   lcd_ButtonDef( BTN_CANCEL,
      BTN_MOM,                      // momentary operation
      BTN_TLXY, lcd_max_x - 63, lcd_max_y - 35,             // starting x,y for buttons
      BTN_TYPE, BUTTON_RELEASE,
      BTN_TEXT, "Cancel",
      BTN_TXTOFFSET, 5, 9,
      BTN_BMP, BMP_med_button, BMP_med_button_dn,
      BTN_END );

   // wait for button press & release
   menu_timeout = SEC_TIMER + 60;
	while( ((button = lcd_GetTouch(100)) == -1) && !secTimeout(menu_timeout) )
   	maintenance();

   // handle selection
   if (button == 256+BTN_NO_OFF)
   {  // update screen
      *choice=FALSE;
   }
   else if (button == 256+BTN_YES_ON)
   {  // update screen
      *choice=TRUE;
   }
	if ((button == BTN_CANCEL) || (button == -1)) return 0; // cancel or timeout
   else
   {  // update scren to show choice
	   lcd_ButtonDef( BTN_YES_ON,
	   BTN_LAT,                      // latching operation
	   BTN_TLXY, x1, 80,             // starting x,y for buttons
	   BTN_TYPE, *choice ? BUTTON_LAT_1 : BUTTON_LAT_0,
	   BTN_BMP, BMP_check_box, BMP_check_box_click,
	   BTN_END );
	   // show optZero choice
	   lcd_ButtonDef( BTN_NO_OFF,
	   BTN_TLXY, x1, 110,             // starting x,y for buttons
	   BTN_TYPE, *choice ? BUTTON_LAT_0 : BUTTON_LAT_1,
	   BTN_BMP, BMP_check_box, BMP_check_box_click,
	   BTN_END );
	   msDelay(500); // wait so you can see the choice
   	return 1;  // got a choice
   }
}
void lcd_helpScreen(char view)
{  // view 0 - main screen help
	//      1 - log screen help
	int button;

   // Logo splash screen
	lcd_BFcolorsB( lcd_BLACK, lcd_WHITE);
	lcd_ClearScreen ();
   if (param.orientLandscape) lcd_Font("10"); else lcd_Font("13");
// lcd_DispText(param.station_name[SYSTEM_NAME].name, 10, 5, MODE_NORMAL);
	lcd_DispText("Remote Station - ", 10, 5, MODE_NORMAL);
   lcd_DispText(FIRMWARE_VERSION, 0, 0, MODE_NORMAL);

   if (view==0)
   {  // main screen help
	   lcd_DispText("\nTo send a carrier:", 0, 0, MODE_NORMAL);
	   lcd_DispText("\nInsert carrier into tube and close the door", 0, 0, MODE_NORMAL);
	   lcd_DispText("\nTouch the send button send it", 0, 0, MODE_NORMAL);
	   lcd_DispText("\n", 0, 0, MODE_NORMAL);
	   lcd_DispText("\nTouch the Menu button for system features", 0, 0, MODE_NORMAL);
	   lcd_DispText("\n", 0, 0, MODE_NORMAL);
	   lcd_DispText("\nHelp Phone Numbers", 0, 0, MODE_NORMAL);
	   lcd_DispText("\nSystem Administrator: ", 0, 0, MODE_NORMAL);
	//   lcd_DispText(param.phoneNum[ADMIN_PHONE], 0, 0, MODE_NORMAL);
	   lcd_DispText("\nMaintenance: ", 0, 0, MODE_NORMAL);
	//   lcd_DispText(param.phoneNum[MAINT_PHONE], 0, 0, MODE_NORMAL);
	   lcd_DispText("\nColombo Pneumatic Tube Systems:\n800-547-2820", 0, 0, MODE_NORMAL);
	} else if (view==1)
   {  // log screen help
	   lcd_DispText("\nDATE TIME of event start", 0, 0, MODE_NORMAL);
	   lcd_DispText("\nDUR is duration in seconds", 0, 0, MODE_NORMAL);
	   lcd_DispText("\nDIR is direction", 0, 0, MODE_NORMAL);
	   lcd_DispText("\nSTATUS codes", 0, 0, MODE_NORMAL);
	   lcd_DispText("\n 0 = normal status", 0, 0, MODE_NORMAL);
	   lcd_DispText("\n 1 = diverter timeout", 0, 0, MODE_NORMAL);
	   lcd_DispText("\n 2 = carrier exit (lift) timeout", 0, 0, MODE_NORMAL);
	   lcd_DispText("\n 3 = delivery overdue timeout", 0, 0, MODE_NORMAL);
	   lcd_DispText("\n 4 = blower timeout", 0, 0, MODE_NORMAL);
	   lcd_DispText("\n 5 = station not ready or cancel dispatch", 0, 0, MODE_NORMAL);
	   lcd_DispText("\n 6 = transaction cancelled/aborted", 0, 0, MODE_NORMAL);
	   lcd_DispText("\n 64 = Door opened event", 0, 0, MODE_NORMAL);
	   lcd_DispText("\nFLAGS (hexidecimal)", 0, 0, MODE_NORMAL);
	   lcd_DispText("\n xxxx xxx1 = Stat Transaction", 0, 0, MODE_NORMAL);
	   lcd_DispText("\n xxxx xx1x = Carrier return function", 0, 0, MODE_NORMAL);
	   lcd_DispText("\n xxxx x1xx = Auto return activated", 0, 0, MODE_NORMAL);
	   lcd_DispText("\n xxxx 1xxx = Door opened in transaction", 0, 0, MODE_NORMAL);
	   lcd_DispText("\n xxx1 xxxx = Auto return performed", 0, 0, MODE_NORMAL);
	   lcd_DispText("\n xx1x xxxx = Secure transaction", 0, 0, MODE_NORMAL);
   }
   // done button
	lcd_Font("16B");
   lcd_ButtonDef( BTN_CANCEL,
	   BTN_MOM,                      // momentary operation
      BTN_TYPE, BUTTON_RELEASE,
      BTN_MARGINS, 5, 330,         // set left and right margins
      BTN_SEP, 63, 43,              // set up for button sep
	   BTN_TXTOFFSET, 10, 9,
	   BTN_TLXY, lcd_max_x-69, lcd_max_y-40,             // starting x,y for buttons
	   BTN_TEXT, "Done",
	   BTN_BMP, BMP_med_button, BMP_med_button_dn,
	   BTN_END );

   // wait for a button press
   menu_timeout = SEC_TIMER + 60;
   while(((button=lcd_GetTouch(100)) == -1) && !secTimeout(menu_timeout) ) maintenance();

}
struct logInfoType
{  char spare[4]; 	     	// available open space for data
   unsigned long xSF_Head; 	// Not used
   unsigned long xSF_Tail;    // Not used
	unsigned long Head;  	// points to latest entry
	unsigned long Tail;  	// points to first entry  (if both equal 0 then log is empty)
	char Ver;
   char spare2;         	// available open space for data
} logInfo;
unsigned long logInfoXaddr;  // address to xmem for logInfo
unsigned long logXaddr;      // address to xmem for data log
int SFAvailable;				  // Indicates if serial flash is available
#define XMemLogMax 100
#define SFLogMax 500000
long MaxLogEntries;   		// maximum number of events in either log
#define LOG_VERSION   3
xmem void initTransLog(void)
{
	struct trans_log_type trans;
   int status;
   int sf;
	long memOffset;
   char sbuf[80];

	// allocate the xmem for header and log
   logInfoXaddr = xalloc(sizeof(logInfo));  // was 10, added 12 more (size of trans)
   logXaddr = xalloc(XMemLogMax * sizeof(trans));

	lcd_Font("16");
   // check for and init the serial flash (only in run mode)
   //// NO SF IN REMOTE: if (OPMODE == 0x80) sf = SF1000Init(); else sf = -3;
   	sf=-3; // FORCE UNUSED
   if (sf==0) // will use SF instead of XMEM
   {	SFAvailable=TRUE;
   	MaxLogEntries=SFLogMax;
      lcd_DispText("Using Serial Flash for event log",0,0,MODE_NORMAL);
   } else
   {  SFAvailable=FALSE;
   	MaxLogEntries=XMemLogMax;
      lcd_DispText("Using xmem for event log",0,0,MODE_NORMAL);
   }

   // get the header to determine head and tail
   status = xmem2root( &logInfo, logInfoXaddr, sizeof(logInfo));

   if (status != 0)
   {  // error reading xmem
   	sprintf(sbuf,"\nError %d initializing transaction log", status);
      lcd_DispText(sbuf,0,0,MODE_NORMAL);
   	//printf("\nError %d initializing transaction log", status);
   }
	// if log version does not match then initialize log
   if (logInfo.Ver != LOG_VERSION)
   {  // reinit log
   	// start over from scratch
      logInfo.Tail=0;
      logInfo.Head=0;
      lcd_DispText("\nTransaction log reset\n",0,0,MODE_NORMAL);
      //printf("\nTransaction log reset\n");

      // save the logInfo data
      logInfo.Ver = LOG_VERSION;
      root2xmem( logInfoXaddr, &logInfo, sizeof(logInfo));
	}
   // make sure head/tail are plausible
   // either tail=0 and head=0..MaxLogEntries-1
   // or tail=1..MaxLogEntries-1 and head=tail-1
   else if ((logInfo.Tail<=logInfo.Head) && (logInfo.Head<MaxLogEntries))
   { // OK
   	sprintf(sbuf, "\nTrans log contains %ld entries\n", logInfo.Head-logInfo.Tail);
      lcd_DispText(sbuf,0,0,MODE_NORMAL);
   }
   else if ((logInfo.Tail>logInfo.Head) && (logInfo.Tail<MaxLogEntries))
   { // OK
      sprintf(sbuf, "\nTrans log contains %ld entries\n", logInfo.Head + (MaxLogEntries-logInfo.Tail));
      lcd_DispText(sbuf,0,0,MODE_NORMAL);
   }
   else
   { // NOT OK so reinitialize head and tail
      sprintf(sbuf,"\nTrans log reset due to inconsistent tail: %ld and head: %ld\n", logInfo.Tail, logInfo.Head);
      lcd_DispText(sbuf,0,0,MODE_NORMAL);
   	logInfo.Tail=0;
      logInfo.Head=0;
      root2xmem( logInfoXaddr, &logInfo, sizeof(logInfo));
   }
}
void analyzeEventLog()
{  // Check consistency of entries in the log and reset tail/head if needed
	// Valid entries have a date/time between 850,000,000 (Dec-2006) and current date/time (+ 1 month)
   // In case of log wrap-around, the date/time of a previous valid entry is greater than the next valid entry
	unsigned long lastTransDateTime;
   unsigned long newTail;
   unsigned long newHead;
   unsigned long savedTail;
   struct trans_log_type trans;
   long entry;
   int button;
   int foundStart;
   char myline[80];     // line to send to the lcd
   lastTransDateTime=0;
   entry=0;
   newTail=0;  // assume for now it will be at the start
   newHead=0;
	savedTail=logInfo.Tail;

	lcd_drawScreen(5, "ANALYZE EVENT LOG");
   lcd_Font("8x12");
   lcd_DispText("Checking Log Consistency", 10, 50, MODE_NORMAL);
   sprintf(myline, "\nCurrent first entry %ld", logInfo.Tail);
	lcd_DispText(myline, 0, 0, MODE_NORMAL);
   sprintf(myline, "\nCurrent last entry %ld", logInfo.Head);
	lcd_DispText(myline, 0, 0, MODE_NORMAL);

   logInfo.Tail=0;  // have to set tail to zero to get absolute entry references
   foundStart=FALSE; // Not yet
   while (entry < MaxLogEntries)
   {  if (entry%100==0)
   	{  // update progress
      	sprintf(myline, "Checking Entry %ld", entry);
			lcd_DispText(myline, 10, 110, MODE_NORMAL);
         maintenance();
		}

   	getTransaction(entry, &trans);
   	if ((trans.start_tm > 850000000) && (trans.start_tm < SEC_TIMER+2592000))
      {  // good entry
      	foundStart=TRUE;
			// is it before or after last entry
         if (trans.start_tm < lastTransDateTime)
         {  // assume it is wrap around of the log
         	newHead=entry-1;
            newTail=entry;
         }
      	entry++;
      }
      else
      {  // invalid entry
         if (foundStart)
      	{  // already found a good entry so stop now
	         if (entry>0)newHead=entry;
	         entry=MaxLogEntries+1;
         }
         else
         {  // didn't find start yet so keep going
         	entry++;
            newTail=entry;  // Hope we find the start
         }
      }
   }
   logInfo.Tail=savedTail;  // reset saved tail
   // show last entry checked
   sprintf(myline, "Checking Entry %ld", MaxLogEntries);
   lcd_DispText(myline, 10, 110, MODE_NORMAL);

   // Show new head and tail and ask if it should be retained
   sprintf(myline, "\nDetected first entry %ld", newTail);
	lcd_DispText(myline, 0, 0, MODE_NORMAL);
   sprintf(myline, "\nDetected last entry %ld", newHead);
	lcd_DispText(myline, 0, 0, MODE_NORMAL);

   // if it changed then ask if it should be kept
   if ((logInfo.Tail != newTail) || (logInfo.Head != newHead))
	{  lcd_DispText("\nAccept Detected Entries?", 0, 0, MODE_NORMAL);
	   lcd_ButtonDef( BTN_CANCEL,
	      BTN_TXTOFFSET, 9, 9,
         BTN_TLXY, 5, 205,             // starting x,y for buttons
         BTN_TEXT, "Accept",
         BTN_END );
	}
   else {lcd_DispText("\nEntries are consistent", 0, 0, MODE_NORMAL);}

   // wait for any button
   menu_timeout = SEC_TIMER + 60;
   while(((button=lcd_GetTouch(100)) == -1) && !secTimeout(menu_timeout) ) maintenance();

   // Was Accept button pressed
   if (button==BTN_CANCEL)
   {	// keep the new entries
   	logInfo.Tail = newTail;
      logInfo.Head = newHead;

	   // save head/tail back to xmem
	   root2xmem( logInfoXaddr, &logInfo, sizeof(logInfo));
   }
}

// TEMPORARY REMOVE UNTIL CARD SCANNING IS NEEDED
/*
void addSecureTransaction(struct secure_log_type secTrans)
{  // map secure trans data into trans log and save
   // also works for standard card scans
	struct trans_log_type trans;
   char extra; // used for station #
   trans.trans_num = secTrans.trans_num;
   trans.start_tm = secTrans.start_tm;
   trans.duration = (secTrans.sid[1] << 8) + secTrans.sid[0];
   trans.source_sta = secTrans.sid[2];
   trans.dest_sta = secTrans.sid[3];
   trans.status = secTrans.status;
   trans.flags = secTrans.sid[4];
   extra = secTrans.flags;  // contains the station #

   if (secTrans.scanType == 2)
   {  trans.status = ESTS_CARD_SCAN;
   	trans.trans_num = transactionCount(); // maybe just temporary
		sendUDPtransaction(trans, extra);  // just send, don't log
   }
   else if (secTrans.scanType == 3)
   {  trans.status = ESTS_UNAUTH_CARD;
   	trans.trans_num = transactionCount(); // maybe just temporary
		sendUDPtransaction(trans, extra);  // just send, don't log
   }
   else addTransaction(trans, extra); // add to log and send


}
*/
void addTransaction(struct trans_log_type trans, char xtra)
{
	long memOffset;
   int sf;
   // Push the transaction into local xmem
   memOffset = logInfo.Head * sizeof(trans);

//   if (SFAvailable)
//   {  // write to serial flash
//	  	while ( (sf=SF1000Write ( memOffset, &trans, sizeof(trans) )) == -3 );
//   }
//   else
//   {  // write to xmem
   	root2xmem( logXaddr+memOffset, &trans, sizeof(trans));
      sf=0;
//   }
	if (sf==0)  // did we save ok?
	{
	   // update the log pointers
	   logInfo.Head++; // increment logHead
	   if (logInfo.Head==MaxLogEntries) logInfo.Head=0;  // check for wraparound
	   if (logInfo.Head==logInfo.Tail)                    // check for full log
	   {  logInfo.Tail++;                           // bump the tail
	      if (logInfo.Tail==MaxLogEntries) logInfo.Tail=0;   // check for wraparound
	   }
   }

   // save head/tail back to xmem
   root2xmem( logInfoXaddr, &logInfo, sizeof(logInfo));
printf("\nAdded transaction at %ld with status %d and flags %d\n", SEC_TIMER, (int)trans.status, (int)trans.flags);

   // now send out the UDP message
//   sendUDPtransaction( trans, xtra );

}
long sizeOfTransLog(void)
{  // how many entries in the log
   if ((logInfo.Tail<=logInfo.Head) && (logInfo.Head<MaxLogEntries))
   { // OK
   	return logInfo.Head-logInfo.Tail;
   }
   else if ((logInfo.Tail>logInfo.Head) && (logInfo.Tail<MaxLogEntries))
   { // OK, head before tail
      return logInfo.Head + (MaxLogEntries-logInfo.Tail);
   }
   else return 0;
}
int getTransaction(long entry, struct trans_log_type *trans)
{  // returns the n'th entry in the trans log (flash or xmem)
   // entry can be 0 .. sizeOfLog-1
   // return value = 0 if success or 1 if entry non-existant
   long memOffset;
   if (entry >= MaxLogEntries) return 1;
   memOffset = ((logInfo.Tail+entry) % MaxLogEntries) * sizeof(*trans);
//   if (SFAvailable) SF1000Read ( memOffset, trans, sizeof(*trans));
//   else xmem2root( trans, logXaddr+memOffset, sizeof(*trans));
	xmem2root( trans, logXaddr+memOffset, sizeof(*trans));
   if (entry >= sizeOfTransLog()) return 1;
   else return 0;
}
long findTransaction(unsigned long transNum)
{  // search the trans log for first trans equal to transNum
   // and return the position in the log as entry
   // if not found will return 0
   long entry;
   struct trans_log_type log;
   int found;

   entry = sizeOfTransLog();
   found = 0;
   while ((entry > 0) && (found == 0))
   {  // loop backwards to find transNum
      getTransaction(entry, &log);
      if (log.trans_num < transNum)
      {  // found one lower, so return entry+1
         found=1;
         entry++;
      }
      else
      {  // didn't find it yet, go back one more
      	entry--;
      }
   }
   // either way, return entry
	return entry;
}
//#define TRANS_PER_PAGE 14
int TRANS_PER_PAGE;
void lcd_showTransactions(char format)
{  // shows the transaction log on the screen
   // format = 1 = only show alarms
   long page;
   long lastPage;
   int button;
   int transPerPage;

   // count how many pages we can show
   page=0;
   TRANS_PER_PAGE = (param.orientLandscape) ? 10 : 14;
   lastPage = sizeOfTransLog() / TRANS_PER_PAGE;
   if (format==1) lastPage = 0;

   lcd_drawScreen(8, "TRANSACTION LOG");		// redraw main screen
	lcd_showTransPage(page, lastPage, format);

	button=0;
   while ((button != BTN_CANCEL) && !secTimeout(menu_timeout))
   {  maintenance();
   	button = lcd_GetTouch(100);
      if (button != -1)
      {  // valid button pressed
         menu_timeout = SEC_TIMER + 60;
         switch( button )
         {
         case BTN_PREV:
         	if (page > 0) page--; else page=lastPage;  // allow wraparound
            lcd_showTransPage(page, lastPage, format);
         	break;
         case BTN_NEXT:
            if (page < lastPage) page++; else page=0;  // allow wraparound
            lcd_showTransPage(page, lastPage, format);
         	break;
         case BTN_HELP:
	      	lcd_helpScreen(1);
			   lcd_drawScreen(8, "TRANSACTION LOG");		// redraw main screen
            lcd_showTransPage(page, lastPage, format);
         	break;
         }
      }
   }
}

void lcd_showTransPage(long page, long lastPage, char format)
{	// page 0 shows the last transaction
	int x;
	int y, dy;
   long entry;
   char j;
   char buf[80];
   char UID[UIDLen];
   char UIDstr[13];
   struct trans_log_type trans;
   struct tm time;

   //strcpy(buf, "mm/dd/yy hh:mm:ss sourcesta.. -> destinsta.. sss sec");
   entry = sizeOfTransLog() -1 - page * TRANS_PER_PAGE;
   j=0;
   y=44; dy=14; x=5;
   lcd_Font("6x9");
   // print header
   sprintf(buf, "DATE     TIME     DUR DIR STATUS FLAGS");
   lcd_DispText(buf, x, y, MODE_NORMAL);
   y+=dy;

   while ( (j < TRANS_PER_PAGE) && (entry >= 0) ) //< sizeOfTransLog()))
   {
		if (getTransaction(entry, &trans) == 0)
      {  mktm(&time, trans.start_tm);
      	lcd_BFcolorsB( lcd_BLACK, lcd_WHITE);
      	if (trans.status <= LAST_TRANS_EVENT)
         {  // This is a transaction
	         sprintf(buf, "%02d/%02d/%02d %02d:%02d:%02d %3d %-3s  %2x     %2x",
	            time.tm_mon, time.tm_mday, time.tm_year%100, time.tm_hour, time.tm_min, time.tm_sec,
//	            trans.source_sta < SYSTEM_NAME ? param.station_name[trans.source_sta].name : "??",
//	            trans.dest_sta < SYSTEM_NAME ? param.station_name[trans.dest_sta].name : "??",
	            trans.duration,
               trans.source_sta == DIR_SEND ? "RCV" : "SND",
	            (int) trans.status, (int) trans.flags );
            if (trans.status != 0)  			 lcd_BFcolorsB( lcd_RED, lcd_WHITE);  // alarm trans colors
	         else if (trans.flags & (FLAG_STAT | FLAG_SECURE)) lcd_BFcolorsD( 0xAA0, 0xFFF); //lcd_BFcolorsB( lcd_YELLOW, lcd_WHITE);  // stat trans colors
            else if (trans.flags & (FLAG_CRETURN | FLAG_ARETURN | FLAG_ARETURNING)) lcd_BFcolorsB( lcd_GREEN, lcd_WHITE);  // c.return trans colors
	         //else     lcd_BFcolorsB( lcd_BLACK, lcd_WHITE);
         } else if (trans.status == ESTS_DOOROPEN)
         {  // This is a door open event
         	sprintf(buf, "         %02d:%02d:%02d %-25s %3d %2x%2x",
            time.tm_hour, time.tm_min, time.tm_sec,
            "DOOR OPEN IN TRANS",
            trans.duration,
            (int) trans.status, (int) trans.flags );
            lcd_BFcolorsB( lcd_MAGENTA, lcd_WHITE);
         } else if (trans.status == ESTS_MANPURGE)
         {  // This is a manual purge event
         	sprintf(buf, "%02d/%02d/%02d %02d:%02d:%02d %-25s %3d %2x%2x",
            time.tm_mon, time.tm_mday, time.tm_year%100, time.tm_hour, time.tm_min, time.tm_sec,
            "MANUAL PURGE",
            trans.duration,
            (int) trans.status, (int) trans.flags );
	      	lcd_BFcolorsB( lcd_BLUE, lcd_WHITE);
         } else if (trans.status == ESTS_AUTOPURGE)
         {  // This is an automatic purge event
         	sprintf(buf, "%02d/%02d/%02d %02d:%02d:%02d %-25s %3d %2x%2x",
            time.tm_mon, time.tm_mday, time.tm_year%100, time.tm_hour, time.tm_min, time.tm_sec,
            "AUTOMATIC PURGE",
            trans.duration,
            (int) trans.status, (int) trans.flags );
	      	lcd_BFcolorsB( lcd_BLUE, lcd_WHITE);
         } else if (trans.status == ESTS_SECURE_REMOVAL)
         {  // This is a secure transaction removal event
	         //   sprintf(buf, "%02d/%02d/%02d %02d:%02d:%02d %-11s %-10s %6ld %2x%2x",
	         //   time.tm_mon, time.tm_mday, time.tm_year%100, time.tm_hour, time.tm_min, time.tm_sec,
	         //   trans.flags < SYSTEM_NAME ? param.station_name[trans.flags].name : "??",
	         //   "SECURE ID",
	         //   (* (unsigned long*) &trans.duration) - CARDBASE,
	         //   (int) trans.status, (int) trans.flags );
            UID[0] = (char) trans.duration >> 8;
            UID[1] = (char) trans.duration && 0xFF;
            UID[2] = trans.source_sta;
            UID[3] = trans.dest_sta;
            UID[4] = trans.flags;
            UID_Decode(UID, UIDstr);
         	sprintf(buf, "%02d/%02d/%02d %02d:%02d:%02d %-10s %-12s",
            time.tm_mon, time.tm_mday, time.tm_year%100, time.tm_hour, time.tm_min, time.tm_sec,
            "SECURE ID", UIDstr);
	      	lcd_BFcolorsD( 0xAA0, 0xFFF);
         } else
         {  // unknown entry type
         	sprintf(buf, "%02d/%02d/%02d %02d:%02d:%02d %-25s %3d %2x%2x",
            time.tm_mon, time.tm_mday, time.tm_year%100, time.tm_hour, time.tm_min, time.tm_sec,
            "UNKNOWN STATUS",
            trans.duration,
            (int) trans.status, (int) trans.flags );
	      	lcd_BFcolorsB( lcd_GREY, lcd_WHITE);
         }
         // Display record unless it is blank or supressed by format
		   if (trans.status != ESTS_BLANK)
         {  if ((format==0) || (trans.status>0 && trans.status<LAST_TRANS_EVENT))
         	{  // ok to show
               lcd_DispText(buf, x, y, MODE_NORMAL);
		         y+=dy;
               j++;
            }
         }
      }
      entry--;
   }
   if (j < TRANS_PER_PAGE)
   {  // clear the rest of the work area
	   lcd_BFcolorsB( lcd_WHITE, lcd_WHITE);
	   lcd_Rectangle(1, y, 320, 204, 1);    // fill left side progress
	   lcd_BFcolorsB( lcd_BLACK, lcd_WHITE);
	}
   lcd_showPage(page+1, lastPage+1);
}
void lcd_showTransSummary()
{
   // uses structure statistics  :  system summary statistics

   char myline[80];     // line to send to the lcd
   char * instr;        // work pointer into myline
   long subtot;         // total of transactions or alarms
   int button;

   button=0;
   while ( (button != BTN_EXIT) && !secTimeout(menu_timeout) )
   {
	   lcd_drawScreen(5, "SUMMARY LOG");
	   lcd_ButtonDef( BTN_CANCEL,
		      BTN_TXTOFFSET, 9, 9,
	         BTN_TLXY, 8, lcd_max_y-40,             // starting x,y for buttons
	         BTN_TEXT, "Reset",
	         BTN_END );

	   lcd_Font("8x12");
	   lcd_DispText(param.stationName, 10, 50, MODE_NORMAL);
	   lcd_DispText("\nTRANSACTIONS:", 0, 0, MODE_NORMAL);
	   sprintf(myline, "\nIncoming:             %ld", statistics.trans_in);
	   lcd_DispText(myline, 0, 0, MODE_NORMAL);
	   sprintf(myline, "\nOutgoing:             %ld", statistics.trans_out);
	   lcd_DispText(myline, 0, 0, MODE_NORMAL);
	   sprintf(myline, "\nTotal Transactions:   %ld", statistics.trans_in + statistics.trans_out);
	   lcd_DispText(myline, 0, 0, MODE_NORMAL);
	   //sprintf(myline, "\n    Grand Total:           %ld", transactionCount());
	   //lcd_DispText(myline, 0, 0, MODE_NORMAL);
	   lcd_DispText("\n\nALARMS", 0, 0, MODE_NORMAL);
	   sprintf(myline, "\nDoor Open During Use: %d", statistics.door_alarm);
	   lcd_DispText(myline, 0, 0, MODE_NORMAL);
//	   sprintf(myline, "\nIncomplete Delivery:  %d", statistics.deliv_alarm);
//	   lcd_DispText(myline, 0, 0, MODE_NORMAL);
	   //sprintf(myline, "\nDiverter Timeout:     %d", statistics.divert_alarm);
	   //lcd_DispText(myline, 0, 0, MODE_NORMAL);
//	   sprintf(myline, "\nCarrier Lift Timeout: %d", statistics.cic_lift_alarm);
// 	   lcd_DispText(myline, 0, 0, MODE_NORMAL);
	   //subtot=statistics.deliv_alarm+statistics.divert_alarm+statistics.cic_lift_alarm;
//	   subtot=statistics.deliv_alarm+statistics.cic_lift_alarm;
//	   sprintf(myline, "\nTotal Alarms:         %ld", subtot);
//	   lcd_DispText(myline, 0, 0, MODE_NORMAL);

	   // wait for any button
	   menu_timeout = SEC_TIMER + 60;
	   while(((button=lcd_GetTouch(100)) == -1) && !secTimeout(menu_timeout) ) maintenance();
      if (button==BTN_CANCEL)
      {  // Ask: Are You Sure?
      	lcd_Font("16B");
         lcd_BFcolorsB( lcd_LGREY, lcd_LGREY);  // unfill color
         lcd_Rectangle(20, 100, 220, 160, 1);    // fill white
         lcd_BFcolorsB( lcd_BLACK, lcd_WHITE);
         lcd_Rectangle(20, 100, 220, 160, 0);    // outline
         lcd_DispText("Press Reset again to confirm\nor Exit to cancel", 30, 115, MODE_TRANS);

	      menu_timeout = SEC_TIMER + 60;
		   while(((button=lcd_GetTouch(100)) == -1) && !secTimeout(menu_timeout) ) maintenance();
         if (button==BTN_CANCEL)
         {  // OK to reset
         	resetStatistics();
         }
         else button=0; // don't really exit, just repaint the screen
      }
   }

}


unsigned long transaction_count;
static unsigned long transCountXaddr;			// physical memory address
/*******************************************************/
unsigned long transactionCount() { return transaction_count; }
/*******************************************************/
nodebug void incrementCounter()
{
   transaction_count++;
   //setCountMessage();
   root2xmem( transCountXaddr, &transaction_count, 4);
   //root2xmem( transCountXaddr+4, &statistics, sizeof(statistics));  // also save stats to xmem
   //syncTransCount(); // send to the slave
}

/*******************************************************/
void loadTransactionCount()
{
	#GLOBAL_INIT
   {	transCountXaddr = 0; }

   // allocate xmem if not already done
   if (transCountXaddr == 0) transCountXaddr = xalloc(32);

   // Read transaction counter
   xmem2root( &transaction_count, transCountXaddr, 4);
   //xmem2root( &statistics, transCountXaddr+4, sizeof(statistics));  // also read stats

   if (transaction_count < 0 || transaction_count > 10000000)
   { resetTransactionCount(0); }

   //setCountMessage();
   //syncTransCount(); // send to the slave
}
/******************************************************************/
void resetTransactionCount(unsigned long value)
{  transaction_count=value;
   //setCountMessage();
   root2xmem( transCountXaddr, &transaction_count, 4);
   //syncTransCount(); // send to the slave
}
void resetStatistics()
{
   // reset system statistics
   statistics.trans_in=0;
   statistics.trans_out=0;
   statistics.deliv_alarm=0;
   statistics.divert_alarm=0;
   statistics.cic_lift_alarm=0;
   statistics.door_alarm=0;
}

