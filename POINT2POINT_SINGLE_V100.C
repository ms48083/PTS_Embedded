/****************************************************************
   POINT2POINT_SINGLE_Vxxx.C - Created from MAINSTREAM_MAIN_V439 3-Aug-2019

   Purpose:    Main station master control program for Pneumatic Tube Control System
               Works with MAINSTREAM_REMOTE_V3xx.

   Target:     Z-World BL2500 Rabbit

   Language:   Dynamic C 9.62

   Created:    Original Jun 12, 2006 by Mike Schwedt (C) MS Technology Solutions

   History:
   03-Aug-19   v100 Original version created from MAINSTREAM_MAIN_V439
   				No inline optic, use CIC for arrival
               Manual mode inputs for vacuum and pressure
               Pause/resume/cancel
               Door locks
               Simplify screen layout
               Added parameters: localSubstation, inlineOptic

*****************************************************************/
#define FIRMWARE_VERSION "Point2Point V1.00"
#define VERS                            1
#define SUBVERS                         00

#define MYDEBUG nodebug
#define STDIO_DISABLE_FLOATS
#define STDIO_ENABLE_LONG_STRINGS
// define PRINT_ON for additional debug printing via printf
#define PRINT_ON 1
#define USE_TCPIP 1
//#define USE_RS485
#memmap xmem  // Required to reduce root memory usage
#class auto
#use "bl25xx.lib"          Controller library
#use "rn_cfg_bl25.lib"     Configuration library
#use "rnet.lib"            RabbitNet library
#use "rnet_driver.lib"     RabbitNet library
//#use "rnet_keyif.lib"      RN1600 keypad library
//#use "rnet_lcdif.lib"      RN1600 LCD library
// #use "RS232.lib"
// setup for serial flash interface on programming port
#define SPI_SER_A
#define SPI_CLK_DIVISOR 10
#define SF1000_CS_PORT			GOCR
#define SF1000_CS_PORTSHADOW	GOCRShadow
#define SF1000_CS_BIT			4
// need to invert the chip select for serial port a
#define CS_ENABLE	 BitWrPortI ( SF1000_CS_PORT, &SF1000_CS_PORTSHADOW, 0, SF1000_CS_BIT );\
	SPIxor = 0xFF;		// invert the received bits
#define CS_DISABLE BitWrPortI ( SF1000_CS_PORT, &SF1000_CS_PORTSHADOW, 1, SF1000_CS_BIT );
#use SF1000.lib
// setup for Reach color LCD
#define	LCD_USE_PORTE				// tell Reach library which serial port to use
#use Reach101.lib

// Setup interrupt handling for carrier arrival input
// Set equal to 1 to use fast interrupt in I&D space
#define FAST_INTERRUPT 0
char latchCarrierArrival;
void my_isr1();
void arrivalEnable(char how);

/* Declare data structures */
#define NUMDATA   6
struct iomessage                // communications message packet
{
   char devAddr;                // device board #
   char command;                // command instruction
   char station;                // carrier station #
   char data[NUMDATA];          // process data
};
struct stats_type               // transaction summary statistics
{
   long trans_in;               // incoming transactions
   long trans_out;              // outgoing transactions
   int  deliv_alarm;            // incomplete delivery timeout alarm
   int  divert_alarm;           // diverter timeout alarm
   int  cic_lift_alarm;         // carrier lift timeout alarm
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
struct secure_log_type           // transaction logging info for secure ID
{  // NO LONGER must be identical in size to trans_log_type
   unsigned long trans_num;     // transaction number
   unsigned long start_tm;      // time of departure
   char sid[5];
   char  status;                // secure removal event = 68
   char  flags;                 // transaction flags, probably 0
   char  scanType;              // type of scan (0=none; 1=secure; 2=standard; 3=unauth)
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

#define NUMUDPDATA 294
struct UDPmessageType  // UDP based messages
{
	char device;
   char devType;  // M for main station
   char command;  // can be heartbeat, event, transaction
   char station;
   unsigned long timestamp;  // MS_TIMER
   char data[NUMUDPDATA];
};
// Define communications using TCP/IP (UDP)
#ifdef USE_TCPIP
//	#define USE_ETHERNET
	#define TCPCONFIG 106
	#define TCP_MODE_ASCII 1
   #define TCP_OPENTIMEOUT 13000
	#define MAX_UDP_SOCKET_BUFFERS 2
	#define MAX_TCP_SOCKET_BUFFERS 1
	//#define DISABLE_TCP		// Not using TCP
   #define MY_STATIC_IP "192.168.0.9"
   #define MY_BASE_IP "192.168.0.%d"
   char myIPaddress[18];  // string to hold runtime IP address
	#define MY_IP_NETMASK "255.255.255.0"
	#define REMOTE_IP    "192.168.0.101" // web server
	//#define REMOTE_IP    "0.0.0.0" //255.255.255.255" /*broadcast*/
	// #define REMOTE_PORT  1236 (this may be causing unimportant listen activity)
	#define CTRL_PORT 1235
   #define HOST_PORT 1236
	#define HOST_LOCAL_PORT 1237
   #define CTRL_LOCAL_PORT 1234
	#use "dcrtcp.lib"
   // #use “http.lib”
   tcp_Socket tcpsock;
	char tcpBuffer[1024];
	int tcpServerCheck(void);
   int TCP_Active;
	udp_Socket HostSock;
   udp_Socket CtrlSock;
   #define UDP_HOST 1
   #define UDP_CTRL 2
	int sendUDPHeartbeatToHost(char sample);
	int sendUDPHeartbeatToCtrl(char sample, char transFlags);
	void checkNsendHeartbeat(char sample, char transFlags);
	int slaveSendUDPHeartbeat(char sample);
	int sendUDPcommand(struct UDPmessageType message, char targ);
	int getUDPcommand(struct UDPmessageType *message);
	void processUDPcommand(struct UDPmessageType message);
	void slaveProcessUDPcommand(struct UDPmessageType message);
	int sendRS485toUDP(struct iomessage RSmsg);
   void sendUDPtransaction(struct trans_log_type, char xtra);
	void monitorUDPactivity(char deviceID);
   void UDPcheckRemoteConfiguration(void);
//   void AckProcessMessage(void);
   char AckValue;
   char AckStrobe;
   char AckAck;

#endif

// Including parameter block structure
// All parameters here are read to / written from FLASH
#define UIDSize 1000
// UIDLen more than 5 is not supported by UID_Send due to command length limit
#define UIDLen  5
struct par_block_type
{  // Define all parameters that are saved in flash
   // They are r/w by readParameterSettings() and writeParameterSettings()
   char sysid;
   char names_init;
	char stacking_ok[2];
   char remoteAudibleAlert;
	char localDiverter;      // local diverter configuration
   char activeStations;     // active station set
   int  autoPurgeTimer;     // auto purge timer in seconds
   int  deliveryTimeout;    // delivery timeout in seconds
   char headDiverter;
   char pw_enabled;
   char admPassword[5];    // password for administration functions - APPEARS UNUSED
   struct {char name[12];} station_name[10];  // 0=main; 1-7=remote; 8=slave; 9=system;
   char slaveController;    // master (0) or slave (>0); who am I
   char activeMain;         // master (0) or slave (>0); who is active
   char adminPassword[5]; // password for administrative functions
   char maintPassword[5]; // password for maintenance functions
   char cfgPassword[5];   // password for system config (Colombo only)
   char phoneNum[11][9];  // phone numbers for stations, admin, maintenance
   char autoReturnTimer;  // how long to wait on unremoved carrier until auto-return
   char mainAudibleAlert;
   char manPurgeTimeout;  // how long in minutes to run manual purge before timeout - currently fixed / non-settable
   char subStaAddressing; // If destination addressing by substations is allowed/enabled
   char blowerType;       // standard or APU blower
   char portMapping[9];   // port map index for remote controllers; array index = remote address
   char systemNum;        // ID for multi-system configurations, also defines IP address
   char secureEnabled;    // If secure transaction feature is enabled
   char areturn_arrive_alert;  // Beep on auto-return
//   char cardL3Digits;     // Constant left 3 digits of valid secure cards
//	unsigned long cardR9Min;  // Secure card ID minimum value for the right 9 digits
//   unsigned long cardR9Max;  // Secure card ID maximum value for the right 9 digits
//   char card2L3Digits;     // Constant left 3 digits of valid secure cards
//	unsigned long card2R9Min;  // Secure card ID minimum value for the right 9 digits
//   unsigned long card2R9Max;  // Secure card ID maximum value for the right 9 digits
//   char version;          // Main version
//   char subversion;       // Main subversion
	char serverID[20];     // server IP or resolvable name
   char mainVisualAlert;  // enable visual alert at main
   char enableTubeDryOpt; // enable menu item selection
   char  syncInterval;     // how often in minutes to ping server for refresh
   char cardCheckEnabled;  // to enable individual card checking instead of range
   char cardFormat;       // defines type of cards (e.g. 26 or 40 bit)
   char point2point;      // operate as a point to point
   char orientLandscape;  // screen orientation
	char inlineOptic;      // arrival by CIC only when inlineOptic = false
	char localSubstation;  // control of remote station with local I/O
   char pauseEnabled;
   // -->> BE SURE to change NUMUDPDATA if more parameters are added <<--
   // -->> AND BE SURE to reduce reserved[] for every addl parameter above <<--
   char reserved[92];    // to separate standard parameters from the user id's
   unsigned long UIDSync; // timestamp (seconds) since last good block
   int UIDRecNo;          // record number of last sent UID
   char UID[UIDSize][UIDLen];
} param;
#define SYSTEM_NAME  9  // last one is the system name   char station_names[12][8];
#define ADMIN_PHONE  9
#define MAINT_PHONE  10
#define NUM_STATIONS 8

// ------------------------------
// MISCELLANEOUS DEFINITIONS HERE
// ------------------------------
char  alarm_silenced_flag;
char diverter_map[9];    // to assign diverter positions to stations
unsigned long diverter_start_time;  // these used primarily by diverter functions
int diverter_status;
char diverterDevices; // set of active diverter devices
unsigned long menu_timeout;
char diverter_setting;
char diverter_attention;
char printlog_active;    // whether printout of trans log is enabled
char  printer_error;     // stops printing on printer_wait timeout
#define HEADDIVERTERSTATION 0x80 // 8=headdiverter station bit
// ------------------------------
// ------------------------------

//////
// RabbitNet Setup
//int DevRN1600;                   // Rabbit Net Device Number Keypad/LCD
int DevRN1100;                   // Rabbit Net Device Number Digital I/O
int RN1100_Found;  // For alternate I/O configuration w/o RN1100
#define OUTCONFIG 0xFFFF         //configure outputs to sinking safe state

/////
//local macros
/////
#define ON           0xFF
#define OFF          0x00
#define FLASH        0x01
#define READY        0x01
#define NOT_READY    0xFF
// macros for I/O processing functions
#define INIT       0
#define RUN        1
#define OPERATE    2
#define TRIGGERED  3
#define RUNNING    4
#define HOLD       5
#define ALL_STATIONS 0xFF   // hardware will support 8 stations
#define LAST_STATION    8   // # of bits from STATION_SET  (DO COUNT SLAVE FOR NOW)!!!
char    STATION_SET;        // active stations
char    aliveStations;     // as programmed from the menu
char    FIRST_DEVADDR;
char    LAST_DEVADDR;
char    availableDevices;
#define ALL_DEVICES   0xFF
#define MAX_DEVADDRS  8   // maximum number of remote devices
#define MASTER  0x00      // Station number of the master
#define SLAVE   0x08      // Station number of the slave
#define SLAVE_b 0x80      // Bit representation
#define SLAVE_DEVADDR 8
#define HEADDIV_DEVADDR 7
//////////////////////////
// Master/slave variables & function calls
char slaveAvailable;   //
char slaveData[NUMDATA];// special storage for slave status data
char slave_cic, slave_doorClosed, slave_rts, slave_rtncarrier, slave_autoReturn; //
char slave_arrive;
char main2mainTrans;   // 0=std.trans; DIR_SEND=mast->slave; DIR_RETURN=slave->mast
char slaveReturnStatus;
char mainStation;
char destMain;
char main2main_trans;
unsigned long lastComTime;
char rts_latch;      /* Used for latching requestToSend */
char activeMainHere; // ONLY FOR USE IN SETUP MENU
MYDEBUG void setActiveMainMsg(void);
MYDEBUG void setActiveMain(char setToMaster);
MYDEBUG void syncDateAndTime(void);	// for master to send clock to slave
MYDEBUG void syncTransCount(void);		// for master to send transaction count to slave
MYDEBUG void displayRemoteMsgs(char * message_num);  // for master to send lcd msgs to slave
void sendParametersToLogger(void);  // to send parameter settings to the data logger
void deliverComControl(char *system_state);  // for master to hand over com to slave
///void slaveProcessCommand(struct iomessage message);
///void slaveProcessIO(void);
///void slaveProcessState(void);
///void slaveSyncTransCount(char *p);  // for slave to receive transaction count
///void slaveEnableCommands(void);
///char slaveGetCommand(struct iomessage *message);
///void slaveSendResponse(struct iomessage message);
///char slaveComActive(void);  // for slave to know if communication is active
///void slaveFinishTransaction(void);
void initTransLog(void);
void downloadTransLog(int numDays); // to local serial port
void uploadTransLog(long start);   // to datalog server
void addTransaction(struct trans_log_type trans, char xtra);
void addSecureTransaction(struct secure_log_type);
int getTransaction(long entry, struct trans_log_type *trans);
long findTransaction(unsigned long transNum);
MYDEBUG long sizeOfTransLog(void);
void checkSerialFlash(void);
void analyzeEventLog(void);

/* Serial communications send and receive commands */
#define NAK 0x15
#define ACK 0x06
#define STX 0x02
#define ETX 0x03
#define DINBUFSIZE 63
#define DOUTBUFSIZE 63
#define FINBUFSIZE 63
#define FOUTBUFSIZE 127

/* Remote communication commands */
#define SET_DIVERTER       'A'
#define DIVERTER_STATUS    'B'
#define REMOTE_PAYLOAD     'C'
#define SECURE_REMOVAL     'C'
#define ACK_SECURE_REMOVAL 'D'
#define RETURN_INPUTS      'E'
#define INPUTS_ARE         'E'  // set equal to RETURN_INPUTS (v2)
#define SET_OUTPUTS        'G'
#define CLEAR_OUTPUTS      'H'
#define SET_MODE           'I'
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
#define SYSTEM_EVENT       'V'
#define ESECURE_REMOVAL    'W'
#define ECARD_SCAN         'w'
#define TRANS_COMPLETE     'X'
#define RESET              'Y'
#define MALFUNCTION        'Z'

// Declare constant system timeouts
#define DIVERTER_TIMEOUT   30000       // How long to set diverter in msec
#define CIC_EXIT_TIMEOUT   15000       // How long for carrier to escape
#define MALFUNC_TIMEOUT    30000       // How long in malfunction loop
#define STAT_TIMEOUT       60          // Priority reset in seconds
#define PASSWORD_TIMEOUT   10000       // Timeout for operator entry

/* Declare I/O device indexes */
#define devInUseSolid  0
#define devInUseFlash  1
#define devAlert       2
#define devAlarm       3
char outputvalue[4];  // digital output buffers

/* Array index values for remote data returned by INPUTS_ARE command. */
#define REMOTE_CIC      0
#define REMOTE_DOOR     1
#define REMOTE_ARRIVE   2
#define REMOTE_RTS      3
#define REMOTE_RTS2     4
#define REMOTE_DIVERTER 5

// Digital I/O definitions
// INPUTS
int readDigInput(char channel);
// dio_ON|OFF must be used ONLY in primatives readDigInput and setDigOutput
// they reflect the real state of the logic inputs and outputs
#define dio_ON  0
#define dio_OFF 1
// Menu/Next button assigned to input I-0
#define di_carrierArrival    		0 //readDigInput(0)
#define di_carrierInChamber  		readDigInput(2)
#define di_doorClosed        		readDigInput(4)
#define di_doorClosed2       		readDigInput(5)
#define di_priorityRequest   		0 //readDigInput(3)
#define di_returnCarrier     		0 //readDigInput(4)
//#define di_pushButton(value) 	!readDigInput(5+value)
//#define di_P2P_carrierArrival 	readDigInput(5)
//#define di_P2P_carrierInChamber	readDigInput(6)
//#define di_P2P_doorClosed			readDigInput(7)
//#define di_requestToSend     	((~digBankIn(1)) & STATION_SET)

//#define di_requestToSend     		((~digBankIn(1)) & 0xFF)
//#define di_diverterPos       		(readDigInput(16) | (readDigInput(17) << 1))
// Remap I/O when RN1100_Found or not
#define di_requestToSend     		readDigInput(0) // (RN1100_Found ? ((~digBankIn(1)) & 0xFF) : 0)
#define di_diverterPos       		(RN1100_Found ? (readDigInput(16) | (readDigInput(17) << 1)) : (readDigInput(8) | (readDigInput(9) << 1)))
// local inputs for remote station control
#define di_carrierInChamber2 		readDigInput(3)
#define di_requestToSend2    		readDigInput(1)
#define di_pause						readDigInput(6)
#define di_pause2						readDigInput(7)
#define di_manVacuum					readDigInput(8)
#define di_manPressure				readDigInput(9)
#define di_alarmReset				readDigInput(10)

// NEED TO DEFINE INPUT FOR SHIFTER POSITIONS
#define di_shifterPosIdle        readDigInput(0)
#define di_shifterPosPrs         readDigInput(0)
#define di_shifterPosVac         readDigInput(0)
MYDEBUG char carrierArrival(char whichMain);  // 0=MASTER; 8=SLAVE
MYDEBUG char carrierInChamber(char whichMain);
MYDEBUG char doorClosed(char whichMain);

// OUTPUTS
MYDEBUG void setDigOutput(int channel, int value);
#define do_shift 0
//#define do_closeDoorAlarm(value)   setDigOutput(6+do_shift,value)
// redefine for RN1100_Found at runtime
//#define do_blowerVac(value)        setDigOutput(12+do_shift,value)
//#define do_blowerPrs(value)        setDigOutput(13+do_shift,value)
//#define do_blower(value)           setDigOutput(9+do_shift,value)
//#define do_blowerDly(value)		  setDigOutput(16+do_shift,value)
//#define do_audibleAlert(value)     setDigOutput(11+do_shift,value)
//#define do_diverter(value)         setDigOutput(8+do_shift,value)
//#define do_alarmSound(value)       setDigOutput(10+do_shift,value)
/////
#define do_blowerVac(value)        RN1100_Found ? setDigOutput(12+do_shift,value) : setDigOutput(5+do_shift,value)
#define do_blowerPrs(value)        RN1100_Found ? setDigOutput(13+do_shift,value) : setDigOutput(6+do_shift,value)
#define do_blower(value)           RN1100_Found ? setDigOutput(9+do_shift,value)  : setDigOutput(99+do_shift,value)
#define do_blowerDly(value)		  RN1100_Found ? setDigOutput(16+do_shift,value) : setDigOutput(99+do_shift,value)
#define do_audibleAlert(value)     RN1100_Found ? setDigOutput(11+do_shift,value) : setDigOutput(99+do_shift,value)
#define do_diverter(value)         RN1100_Found ? setDigOutput(8+do_shift,value)  : setDigOutput(99+do_shift,value)
#define do_alarmSound(value)       RN1100_Found ? setDigOutput(10+do_shift,value) : setDigOutput(4+do_shift,value)
#define do_doorUnLock(value)		  setDigOutput(1,value)
#define do_priorityLight(value)    setDigOutput(99+do_shift,value)
#define do_CICLight(value)         ((void)0)
#define do_visualAlert(value)      setDigOutput(99+do_shift,value)
void setAlerts(char how);    // sets both audible and visual alerts
//#define do_P2P_CICLight(value) 	  setDigOutput(17+do_shift,value)
//#define do_P2P_alert(value)	 	  setDigOutput(18+do_shift,value)
// local outputs for remote station control
#define do_CICLight2(value)         ((void)0)

#define beep(value)                rn_keyBuzzerAct(DevRN1600, value, 0)
MYDEBUG void inUse(char how, char station_b);
MYDEBUG void alarm(char how);
//void alert(char how, char station_b);
//void lockDoor(char how);  // lock (TRUE) or unlock (FALSE)

//MYDEBUG char getKey(void);
// Functions for digital input based keys
//char  functionButton(char which, char how);
#define  FUNC_BUTT  0x00
#define  FUNC_F1    0x01
#define  FUNC_F2    0x02
#define  FUNC_TRIG  0x01                // triggered (latched) mode
#define  FUNC_CURR  0x02                // current button state
#define  FUNC_HELD  0x03                // auto repeat key
#define  FUNC_DELAY 1000                // auto repeat delay
#define  FUNC_RATE  100                 // auto repeat rate


// LCD definitions
#define DISPROWS 4    //number of lines in display
#define DISPCOLS 16    //number of columns in display
MYDEBUG void initializeMessages(void);
MYDEBUG void message_add(char lcd, char msg1, char msg2, char how);
MYDEBUG void message_del(char lcd, char msg1, char msg2);
MYDEBUG void message_show(char how);
MYDEBUG void message_refresh(void);
MYDEBUG void refresh_dynamic_message(char msg);
MYDEBUG void refresh_PIN_message(char msg1, char msg2);
void lcd_initialize(void);
void lputs(char line, char* msg);
void reset_msg_timer(char how);
void lcd_print(char lcd, char line, char* message);
void lcd_show_cursor(char lcd, char line, char pos);
MYDEBUG void setTimeMessages(char message, unsigned long timedate);
MYDEBUG void setFlagMessage(char flag);
MYDEBUG void setParValueMessage(char MSG_message, unsigned long par_value, char * par_units);
void update_cic_door_msgs(void);
void maintenance(void);
#define EMPTY        255      // empty message
#define QUEUESIZE    20       // max number of messages per lcd
#define SCROLLTIME   2000     // 2 seconds
#define NOW          1        // show message immediately
#define NEXT         2        // show message next
#define ONESHOT      3        // show message once right now
#define ONESHOTA     4        // show message once first of 2 oneshots
#define LCDCOUNT     2        // how many lcds connected
#define TRANSLCD     0        // Trasaction Status display
#define SYSTEMLCD    1        // System Status display
/* Define STATIC LCD messages */
#define MSGCOUNT  106
const char * const sysmsg[MSGCOUNT] = {
"Insert Carrier and","Touch Send Button ",
"NEXT TRANSACTION  ","  STAT PRIORITY   ",
"  LOG TO PRINTER  ","  LOCAL DIVERTER  ",
"  ENTER PASSWORD  "," VERIFY PASSWORD  ",
"  TRANS TIMEOUT   ","  PLEASE WAIT...  ",
"   SYSTEM ALARM   "," SEE MAINTENANCE  ",
" SETTING DIVERTER ","    DOOR OPEN     ",
"Carrier in Chamber","Delivery Unremoved",
" DELIVERY OVERDUE ","  CARRIER RETURN  ",
"   MANUAL PURGE   ","TRANSACTN CANCELED",
"   BUSY OR OFF    ","    DEVICE #      ",
"   WAITING FOR    "," SYSTEM CONFIG    "," SET TIME & DATE  ",
" PENDING DELIVERY ","     IN ROUTE     "," AT HEAD DIVERTER ",
"     TIMEOUT      ","      ERROR       ","  REMOTE STATION  ","  FUNCTION MODE   ",
"    SET HOURS     ","   SET MINUTES    ","   SET SECONDS    ",
"     SET DAY      ","    SET MONTH     ","     SET YEAR     ",
" (+)=UP (-)=DOWN  "," (+)=ON  (-)=OFF  "," AUTO PURGE TIMER ",
" BEEP ON ARRIVAL  ","  SILENCE ALARM   ","   RESET ALARM    ",
"   SYSTEM RESET   ","  SYSTEM IS BUSY  ",
"   CAN NOT SEND   ","can't send reason ",
"  MANUAL VACUUM   ","  MANUAL PRESSURE ",
"   system name    ","  OPTIC BLOCKED   ",
"        @M        ",
"        @1        ","        @2        ","        @3        ","        @4        ",
"        @5        ","        @6        ","        @7        ","        @8        ",
"        <M        ",
"        <1        ","        <2        ","        <3        ","        <4        ",
"        <5        ","        <6        ","        <7        ","        <8        ",
"        >M        ",
"        >1        ","        >2        ","        >3        ","        >4        ",
"        >5        ","        >6        ","        >7        ","        >8        ",
"SECURE PIN = #### ","  SYSTEM PAUSED   ",
" empty 7 xxxxxxx  ","  empty 8 xxxxxx  ","  empty 9 xxxxxx  ",
" --END OF MENU--- ","    NOT READY     ","   ERROR SAVING   ",
"  STACK AT MAIN   "," STACK AT REMOTE  ","  RESET COUNTER   ",
"  TIME: HR:MN     ","  DATE: MO/DY/YR  ","                  ",
"  CURRENTLY Oxx   ","  MO/DY/YR HR:MN  ","    DO NOT USE    ",
" CHECKING SERVER  "," PLEASE WAIT ...  ",
"  HEAD DIVERTER   ","  SETTING BLOWER  ",
" PARAMETER VALUE  "," empty 12 xxxxxxx ",
"SECURE TRANSACTION","   SET AS SLAVE   "," SET ACTIVE MAIN  ",
"                  "};      // last element always blank

// Define message indexes into rom or ram messages
enum messageIDs {
 MSG_SDEFAULT    = 0,
 MSG_SDEFAULT2,
 MSG_STAT,
 //MSG_STAT2
 MSG_LOG2PRINTER  = 4,
 MSG_LOCALDIVERTER,
 MSG_ENTERPW,
 // no 7
 MSG_SET_TIMEOUT  = 8,
 MSG_WAIT,
 MSG_ALARM,
 // no 11
 MSG_SETTING     = 12,
 MSG_DROPEN,
 MSG_CIC,
 MSG_ARVALRT,
 MSG_ODUE,
 MSG_RTRN,
 MSG_PURGE,
 // no 19
 MSG_NOCOMM      = 20,
 // no 21,22
 MSG_SYS_CONFIG  = 23,
 MSG_SET_CLOCK,
 MSG_PENDING,
 MSG_INROUTE,
 MSG_AT_HEADDIV,
 MSG_TIMEOUT,
 MSG_ERROR,
 MSG_REMOTESTA,
 MSG_FUNCTION,
 MSG_SET_HOURS,
 // no 33-37
 MSG_INC_DEC     = 38,
 MSG_ON_OFF,
 MSG_APURGE_TIMER,
 MSG_ARRIVE_SND,
 MSG_ALARM_SND,
 MSG_ALARM_RESET,
 MSG_RESET,
 MSG_SYSBUSY,
 MSG_CANT_SEND,
 MSG_CANT_WHY,
 MSG_MAN_PRESSURE,
 MSG_MAN_VACUUM,
 MSG_SYSTEM_NAME,
 MSG_OPTIC_BLOCKED,
 MSG_AT,                 // points to message before station 1
 // no 53-60
 MSG_FROM        = 61,   //   ..
 // no 62-69
 MSG_TO          = 70,   //   ..
 // no 71-78
 MSG_PIN         = 79,
 MSG_PAUSED,
 MSG_empty7_xxx,
 MSG_empty8_xxx,
 MSG_empty9_xxx,
 MSG_END_OF_MENU,
 MSG_NOT_READY,
 MSG_SAVE_ERROR,
 MSG_MSTACKING,
 MSG_RSTACKING,
 MSG_RESET_COUNT,
 MSG_TIME,
 MSG_DATE,
 MSG_TRANS_COUNT,
 MSG_FLAG_SET,     // on/off flag setting
 MSG_DATE_TIME,
 MSG_DONTUSE,
 MSG_CHECKING_SERVER1,
 MSG_CHECKING_SERVER2,
 MSG_HEADDIVERTER,
 MSG_BLOWER,
 MSG_PAR_VALUE,
 MSG_empty12_xxx,
 MSG_SECURE,
 MSG_SETASSLAVE,
 MSG_SETACTIVEMAIN,
 MSG_BLANK			// always the last message
};

#define MSG_TDEFAULT    MSG_DATE_TIME
//#define MSG_TDEFAULT    MSG_SYSTEM_NAME
#define MSG_TDEFAULT2   MSG_TRANS_COUNT

// Define structure and array for variable type messages
// message length is 16 or 20 (18?) (+1 for null terminator \n)
#define MSG_LEN 18
// Define leader offset into messages (0 for 16 char lcd, 2 for 20 char lcd)
#define MSG_OFFSET 1
struct msgbuftype {char msg[MSG_LEN+1];} sys_message[MSGCOUNT];

// Diagnostic Menu/Setup/Parameters function definitions
//void displayFunctionMessage(char mnu_base, char msg_item, char msg_value);
char functionSetTime(char what_timer, char index, unsigned long *time_val);
char functionSetParValue(char menu_item, unsigned long * par_value,
         unsigned long par_min, unsigned long par_max, char * par_units);
char functionDefineStations(void);
MYDEBUG int readParameterSettings(void);
MYDEBUG int writeParameterSettings(void);
MYDEBUG int syncParametersToRemote(void);

// USER ID MANAGEMENT DEFINITIONS
int UID_Add(char *ID);
int UID_Del(char *ID);
int UID_Search(char *ID);
int UID_Get(int UIndex);
int UID_Send(char *ID, char how);
unsigned int UID_Cksum();
int UID_Decode(char *UID, char *Result);
int UID_Encode(char *UID, char *Result);
int UID_Parse(char *UIDBuffer);
int UID_ResyncAll(void);
int UID_Not_Zero(char *UID);
void UID_Clear(char *UID);
void UID_Copy(char *Udest, char *Usrc);
unsigned long cvtTime(char *timestring, char setRTC);
unsigned long pingTimer;
int Sync_Clock(char *Buffer);
int Sync_Trans_Log(char *Buffer);
void Param_Flash_Write(char command);
int UID_GetAddOnly;

// General/Misc definitions
char init_communication(void);
void print_comstats(void);
void comIsGood(char device);

MYDEBUG void showactivity(void);
MYDEBUG void msDelay(unsigned int delay);
MYDEBUG char secTimeout(unsigned long ttime);
MYDEBUG char Timeout(unsigned long start_time, unsigned long duration);
MYDEBUG void toggleLED(char LEDDev);
MYDEBUG void initialize_system(void);
MYDEBUG char bit2station(char station_b);
MYDEBUG char station2bit(char station);
MYDEBUG char firstBit(char bitdata);
//void diverter(char how);
MYDEBUG void set_diverter(char station);
MYDEBUG void check_diverter(void);
MYDEBUG char divertersReady(char station, char headDiverter, char mainDiverter);
MYDEBUG void setDiverterConfiguration(void);
MYDEBUG char  valid_send_request(char station_b);
// transaction request queue functions   USE BIT VALUES
MYDEBUG void queueAdd(char source, char dest);   // adds request to queue
MYDEBUG void queueDel(char source, char dest);   // deletes request from queue
MYDEBUG void queueNext(char *source, char *dest);  // returns next queue entry
MYDEBUG char queueVerify(char cic_data, char door_data);  // validate queue entries
char send_n_get(struct iomessage message, struct iomessage *response);
char send_command(struct iomessage);
char get_response(struct iomessage *);
MYDEBUG void incrementCounter(void);
MYDEBUG void reset_statistics(void);
MYDEBUG void buildStationNames(void);
//void  initializeMessages(void);
unsigned long transactionCount(void);
MYDEBUG void loadTransactionCount(void);
MYDEBUG void resetTransactionCount(unsigned long value);
MYDEBUG void resetStatistics(void);
MYDEBUG void show_extended_data(void);
char extendedData[9][4]; // Extended remote data, devices 0-8
void purgeSystem(char purge_mode);
#define PURGE_MANUAL    0
#define PURGE_AUTOMATIC 1
#define PURGE_DRYING    2
void exercise_outputs(void);
void show_comm_status(void);
void show_comm_test(char wait);
void show_ip_address(void);
void checkRemoteConfiguration(void);
// PRINTER FUNCTIONS
//void print_line(char *buf);
//void printer_wait(void);
//void print_transaction(struct trans_log_type log);
//void print_summary(struct stats_type stats, char how);
//void print_malfunction(struct trans_log_type log);
void print_event(char *event);
long nextAutoPrint;
//void reset_printer(void);
//void check_auto_report(struct stats_type stats);
//void reset_auto_print(void);
void time2string(unsigned long time, char *buf, int format);
void lcd_splashScreen(char view);
void lcd_helpScreen(char view);
void lcd_drawScreen(char whichScreen, char *title);
void lcd_displayMsgs(char * message_num);  // to show msgs on touch screen
int countSpaces(char * msg);
int psTextPos(char * msg);
void lcd_resetMsgBackColor(void);
void lcd_refreshMsgs(void);
void lcd_printMessage(char line, char * msg);
void lcd_processTouchScreen(int button);
void lcd_enterSetupMode(char operatorLevel);
void lcd_ShowTransactionProgress(char source, char dest, char direction, int progress, unsigned long transTimer);
int lcd_SelectChoice(char *choice, char *label, char *optZero, char *optOne);
char lcd_GetNumberEntry(char *description, unsigned long * par_value,
         unsigned long par_min, unsigned long par_max, char * par_units, char max_digits, char nc_as_ok);
char lcd_enableAdvancedFeatures(char menu_level, char * description);
char lcd_getPin(char *PIN, char *description);
int  lcd_Center(char * string, char font);
void lcd_clearMiddle(void);
void lcd_ShowKeypad(char opts);
void lcd_drawKeyboard(void);
char lcd_editStationNames(void);
char lcd_editServerName(void);
char lcd_editPhoneNums(void);
char lcd_editPortMapping(char * title);
void lcd_showCIC(char forceit);
void lcd_setClock(void);
void lcd_showClockMessage(void);
char lcd_defineActiveStations(void);
void lcd_showTransSummary(void);
void lcd_showDirectory(char mode);
void lcd_showPage(long thisPage, long lastPage);
void lcd_showTransPage(long page, long lastPage, char format);
void lcd_showTransactions(char format);
void lcd_show_inputs(void);
void lcd_sendToSet(char station, char direction);
char lcd_sendTo(void);
char lcd_returnCarrier(void);
char lcd_autoReturn(void);
//void lcd_screenSaver(char brightness);
//void lcd_smartPrompt(char promptNum);
char * ltrim(char * text);
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
#define	BMP_lg_button         23
#define	BMP_lg_button_dn      24
#define	BMP_Front_LOGO 		 25

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


// State Processing definitions
void processSystemIO(void);
//char processKeyInput(void);
MYDEBUG void processCycleTime(void);
void processMainArrivalAlert(void);
void processStatPriority(void);
//void processOutputLamps(char olOperation);
char checkDoorWhileRunning(char calling_state);

/* Declare system states */
#define  IDLE_STATE             0x01
#define  PREPARE_SEND           0x02
#define  WAIT_FOR_DIVERTERS     0x03
#define  BEGIN_SEND             0x04
#define  WAIT_FOR_MAIN_DEPART   0x05
#define  WAIT_FOR_REM_ARRIVE    0x06
#define  HOLD_TRANSACTION       0x07
#define  WAIT_FOR_REM_DEPART    0x08
#define  WAIT_FOR_MAIN_ARRIVE   0x09
#define  WAIT_FOR_HEADDIVERTER  0x0A
#define  SHIFT_HEADDIVERTER     0x0B
#define  CANCEL_STATE           0x0C
#define  MALFUNCTION_STATE      0x0D
#define  FINAL_COMMAND          0x0E
#define  MANUAL_VACUUM			  0x0F
#define  MANUAL_PRESSURE        0x10
#define  PAUSE_TRANSACTION      0x11

#define  DIR_SEND             1
#define  DIR_RETURN           2

// Declare blower interfaces
#define blwrType_STD  1
#define blwrType_APU  2
#define blwrOFF  0
// in standard blower systems blwrIDLE and blwrOFF are both 0 = OFF.
// in APU blower systems blwrIDLE is 1
#define blwrIDLE 1
#define blwrVAC  2
#define blwrPRS  3

char blwrConfig; // later to become parameter from eeprom
char blwrError;
char remoteBlwrPos;
void blower(char blowerOperatingValue);  // use blwrOFF, blwrIDLE, blwrVAC, blwrPRS
char processBlower(void);
char blowerPosition(void);
void initBlower(void);
char blowerReady(void);

// Blower state is based on system_state -> [system_state][blwrConfig]
// blwrConfig is either standard straight through system (0) or head-diverter (1)
// independent of blower type standard or APU
const char blowerStateTable[18][2] = {
	blwrOFF,  blwrOFF,	 // Undefined state 0
	blwrIDLE, blwrIDLE,  // 0x01 (Idle)
	blwrIDLE, blwrIDLE,  // 0x02
	blwrIDLE, blwrIDLE,  // 0x03
	blwrIDLE, blwrIDLE,  // 0x04
	blwrPRS,  blwrVAC,   // 0x05 (Main departure)
	blwrPRS,  blwrPRS,   // 0x06 (Remote arrival)
	blwrIDLE, blwrIDLE,  // 0x07
	blwrVAC,  blwrVAC,   // 0x08 (Remote departure)
	blwrVAC,  blwrPRS,   // 0x09 (Main arrival)
	blwrIDLE, blwrVAC,   // 0x0A (Wait for headdiverter)
	blwrIDLE, blwrIDLE,  // 0x0B
	blwrIDLE, blwrIDLE,  // 0x0C
	blwrOFF,  blwrOFF,   // 0x0D
	blwrIDLE, blwrIDLE,  // 0x0E };// 0x0E
   blwrVAC,  blwrVAC,   // 0x0F
   blwrPRS,  blwrPRS,   // }; // 0x10
	blwrOFF,  blwrOFF};  // 0x11


// Globals, system parameters, variables for process state information
char op_mode, saved_state;
unsigned long arrival_time;
unsigned long state_timer;
char diagnostic_mode;
char system_state, systemStation, systemStationb, malfunctionActive;
char statTrans, system_direction, memory_state;
char secureTrans, autoRtnTrans;
int securePIN[10];  // 0=in_process; 1-7=station; 8=slave
// offset to get printed card id number
#define CARDBASE 0xE8777C00
// offsets to handle whole 38 bits
#define CARDHBOFFSET  119
#define CARDLBOFFSET  3676144640
struct secure_log_type secureTransLog[10];
char secureAck;
char checkSecureRemoval(void);
char btnStatFlag;
char btnSecureFlag;
char btnAReturnFlag;
char arrival_from;
char arrival_alert;               // flag to track unremoved deliveries
unsigned long autoReturnTime[10]; // timers for handling auto-return feature
unsigned long arrivalTime[10];    // timers for when remote arrivals occur
char autoReturnCarrier(void);     // function to determine if an auto-return is ready to go
unsigned long main_arrival;
unsigned long slave_arrival;
unsigned long stat_expire;
static struct stats_type statistics;

void main()
{
//char mybuf[13];
   /* Declare local variables */
   struct iomessage testcmd, testrsp;
   unsigned long testtime, t1, t2, comfail;
   char i;
   char tcpHeartBeat;
   char * genptr;
   auto rn_search newdev;
   int status;
   unsigned long lastHeartBeat;      // host network
   unsigned long heartbeat_timer;
   unsigned long heartbeat_interval;

	lastHeartBeat = 0;
   //unsigned long TCPCheckTimer;
   tcpHeartBeat=0;

   // Initialize the controller
   brdInit();              // Initialize the controller
   rn_init(RN_PORTS, 1);   // Initialize controller RN ports

   // Verify that the Rabbitnet boards are connected
   newdev.flags = RN_MATCH_PRDID;
   newdev.productid = RN1100;
   if ((DevRN1100 = rn_find(&newdev)) == -1)
   {
      printf("\n no RN1100 found\n");
      RN1100_Found = 0;
   }
   else
   {  status = rn_digOutConfig(DevRN1100, OUTCONFIG);  //configure safe state
      RN1100_Found = -1;
	}

   // show startup screen
   initialize_system();
   lcd_splashScreen(0);

	// allocate xmem for transaction log
	initTransLog();

   hitwd();          // "hit" the watchdog timer

   /* Initialize digital outputs */
   inUse(OFF, ALL_STATIONS);
   setAlerts(OFF);
   alarm(OFF);
	initBlower();

   init_communication();          // Initialize serial communications

   /* Initialize LCD displays */
   hitwd();
   //lcd_initialize();
   //lcd_splashScreen(0);
	//lcd_print(TRANSLCD, 0, FIRMWARE_VERSION);
   //lcd_print(TRANSLCD, 1, "PLEASE WAIT...  ");

   //hitwd();

   // Initialize other system configuration
   initializeMessages();     // Initialize dynamic (RAM) messages
   loadTransactionCount();   // Read transaction counter from EEPROM
   setDiverterConfiguration();     // setup diverter mappings.

#ifdef USE_TCPIP
   // move sock_init up here to avoid unexpected interrupt from calling tcp_tick before sock_init
   if(sock_init()) printf("IP sock_init() failed\n");	// Initialize UDP communications
#endif
	TCP_Active = -1;  // assume it is active for now

   /* Exercise all lighted outputs except on watchdog reset */
   // if (!wderror())
   if (1) // don't have way to check if reset was by watchdog
   {
	   // show missing H/W on LCD
	   if (DevRN1100 == -1)
	   {  //lcd_print(1, 0, "NO RN1100 FOUND...  ");
		   lcd_DispText("No RN1100 detected\n",0, 0, MODE_NORMAL);
	      //msDelay(1000);
	   }

      // Perform bulb check
	   //lcd_DispText("Indicator Check\n",0, 0, MODE_NORMAL);
      //exercise_outputs();
      if (param.slaveController == 0)
      {
      	// MUST DO THIS, TO AVOID WATCHDOG RESET???
	      checkRemoteConfiguration();             // how many remotes, etc.

	      // get and show extended data
	      //testcmd.devAddr=ALL_DEVICES;
	      //testcmd.command=RETURN_EXTENDED;
	      //testcmd.station=0;
	      //testcmd.data[0]=0;
	      //testcmd.data[1]=0;
	      //testcmd.data[2]=0;
	      //send_n_get(testcmd, &testrsp);
	      // Show extended data
	      //show_extended_data();
		   hitwd();

	      // send message for 3 seconds and show statistics
	      //testcmd.command=RETURN_INPUTS;
	      //for (i=0; i<3; i++)
	      //{
	      //   hitwd();
	      //   testtime=MS_TIMER;
	      //   while (MS_TIMER-testtime < 1000)
	      //   { msDelay(7);
	      //     send_n_get(testcmd, &testrsp);
	      //   }
	      //}
	      //show_comm_test(0); // don't wait
// THIS MAY BE CAUSING WATCHDOG RESET FOR JOE, MOVE TO AFTER ETHERNET INITIALIZATION
//         syncDateAndTime();  // put the clock out on the com bus
//         if (syncParametersToRemote())
//         {
//			   lcd_DispText("REMOTES NOT SYNC'D\n",0, 0, MODE_REV);
//            msDelay(800);
//         }
//         sendParametersToLogger(); // send to logger
//         if (slaveAvailable) setActiveMain(ON); // default to receive here
	   }
	   else
	   {
         // what to do for slave startup
	   	system_state=IDLE_STATE;
         STATION_SET = 0; // incoming commands will set this up
         setActiveMain(OFF);  // default to not receive here
	   }
		// setActiveMainMsg();  // enable or disable active main message
   }
   else
   {  // DON'T HAVE A WATCHDOG RESET INDICATION
      //lcd_print(SYSTEMLCD, 0, "  SYSTEM RESET   ");
      //lcd_print(SYSTEMLCD, 1, "WATCHDOG TIMEOUT ");
      //msDelay(3000);
   }

   //lcd_DispText("Setup interrupt handler\n",0, 0, MODE_NORMAL);
   // setup interrupt handler for arrival optics
#if __SEPARATE_INST_DATA__ && FAST_INTERRUPT
	interrupt_vector ext1_intvec my_isr1;
#else
	SetVectExtern3000(1, my_isr1);
   // re-setup ISR's to show example of retrieving ISR address using GetVectExtern3000
	//SetVectExtern3000(1, GetVectExtern3000(1));
#endif
   arrivalEnable(OFF); // disable interrupt and clear latch

#ifdef USE_TCPIP
	// setup default static IP using THIS_DEVICE (myIPaddress = MY_BASE_IP + THIS_DEVICE)
   lcd_DispText("Initialize Ethernet...\n",0, 0, MODE_NORMAL);
	sprintf(myIPaddress, MY_BASE_IP, param.systemNum);
   hitwd();
   //printf("My static IP address: %s\n", myIPaddress);
   // configure the interface
   //if(sock_init()) printf("IP sock_init() failed\n");	// Initialize UDP communications
	ifconfig(IF_ETH0,
				IFS_DHCP, 1,
				IFS_IPADDR,aton(myIPaddress),
				IFS_NETMASK,aton(MY_IP_NETMASK),
				IFS_DHCP_FALLBACK, 1,
	         IFS_UP,
	         IFS_END);
   // DON'T OPEN AND LEAVE OPEN THE UDP SOCKET, ONLY OPEN TO SEND THEN CLOSE IT
   //if(!udp_open(&sock, LOCAL_PORT, -1/*resolve(REMOTE_IP)*/, REMOTE_PORT, NULL))
   //{  printf("udp_open failed!\n");
   //   //lcd_print(1, 0, "No IP Communications");
   //   msDelay(1000);
   //}

   // wait a little till the interface is up
   testtime = MS_TIMER;
   hitwd();
   while (!ifstatus(IF_ETH0) && ((MS_TIMER - testtime) < 30000))
   {  tcp_tick(NULL);
   	msDelay(100);
      printf("-");
      lcd_DispText("-", 0, 0, MODE_NORMAL);
   }
   lcd_DispText("\n", 0, 0, MODE_NORMAL);
   if (!ifstatus(IF_ETH0))
   {  TCP_Active = 0;  // disable if timed out
	   lcd_DispText("ETHERNET NOT ACTIVE\n",0, 0, MODE_REV);
      msDelay(800);
   }
   hitwd();

   lcd_DispText("Opening UDP Socket...\n",0, 0, MODE_NORMAL);
   // NEED UDP SOCKET OPEN FOR RECEIVE
   if (param.slaveController)
   {  if(!udp_open(&CtrlSock, CTRL_PORT, -1/*resolve(REMOTE_IP)*/, CTRL_LOCAL_PORT, NULL))
      { printf("udp_open failed!\n"); }
   } else
   {  if(!udp_open(&CtrlSock, CTRL_LOCAL_PORT, -1/*resolve(REMOTE_IP)*/, CTRL_PORT, NULL))
      { printf("udp_open failed!\n"); }
   }
   // check remote configuration over UDP
   if (param.point2point == FALSE) UDPcheckRemoteConfiguration();

   // RS485 -> Ethernet stuff moved from above
   if (param.slaveController == 0)
   {
	   lcd_DispText("Synchronize Remotes...\n",0, 0, MODE_NORMAL);
      syncDateAndTime();  // put the clock out on the com bus
      if (syncParametersToRemote())
      {
         lcd_DispText("REMOTES NOT SYNC'D\n",0, 0, MODE_REV);
         msDelay(800);
      }
      lcd_DispText("Notify Logger...\n", 0, 0, MODE_NORMAL);
      sendParametersToLogger(); // send to logger
      if (slaveAvailable) setActiveMain(ON); // default to receive here
   }


	// setup timer to poll server
   //TCPCheckTimer = SEC_TIMER;

#endif
	// Setup standard display of color touchscreen
   lcd_drawScreen(1,lcd_WITH_BUTTONS);
   message_del(0, EMPTY, EMPTY);    // initialize both lcd queues


	/* BEGIN MAIN LOOP */
   if (param.slaveController==0)
   {
	   while(1)
	   {
	      maintenance();  // watchdog, led activity, UDP commands, inUse lights
	      //rn_keyProcess(DevRN1600, 0);  // process keypad device
	      showactivity();      // flashes the board led
	      show_comm_status();  // update lcd with comm status
         // process touch buttons before updating lcd display
         lcd_processTouchScreen(-2);  // get a new button press
	      message_show(NEXT);  // show next message in queue
	      processSystemIO();   // check system processes
			if (SEC_TIMER > lastHeartBeat)
         {	// Send a tcp heartbeat every x seconds
         	sendUDPHeartbeatToHost(tcpHeartBeat++);
            lastHeartBeat = SEC_TIMER + 4;
         }
         // control network heartbeat inside processSystemIO
	   }
   } else // SLAVE MAIN LOOP
   {
///		slaveEnableCommands();
		heartbeat_interval=100;
		while(1)
      {
	      maintenance();  // watchdog, led activity, UDP commands, inUse lights
	      //rn_keyProcess(DevRN1600, 0);  // process keypad device
	      showactivity();      // flashes the board led
///         slaveProcessIO();

	      if ((MS_TIMER - heartbeat_timer) > heartbeat_interval)
	      {  // Send a UDP heartbeat
///	         slaveSendUDPHeartbeat(1);
	         heartbeat_timer=MS_TIMER;
	      }

      }
   }
}
nodebug root interrupt void my_isr1()
{
	latchCarrierArrival=TRUE;
   // never deactivate the interrupt during the service routine
  	//WrPortI(I1CR, &I1CRShadow, 0x00);		// disble external INT1 on PE1
}
void arrivalEnable(char how)
{
   // should call this routine once to enable interrupt and once to disable

   latchCarrierArrival=FALSE;    // clear existing latch
   if ((how) && (param.inlineOptic))
   {   //outport(ITC,(inport(ITC))|0x02);      // enable  INT1
   	//WrPortI(I1CR, &I1CRShadow, 0x09);		// enable external INT1 on PE1, rising edge, priority 1
   	WrPortI(I1CR, &I1CRShadow, 0x0E);		// enable external INT1 on PE1, rising and falling edge, priority 2
   } else
   {   //outport(ITC,(inport(ITC))&~0x02);     // disable INT1
   	WrPortI(I1CR, &I1CRShadow, 0x00);		// disble external INT1 on PE1
   }
}

void initialize_system()
{  // Setup system i/o processing routine
   char i;

   if ( readParameterSettings() != 0)
   {  //lcd_print(1, 0, "ERROR READING PARAMS");
	   lcd_DispText("ERROR LOADING PARAMETERS\n",0, 0, MODE_NORMAL);
      //msDelay(800);
   }
   // data type mismatch, force two parameters to char
   param.deliveryTimeout &= 0xFF;
   param.autoPurgeTimer &= 0xFF;

   // make sure systemNum is valid
   if (param.systemNum > 20) param.systemNum=1;  // default to 1
   //param.version = VERS;
   //param.subversion = SUBVERS;

   // Set default for 2nd card range if not yet initialized
//   if (param.card2L3Digits == 255)
//   {  param.card2L3Digits = 155;
//      param.card2R9Min = 437355717;
//      param.card2R9Max = 437357715;
//   }

	// HARD CONFIG FOR POINT 2 POINT SINGLE CONTROLLER
   param.inlineOptic=FALSE;
   param.point2point=TRUE;
   param.localSubstation=TRUE;
   param.activeStations=1;
   availableDevices=0;
   ////////////////////////////////

   // setup other globals
   if (param.syncInterval > 0) pingTimer = SEC_TIMER + (60 * (int)param.syncInterval);  // timer for sync with server
   else                        pingTimer = 0xFFFFFFFF;  // Never
   statTrans=0;
   autoRtnTrans=0;
   secureTrans=0;
   slaveAvailable = FALSE;  // ASSUME NOT
	slaveReturnStatus = 0;
   btnStatFlag=0;
   btnSecureFlag=0;
   btnAReturnFlag=0;
   for (i=0; i<4; i++) { outputvalue[i]=0; }    // clear expansion bus output buffers
   arrival_alert=0;
   for (i=0; i<10; i++) { autoReturnTime[i]=0; arrivalTime[i]=0; }   // initialize auto-return, arrival timers
   main_arrival=0;
   arrival_from=0;
   systemStation=0;
   systemStationb=0;
   diverterDevices=0;
   //param.mainAudibleAlert=TRUE;  make sticky v4.20
   alarm_silenced_flag=FALSE;
   malfunctionActive=FALSE;
   echoLcdToTouchscreen=TRUE;
	latchCarrierArrival=FALSE;  // clear latch
   param.manPurgeTimeout = 5;  // fixed at 5 minutes
   reset_statistics();
   queueDel(0,0);            // initialize request queue
   // back door way to toggle screen orientation
   if (di_priorityRequest==1) param.orientLandscape = !param.orientLandscape;
   set_lcd_limits();

   for (i=0; i<NUMDATA; i++) slaveData[i]=0;  // init slave i/o

   // check headdiverter configuration and make sure it is valid
   if ((param.headDiverter != TRUE) && (param.headDiverter != FALSE)) param.headDiverter=FALSE;
   if (param.headDiverter) param.activeStations |= 0x80;  // station 8 programmed by headdiverter setting
   STATION_SET = param.activeStations;

   // check sub station addressing to make sure it is valid
   if (param.subStaAddressing != TRUE) param.subStaAddressing=FALSE;

   // make sure passwords are valid (4 digits, trailing null)
   for (i=0; i<4; i++)
   {
   	if ((param.adminPassword[i] < '0') || (param.adminPassword[i] > '9'))
      	param.adminPassword[0]=0; // reset pw
   	if ((param.maintPassword[i] < '0') || (param.maintPassword[i] > '9'))
      	param.maintPassword[0]=0; // reset pw
   	if ((param.cfgPassword[i] < '0') || (param.cfgPassword[i] > '9'))
      	strcpy(param.cfgPassword, "2321"); // reset pw
   }
   // make sure phone numbers are valid
   for (i=0; i<11; i++) { if (strlen(param.phoneNum[i]) > 9) param.phoneNum[i][0]=0; }

   // setup transaction count message
   setParValueMessage(MSG_TRANS_COUNT, transactionCount(), "");

   // init variables for secure transactions
   secureAck=0;
   for (i=0; i<10; i++)
   {  UID_Clear(secureTransLog[i].sid);
   	secureTransLog[i].start_tm=0;
      secureTransLog[i].scanType=0;
      securePIN[i]=-1;
   }
   param.UIDRecNo = 0;   // initially no limit on UID record paging
   Param_Flash_Write(1); // reset timer
   UID_GetAddOnly = 0;   // ok to accept deletes
   // make sure server IP is valid otherwise reset to blank
   if (param.serverID[0] > 0)
   {  // 1st digit must be valid range "0..9"
      if (param.serverID[0] < '0' || param.serverID[0] > '9') param.serverID[0]=0;
   }
}

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
		print_comstats();
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

char remote_data[6];
// event log is modified in multiple modules so make global
struct trans_log_type eventlog;
void processSystemIO()
{
   struct iomessage command, response, xtended;
	static struct iomessage finalcmd;
   struct UDPmessageType UDPmessage;
   char received, k, kbit, XtraCic, FirstXtraCic;
   static char new_state;
   static char systemStartup;
   char func, menu_item, func_mode;
   static char first_time_here;
   static char lcd_is_bright;
   char tempData;
   //char remote_data[4];
   static struct trans_log_type translog;
//   int safetyStatus;
   static unsigned long auto_purge_timer;
   static unsigned long lastComTimer;
   static int transProgress;
   static unsigned long doorOpenTimer;  // How long door is open during a transaction in seconds

   //char key;
   char mainFlashAtSlave;
	char prev_arrival_alert;
   // setup structure for handling transaction types and info lookup
   enum TRANTYPES
   { 	TT_NONE = 0,
      TT_MAIN2REMOTE,
      TT_REMOTE2MAIN,
      TT_SLAVE2REMOTE,
      TT_REMOTE2SLAVE,
      TT_MAIN2SLAVE,
      TT_SLAVE2MAIN
   };
   enum TRANSINFOS
   {  TT_MAINSTATION = 0,
   	TT_MAIN2MAIN_TRANS,
      TT_SYSTEM_DIRECTION,
      TT_FROM,					// For head diverter alignment
      TT_TO                // For head diverter alignment
   };
   enum TRANSLOCS
   {  TT_MAIN = 1,
   	TT_REMOTE,
      TT_SLAVE
   };
	static char transactionType;
   // create lookup table for transaction parameters
   static const char transTypeInfo[7][5] = {  // mainStation, main2main_trans, system_direction, from, to
   0, 0, 0,	0, 0,							// NONE
   MASTER, 0, DIR_SEND, TT_MAIN, TT_REMOTE,				// MAIN TO REMOTE
   MASTER, 0, DIR_RETURN, TT_REMOTE, TT_MAIN,			// REMOTE TO MAIN
   SLAVE,  0, DIR_SEND, TT_SLAVE, TT_REMOTE,				// SLAVE TO REMOTE
   SLAVE,  0, DIR_RETURN, TT_REMOTE, TT_SLAVE,			// REMOTE TO SLAVE
   MASTER, DIR_SEND, DIR_SEND, TT_MAIN, TT_SLAVE,		// MAIN TO SLAVE
   SLAVE,  DIR_RETURN, DIR_SEND, TT_SLAVE, TT_MAIN };	// SLAVE TO MAIN

   #GLOBAL_INIT
   {
	   first_time_here = TRUE;
	   system_state=CANCEL_STATE;
      new_state=system_state;
	   state_timer=MS_TIMER;
      lastComTimer=0;
      transProgress=101;  // handle CIC only;  use 999 to clear the area
      systemStartup=TRUE;  // resets after getting through FINAL_COMMAND
      lcd_is_bright=TRUE;
      remote_data[REMOTE_CIC]=0;  // clear out because slave will use it and it may not be initialized
      remote_data[REMOTE_DIVERTER]=0; // in case no diverters
      remote_data[REMOTE_RTS]=0;
      remote_data[REMOTE_ARRIVE]=0;
	  	//heartbeat_interval=500;
      //heartbeat_timer=0;
	}

   // First process items always needing attention
   check_diverter();
   // processBlower(); NOW HANDLED IN maintenance()
   processCycleTime();
   //processMainArrivalAlert(); NOW HANDLED IN maintenance()
   // check for secure removals
   checkSecureRemoval();
   if (secureAck)
   {  // send message to remotes to acknowledge secure id received
	   command.devAddr=ALL_DEVICES;
	   command.command=ACK_SECURE_REMOVAL;
      command.data[0]=secureAck;
      if (send_command(command)) secureAck = 0;
   }
   // send periodic heartbeat on UDP control network
   checkNsendHeartbeat(1, translog.flags);
//   if ((MS_TIMER - heartbeat_timer) > heartbeat_interval)
//   {  // Send a UDP heartbeat
//      sendUDPHeartbeatToCtrl(1, translog.flags);
//      heartbeat_timer=MS_TIMER;
//      monitorUDPactivity(0);  // force a refresh of station activity in case all are off
//   }


   lcd_ShowTransactionProgress(mainStation, systemStation, system_direction, transProgress, SEC_TIMER-translog.start_tm);
   //key = getKey(); // processKeyInput();

   // Handle STAT processing
   processStatPriority();


   // if main door open, add message, else del.
   if ( di_doorClosed )
   {  message_del(TRANSLCD, MSG_DROPEN, MSG_AT); }
   else
   {  message_add(TRANSLCD, MSG_DROPEN, MSG_AT, NEXT); }

   // show carrier in chamber
   if ( di_carrierInChamber )
   {
      //message_add(SYSTEMLCD, MSG_CIC, MSG_AT, NEXT);
      message_add(TRANSLCD, MSG_CIC, MSG_AT, NEXT);
      // and check to flash light 8 at slave if main has CIC
      if (param.stacking_ok[0]) mainFlashAtSlave=0; else mainFlashAtSlave=SLAVE_b;
   }
   else
   {
      //message_del(SYSTEMLCD, MSG_CIC, MSG_AT);
      message_del(TRANSLCD, MSG_CIC, MSG_AT);
      message_del(TRANSLCD, MSG_CANT_SEND, MSG_CANT_WHY); // no CIC, no reason
      mainFlashAtSlave=0;
   }
   // Also handle CIC and Door messages in the TRANSLCD
   // IF YOU DELETE NEXT LINE THEN REMOVE THE FUNCTION
   // update_cic_door_msgs();

   // show if arrival optic is blocked
   if ( di_carrierArrival && (system_state==IDLE_STATE) )
      message_add(SYSTEMLCD, MSG_OPTIC_BLOCKED, MSG_AT, NEXT);
   else
      message_del(SYSTEMLCD, MSG_OPTIC_BLOCKED, MSG_AT);

// DON'T RUN COMMUNICATIONS ANY MORE THAN THE BUS CAN HANDLE
if (1) // DO NOT RESTRICT TCP/UDP (MS_TIMER - lastComTimer > 10)
{  lastComTimer = MS_TIMER;

	// First things first, process UDP control commands
   if (getUDPcommand(&UDPmessage) > 0) processUDPcommand(UDPmessage);

   // Check some remote inputs ... sends some data also
   command.devAddr=ALL_DEVICES;
   command.command=RETURN_INPUTS;
   if (param.subStaAddressing) command.station=mainStation;
   else  command.station=param.activeMain;
   // send the main arrival state to indicate not ready for transaction
   if (main_arrival || slave_arrival) command.data[0]=arrival_from;
   else command.data[0]=0;
   // send the system_state (for headdiverter controller to operate blower)
   command.data[1]=system_state;
   command.data[2]=outputvalue[devInUseSolid] & ~mainFlashAtSlave;		// was system_direction;
   command.data[3]=(outputvalue[devInUseFlash] & ~SLAVE_b) | mainFlashAtSlave;
   command.data[4]=STATION_SET;

   //if (send_n_get(command, &response))
   //{
      //if (response.command==INPUTS_ARE)
      //{
        // store remote response in local var
        //for (k=0; k<5; k++)
        //{  tempData = response.data[k];
        //   tempData &= response.station;  // Turn off unused bits
        //   remote_data[k] |=  (tempData & response.station);  // ON Bits
        //   remote_data[k] &= ~(tempData ^ response.station); // OFF bits
        //}
		  // capture remote_data I/O when controlling a remote station with local I/O
        if (param.localSubstation==TRUE)
        {  //
        	  remote_data[REMOTE_RTS] = di_requestToSend2;
           remote_data[REMOTE_CIC] = di_carrierInChamber2;
           remote_data[REMOTE_DOOR] = di_doorClosed2;
			  // remote pause/suspend input?
        }

        // force unused doors closed (on) and other unused inputs off
        remote_data[REMOTE_DOOR] |= ~STATION_SET;
        remote_data[REMOTE_CIC] &= STATION_SET;
        remote_data[REMOTE_RTS] &= STATION_SET;
        remote_data[REMOTE_ARRIVE] &= STATION_SET;
		  if (param.subStaAddressing && slaveAvailable) remote_data[REMOTE_RTS2] &= STATION_SET;
        else remote_data[REMOTE_RTS2] = 0;

	      // store slave response in local vars
	      if (slaveAvailable)
	      {  //if (slaveData[0] & 0x01)      // slave carrier in chamber
	         //{   slave_cic = TRUE;
            //    message_add(SYSTEMLCD, MSG_CIC, MSG_AT+SLAVE, NEXT);
	         //} else
	         //{  slave_cic=FALSE;
	         //   message_del(SYSTEMLCD, MSG_CIC, MSG_AT+SLAVE);
            //}
	         //if (slaveData[0] & 0x02)      // slave door
	         //{   slave_doorClosed = TRUE;  // closed
	         //    remote_data[REMOTE_DOOR] |= SLAVE_b;
            //    message_del(SYSTEMLCD, MSG_DROPEN, MSG_AT+SLAVE);
	         //} else
	         //{   slave_doorClosed = FALSE;
            //    message_add(SYSTEMLCD, MSG_DROPEN, MSG_AT+SLAVE, NEXT);
            //}
            //if (slaveData[0] & 0x04)		// show message if slave optic blocked
				//{  message_add(SYSTEMLCD, MSG_OPTIC_BLOCKED, MSG_AT+SLAVE, NEXT);
            //} else
            //{  message_del(SYSTEMLCD, MSG_OPTIC_BLOCKED, MSG_AT+SLAVE);
            //}
            //if (slaveData[0] & 0x08) btnStatFlag=1;  // Set stat from slave
            //if (slaveData[0] & 0x10) btnStatFlag=2;  // clear stat from slave
            //if (slaveData[0] & 0x20) btnSecureFlag=1;  // Set secure from slave
            //if (slaveData[0] & 0x40) btnSecureFlag=2;  // clear secure from slave
	         //slave_rts = slaveData[1];     // slave request to send
	         //slaveReturnStatus = slaveData[2];  // slave status

	         // check status change request
	         if (system_state==IDLE_STATE || system_state==MALFUNCTION_STATE)
	         {  //if (slaveReturnStatus & 0x02) param.activeMain=SLAVE;
	            //if (slaveReturnStatus & 0x04) param.activeMain=MASTER;
               //if (slaveReturnStatus & 0x06)   // active main changed
               //{  // logActiveChange(8);
               //   setActiveMainMsg();
               //   activeMainHere = (param.activeMain==MASTER);  // FOR SETUP MENU
               param.activeMain = MASTER;  // FORCE ALWAYS
               activeMainHere = TRUE;		 // FORCE ALWAYS
               //}
	            ////if (slaveReturnStatus & 0x08) deliverComControl(&system_state);
               if (slaveReturnStatus & 0x20) system_state=CANCEL_STATE;
               if (slaveReturnStatus & 0x40) slave_rtncarrier=TRUE; else slave_rtncarrier=FALSE;
               if (slaveReturnStatus & 0x80) slave_autoReturn=TRUE; else slave_autoReturn=FALSE;
	         }

	         // flash inuse when cic and not stacking
	         if (slave_cic && !param.stacking_ok[0])
            {
	            // flash those with carriers unless stacking is allowed
	            if (malfunctionActive) inUse(FLASH, SLAVE_b);
	            else inUse(FLASH, SLAVE_b & ~systemStationb);
	         } else
            {  // no carrier or stacking OK so don't flash unless system station
            	if (systemStationb != SLAVE_b) inUse(OFF, SLAVE_b);
            }

	         // check for remote to cancel transaction
	         if (slaveReturnStatus & 0x01 && (mainStation==SLAVE))
	            system_state=CANCEL_STATE;    // cancel immediately

	      } else slave_doorClosed = TRUE;  // need to close unused door


        // flash inuse when cic and not stacking
        if (remote_data[REMOTE_CIC] && !param.stacking_ok[1])
        {
          // flash those with carriers unless stacking is allowed
          if (malfunctionActive) inUse(FLASH, remote_data[REMOTE_CIC]);
          else inUse(FLASH, remote_data[REMOTE_CIC] & ~systemStationb);
        }
        // turn others off except system station
        if (malfunctionActive) inUse(OFF, ~(remote_data[REMOTE_CIC]|SLAVE_b));
        else inUse(OFF, ~remote_data[REMOTE_CIC] & ~(systemStationb|SLAVE_b));

        // for each with the door open and no cic, clear the arrival-alert and auto-return timer
        if (arrival_alert)
        {  prev_arrival_alert = arrival_alert;  // save to see what has changed
           arrival_alert &= (remote_data[REMOTE_CIC]
                         | remote_data[REMOTE_DOOR])
                         & ~systemStationb;
           // for each zero bit clear the auto-return timer, secure log, and PIN message
           for (k=1; k<=LAST_STATION; k++)
           {  kbit=station2bit(k);      // setup bit equivalent
	           // if arrived more than 15 seconds ago and no CIC then clear alert
	           if (((SEC_TIMER-arrivalTime[k]) > 15) && ((remote_data[REMOTE_CIC] & kbit)==0))
	           {  // then clear only based on CIC without regard for the door
	              arrival_alert &= ~kbit;
	           }
              // deal with reset of arrival alert(s)
	           if ((arrival_alert & kbit)==0)
           	  {  // is this a change in state
                 if (prev_arrival_alert & kbit) securePIN[k]=-1; // clear the pin
                 // reset auto return timer
				     autoReturnTime[k]=0;
                 arrivalTime[k]=0;
                 //secureTransLog[k].sid=0;
                 //secureTransLog[k].start_tm=0;
   		        message_del(SYSTEMLCD, MSG_PIN, MSG_AT+k);  // delete PIN message
              }
           }
        }

        // clear slave arrival alert as necessary
        if (slave_arrival)
        {  if ( (slave_doorClosed==FALSE && slave_cic==FALSE)
               || ((mainStation==SLAVE) && (system_direction==DIR_SEND)) )
           {  slave_arrival=0;
		        message_del(SYSTEMLCD, MSG_ARVALRT, MSG_AT+SLAVE);
		        message_del(SYSTEMLCD, MSG_PIN, MSG_AT+SLAVE);
              arrival_from=0;
              autoReturnTime[SLAVE]=0;
           }
        }

// FOR EACH STATION WITH ACK SECURE, LOG THE BADGE ID AND REMOVE PIN MESSAGE
//      message_add(SYSTEMLCD, MSG_PIN, MSG_AT+systemStation);

       // for each station, check several conditions.
       for (k=1; k<=LAST_STATION; k++)
       {
          kbit=station2bit(k);      // setup bit equivalent

          // if station not reporting and it should, show a message
         /// if (((kbit & response.station)==0) && (kbit & param.activeStations))
         ///      message_add(SYSTEMLCD, MSG_NOCOMM, MSG_AT+k, NEXT);
         /// else message_del(SYSTEMLCD, MSG_NOCOMM, MSG_AT+k);

          // for each with a door open, add door message, else del.
          if (remote_data[REMOTE_DOOR] & kbit)
             	message_del(SYSTEMLCD, MSG_DROPEN, MSG_AT+k);
          else message_add(SYSTEMLCD, MSG_DROPEN, MSG_AT+k, NEXT);

          // for each with a carrier, add carrier message ... else del.
          if (remote_data[REMOTE_CIC] & kbit)
             	message_add(SYSTEMLCD, MSG_CIC, MSG_AT+k, NEXT);
          else message_del(SYSTEMLCD, MSG_CIC, MSG_AT+k);

          // for each with a request-to-send, add pending message, else del.
          if ((remote_data[REMOTE_RTS] & kbit) || (remote_data[REMOTE_RTS2] & kbit))
             	message_add(SYSTEMLCD, MSG_PENDING, MSG_FROM+k, NEXT);
          else message_del(SYSTEMLCD, MSG_PENDING, MSG_FROM+k);

          // for each with arrival-alert, add warning, else del.
          if (arrival_alert & kbit)
             	message_add(SYSTEMLCD, MSG_ARVALRT, MSG_AT+k, NEXT);
          else
          {    message_del(SYSTEMLCD, MSG_ARVALRT, MSG_AT+k);
          }

          // for each with blocked arrival optic add message
          if (remote_data[REMOTE_ARRIVE] & kbit)
             	message_add(SYSTEMLCD, MSG_OPTIC_BLOCKED, MSG_AT+k, NEXT);
          else message_del(SYSTEMLCD, MSG_OPTIC_BLOCKED, MSG_AT+k);

        }
        // check for head diverter blocked optic
        if (param.headDiverter)
        {  if (remote_data[REMOTE_ARRIVE] & 0x80)  // HARD CODE TO BIT 8
	            message_add(SYSTEMLCD, MSG_OPTIC_BLOCKED, MSG_AT_HEADDIV, NEXT);
	        else message_del(SYSTEMLCD, MSG_OPTIC_BLOCKED, MSG_AT_HEADDIV);
        }
      //} end if inputs-are
   //} end if send-n-get
} // end if DON'T RUN COMMUNICATIONS ANY MORE THAN THE BUS CAN HANDLE

   // if system_reset (Abort) and state is not normal|malfunc, cancel
   // DON'T HAVE A GOOD WAY TO DO THIS AT THE MOMENT
   //while (  functionButton(FUNC_BUTT, FUNC_CURR)
   //   && functionButton(FUNC_F1, FUNC_CURR)
   //   && functionButton(FUNC_F2, FUNC_CURR)
   //   && (system_state != IDLE_STATE)
   //   && (system_state != MALFUNCTION_STATE)) system_state=CANCEL_STATE;


   // LAST REMOTE COMMAND BEFORE PROCESSING MUST BE "RETURN_INPUTS"


   switch (system_state)
   {
      case IDLE_STATE:
         //lcd_screenSaver(LCD_DIM);  // handle backlight screen saver
         transactionType = TT_NONE;  // make sure we start with no request
         destMain = MASTER;
	      transProgress=101;  // no activity
         doorOpenTimer=0;
         translog.flags = 0;
			translog.status = 0;
         eventlog.flags = 0;
         eventlog.status = 0;

	      // Check carrier return function
	      if (di_returnCarrier || lcd_returnCarrier())
	      {  // determine which carrier to return
            systemStationb = (di_requestToSend | station2bit(lcd_sendTo())) & STATION_SET;
	         systemStation=bit2station(systemStationb);
         	if ( di_doorClosed
	              && (   ( systemStation!=SLAVE && (systemStationb & remote_data[REMOTE_CIC] & remote_data[REMOTE_DOOR]))
                     || ( systemStation==SLAVE && slave_doorClosed && slave_cic) ) )
            {
               translog.flags = FLAG_CRETURN;
	            UID_Clear(secureTransLog[systemStation].sid);      // clear secure handling
	            secureTransLog[systemStation].start_tm=0;
	            if ((systemStation==SLAVE) && (slave_cic))
	            {  transactionType = TT_REMOTE2MAIN; // FORCE TT_SLAVE2MAIN;
	            } else if ( (systemStation!=SLAVE) &&
	                  ( systemStationb & remote_data[REMOTE_CIC] & remote_data[REMOTE_DOOR]) )
	            {  transactionType = TT_REMOTE2MAIN;
	            }
	         }
	      }
	      // check manual blower control functions
         if (di_manVacuum && !di_manPressure && di_doorClosed && di_doorClosed2 && ((MS_TIMER-state_timer) > 2000))
         {  message_add(SYSTEMLCD, MSG_MAN_VACUUM, MSG_BLANK, NEXT);
         	new_state=MANUAL_VACUUM; //blower(blwrVAC);
         }
         else if (di_manPressure && !di_manVacuum && di_doorClosed && di_doorClosed2 && ((MS_TIMER-state_timer) > 2000))
         {  message_add(SYSTEMLCD, MSG_MAN_PRESSURE, MSG_BLANK, NEXT);
         	new_state=MANUAL_PRESSURE; //blower(blwrPRS);
         }
         //else
         //{  //blower(blwrOFF); }

	      // check remote requests next
	      else if (!statTrans && (remote_data[REMOTE_RTS] & STATION_SET))
	      { // which active main?
	         if (param.activeMain==SLAVE && slaveAvailable)
	         {  // send to slave
	            if ( ((slave_cic==FALSE && slave_arrival==0) || param.stacking_ok[0]) && (slave_doorClosed==TRUE) )
	            {  transactionType = TT_REMOTE2SLAVE;
	               systemStation=firstBit(remote_data[REMOTE_RTS]);
 	            }
	         } else    // active_main == MASTER
	         {  // only if (no carrier and no arrival_alert) or (stacking is ok)
	            if ( (di_carrierInChamber==FALSE && main_arrival==0) || param.stacking_ok[0])
	            {  // OK if main door is closed otherwise signal alert
	               if (di_doorClosed)
	               {  transactionType = TT_REMOTE2MAIN;
	                  systemStation=firstBit(remote_data[REMOTE_RTS]);
	               } else {  setAlerts(ON); }
	            }
            }
         }  // check for sub station addressing
         else if (!statTrans && (remote_data[REMOTE_RTS2] & STATION_SET))
	      { // can only be to slave, make sure parameter is set
	         if (param.subStaAddressing && slaveAvailable)
	         {  // send to slave
	            if ( ((slave_cic==FALSE && slave_arrival==0) || param.stacking_ok[0]) && (slave_doorClosed==TRUE) )
	            {  transactionType = TT_REMOTE2SLAVE;
	               systemStation=firstBit(remote_data[REMOTE_RTS2]);
 	            }
	         }
         }
			// Check auto-return function
         else if (autoReturnCarrier())
         {
            systemStationb = autoReturnCarrier();
            // doors & cic already checked
            systemStation=bit2station(systemStationb);
	         UID_Clear(secureTransLog[systemStation].sid);		// clear secure handling
	         secureTransLog[systemStation].start_tm=0;
            translog.flags = FLAG_ARETURNING;
            if (systemStation==SLAVE)
            {  transactionType = TT_REMOTE2MAIN; // FORCE TT_SLAVE2MAIN;TT_SLAVE2MAIN;
            } else
            {  transactionType = TT_REMOTE2MAIN;
            }
         }
	      else if (SEC_TIMER > pingTimer)  // check with server
	      {  // update next check time then check server
            pingTimer = SEC_TIMER + (60 * (int)param.syncInterval);
	         tcpServerCheck();
	      }
         else
         {  // check main request
            systemStationb = (di_requestToSend | station2bit(lcd_sendTo())) & STATION_SET;
            // proceed if good send request, no carrier or stacking ok except when secure
            // OR LAST UNREMOVED WAS SECURE secureTransLog[i].start_tm <> 0
            // OR LAST UNREMOVED IS AUTO RETURN autoReturnTime[station] <> 0
            //   && ( !(remote_data[REMOTE_CIC] & systemStationb) || (param.stacking_ok[1] && !secureTrans)))
            if (valid_send_request(systemStationb)==0)
            {  systemStation=bit2station(systemStationb);
            	// count how many are hard button versus soft button
//               if (di_requestToSend) {rtsHardCount++;} else {rtsSoftCount++;}
	            // check for main to main transaction
	            //if (systemStation==SLAVE) { transactionType = TT_MAIN2SLAVE; }
	            //else							  { transactionType = TT_MAIN2REMOTE; }
               transactionType = TT_MAIN2REMOTE;
               // Set the auto-return flag if requested
/// why??      if (lcd_autoReturn() || autoRtnTrans) translog.flags = FLAG_ARETURN;
            }
         }
			// do we have a transaction to process?
         if (transactionType != TT_NONE)
         {  // setup to process a transaction request
            mainStation = transTypeInfo[transactionType][TT_MAINSTATION];
            main2main_trans = transTypeInfo[transactionType][TT_MAIN2MAIN_TRANS];
            system_direction = transTypeInfo[transactionType][TT_SYSTEM_DIRECTION];
            systemStationb = station2bit(systemStation);
            inUse(FLASH, systemStationb);
            new_state=PREPARE_SEND;
            arrivalEnable(OFF); // make sure we clear every time
            if ((transactionType == TT_MAIN2SLAVE) || (transactionType == TT_REMOTE2SLAVE)) destMain=SLAVE;
            // leaving this s

         }
         else // check for directory display
         {  k=di_requestToSend;
            if (k & STATION_SET)
            {  systemStationb=k;
               systemStation=bit2station(systemStationb);
               if (systemStation)
                  message_add(TRANSLCD, MSG_REMOTESTA, MSG_AT+systemStation, ONESHOT);
            }
            // nothing else to do here so reset systemStation
            systemStation=0;
            systemStationb=0;
         }

         FirstXtraCic=TRUE;   // Used in HOLD_TRANSACTION
         // when leaving this state blank out the menu buttons and reset screensaver
         if (new_state != system_state)
         {  lcd_drawScreen(1,lcd_NO_BUTTONS);
				//lcd_screenSaver(LCD_BRIGHT);
         }
         break;


      case PREPARE_SEND:      // does job for send and return
	      transProgress=10;  	// % progress
         // turn off priority if it was on
         // priority = FALSE; TURN OFF LATER (v2.47)
         message_add(SYSTEMLCD, MSG_SYSBUSY, MSG_DONTUSE, NEXT);
         message_del(SYSTEMLCD, MSG_STAT, MSG_STAT+1);
         setAlerts(OFF);
         do_priorityLight(OFF);

         if (param.point2point==FALSE)
	         message_add(TRANSLCD, MSG_SETTING, MSG_TO+systemStation, NOW);

         // save transaction data
         translog.trans_num = transactionCount()+1;
         eventlog.trans_num = translog.trans_num;
         translog.start_tm = SEC_TIMER;
         translog.duration = 0;
         translog.source_sta=mainStation;
         translog.dest_sta=mainStation;
         if (system_direction==DIR_SEND)
         {  translog.dest_sta=systemStation;
            // some things only when sending
	         if (autoRtnTrans) translog.flags |= FLAG_ARETURN;
	         if (statTrans) translog.flags |= FLAG_STAT; //0x10;
	         if (secureTrans)
	         {  translog.flags |= FLAG_SECURE; //0x20
               // setup secure transaction log
               secureTransLog[systemStation].trans_num = translog.trans_num;
               secureTransLog[systemStation].start_tm = translog.start_tm; // will add delta-t's later
               UID_Clear(secureTransLog[systemStation].sid);   // will come later from the card swipe
               secureTransLog[systemStation].status = ESTS_SECURE_REMOVAL;
               secureTransLog[systemStation].flags = 0; // station will be set later systemStation;
	            // setup the secure pin
               securePIN[systemStation] = securePIN[0];  // move into position
               securePIN[0]=-1;

               // update the messages on the screen
               message_add(SYSTEMLCD, MSG_PIN, MSG_AT+systemStation, NEXT);
               message_del(TRANSLCD, MSG_STAT, MSG_PIN);

	         }
            else securePIN[systemStation]=-1;  // otherwise no PIN
         }
         else translog.source_sta=systemStation;
         if (transactionType == TT_SLAVE2MAIN) translog.dest_sta=MASTER;

         // tell all system(s) to get ready (set diverters)
         set_diverter(systemStation);
         command.devAddr=ALL_DEVICES;
         command.command=SET_DIVERTER;
         command.station=systemStation;
			command.data[0]=transTypeInfo[transactionType][TT_FROM];  // For head diverter
			command.data[1]=mainStation;  // For slave
         command.data[2]=param.subStaAddressing;
         command.data[3]=param.blowerType;
         //command.data[4]=0xFF;  // to help ident main messages
         command.data[4]=translog.flags;
         received=send_command(command);
         //if (received) new_state=WAIT_FOR_DIVERTERS;
         if (1) new_state=WAIT_FOR_DIVERTERS;
         else new_state=CANCEL_STATE;
         break;

      case WAIT_FOR_DIVERTERS:   // does job for send and return
	      transProgress=20;  	// % progress
         // Ready?
	      if (divertersReady(systemStation, transTypeInfo[transactionType][TT_FROM], mainStation)
             && blowerReady())
	      {
           new_state=BEGIN_SEND;    // serves both directions
           if ((system_direction==DIR_SEND) && (main2main_trans==0))
           {  message_add(TRANSLCD, MSG_INROUTE, MSG_TO+systemStation, NOW);
           }else if (system_direction==DIR_RETURN && main2main_trans==0)
           {  message_add(TRANSLCD, MSG_INROUTE, MSG_FROM+systemStation, NOW);
           }else if (main2main_trans==DIR_SEND)
           {  message_add(TRANSLCD, MSG_INROUTE, MSG_TO+SLAVE, NOW);
           } else
           {  message_add(TRANSLCD, MSG_INROUTE, MSG_TO, NOW);
           }
           message_del(TRANSLCD, MSG_SETTING, MSG_TO+systemStation);
	      }

	      // only wait for a little time
	      if ((MS_TIMER-state_timer >= DIVERTER_TIMEOUT) || (blwrError==TRUE))
	      {  // timeout
	      	new_state=MALFUNCTION_STATE;
           	if (blowerReady()==TRUE)
            {   // diverter timeout
	            statistics.divert_alarm++;
	            translog.status=STS_DIVERTER_TOUT; translog.duration=(int)(SEC_TIMER-translog.start_tm);
	            message_add(TRANSLCD, MSG_TIMEOUT, MSG_SETTING, NEXT);
            } else
            {   // blower timeout
	            statistics.divert_alarm++;
	            translog.status=STS_BLOWER_TOUT; translog.duration=(int)(SEC_TIMER-translog.start_tm);
	            message_add(TRANSLCD, MSG_TIMEOUT, MSG_BLOWER, NEXT);
            }
	      }

         // while waiting, if different request to send, cancel.
         if (di_requestToSend
            && (systemStationb != di_requestToSend)
            && (system_direction==DIR_SEND))
         {  // Cancel the transaction
	         translog.status=STS_CANCEL_BY_SEND; translog.duration=(int)(SEC_TIMER-translog.start_tm);
	         //print_transaction( translog );
	         addTransaction( translog , 0);
            //msDelay(1000); // wait so input doesn't trigger another trans right away
	         new_state=CANCEL_STATE;
         }
         break;

      case BEGIN_SEND:          // works for both directions
	      transProgress=30;  	// % progress
         // remote should be ready now, but set timeout just in case
         if (MS_TIMER-state_timer >= 2000)
         {  new_state=MALFUNCTION_STATE;   // Remote not ready or cancel dispatch
            translog.status=STS_TRANS_CANCEL; translog.duration=(int)(SEC_TIMER-translog.start_tm);
         }

         // before sending start command double check for cic/stacking
         command.data[0]=FALSE;
	      if (system_direction==DIR_SEND && main2main_trans==0)
	      { // main to remote -- check remote carrier or remote stacking
	         if ( !(remote_data[REMOTE_CIC] & systemStationb) || param.stacking_ok[1] ) command.data[0]=TRUE; // ok to continue
         } else if (system_direction==DIR_SEND && main2main_trans==DIR_SEND)
         {  // main to slave
		      if ( param.stacking_ok[0] || !carrierInChamber(SLAVE)) command.data[0]=TRUE;  // ok to continue
         } else if (system_direction==DIR_SEND && main2main_trans==DIR_RETURN)
         {  // slave to main
		      if ( param.stacking_ok[0] || !carrierInChamber(MASTER)) command.data[0]=TRUE;  // ok to continue
	      } else
         { // all other cases check main carrier or main stacking (master or slave)
		      if ( param.stacking_ok[0] || !carrierInChamber(mainStation)) command.data[0]=TRUE;  // ok to continue
	      }

         // are we ok to send?
         if (command.data[0] == TRUE)
         {
            // make sure remotes are ready
            command.devAddr=ALL_DEVICES;
            command.command=ARE_YOU_READY;
            command.station=systemStation;
            command.data[0]=system_direction;
            command.data[1]=mainStation;
            command.data[2]=main2main_trans;
            //command.data[3]=translog.flags;

            if (1) //(send_n_get(command, &response))
            {
               // if ((response.data[0] & STATION_SET) == STATION_SET)  // all stations ok?
               if (1) //(response.data[0] == availableDevices)  // all stations ok?
               {
                  // turn on blower
                  inUse(ON, systemStationb);
                  message_del(TRANSLCD, MSG_REMOTESTA, MSG_NOT_READY);
                  if (system_direction==DIR_SEND)
                  {  if (carrierInChamber(mainStation))
                     {  //blower(blowerSend);
                        new_state=WAIT_FOR_MAIN_DEPART;
                     }else
                     {  print_event("TRANSACTION CANCELED - CARRIER REMOVED");
				            translog.status=STS_CANCEL_BY_CIC; translog.duration=(int)(SEC_TIMER-translog.start_tm);
                        //print_transaction( translog );
				            addTransaction( translog, 0 );
                        new_state=CANCEL_STATE;
                     }
                  }else
                  {  if (remote_data[REMOTE_CIC] & systemStationb)
                     {  //blower(blowerReturn);
                        new_state=WAIT_FOR_REM_DEPART;
                     }else
                     {  print_event("TRANSACTION CANCELED - CARRIER REMOVED");
				            translog.status=STS_CANCEL_BY_REMOTE; translog.duration=(int)(SEC_TIMER-translog.start_tm);
                        //print_transaction( translog );
				            addTransaction( translog, 0 );
                        new_state=CANCEL_STATE;
                     }
                  }
               } else
               {  // remote door?
                  message_add(TRANSLCD, MSG_REMOTESTA, MSG_NOT_READY, NOW);
                  print_event("STATION(S) NOT READY FOR DISPATCH");
				      translog.status=STS_CANCEL_BY_REMOTE; translog.duration=(int)(SEC_TIMER-translog.start_tm);
                  //print_transaction( translog );
                  addTransaction( translog, 0 );
                  new_state=CANCEL_STATE;
               }
            }
         } else
         {  new_state=CANCEL_STATE;  // no stacking allowed
	         translog.status=STS_CANCEL_BY_STACK; translog.duration=(int)(SEC_TIMER-translog.start_tm);
	         //print_transaction( translog );
	         addTransaction( translog, 0 );
         }
         break;

      case WAIT_FOR_MAIN_DEPART:
         // check for doors opening
         new_state=checkDoorWhileRunning(system_state);

         // wait for carrier to exit
         if (carrierInChamber(mainStation))
         {
            if (MS_TIMER-state_timer >= CIC_EXIT_TIMEOUT)
            {  // been waiting too long, cancel
               statistics.cic_lift_alarm++;
               translog.status=STS_DEPART_TOUT; translog.duration=(int)(SEC_TIMER-translog.start_tm);
               //print_malfunction( translog );
               addTransaction( translog, 0 );
            	if (blwrError==TRUE)
               {  // blower timeout error
	               new_state=MALFUNCTION_STATE;
               } else
               {  // just cancel
	               new_state=CANCEL_STATE;
               }
            } // else, keep waiting
         } else
         {
		      transProgress=40;  	// % progress
            if (param.headDiverter==TRUE) new_state=WAIT_FOR_HEADDIVERTER;
            else
            {  new_state=WAIT_FOR_REM_ARRIVE;
               transProgress=60;  	// % progress
            }
         }

         break;

      case WAIT_FOR_REM_ARRIVE:
       // keep polling remote status until received, or timeout
       //if (remote_data[REMOTE_ARRIVE] & 0x7F) // at any station 1-7 (exclude headdiverter)
       if ((remote_data[REMOTE_ARRIVE] & systemStationb) // at THE station
            || (param.inlineOptic==FALSE && di_carrierInChamber2)) // or CIC when no inline optic present
       {  // handle arrival, transaction complete
          inUse(OFF, systemStationb);
          message_del(TRANSLCD, MSG_INROUTE, MSG_TO+systemStation);
          message_del(SYSTEMLCD, MSG_SYSBUSY, MSG_DONTUSE);
          arrival_alert |= systemStationb;        // set arrival
          arrivalTime[systemStation] = SEC_TIMER; // set arrival time
          // shall we set the auto return timer?
          if (translog.flags & FLAG_ARETURN) autoReturnTime[systemStation] = SEC_TIMER;

          // print transaction data
          statistics.trans_out++;
          translog.duration = (int)(SEC_TIMER-translog.start_tm);
          // if secure trans add duration to secure log (PROBABLY IRRELEVANT NOW)
          if (translog.flags & FLAG_SECURE) secureTransLog[systemStation].start_tm += translog.duration;
          //if (printlog_active) print_transaction(translog);
          addTransaction( translog, 0 );

          // notify destination of completion
          finalcmd.command=TRANS_COMPLETE;
          finalcmd.station=systemStation;
          finalcmd.data[0]=DIR_SEND;
          // check if (stat alert is active) or (auto return is active) or (alert always)
          if ( (statTrans==TRUE) || (autoRtnTrans==TRUE) || (param.remoteAudibleAlert==TRUE) )
          {
             finalcmd.data[1]=1;
          } else {
             finalcmd.data[1]=0;
          }
          finalcmd.data[3]=translog.status;  // provide transaction status to remotes for local logging
          finalcmd.data[4]=translog.flags;  // provide transaction status to remotes for local logging
          new_state=FINAL_COMMAND;
          systemStation=0;
          systemStationb=0;
 	       transProgress=90;  	// % progress
       } else
       {

			// check for doors opening
	      new_state=checkDoorWhileRunning(system_state);

	      // Check for timeout
	      //if (MS_TIMER-state_timer >= DELIVER_TIMEOUT)
	      if ((int)(SEC_TIMER-translog.start_tm) > param.deliveryTimeout + doorOpenTimer)
	      {  message_add(TRANSLCD, MSG_ODUE, MSG_TO+systemStation, NEXT);
	        new_state=MALFUNCTION_STATE;
	        statistics.deliv_alarm++;
	        translog.status=STS_ARRIVE_TOUT; translog.duration=(int)(SEC_TIMER-translog.start_tm);
	        //blower(OFF);
	      }
       }
        break;

      case WAIT_FOR_REM_DEPART:
       transProgress=40;  	// % progress
       // check for doors opening
       new_state=checkDoorWhileRunning(system_state);

       // wait for carrier to exit
       if (MS_TIMER-state_timer >= CIC_EXIT_TIMEOUT)
       {  // been waiting too long, cancel
          statistics.cic_lift_alarm++;
          //blower(OFF);
          translog.status=STS_DEPART_TOUT; translog.duration=(int)(SEC_TIMER-translog.start_tm);
          //print_malfunction( translog );
			 addTransaction( translog, 0 );
          if (blwrError==TRUE)
          {  // blower timeout error
             new_state=MALFUNCTION_STATE;
          } else
          {  // just cancel
             new_state=CANCEL_STATE;
          }
       }

       if (~remote_data[REMOTE_CIC] & systemStationb)
       {
          if (param.headDiverter==TRUE) new_state=WAIT_FOR_HEADDIVERTER;
          else
          {   new_state=WAIT_FOR_MAIN_ARRIVE;
              if (destMain == MASTER) arrivalEnable(ON);      // enable arrival interrupt
              transProgress=60;  	// % progress

          }
       }

       break;

      case WAIT_FOR_HEADDIVERTER:
       // check for doors opening
       new_state=checkDoorWhileRunning(system_state);

       // wait for arrival at headdiverter controller
	      if (remote_data[REMOTE_ARRIVE] & HEADDIVERTERSTATION) // at headdiverter
	      {
	         // shift headdiverter diverter
	         message_add(TRANSLCD, MSG_SETTING, MSG_TO+systemStation, NOW);
	         // tell all system(s) to get ready (set diverters)
	         set_diverter(systemStation);
	         command.devAddr=ALL_DEVICES;
	         command.command=SET_DIVERTER;
	         command.station=systemStation;
				command.data[0]=transTypeInfo[transactionType][TT_TO];  // For head diverter
				command.data[1]=mainStation;  // For slave??
            command.data[2]=param.subStaAddressing;
            command.data[3]=param.blowerType;
	         received=send_command(command);
	         //if (received) new_state=SHIFT_HEADDIVERTER;
	         if (1) new_state=SHIFT_HEADDIVERTER;
	         else new_state=CANCEL_STATE;
	      }

       // check for timeout
         if ((int)(SEC_TIMER-translog.start_tm) > param.deliveryTimeout + doorOpenTimer)
         {  message_add(TRANSLCD, MSG_ODUE, MSG_AT+systemStation, NEXT);
            statistics.deliv_alarm++;
            new_state=MALFUNCTION_STATE;
            translog.status=STS_ARRIVE_TOUT; translog.duration=(int)(SEC_TIMER-translog.start_tm);
            //blower(OFF);
         }

       break;

      case SHIFT_HEADDIVERTER:
	      transProgress=50;  	// % progress
        // wait for headdiverter diverter to be set
        // check for timeout
       if (MS_TIMER-state_timer >= DIVERTER_TIMEOUT)
       {  // timeout
         statistics.divert_alarm++;
         new_state=MALFUNCTION_STATE;
         translog.status=STS_DIVERTER_TOUT; translog.duration=(int)(SEC_TIMER-translog.start_tm);
         message_add(TRANSLCD, MSG_TIMEOUT, MSG_SETTING, NEXT);
       }

       // wait for headdiverter diverter to be set
       if (divertersReady(systemStation, transTypeInfo[transactionType][TT_TO], mainStation))
        {  // new_state=WAIT_FOR_xxx_ARRIVE;    // serves both directions
           if ((system_direction==DIR_SEND) && (main2main_trans == 0))
           {  // message_add(TRANSLCD, MSG_INROUTE, MSG_TO+systemStation, NOW);
              new_state=WAIT_FOR_REM_ARRIVE;
           }else
           {  // message_add(TRANSLCD, MSG_INROUTE, MSG_FROM+systemStation, NOW);
              if (destMain == MASTER) arrivalEnable(ON);      // enable arrival interrupt
              new_state=WAIT_FOR_MAIN_ARRIVE;
           }
	        transProgress=60;  	// % progress

           message_del(TRANSLCD, MSG_SETTING, MSG_TO+systemStation);
        }

       break;

      case WAIT_FOR_MAIN_ARRIVE:

         // check for doors opening
         new_state=checkDoorWhileRunning(system_state);

         // Check for timeout
         //if (MS_TIMER-state_timer >= DELIVER_TIMEOUT)
         if ((int)(SEC_TIMER-translog.start_tm) > param.deliveryTimeout + doorOpenTimer)
         {  message_add(TRANSLCD, MSG_ODUE, MSG_FROM+systemStation, NEXT);
            statistics.deliv_alarm++;
            new_state=MALFUNCTION_STATE;
            translog.status=STS_ARRIVE_TOUT; translog.duration=(int)(SEC_TIMER-translog.start_tm);
         }

         // if tube arrived ...
         if ((carrierArrival(destMain) || latchCarrierArrival)
            || (param.inlineOptic==FALSE && di_carrierInChamber))

         {  // handle arrival, transaction complete
            inUse(OFF, systemStationb);
            arrivalEnable(OFF);      // disable arrival interrupt
            message_del(TRANSLCD, MSG_INROUTE, MSG_FROM+systemStation);
            message_del(TRANSLCD, MSG_INROUTE, MSG_TO+destMain);
            if (destMain==MASTER)
            {  if (translog.flags & FLAG_ARETURNING) main_arrival=1;  // constant on alert
            	else main_arrival=SEC_TIMER;   // set equal to current time for pulsed alert
            }
            else slave_arrival=SEC_TIMER;
            // shall we set the auto return timer?
            if (translog.flags & FLAG_ARETURN) autoReturnTime[destMain]=SEC_TIMER;
            //lcd_smartPrompt(3); // remove CIC
            arrival_from|=systemStationb;
            message_add(SYSTEMLCD, MSG_ARVALRT, MSG_AT+destMain, NEXT);
            message_del(SYSTEMLCD, MSG_SYSBUSY, MSG_DONTUSE);

            // print transaction data
            statistics.trans_in++;
            translog.duration = (int)(SEC_TIMER-translog.start_tm);
            //if (printlog_active) print_transaction(translog);
            addTransaction( translog, 0 );

            // notify destination of completion
            finalcmd.command=TRANS_COMPLETE;
            finalcmd.station=systemStation;
            finalcmd.data[0]=DIR_RETURN;
	         finalcmd.data[1]=0; // no remote alert
	         finalcmd.data[2]=mainStation;
	         finalcmd.data[3]=translog.status;  // provide transaction status to remotes for local logging
	         finalcmd.data[4]=translog.flags;  // provide transaction status to remotes for local logging
            new_state=FINAL_COMMAND;
            systemStation=0;
            systemStationb=0;
		      transProgress=90;  	// % progress
         } else
         {  // always re-activate the interrupt controller to attempt to fix spurious lost arrivals
           	// WrPortI(I1CR, &I1CRShadow, 0x0E);		// enable external INT1 on PE1, rising edge, priority 1
         }


         break;

      case HOLD_TRANSACTION:      // serves both send and return
       translog.flags|=FLAG_DOOROPEN;
       // restore send when both doors close
       if ((remote_data[REMOTE_DOOR] & systemStationb) && doorClosed(mainStation) && doorClosed(destMain))
       {
          // check if user inserted a new carrier while the door was open
          XtraCic = FALSE;
          if ((memory_state == WAIT_FOR_REM_ARRIVE) && carrierInChamber(mainStation)) XtraCic = TRUE;
          if ((memory_state == WAIT_FOR_MAIN_ARRIVE) && (remote_data[REMOTE_CIC] & systemStationb)) XtraCic = TRUE;
          if  (memory_state == WAIT_FOR_HEADDIVERTER)
          {  if ((system_direction == DIR_SEND) && carrierInChamber(mainStation)) XtraCic = TRUE;
             if ((system_direction == DIR_RETURN) && (remote_data[REMOTE_CIC] & systemStationb)) XtraCic = TRUE;
          }
          if (XtraCic)
          {
            // stay in the alert/alarm mode
            if (FirstXtraCic)
            {  print_event("EXTRA CARRIER INSERTED IN CHAMBER IN TRANSACTION FROM SOURCE");
               FirstXtraCic=FALSE;
               eventlog.flags |= EFLAG_EXTRA_CIC;
            }
          } else
          {  // no problem, resume transaction
             new_state=memory_state;
             setAlerts(OFF);
             // Turn off remote alert
             command.devAddr=ALL_DEVICES;
             command.command=SET_OUTPUTS;
             command.station=systemStation;
             command.data[devInUseSolid]=0;
             command.data[devInUseFlash]=0;
             command.data[devAlert]=0;
             command.data[devAlarm]=0;
             send_command(command);
             print_event("TRANSACTION RESUMED");
             FirstXtraCic=TRUE;
             doorOpenTimer += ((MS_TIMER - state_timer)/1000); // door open timer in seconds
             // capture and log event information
             eventlog.duration=(int)(SEC_TIMER-eventlog.start_tm);
				 addTransaction( eventlog, 0 );  // transaction resumed, log the event
             eventlog.flags=0;  // clear flags after logging
          }
       } // was there an arrival while the door is opened?
//       else if (  ((memory_state == WAIT_FOR_REM_ARRIVE) &&
//                ||((memory_state == WAIT_FOR_MAIN_ARRIVE) &&
//          		 ||((memory_state == WAIT_FOR_HEADDIVERTER) &&
//       { // carrier arrival while the door was opened so continue right away
//       }
       else
       {  // otherwise toggle alert on 2 off 5
          if (((MS_TIMER-state_timer) % 7) >= 2) setAlerts(OFF);
          else setAlerts(ON);
       }

       // Check for timeout
       //if (MS_TIMER-state_timer >= DELIVER_TIMEOUT)
/* v327: don't timeout a transaction while the door is open (UNLESS ITS A REALLY LONG TIME???)
       if ((int)(SEC_TIMER-translog.start_tm) > param.deliveryTimeout + doorOpenTimer)
       {  new_state=MALFUNCTION_STATE;
          statistics.deliv_alarm++;
          translog.flags|=3; translog.duration=SEC_TIMER;
       }
*/
       break;

		case MANUAL_VACUUM:
         if (di_manVacuum && !di_manPressure && di_doorClosed && di_doorClosed2)
         {  //new_state=MANUAL_VACUUM; //blower(blwrVAC);
         	//if (first_time_here)
            //{  lcd_printMessage(2, "  MANUAL VACUUM   ");
            //}
         }
         else
         {  new_state=CANCEL_STATE;
         	message_del(SYSTEMLCD, MSG_MAN_VACUUM, MSG_BLANK);
         }
       break;

		case MANUAL_PRESSURE:
         if (di_manPressure && !di_manVacuum && di_doorClosed && di_doorClosed2)
         {  //new_state=MANUAL_PRESSURE; //blower(blwrPRS);
         	//if (first_time_here)
            //{  lcd_printMessage(2, "  MANUAL PRESSURE ");
            //}
         }
         else
         {  new_state=CANCEL_STATE;
         	message_del(SYSTEMLCD, MSG_MAN_PRESSURE, MSG_BLANK);
         }
       break;

      case PAUSE_TRANSACTION:
      	if (first_time_here)
         {  message_add(TRANSLCD, MSG_PAUSED, MSG_BLANK, NEXT);
         }
         // STILL PAUSED?
         if (di_pause || di_pause2)
         {  // remain here
         } else
         {  // resume
	         setAlerts(OFF);
	         alarm(OFF);
            message_del(TRANSLCD, MSG_PAUSED, MSG_BLANK);
            new_state=memory_state;
         }
       break;

      case CANCEL_STATE:
	      // reset all outputs and return to normal
	      inUse(OFF, systemStationb);
	      setAlerts(OFF);
	      alarm(OFF);
	      arrivalEnable(OFF);      // disable arrival interrupt
			alarm_silenced_flag=FALSE;   // reset alarm sound for next alarm
         blwrError=FALSE;
        // clear lcd message buffer unless just starting up
        if (systemStartup==FALSE)
	     {  message_del(SYSTEMLCD, EMPTY, EMPTY);
   	     message_del(TRANSLCD, EMPTY, EMPTY);
        }

        finalcmd.command=RESET;
        finalcmd.station=0;
        new_state=FINAL_COMMAND;
        systemStation=0;
        systemStationb=0;
        malfunctionActive=FALSE;

        // turn off priority if it was on
        statTrans = FALSE;
        secureTrans = FALSE;
        autoRtnTrans = FALSE;
        break;

      case MALFUNCTION_STATE:
		  // first time into malfunction?
        if (malfunctionActive==FALSE)
        {
           inUse(OFF, ALL_STATIONS);
           setAlerts(OFF);
           //print_malfunction( translog );
           addTransaction( translog, 0 );
           lcd_drawScreen(1,lcd_WITH_BUTTONS); // redraw screen with buttons to allow for alarm menu
           // report blower errors
           if (blwrError==TRUE)
           {   message_add(TRANSLCD, MSG_TIMEOUT, MSG_BLOWER, NEXT);
           }
        }
        arrivalEnable(OFF);      // disable arrival interrupt
        if (mainStation == MASTER) alarm(ON);
        // turn off priority if it was on
        statTrans = FALSE;
        secureTrans = FALSE;
        autoRtnTrans = FALSE;
        // signal alarm at remote
        command.devAddr=ALL_DEVICES;
        command.command=MALFUNCTION;
        command.station=systemStation;
        send_command(command);

        // put up lcd message
        message_add(SYSTEMLCD, MSG_ALARM, MSG_ALARM+1, NOW);
        malfunctionActive=TRUE;

        // check reset by digital input
        if (di_alarmReset) new_state=CANCEL_STATE;

        break;

      case FINAL_COMMAND:
         // set the command before entering this state
         // remains in this state until command received or timeout
         //if (systemStartup==FALSE) transProgress=999;  // blank out progress bar
         //else transProgress=101; // don't blank progress bar
// P2P WORK TO BE DONE HERE
         finalcmd.devAddr=ALL_DEVICES;
         if (send_command(finalcmd))
         {
            new_state=IDLE_STATE;
            if (finalcmd.command==TRANS_COMPLETE) incrementCounter();
            // turn off priority if it was on
            statTrans = FALSE;
	         secureTrans = FALSE;
	         autoRtnTrans = FALSE;
            transProgress=101; // don't blank progress bar
            if (systemStartup==FALSE) lcd_drawScreen(1,lcd_WITH_BUTTONS);  // Redraw screen with buttons
		      systemStartup=FALSE;  // no longer in system startup mode
         }

         // check timeout
         if (MS_TIMER-state_timer >= 1000)
         {  new_state=IDLE_STATE;
         	if (systemStartup==FALSE) lcd_drawScreen(1,lcd_WITH_BUTTONS);  // Redraw screen with buttons
		      systemStartup=FALSE;  // no longer in system startup mode
			}
         break;

      default:
         // unknown state, execute cancel code
         new_state=CANCEL_STATE;
   }

   // if state change, reset state timer
   if (new_state != system_state)
   {
      // set blower for new state
      blower(blowerStateTable[new_state][blwrConfig]);
      state_timer=MS_TIMER;
      system_state=new_state;
      first_time_here=TRUE;    // wasn't here yet
      sendUDPHeartbeatToHost(0);     // notify network on state change

      // set door lock for new state
      if (new_state==IDLE_STATE) do_doorUnLock(ON);
      else do_doorUnLock(OFF);
   }
   else first_time_here=FALSE; // were already here


}
char checkSecureRemoval(void)
{  // Look into secure tracking structure for any removed secure transactions
   char i;

   // secureAck = 0;  // to handshake back that it was handled
   for (i=1; i<9; i++)
   {  // anything to do?  when both sid and start time are non zero
      if (UID_Not_Zero(secureTransLog[i].sid) && secureTransLog[i].start_tm)
      {  // handle secure removal if scan type =1
         // flags contains the delta-t number of seconds 1:10 scale
         //secureTransLog[i].start_tm += secureTransLog[i].flags*6;
         secureTransLog[i].flags=i;  // flags to contain the station number
      	// log it - will get warning about wrong parameter type, ok to ignore
         addSecureTransaction(secureTransLog[i]);
      	// clear PIN message
         message_del(SYSTEMLCD, MSG_PIN, MSG_AT+i);
         // acknowledge it ... again
         secureAck |= station2bit(i);
         // clear it
         UID_Clear(secureTransLog[i].sid);
         secureTransLog[i].start_tm = 0;
      }
   }
}
char autoReturnCarrier()
{  // check if an auto return timer is expired and the return can be started
   char k, kb;
   char returnVal;
   returnVal=0;

	// are there any at all?
   if (arrival_alert)
   {  k=1; kb=1;
   	while (k<=LAST_STATION && returnVal==0)
      {  // for each bit check the auto-return timer
         if ( ((arrival_alert & kb) != 0) && (autoReturnTime[k] != 0))
         {  // auto return is active, now check the time remaining
            if ((SEC_TIMER - autoReturnTime[k]) > param.autoReturnTimer*60)
            {  // and make sure the doors are closed, etc. so it can start
               if ( di_doorClosed && (kb & remote_data[REMOTE_CIC] & remote_data[REMOTE_DOOR]) )
               {  // ok to go, set this bit in the return value
               	returnVal = kb;
               }
            }
         }
         // next bit and station
         kb=1<<k;
      	k++;
      }
   }
   // Check slave auto return separate
   if ((returnVal==0) && slave_arrival)
	{  // is there an auto-return timer set?
   	if (autoReturnTime[SLAVE] != 0)
      {  // has the time expired
      	if ( (SEC_TIMER - autoReturnTime[SLAVE]) > (param.autoReturnTimer*60))
	      {  // and make sure the doors are closed, etc. so it can start
	         if ( (slave_doorClosed==TRUE) && (slave_cic==FALSE))
            {  // ok to go, set this bit in the return value
		         returnVal = 1<<(SLAVE-1);
            }
         }
      }
	}

   return returnVal;
}

char checkDoorWhileRunning(char calling_state)
{  // Check for main or remote doors opening
   struct iomessage command;
   char event_string[80];     // line to send to the printer
   char rtnval;

   rtnval = calling_state;  // return same state by default

   // check for main station door opening
   if ((!doorClosed(mainStation)) || (!doorClosed(destMain)))
   {
	   setAlerts(ON);
	   rtnval=HOLD_TRANSACTION;
	   memory_state=calling_state;
	   strcpy(event_string, "DOOR OPEN IN TRANSACTION (#) ");
      event_string[26] = 97+calling_state;
	   strcat(event_string, sys_message[MSG_AT+mainStation].msg);
	   print_event(event_string);
      eventlog.start_tm=SEC_TIMER;
      eventlog.source_sta=mainStation;
      eventlog.dest_sta=calling_state;
      eventlog.status=ESTS_DOOROPEN;
      eventlog.flags|=EFLAG_MAINDOOR;

      //st_time_diff = MS_TIMER - st_timer; // hold transfer timeout
      //message_add(SYSTEMLCD, MSG_SYSALARM, MSG_DOOR_OPEN, NOW);
      //door_warning = TRUE;
   }
   // check for remote station door opening
   if (~remote_data[REMOTE_DOOR] & systemStationb)
   {
      setAlerts(ON);
      // Turn on remote alert
      command.devAddr=ALL_DEVICES;
      command.command=SET_OUTPUTS;
      command.station=systemStation;
      command.data[devInUseSolid]=0;
      command.data[devInUseFlash]=0;
      command.data[devAlert]=systemStationb;
      command.data[devAlarm]=0;
// P2P WORK TO BE DONE HERE
      send_command(command);
      rtnval=HOLD_TRANSACTION;
      memory_state=calling_state;
      strcpy(event_string, "DOOR OPEN IN TRANSACTION (#) ");
      event_string[26] = 97+calling_state;
      strcat(event_string, sys_message[MSG_AT+systemStation].msg);
      print_event(event_string);
      // capture event information
      eventlog.start_tm=SEC_TIMER;
      eventlog.source_sta=systemStation;
      eventlog.dest_sta=calling_state;
      eventlog.status=ESTS_DOOROPEN;
      eventlog.flags|=EFLAG_SUBSDOOR;

      //st_time_diff = MS_TIMER - st_timer; // hold transfer timeout
      //message_add(SYSTEMLCD, MSG_SYSALARM, MSG_DOOR_OPEN, NOW);
      //door_warning = TRUE;
   }

   // ALSO CHECK PAUSE
   if ((param.pauseEnabled) && (di_pause || di_pause2))
   {  // enter pause mode
      memory_state=calling_state;
      setAlerts(ON);
      alarm(ON);
   	rtnval = PAUSE_TRANSACTION;
   }

   return rtnval;
}

void processMainArrivalAlert(void)
{
	// clear main arrival alert when door opens and no more carrier
	if (main_arrival)
	{
	   // check for 10 second arrival alert signal
      if (main_arrival==1)
      {  if (param.areturn_arrive_alert)          		 do_audibleAlert(ON);  // constant on
         if (param.mainVisualAlert)                    do_visualAlert(ON);
      }
	   else if ( (SEC_TIMER - main_arrival) %20 >= 10 )
      {  if (param.mainAudibleAlert)						 do_audibleAlert(ON);  // pulsed on
         if (param.mainVisualAlert)                    do_visualAlert(ON);
      }
	   else
      {                                       			 do_audibleAlert(OFF);
      																 do_visualAlert(OFF);
      }

	   // check to clear main arrival alert
	   if (!di_doorClosed && !di_carrierInChamber)
	   {  main_arrival=0;
	      arrival_from=0;
	      message_del(SYSTEMLCD, MSG_ARVALRT, MSG_AT);
	      setAlerts(OFF);
	      latchCarrierArrival=FALSE;    // clear latch
         //lcd_smartPrompt(0); // clear smartPrompt
	   }
	}
}

void processStatPriority(void)
{  // use btnStatFlag and btnSecureFlag to communicate between lcd screen functions and here
   // btnStatFlag = 1 - turn on stat
   // btnStatFlag = 2 - turn off stat
   // btnSecureFlag = 1 - turn on secure
   // btnSecureFlag = 2 - turn off secure
   //             = else - no change

   // activate priority (next transaction) if requested
   if (di_priorityRequest || (btnStatFlag==1))
   {  statTrans = TRUE;
      btnStatFlag = 0;
      stat_expire = SEC_TIMER + STAT_TIMEOUT;
      do_priorityLight(ON);
      message_add(SYSTEMLCD, MSG_STAT, MSG_STAT+1, NOW);
   }
   // check for secure activation
   if (btnSecureFlag==1)
   {  secureTrans = TRUE;
      btnSecureFlag=0;
      stat_expire = SEC_TIMER + STAT_TIMEOUT;  // use stat timeout
      // Generage random PIN and display on the screen, make sure its not 0000
      securePIN[0] = (int)(MS_TIMER % 9880)+110; // range of 0110 to 9989
      //sprintf(&sys_message[MSG_PIN].msg[MSG_OFFSET+12], "%04d", securePIN[0]);
      message_add(TRANSLCD, MSG_STAT, MSG_PIN, NOW);

   }
   // check for auto return activation
   if (btnAReturnFlag==1)
   {  autoRtnTrans = TRUE;
      stat_expire = SEC_TIMER + STAT_TIMEOUT;  // use stat timeout
      btnAReturnFlag=0;
   }
   // deactivate secure after timeout or 2nd button press AND NO CIC
   if ((secureTrans && (SEC_TIMER > stat_expire) && (system_state == IDLE_STATE) && (!carrierInChamber(mainStation))) || (btnSecureFlag==2))
   {  secureTrans = FALSE;
      btnSecureFlag=0;
      // Remove PIN from the display
      securePIN[0] = -1;
      message_del(TRANSLCD, MSG_STAT, MSG_PIN);
      // deactivate screen button
      lcd_SetState( BTN_SECURE, 0);
   }
   // deactivate Auto Return after timeout or 2nd button press AND NO CIC
   if ((autoRtnTrans && (SEC_TIMER > stat_expire) && (system_state == IDLE_STATE) && (!carrierInChamber(mainStation))) || (btnAReturnFlag==2))
   {  autoRtnTrans = FALSE;
      btnAReturnFlag=0;
      // deactivate screen button
      lcd_SetState( BTN_ARETURN, 0);
   }
   // deactivate STAT priority after timeout, if not in a transaction AND NO CIC
   if ((statTrans && (SEC_TIMER > stat_expire) && (system_state == IDLE_STATE) && (!carrierInChamber(mainStation))) || (btnStatFlag==2))
   {  statTrans = FALSE;
      btnStatFlag = 0;
      do_priorityLight(OFF);
      // deactivate screen button
      lcd_SetState( BTN_STAT, 0);
      message_del(SYSTEMLCD, MSG_STAT, MSG_STAT+1);
   }
}

const char msgTable[4][2] = {MSG_BLANK, MSG_BLANK,
 									  MSG_BLANK, MSG_DROPEN,
                             MSG_CIC,   MSG_BLANK,
                             MSG_CIC,   MSG_DROPEN};
void update_cic_door_msgs(void)
{  // Add or remove CIC and Door messages from the TRANSLCD

	static char lastCIC, lastDoor;
   char CIC, Door;
   char theRow;
   char i;
   #GLOBAL_INIT { lastCIC=0; lastDoor=0; }

   // Note that ideal state of CIC and Door digital inputs is opposite each other
   CIC = di_carrierInChamber ? 1 : 0;  // ensure it is only 1 or 0
   Door = di_doorClosed ? 0 : 1;       // ensure it is only 0 or 1
   if ((lastCIC != CIC) || (lastDoor != Door))
   {  // One or the other changed so update all messages
   	lastCIC = CIC;
      lastDoor = Door;
      theRow = lastCIC*2 + lastDoor;
      // don't actually use case 0,0
      if (i>0) message_add(TRANSLCD, msgTable[theRow][0], msgTable[theRow][1], NEXT);
      // delete the other messages
      for (i=1; i<4; i++)
		{  if (i != theRow) message_del(TRANSLCD, msgTable[i][0], msgTable[i][1]);
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

   // initialize LED states off
   #GLOBAL_INIT
   {  LEDState[0]=0; LEDState[1]=0; LEDState[2]=0; LEDState[3]=0;
   }

   LEDState[LEDDev] ^= 1;               // toggle LED state on/off
   ledOut(LEDDev, LEDState[LEDDev]);    // send LED state

}

/******************************************************************/
// Blower Processing Routines - supports both standard and APU blowers
/******************************************************************/
// Interface definition
// char blwrConfig;     // to become a parameter from eeprom
// blwrConfig 0 = Straight through pipeing
// blwrConfig 1 = Head diverter configuration
// #define blwrOFF  0
// #define blwrIDLE 1
// #define blwrVAC  2
// #define blwrPRS  3
// void initBlower(void);
// void blower(char blowerOperatingValue);  // use blwrOFF, blwrIDLE, blwrVAC, blwrPRS
// char processBlower(void);
// char blowerPosition(void);

char blower_mode, blower_limbo, last_shifter_pos, lastDir;
unsigned long blower_timer;

void initBlower()
{   // trys to find any shifter position, then goes to idle
    unsigned long mytimer;

    blwrConfig = 0; // Not a head diverter configuration

    // Clear global blower variables
    last_shifter_pos=0;
    lastDir=0;
    blower_limbo=FALSE;
    remoteBlwrPos=blwrIDLE;
    blwrError=FALSE;

    // turn off all outputs
    do_blower(OFF);
    do_blowerVac(OFF);
    do_blowerPrs(OFF);
    blower_mode=blwrOFF;

    // remainder of initialization is for APU blowers only
    if ((param.blowerType == blwrType_APU) && (param.headDiverter == FALSE))
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
             if (blowerPosition==0) blwrError=TRUE;  // Blower timeout
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
   {  blwrConfig = 0; // Not a head diverter configuration
      lastHow = 0;
   }

   // Process standard blowers
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
	      msDelay(100);                   /* wait a little while */
	   }

	   /* turn on appropriate bit */
	   if (how == blwrPRS)      do_blowerPrs(ON);
	   else if (how == blwrVAC) do_blowerVac(ON);
	   else
      {  do_blowerPrs(OFF);
	      do_blowerVac(OFF);
	   }
	   // Add alternate blower on/off control per Joe request on 10-Jul-07
   	if (how != blwrOFF) do_blower(ON);
		else do_blower(OFF);

	   // remember last setting
	   lastHow = how;
      blower_timer = MS_TIMER;
      blower_mode = how;

   } // end of standard blower
   // process APU blowers
   else if (param.blowerType == blwrType_APU)
   {
	   // store in local work space
	   blower_mode=request;
	   blower_limbo=FALSE;

	   /* clear previous state */
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

    // Turn on secondary output 2 seconds after first output goes on
    if ((MS_TIMER - blower_timer > 2000) && (blower_mode != OFF))
       	do_blowerDly(ON);
    else	do_blowerDly(OFF);

    // Remainder only for APU blowers if there is no head diverter
    if ((param.blowerType == blwrType_APU) && (param.headDiverter == FALSE))
    {

	    if (blower_mode==OFF) return rtnval;  // nothing happening, go home.
	    if (blower_mode==ON) return rtnval;   // running (ON is generic run mode) ... nothing to do

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
char blowerReady()
{  // Checks for APU blowers to be in the idle position
	char rtnval;
   rtnval=TRUE;  // assume ready
   if (param.blowerType==blwrType_APU)  // STD blowers always ready
   {  if (param.headDiverter==TRUE)     // local or remote APU
      {  if (remoteBlwrPos!=blwrIDLE) rtnval=FALSE;
      } else
      {  if (blowerPosition()!=blwrIDLE) rtnval=FALSE;
      }
	   if (blwrError==TRUE) rtnval=FALSE;  // Blower timeout detected?
   }
   return rtnval;
}

// DIVERTER PROCESSING ROUTINES
void setDiverterConfiguration()
{
   // diverter configuration set by localDiverter byte
   // 0 = no local diverter; 1 = local diverter;

   // sets up values in the diverter_map station array
   // Array index is station # 1..7;
   // Value is diverter address; leg 1 or 2 or 0=n/a

   if (param.localDiverter == 0)
   {  // All stations n/a
      diverter_map[0] = 0; diverter_map[1] = 0;
      diverter_map[2] = 0; diverter_map[3] = 0;
      diverter_map[4] = 0; diverter_map[5] = 0;
      diverter_map[6] = 0; diverter_map[7] = 0;
      diverter_map[8] = 0;

   } else {
      // Station 1..4 on diverter leg 1;   5..7 on leg 2
      diverter_map[0] = 0; diverter_map[1] = 1;
      diverter_map[2] = 1; diverter_map[3] = 1;
      diverter_map[4] = 1; diverter_map[5] = 2;
      diverter_map[6] = 2; diverter_map[7] = 2;
      diverter_map[8] = 0;  // no local diverter for slave
   }
}
/******************************************************************/
char divertersReady(char station, char headDiverter, char mainDiverter)
{  // Indicates if all local and remote diverters are set to
   // the specified station position.  Returns TRUE or FALSE.

   struct iomessage command, response;
   char rtnval;

	if (param.point2point==TRUE) rtnval=TRUE;
   else
   {
	   rtnval=FALSE;   // assign default value
	   command.devAddr=ALL_DEVICES;
	   command.command=DIVERTER_STATUS;
	   command.station=station;
	   command.data[0]=headDiverter;
	   command.data[1]=mainDiverter;
	   // also pass along the secure PIN for the active station
	   command.data[2]=(char)(securePIN[systemStation] >> 8);   // high byte
	   command.data[3]=(char)(securePIN[systemStation] & 0xFF); // low byte
	   response.data[0]=0;
	// P2P WORK TO BE DONE HERE
///	   send_n_get(command, &response);
	   //if ( ( (response.data[0] & STATION_SET)==STATION_SET)
///	   if ( (response.data[0]==availableDevices)
		if ( (remote_data[REMOTE_DIVERTER]==diverterDevices)
	       && ( (diverter_attention==FALSE)
	             || (diverter_map[station]==0) ) )
	   {   rtnval=TRUE;  }

	   // setup the diverter status to find out which device is not ready
	   diverter_status = remote_data[REMOTE_DIVERTER]; //response.data[0];
	   if ((diverter_map[station]==0) || (diverter_attention==FALSE)) diverter_status |= 0x100;
   }
   return rtnval;
}
/******************************************************************/
void set_diverter(char station)
{  /* Controls the setting of diverter positions
      Return from this routine is immediate.  If diverter is not
      in position, it is turned on and a timer started.

      You MUST repeatedly call check_diverter() to complete processing
      of the diverter control position
   */
   // reset remote diverter status
   remote_data[REMOTE_DIVERTER]=0; // force reset

   /* use mapping from station to diverter position */

   if ((station>0) && (station<9))     // Valid station #
   {
      diverter_setting = diverter_map[station];    // mapped setting

      // if position<>ANY (any=0) and not in position
      if ((diverter_setting>0) && (diverter_setting!=di_diverterPos))
      {
         /* turn on diverter and start timer */
         do_diverter(ON);
         diverter_start_time = MS_TIMER; // + DIVERTER_TIMEOUT;
         diverter_attention = TRUE;
         // NOT USED IN MAIN diverter_station=station;
      }
   }
   return;
}
/******************************************************************/
void check_diverter()
{  /* Poll type processing of diverter control system */
   /* Allows other processes to be acknowleged when setting diverter */

   if (diverter_attention)
   {
      if ((diverter_setting == di_diverterPos) || (MS_TIMER-diverter_start_time > DIVERTER_TIMEOUT))
      {
         // turn off diverter and clear status
         do_diverter(OFF);
         diverter_attention = FALSE;
         // NOT USED IN MAIN: diverter_station=0;
      }
   }
}

//**************************************
//   DIGITAL INPUT/OUTPUT FUNCTIONS
//**************************************
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
      { if (rn_digIn(DevRN1100, channel-16, &rtnval, 0) == -1) rtnval=0; }
      else rtnval = 0;
   }
   // deal with logic inversion
   if (rtnval) rtnval = dio_ON;
   else        rtnval = dio_OFF;
   return rtnval;
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
void alarm(char how)
{
   if (how)
   {  // turn on light
      //do_alarmLight(how);  removed to substitute for alternate blower output, request by Joe 10-Jul-07
      // if sound flag on, turn noise on, else off.
      if (alarm_silenced_flag) do_alarmSound(OFF);
      else do_alarmSound(how);
   }
   else
   {  // turn off light and sound
      do_alarmSound(how);
      //do_alarmLight(how);
   }

   return;
}

void inUse(char how, char station_b)
{	// Only sets the global flags
   // Call Maintenance to actually turn on/off lights

   if (how == ON)
   {  /* solid on, flash off */
      outputvalue[devInUseSolid] |= station_b & STATION_SET;
      outputvalue[devInUseFlash] &= ~(station_b & STATION_SET);
   }
   else if (how == OFF)
   {  /* solid off, flash off */
      outputvalue[devInUseSolid] &= ~(station_b & STATION_SET);
      outputvalue[devInUseFlash] &= ~(station_b & STATION_SET);
   }
   else if (how == FLASH)
   {  /* solid off, flash on */
      outputvalue[devInUseSolid] &= ~(station_b & STATION_SET);
      outputvalue[devInUseFlash] |= station_b & STATION_SET;
   }
   // Turn off unused stations
   outputvalue[devInUseSolid] &= STATION_SET;
   outputvalue[devInUseFlash] &= STATION_SET;

   return;
}
// door lock routines
//unsigned long doorUnLockTimer;
//void lockDoor(char how)
//{  // lock (TRUE) or unlock (FALSE) the door
//	// trigger door output timer
//	//#GLOBAL_INIT {doorUnLockTimer=0;}
//	//doorUnLockTimer = MS_TIMER;
//   if (how) do_doorUnLock(ON);
//   else do_doorUnLock(OFF);
//}

// define input functions which take master/slave as a parameter
char carrierInChamber(char whichMain)
{	if (whichMain==MASTER) return di_carrierInChamber;
	else return slave_cic; //Data[0] & 0x01;
}
char carrierArrival(char whichMain)
{	if (whichMain==MASTER) return di_carrierArrival;
	else return slave_arrive; // Data[0] & 0x04;
}
char doorClosed(char whichMain)
{	if (whichMain==MASTER) return di_doorClosed;
	else return slave_doorClosed; //Data[0] & 0x02;
}
void setAlerts(char how)
{  // sets both audible and visual alerts
	do_audibleAlert(how);
	do_visualAlert(how);
}

char valid_send_request(char station_b)
{  // make sure conditions are right to send a carrier
   // put up a brief message if not
   // checks for local and remote CIC, door, stacking, auto-return
   // returns zero when all conditions are ready to go
   char ecode, stanum;
   stanum = bit2station(station_b);
   if (stanum > 0)
   {
      // make sure local inputs are correct
      if (di_carrierInChamber && di_doorClosed)
      {
         ecode=0;    // good local request ... for now
         // make sure no remote carrier unless stacking is ok and not a pending secure or pendign auto return
	      if (remote_data[REMOTE_CIC] & station_b)
         {  // remote CIC but maybe this is ok
				if (secureTransLog[stanum].start_tm != 0)
            {  // this is not ok
               ecode=4;
               //reset_msg_timer(NEXT);
               //lcd_printMessage(0, "   CAN NOT SEND   ");
               //lcd_printMessage(1, "SECURE NOT REMOVED");
               strncpy(sys_message[MSG_CANT_WHY].msg, "SECURE NOT REMOVED", MSG_LEN);

            }
            else if (autoReturnTime[stanum] !=0)
            {  // this is not ok
            	ecode=5;
               //reset_msg_timer(NEXT);
               //lcd_printMessage(0, "   CAN NOT SEND   ");
               //lcd_printMessage(1, "AUTO-RETURN ACTIVE");
               strncpy(sys_message[MSG_CANT_WHY].msg, "AUTO-RETURN ACTIVE", MSG_LEN);
            }
            else if (param.stacking_ok[1] == FALSE)
            {  // this is not ok
            	ecode=3;
               //reset_msg_timer(NEXT);
               //lcd_printMessage(0, "   CAN NOT SEND   ");
               //lcd_printMessage(1, "STACKING UNALLOWED");
               strncpy(sys_message[MSG_CANT_WHY].msg, "STACKING UNALLOWED", MSG_LEN);
            }
            else if (autoRtnTrans)
            {  // this is not ok
            	ecode=6;
               //reset_msg_timer(NEXT);
               //lcd_printMessage(0, "   CAN NOT SEND   ");
               //lcd_printMessage(1, "AUTO-RTN ON TO CIC");
               strncpy(sys_message[MSG_CANT_WHY].msg, "AUTO-RTN ON TO CIC", MSG_LEN);
            }
            else if (secureTrans)
            {  // this is not ok
            	ecode=7;
               //reset_msg_timer(NEXT);
               //lcd_printMessage(0, "   CAN NOT SEND   ");
               //lcd_printMessage(1, " SECURE ON TO CIC ");
               strncpy(sys_message[MSG_CANT_WHY].msg, " SECURE ON TO CIC ", MSG_LEN);
            }

            // else then this is fine to stack
			} // else no carrier so no problem
         // check for remote door open
	      if ((remote_data[REMOTE_DOOR] & station_b)==0)
         {  // this is not ok
         	ecode=8;
            strncpy(sys_message[MSG_CANT_WHY].msg, " REMOTE DOOR OPEN ", MSG_LEN);
         }
         // check for remote optic blocked
         if (remote_data[REMOTE_ARRIVE] & station_b)
         {	// this is not ok
         	ecode=9;
            strncpy(sys_message[MSG_CANT_WHY].msg, " REMOTE OPTIC ERR ", MSG_LEN);
         }

	      // if can not send then show the reason why message
	      if (ecode) message_add(TRANSLCD, MSG_CANT_SEND, MSG_CANT_WHY, NOW);
         else       message_del(TRANSLCD, MSG_CANT_SEND, MSG_CANT_WHY);

      } else
      {
         ecode=2;    // local door open or no carrier
         message_del(TRANSLCD, MSG_CANT_SEND, MSG_CANT_WHY);
      }

   } else ecode=1;   // no request or multiple inputs

   return ecode;
}

/*******************************************************/
// transaction request queue functions
#define queue_len 9
struct trans_queue_type
{
    char source;
    char dest;
} trans_queue[queue_len];
char queue_last;

void queueAdd(char tsource, char tdest)
{
    // adds request to queue  USE BIT VALUES

    char i;
    char found;

    // add only if source does not already exist .. otherwise replace
    found=FALSE;
    for (i=0; i<queue_len; i++)
    {   if (trans_queue[i].source==tsource)
        { // replace existing entry
          trans_queue[i].dest = tdest;
          found=TRUE;
        }
    }
    if (found==FALSE)
    {  // not found so add after last current entry
       queue_last = (queue_last+1) % queue_len;
       trans_queue[queue_last].source=tsource;
       trans_queue[queue_last].dest=tdest;
    }
}

void queueDel(char tsource, char tdest)
{
    // deletes request from queue
    // call with (0,0) to initialize
    char i;

    if (tsource==0 && tdest==0)
    {  for (i=0; i<queue_len; i++)
       {  trans_queue[i].source=0;
          trans_queue[i].dest=0;
       }
       queue_last=0;
    } else // find specified entry and delete it
    {  for (i=0; i<queue_len; i++)
       {  if (trans_queue[i].source==tsource && trans_queue[i].dest==tdest)
          {   trans_queue[i].source=0;
              trans_queue[i].dest=0;
              // decrement last if it is the current entry
              if (queue_last==i) queue_last = (queue_last+queue_len-1) % queue_len;
          }
       }
    }
}

void queueNext(char *tsource, char *tdest)
{  // returns next queue entry

   char i;
   char found;

   // start at last+1 until a non-zero entry or back to last

   *tsource=0; *tdest=0;   // initialize to zero

   found=FALSE;
   i=queue_last;
   do
   {   // increment to next queue entry
       i=(i+1) % queue_len;
       if (trans_queue[i].source != 0)
       {   // this must be it
           *tsource=trans_queue[i].source;
           *tdest=trans_queue[i].dest;
           found=TRUE;
       }
   } while (found==FALSE && i!=queue_last);
}

char queueVerify(char cic_data, char door_data)
{
    // step through each queue entry and kill if no carrier or door open
    // returns bit set of each station in queue (to flash in-use)

    char i;
    char rtnval;

    rtnval=0;
    for (i=0; i<queue_len; i++)
    { if (trans_queue[i].source != 0)  // if valid queue entry
      {
        if ((trans_queue[i].source & cic_data & door_data) == 0)
        {   // kill it ... no more carrier or door is open
            queueDel(trans_queue[i].source, trans_queue[i].dest);
        } else
        {   // include these bits in return val
            rtnval |= trans_queue[i].source | trans_queue[i].dest;
        }
      }
    }
    return rtnval;
}


//**************************************
//   LCD MESSAGE FUNCTIONS
//**************************************
static unsigned long msgtimer;
struct queue
{  char curentry;
   char cursize;
   char line1[QUEUESIZE];
   char line2[QUEUESIZE];
} msgqueue[2];
char oneshot[2][2];   // oneshot buffer [lcd][line]
char currentmsg[2][2];  // current display message

void message_del(char lcd, char msgline1, char msgline2)
{
   char k, l, loc;

   if ( (lcd != 0) && (lcd != 1) ) return;      // invalid lcd#

   if (msgline1==EMPTY)
   {  // delete all messages, reinitialize queue
      for (l=0; l<LCDCOUNT; l++)
      {
       for (k=0; k<QUEUESIZE; k++)
       {  msgqueue[l].line1[k] = EMPTY;
          msgqueue[l].line2[k] = EMPTY;
       }
       msgqueue[l].curentry = 0;
       msgqueue[l].cursize = 0;
       oneshot[l][0]=EMPTY;    // clear oneshot buffers
       oneshot[l][1]=EMPTY;
      }
      message_show(NOW);
      //
   }
   else
   {  // remove message from queue
      k=0;
      // find k, the queue entry number
      while ( ((msgline1 != msgqueue[lcd].line1[k])
          || ( (msgline2 != msgqueue[lcd].line2[k])
          || (msgline2 == EMPTY) ))
          && (k < QUEUESIZE-1) ) k++;

      // did we find a matching message?
      if ( (msgline1 == msgqueue[lcd].line1[k])
       && ( (msgline2 == msgqueue[lcd].line2[k])
       || (msgline2 == EMPTY) ) )
      {  // yes so remove the entry
	      loc = k;
	      for (k=loc; k<QUEUESIZE-1; k++)
	      {  msgqueue[lcd].line1[k] = msgqueue[lcd].line1[k+1];
	       msgqueue[lcd].line2[k] = msgqueue[lcd].line2[k+1];
	      }
	      msgqueue[lcd].cursize--;                  // decrement current size
	      if (msgqueue[lcd].cursize==0) msgqueue[lcd].curentry=0;
	      msgqueue[lcd].line1[QUEUESIZE-1] = EMPTY; // insert empty entry at end
	      if (msgqueue[lcd].line1[ msgqueue[lcd].curentry ] == EMPTY)
	      {
	       msgqueue[lcd].curentry = 0;
	       message_show(NOW);   // show next
	      }
	      if (msgqueue[lcd].curentry == loc)
	       message_show(NOW);   // show next
      }
   }
   return;
}

void message_add(char lcd, char msgline1, char msgline2, char how)
{  int k;
   // how = NOW       // show message immediately
   // how = NEXT      // show message next
   // how = ONESHOTA  // show message once first of 2 oneshots
   // how = ONESHOT   // show message once right now

   if ( (lcd != 0) && (lcd != 1) ) return;      // invalid lcd#

   if (how==ONESHOT || how==ONESHOTA)
   {  // show only once
      oneshot[lcd][0]=msgline1;
      oneshot[lcd][1]=msgline2;
      if (how==ONESHOT) message_show(ONESHOT);
      return;
   }

   // insert as next message if not already in queue
   k=0;
   if (msgqueue[lcd].cursize==0)
   {  msgqueue[lcd].curentry=0;
      msgqueue[lcd].line1[0] = msgline1;
      msgqueue[lcd].line2[0] = msgline2;
      msgqueue[lcd].cursize++;
   }
   else
   {
      while ( ( (msgline1 != msgqueue[lcd].line1[k])
       || (msgline2 != msgqueue[lcd].line2[k]) )
          && (k < QUEUESIZE-1) ) k++;
      if ( (msgline1 != msgqueue[lcd].line1[k])
        || (msgline2 != msgqueue[lcd].line2[k]) ) // must not be in q already
      {  for (k=QUEUESIZE-2; k>msgqueue[lcd].curentry; k--)
         {
            msgqueue[lcd].line1[k+1] = msgqueue[lcd].line1[k]; // shift over
            msgqueue[lcd].line2[k+1] = msgqueue[lcd].line2[k]; // shift over
         }
         msgqueue[lcd].line1[ msgqueue[lcd].curentry+1 ] = msgline1;
         msgqueue[lcd].line2[ msgqueue[lcd].curentry+1 ] = msgline2;
         msgqueue[lcd].cursize++;                     // increment current size
      }
      else return;      // no update if already in queue
   }                       // fix queue when size=1 and not current
   if ( (how == NOW ) || ( msgqueue[lcd].cursize==1 ) )
   {  // make current now, otherwise update in time
      message_show(NOW);
   }
   return;
}

void message_show(char how)
{
   char lcd, l1, l2, k;

   if ( (how==NOW) || (how==ONESHOT) || (MS_TIMER - msgtimer >= SCROLLTIME) )
   {  if (how != ONESHOT)
      {
        // index to next message (both lcds)
        for (lcd=0; lcd<LCDCOUNT; lcd++)
        {
          if (msgqueue[lcd].cursize > 0)
          {
             k=msgqueue[lcd].curentry + 1;
             while ((msgqueue[lcd].line1[k] == EMPTY)
                    && (k != msgqueue[lcd].curentry))
                   k=(k+1)%QUEUESIZE;
             if (msgqueue[lcd].line1[k] != EMPTY) msgqueue[lcd].curentry = k;
          }
        }
      }
      msgtimer = MS_TIMER; // + SCROLLTIME;

      // display messages
      for (lcd=0; lcd<LCDCOUNT; lcd++)
      {
        if ((how==ONESHOT) && (oneshot[lcd][0] != EMPTY))
        {
          l1=oneshot[lcd][0];        // use oneshot buffer
          l2=oneshot[lcd][1];
          oneshot[lcd][0]=EMPTY;     // clear oneshot buffer
        } else {                      // load regular message
          l1=msgqueue[lcd].line1[ msgqueue[lcd].curentry ];
          l2=msgqueue[lcd].line2[ msgqueue[lcd].curentry ];
        }

        if (l1==EMPTY)          // show default msg if empty queue
        {  setTimeMessages(MSG_DATE_TIME, SEC_TIMER);  // default uses time message
           if (lcd==TRANSLCD)
           {  l1=MSG_TDEFAULT;
              l2=MSG_TDEFAULT2;
           }
           else
           {  l1=MSG_SDEFAULT;
              l2=MSG_SDEFAULT2;
           }
        }
        // check for timer message refresh before display
        //refresh_dynamic_message(l1);
        //refresh_dynamic_message(l2);
        // refresh PIN message, have to assume line 2 is target station
        //if (l1==MSG_PIN) refresh_PIN_message(l1, l2);
        //if (l2==MSG_PIN) refresh_PIN_message(l2, l1);

        // double check message range before display
        if ((l1>=0) && (l1<MSGCOUNT))
        {  //lcd_print(lcd, 0, sys_message[l1].msg);
           currentmsg[lcd][0] = l1;
        }
        if ((l2>=0) && (l2<MSGCOUNT))
        {  //lcd_print(lcd, 1, sys_message[l2].msg);
           currentmsg[lcd][1] = l2;
        }
      }
      // send command to remote systems with lcd's to display same msg
      displayRemoteMsgs(&currentmsg[0][0]);
      if (echoLcdToTouchscreen) lcd_displayMsgs(&currentmsg[0][0]);  // color touch screen

      for (lcd=0; lcd<LCDCOUNT; lcd++)
      {
        // show messages after sending to slave
        //lcd_print(lcd, 0, sys_message[currentmsg[lcd][0]].msg);
        //lcd_print(lcd, 1, sys_message[currentmsg[lcd][1]].msg);
        printf("%s\n%s\n", sys_message[currentmsg[lcd][0]].msg, sys_message[currentmsg[lcd][1]].msg);
      }

   }
}
void refresh_dynamic_message(char msg)
{  // Update display of dynamic messages
   // timer messages need to refresh every second

   char lcd, line;
   unsigned long timepar;

   switch (msg)
   {
/*   case MSG_EXPOSE_TIME:
      // count up or count down
      if (op_mode == OPMODE_MANUAL) timepar = (MS_TIMER - exposure_start_time) / 1000;
      else timepar = param.exposureTimer - (MS_TIMER - exposure_start_time)/1000;
      sys_message[MSG_EXPOSE_TIME].msg[9]  = 48 + (sample_number / 10);
      sys_message[MSG_EXPOSE_TIME].msg[10] = 48 + (sample_number % 10);
      setTimeMessages(MSG_EXPOSE_TIME, timepar);
      //lcd_print(lcd, line, sys_message[MSG_EXPOSE_TIME].msg);
      break;
*/
   }
}
void refresh_PIN_message(char msg1, char msg2)
{  // Update display of secure PIN messages
   int station;

   // already know that msg1 = MSG_PIN
   // need to determine what station this is for as offset from MSG_AT
   station = (int)msg2 - (int)MSG_AT;
   if ((station < 0) || (station > 8)) station=0;  // if not MSG_AT then default to station 0
   // NOTE space in format after %04d matches with extra space at end of message
   sprintf(&sys_message[MSG_PIN].msg[MSG_OFFSET+12], "%04d ", securePIN[station]);
}

void lcd_show_cursor(char lcd, char line, char pos)
{  // Show or hide the lcd cursor
   if (lcd >= LCDCOUNT)
   {  //rn_dispCursor(DevRN1600, RNDISP_CUROFF, 0);
   } else
   {
      //rn_dispCursor(DevRN1600, RNDISP_CURON, 0);
      //rn_dispGoto (DevRN1600, pos, line+lcd*2, 0);
   }
}
void reset_msg_timer(char how)
{  // setup for next message in time or show next message now (soon)
	if (how==NOW) msgtimer = MS_TIMER + 100 - SCROLLTIME;
   else if (how=NEXT) msgtimer = MS_TIMER;
}
void lcd_print(char lcd, char line, char* message)
{
    lputs(line+lcd*2, message);
}
void lputs(char line, char* msg)
{
   //char c;
   //rn_dispGoto(DevRN1600, 0, line, 0);
   //rn_dispPrintf (DevRN1600, 0, msg);
   //for (c=0; c<MSG_LEN; c++) rn_dispPutc(DevRN1600, msg[c], 0);
	printf("%s \n", msg);
}

void lcd_initialize()
{
}
/******************************************************************/
void initializeMessages(void)
{  // Setup messages for lcd display

   char j;

   // load ram messages with contents of rom
   for (j=0; j<MSGCOUNT; j++)
       strcpy(sys_message[j].msg, sysmsg[j]);

   // set station name messages
   buildStationNames();
}
/******************************************************************/
const char * const defaultStaName[] = { "MAIN", "STATION 1", "STATION 2", "STATION 3",
"STATION 4", "STATION 5", "STATION 6", "STATION 7", "SLAVE MAIN", "SYSTEM" };
void buildStationNames()
{
   char j, k, start, mybuf[MSG_LEN+1];

   // check if names are initialized
    if (param.names_init != 69)      // init flag
    {
      // use default names
	   for (j=0; j<SYSTEM_NAME; j++)     // SYSTEM_NAME always the last name
	   {
	      strcpy(param.station_name[j].name, defaultStaName[j]);
      }
      param.names_init = 69;
    }

   // build system name message
   strcpy(sys_message[MSG_SYSTEM_NAME].msg, sys_message[MSG_BLANK].msg); // blank it
   strcpy(mybuf, param.station_name[SYSTEM_NAME].name);
   start=(MSG_LEN - strlen(mybuf)) / 2;  // center it
	for (k=0; k<strlen(mybuf); k++)  // place in destination
     sys_message[MSG_SYSTEM_NAME].msg[k+start]=mybuf[k];

   // build station name messages
   for (j=0; j<SYSTEM_NAME; j++)     // SYSTEM_NAME always the last name
   {
     // clear out destinations
     strcpy(sys_message[MSG_AT+j].msg, sys_message[MSG_BLANK].msg);
     strcpy(sys_message[MSG_FROM+j].msg, sys_message[MSG_BLANK].msg);
     strcpy(sys_message[MSG_TO+j].msg, sys_message[MSG_BLANK].msg);

     strcpy(mybuf, "AT ");
     strcat(mybuf, param.station_name[j].name);
     // center message
     start = (MSG_LEN - strlen(mybuf)) / 2;
     // place in destination
     for (k=0; k<strlen(mybuf); k++)
          sys_message[MSG_AT+j].msg[k+start]=mybuf[k];

     strcpy(mybuf, "FROM ");
     strcat(mybuf, param.station_name[j].name);
     // center message
     start = (MSG_LEN - strlen(mybuf)) / 2;
     // place in destination
     for (k=0; k<strlen(mybuf); k++)
          sys_message[MSG_FROM+j].msg[k+start]=mybuf[k];

     strcpy(mybuf, "TO ");
     strcat(mybuf, param.station_name[j].name);
     // center message
     start = (MSG_LEN - strlen(mybuf)) / 2;
     // place in destination
     for (k=0; k<strlen(mybuf); k++)
          sys_message[MSG_TO+j].msg[k+start]=mybuf[k];

    }
}
char functionDefineStations()
{  // define the active stations
}
/******************************************************************/
void functionSetNames()
{  // edit main & remote station names
}

// time structure used in several of the following routines
//struct tm time;
void setCountMessage(void)
{  // store long numeric value transactionCount into message buffer
   char j, k, mybuf[MSG_LEN+1];
   strcpy(sys_message[MSG_TRANS_COUNT].msg, sys_message[MSG_BLANK].msg);
   ltoa(transactionCount(), mybuf);
   // stuff mybuf into center of message
   k=(MSG_LEN-strlen(mybuf))/2;
   for (j=0; j<strlen(mybuf); j++)
       sys_message[MSG_TRANS_COUNT].msg[j+k]=mybuf[j];
}
void setParValueMessage(char MSG_message, unsigned long par_value, char * par_units)
{  // store long numeric value into message buffer
   char j, k, mybuf[MSG_LEN+1];
   // blank out existing message
   strcpy(sys_message[MSG_message].msg, sys_message[MSG_BLANK].msg);
   // convert value to string and append units text
   ltoa(par_value, mybuf);
   strcat(mybuf, " ");
   strcat(mybuf, par_units);
   // place mybuf and units text into center of message
   k=(MSG_LEN-strlen(mybuf))/2;
   for (j=0; j<strlen(mybuf); j++)
       sys_message[MSG_message].msg[j+k]=mybuf[j];
}
void setTimeMessages(char message, unsigned long timedate)
{  // fills in the time and date into the sys_message messages
   // timedate in seconds

   struct tm time;
   //char bad;
   //bad=63;  // question mark
   // convert unsigned long to time structure
   mktm(&time, timedate);

   //if ( tm_rd( &time ) )
   //{       // error reading date/time
   //} else
   if (message == MSG_DATE_TIME)
   {
      // fill in the time values
/*    FOR ORIGINAL TIME MESSAGE
      sys_message[MSG_TIME].msg[MSG_OFFSET + 7] = 48 + (time.tm_hour / 10);
      sys_message[MSG_TIME].msg[MSG_OFFSET + 8] = 48 + (time.tm_hour % 10);
      sys_message[MSG_TIME].msg[MSG_OFFSET + 10] = 48 + (time.tm_min / 10);
      sys_message[MSG_TIME].msg[MSG_OFFSET + 11] = 48 + (time.tm_min % 10);
      //sys_message[MSG_TIME].msg[13] = 48 + (time.tm_sec / 10);
      //sys_message[MSG_TIME].msg[14] = 48 + (time.tm_sec % 10);
*/

      // and second message
      sys_message[MSG_DATE_TIME].msg[MSG_OFFSET + 10] = 48 + (time.tm_hour / 10);
      sys_message[MSG_DATE_TIME].msg[MSG_OFFSET + 11] = 48 + (time.tm_hour % 10);
      sys_message[MSG_DATE_TIME].msg[MSG_OFFSET + 13] = 48 + (time.tm_min / 10);
      sys_message[MSG_DATE_TIME].msg[MSG_OFFSET + 14] = 48 + (time.tm_min % 10);

      // fill in the date values
//      sys_message[MSG_DATE].msg[MSG_OFFSET + 7] = 48 + (time.tm_mon / 10);
//      sys_message[MSG_DATE].msg[MSG_OFFSET + 8] = 48 + (time.tm_mon % 10);
//      sys_message[MSG_DATE].msg[MSG_OFFSET + 10] = 48 + (time.tm_mday / 10);
//      sys_message[MSG_DATE].msg[MSG_OFFSET + 11] = 48 + (time.tm_mday % 10);
//      sys_message[MSG_DATE].msg[MSG_OFFSET + 13] = 48 + ((time.tm_year / 10) % 10);
//      sys_message[MSG_DATE].msg[MSG_OFFSET + 14] = 48 + (time.tm_year % 10);

      // and second message
      sys_message[MSG_DATE_TIME].msg[MSG_OFFSET + 1] = 48 + (time.tm_mon / 10);
      sys_message[MSG_DATE_TIME].msg[MSG_OFFSET + 2] = 48 + (time.tm_mon % 10);
      sys_message[MSG_DATE_TIME].msg[MSG_OFFSET + 4] = 48 + (time.tm_mday / 10);
      sys_message[MSG_DATE_TIME].msg[MSG_OFFSET + 5] = 48 + (time.tm_mday % 10);
      sys_message[MSG_DATE_TIME].msg[MSG_OFFSET + 7] = 48 + ((time.tm_year / 10) % 10);
      sys_message[MSG_DATE_TIME].msg[MSG_OFFSET + 8] = 48 + (time.tm_year % 10);
   }
   else if (message == MSG_PAR_VALUE)
   {
      // fill in the time values
      //sys_message[MSG_PAR_VALUE].msg = "    HH:MM:SS    ";
      strcpy(sys_message[MSG_PAR_VALUE].msg, sys_message[MSG_BLANK].msg);
      sys_message[MSG_PAR_VALUE].msg[MSG_OFFSET + 4] = 48 + (time.tm_hour / 10);
      sys_message[MSG_PAR_VALUE].msg[MSG_OFFSET + 5] = 48 + (time.tm_hour % 10);
      sys_message[MSG_PAR_VALUE].msg[MSG_OFFSET + 6] = ':';
      sys_message[MSG_PAR_VALUE].msg[MSG_OFFSET + 7] = 48 + (time.tm_min / 10);
      sys_message[MSG_PAR_VALUE].msg[MSG_OFFSET + 8] = 48 + (time.tm_min % 10);
      sys_message[MSG_PAR_VALUE].msg[MSG_OFFSET + 9] = ':';
      sys_message[MSG_PAR_VALUE].msg[MSG_OFFSET + 10] = 48 + (time.tm_sec / 10);
      sys_message[MSG_PAR_VALUE].msg[MSG_OFFSET + 11] = 48 + (time.tm_sec % 10);
   }

}
void setFlagMessage(char flag)
{
   // flag message is on or off
   if (flag)
   {  sys_message[MSG_FLAG_SET].msg[MSG_OFFSET + 12]=78;          // ON
      sys_message[MSG_FLAG_SET].msg[MSG_OFFSET + 13]=32;
   } else
   {  sys_message[MSG_FLAG_SET].msg[MSG_OFFSET + 12]=70;          // OFF
      sys_message[MSG_FLAG_SET].msg[MSG_OFFSET + 13]=70;
   }
}
void show_extended_data()
{  // display watchdog, powerlow resets and versions on the lcd
   int i;
   int ver;
   char msgbuf[30];

   //lcd_print(SYSTEMLCD, 0, " REMOTE RESETS  ");
   //strcpy(msgbuf, "#n WD:___ PL:___");
	lcd_DispText("Remote Vers ", 0, 0, MODE_NORMAL);
   for (i=1; i<8; i++)
   {
      if ( (1 << (i-1)) & (int)availableDevices )  // if i is a remote devAddr
      {  ver = extendedData[i][2];          // move to int as DC treats char different - could be a bug???
      	sprintf(msgbuf, "4.%02d; ", ver);
         lcd_DispText(msgbuf,0, 0, MODE_NORMAL);
        //lcdcmd(LINE1);
        //msgbuf[1]=48+i;  // remote #
        //msgbuf[6]=48+(extendedData[i][0] / 100);
        //msgbuf[7]=48+((extendedData[i][0] / 10) % 10);
        //msgbuf[8]=48+(extendedData[i][0] % 10);
        //msgbuf[13]=48+(extendedData[i][1] / 100);
        //msgbuf[14]=48+((extendedData[i][1] / 10) %10);
        //msgbuf[15]=48+(extendedData[i][1] % 10);
        //lputs(0, msgbuf);
      }
   }
   // msDelay(1200);
   lcd_DispText("\n", 0, 0, MODE_NORMAL);
}

unsigned long transaction_count;
static unsigned long transCountXaddr;			// physical memory address
/*******************************************************/
unsigned long transactionCount() { return transaction_count; }
/*******************************************************/
nodebug void incrementCounter()
{
   transaction_count++;
   setCountMessage();
   root2xmem( transCountXaddr, &transaction_count, 4);
   root2xmem( transCountXaddr+4, &statistics, sizeof(statistics));  // also save stats to xmem
   syncTransCount(); // send to the slave
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
   xmem2root( &statistics, transCountXaddr+4, sizeof(statistics));  // also read stats

   if (transaction_count < 0 || transaction_count > 10000000)
   { resetTransactionCount(0); }

   setCountMessage();
   syncTransCount(); // send to the slave
}
/******************************************************************/
void resetTransactionCount(unsigned long value)
{  transaction_count=value;
   setCountMessage();
   root2xmem( transCountXaddr, &transaction_count, 4);
   syncTransCount(); // send to the slave
}
void resetStatistics(void)
{
	statistics.trans_in=0;
	statistics.trans_out=0;
	statistics.deliv_alarm=0;
	statistics.divert_alarm=0;
	statistics.cic_lift_alarm=0;
   root2xmem( transCountXaddr+4, &statistics, sizeof(statistics));
}
/******************************************************************/
// MAKING DESIGN ASSUMPTION --> MNU_xxx value is the same as the index in menu[]
// define menu item numbers
enum MenuItems {
	   MNU_MARRIVAL_BEEP,
	   MNU_RARRIVAL_BEEP,
	   MNU_ARETURN_BEEP,
      MNU_VISUAL_ALERT,
	   MNU_ALARM_SOUND,
	   MNU_PURGE,
	   MNU_ALARM_RESET,
      MNU_SETACTIVEMAIN,
	   MNU_SET_CLOCK,
	   MNU_COMM_STAT,
	   MNU_MSTACKING,
	   MNU_RSTACKING,
	   MNU_RESET_COUNT,
	   MNU_SET_STANAMES,
      MNU_SET_PHONENUMS,
	   MNU_DIVERTER_CFG,
	   MNU_SETPASSWORD,
      MNU_CLEARPASSWORDS,
	   MNU_SETTIMEOUT,
	   MNU_HEADDIVERTER,
	   MNU_AUTO_PURGE,
	   MNU_AUTOPUR_TIMER,
      MNU_SETASSLAVE,
	   MNU_DEF_STATIONS,
      MNU_ADVANCED_SETUP,
      MNU_RESET,
      MNU_ADMIN_FEATURES,
      MNU_MAINT_FEATURES,
      MNU_SYSTEM_CONFIG,
      MNU_VIEW_LOGS,
      MNU_VIEW_TLOG, // Transaction log
      MNU_VIEW_SLOG, // Summary log
      MNU_VIEW_ELOG, // Event log
      MNU_VIEW_ALOG, // Alarm log
      MNU_AUTO_RETURN,
      MNU_DNLD_TLOG,
      MNU_UPLD_TLOG,
      MNU_CHECK_SFLASH,
      MNU_ANALYZE_LOG,
      MNU_SUBSTA_ADDR, // for remote to select destination (main or sattelite)
      MNU_BLOWER_TYPE,
      MNU_PORT_MAPPING,
      MNU_LCD_CALIBRATE,
      MNU_SET_SYSNUM,
      MNU_SHOW_IPADDR,
      MNU_SECURE_SETUP,
      MNU_SECURE_ENABLE,
      MNU_CARDID_ENABLE,
      MNU_CARD_FORMAT,
      MNU_SERVER_ADDRESS,
      MNU_SYNC_INTERVAL,
      //MNU_SECURE_R9MAX,
      //MNU_SECURE2_L3DIGITS,
      //MNU_SECURE2_R9MIN,
      //MNU_SECURE2_R9MAX,
      MNU_SHOW_INPUTS,
      MNU_TUBE_DRYER,
      MNU_ENAB_DRYER_OPT,
      MNU_RESYNC_USERS,
      MNU_CHECK_SVR_NOW,
      MNU_LCD_ORIENTATION,
      MNU_ENABLE_PAUSE,
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
 MNU_MARRIVAL_BEEP,  "MAIN AUDIBLE ALERT",MNU_FLAG, &param.mainAudibleAlert, 0, 1, "",
 MNU_RARRIVAL_BEEP,  "REMOTE AUDIBLE ALRT",MNU_FLAG, &param.remoteAudibleAlert, 0, 1, "",
 MNU_ARETURN_BEEP,   "AUTO RETURN AUDIBLE",MNU_FLAG, &param.areturn_arrive_alert, 0, 1, "",
 MNU_VISUAL_ALERT,   "MAIN VISUAL ALERT",  MNU_FLAG, &param.mainVisualAlert, 0, 1, "",
 MNU_ALARM_SOUND,    "SILENCE ALARM",      MNU_FLAG, &alarm_silenced_flag, 0, 0, "",
 MNU_PURGE,          "MANUAL PURGE",       MNU_OTHR, &NOch, 0, 0, "",
 MNU_ALARM_RESET,    "RESET ALARM",        MNU_OTHR, &NOch, 0, 0, "",
 MNU_SETACTIVEMAIN,  "SET ACTIVE MAIN",    MNU_OTHR, &activeMainHere, 0, 0, "",
 MNU_SET_CLOCK, 		"SET CLOCK",          MNU_CLK,  &NOch, 0, 0, "",
 MNU_COMM_STAT,      "SHOW COMMUNICATIONS",MNU_OTHR, &NOch, 0, 0, "",
 MNU_MSTACKING,      "STACK AT MAIN",      MNU_FLAG, &param.stacking_ok[0], 0, 1, "",
 MNU_RSTACKING,      "STACK AT REMOTES",   MNU_FLAG, &param.stacking_ok[1], 0, 1, "",
 MNU_RESET_COUNT,    "SET TRANS COUNT",    MNU_OTHR, &NOch, 0, 0, "",
 MNU_SET_STANAMES,   "SET STATION NAMES",  MNU_OTHR, &NOch, 0, 0, "",
 MNU_SET_PHONENUMS,  "SET PHONE NUMBERS",  MNU_OTHR, &NOch, 0, 0, "",
 MNU_DIVERTER_CFG,   "USE LOCAL DIVERTER", MNU_FLAG, &param.localDiverter, 0, 1, "",
 MNU_SETPASSWORD,    "SET PASSWORD",       MNU_OTHR, &NOch, 0, 0, "",
 MNU_CLEARPASSWORDS, "CLEAR PASSWORDS",    MNU_OTHR, &NOch, 0, 0, "",
 MNU_SETTIMEOUT,     "DELIVERY TIMEOUT",   MNU_VAL,  (char *)&param.deliveryTimeout, 10, 240, "SEC",
 MNU_HEADDIVERTER,   "USE HEAD DIVERTER",  MNU_FLAG, &param.headDiverter, 0, 1, "",
 MNU_AUTO_PURGE,     "AUTOMATIC PURGE",    MNU_OTHR, &NOch, 0, 0, "",
 MNU_AUTOPUR_TIMER,  "AUTO PURGE TIMER",   MNU_VAL,  (char *)&param.autoPurgeTimer, 10, 200, "SEC",
 MNU_SETASSLAVE,     "SET AS SLAVE DEVICE",MNU_FLAG, &param.slaveController, 0, 1, "",
 MNU_DEF_STATIONS,   "SET ACTIVE STATIONS",MNU_OTHR, &NOch, 0, 0, "",
 MNU_ADVANCED_SETUP, "Only in old menu",   MNU_FLAG, &diagnostic_mode, 0, 0, "",
 MNU_RESET,          "RESTART SYSTEM",     MNU_OTHR, &NOch, 0, 0, "",
 MNU_ADMIN_FEATURES, "ADMIN FEATURES",     MNU_OTHR, &NOch, 0, 0, "",
 MNU_MAINT_FEATURES, "MAINT FEATURES",     MNU_OTHR, &NOch, 0, 0, "",
 MNU_SYSTEM_CONFIG,  "SYSTEM CONFIG",      MNU_OTHR, &NOch, 0, 0, "",
 MNU_VIEW_LOGS,      "VIEW LOGS",          MNU_OTHR, &NOch, 0, 0, "",
 MNU_VIEW_TLOG,      "VIEW TRANSACTIONS",  MNU_OTHR, &NOch, 0, 0, "",
 MNU_VIEW_SLOG,      "VIEW SUMMARY LOG",   MNU_OTHR, &NOch, 0, 0, "",
 MNU_VIEW_ELOG,      "VIEW EVENT LOG",     MNU_OTHR, &NOch, 0, 0, "",
 MNU_VIEW_ALOG,      "VIEW RECENT ALARMS", MNU_OTHR, &NOch, 0, 0, "",
 MNU_AUTO_RETURN,    "AUTO RETURN TIMER",  MNU_VAL,  (char *)&param.autoReturnTimer, 0, 60, "MIN",
 MNU_DNLD_TLOG,      "DOWNLOAD TRANS LOG", MNU_OTHR, &NOch, 0, 0, "",
 MNU_UPLD_TLOG,      "UPLOAD TRANS LOG",   MNU_OTHR, &NOch, 0, 0, "",
 MNU_CHECK_SFLASH,   "CHECK SERIAL FLASH", MNU_OTHR, &NOch, 0, 0, "",
 MNU_ANALYZE_LOG,    "ANALYZE EVENT LOG",  MNU_OTHR, &NOch, 0, 0, "",
 MNU_SUBSTA_ADDR,    "SUB-STA ADDRESSING", MNU_FLAG, &param.subStaAddressing, 0, 1, "",
 MNU_BLOWER_TYPE,    "SET BLOWER TYPE",    MNU_OTHR, &param.blowerType, 0, 0, "",
 MNU_PORT_MAPPING,   "SET PORT MAPPING",   MNU_OTHR, &NOch, 0, 0, "",
 MNU_LCD_CALIBRATE,  "CALIBRATE SCREEN",   MNU_OTHR, &NOch, 0, 0, "",
 MNU_SET_SYSNUM,     "SET SYSTEM NUMBER",  MNU_VAL,  &param.systemNum, 1, 20, "",
 MNU_SHOW_IPADDR,    "SHOW IP ADDRESS",    MNU_OTHR, &NOch, 0, 0, "",
 MNU_SECURE_SETUP,   "SETUP SECURE TRANS", MNU_OTHR, &NOch, 0, 0, "",
 MNU_SECURE_ENABLE,  "ENABLE SECURE TRANS",MNU_FLAG, &param.secureEnabled, 0, 1, "",
 MNU_CARDID_ENABLE,  "ENABLE CARD CHECK",  MNU_FLAG, &param.cardCheckEnabled, 0, 1, "",
 MNU_CARD_FORMAT,    "CARD SIZE 26 OR 40", MNU_VAL,  &param.cardFormat, 26, 40, "BIT",
 MNU_SERVER_ADDRESS, "SERVER ADDRESS",     MNU_OTHR, &NOch, 0, 0, "",
 MNU_SYNC_INTERVAL,  "SYNC INTERVAL",      MNU_VAL,  &param.syncInterval, 0, 255, "MIN",
// MNU_SECURE_R9MAX,   "CARD1 RIGHT 9 MAX",  MNU_VAL,  (char *)&param.cardR9Max, 200000000, 618000000, "ID",
// MNU_SECURE2_L3DIGITS,"CARD2 LEFT 3 DIGITS",MNU_VAL, &param.card2L3Digits, 155, 155, "ID",
// MNU_SECURE2_R9MIN,   "CARD2 RIGHT 9 MIN", MNU_VAL,  (char *)&param.card2R9Min, 000000000, 999999999, "ID",
// MNU_SECURE2_R9MAX,   "CARD2 RIGHT 9 MAX", MNU_VAL,  (char *)&param.card2R9Max, 000000000, 999999999, "ID",
 MNU_SHOW_INPUTS,    "SHOW INPUTS",        MNU_OTHR, &NOch, 0, 0, "",
 MNU_TUBE_DRYER,     "RUN TUBE DRYER",     MNU_OTHR, &NOch, 0, 0, "",
 MNU_ENAB_DRYER_OPT, "ENABLE TUBE DRYER",  MNU_FLAG, &param.enableTubeDryOpt, 0, 1, "",
 MNU_RESYNC_USERS,   "SYNCHRONIZE USERS",  MNU_OTHR, &NOch, 0, 0, "",
 MNU_CHECK_SVR_NOW,  "CHECK SERVER NOW",   MNU_OTHR, &NOch, 0, 0, "",
 MNU_LCD_ORIENTATION,"ORIENT LANDSCAPE",   MNU_FLAG, &param.orientLandscape, 0, 1, "",
 MNU_ENABLE_PAUSE,   "ENABLE PAUSE",       MNU_FLAG, &param.pauseEnabled, 0, 1, "",
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
      menuOrder[i++]=MNU_ALARM_SOUND;
      menuOrder[i++]=MNU_PURGE;
      menuOrder[i++]=MNU_ALARM_RESET;
   	break;
   case 1: // Basic User
      if ((slaveAvailable || param.slaveController) && (param.subStaAddressing==FALSE)) menuOrder[i++]=MNU_SETACTIVEMAIN;
      if (param.point2point==FALSE)
      {  // no purge
	      menuOrder[i++]=MNU_AUTO_PURGE;
   	   menuOrder[i++]=MNU_PURGE;
      }
      menuOrder[i++]=MNU_VIEW_LOGS;
      if (param.slaveController == 0)
      {  // these menu items not allowed at slave
			//// menuOrder[i++]=MNU_ADMIN_FEATURES;
	      menuOrder[i++]=MNU_MAINT_FEATURES;
      }
		menuOrder[i++]=MNU_ADMIN_FEATURES; // do allow admin for slave
      menuOrder[i++]=MNU_SYSTEM_CONFIG;
   	break;
   case 2: // Administrator
   	if (param.enableTubeDryOpt) menuOrder[i++]=MNU_TUBE_DRYER;
      menuOrder[i++]=MNU_ENABLE_PAUSE;
      menuOrder[i++]=MNU_MARRIVAL_BEEP;
      menuOrder[i++]=MNU_VISUAL_ALERT;
      menuOrder[i++]=MNU_RARRIVAL_BEEP;
      menuOrder[i++]=MNU_ARETURN_BEEP;
      menuOrder[i++]=MNU_MSTACKING;
      menuOrder[i++]=MNU_RSTACKING;
      menuOrder[i++]=MNU_AUTOPUR_TIMER;
      menuOrder[i++]=MNU_AUTO_RETURN;
	   //   menuOrder[i++]=MNU_DNLD_TLOG;
      menuOrder[i++]=MNU_UPLD_TLOG;
      menuOrder[i++]=MNU_SET_CLOCK;
      menuOrder[i++]=MNU_RESET_COUNT;
      menuOrder[i++]=MNU_SET_STANAMES;
      menuOrder[i++]=MNU_SET_PHONENUMS;
      menuOrder[i++]=MNU_SETPASSWORD;
   	break;
   case 3: // Maintenance
      menuOrder[i++]=MNU_SETTIMEOUT;
      if (param.point2point==FALSE)
      {  // no purge
	      menuOrder[i++]=MNU_AUTO_PURGE;
	      menuOrder[i++]=MNU_PURGE;
      }
      menuOrder[i++]=MNU_CHECK_SVR_NOW;
      menuOrder[i++]=MNU_SETPASSWORD;
      //menuOrder[i++]=MNU_DIV_TIMEOUT;
      //menuOrder[i++]=MNU_LIFT_TIMEOUT;
   	break;
   case 4: // System configuration (Colombo only)
      menuOrder[i++]=MNU_ENAB_DRYER_OPT;
      menuOrder[i++]=MNU_DIVERTER_CFG;
      menuOrder[i++]=MNU_HEADDIVERTER;
      menuOrder[i++]=MNU_BLOWER_TYPE;
      //menuOrder[i++]=MNU_REMOTEDIVERTER;
      menuOrder[i++]=MNU_DEF_STATIONS;
      menuOrder[i++]=MNU_SET_SYSNUM;
      if (param.point2point==FALSE)
      {  // no slave
	      menuOrder[i++]=MNU_SETASSLAVE;
      }
      menuOrder[i++]=MNU_SUBSTA_ADDR;
      menuOrder[i++]=MNU_SECURE_SETUP;
      menuOrder[i++]=MNU_PORT_MAPPING;
      menuOrder[i++]=MNU_SHOW_IPADDR;
      menuOrder[i++]=MNU_COMM_STAT;
      menuOrder[i++]=MNU_SETPASSWORD;
      menuOrder[i++]=MNU_CLEARPASSWORDS;
      menuOrder[i++]=MNU_CHECK_SFLASH;
      menuOrder[i++]=MNU_ANALYZE_LOG;
      menuOrder[i++]=MNU_LCD_ORIENTATION;
      menuOrder[i++]=MNU_LCD_CALIBRATE;
      menuOrder[i++]=MNU_SHOW_INPUTS;
      menuOrder[i++]=MNU_RESET;
   	break;
   case 5: // view logs
      menuOrder[i++]=MNU_VIEW_SLOG; // transaction summary
      menuOrder[i++]=MNU_VIEW_TLOG; // transaction log
      //menuOrder[i++]=MNU_VIEW_ELOG; // event log
      menuOrder[i++]=MNU_VIEW_ALOG; // alarm log
   	break;
   case 6: // secure card setup
      menuOrder[i++]=MNU_SECURE_ENABLE;
		menuOrder[i++]=MNU_CARDID_ENABLE;
      menuOrder[i++]=MNU_SERVER_ADDRESS;
      menuOrder[i++]=MNU_SYNC_INTERVAL;
      menuOrder[i++]=MNU_RESYNC_USERS;
      menuOrder[i++]=MNU_CARD_FORMAT;
      //menuOrder[i++]=MNU_SECURE_R9MAX;
      //menuOrder[i++]=MNU_SECURE2_L3DIGITS;
      //menuOrder[i++]=MNU_SECURE2_R9MIN;
      //menuOrder[i++]=MNU_SECURE2_R9MAX;
   	break;
	}
   menuOrder[i]=MNU_LAST;

}
//#define PAGE_SIZE 5
const char * const title[7] = { "ALARM MENU", "MENU", "ADMIN MENU", "MAINTENANCE MENU", "SYS CONFIG MENU", "LOGS MENU","SECURE TRANS MENU" };
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
      if (menu[menu_idx].item == MNU_ALARM_SOUND) alarm(ON); // update alarm if needed
      break;
   case MNU_TIME:
      //ptr_ulong = (unsigned long *)menu[menu_idx].parval;
      //par_changed |= functionSetTime(menu[menu_idx].item, menu_idx, ptr_ulong);
      break;
   case MNU_CLK:
		lcd_setClock();      //clock_in_seconds = SEC_TIMER;
      syncDateAndTime(); // send to slave
      break;
   case MNU_OTHR:
      switch (menu[menu_idx].item)
      {
      case MNU_SERVER_ADDRESS:
			parChanged = lcd_editServerName();
         break;
      case MNU_CHECK_SVR_NOW:
      	pingTimer = SEC_TIMER - 1; // force check now
         *menu_level=99; // exit menu
      	break;
      case MNU_RESYNC_USERS:
      	work=FALSE;
			lcd_SelectChoice(&work, menu[menu_idx].msg1, "YES", "NO");
         if (work)
         {  // yes please resync
	         UID_ResyncAll();
         }
         *menu_level=99; // exit menu
         break;
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
      case MNU_SECURE_SETUP:
         *menu_level=6;  // Secure card setup items
         break;
      case MNU_BLOWER_TYPE:
      	if (*menu[menu_idx].parval==blwrType_STD) work=TRUE; else work=FALSE;
			parChanged=lcd_SelectChoice(&work, menu[menu_idx].msg1, "STD", "APU");
         if (parChanged) *menu[menu_idx].parval = work ? blwrType_STD : blwrType_APU;
      	break;
      case MNU_ALARM_RESET:           // alarm reset
      	work=FALSE;
			lcd_SelectChoice(&work, menu[menu_idx].msg1, "YES", "NO");
         if (work)
         {  // yes please reset, added here to make sure alarm goes off at slave
         	system_state=CANCEL_STATE;
            alarm(OFF);
         }
         *menu_level=99; // exit menu
         break;
      case MNU_SETACTIVEMAIN:
			lcd_SelectChoice(menu[menu_idx].parval, menu[menu_idx].msg1,
         	param.station_name[0].name, param.station_name[SLAVE].name);
         setActiveMain(*menu[menu_idx].parval);
      	break;
      case MNU_PURGE:                  // purge system
         purgeSystem(PURGE_MANUAL);    // 0 = manual; 1 = auto
         break;
      case MNU_AUTO_PURGE:             // purge system
         purgeSystem(PURGE_AUTOMATIC); // 0 = manual; 1 = auto
         break;
      case MNU_TUBE_DRYER:             // purge system
         purgeSystem(PURGE_DRYING);    // 2 = tube drying
         break;
      case MNU_RESET_COUNT:
      	maxDigits=6;
         temp_ulong = transactionCount();
	      if (lcd_GetNumberEntry(menu[menu_idx].msg1, &temp_ulong,
         		 0, 999999, menu[menu_idx].units, maxDigits, 0))
            resetTransactionCount(temp_ulong);
         break;
      case MNU_SET_STANAMES:        // set station names
	      parChanged=lcd_editStationNames();
         if (parChanged) setActiveMainMsg();  // reset rcv @ message if needed
         break;
      case MNU_SET_PHONENUMS:       // set phone numbers
      	parChanged=lcd_editPhoneNums();
      	break;
      case MNU_PORT_MAPPING:			// define the remote port/station mapping
      	parChanged=lcd_editPortMapping(menu[menu_idx].msg1);
         break;
      case MNU_DEF_STATIONS:        // define active stations
         parChanged=lcd_defineActiveStations();
         break;
      case MNU_SETPASSWORD:        // set new password
         // get the PIN for the given menu level
         if (*menu_level==2) parChanged |= lcd_getPin(param.adminPassword, "SET ADMIN PASSWORD");
         else if (*menu_level==3) parChanged |= lcd_getPin(param.maintPassword, "SET MAINT PASSWORD");
         else if (*menu_level==4) parChanged |= lcd_getPin(param.cfgPassword, "SET CONFIG PASSWORD");
         break;
      case MNU_CLEARPASSWORDS:    // reset passwords
      	param.adminPassword[0]=0;
         param.maintPassword[0]=0;
         // param.cfgPassword[0]=0; // SHOULD NOT BE NECESSARY
         parChanged=TRUE;
         break;
      case MNU_COMM_STAT:
         show_comm_test(1); // wait for button press
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
      case MNU_DNLD_TLOG:
        // how many days of the log should be downloaded?
        temp_ulong=0;
 	     if (lcd_GetNumberEntry("DOWNLOAD LOG, Last", &temp_ulong, 0, 9999, "Days", 4, 1))
	      {  // Download last x days of the log
         	if (temp_ulong == 0) temp_ulong = 9999;  // essentially the whole log
	        	downloadTransLog((int)temp_ulong);
         }
         break;
      case MNU_UPLD_TLOG:
         // give some status
	      lcd_drawScreen(3, "UPLOAD LOG");
	      lcd_Font("24");
	      lcd_DispText("In progress ...", 40, 80, MODE_NORMAL); // show the default name

      	uploadTransLog(0);
         break;
      case MNU_CHECK_SFLASH:
      	checkSerialFlash();
         break;
      case MNU_ANALYZE_LOG:
      	analyzeEventLog();
         break;
      case MNU_LCD_CALIBRATE:
			lcd_Calibrate();
			while ( lcd_Origin(0,0) != lcd_SUCCESS ) msDelay(1000); // wait for done calibrate
      	break;
      case MNU_SHOW_IPADDR:
      	show_ip_address();
      	break;
      case MNU_SHOW_INPUTS:
      	lcd_show_inputs();
         break;
      case MNU_RESET:
         // go into long loop to allow watchdog reset
         clock_in_seconds = SEC_TIMER;
         lcd_drawScreen(3, menu[menu_idx].msg1);
	      lcd_Font("24");
	      lcd_DispText("SYSTEM RESTARTING", 10, 85, MODE_NORMAL);
         #asm
            ipset 3          ; disable interrupts so that the periodic ISR doesn't hit the watchdog.
            ld a,0x53        ; set the WD timeout period to 250 ms
            ioi ld (WDTCR),a
         #endasm
         while (SEC_TIMER - clock_in_seconds < 5);
	      lcd_DispText("UNABLE TO RESET", 10, 115, MODE_NORMAL);
         msDelay(1000);
         break;
      }
	}
   return parChanged;
}
/******************************************************************/

void setActiveMain(char setToMaster)
{
	if (setToMaster) param.activeMain = MASTER;
   else             param.activeMain = SLAVE;

   if (param.slaveController)
   {  // set flag to send new status
   	// FORCE OFF FOR NOW
      //if (param.activeMain==MASTER) slaveReturnStatus |= 0x04;
      //else slaveReturnStatus |= 0x02;
   }
   setActiveMainMsg();

}
void setActiveMainMsg()
{  // setup active main message shown on lcd
	char i;
	char mybuf[20];
	char start;

   // Only show Rcv@ if slave detected and sub station addressing is off
   if (0) // FORCE OFF ((slaveAvailable || param.slaveController) && (param.subStaAddressing==FALSE))
   {
	   //strcpy(sys_message[MSG_SDEFAULT2].msg, "  RCV @ ");
	   strcpy(mybuf, "RCV @ ");
	   if (param.activeMain == MASTER) strncat(mybuf, param.station_name[0].name,10);
	   else                            strncat(mybuf, param.station_name[8].name,10);

	   // center message
	   start = (MSG_LEN - strlen(mybuf)) / 2;
	   // place in destination
	   // leading spaces
	   for (i=0; i<start; i++)
	      sys_message[MSG_SDEFAULT2].msg[i]=32;
	   // message
	   for (i=0; i<strlen(mybuf); i++)
	      sys_message[MSG_SDEFAULT2].msg[i+start]=mybuf[i];
	   // trailing spaces
	    for (i=start+strlen(mybuf); i<MSG_LEN; i++)
	      sys_message[MSG_SDEFAULT2].msg[i]=32;
	    // end of string
	    sys_message[MSG_SDEFAULT2].msg[MSG_LEN]=0;
   } else
   {  // go back to system startup default
   	strcpy(sys_message[MSG_SDEFAULT2].msg, sysmsg[MSG_SDEFAULT2]);
   }

}
/******************************************************************/

// set clock functions
char functionSetTime(char what_timer, char index, unsigned long *time_val)
{  // what_timer is the base menu item

}
char functionSetParValue(char menu_item, unsigned long * par_value,
                         unsigned long par_min, unsigned long par_max,
                         char * par_units)
{  // allow changing an unsigned long parameter value

}
/******************************************************************/
// define purge system states
#define  PURGE_ARE_YOU_SURE      0x20
#define  PURGE_GET_DIRECTION     0x21
#define  PURGE_GET_STATION       0x22
#define  PURGE_SET_DIVERTER      0x23
#define  PURGE_WAIT_DIVERTER     0x24
#define  PURGE_RUN               0x25
#define  PURGE_GET_HEADDIVERTER  0X26
#define  PURGE_RUN_DONE          0x27
#define  PURGE_RUN_AUTO          0x28
#define  PURGE_RUN_DRYING        0x29
#define  PURGE_EXIT              0x30

void purgeSystem(char purge_mode)
{
   // Input parameter purge_mode is manual (0) or automatic (1) or tube drying (2)
   // Definitions for calling are PURGE_MANUAL, PURGE_AUTOMATIC, PURGE_DRYING

   struct iomessage command, response;
   struct UDPmessageType UDPmessage;
   char purge_state, new_state, k, station_msg;
   char mymessage[17];
   char i;  // loop counter
   char remote_data_p[5];
   char first_time_here;
   char align_headdiverter;
   char blower_purge_state;   // used to tell headdiverter blower what to do
   char first_purge_state;    // initial state of the purge mode
   char auto_station;
   char auto_blower;
   char MSG_direction;			// to indicate if auto purge is TO or FROM station
   char title[15];
   int button; // from color lcd
   int col;
   unsigned long ms_autoPurgeTimer;
   ms_autoPurgeTimer = param.autoPurgeTimer * 1000ul;  // convert seconds to milliseconds
	echoLcdToTouchscreen=FALSE;  // Touchscreen has own messages, don't echo from lcd

   eventlog.start_tm=SEC_TIMER;

   // if auto purge, build up for control loop
   auto_station = 1;  // set first auto purge station
   state_timer=MS_TIMER;
   if (purge_mode==PURGE_AUTOMATIC)
   {  //
	   eventlog.status=ESTS_AUTOPURGE;
      //first_purge_state=PURGE_SET_DIVERTER;
      first_purge_state=PURGE_ARE_YOU_SURE;
      // operation depends on headdiverter config
       if (param.headDiverter==TRUE)
       {
           auto_blower=blwrPRS;  // always use pressure
           MSG_direction=MSG_TO;
           align_headdiverter=1;  // align main
           systemStation=0;    // no remote station
           systemStationb=0;
           system_direction=DIR_SEND;
       }
       else
       {
           auto_blower=blwrVAC;  // always use vacuum
           MSG_direction=MSG_FROM;
           align_headdiverter=0;   // no headdiverter
           systemStationb=firstBit(STATION_SET);
           systemStation=bit2station(systemStationb);
           system_direction=DIR_RETURN;
       }
       strcpy(title, "AUTO PURGE");
   }
   else if (purge_mode==PURGE_DRYING)
   {
	   eventlog.status=ESTS_MANPURGE;
	   // set entry state based on if headdiverter is used
	   if (param.headDiverter==TRUE) first_purge_state=PURGE_GET_HEADDIVERTER;
	   else first_purge_state=PURGE_GET_STATION;
      MSG_direction=MSG_FROM;
	   system_direction = DIR_SEND;       // undefined
	   align_headdiverter = 2;              // undefined (align remote)
	   strcpy(title, "RUN TUBE DRYER");  // 14 characters max
   }
   else // must be PURGE_MANUAL
   {
	   eventlog.status=ESTS_MANPURGE;
	   // set entry state based on if headdiverter is used
	   if (param.headDiverter==TRUE) first_purge_state=PURGE_GET_HEADDIVERTER;
	   else first_purge_state=PURGE_GET_STATION;
	   system_direction = DIR_SEND;       // undefined
	   align_headdiverter = 2;              // undefined (align remote)
	   strcpy(title, "PURGE");
   }
   purge_state = first_purge_state;
   new_state = purge_state;
   blower_purge_state = IDLE_STATE;   // remote blower idle
   first_time_here = TRUE;            // use to show state message once per state change
   station_msg = 1;                   // anything
   //eventlog.source_sta
   eventlog.dest_sta=0;  // initial dest
   eventlog.flags=0;		 // initial dest(s)


   inUse(FLASH, STATION_SET);

   // send command to tell remotes
   command.devAddr=ALL_DEVICES;
   command.command=SET_OUTPUTS;
   command.station=systemStation;
   command.data[devInUseSolid]=0;
   command.data[devInUseFlash]=0;
   command.data[devAlert]=0;
   command.data[devAlarm]=1; // 1=purge
   send_command(command);
   // display initial messages
   //if (purge_mode) message_add(TRANSLCD, MSG_FUNCTION, MSG_AUTO_PURGE, ONESHOTA);
   //else            message_add(TRANSLCD, MSG_FUNCTION, MSG_PURGE, ONESHOTA);
   //message_add(SYSTEMLCD, MSG_BLANK, MSG_BLANK, ONESHOT);
   // Setup color lcd display and buttons
	lcd_drawScreen(5, title);
	lcd_Font("24");

   while ((purge_state != PURGE_EXIT) && !secTimeout(menu_timeout) )
   {
      hitwd();                // hit the watchdog timer
      showactivity();
      check_diverter();
      processBlower();
#ifdef USE_TCPIP
	   tcp_tick(NULL);
	   if (getUDPcommand(&UDPmessage) > 0) processUDPcommand(UDPmessage);
      checkNsendHeartbeat(1, 0);    // parameters not used
#endif

      button = lcd_GetTouch(10);

      // Exit anytime FUNC is triggered
      // except in PURGE_RUN then go back to first_purge_state (through PURGE_RUN_DONE)
      // if (functionButton(FUNC_BUTT, FUNC_TRIG))
      if (button == BTN_EXIT)
      {   //if (headDiverter==TRUE && purge_state==PURGE_GET_DIRECTION)
          if (purge_state==PURGE_RUN)
               new_state=PURGE_RUN_DONE;
          else new_state=PURGE_EXIT;
      }
      // Exit anytime anything is pressed in auto purge or tube dry
      if (purge_mode==PURGE_AUTOMATIC || purge_mode==PURGE_DRYING)  // check for cancel of auto purge
      {
         if ( di_requestToSend || (button == BTN_CANCEL) || (button == BTN_EXIT) )
         {  new_state=PURGE_EXIT;
            purge_state=PURGE_EXIT; // Force in right away
         }
      }

      // enter local loop to process purge
      switch (purge_state) {
      case PURGE_ARE_YOU_SURE:
         if (first_time_here==TRUE)
         {  // show Start button
	         lcd_Font("16B");
	         lcd_ButtonDef( 1,
	            BTN_MOM,                      // momentary operation
	            BTN_TLXY, 5, lcd_max_y-35,             // starting x,y for buttons
	            BTN_TYPE, BUTTON_RELEASE,
	            BTN_MARGINS, 5, 330,         // set left and right margins
	            BTN_SEP, 63, 43,              // set up for button sep
	            BTN_TEXT, "Start",
	            BTN_TXTOFFSET, 9, 9,
	            BTN_BMP, BMP_med_button, BMP_med_button_dn,
	            BTN_END );

            // show message prompt
            lcd_DispText("PRESS START TO BEGIN", 50, 50, MODE_NORMAL);
				lcd_Font("24");
         }

         if (button==1) new_state=PURGE_SET_DIVERTER;

      	break;

      case PURGE_GET_HEADDIVERTER:            // get headdiverter alignment
         if (first_time_here==TRUE)
         {
            strcpy(mymessage, "(+)  TO         ");
            for (k=0; k<strlen(param.station_name[0].name); k++)
            { mymessage[k+8]=param.station_name[0].name[k]; }
            //lcd_print(SYSTEMLCD, 0, mymessage);
            //lcd_print(SYSTEMLCD, 1, "(-)  TO REMOTE  ");
			   lcd_drawScreen(5, title);
	         lcd_DispText("SETUP DIVERTER", 50, 50, MODE_NORMAL);
	         lcd_DispText(ltrim(sys_message[MSG_TO].msg), 140, 90, MODE_NORMAL);
	         lcd_ButtonDef( 1,
	            BTN_MOM,                      // momentary operation
	            BTN_TLXY, 100, 85,               // starting x,y for buttons
	            BTN_TYPE, BUTTON_RELEASE,
	            BTN_MARGINS, 5, 315,        // set left and right margins
	            BTN_SEP, 35, 35,              // set up for button sep
	            BTN_TEXT, "",
	            BTN_BMP, BMP_button_up, BMP_button_dn,
	            BTN_END );
	         lcd_DispText("TO SUB STATION", 140, 125, MODE_NORMAL);
	         lcd_ButtonDef( 2,
	            BTN_TLXY, 100, 120,               // starting x,y for buttons
	            BTN_BMP, BMP_button_up, BMP_button_dn,
	            BTN_END );
            if ((slaveAvailable) || (param.slaveController))
            {
               lcd_DispText(ltrim(sys_message[MSG_TO+SLAVE].msg), 140, 160, MODE_NORMAL);
	            lcd_ButtonDef( 3,
	               BTN_TLXY, 100, 155,               // starting x,y for buttons
	               BTN_BMP, BMP_button_up, BMP_button_dn,
	               BTN_END );
            }
         }

         // TO MAIN
         if (button == 1)
         {   new_state=PURGE_SET_DIVERTER;
             align_headdiverter=1;  // align main
             systemStation=0;    // no remote station
             systemStationb=0;
         }
         // TO REMOTE
         if (button == 2)
         {   new_state=PURGE_GET_STATION;
             align_headdiverter=2;  // align remote
         }
         // TO SLAVE
         if (button == 3)
         {   new_state=PURGE_SET_DIVERTER;
             align_headdiverter=3;  // align slave
             systemStation=0;    // no remote station
             systemStationb=0;
         }

         break;

      case PURGE_GET_STATION:            // get station selection

         if (first_time_here==TRUE)
         {
            //lcd_print(SYSTEMLCD, 0, sys_message[MSG_SEL_STATION].msg);
            //lcd_print(SYSTEMLCD, 1, "    TO PURGE    ");
			   lcd_drawScreen(5, title);
	         lcd_DispText("SELECT STATION TO PURGE", 20, 50, MODE_NORMAL);
            // draw station buttons
            col=40;
            for (i=0; i<LAST_STATION; i++)
            {  if ((1<<i) & STATION_SET)
            	{  mymessage[0]=49+i; mymessage[1]=0;
	               lcd_ButtonDef( i,    // station# - 1
	                  BTN_MOM,                      // momentary operation
	                  BTN_TLXY, col, 90,               // starting x,y for buttons
	                  BTN_TYPE, BUTTON_RELEASE,
	                  BTN_TEXT, mymessage,
	                  BTN_BMP, BMP_button_up, BMP_button_dn,
	                  BTN_END );
                  col+=40;
               }
            }
         }

         if (di_requestToSend & STATION_SET)
         {  systemStation=firstBit(di_requestToSend & STATION_SET);
            systemStationb=station2bit(systemStation);
            new_state=PURGE_SET_DIVERTER;
         }
         if ((button >= 0) && (button < LAST_STATION))
         {  systemStation = firstBit((1<<button) & STATION_SET);
            systemStationb=station2bit(systemStation);
            new_state=PURGE_SET_DIVERTER;
         }

         break;

      case PURGE_SET_DIVERTER:  // set diverter and continue

         if (first_time_here==TRUE)
         {
            //message_add(TRANSLCD, MSG_FUNCTION, MSG_PURGE, ONESHOTA);
            //message_add(SYSTEMLCD, MSG_SETTING, MSG_TO+systemStation, ONESHOT);
            //lcd_print(SYSTEMLCD, 0, sys_message[MSG_SETTING].msg);
            //lcd_print(SYSTEMLCD, 1, sys_message[MSG_TO+systemStation].msg);
			   lcd_drawScreen(5, title);
	         lcd_DispText(ltrim(sys_message[MSG_SETTING].msg), 50, 50, MODE_NORMAL);
            if (align_headdiverter==1)  // align main
		         lcd_DispText(ltrim(sys_message[MSG_TO].msg), 50, 80, MODE_NORMAL);
            else if (align_headdiverter==3) // align slave
		         lcd_DispText(ltrim(sys_message[MSG_TO+SLAVE].msg), 50, 80, MODE_NORMAL);
            else
		         lcd_DispText(ltrim(sys_message[MSG_TO+systemStation].msg), 50, 80, MODE_NORMAL);
            state_timer=MS_TIMER;
         }

         // Stay in this state until diverter command received
         set_diverter(systemStation);
         command.devAddr=ALL_DEVICES;
         command.command=SET_DIVERTER;
         command.station=systemStation;
         command.data[0]=align_headdiverter;
         command.data[1]=0; // mainStation
         command.data[2]=param.subStaAddressing;
         command.data[3]=param.blowerType;
// P2P WORK TO BE DONE HERE
         if (send_command(command)) new_state=PURGE_WAIT_DIVERTER;

         break;

      case PURGE_WAIT_DIVERTER:

         // no additional messages to show here
         if (MS_TIMER-state_timer >= DIVERTER_TIMEOUT)
         {  // timeout
             //lcd_print(SYSTEMLCD, 1, sys_message[MSG_TIMEOUT].msg);
             lcd_DispText(sys_message[MSG_TIMEOUT].msg, 30, 180, MODE_NORMAL);
             sprintf(mymessage, "STATUS = %x", diverter_status);
             lcd_DispText(mymessage, 0, 0, MODE_NORMAL);
             // just wait for exit key
             //new_state=PURGE_EXIT;
         }

         // Stay in this state until diverter in position
         msDelay(10);
         if (divertersReady(systemStation, align_headdiverter, 0))
         {
             station_msg = MSG_TO + systemStation;
             if (purge_mode==PURGE_AUTOMATIC) new_state=PURGE_RUN_AUTO;
             else if (purge_mode==PURGE_DRYING) new_state=PURGE_RUN_DRYING;
             else new_state=PURGE_RUN;
         }
         break;

      case PURGE_RUN:

      	// capture the station being purged in the event log
			if (eventlog.dest_sta==0) eventlog.dest_sta=systemStation;  // first destination recorded
         eventlog.flags |= systemStationb;  // capture any destination selected

         if (first_time_here==TRUE)
         {
             //message_add(TRANSLCD, MSG_FUNCTION, MSG_PURGE, ONESHOTA);
             //message_add(SYSTEMLCD, station_msg, MSG_BLANK, ONESHOT);
            // lcd_print(SYSTEMLCD, 0, " F1=PRS  F2=VAC ");
            //lcd_print(SYSTEMLCD, 0, "(+)=PRS (-)=VAC ");
            //lcd_print(SYSTEMLCD, 1, sys_message[MSG_BLANK].msg);
			   lcd_drawScreen(5, title);
	         lcd_DispText("PRESS TO OPERATE BLOWER", 32, 50, MODE_NORMAL);
            if (align_headdiverter==1)  // align main
		         lcd_DispText(ltrim(sys_message[MSG_TO].msg), 62, 75, MODE_NORMAL);
            else if (align_headdiverter==3) // align slave
		         lcd_DispText(ltrim(sys_message[MSG_TO+SLAVE].msg), 62, 75, MODE_NORMAL);
            else
		         lcd_DispText(ltrim(sys_message[MSG_TO+systemStation].msg), 62, 75, MODE_NORMAL);
	         lcd_DispText("PRESSURE", 140, 110, MODE_NORMAL);
	         lcd_ButtonDef( 3,
	            BTN_LAT,                      // momentary operation
	            BTN_TLXY, 100, 103,               // starting x,y for buttons
	            BTN_TYPE, BUTTON_LAT_0,
	            BTN_MARGINS, 5, 315,        // set left and right margins
	            BTN_SEP, 35, 35,              // set up for button sep
	            BTN_TEXT, "","",
	            BTN_BMP, BMP_button_off, BMP_button_on,
	            BTN_END );
	         lcd_DispText("VACUUM", 140, 147, MODE_NORMAL);
	         lcd_ButtonDef( 4,
	            BTN_TLXY, 100, 140,               // starting x,y for buttons
	            BTN_BMP, BMP_button_off, BMP_button_on,
	            BTN_END );
            //lcd_DispBitmap(BMP_LEDOFF,
         }
         // Pressure
         // if (functionButton(FUNC_F1, FUNC_CURR))
         if (button%256==3)
         {  if ((blower_purge_state != WAIT_FOR_REM_ARRIVE) || (button==259))
            {
                blower_purge_state = WAIT_FOR_REM_ARRIVE;  // always pressure
                blower(blwrPRS);
                //lcd_print(SYSTEMLCD, 1, "    PRESSURE    ");
                //lcd_DispText("PRESSURE", 120, 155, MODE_NORMAL);
                lcd_SetState( 4, 0 ); // make sure vacuum button is off
                system_direction=DIR_SEND;
            }
            // NEW to toggle on each key press
            else
            {
                blower_purge_state=IDLE_STATE;
                blower(blwrIDLE);
                //lcd_print(SYSTEMLCD, 1, sys_message[MSG_BLANK].msg);
                //lcd_DispText("                  ", 120, 155, MODE_NORMAL);
            }
            menu_timeout = SEC_TIMER + 60 * param.manPurgeTimeout;  // 5 min timeout while purging
         }
         // Vacuum
         // else if (functionButton(FUNC_F2, FUNC_CURR))
         else if (button%256==4)
         {  if ((blower_purge_state != WAIT_FOR_REM_DEPART) || (button==260))
            {
                blower_purge_state = WAIT_FOR_REM_DEPART;  // always vacuum
                blower(blwrVAC);
                //lcd_print(SYSTEMLCD, 1, "     VACUUM     ");
                //lcd_DispText("VACUUM    ", 120, 155, MODE_NORMAL);
                lcd_SetState( 3, 0 ); // make sure pressure button is off
                system_direction=DIR_RETURN;
            }
            // NEW to toggle on each key press
            else
            {
                blower_purge_state=IDLE_STATE;
                blower(blwrIDLE);
                //lcd_print(SYSTEMLCD, 1, sys_message[MSG_BLANK].msg);
                //lcd_DispText("                  ", 120, 155, MODE_NORMAL);
            }
            menu_timeout = SEC_TIMER + 60 * param.manPurgeTimeout;  // 5 min timeout while purging
         }

          // show message if arrival at headdiverter
         if ((remote_data_p[REMOTE_ARRIVE] & HEADDIVERTERSTATION)
            && (system_direction==DIR_RETURN))
         {
            //lcd_print(SYSTEMLCD, 1, "CARRIER @ BLOWER");
            lcd_DispText("CARRIER @ BLOWER", 120, 180, MODE_NORMAL);
         }

         break;

      case PURGE_RUN_AUTO:
         // wait for auto purge duration then go to the next station
         if (first_time_here==TRUE)
         {
            blower(auto_blower);
            if (auto_blower == blwrPRS) blower_purge_state = WAIT_FOR_REM_ARRIVE;
            else blower_purge_state = WAIT_FOR_REM_DEPART;
            //lcd_print(SYSTEMLCD, 0, "    PURGING     ");
            //lcd_print(SYSTEMLCD, 1, sys_message[station_msg].msg);
				lcd_clearMiddle();
	         lcd_DispText("PURGING ", 20, 50, MODE_NORMAL);
            //lcd_DispText(ltrim(sys_message[MSG_TO+systemStation].msg), 0, 0, MODE_NORMAL);
            if (align_headdiverter==3)  // align slave
		         lcd_DispText(ltrim(sys_message[MSG_direction+SLAVE].msg), 0, 0, MODE_NORMAL);
            else
		         lcd_DispText(ltrim(sys_message[MSG_direction+systemStation].msg), 0, 0, MODE_NORMAL);
         }

         menu_timeout = SEC_TIMER + 60;  // Don't timeout while purging

         if (MS_TIMER-state_timer > ms_autoPurgeTimer)
         {   // turn off blower
             blower(blwrIDLE);   // Always turn off the blower when leaving ... just in case.
             blower_purge_state=IDLE_STATE;

             // go to the next station
             if ((align_headdiverter==1) && (slaveAvailable || param.slaveController))
             {
                align_headdiverter=3;  // align slave
             } else
             {
	             systemStation++;
	             align_headdiverter=2;  // align remote
	             // make sure its an active station
	             while ( ((station2bit(systemStation) & STATION_SET)==0)
	                   && (systemStation < 8) ) systemStation++;
             }

            // set the next state
            if (systemStation >= 8) new_state=PURGE_EXIT;
            else
            {  new_state = PURGE_SET_DIVERTER;
               systemStationb = station2bit(systemStation);
            }
         }
         break;

      case PURGE_RUN_DRYING:
         // wait indefinitely until a key is pressed
         // wait for auto purge duration then go to the next station
         if (first_time_here==TRUE)
         {
            blower(blwrVAC);
            blower_purge_state = WAIT_FOR_REM_DEPART;
            system_direction=DIR_RETURN;
				lcd_clearMiddle();
	         lcd_DispText("DRYING ", 20, 50, MODE_NORMAL);
            if (align_headdiverter==3)  // align slave
		         lcd_DispText(ltrim(sys_message[MSG_direction+SLAVE].msg), 0, 0, MODE_NORMAL);
            else
		         lcd_DispText(ltrim(sys_message[MSG_direction+systemStation].msg), 0, 0, MODE_NORMAL);
            lcd_DispText("Press Exit To Stop Drying", 20, 80, MODE_NORMAL);
         }
         menu_timeout = SEC_TIMER + 60;  // Don't timeout while purging
         break;

      case PURGE_RUN_DONE:  // used to force one time through loop for remote command of blower
         blower_purge_state=IDLE_STATE;
         blower(blwrIDLE);
         // go to next state after one extra time through ... to set blowers properly
         if (first_time_here!=TRUE) new_state=first_purge_state;
         break;

      case PURGE_EXIT:
         // leaving purge here
         blower(blwrIDLE);   // Always turn off the blower when leaving ... just in case.
         blower_purge_state=IDLE_STATE;
         break;

      default:
         new_state=PURGE_EXIT;
         break;
      } // end switch

      // check for purge state change
      if (purge_state != new_state)
      {
         first_time_here=TRUE;    // wasn't here yet
         purge_state=new_state;   // this is where to be
         state_timer=MS_TIMER;
      }
      else first_time_here=FALSE; // were already here

	  // Check remote inputs only if processing PURGE_WAIT_DIVERTER or PURGE_RUN
      // or PURGE_RUN_DONE or PURGE_EXIT
	  if ( (purge_state==PURGE_WAIT_DIVERTER)
        || (purge_state==PURGE_RUN)
        || (purge_state==PURGE_RUN_AUTO)
        || (purge_state==PURGE_RUN_DRYING)
        || (purge_state==PURGE_RUN_DONE)
        || (purge_state==PURGE_EXIT) )
	  {
        // Add check for remote inputs in V2.34 for Up-Receive control
        // and now for headdiverter too
        command.devAddr=ALL_DEVICES;
        command.command=RETURN_INPUTS;
        command.station=0;
        command.data[0]=0;
        command.data[1]=blower_purge_state;    // send blower state
        // command.data[2]=system_direction;      // send diverter direction
	     command.data[2]=outputvalue[devInUseSolid];     // was system_direction;
	     command.data[3]=outputvalue[devInUseFlash];
        command.data[4]=STATION_SET;

		 msDelay(10);
// P2P WORK TO BE DONE HERE
// NOT BY COMMAND BUT INSPECTION OF REGULAR HEARTBEAT INPUT
//// REMOVE FOR ETHERNET PROCESS, NEED TO IMPLEMENT DUAL MODE RS485/ETHERNET
/*///		 if (send_n_get(command, &response))
		 {
			if (response.command==INPUTS_ARE)
			{
			  // store remote response in local var
			  for (k=0; k<4; k++) remote_data_p[k]=response.data[k];
			  // force unused doors closed (on) and other unused inputs off
			  remote_data_p[REMOTE_DOOR] |= ~STATION_SET;
			  remote_data_p[REMOTE_CIC] &= STATION_SET;
			  remote_data_p[REMOTE_RTS] &= STATION_SET;
         } // end if
       } // end if */
     } // end if
   } // end while
   inUse(OFF, STATION_SET);
	echoLcdToTouchscreen=TRUE;  // Reenable echo of lcd to Touchscreen
   eventlog.duration=(int)(SEC_TIMER-eventlog.start_tm);
   addTransaction( eventlog, 0 );  // transaction resumed, log the event

   // send command to tell remotes
   command.devAddr=ALL_DEVICES;
   command.command=SET_OUTPUTS;
   command.station=systemStation;
   command.data[devInUseSolid]=0;
   command.data[devInUseFlash]=0;
   command.data[devAlert]=0;
   command.data[devAlarm]=0; // 1=purge
// COMMAND DEPRECIATED
//   send_command(command);

}

int readParameterSettings(void)
{  // Read the parameter block param
   int rtn_code;
   rtn_code = readUserBlock(&param, 0, sizeof(param));
   if (rtn_code || (param.sysid != 63))
   {  // fault or parameters not initialized
      // set default parameters
      param.sysid=63;
   	param.names_init=0;
		param.stacking_ok[0]=FALSE;
		param.stacking_ok[1]=FALSE;
   	param.remoteAudibleAlert=TRUE;
		param.localDiverter=FALSE;      // local diverter configuration
   	param.activeStations=1;         // active station set
		param.autoPurgeTimer=10;        // auto purge timer in seconds
   	param.deliveryTimeout=60;       // delivery timeout in seconds
   	param.headDiverter=FALSE;
		param.pw_enabled=FALSE;
   	param.slaveController=MASTER;    // master (0) or slave (SLAVE)
	   param.systemNum=1;
		param.serverID[0]=0;
   }
   param.activeMain = MASTER;  // always default to master
   return rtn_code;
}
int writeParameterSettings(void)
{  // Write the parameter block param
   int rtn_code;
   rtn_code = writeUserBlock(0, &param, sizeof(param));
   return rtn_code;
}
int syncParametersToRemote(void)
{  // returns 0 if no error, non-zero otherwise
	int status, i;
   struct iomessage command;

   // First update card parameters
   command.command = SET_CARD_PARAMS;
   command.station = 0;
   command.devAddr = ALL_DEVICES;

	// card format
   command.data[0]=0; // card format
   command.data[1]=param.cardFormat;
   send_command(command);
   msDelay(10);

	// Individual card checking
   command.data[0]=6; // Left 3 digits
   command.data[1]=param.cardCheckEnabled;
   send_command(command);
   msDelay(10);
	// Left 3 digits
//   command.data[0]=0; // Left 3 digits
//   command.data[1]=param.cardL3Digits;
//   send_command(command);
//   msDelay(10);
   // Right 9 minimum
//   command.data[0]=1; // Right 9 min
//   *(unsigned long *)&command.data[1] = param.cardR9Min;
//   send_command(command);
//   msDelay(10);
   // Right 9 maximum
//   command.data[0]=2; // Right 9 max
//   *(unsigned long *)&command.data[1] = param.cardR9Max;
//   send_command(command);
//   msDelay(10);
   //// SECOND CARD RANGE
	// Left 3 digits
//   command.data[0]=3; // Left 3 digits
//   command.data[1]=param.card2L3Digits;
//   send_command(command);
//   msDelay(10);
   // Right 9 minimum
//   command.data[0]=4; // Right 9 min
//   *(unsigned long *)&command.data[1] = param.card2R9Min;
//   send_command(command);
//   msDelay(10);
   // Right 9 maximum
//   command.data[0]=5; // Right 9 max
//   *(unsigned long *)&command.data[1] = param.card2R9Max;
//   send_command(command);
//   msDelay(10);

	// send command to remote to update and save other parameters
	command.command=SET_PARAMETERS;
	command.station=0;
   status=0; // assume none good
   for (i=FIRST_DEVADDR; i<=LAST_DEVADDR; i++)
   {  if ( (1 << (i-1)) & availableDevices )  // if i is a remote devAddr
		{	command.devAddr=i;
	   	command.data[0]=param.portMapping[i];
			command.data[1]=param.blowerType;
         command.data[2]=i; // for UDP conversion
			if (send_command(command)==1) status |= (1 << (i-1));
      }
   }

   return (status == availableDevices) ? 0 : 1;
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
      aliveStations=0;
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
      aliveStations = 0;
      for (idx=NUM_STATIONS; idx>0; idx--)  // count down
      {  aliveStations = (aliveStations << 1);            // shift
      	aliveStations |= (recentDevices[idx]>0) ? 1 : 0;  // include bit
		   // clear the history buffer
		   recentDevices[idx]=0;
      }
   }
#endif
}

#ifdef USE_TCPIP
int getUDPcommand(struct UDPmessageType *UDPmessage)
{   // ONLY FOR CONTROL SOCKET/PORT FOR NOW.  LOGGER/HOST SUPPORT LATER.
    // SHARE LOCAL PORT OR PROCESS TWO SOCKETS
	// be sure to allocate enough space in buffer to handle possible incoming messages (up to 128 bytes for now)
   auto char   buf[NUMUDPDATA+10];
   auto int    length, retval;
   int idx;
   int InPort, OutPort;
   char * UDPbuffer;  // to index into UDPmessage byte by byte

   length = sizeof(buf);
   retval = udp_recv(&CtrlSock, buf, length);
   if (retval < -1) {
      printf("Error %d receiving datagram!  Closing and reopening socket...\n", retval);
      sock_close(&CtrlSock);
      // swap ports for slave
      if (param.slaveController) {InPort = CTRL_PORT; OutPort = CTRL_LOCAL_PORT;}  // reverse ports
   	else								{InPort = CTRL_LOCAL_PORT; OutPort = CTRL_PORT;}
	   if(!udp_open(&CtrlSock, InPort, -1/*resolve(REMOTE_IP)*/, OutPort, NULL)) {
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
         // acknowledge com from device by clearing bit
         comIsGood(UDPmessage->device);
      }
	}
   return retval;
}
void processUDPcommand(struct UDPmessageType UDPmessage)
{  // Handles incoming packets from the control system
   // Host/logger commands not supported
#ifdef USE_TCPIP
	// Handle any incoming UDP packets for me
   int b_station;

  	// if device type is 2 include in diverter device set device
   if (UDPmessage.devType == 2) diverterDevices |= station2bit(UDPmessage.device);

   /* determine what type of command */
   switch(UDPmessage.command)
   {
      case INPUTS_ARE:
			// place returned values into local variables
         // Note: need to shift input bit positions according to station number
         b_station = station2bit(UDPmessage.station);
         if (UDPmessage.data[0]) remote_data[REMOTE_CIC] |= b_station;   // bit on
         else                    remote_data[REMOTE_CIC] &= ~b_station;  // bit off
         if (UDPmessage.data[1]) remote_data[REMOTE_DOOR] |= b_station;   // bit on
         else                    remote_data[REMOTE_DOOR] &= ~b_station;  // bit off
         if (UDPmessage.data[2]) remote_data[REMOTE_ARRIVE] |= b_station;   // bit on
         else                    remote_data[REMOTE_ARRIVE] &= ~b_station;  // bit off
         if (UDPmessage.data[3]) remote_data[REMOTE_RTS] |= b_station;   // bit on
         else                    remote_data[REMOTE_RTS] &= ~b_station;  // bit off
         if (UDPmessage.data[4]) remote_data[REMOTE_RTS2] |= b_station;   // bit on
         else                    remote_data[REMOTE_RTS2] &= ~b_station;  // bit off

         // save version number
         // special handling if slave station
			if (UDPmessage.station == SLAVE)
         {  slaveAvailable=TRUE;
	         if (UDPmessage.data[0]) slave_cic=TRUE;   // bit on
	         else                    slave_cic=FALSE;  // bit off
	         if (UDPmessage.data[1]) slave_doorClosed=TRUE;   // bit on
	         else                    slave_doorClosed=FALSE;  // bit off
	         //if (UDPmessage.data[2]) remote_data[REMOTE_ARRIVE] |= b_station;   // bit on
	         //else                    remote_data[REMOTE_ARRIVE] &= ~b_station;  // bit off
            if (UDPmessage.data[2]) slave_arrive=TRUE;
            else                    slave_arrive=FALSE;
	         // DON'T USE //if (UDPmessage.data[3]) slave_rts=b_station; //TRUE;   // bit on
	         //else                    slave_rts=FALSE;  // bit off
              slave_rts=0; // FORCE OFF
            slaveReturnStatus = UDPmessage.data[7];
         }

         // Secure Card ID starts at .data[9]
         //secureID = *(unsigned long *)&UDPmessage.data[9];
         //memcpy(secureID, &UDPmessage.data[9], sizeof(secureID));
///         if (UID_Not_Zero(&UDPmessage.data[9]))
         {  // non-zero ID to be cached
	         //secureCard[UDPmessage.station].id = secureID;
///            memcpy(secureCard[UDPmessage.station].id, &UDPmessage.data[9], UIDLen);
///	         secureCard[UDPmessage.station].time = UDPmessage.data[14];
///            secureCard[UDPmessage.station].scanType = UDPmessage.data[8];
	         // tell remote we got the card ID
///            secureAck = UDPmessage.station;
//	         if (secureCard[UDPmessage.station].id)
//	            secureAck = UDPmessage.station;
//	         else
//	            secureAck = 0;
         }

      break;
      case DIVERTER_STATUS:
      	// capture device into remote_data[REMOTE_DIVERTER]
         b_station = station2bit(UDPmessage.device);
         if (UDPmessage.data[0] && (UDPmessage.station==systemStation))
               remote_data[REMOTE_DIVERTER] |= b_station;   // bit on
         else  remote_data[REMOTE_DIVERTER] &= ~b_station;  // bit off

      break;
   }
   // track the active stations to report when a station stops communicating
   monitorUDPactivity(UDPmessage.device);

#endif
}
void slaveProcessUDPcommand(struct UDPmessageType UDPmessage)
{   // Handles incoming requests from the diverter controller

	static char lastTransStation;  // to keep track of when transStation changes
	char UID[15];
   char bUID[7];
   int i;
   static char new_state;
   static char haveButtons;
   static char lastStation;
   char transFlags, systemDirection, purgeMode; // used locally only
   #GLOBAL_INIT
   {  haveButtons=TRUE; lastStation=0; }


  	//printf("\n  Got UDP message %c at %ld, %d, %d, %ld", UDPmessage.command, UDPmessage.timestamp, latchCarrierArrival, arrivalISRstate, arrivalDuration);
   /* determine what type of command */
   switch(UDPmessage.command)
   {
   	case UID_ADD:
      case UID_DEL:
      	break;

      case CANCEL_PENDING:
		   rts_latch = 0;	// clear latch for all stations
		   break;

   	case TRANS_COMPLETE:
         // handle completed transaction
///	      slaveFinishTransaction();
	      lcd_drawScreen(1,lcd_WITH_BUTTONS);
	      //haveButtons=TRUE;
      	break;
	   case DISPLAY_MESSAGE:
	      // display messages on lcd if installed
	      // first update variable messages
	      setCountMessage();
	      setTimeMessages(MSG_DATE_TIME, SEC_TIMER);
	      // fixup messages in function_mode at master
	      if (UDPmessage.data[0] == MSG_FUNCTION)
	      {  UDPmessage.data[2]=MSG_WAIT;
	         UDPmessage.data[3]=MSG_BLANK;
	      }
	      lcd_displayMsgs(&UDPmessage.data[0]);

	      break;

	   case SET_TRANS_COUNT:
	      ///slaveSyncTransCount(&UDPmessage.data[0]);
	      break;

      case REMOTE_PAYLOAD:  // standard data distribution message
         arrival_from = UDPmessage.data[0];
			new_state = UDPmessage.data[1];
         //transStation = UDPmessage.data[3];
			systemStation = UDPmessage.data[3];
		   systemStationb=station2bit(systemStation);
         mainStation = UDPmessage.data[4];
///         diverterStation = UDPmessage.data[5];
///         param.subStaAddressing = UDPmessage.data[6];
         transFlags = UDPmessage.data[7];
         // do we have a secure ack?
///         if (isMyStation(UDPmessage.data[8]))
///         {  UID_Clear(secureCard.id);  // clear secureCardID
///            UID_Clear(stdCard.id);  // and standard card
///				UID_Clear(unauthCard.id);
///         }
         malfunctionActive = (new_state == MALFUNCTION_STATE);
         systemDirection = UDPmessage.data[11];
         main2main_trans = systemDirection;  // ALWAYS FOR NOW
         purgeMode = UDPmessage.data[12];

         // if station is toggled then respond to start of transaction
         if (systemStation != lastStation)
         {
	         // check destination station ... is it me?
	         if ((systemStation==SLAVE) && (systemDirection==DIR_SEND))
	         { // door must be closed, set arrival enable
	            if (di_doorClosed)
	            {  arrivalEnable(ON);
	            } else
	            {  // door open so not ready
	               setAlerts(ON);
	            }
	         }
         }
         lastStation=systemStation;
      	break;

	   case ARE_YOU_READY:  // ready to begin transaction
	      // need to check for both source and destination stations
///         response.command=ARE_YOU_READY;
	      systemStation=UDPmessage.station;
	      systemStationb=station2bit(systemStation);
	      system_direction=UDPmessage.data[0];
	      mainStation=UDPmessage.data[1];
	      main2main_trans=UDPmessage.data[2];

	      // clear latch for these stations
	      rts_latch &= ~systemStationb;

///	      response.data[0]=SLAVE_b;     // assume ready for now

	      // check destination station ... is it me?
	      // 1) main to slave
	      if ((mainStation==MASTER) && (main2main_trans==DIR_SEND))
	      { // door must be closed, set arrival enable
	         if (di_doorClosed)
	         {  arrivalEnable(ON);
	         } else
	         {  // door open so not ready
///	            response.data[0]=0;           // not ready
	            setAlerts(ON);
	         }
	      }
	      // 2) remote to slave
	      else if ((mainStation==SLAVE) && (system_direction==DIR_RETURN))
	      { // door must be closed, set arrival enable
	         if (di_doorClosed)
	         {  arrivalEnable(ON);
	         } else
	         {  // door open so not ready
///	            response.data[0]=0;           // not ready
	            setAlerts(ON);
	         }
	      }
	      // 3) slave to any
	      else if ((mainStation==SLAVE) && (system_direction==DIR_SEND))
	      { // door must be closed, carrier in chamber
	         if (di_doorClosed && di_carrierInChamber)
	         {  // do nothing, ready to go
	         } else
	         {  // door open or no carrier so not ready
///	            response.data[0]=0;           // not ready
	            setAlerts(ON);
	         }
	      }
	      // 4) don't care, always ready
	      else
	      {
	        // do nothing
	      }
///	      slaveSendResponse(response);
	      lcd_drawScreen(1,lcd_NO_BUTTONS);  // take away buttons
	      haveButtons=FALSE;
	      break;

	   case SET_DIVERTER:
	      mainStation=UDPmessage.data[1];
	      param.subStaAddressing=UDPmessage.data[2];
	      param.blowerType=UDPmessage.data[3];
	      // set_diverter(mainStation);  NEVER A SLAVE LOCAL DIVERTER
	      break;

      case SET_CARD_PARAMS:
      	break;

	   case MALFUNCTION:
	      // turn on alarm output only
	      setAlerts(OFF);
	      if (mainStation==SLAVE) alarm(ON); // alarm(ON) appears in process state too
	      rts_latch=0;
///	      for (i=0; i<NUMDATA; i++) response.data[i]=0; // clear remote i/o
	      arrivalEnable(OFF);      // disable arrival interrupt
	      if (haveButtons==FALSE)
	      {  // no buttons yet so redraw screen to allow responding to an alarm
	         lcd_drawScreen(1,lcd_WITH_BUTTONS);
	         haveButtons=TRUE;
	      }
	      break;

	   case RESET:
	      // Turn off all outputs
	      // SEEMS LIKE SOMETIMES MAY NOT COME HERE WHEN SLAVE ISSUED AN ALARM RESET
	      // WHICH PASSES CANCEL_STATE BACK TO MAIN, LEADING TO A RESET COMMAND IN FINAL_COMMAND
	      lcd_drawScreen(1,lcd_WITH_BUTTONS);
	      inUse(OFF, ALL_STATIONS);           // clear local i/o
	      setAlerts(OFF);
	      alarm(OFF);
	      // rts_latch=0;
	      mainStation=0;
	      systemStation=0;
	      system_direction=0;
///	      for (i=0; i<NUMDATA; i++) response.data[i]=0; // clear remote i/o
	      arrivalEnable(OFF);      // disable arrival interrupt
	      slave_arrival=0;
	      arrival_from=0;
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
}

int sendRS485toUDP(struct iomessage RSmsg)
{
	struct UDPmessageType UDPmsg;

   // map RSmsg into UDPmsg
   // but only command, station, and data ... AND NOT devaddr
   //UDPmsg.device = RSmsg.devAddr;
   UDPmsg.command = RSmsg.command;
   UDPmsg.station = RSmsg.station;
   UDPmsg.data[0] = RSmsg.data[0];
   UDPmsg.data[1] = RSmsg.data[1];
   UDPmsg.data[2] = RSmsg.data[2];
   UDPmsg.data[3] = RSmsg.data[3];
   UDPmsg.data[4] = RSmsg.data[4];
   UDPmsg.data[5] = RSmsg.data[5];

   // send it
   sendUDPcommand(UDPmsg, UDP_CTRL);
}
int sendUDPcommand(struct UDPmessageType UDPmessage, char targ)
{
	// Send the supplied command over UDP
   // Appends some info to the standard message
   //   .device
   //   .devType
   //   .timestamp

   auto int    length, retval;
   char * UDPbuffer;  // to index into UDPmessage byte by byte
   int InPort, OutPort;

   // fill the packet with additional data
   UDPmessage.device = param.systemNum;  // system id
   UDPmessage.devType = 1; // 1=main; 2=diverter; 3=station; 4=slave
   UDPmessage.timestamp = MS_TIMER;

   length = sizeof(UDPmessage); // how many bytes to send
   UDPbuffer = (char *) &UDPmessage;

   /* send the packet */
   if (targ == UDP_HOST)
   {  // main on the host/logger port
	   if(!udp_open(&HostSock, HOST_LOCAL_PORT, -1/*resolve(REMOTE_IP)*/, HOST_PORT, NULL)) {
	      printf("udp_open failed!\n");
	   } else {
	      retval = udp_send(&HostSock, UDPbuffer, length);
	      if (retval < 0) {
	         printf("Error sending datagram!  Closing and reopening socket...\n");
	      }
	   }
	   sock_close(&HostSock);
   }
   else
   {  if (param.slaveController)
   	{  // slave on the control socket
      	InPort = CTRL_PORT;
      	OutPort = CTRL_LOCAL_PORT;  // reverse ports
         UDPmessage.devType = 3; // send as a remote station
		   UDPmessage.device = SLAVE;  // system id; for slave: device = station = 8
      }
   	else
      {  // main on the control socket
      	InPort = CTRL_LOCAL_PORT;
         OutPort = CTRL_PORT;
      }
	   if(!udp_open(&CtrlSock, InPort, -1/*resolve(REMOTE_IP)*/, OutPort, NULL)) {
	      printf("udp_open failed!\n");
	   } else {
	      retval = udp_send(&CtrlSock, UDPbuffer, length);
	      if (retval < 0) {
	         printf("Error sending datagram!  Closing and reopening socket...\n");
	      }
	   }
	   // sock_close(&CtrlSock); DON'T CLOSE BECAUSE WE NEED IT OPEN FOR READING
   }

}
int sendUDPHeartbeatToHost(char sample)
{  // Sends an INPUTS_ARE message to the host

	struct UDPmessageType HBmessage;
   unsigned long timer_val;
   char * cptr;  // pointer to reference into timer value
   int idx;
	timer_val=SEC_TIMER;
   cptr = (char *) &timer_val;
   idx=0;

   HBmessage.command = INPUTS_ARE;
   HBmessage.station = param.systemNum;  // provide the real station number
   HBmessage.data[idx++] = *cptr++; // (char) SEC_TIMER; // Seconds timer
   HBmessage.data[idx++] = *cptr++; //(char) SEC_TIMER; // Seconds timer
   HBmessage.data[idx++] = *cptr++; //(char) SEC_TIMER; // Seconds timer
   HBmessage.data[idx++] = *cptr++; //(char) SEC_TIMER; // Seconds timer
   HBmessage.data[idx++] = 2;  // packet type = heartbeat
   HBmessage.data[idx++] = system_state;
   HBmessage.data[idx++] = systemStation;
   HBmessage.data[idx++] = mainStation;
   HBmessage.data[idx++] = system_direction;
   HBmessage.data[idx++] = main2main_trans;
   HBmessage.data[idx++] = destMain;
   HBmessage.data[idx++] = remote_data[REMOTE_CIC];
   HBmessage.data[idx++] = remote_data[REMOTE_DOOR];
   HBmessage.data[idx++] = remote_data[REMOTE_ARRIVE];
   HBmessage.data[idx++] = remote_data[REMOTE_RTS];
   HBmessage.data[idx++] = remote_data[REMOTE_RTS2];
   HBmessage.data[idx++] = 0; // local inputs
   HBmessage.data[idx++] = 0; // slave inputs
   HBmessage.data[idx++] = 0; //
   HBmessage.data[idx++] = 0; //
	// max is 20 data elements

   sendUDPcommand(HBmessage, UDP_HOST); // ignore return value, will send again periodically
}
int sendUDPHeartbeatToCtrl(char sample, char transFlags)
{  // Sends a periodic message to the remote stations
#ifdef USE_TCPIP
	struct UDPmessageType HBmessage;

   HBmessage.command = REMOTE_PAYLOAD;
   HBmessage.station = ALL_STATIONS;
   HBmessage.data[0] = arrival_from; //
   HBmessage.data[1] = system_state; //
   HBmessage.data[2] = 0; // remote alert - not used
   HBmessage.data[3] = systemStation; //
   HBmessage.data[4] = mainStation; //
///   HBmessage.data[5] = diverterStation; // from setDiverter(station)
///command.data[2]=main2main_trans;
   HBmessage.data[6] = param.subStaAddressing; //
   HBmessage.data[7] = transFlags; //
   HBmessage.data[8] = secureAck; // secure ACK
   secureAck = 0; // clear Ack after sending
///   HBmessage.data[9] = securePIN_hb; // secure PIN high byte
///   HBmessage.data[10] = securePIN_lb; // secure PIN low byte
   HBmessage.data[11] = system_direction; // transaction direction
   HBmessage.data[12] = 0; // purge_mode;
   HBmessage.data[13] = VERS; // Version
   HBmessage.data[14] = SUBVERS; // Sub-version
   HBmessage.data[15] = AckValue;
   HBmessage.data[16] = AckStrobe;
   HBmessage.data[17] = AckAck;

   sendUDPcommand(HBmessage, UDP_CTRL); // ignore return value, will send again periodically
#endif
}
int slaveSendUDPHeartbeat(char sample)
{  // Sends an INPUTS_ARE message to the host

	struct UDPmessageType HBmessage;
   char * sid;

   HBmessage.command = INPUTS_ARE;
   HBmessage.station = SLAVE; // THIS_DEVICE;  // provide the real station number
   HBmessage.data[0] = di_carrierInChamber ? 0x01 : 0; // CIC
   HBmessage.data[1] = di_doorClosed; // Door closed
   HBmessage.data[2] = 0; //carrierArrival(MY_STATIONS); // Arrival
   // but if Arrival is none then send latched arrival under some conditions
   if ((HBmessage.data[2]==0) && (latchCarrierArrival) && (system_state==WAIT_FOR_REM_ARRIVE))
	   HBmessage.data[2] = latchCarrierArrival; // Arrival
   HBmessage.data[3] = rts_latch | station2bit(lcd_sendTo());  // RTS latch
   HBmessage.data[4] = 0; // rts2_latch; // RTS latch - 2nd destination
   HBmessage.data[5] = VERS; // Version
   HBmessage.data[6] = SUBVERS; // Sub-version
   // prepare slaveReturnStatus
   if (slave_arrival) slaveReturnStatus |=0x10;  // slave arrival flag
   if (di_returnCarrier || lcd_returnCarrier()) slaveReturnStatus |= 0x40;   // return carrier_return request
   if (lcd_autoReturn() || autoRtnTrans) slaveReturnStatus |= 0x80;  // auto return requested
   HBmessage.data[7] = slaveReturnStatus; //
   HBmessage.data[8] = 0; // nothing initialize
   HBmessage.data[15] = AckValue;
   HBmessage.data[16] = AckStrobe;
   HBmessage.data[17] = AckAck;
   sendUDPcommand(HBmessage, UDP_CTRL); // ignore return value, will send again periodically
   slaveReturnStatus=0;  // I expect immediate response

}

void sendUDPtransaction(struct trans_log_type translog, char xtra )
{
	struct UDPmessageType TLmessage;
   char * TLbuffer;  // to index into translog byte by byte
   //unsigned long timer_val;
   //char * cptr;  // pointer to reference into timer value
   int idx;
	//timer_val=SEC_TIMER;
   //cptr = (char *) &timer_val;
   //idx=0;

   // blank out the buffer
   for (idx = 0; idx < NUMUDPDATA; idx++)
      TLmessage.data[idx] = 0;

   // setup message to announce transaction complete
   TLmessage.station = param.systemNum;  // provide the real station number if possible
   if (translog.status <= LAST_TRANS_EVENT)
	   TLmessage.command = TRANS_COMPLETE;
   else if (translog.status == ESTS_SECURE_REMOVAL)
	{  TLmessage.command = ESECURE_REMOVAL;
      TLmessage.station = xtra;
//      TLmessage.data[sizeof(translog)] = param.cardL3Digits-CARDHBOFFSET; // add on 5th byte of 38 bit card id
   } // is it a standard or unauthorized card scan?
   else if ((translog.status == ESTS_CARD_SCAN) || (translog.status == ESTS_UNAUTH_CARD))
	{  TLmessage.command = ECARD_SCAN;
   	TLmessage.station = xtra;
   }
   else // otherwise must be some kind of event
	   TLmessage.command = SYSTEM_EVENT;

   // stuff the transaction log data straight into UDP buffer
   TLbuffer = (char *) &translog;
   for (idx = 0; idx < sizeof(translog); idx++)
      TLmessage.data[idx] = TLbuffer[idx];

   sendUDPcommand(TLmessage, UDP_HOST); // send to the host logging computer
   //TLmessage.station=systemStation; // using global
   //sendUDPcommand(TLmessage, UDP_CTRL); // also send to the sub stations
}

void sendParametersToLogger(void)
{  // send the full parameter set over TCP/IP UDP
	struct UDPmessageType PBmessage;
   char * PBbuffer;  // to index into translog byte by byte
   int idx;

   // blank out the buffer
   for (idx = 0; idx < NUMUDPDATA; idx++)
      PBmessage.data[idx] = 0;

   // setup message to announce transaction complete
   PBmessage.station = param.systemNum;  // provide the real station number if possible
   PBmessage.command = SET_PARAMETERS;

   // stuff the transaction log data straight into UDP buffer
   PBbuffer = (char *) &param;
   // for (idx = 0; idx < sizeof(param); idx++)
   for (idx = 0; idx < NUMUDPDATA; idx++)  // only send as much as fits in buffer.
      PBmessage.data[idx] = PBbuffer[idx];

   sendUDPcommand(PBmessage, UDP_HOST); // ignore return value, will send again periodically

}

#endif
void maintenance(void)
{  // handle all regularly scheduled calls

   hitwd();                // hit the watchdog timer
   showactivity();
   //rn_keyProcess(DevRN1600, 0);  // process keypad device

   // process command communications
#ifdef USE_TCPIP
   tcp_tick(NULL);
#endif

   // handle carrier in chamber light
   do_CICLight(di_carrierInChamber);
   // handle for local control
   if (param.localSubstation) do_CICLight2(di_carrierInChamber2);

   // process flashing inUse lights
   if (RN1100_Found)
	{  if ((MS_TIMER %500) < 300) digBankOut(0, ~(outputvalue[devInUseFlash] | outputvalue[devInUseSolid]));
   	else digBankOut(0, ~outputvalue[devInUseSolid]);
	}
   // process blower
   processBlower();

   // process arrival alert
	processMainArrivalAlert();

   // process remote arrival alerts


}
void exercise_outputs()
{
   /* Exercise all lighted outputs */
   alarm(OFF);
   setAlerts(ON);
   inUse(FLASH, ALL_STATIONS);
   hitwd();          // "hit" the watchdog timer
   msDelay(1000);
   setAlerts(OFF);
   hitwd();          // "hit" the watchdog timer
   msDelay(1000);
   hitwd();
   inUse(ON, ALL_STATIONS); maintenance(); msDelay(200);
   inUse(OFF, ALL_STATIONS); maintenance(); msDelay(200);
   inUse(ON, ALL_STATIONS); maintenance();  msDelay(200);
   inUse(OFF, ALL_STATIONS); maintenance();
   hitwd();          // "hit" the watchdog timer

}

/******************************************************************/
char station2bit(char station)
{  /* Converts the station number assignment to an equivalent
      bit value.  ie. station 3 -> 0100 (=4) .. same as 2^(sta-1)
      NOTE: Returns 0 if station is not 1..8  */

   if (station) return 1 << (station-1);
   else return 0;
}
/******************************************************************/
char bit2station(char station_b)
{  /* Converts a bit assignment to the bit number value
      ie. 0100 -> station 3.
NOTE: Returns 0 if more than 1 station bit is active */

   char stanum, i;
   stanum = 0;

   for (i=0; i<8; i++)
   {
      if (1<<i == station_b) stanum=i+1;
   }
   return stanum;
}
/******************************************************************/
char firstBit(char bitdata)
{  // Return # of first bit that is on (1-8)
   char i;
   i=0;
   while (i<8 && ((1<<i & bitdata)==0)) i++;
   if (i<8) return i+1;
   else     return 0;
}

/******************************************************************/
/* Communications drivers */
#define CMD_LENGTH  12
#define RCV_TIMEOUT 70
#define ACK_TIMEOUT 30
//#define BUF_COUNT 1
//#define BUF_SIZE 10
#define BAUD_RATE 19200
int badcmd;
#define CSTATS 10
unsigned long commstat[CSTATS];
char comError, wasComError;
const char * const commlbl[CSTATS] = {
"Messages Sent   ","Messages Recvd  ","Msgs Not Recvd  ","Msgs Answered   ",
"# BAD CHECKSUM  ","# INCOMPLETE INP","# NO STX/ETX    ","# UNKNOWN ACK   ",
"# NAK           ","# NIL           " };


// message buffer for general work use
struct iomessage workmsg;
void comIsGood(char device)
{  // capture device as good com
	comError &= (~station2bit(device));
}
void print_comstats(void)
{  static unsigned long laststats[CSTATS];
   // using item 0, 1, 3
   // calc difference
	printf("\n sent=%ld acked=%ld rcvd=%ld", commstat[0]-laststats[0], commstat[1]-laststats[1], commstat[3]-laststats[3]);
   laststats[0]=commstat[0];
   laststats[1]=commstat[1];
   laststats[3]=commstat[3];

}
void disable485whenDone(void)
{
   while (serDwrUsed());          // wait for all bytes to be transmitted
   while (RdPortI(SDSR) & 0x0C);  // wait for last byte to complete
   serDrdFlush();                 // flush the read buffer
   ser485Rx();                    // disable transmitter
}
char init_communication()
{
   char stat, i;

   // Initialize packet driver
   // Coyote uses port D for RS485
   // use default buffer sizes (31 bytes, tx & rx)
   serDopen(BAUD_RATE);
   ser485Rx();
   serDrdFlush();     // clear receive buffer
   serDwrFlush();     // clear transmit buffer

   badcmd=0;

   workmsg.devAddr=ALL_DEVICES;
   workmsg.command=RESET;
   workmsg.station=0;
   stat = send_command( workmsg );   // return code to determine comm status

   // clear comm status counters
   for (i=0; i<CSTATS; i++) commstat[i]=0;
   comError=0;
   wasComError=0;   // reset com error state change flag

   return stat;
}
unsigned long t1, t2, t3, t4;    // debug
char send_n_get(struct iomessage message, struct iomessage *response)
{
   // Transmits message to 1 or all devices and combines responses for return
   // Verify success by response.devAddr -- equals those that answered BIT-WISE (message.devAddr=3 returns 0x04)
   // Uses global data structure extendedData[]
   // Break out response for Secure ID
   char i, j, firstDevAddr, lastDevAddr;
   char good,retry,anygood;

// P2P WORK TO BE DONE HERE ????
if (param.point2point==TRUE) return 0;

   response->devAddr=0;
   response->command=0; // message.command;
   response->station=0;
   for (i=0; i<NUMDATA; i++) response->data[i]=0;

   // process send/return for each controller necessary
   if (message.devAddr == ALL_DEVICES)
   {
      firstDevAddr=FIRST_DEVADDR;
      lastDevAddr=LAST_DEVADDR;
   } else
   {
      firstDevAddr=message.devAddr;
      lastDevAddr=message.devAddr;
   }
   comError=0;   // reset only once each pass through this routine
                  // so that 1 bad device & 1 good device still flags error
   for (i=firstDevAddr; i<=lastDevAddr; i++)
   {  if ( (1 << (i-1)) & availableDevices )  // if i is a remote devAddr
      {
         // set device id
         message.devAddr=i;
         if (i != firstDevAddr) msDelay(2);   // wait for data lines to clear
         retry=3;
         good=FALSE;
         while (!good && retry)
         {
            if (send_command(message))
            {  // sent command ok, now get response
               workmsg.command=0;   // clear for input of response
               get_response(&workmsg);
               if (workmsg.command==message.command)
               {  // OR all the input responses together
                  // for (j=0; j<4; j++) response->data[j] |= workmsg.data[j];
                  // Catch blower error from head diverter - NOT A GOOD WAY TO DO IT BUT EASY FOR NOW, FIX LATER
                  if ((workmsg.devAddr==HEADDIV_DEVADDR) && (workmsg.command==RETURN_INPUTS))
                  {  // catch blower error from head diverter
                     if (workmsg.data[4]==0xFF)
                     {  // blower timeout
	                     blwrError=TRUE;
                     } else
                     {  // just return blower position
                     	remoteBlwrPos = workmsg.data[4];
                     }
	                  workmsg.data[4]=0;
                  }
                  if (workmsg.devAddr==SLAVE_DEVADDR && workmsg.command==RETURN_INPUTS)
                  {  // store slave return_input data in global array
   		            for (j=0; j<NUMDATA; j++) slaveData[j] = workmsg.data[j];
                  }
                  else // standard response
                  {  for (j=0; j<NUMDATA; j++) response->data[j] |= workmsg.data[j];
                  }
                  response->devAddr |= workmsg.devAddr; // only works for 1 device at a time (1 << workmsg.devAddr-1);  // return responders as bit-wise
                  response->station |= workmsg.station;
                  response->command = workmsg.command;
                  good=TRUE;
                  anygood=TRUE;
                  // if extended data command then store data in alternate location
                  if (workmsg.command==RETURN_EXTENDED) for (j=0; j<4; j++) extendedData[i][j]=workmsg.data[j];
               }
               // could get an alternate message instead of a direct answer to the requested message
               else if (workmsg.command==SECURE_REMOVAL) // can be std scan as well
               {  // store secure badge id and delta-time
                //if (secureTransLog[workmsg.station].start_tm)
                  {  // not yet logged
	                  //secureTransLog[workmsg.station].sid = *(unsigned long *) &workmsg.data[0];
                     UID_Copy(secureTransLog[workmsg.station].sid, workmsg.data);

							// update start_tm to be NOW
                     secureTransLog[workmsg.station].start_tm = SEC_TIMER;
							secureTransLog[workmsg.station].scanType = workmsg.data[5];

   	               //secureTransLog[workmsg.station].flags = workmsg.data[4];  // save scaled delta-t for now
                  }
                  // acknowledge secure removal
			         secureAck |= station2bit(workmsg.station);

                  // BUT DON'T SET GOOD TO BE TRUE SO AS TO FORCE A RETRY ON RETURN_INPUTS
               } else retry--;
            } else retry--;
         }
         if (!good) comError |= 1<<(i-1);   // set flag on failed communications
      }
   }

   return anygood;
}

char get_response(struct iomessage *message)
{
   char i, status;
   char rcvbuf[CMD_LENGTH];
   int rcvcount;
   char cksum;
   unsigned long tstart;

   message->command=0;  /* assume no response for now */
// P2P WORK TO BE DONE HERE ?????
   /* enable the receiver to get cmdlen characters */
   ser485Rx();  // should have been done already
   // rcvcount = CMD_LENGTH;
	t3=MS_TIMER;      // debug
   //ser_rec_z1(rcvbuf, &rcvcount);

   // align frame to <STX>
   while ((serDrdUsed() > 0) && (serDpeek() != STX)) serDgetc(); //printf("gr tossing %d\n",serDgetc());

   rcvcount = 0;
   tstart = MS_TIMER;
   while (rcvcount==0 && (MS_TIMER - tstart < RCV_TIMEOUT))
      rcvcount = serDread(rcvbuf, CMD_LENGTH, RCV_TIMEOUT);

	t4=MS_TIMER;      // debug
   if (rcvcount != CMD_LENGTH)
   {  /* not all characters received */
      // ser_kill_z1();       /* disable and return failure */
      ++commstat[5];       // increment status counter
	   #ifdef PRINT_ON
	      printf("\n INCOMPLETE INPUT: (%d) ",rcvcount);
	      for (i=0; i<rcvcount; i++) printf(" %d", rcvbuf[i]);
	   #endif
      status=FALSE;
   } else
   {  /* got the right number of characters */
      if ( (rcvbuf[0] != STX)
      || (rcvbuf[CMD_LENGTH-2] != ETX))
      {
		    ++commstat[6];       // increment status counter
	      #ifdef PRINT_ON
	         printf("\n BAD INPUT: No STX %d and/or ETX %d", rcvbuf[0],rcvbuf[CMD_LENGTH-2]);
	      #endif
		   status=FALSE;
      }
      else
      {  /* check the checksum */
	   	cksum=0;
	      for (i=0; i<CMD_LENGTH-1; i++) cksum += rcvbuf[i];

    		if (cksum != rcvbuf[CMD_LENGTH-1])
			{  /* bad checksum */
      		++commstat[4];       // increment status counter
	         #ifdef PRINT_ON
	           printf("\n CHECKSUM ERROR: Calculated %d not equal to xmitted %d",cksum, rcvbuf[CMD_LENGTH-1]);
	         #endif
	         status=FALSE;
	      }
	      else
	      {  /* command is good */
	         ++commstat[3];       // increment status counter - # returned
	         message->devAddr = rcvbuf[1];
	         message->command = rcvbuf[2];
	         message->station = rcvbuf[3];
		      for (i=0; i<NUMDATA; i++) message->data[i]=rcvbuf[i+4];
	         status=TRUE;
	      }
      }
   }
//printf(" :: %ld  %ld  %ld \n", t2-t1, t3-t2, t4-t3);    // debug
   return status;
}
/******************************************************************/
char txcounter;
char send_command(struct iomessage message)
{  /* Transmits a message/command to the remote system(s).
      Command string defined as:
      <STX> <devAddr> <Command> <Station> <Data0..3> <ETX> <CHKSUM> */

   // Set <devAddr> = ALL_DEVICES to transmit command to all devices
   char i,good,retry;
   char cmdstr[CMD_LENGTH];
   char cmdlen;
   char rcvbuf[CMD_LENGTH];
   unsigned long timeout, cmdsent;
   char mybuf[100];
   char tmpbuf[10];
   int bufchrs;
//unsigned long t1, t2;
   mybuf[0]=0;

   good=0;                        /* return value -- assume nogood*/
   retry=3;

   ++commstat[0];       // increment status counter -- total sent

   hitwd();             // hit watchdog incase comm loop
//msDelay(5);
	// first things first, send on UDP control socket
   sendRS485toUDP(message);
   good=1; // assume good

#ifdef USE_RS485
   // now send on RS485
   while (!good && retry)
   {
      // enable transmitter a bit sooner
      ser485Tx();

      /* assemble the full command string */
      cmdlen = CMD_LENGTH;
      cmdstr[0] = STX;
      cmdstr[1] = message.devAddr;                       // one or all devices
      cmdstr[2] = message.command;
      cmdstr[3] = message.station;
      for (i=0; i<NUMDATA; i++) cmdstr[i+4]=message.data[i];
      cmdstr[cmdlen-2] = ETX;
      cmdstr[cmdlen-1] = 0;
      for (i=0; i<cmdlen-1; i++) cmdstr[cmdlen-1] += cmdstr[i]; /* checksum */

      // enable transmitter and send
      //ser485Tx();
      cmdlen = serDwrite(cmdstr, CMD_LENGTH);
      disable485whenDone();
//t1 = MS_TIMER;
      // get ack if sending to one device
      if (message.devAddr != ALL_DEVICES)
      {  // try to throw out first character in case of immediate junk
	      // cmdlen = serDread(rcvbuf, 1, 0);
         /* should get an ACK or NAK within a short timeout */
         rcvbuf[0]=0;
         cmdlen=0;                       // one character answer
         timeout = MS_TIMER;
	      //serDrdFlush(); // try to flush the read buffer again
         while ((rcvbuf[0] != ACK) && (rcvbuf[0] != NAK) && (MS_TIMER - timeout < ACK_TIMEOUT))
         	cmdlen = serDread(rcvbuf, 1, 0);
//t2 = MS_TIMER;
//printf("\n%ld\n", t2-t1);
         // pre check for ACK, NAK and if not try getting another character
         //if ((cmdlen) && (rcvbuf[0] != ACK) && (rcvbuf[0] != NAK))
         //   { cmdlen = 0;
		   //      while (cmdlen==0 && (MS_TIMER - timeout < ACK_TIMEOUT))
		   //         cmdlen = serDread(rcvbuf, 1, 0);
         //   }

//   printf("it took %ld ms to get %d response %d\n", MS_TIMER-timeout, cmdlen, rcvbuf[0]);
//   if (rcvbuf[0] != ACK) {msDelay(1); printf("peeking at %d\n", serDpeek());}

         if (rcvbuf[0] == ACK)         // received and acknowleged
         {  good = 1;                  // TRUE
         }
         else if (rcvbuf[0] == NAK)    // Not received properly
         {
       		++commstat[8];             // increment status counter
				//if (PRINT_ON) printf("NAK  ");
	         #ifdef PRINT_ON
	           strcat(mybuf,"NAK ");
	         #endif
         }
         else if (rcvbuf[0])           // Unknown response
         {
       		++commstat[7];             // increment status counter
				//if (PRINT_ON) printf("DK  ");
				#ifdef PRINT_ON
               strcat(mybuf,"DK ");
               itoa((int)rcvbuf[0], tmpbuf);
               strcat(mybuf,tmpbuf);
            #endif
         }
         else                          // No response
         {
       		/* Re-initialize serial port 1 */
       		//ser_init_z1(4, 19200/1200);
            //serDclose();
            //serDopen(BAUD_RATE);
            ++commstat[9];                                  // increment status
				//if (PRINT_ON) printf("NIL  ");
				#ifdef PRINT_ON
            	strcat(mybuf,"NIL ");
            #endif
         }
         --retry;
      } else good = 1;
   }
   // Update communication statistics and flags
   if (good) { ++commstat[1]; }   // don't reset comError flag here
   else { ++commstat[2]; }

	#ifdef PRINT_ON
      if (strlen(mybuf) > 0) printf("%ld %s\n",SEC_TIMER, mybuf);
   #endif
#endif

   return good;
}

void show_comm_status()
{
   static unsigned long updateclock;
   char eventMsg[40];

   #GLOBAL_INIT
   {  updateclock = 0; }

   // now=SEC_TIMER;

   if (SEC_TIMER > updateclock)
   {
      updateclock=SEC_TIMER+2;

      // put up or remove communications status message
      if (comError)
      {  sys_message[MSG_NOCOMM+1].msg[MSG_OFFSET+11]=48+firstBit(comError);  // show device #
         message_add(SYSTEMLCD, MSG_NOCOMM+1, MSG_NOCOMM, NEXT);
      }
      else message_del(SYSTEMLCD, MSG_NOCOMM+1, MSG_NOCOMM);
      comError = availableDevices;  // reset for checking
   }
   // else if (updateclock-2 > SEC_TIMER) updateclock=SEC_TIMER;  // means updateclock is grossly out of whack

   // log to printer on failure and restore
//   if ((comError!=0) && (wasComError==0))
//   {   // error occured now
//   	strcpy(eventMsg, "LOST REMOTE COMMUNICATIONS @ DEV #");
//      eventMsg[33]=48+firstBit(comError);
//      print_event(eventMsg);
//      wasComError=comError;
//   } else if ((wasComError!=0) && (comError==0))
//   {  // error cleared now
//      print_event("REMOTE COMMUNICATIONS RESTORED");
//      wasComError=comError;
//   }
}

void show_comm_test(char wait)
{  // Show communication statistics on the LCD display
   char i, msgbuf[16];//, shortlbl[3];
   char y;

   if (wait)
   {  lcd_drawScreen(5, "COMMUNICATIONS");
	   lcd_Font("16B");
   }
   else
   {  // show first label only
      lcd_DispText(commlbl[0], 0, 0, MODE_NORMAL);
   }
   //strcpy(shortlbl, "#:"); // use single character label for nowait screen
   y=60;
   // loop through each statistic
   for (i=0; i<CSTATS; i++)
   {
      if (commstat[i])        // only show if non-zero
      {
        //lputs(0, commlbl[i]);		// write the descriptive label
        ltoa(commstat[i],msgbuf);
        //lputs(1,"                ");
        //lputs(1, msgbuf);			// write the statistic value
        if (wait)
        {  lcd_DispText(commlbl[i], 20, y, MODE_NORMAL);
	        lcd_DispText(msgbuf, 160, y, MODE_NORMAL);
	        y=y+20;
        } else // show on plain screen
        {  //shortlbl[0]=65+i; // replace first character with ABC label
           //lcd_DispText(shortlbl, 0, 0, MODE_NORMAL);
	        //lcd_DispText(":", 0, 0, MODE_NORMAL);
	        lcd_DispText(" ", 0, 0, MODE_NORMAL);
	        lcd_DispText(msgbuf, 0, 0, MODE_NORMAL);
        }
        //msDelay(1500);
      }
   }
   if (wait) // wait for any button press or timeout
	{  menu_timeout = SEC_TIMER + 60;
	   while( (lcd_GetTouch(100) == -1) && !secTimeout(menu_timeout) ) maintenance();
   }
   else
	   lcd_DispText("\n", 0, 0, MODE_NORMAL);
   	msDelay(1500);
}
void show_ip_address(void)
{  // Show ip address on the LCD display
   unsigned long ipAddr, netMask, router;
   char HWA[6];
	char s_ipAddr[20];
   char s_outBuf[40];

   lcd_drawScreen(5, "IP CONFIGURATION");
	lcd_Font("16B");
   // get ip info
	ifconfig(IF_ETH0,
       IFG_IPADDR, &ipAddr,
       IFG_NETMASK, &netMask,
       IFG_ROUTER_DEFAULT, &router,
       IFG_HWA, HWA,
       IFS_END);

   // Convert to dotted string and display with a label
   inet_ntoa(s_ipAddr, ipAddr);
   sprintf(s_outBuf, "%-15.15s", s_ipAddr);
   lcd_DispText("IP ADDR", 40, 80, MODE_NORMAL); lcd_DispText(s_outBuf, 120, 80, MODE_NORMAL);
   inet_ntoa(s_ipAddr, netMask);
   sprintf(s_outBuf, "%-15.15s", s_ipAddr);
   lcd_DispText("NETMASK", 40, 100, MODE_NORMAL); lcd_DispText(s_outBuf, 120, 100, MODE_NORMAL);
   inet_ntoa(s_ipAddr, router);
   sprintf(s_outBuf, "%-15.15s", s_ipAddr);
   lcd_DispText("ROUTER", 40, 120, MODE_NORMAL); lcd_DispText(s_outBuf, 120, 120, MODE_NORMAL);
   sprintf(s_outBuf, "%02X-%02X-%02X-%02X-%02X-%02X",
   	(int)HWA[0], (int)HWA[1], (int)HWA[2], (int)HWA[3], (int)HWA[4], (int)HWA[5]);
   lcd_DispText("MAC ADDR", 40, 140, MODE_NORMAL); lcd_DispText(s_outBuf, 120, 140, MODE_NORMAL);

	menu_timeout = SEC_TIMER + 60;
   while( (lcd_GetTouch(100) == -1) && !secTimeout(menu_timeout) ) maintenance();
}
/******************************************************************/
void checkRemoteConfiguration(void)
{  // determine which remotes are connected to the bus

   struct iomessage testcmd, testrsp;
   char i, workset, mymsg[20], mybuf[10];
   int j;

// P2P WORK TO BE DONE HERE
if (param.point2point) return;

   slaveAvailable = FALSE;  // ASSUME NOT
   FIRST_DEVADDR=0;   // number
   LAST_DEVADDR=0;    // number
   workset=0;
   //ALL_DEVADDRS=((1 << MAX_DEVADDRS) - 1 ) | 0x80; // bits ... including high bit
   availableDevices = 0xFF;  // address all possible devices

   //lcd_print(SYSTEMLCD, 0, " CHECKING REMOTE");
   //lcd_print(SYSTEMLCD, 1, " CONFIGURATION  ");
	lcd_DispText("Detecting RS485 Devices...\n",0, 0, MODE_NORMAL);
   // hold for a few seconds
   //msDelay(100);

   strcpy(mymsg, "  ");
   STATION_SET = 0;                  // startoff with none
//// TEMPORARILY REMOVED TO SPEED STARTUP
/*//   for (j=1; j<=MAX_DEVADDRS; j++)
   {  mybuf[0]=48+j; mybuf[1]=32; mybuf[2]=0;
	   lcd_DispText(mybuf, 0, 0, MODE_NORMAL);
      hitwd();
      testcmd.station=0;
      testcmd.devAddr=j;
      testcmd.data[0]=0;  // main ok to receive
      testcmd.data[1]=IDLE_STATE;  // system state
      testcmd.data[2]=0; // slave inUse ON
      testcmd.data[3]=0; // slave inUse Flash
      testcmd.data[4]=STATION_SET; // who's available
      testcmd.command=RETURN_INPUTS;
      msDelay(100);       // seems to improve communications
      i = send_n_get( testcmd, &testrsp );      // are you alive
      if (testrsp.devAddr==testcmd.devAddr)
      {  // this one is alive
         STATION_SET |= testrsp.station;        // who do you own
         if (FIRST_DEVADDR==0) FIRST_DEVADDR=j;
         LAST_DEVADDR=j;
			// if (testrsp.station & SLAVE_b) slaveAvailable=TRUE;
			if (testrsp.devAddr == SLAVE_DEVADDR) slaveAvailable=TRUE;
         workset |= (1 << (j-1));                // include this bit
         itoa(j, mybuf);
         strcat(mymsg, " #");
         strcat(mymsg, mybuf);
      }
   }
*///

   STATION_SET &= param.activeStations;  // allow only those programmed in diag menu
   availableDevices = workset; // | 0x80;     // also set high bit, meaning ALL

   if (strlen(mymsg) <= 3) strcpy(mymsg, "- NONE -");

   // pad the message buffer
   //for (j=strlen(mymsg); j<16; j++) strcat(mymsg, " ");

   // show the results of the query
   //lcd_print(SYSTEMLCD, 0, "RESPONSE OK FROM");
   //lcd_print(SYSTEMLCD, 1, mymsg);
   lcd_DispText("\nResponse Received From ",0, 0, MODE_NORMAL);
   lcd_DispText(mymsg,0, 0, MODE_NORMAL);
   lcd_DispText("\n", 0, 0, MODE_NORMAL);

   // reset communications statistics
   init_communication();

   // hold for a few seconds
   //msDelay(2000);
}
void UDPcheckRemoteConfiguration(void)
{  // determine which remotes are connected to the bus

   // struct iomessage testcmd, testrsp;
	struct UDPmessageType UDPmessage;
   char mymsg[30];
   char myVer;
   int rcvCount, deviceSet, lastDeviceSet;
   int deviceSetD, lastDeviceSetD;
   unsigned long checkTill;

   // data index for incoming version & subversion depends on mode
	if (param.slaveController){ myVer = 13; }
   else {myVer = 5;}

	// Listen on the UDP control socket for activity
   slaveAvailable = FALSE;  // ASSUME NOT
   lcd_ClearScreen();
	lcd_DispText("Detecting UDP Devices...\n",0, 0, MODE_NORMAL);
   // hold for a few seconds
   //msDelay(100);

   // strcpy(mymsg, "                   ");
   STATION_SET = 0;                  // startoff with none
   deviceSet = 0;
   lastDeviceSet = 0;
   deviceSetD = 0;
   lastDeviceSetD = 0;
   rcvCount = 0;
   FIRST_DEVADDR=1;
   LAST_DEVADDR=1; // MUST ALWAYS BE AT LEAST ONE?

	// listen for 2 seconds
   checkTill = MS_TIMER + 2000;
   while (MS_TIMER <= checkTill)
   {
      // retrieve a packet
	   if (getUDPcommand(&UDPmessage) > 0)
      {  processUDPcommand(UDPmessage);  // process it as normal first
      	rcvCount=rcvCount+1;
      	// check who it is
         if (UDPmessage.device > 0)
         {  // include the device in the deviceSet
         	if (UDPmessage.device > LAST_DEVADDR) LAST_DEVADDR = UDPmessage.device;
         	if (UDPmessage.devType == 2)
            { // capture diverter device
	         	deviceSetD |= (1 << (UDPmessage.device-1));
            } else
	         { // capture remote station
            	deviceSet |= (1 << (UDPmessage.device-1));
            }
	         // If new, show this device & station
            if ((deviceSet != lastDeviceSet) || (deviceSetD != lastDeviceSetD))
            {	sprintf(mymsg, "Dev %d-%d; Sta %d; Ver %d.%d\n", UDPmessage.devType, UDPmessage.device, UDPmessage.station,
	            	UDPmessage.data[myVer],UDPmessage.data[myVer+1]);
				   lcd_DispText(mymsg, 0, 0, MODE_NORMAL);
               lastDeviceSet = deviceSet;
               lastDeviceSetD = deviceSetD;
					if (UDPmessage.device == SLAVE_DEVADDR) slaveAvailable=TRUE;
          	}
         }
         // check which station(s)
         if (UDPmessage.station > 0)
         {
	         STATION_SET |= (1 << (UDPmessage.station-1));        // who do you own
         }
      }
      // hit & tickle
      hitwd();
	   tcp_tick(NULL);
      //if (FIRST_DEVADDR==0) FIRST_DEVADDR=j;
      //LAST_DEVADDR=j;
   }
   STATION_SET &= param.activeStations;  // allow only those programmed in diag menu
   availableDevices = deviceSet; //

   // show the results of the process
   lcd_DispText("\nMessages Received: ",0, 0, MODE_NORMAL);
   sprintf(mymsg, "%d\n", rcvCount);
   lcd_DispText(mymsg,0, 0, MODE_NORMAL);

   // hold for a few seconds
   msDelay(2000);
}

nodebug void reset_statistics()
{
   // reset system statistics
   statistics.trans_in=0;
   statistics.trans_out=0;
   statistics.deliv_alarm=0;
   statistics.divert_alarm=0;
   statistics.cic_lift_alarm=0;
}

void time2string(unsigned long time, char *buf, int format)
{  // Converts the time value into a string as specified by format
   // time    time value to be converted
   // buf     destination string ... make sure you allocate enough room
   // format  how the time should be converted
   //         1 = HH:MM:SS     2 = HH:MM
   //         3 = MM/DD/YY     4 = MM/DD/YY HH:MM:SS

   struct tm mytime;
   char * myptr;

   // convert long time to structure
   mktm( &mytime, time);
   myptr=buf;

   // convert date
   if (format==3 || format==4)
   {
      // fill in the date values
      *myptr++ = 48 + (mytime.tm_mon / 10);
      *myptr++ = 48 + (mytime.tm_mon % 10);
      *myptr++ = '/';
      *myptr++ = 48 + (mytime.tm_mday / 10);
      *myptr++ = 48 + (mytime.tm_mday % 10);
      *myptr++ = '/';
      *myptr++ = 48 + ((mytime.tm_year / 10) % 10);
      *myptr++ = 48 + (mytime.tm_year % 10);

   }

   // convert time
   if (format==1 || format==2 || format==4)
   {
      if (format==4) *myptr++ = 32;   // space seperator

      // fill in the time values
      *myptr++ = 48 + (mytime.tm_hour / 10);
      *myptr++ = 48 + (mytime.tm_hour % 10);
      *myptr++ = ':';
      *myptr++ = 48 + (mytime.tm_min / 10);
      *myptr++ = 48 + (mytime.tm_min % 10);

      if (format==1 || format==4)  // format seconds
      {  *myptr++ = ':';
         *myptr++ = 48 + (mytime.tm_sec / 10);
         *myptr++ = 48 + (mytime.tm_sec % 10);
      }
   }

  *myptr = 0;    // null terminator

}

/******************************************************/
// Printout functions
/******************************************************/
void print_event(char *event)
{
   char myline[80];     // line to send to the printer
   char * instr;        // work pointer into myline

   instr=myline;
   time2string(SEC_TIMER, instr, 4);      // event time
   strcat(myline, " ");
   strcat(myline, event);
///   reset_printer();  // reset/retry printer each time
///   print_line(myline);
	printf("\n"); printf(myline);

}
/*
void print_malfunction(struct trans_log_type log)
{
   char myline[80];     // line to send to the printer
   char * instr;        // work pointer into myline


   strcpy(myline, "ALARM ");
   instr=myline+strlen(myline);
   time2string(log.duration, instr, 4);      // malfunction time
   strcat(myline," ");
   instr=myline+strlen(myline);
   switch (log.status & 0x0F)
   {
   case STS_DIVERTER_TOUT: strcat(myline, "DIVERTER TIMEOUT");		break;
   case STS_DEPART_TOUT: 	strcat(myline, "DEPARTURE TIMEOUT");	break;
   case STS_ARRIVE_TOUT: 	strcat(myline, "DELIVERY OVERDUE");   	break;
   case STS_BLOWER_TOUT: 	strcat(myline, "BLOWER TIMEOUT");     	break;
   case STS_TRANS_CANCEL: 	strcat(myline, "CANCEL DISPATCH");    	break;
   //case 5: strcat(myline, "COMMUNICATIONS LOST"); 	break;
	}

///   reset_printer();  // reset/retry printer each time
///   print_line(myline);

   // print_transaction( log );  // now print normal log

}


void print_transaction(struct trans_log_type log)
{
   char myline[80];     // line to send to the printer
   char * instr;        // work pointer into myline
   long transtime;      // how long transaction took

   // format the message line
   time2string(log.start_tm, myline, 4);      // start time
   strcat(myline," ");
   instr=myline+strlen(myline);
   sprintf(instr, "%-12s", param.station_name[log.source_sta].name);
   strcat(myline, "-->> ");
   instr=myline+strlen(myline);
   time2string(log.duration, instr, 1);
   transtime=log.duration-log.start_tm;
   instr=myline+strlen(myline);
   sprintf(instr, " %-12s %4ld sec", param.station_name[log.dest_sta].name, transtime);

///   reset_printer();  // reset/retry printer each time
///   print_line(myline);
} */
/******************************************************/
/*const char prLine[] = "-----------------------------------------------------------------";
void print_summary(struct stats_type stats, char how)
{
   // stats  :  system summary statistics
   // how    :  automatic report (0) or on-demand report (1)

   char myline[80];     // line to send to the printer
   char * instr;        // work pointer into myline
   long subtot;         // total of transactions or alarms
/*
   reset_printer();  // reset/retry printer each time
   print_line(prLine);
   print_line(station_name[SYSTEM_NAME].name);  // print system name
   if (how==0) {   strcpy(myline, "  Daily ");
   } else { strcpy(myline, "  On-Demand "); }
   strcat(myline, "Transaction Summary Printed ");
   instr = myline + strlen(myline);
   time2string(clock(), instr, 4);
   print_line(myline);
   print_line(prLine);
   print_line(" Transactions:");
   sprintf(myline, "    Incoming:              %ld", stats.trans_in);
   print_line(myline);
   sprintf(myline, "    Outgoing:              %ld", stats.trans_out);
   print_line(myline);
   sprintf(myline, "    Daily Total:           %ld", stats.trans_in + stats.trans_out);
   print_line(myline);
   sprintf(myline, "    Grand Total:           %ld", transactionCount());
   print_line(myline);
   print_line(" ");
   print_line(" Alarms:");
   sprintf(myline, "    Incomplete Delivery:   %d", stats.deliv_alarm);
   print_line(myline);
   sprintf(myline, "    Diverter Timeout:      %d", stats.divert_alarm);
   print_line(myline);
   sprintf(myline, "    Other Timeouts:        %d", stats.cic_lift_alarm);
   print_line(myline);
   subtot=stats.deliv_alarm+stats.divert_alarm+stats.cic_lift_alarm;
   sprintf(myline, "    Total:                 %ld", subtot);
   print_line(myline);
   print_line(prLine);

}*/
/*
void check_auto_report(struct stats_type stats)
{

   if (SEC_TIMER >= nextAutoPrint)
   {
      // time to print automatic summary report
      print_summary(stats, 0);

      // increment to next day (86400 seconds per day)
      while (nextAutoPrint <= SEC_TIMER) { reset_auto_print(); }

      // reset statistics
      reset_statistics();
   }
}
void reset_auto_print()
{  // set auto print statistics for midnight tomorrow
   nextAutoPrint = (((SEC_TIMER+1) / 86400) +1) * 86400;
} */

//*************************************
// functions for master/slave operation
//*************************************
void syncDateAndTime()
{  // for master to send the date/time to the slave
   struct iomessage command;
	struct tm time;
   tm_rd(&time);  // get the time from the RTC

   command.devAddr=ALL_DEVICES;
   command.command=SET_DATE_TIME;
   //command.station=0;
   command.station=time.tm_sec;  // put seconds into station
   command.data[0] = time.tm_year;
   command.data[1] = time.tm_mon;
   command.data[2] = time.tm_mday;
   command.data[3] = time.tm_hour;
   command.data[4] = time.tm_min;
   send_command(command);
}
void syncTransCount()
{  // for master to send traction count to slave
   struct iomessage command;
   char *p;

   p=(char*)&transaction_count;

   command.devAddr=ALL_DEVICES;
   command.command=SET_TRANS_COUNT;
   command.station=0;
   command.data[0] = *p++;
   command.data[1] = *p++;
   command.data[2] = *p++;
   command.data[3] = *p;
   command.data[4] = 0;
   msDelay(10);  // just sent a command so wait a bit
   send_command(command);
}
void displayRemoteMsgs(char * message_num)
{  // for master to send lcd messages to slave
   struct iomessage command;
   char i;

   command.devAddr = ALL_DEVICES;
   command.command = DISPLAY_MESSAGE;
   for (i=0; i<4; i++) command.data[i]=message_num[i];

   send_command(command);
}

nodebug
void msDelay(unsigned int delay)
{
   auto unsigned long start_time;
   start_time = MS_TIMER;
   //if (delay < 500) while( (MS_TIMER - start_time) <= delay );
   //else
   while( (MS_TIMER - start_time) <= delay ) hitwd();
}
nodebug char secTimeout(unsigned long ttime)
{
   return (ttime > SEC_TIMER) ? FALSE : TRUE;
}
char Timeout(unsigned long start_time, unsigned long duration)
{
   return ((MS_TIMER - start_time) < duration) ? FALSE : TRUE;
}
void lcd_helpScreen(char view)
{  // view not applicable - reserved for future use
	int button;

   // Logo splash screen
	lcd_BFcolorsB( lcd_BLACK, lcd_WHITE);
	lcd_ClearScreen ();
   lcd_Font("13");
   lcd_DispText(param.station_name[SYSTEM_NAME].name, 10, 5, MODE_NORMAL);
   lcd_DispText(" - ", 0, 0, MODE_NORMAL);
   lcd_DispText(FIRMWARE_VERSION, 0, 0, MODE_NORMAL);
   lcd_DispText("\nTo send a carrier:", 0, 0, MODE_NORMAL);
   lcd_DispText("\nInsert carrier into tube and close the door", 0, 0, MODE_NORMAL);
	if (param.point2point==TRUE)
   {  // show only point to point information
	   lcd_DispText("\nPress the send button", 0, 0, MODE_NORMAL);
   } else
	{	// show full system information
	   lcd_DispText("\nPress the send button for the destination", 0, 0, MODE_NORMAL);
	   lcd_DispText("\nOR", 0, 0, MODE_NORMAL);
	   lcd_DispText("\nPress the Send To Directory button", 0, 0, MODE_NORMAL);
	   lcd_DispText("\nPress the send button for the destination", 0, 0, MODE_NORMAL);
	   lcd_DispText("\n", 0, 0, MODE_NORMAL);
	   lcd_DispText("\nPress the STAT button for priority dispatch", 0, 0, MODE_NORMAL);
	   lcd_DispText("\nPress the Secure button to enable secure handling", 0, 0, MODE_NORMAL);
	   lcd_DispText("\nPress the Auto Return button to enable automatic return", 0, 0, MODE_NORMAL);
	   lcd_DispText("\nPress the Return Carrier button to return a carrier", 0, 0, MODE_NORMAL);
	}
   lcd_DispText("\nPress the Menu button to access system features", 0, 0, MODE_NORMAL);
   lcd_DispText("\n", 0, 0, MODE_NORMAL);
   lcd_DispText("\nHelp Phone Numbers", 0, 0, MODE_NORMAL);
   lcd_DispText("\nSystem Administrator: ", 0, 0, MODE_NORMAL);
   lcd_DispText(param.phoneNum[ADMIN_PHONE], 0, 0, MODE_NORMAL);
   lcd_DispText("\nMaintenance: ", 0, 0, MODE_NORMAL);
   lcd_DispText(param.phoneNum[MAINT_PHONE], 0, 0, MODE_NORMAL);
   lcd_DispText("\nColombo Pneumatic Tube Systems: 800-547-2820  ", 0, 0, MODE_NORMAL);

	lcd_Font("16B");
   lcd_ButtonDef( BTN_CANCEL,
	   BTN_MOM,                      // momentary operation
      BTN_TYPE, BUTTON_RELEASE,
      BTN_MARGINS, 5, 330,         // set left and right margins
      BTN_SEP, 63, 43,              // set up for button sep
	   BTN_TXTOFFSET, 10, 9,
	   //BTN_TLXY, 257, 205,             // starting x,y for buttons
	   BTN_TLXY, lcd_max_x-63, lcd_max_y-35,             // starting x,y for buttons
	   BTN_TEXT, "Done",
	   BTN_BMP, BMP_med_button, BMP_med_button_dn,
	   BTN_END );

   // wait for a button press
   menu_timeout = SEC_TIMER + 60;
   while(((button=lcd_GetTouch(100)) == -1) && !secTimeout(menu_timeout) ) maintenance();

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
	   lcd_DispText("info@colombopts.com\n", 0, 0, MODE_NORMAL);
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
   //  8: transaction log: pgup, pgdn, done
   //  9: header, keypad, next, exit
   // 10: header keyboard, OK, cancel
   int x, dx, dy;
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
//	   lcd_Rectangle(5, 5, lcd_max_x-5, 40, 1);        // inside paint
	   lcd_Rectangle(1, 1, lcd_max_x-1, 40, 1);        // inside paint
	   lcd_BFcolorsB( lcd_BLACK, lcd_WHITE);   // border color
//	   lcd_Rectangle(5, 5, lcd_max_x-5, 40, 0);        // border paint
	   lcd_Rectangle(0, 0, lcd_max_x, 40, 0);        // border paint
	   lcd_Font("24B");
	   lcd_DispText(title, lcd_Center(title, 1), 14, MODE_TRANS);
   }

   // now handle screen specific stuff
   switch (whichScreen)
   {
   case 1:
	   if (title[0] != 0) // Buttons? (any) or not ("")
		{
			if (param.point2point==FALSE)
	      {
	         // Draw a box for top row of bottom buttons - Transaction Options
            ypos = (param.orientLandscape) ? 197 : 270;
            xpos = (param.orientLandscape) ? 5 : 0; // xpos as border offset
	         lcd_BFcolorsB( lcd_LBLUE, lcd_LBLUE );   // inside color
	         lcd_Rectangle(xpos, 139, lcd_max_x-xpos, ypos, 1);      // inside paint 158 -> 140
	         lcd_BFcolorsB( lcd_BLUE, lcd_WHITE);     // border color
	         lcd_Rectangle(xpos, 139, lcd_max_x-xpos, ypos, 0);      // border paint
	         lcd_Font("13B");
	         lcd_BFcolorsB( lcd_BLACK, lcd_WHITE);   // border color
            xpos = (param.orientLandscape) ? 15 : 25;
	         lcd_DispText("---- Transaction Options ----", xpos, 143, MODE_TRANS);

	         // Draw a box for Return from
            ypos = lcd_max_y-43;
            if (param.orientLandscape)
            {  // only for landscape
	            lcd_BFcolorsB( lcd_LBLUE, lcd_LBLUE );   // inside color
	            lcd_Rectangle(lcd_max_x-105, ypos, lcd_max_x-5, ypos+40, 1);      // inside paint
	            lcd_BFcolorsB( lcd_BLUE, lcd_WHITE);     // border color
	            lcd_Rectangle(lcd_max_x-105, ypos, lcd_max_x-5, ypos+40, 0);      // border paint
            }
	         // Draw a box for menu/help
            xpos = (param.orientLandscape) ? 5 : 0;
            dx = (param.orientLandscape) ? 142 : 239;
				ypos = (param.orientLandscape) ? 196 : 270;
            dy = (param.orientLandscape) ? 40 : 49;
	         lcd_BFcolorsB( lcd_LGREEN, lcd_LGREEN );   // inside color
	         lcd_Rectangle(xpos, ypos, dx, ypos+dy, 1);      // inside paint
	         lcd_BFcolorsB( lcd_BLUE, lcd_WHITE);     // border color
////      lcd_Rectangle(5, ypos, 215, ypos+40, 0);      // border paint
	         lcd_Rectangle(xpos, ypos, dx, ypos+dy, 0);      // border paint

	         // Draw bottom buttons
	         lcd_BFcolorsD( lcd_BLACK, lcd_VLTGRAY_D );
	         lcd_Font("16B");
            xpos = (param.orientLandscape) ? 13 : 23;
	         lcd_ButtonDef( BTN_MENU,
	            BTN_MOM,                      // momentary operation
	            BTN_TLXY, xpos, lcd_max_y-38, //5, 205,             // starting x,y for buttons
	            BTN_TYPE, BUTTON_RELEASE,
	            BTN_MARGINS, 5, 330,         // set left and right margins
	            BTN_SEP, 63, 43,              // set up for button sep
	            BTN_TEXT, "Menu",
	            BTN_TXTOFFSET, 10, 9,
	            BTN_BMP, BMP_med_button, BMP_med_button_dn,
	            BTN_END );

            xpos = (param.orientLandscape) ? 80 : 157;
	         lcd_ButtonDef( BTN_HELP,
	            //BTN_TLXY, 257, 205,
	            BTN_TLXY, xpos, lcd_max_y-38, //70, 205,
	            BTN_TXTOFFSET, 12, 9,
	            BTN_TEXT, "Help",
	            BTN_BMP, BMP_med_button, BMP_med_button_dn,
	            BTN_END );

	         //lcd_Font("13");
	         //lcd_Font("13B");
            if (param.orientLandscape)
            {
	            xpos = 220;
	            ypos = 153;
	            lcd_ButtonDef( BTN_DIRECTORY,
	               //BTN_TLXY, 114, 198,
	               BTN_TLXY, xpos, ypos, //135, 198,
	               //BTN_TEXT, "  Station\nDirectory",
	               //BTN_TEXT, "Send to /\nDirectory",
	               BTN_TEXT, " Send a\nCarrier",
	               BTN_TXTOFFSET, 15, 4, //9, //14, 4,
	               //BTN_BMP, BMP_92x40_button, BMP_92x40_button_dn,
	               BTN_BMP, BMP_92x40_button, BMP_92x40_button_dn,
	               BTN_END );
	            //lcd_Font("16B");

	            lcd_Font("13B");
	            ypos = 202;
	            lcd_ButtonDef( BTN_CRETURN,
	               //BTN_TLXY, 257, 160,
	               BTN_TLXY, xpos, ypos, //70, 160,
	               BTN_TXTOFFSET, 15, 3,
	               BTN_TEXT, "Return a\n Carrier",
	               BTN_BMP, BMP_92x32_button, BMP_92x32_button_dn,
	               BTN_END );
            } else
            {
	            xpos = 15;
	            ypos = 212;
	            lcd_ButtonDef( BTN_DIRECTORY,
	               BTN_TLXY, xpos, ypos, //135, 198,
	               BTN_TEXT, "  SEND\nCARRIER",
	               BTN_TXTOFFSET, 17, 4, //9, //14, 4,
	               //BTN_BMP, BMP_92x40_button, BMP_92x40_button_dn,
	               BTN_BMP, BMP_92x40_button, BMP_92x40_button_dn,
	               BTN_END );
	            //lcd_Font("16B");

	            //lcd_Font("13B");
	            xpos = 135;
	            lcd_ButtonDef( BTN_CRETURN,
	               BTN_TLXY, xpos, ypos, //70, 160,
	               BTN_TXTOFFSET, 18, 4,
	               BTN_TEXT, "RETURN\nCARRIER",
	               BTN_BMP, BMP_92x40_button, BMP_92x40_button_dn,
	               BTN_END );
            }
	//       lcd_Font("16B");

	         lcd_Font("13B");
            dx = (param.orientLandscape) ? 0 : 10;
	         lcd_ButtonDef( BTN_ARETURN, BTN_LAT,
	            //BTN_TLXY, 114, 160,
	            BTN_TLXY, dx+147, 161,
	            BTN_TYPE, autoRtnTrans ? BUTTON_LAT_1 : BUTTON_LAT_0,
	            BTN_TXTOFFSET, 7, 3, 7, 3,
	            BTN_TEXT, " Auto\nReturn", " Auto\nReturn",
	            BTN_BMP, BMP_med_button, BMP_med_grn_button_dn, // BMP_92x32_button, BMP_92x32_button_dn,
	            BTN_END );
	         lcd_Font("16B");

	         lcd_ButtonDef( BTN_STAT, BTN_LAT,
	            BTN_TLXY, dx+13, 161,
	            BTN_TYPE, statTrans ? BUTTON_LAT_1 : BUTTON_LAT_0,
	            BTN_TEXT, "STAT", "STAT",
	            BTN_TXTOFFSET, 12, 9, 12, 9,
	            BTN_BMP, BMP_med_button, BMP_med_grn_button_dn,
	            BTN_END );
	         // show secure button only if the feature is enabled
	         if (param.secureEnabled)
	            lcd_ButtonDef( BTN_SECURE, BTN_LAT,
	               BTN_TLXY, dx+80, 161,
	               BTN_TYPE, secureTrans ? BUTTON_LAT_1 : BUTTON_LAT_0,
	               BTN_TEXT, "Secure", "Secure",
	               BTN_TXTOFFSET, 7, 9, 7, 9,
	               BTN_BMP, BMP_med_button, BMP_med_grn_button_dn,
	               BTN_END );
	         else
	            lcd_ButtonDef( BTN_FUTURE, BTN_MOM,
	               BTN_TLXY, dx+80, 161,
	               BTN_TYPE, BUTTON_RELEASE,
	               BTN_TEXT, "Future",
	               BTN_TXTOFFSET, 7, 9,
	               BTN_BMP, BMP_med_button, BMP_med_button,
	               BTN_END );


	         // Show CIC light on the screen if we are also showing buttons
	         lcd_showCIC(1);
	      }
   		else // point to point stuff param.point2point
	      {
	         // Draw a box for menu/help and logo
	         lcd_BFcolorsB( lcd_LGREEN, lcd_LGREEN );   // inside color
	         lcd_Rectangle(0, 139, 95, 240, 1);      // inside paint
	         lcd_Rectangle(225, 139, 319, 240, 1);      // inside paint
	         lcd_BFcolorsB( lcd_BLUE, lcd_WHITE);     // border color
	         lcd_Rectangle(0, 139, 319, 240, 0);      // border paint
	         lcd_Rectangle(95, 139, 225, 240, 0);      // border paint

	         // Draw bottom buttons
	         lcd_BFcolorsD( lcd_BLACK, lcd_VLTGRAY_D );
	         lcd_Font("16B");
	         lcd_ButtonDef( BTN_MENU,
	            BTN_MOM,                      // momentary operation
	            BTN_TLXY, 20, 172, //5, 205,             // starting x,y for buttons
	            BTN_TYPE, BUTTON_RELEASE,
	            BTN_MARGINS, 5, 330,         // set left and right margins
	            BTN_SEP, 63, 43,              // set up for button sep
	            BTN_TEXT, "Menu",
	            BTN_TXTOFFSET, 10, 9,
	            BTN_BMP, BMP_med_button, BMP_med_button_dn,
	            BTN_END );

	         lcd_ButtonDef( BTN_HELP,
	            //BTN_TLXY, 257, 205,
	            BTN_TLXY, 243, 172, //70, 205,
	            BTN_TXTOFFSET, 12, 9,
	            BTN_TEXT, "Help",
	            BTN_BMP, BMP_med_button, BMP_med_button_dn,
	            BTN_END );

	         lcd_Font("24B");

            // Draw logo
				lcd_DispBitmap( BMP_Front_LOGO, 100, 169 );

	         //lcd_ButtonDef( BTN_SEND,
	         //   BTN_TLXY, 55, 147, //135, 198,
	         //   BTN_TEXT, "SEND",
	         //   BTN_TXTOFFSET, 17, 8, //9, //14, 4,
	         //   BTN_BMP, BMP_92x40_button, BMP_92x40_button_dn,
	         //   BTN_END );
	         //lcd_Font("16B");

	         //lcd_Font("16B");
	         //lcd_ButtonDef( BTN_CRETURN,
	         //   BTN_TLXY, 175, 147, //70, 160,
	         //   BTN_TXTOFFSET, 13, 8,
	         //   BTN_TEXT, "Return",
	         //   BTN_BMP, BMP_92x40_button, BMP_92x40_button_dn,
	         //   BTN_END );

	      }
      }
	   // Draw LCD Message boxes
	   //lcd_BFcolorsB( lcd_LGREY, lcd_LGREY);  // inside color
	   //lcd_Rectangle(5, 5, 315, 71, 1);       // inside paint
	   //lcd_Rectangle(5, 71, 315, 137, 1);     // inside paint
	   lcd_BFcolorsB( lcd_BLUE, lcd_LGREY);  // border color
	   //lcd_Rectangle(5, 5, lcd_max_x-5, 72, 0);       // border paint
	   //lcd_Rectangle(5, 72, lcd_max_x-5, 139, 0);     // border paint
	   lcd_Rectangle(0, 0, lcd_max_x, 72, 0);       // border paint
	   lcd_Rectangle(0, 72, lcd_max_x, 139, 0);     // border paint
     	lcd_refreshMsgs();
      // since this screen was just cleared, force a refresh of the lcd messages
      reset_msg_timer(NOW);

      break;

   case 2:

	   // Draw bottom buttons
	   lcd_Font("16B");
	   lcd_ButtonDef( BTN_PREV,
	      BTN_MOM,                      // momentary operation
	      BTN_TLXY, 5, lcd_max_y-35,             // starting x,y for buttons
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
	      BTN_TLXY, lcd_max_x-63, lcd_max_y-35,             // starting x,y for buttons
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
	      BTN_TLXY, 5, lcd_max_y-35,             // starting x,y for buttons
		   BTN_TYPE, BUTTON_RELEASE,
	      BTN_TEXT, "OK",
	      BTN_MARGINS, 5, 330,         // set left and right margins
	      BTN_TXTOFFSET, 14, 9,
	      BTN_BMP, BMP_med_button, BMP_med_button_dn,
	      BTN_END );
	   lcd_ButtonDef( BTN_CANCEL,
	      BTN_TXTOFFSET, 5, 9,
	      BTN_TLXY, lcd_max_x-63, lcd_max_y-35,           // starting x,y for buttons
	      BTN_TEXT, "Cancel",
	      BTN_END );
      break;
   case 5:

	   // Draw bottom buttons
	   lcd_Font("16B");
	   lcd_ButtonDef( BTN_EXIT,
	      BTN_MOM,                      // momentary operation
	      BTN_TLXY, lcd_max_x-63, lcd_max_y-35,             // starting x,y for buttons
		   BTN_TYPE, BUTTON_RELEASE,
	      BTN_MARGINS, 5, 330,         // set left and right margins
	      BTN_SEP, 63, 43,              // set up for button sep
	      BTN_TEXT, "Exit",
	      BTN_TXTOFFSET, 15, 9,
	      BTN_BMP, BMP_med_button, BMP_med_button_dn,
	      BTN_END );
      break;
   case 6:  // keyboard entry
      lcd_drawKeyboard();
	   // Draw bottom buttons
	   lcd_Font("16B");
	   lcd_ButtonDef( BTN_NEXT,
	      BTN_MOM,                      // momentary operation
	      BTN_TLXY, 5, lcd_max_y-35,             // starting x,y for buttons
		   BTN_TYPE, BUTTON_RELEASE,
	      BTN_TEXT, "Next",
	      BTN_MARGINS, 5, 330,         // set left and right margins
	      BTN_TXTOFFSET, 8, 9,
	      BTN_BMP, BMP_med_button, BMP_med_button_dn,
	      BTN_END );
	   lcd_ButtonDef( BTN_CANCEL,
	      BTN_MOM,                      // momentary operation
	      BTN_TLXY, lcd_max_x-63, lcd_max_y-35,             // starting x,y for buttons
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
	      BTN_TLXY, lcd_max_x-63, lcd_max_y-35,             // starting x,y for buttons
		   BTN_TYPE, BUTTON_RELEASE,
	      BTN_MARGINS, 5, 330,         // set left and right margins
	      BTN_SEP, 63, 43,              // set up for button sep
	      BTN_TEXT, "Exit",
	      BTN_TXTOFFSET, 15, 9,
	      BTN_BMP, BMP_med_button, BMP_med_button_dn,
	      BTN_END );
      // print date/time message
      lcd_showClockMessage();
      //lcd_Font("14x24"); // "16x32i");
	   //lcd_DispText(sys_message[MSG_DATE_TIME].msg, 10, 60, MODE_NORMAL);
      // Draw up/dn buttons
      lcd_ButtonDef( 0, // month +
	      BTN_MOM,                      // momentary operation
	      BTN_TLXY,    30, 100,             // starting x,y for buttons
	      BTN_MARGINS, 30, 210,         // set left and right margins
		   BTN_TYPE, BUTTON_RELEASE,
	      BTN_SEP, 41, 40,              // set up for button sep
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

   case 8:  // PgUp, PgDn, Done
	   // Draw bottom buttons
	   lcd_Font("16B");
	   lcd_ButtonDef( BTN_PREV,
	      BTN_MOM,                      // momentary operation
	      BTN_TLXY, 5, lcd_max_y-35,             // starting x,y for buttons
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
	   lcd_ButtonDef( BTN_CANCEL,
	      BTN_TLXY, lcd_max_x-63, lcd_max_y-35,             // starting x,y for buttons
	      BTN_TEXT, "Exit",
	      BTN_END );

   	break;
   case 9:

      lcd_ShowKeypad(1);

	   // Draw bottom buttons
	   lcd_Font("16B");
	   lcd_ButtonDef( BTN_NEXT,
	      BTN_MOM,                      // momentary operation
	      BTN_TLXY, 5, lcd_max_y-35,             // starting x,y for buttons
		   BTN_TYPE, BUTTON_RELEASE,
	      BTN_TEXT, "Next",
	      BTN_MARGINS, 5, 330,         // set left and right margins
	      BTN_TXTOFFSET, 8, 9,
	      BTN_BMP, BMP_med_button, BMP_med_button_dn,
	      BTN_END );
	   lcd_ButtonDef( BTN_CANCEL,
	      BTN_MOM,                      // momentary operation
	      BTN_TLXY, lcd_max_x-63, lcd_max_y-35,             // starting x,y for buttons
		   BTN_TYPE, BUTTON_RELEASE,
	      BTN_MARGINS, 5, 330,         // set left and right margins
	      BTN_SEP, 63, 43,              // set up for button sep
	      BTN_TEXT, "Exit",
	      BTN_TXTOFFSET, 15, 9,
	      BTN_BMP, BMP_med_button, BMP_med_button_dn,
	      BTN_END );
      break;
   case 10:  // keyboard entry
      lcd_drawKeyboard();
	   // Draw bottom buttons
	   lcd_Font("16B");
	   lcd_ButtonDef( BTN_OK,
	      BTN_MOM,                      // momentary operation
	      BTN_TLXY, 5, lcd_max_y-35,             // starting x,y for buttons
		   BTN_TYPE, BUTTON_RELEASE,
	      BTN_TEXT, "OK",
	      BTN_MARGINS, 5, 330,         // set left and right margins
	      BTN_TXTOFFSET, 14, 9,
	      BTN_BMP, BMP_med_button, BMP_med_button_dn,
	      BTN_END );
	   lcd_ButtonDef( BTN_CANCEL,
	      BTN_TXTOFFSET, 5, 9,
	      BTN_TLXY, lcd_max_x-63, lcd_max_y-35,           // starting x,y for buttons
	      BTN_TEXT, "Cancel",
	      BTN_END );
      break;
   }

}
int lastBkColor;
char lastMsg[4];
void lcd_resetMsgBackColor(void) { lastBkColor = -1; }
void lcd_refreshMsgs(void)
{ 	lastMsg[0]=MSGCOUNT;  // actual msg will always be less than MSGCOUNT
  	lastMsg[1]=MSGCOUNT;
  	lastMsg[2]=MSGCOUNT;
  	lastMsg[3]=MSGCOUNT;
}
void lcd_displayMsgs(char * msgNum)
{  // show traditional lcd messages on the color touch screen
//	static int lastBkColor;
	int newColor;
   int xpos, xoff;
   char myBuf[MSG_LEN+1]; // for date time display
   #GLOBAL_INIT
   {  lastBkColor = -1;
   }

   // set font
   if (param.orientLandscape)
   {  lcd_Font("16x32i");
   	xpos=16;
   } else
   {  lcd_Font("24B");
   	xpos=16;
   }

   // TRANS LCD row 1,2
   // check for showing alarm message in red
   switch (msgNum[0])
   // set inside color
//   {  case MSG_ALARM:  lcd_BFcolorsB(lcd_WHITE, lcd_RED); break;
//      case MSG_CIC:    lcd_BFcolorsB(lcd_BLACK, lcd_GREEN); break;
//      case MSG_DROPEN: lcd_BFcolorsB(lcd_BLACK, lcd_YELLOW); break;
//      default:         lcd_BFcolorsB(lcd_BLACK, lcd_WHITE); break;
   {  case MSG_ALARM:     newColor = lcd_RED; break;
      case MSG_CANT_SEND: newColor = lcd_RED; break;
      case MSG_CIC:       newColor = lcd_GREEN; break;
      case MSG_DROPEN:    newColor = lcd_YELLOW; break;
      default:            newColor = lcd_WHITE; break;
   }

   if (msgNum[0]==MSG_PIN) refresh_PIN_message(msgNum[0], msgNum[1]);
   if (msgNum[1]==MSG_PIN) refresh_PIN_message(msgNum[1], msgNum[0]);
   //lcd_DispText(sys_message[msgNum[0]].msg, 30, 12, MODE_NORMAL);
   //lcd_DispText(sys_message[msgNum[1]].msg, 30, 42, MODE_NORMAL);
   // row 1,2
   if ((msgNum[0] != lastMsg[0]) || (msgNum[1] != lastMsg[1]))
	{  // refresh
	   //if (newColor != lastBkColor)
	   //{  // set the fill color
	      lcd_BFcolorsB(newColor, newColor);
	   //   lastBkColor = newColor;
	   //} else
      //{  lcd_BFcolorsB(lcd_WHITE, newColor);   // set fill color
   	//}
      ///lcd_Rectangle(6, 6, lcd_max_x-6, 71, 1);       // inside paint
      lcd_Rectangle(1, 1, lcd_max_x-1, 71, 1);       // inside paint
	   lcd_BFcolorsB(lcd_BLACK, newColor);  			// reset text color
      if (param.orientLandscape)
      {
	      lcd_DispText(sys_message[msgNum[0]].msg, xpos, 8, MODE_NORMAL);
	      lcd_DispText(sys_message[msgNum[1]].msg, xpos, 38, MODE_NORMAL);
      } else
      {
			if (msgNum[0] == MSG_DATE_TIME)
         {  xpos = 35;
         } else {
	      	xpos = psTextPos(sys_message[msgNum[0]].msg);
         }
	      lcd_DispText(sys_message[msgNum[0]].msg, xpos, 8, MODE_NORMAL);
	      xpos = psTextPos(sys_message[msgNum[1]].msg);
	      lcd_DispText(sys_message[msgNum[1]].msg, xpos, 38, MODE_NORMAL);
      }
      lastMsg[0] = msgNum[0];
      lastMsg[1] = msgNum[1];
   }
   else if (msgNum[0] == MSG_DATE_TIME)
	{	// always rewrite the clock
      lcd_BFcolorsB(lcd_BLACK, newColor);  			// reset text color
      xoff = (param.orientLandscape) ? 0 : 35-xpos;
      lcd_DispText(sys_message[msgNum[0]].msg, xpos+xoff, 8, MODE_NORMAL);
   }

   // SYSTEM LCD row 3,4
   //lcd_DispText(sys_message[msgNum[2]].msg, 30, 90, MODE_NORMAL);
   //lcd_DispText(sys_message[msgNum[3]].msg, 30, 120, MODE_NORMAL);
   if (msgNum[2]==MSG_PIN) refresh_PIN_message(msgNum[2], msgNum[3]);
   if (msgNum[3]==MSG_PIN) refresh_PIN_message(msgNum[3], msgNum[2]);
   // row 3,4
   if ((msgNum[2] != lastMsg[2]) || (msgNum[3] != lastMsg[3]))
	{  // refresh
	   lcd_BFcolorsB(lcd_WHITE, lcd_WHITE);
	   ///lcd_Rectangle(6, 73, lcd_max_x-6, 138, 1);     // inside paint
	   lcd_Rectangle(1, 73, lcd_max_x-1, 138, 1);     // inside paint
	   // check for showing alarm message in red
	   if (msgNum[2] == MSG_ALARM) lcd_BFcolorsB(lcd_WHITE, lcd_RED);
	   else lcd_BFcolorsB(lcd_BLACK, lcd_WHITE); // inside color
      if (param.orientLandscape)
      {
	      lcd_DispText(sys_message[msgNum[2]].msg, xpos, 75, MODE_NORMAL);
	      lcd_DispText(sys_message[msgNum[3]].msg, xpos, 105, MODE_NORMAL);
      } else
		{
     	   xpos = psTextPos(sys_message[msgNum[2]].msg);
	      lcd_DispText(sys_message[msgNum[2]].msg, xpos, 75, MODE_NORMAL);
     	   xpos = psTextPos(sys_message[msgNum[3]].msg);
	      lcd_DispText(sys_message[msgNum[3]].msg, xpos, 105, MODE_NORMAL);
		}
      lastMsg[2] = msgNum[2];
      lastMsg[3] = msgNum[3];
   }


   // display clock and trans counter
   //lcd_Font("10");
   //lcd_BFcolorsB(lcd_BLACK, lcd_LGREEN);
   // steal date/time from sys_message
/* IF THE DATE/TIME AT THE BOTTOM COME BACK, NEED TO SURPRESS DURING A TRANSACTION
   strcpy(myBuf, &sys_message[MSG_DATE_TIME].msg[2]);
   myBuf[8]=0;
   myBuf[14]=0;
	lcd_DispText(myBuf, 158, 201, MODE_NORMAL);
	lcd_DispText(&myBuf[9], 165, 213, MODE_NORMAL);
   ltoa(transactionCount(), myBuf);
	lcd_DispText(myBuf, 190-strlen(myBuf)*10, 225, MODE_NORMAL);
*/

}
int countSpaces(char * msg)
{  // how many leading spaces
   int i;
   i=0;
   while (msg[i]==32) { i++; }
   return i;
}
int psTextPos(char * msg)
{  // based on how many of each character size
   // used only for portrait layout with proportional sized font 24B
   int i;
   int upper, lower, space;
   int ulen, llen, slen;
   int pos;
   ulen=15; llen=11; slen=6;  // for proportional spacing calculation
   // start at zero
   upper=0;
   lower=0;
   space=0;
   i=0;
   while (msg[i]!=0)
   {  if (msg[i]==32) space++;
      else if (msg[i] <= 90) upper++;
      else lower++;
	   i++;
   }
   // position at mid minus half the virtual length
	pos = 120 - ( ((upper*ulen) + (lower*llen) + (space*slen))/2 );
   if (pos < 5) pos=5;

   return pos;
}
void lcd_printMessage(char line, char * msg)
{  // show traditional lcd messages on the color touch screen
   //char pw; // pen width
   //pw=3;
   int y;
   y=0;
   lcd_Font("16x32i");

   switch (line)
   {
   case 0: y=8; break;
   case 1: y=38; break;
   case 2: y=75; break;
   case 3: y=105; break;
   }
   if (y>0)
   {
	   lcd_BFcolorsB(lcd_BLACK, lcd_WHITE); // inside color
   	lcd_DispText(msg, 16, y, MODE_NORMAL);
   }
}
void lcd_processTouchScreen(int button)
{  // buttons from the main display
   // optionally process button passed as parameter
   // if button = -2 then check for a new button press
	//int button;
   if (button==-2) button = lcd_GetTouch(1);
   // reset screen saver if a button is pressed
   //if (button != -1) lcd_screenSaver(LCD_BRIGHT);

   switch (button)
   {
   case BTN_MENU:
	   if (system_state==MALFUNCTION_STATE) lcd_enterSetupMode(0);
      // checking for IDLE is redundant since buttons are removed during a transaction
      else if (system_state==IDLE_STATE) lcd_enterSetupMode(1);
	   lcd_drawScreen(1, lcd_WITH_BUTTONS);		// redraw main screen
   	break;
	case BTN_SEND:
		lcd_sendToSet(SLAVE,0);
   	break;
   case BTN_DIRECTORY:
      lcd_showDirectory(0);
	   lcd_drawScreen(1, lcd_WITH_BUTTONS);		// redraw main screen
      break;
   case BTN_HELP:
      lcd_helpScreen(0); // help->about
      lcd_drawScreen(1, lcd_WITH_BUTTONS);		// redraw main screen
      break;
   case BTN_STAT+256:  // Stat on
   	btnStatFlag=1;
   	break;
   case BTN_STAT:      // Stat off
   	btnStatFlag=2;   // Use =2 to trigger clearing of stat (timer, lcd message, )
      break;
   case BTN_CRETURN:  // aka Return From...
		//lcd_sendToSet(SLAVE,1); // for simple point to point main/slave
      lcd_showDirectory(1);
	   lcd_drawScreen(1, lcd_WITH_BUTTONS);		// redraw main screen
   	break;
   case BTN_ARETURN+256:  // Auto return on
		btnAReturnFlag=1;
   	break;
   case BTN_ARETURN:      // Auto return off
      //lcd_showDirectory(2);
	   //lcd_drawScreen(1, lcd_WITH_BUTTONS);		// redraw main screen
      btnAReturnFlag=2;
   	break;
   case BTN_SECURE+256:  // Secure on
   	btnSecureFlag=1;
   	break;
   case BTN_SECURE:      // Secure off
   	btnSecureFlag=2;
   	break;
   }

}
char lcd_stationButton;  // for holding send-to requests
char lcd_returnCarrierButton; // for holding returnCarrier requests
char lcd_autoReturnButton;   // for holding auto-return requests
void lcd_showDirectory(char mode)
{  // show list of all active stations
   // mode 0 = standard directory
   //      1 = carrier-return directory
   //      2 = auto-return directory
	char i;
   int button;
   int xpos, ypos, dy;
   char buf[4];
   char pin_msg[5];
   buf[0]=0; buf[1]=0; // for station number text

   if (mode==0)      lcd_drawScreen(5, "STATION DIRECTORY");
   else if (mode==1) lcd_drawScreen(5, "RETURN CARRIER FROM");
   else              lcd_drawScreen(5, "SEND + AUTO RETURN");

   lcd_Font("16B");
   xpos=10; // initial x,y button position
   ypos=50;
   dy=39;
   for (i=0; i<LAST_STATION; i++)
   {  if ((1<<i) & STATION_SET)
   	{  buf[0]='1'+i;
	      // show a button with the station number
	      lcd_ButtonDef( i+1, BTN_MOM,       // momentary operation
	         BTN_TLXY, xpos, ypos,           // starting x,y for buttons
	         BTN_MARGINS, 5, 200,          // set left and right margins
	         BTN_TYPE, BUTTON_RELEASE,
	         BTN_TEXT, buf,
	         BTN_TXTOFFSET, 12, 9,
	         BTN_BMP, BMP_button_up, BMP_button_dn,
	         BTN_END );
         if (remote_data[REMOTE_CIC] & (1<<i)) lcd_BFcolorsB(lcd_BLACK, lcd_GREEN);
         else lcd_BFcolorsB(lcd_BLACK, lcd_WHITE);
      	lcd_DispText(param.station_name[i+1].name, xpos+40, ypos, MODE_NORMAL);
         lcd_BFcolorsB(lcd_BLACK, lcd_WHITE);
         lcd_DispText(param.phoneNum[i+1], xpos+40, ypos+18, MODE_NORMAL);
         // show secure pin if one is active
         if (securePIN[i+1]>=0)
         {  sprintf(pin_msg, "%04d", securePIN[i+1]);
	         lcd_BFcolorsB(lcd_RED, lcd_WHITE);
         	lcd_DispText(pin_msg, xpos+110, ypos+18, MODE_NORMAL);
         }

         // TO DO
         if (xpos==10) xpos = (param.orientLandscape) ? 165 : 125;
         else        { xpos=10; ypos+=dy; }
      }
   }
   // What about slave station
   if (slaveAvailable)
   {  // show a button with the station number
   	buf[0]='0'+SLAVE;
      lcd_ButtonDef( SLAVE, BTN_MOM,       // momentary operation
         BTN_TLXY, xpos, ypos,           // starting x,y for buttons
         BTN_TEXT, buf,
         BTN_END );
      if (slave_cic) lcd_BFcolorsB(lcd_BLACK, lcd_GREEN);
      else lcd_BFcolorsB(lcd_BLACK, lcd_WHITE);
      lcd_DispText(param.station_name[i+1].name, xpos+40, ypos, MODE_NORMAL);
      lcd_BFcolorsB(lcd_BLACK, lcd_WHITE);
      lcd_DispText(param.phoneNum[i+1], xpos+40, ypos+18, MODE_NORMAL);
   }
     // What about main station at the slave
   else if (param.slaveController)
   {  // show a button with the station number
   	buf[0]=param.station_name[0].name[0]; // take first character of name //='0';
      lcd_ButtonDef( SLAVE, BTN_MOM,       // momentary operation
         BTN_TLXY, xpos, ypos,           // starting x,y for buttons
         BTN_TEXT, buf,
         BTN_END );
      lcd_BFcolorsB(lcd_BLACK, lcd_WHITE);
      lcd_DispText(param.station_name[0].name, xpos+40, ypos, MODE_NORMAL);
      lcd_BFcolorsB(lcd_BLACK, lcd_WHITE);
      lcd_DispText(param.phoneNum[0], xpos+40, ypos+18, MODE_NORMAL);
   }

   // show legend at bottom for CIC
   lcd_BFcolorsB(lcd_BLACK, lcd_GREEN);
   lcd_DispText("Green", 5, 220, MODE_NORMAL);
   lcd_BFcolorsB(lcd_BLACK, lcd_WHITE);
   lcd_DispText(" = Carrier In Chamber", 0, 0, MODE_NORMAL);
   // Display active (next) secure pin if activated
   //if ((mode != 1) && (securePIN[0] >= 0)) lcd_DispText(&sys_message[MSG_PIN].msg[MSG_OFFSET+3], 5, 205, MODE_NORMAL);
   if ((mode != 1) && (securePIN[0] >= 0))
   {  refresh_PIN_message(MSG_PIN, MSG_AT);  // MSG_AT leads to PIN 0 (next trans)
      lcd_BFcolorsB(lcd_RED, lcd_WHITE);
      lcd_DispText(sys_message[MSG_PIN].msg, 5, 205, MODE_NORMAL);
   }

   // Wait for a button
   menu_timeout = SEC_TIMER + 60;
   while(((button=lcd_GetTouch(100)) == -1) && !secTimeout(menu_timeout) ) maintenance();
   // treat button as a send-to request or return-from request
   if ((button >= 0) && (button <= SLAVE)) lcd_stationButton = button;
   // set the function flags as needed
   lcd_returnCarrierButton = FALSE;
   lcd_autoReturnButton = FALSE;
   if (button != BTN_EXIT)
   {  // set the appropriate flags
	   if (mode==1) lcd_returnCarrierButton=TRUE;  // is this a return carrier request?
	   if (mode==2) lcd_autoReturnButton=TRUE;     // is this an auto return request?
   }

}
char lcd_autoReturn(void)
{  // returns the lcd_autoReturnButton
	char rtnVal;
   #GLOBAL_INIT
   { lcd_autoReturnButton=0; }

   rtnVal = lcd_autoReturnButton;
   lcd_autoReturnButton=0; // reset whenever it is accesses
	return rtnVal;
}
char lcd_returnCarrier(void)
{  // returns the lcd_returnCarrierButton
	char rtnVal;
   #GLOBAL_INIT
   { lcd_returnCarrierButton=0; }

   rtnVal = lcd_returnCarrierButton;
   lcd_returnCarrierButton=0; // reset whenever it is accesses
	return rtnVal;
}
void lcd_sendToSet(char station, char direction)
{  // sets lcd_stationButton
	lcd_stationButton=station;
   if (direction) lcd_returnCarrierButton=TRUE;
}
char lcd_sendTo(void)
{  // returns the lcd_stationButton
	char rtnVal;
   #GLOBAL_INIT
   { lcd_stationButton=0; }

   rtnVal = lcd_stationButton;
   lcd_stationButton=0; // reset whenever it is accesses
	return rtnVal;
}
int lcd_Center(char * string, char font)
{
	int temp;
   temp = param.orientLandscape ? 160 : 120;
   if (font==1)      temp = temp - (int)(strlen(string) * 7);  // "24"
   else if (font==2) temp = temp - (int)(strlen(string) * 4);  // "16"
//   if (temp < 6) temp = 6;
   if (temp < 2) temp = 2;
	return temp;
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
               // Tell remotes to update/save parameters
               if (syncParametersToRemote())
	               lcd_DispText("\nREMOTES NOT SYNC'D", 0, 0, MODE_REV);
	            msDelay(800);
               sendParametersToLogger(); // send parameters to data logger
            }
         	keepLooping = FALSE;
         }
	      //else if (button == BTN_CANCEL) keepLooping = FALSE;

         // show the same or next menu page if needed
         if (showPage && keepLooping) anotherPage = lcd_ShowMenu(operatorLevel, page);
      }

	}

}


void lcd_ShowTransactionProgress(char source, char dest, char direction, int progress, unsigned long transTimer)
{	// call this routine to update the display with transaction progress
	// source, dest are system station numbers 0-8
   // direction is DIR_SEND (left to right) or DIR_RETURN (right to left)
   // progress is % of progress into transaction
   // transStartTime is time into transaction so far in seconds

   // CONSIDER TO SHOW TRANSACTION TIMER

	static char lastSource;
   static char lastDest;
   static char lastDirection;
   static char lastProgress;  // as percentage 0-100
   //static char last_CIC;
   static unsigned long lastTransTimer;
   static int x1, x2, y1, y2;
   int xmid;
   static char pw;  				// pen width

   #GLOBAL_INIT
   {	// initialize local variables
   	lastSource=0;
      lastDest=0;
      lastDirection=0;
      lastProgress=0;
      lastTransTimer=0;
      x1=6; x2=314;
      y1=176; y2=199;
      pw=1;
   }
   if (param.orientLandscape)
   {  x1=6; x2=314;
      y1=176; y2=199;
   } else
   {  x1=6; x2=234;
      y1=256; y2=279;
   }

   // show source station name
   if (source != lastSource)
   {
   	lastSource=source;
   }

   // show destination station name
   if (dest != lastDest)
   {
   	lastDest=dest;
   }

   // show state progress
   if ((progress != lastProgress)) // || (di_carrierInChamber != last_CIC))
   {
	   // draw a box using background
   	// draw a boarder box
      lcd_PenWidth(pw);

      // last_CIC=di_carrierInChamber;  // always remember last
      if (progress <= 100)
      {  // any other progress just blank it out
	      lcd_BFcolorsB( lcd_WHITE, lcd_WHITE);   // border color
         //?//lcd_Rectangle(12, 163, 150, 203, 1);  // blank out CIC
	      lcd_BFcolorsB( lcd_BLUE, lcd_WHITE);   // border color
	      lcd_Rectangle(x1-pw, y1-pw, x2+pw, y2+pw, 0);
	      // fill box with specific progress and remainder
         lcd_Font("18BC");
	      if (direction == DIR_SEND)
	      {  // progress left to right
	         xmid = x1 + (int)((x2-x1) * (long)progress/100);
	         lcd_Rectangle(x1, y1, xmid, y2, 1);    // fill left side progress
	         lcd_BFcolorsB( lcd_WHITE, lcd_WHITE);  // unfill color
	         lcd_Rectangle(xmid, y1, x2, y2, 1);    // fill right side remaining
		      lcd_BFcolorsB( lcd_BLACK, lcd_WHITE);
				lcd_DispText("-->>", xmid, y1+pw, MODE_TRANS); // show direction arrow
	      } else
	      {  // progress right to left
	         xmid = x2 - (int)((x2-x1) * (long)progress/100);
	         lcd_Rectangle(xmid, y1, x2, y2, 1);    // fill right side progress
	         lcd_BFcolorsB( lcd_WHITE, lcd_WHITE);  // unfill color
	         lcd_Rectangle(x1, y1, xmid, y2, 1);    // fill left side remaining
		      lcd_BFcolorsB( lcd_BLACK, lcd_WHITE);
            lcd_DispText("<<--", xmid, y1+pw, MODE_TRANS); // show direction arrow
	      }
         // show the station names; NOTE: source is really main station and dest is remote station
         lcd_DispText(param.station_name[source].name, 10, y1+pw, MODE_TRANS);
         xmid = (lcd_max_x - 15) - 11*strlen(param.station_name[dest].name);
         lcd_DispText(param.station_name[dest].name, xmid, y1+pw, MODE_TRANS);
      }
      else if (progress == 999)
      {  // blank out when progress = 999
	      lcd_BFcolorsB( lcd_WHITE, lcd_WHITE);   // border color
	      lcd_Rectangle(x1-pw, y1-pw, x2+pw, y2+pw, 1);
      } else
      {  // otherwise handle CIC
         lcd_showCIC(0);
      }
      lastProgress=progress;
   } else
   {  if (progress >= 100) lcd_showCIC(0);  // only if no progress bar showing
   }

   // show transaction timer
   if (transTimer != lastTransTimer)
   {
   	lastTransTimer=transTimer;
   }

}
void lcd_showCIC(char forceit)
{  // show CIC on screen
return;

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
   lcd_Font("18BC");
   lcd_DispText(description, 15, 50, MODE_NORMAL);
	lcd_DispBitmap( BMP_input_box, 45, 80 );
   lcd_DispText(par_units, 132, 88, MODE_NORMAL);

   // show actual, min and max
   lcd_Font("16B");
   lcd_DispText("Min = ", 45, 120, MODE_NORMAL);
   lcd_DispText(ltoa((long)par_min, number), 0, 0, MODE_NORMAL);
   lcd_DispText("Current = ", 22, 140, MODE_NORMAL);
   lcd_DispText(ltoa((long)*par_value, number), 0, 0, MODE_NORMAL);
   lcd_DispText("Max = ", 42, 160, MODE_NORMAL);
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
				lcd_DispText(number, 50, 87, MODE_TRANS); // show the number
	      } else if ((button == BTN_DEL) && (numDigits > 0))
         {  numDigits--;
         	number[numDigits]=0;  // trailing null
            // reprint the screen
	         lcd_DispBitmap( BMP_input_box, 45, 80 );
	         lcd_DispText(number, 50, 87, MODE_TRANS); // show the number
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
               {  lcd_DispText("NO CHANGE MADE", 50, 180, MODE_REV);
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
	            { lcd_DispText("OUTSIDE RANGE", 50, 180, MODE_REV);
	              msDelay(750);
	              numDigits=0;
	              number[0]=0; number[1]=0;
	              lcd_DispBitmap( BMP_input_box, 45, 80 );
	              lcd_DispText(number, 50, 87, MODE_TRANS); // show the number
               }
            }
         }
      }
   }
   return rtnval;
}
void lcd_setClock()
{
	int button;
   char keepLooping;
   struct tm time;
   unsigned long timedate;

   timedate=SEC_TIMER;
   mktm(&time, timedate);

   setTimeMessages(MSG_DATE_TIME, timedate);
	lcd_drawScreen(7, "SET DATE & TIME");

   keepLooping=TRUE;
   menu_timeout = SEC_TIMER + 60;
   while ( (keepLooping) && !secTimeout(menu_timeout) )
   {
      maintenance();  // watchdog, led activity, UDP commands
      button = lcd_GetTouch(100);
      if (button != -1)
      {  // valid button pressed
      	menu_timeout = SEC_TIMER + 60;
         switch( button )
         {
         case BTN_SAVE:  // Done
            keepLooping=FALSE;
            break;
			case 0: // +month
            if (time.tm_mon<12) time.tm_mon++;
            break;
         case 1: // -month
         	if (time.tm_mon>1) time.tm_mon--;
            break;
         case 2: // +day
         	if (time.tm_mday<31) time.tm_mday++;
            break;
         case 3: // -day
         	if (time.tm_mday>1) time.tm_mday--;
            break;
			case 4: // +year
         	if (time.tm_year<140) time.tm_year++;
            break;
         case 5: // -year
         	if (time.tm_year>100) time.tm_year--;
            break;
         case 6: // +hour
         	if (time.tm_hour<23) time.tm_hour++;
            break;
         case 7: // -hour
         	if (time.tm_hour>0) time.tm_hour--;
            break;
         case 8: // +minute
         	if (time.tm_min<59) time.tm_min++;
            break;
         case 9: // -minute
         	if (time.tm_min>0) time.tm_min--;
         	break;
         }
	      // update and print date/time message
		   timedate = mktime(&time);
		   setTimeMessages(MSG_DATE_TIME, timedate);
         lcd_showClockMessage();
      }
   }
	// put time back into timer and real-time-clock
   //write_rtc(timedate);
	if (!secTimeout(menu_timeout))
   {  // update only if not exiting due to timeout
	   tm_wr(&time);
	   SEC_TIMER = timedate;
	   menu_timeout = SEC_TIMER + 60;
	}
}
void lcd_showClockMessage()
{  // especially for lcd_setClock
   lcd_Font("14x24"); //"16x32i");
   lcd_DispText(sys_message[MSG_DATE_TIME].msg, 5, 58, MODE_NORMAL);
}
char lcd_defineActiveStations()
{  // define the active stations
   // returns TRUE if activestations were changed
   char rtsIn;
   char done;
   char j, col; //, msgbuf[17];
   int xpos, ypos, dy;
   char original;
   char inputStations;

   int button;

	lcd_drawScreen(5, "SET ACTIVE STATIONS");
	//lcd_DispText("PRESS ACTIVE SEND BUTTONS", 60, 50, MODE_NORMAL);
   lcd_Font("12x24");
   //strcpy(msgbuf, sys_message[MSG_BLANK].msg);  // buffer to show stations
   // Draw all station buttons
   xpos=10; // initial x,y button position
   ypos=50;
   dy=39;
   lcd_Font("16B");
   for (j=1; j<=LAST_STATION; j++)
   {	// button
      lcd_ButtonDef( j,
         BTN_LAT,                     // latched operation
         BTN_TYPE, (station2bit(j) & STATION_SET) ? BUTTON_LAT_1 : BUTTON_LAT_0,
         //BTN_MARGINS, 5, 200,          // set left and right margins
         //BTN_SEP, 35, 35,              // set up for button sep
         BTN_TEXT, "","",
         BTN_TLXY, xpos, ypos,           // starting x,y for buttons
         BTN_BMP, BMP_button_off, BMP_button_on,
         BTN_END );
		// and station name
      lcd_DispText(param.station_name[j].name, xpos+40, ypos+8, MODE_NORMAL);
		// update position
      if (xpos==10) xpos = (param.orientLandscape) ? 165 : 125;
      else        { xpos=10; ypos+=dy; }
   }

   original = STATION_SET;  // save for later
   inputStations = STATION_SET;
   if (param.headDiverter==TRUE) inputStations |= 0x80;

   done=FALSE;
   // Loop until done or timeout
	while ((done==FALSE) && !secTimeout(menu_timeout))
   {
		maintenance();  // watchdog, led activity, UDP commands
	   button = lcd_GetTouch(100);

	   // If BTN_SAVE, save changes
	   if (button == BTN_EXIT)  // Exit button
	   {
         STATION_SET = inputStations;
         done=TRUE;
      }
      // If station button, capture change
      rtsIn = button%256;
      if ((button > 0) && (rtsIn <= LAST_STATION))
	   {
	      if (rtsIn)
	      {  if (button>256) // toggle on
         		inputStations |= station2bit(rtsIn);
            else //toggle off
         		inputStations &= ~station2bit(rtsIn);
	         menu_timeout = SEC_TIMER + 60;  // reset menu timeout
	      }
      }
   }
   if (STATION_SET==0) STATION_SET = original;  // don't allow zero stations
   param.activeStations = STATION_SET;
   if (STATION_SET == original) return FALSE;
   else return TRUE;
}

char lcd_editServerName()
{  // Returns TRUE if name changed
   char j, changed, col, msgbuf[25];
   char keepLooping;
   int button;
   char inKey;
   changed=0;

	lcd_drawScreen(10, "EDIT SERVER NAME");
   lcd_Font("12x24");

   strncpy(msgbuf, "_", sizeof(param.serverID));  // fill with "_"
   lcd_DispText(msgbuf, 20, 50, MODE_NORMAL); // show the name
   lcd_DispText(param.serverID, 20, 50, MODE_NORMAL);  // show existing name
   lcd_Font("12x24");
   col=0;
   keepLooping=TRUE;
   while ( (keepLooping) && !secTimeout(menu_timeout) )
   {
      maintenance();  // watchdog, led activity, UDP commands
      button = lcd_GetTouch(100);
      if (button != -1)
      {  // valid button pressed
         menu_timeout = SEC_TIMER + 60;
         switch( button )
         {
         case BTN_OK:  // next message
            keepLooping=FALSE;
            break;
         case BTN_CANCEL:  // finish
         	changed=FALSE;
            keepLooping=FALSE;
            break;
         case 8:  // backspace
            if (col>0)
            {  if (col<11) msgbuf[col]='_';
               col--;
               msgbuf[col]='_';
               lcd_DispText(msgbuf, 20, 50, MODE_NORMAL); // show the name
            }
            break;
         default:  // ascii key
            if (col<sizeof(param.serverID))
            {  msgbuf[col]=button;
               col++;
               //msgbuf[col]='_';
               lcd_DispText(msgbuf, 20, 50, MODE_NORMAL); // show the name
               changed=TRUE;
            }
            break;
         }
      }
   }
	if (changed)
   {	// store back into parameter space
	   // replace _ with space
	   // trim spaces
	   j=sizeof(param.serverID);
	   while (j>0)
	   {  // replace _ with null
	      if (msgbuf[j]=='_') msgbuf[j]=0;
	      // trim spaces
	      if (msgbuf[j]>32)  // first non-space
	      {  msgbuf[j+1]=0;
	         j=0;
	      } else --j;
	   }

	   // put name back into main storage
	   if (msgbuf[0]!='_') strcpy(param.serverID, msgbuf);

// check server connection??

   }

   return changed;
}



nodebug char lcd_editStationNames()
{  // Returns TRUE if names changed
   char pos, j, changed, row, col, msgbuf[17];
   char keepLooping;
   int button;
   char inKey;
   struct iomessage message;  // in order to transmit to slave
   row=0;
   changed=0;

	lcd_drawScreen(6, "EDIT STATION NAMES");
   lcd_Font("12x24");

   while ((row <= SYSTEM_NAME) && !secTimeout(menu_timeout))  // row < number of names (0..SYSTEM_NAME)
   {
      // display this station name
      //strcpy(msgbuf, param.station_name[row].name);
      //strncat(msgbuf, "                ", 16-strlen(msgbuf));
      strcpy(msgbuf, "___________");  // fill with underscores
      //msgbuf[15]=0x30+row;  // display number
		lcd_DispText(msgbuf, 20, 50, MODE_NORMAL); // show the name
      lcd_DispText(param.station_name[row].name, 20, 50, MODE_NORMAL);  // show existing name
	   lcd_Font("16B");
      lcd_DispText("(", 180, 55, MODE_NORMAL); // show the default name
		lcd_DispText(defaultStaName[row], 0, 0, MODE_NORMAL); // show the default name
		lcd_DispText(")           ", 0, 0, MODE_NORMAL); // blank out leftovers
	   lcd_Font("12x24");
      col=0;
      keepLooping=TRUE;
	   while ( (keepLooping) && !secTimeout(menu_timeout) )
	   {
	      maintenance();  // watchdog, led activity, UDP commands
	      button = lcd_GetTouch(100);
	      if (button != -1)
	      {  // valid button pressed
	         menu_timeout = SEC_TIMER + 60;
	         switch( button )
	         {
            case BTN_NEXT:  // next message
               // row++; loop will increment row
               keepLooping=FALSE;
               break;
            case BTN_CANCEL:  // finish
               keepLooping=FALSE;
               // row=99;
               break;
            case 8:  // backspace
               if (col>0)
               {  if (col<11) msgbuf[col]='_';
               	col--;
               	msgbuf[col]='_';
                  lcd_DispText(msgbuf, 20, 50, MODE_NORMAL); // show the name
               }
               break;
            default:  // ascii key
               if (col<11)
               {  msgbuf[col]=button;
                  col++;
                  //msgbuf[col]='_';
               	lcd_DispText(msgbuf, 20, 50, MODE_NORMAL); // show the name
		            changed=TRUE;
               }
               break;
            }
         }
      }
      // replace _ with space

      // trim spaces
      j=10;
      while (j>0)
      {  // replace _ with null
         if (msgbuf[j]=='_') msgbuf[j]=0;
      	// trim spaces
      	if (msgbuf[j]>32)  // first non-space
         { 	msgbuf[j+1]=0;
         	j=0;
         } else --j;
      }

      // put name back into main storage
      if (msgbuf[0]!='_') strcpy(param.station_name[row].name, msgbuf);

      // send name to remote station controller
      message.devAddr=ALL_DEVICES;
      message.command=SET_STATION_NAME;
      message.station=row;
      for (j=0; j<3; j++)
      {
	      pos=j*4;
	      message.data[0] = pos;
	      message.data[1] = param.station_name[row].name[pos];
	      message.data[2] = param.station_name[row].name[pos+1];
	      message.data[3] = param.station_name[row].name[pos+2];
	      message.data[4] = param.station_name[row].name[pos+3];

			// send this chunk
	      msDelay(10); send_command(message);
      }
      if (button==BTN_CANCEL) row=99;
      else row++;  // next item
   }

   if (changed)
   {  // tell slave to finalize update
      message.devAddr=ALL_DEVICES;
      message.command=SET_STATION_NAME;
      message.station=69;  // save to eeprom
      msDelay(10); send_command(message);
   }

   // rebuild station name messages
   buildStationNames();

   return changed;
}
nodebug char lcd_editPhoneNums()
{  // Returns TRUE if phone numbers changed
   char pos, j, changed, row, col, msgbuf[10];
   char keepLooping;
   char label[20];
   int button;
   struct iomessage message;  // in order to transmit to slave
   row=0;
   changed=0;

	lcd_drawScreen(9, "SET PHONE NUMBERS");
   lcd_Font("12x24");

   while ((row <= 10) && !secTimeout(menu_timeout))  // row < number of names (0..SYSTEM_NAME)
   {
      // display this station phone number and details
      strcpy(msgbuf, "        ");  // fill with spaces
	   lcd_Font("18BC");
      lcd_BFcolorsB(lcd_BLACK, lcd_WHITE);
      sprintf(label, "Phone # %d ", row);
      lcd_DispText(label, 50, 50, MODE_NORMAL);
	   lcd_DispBitmap( BMP_input_box, 45, 80 );
	   lcd_Font("8x16");
      // white out previous station name and number
      lcd_BFcolorsB(lcd_WHITE, lcd_WHITE);
      lcd_Rectangle(50, 115, 130, 155, 1);
      // show editable number, station name and existing number
      lcd_BFcolorsB(lcd_BLACK, lcd_LGREY);
		lcd_DispText(msgbuf, 54, 87, MODE_NORMAL);
      lcd_DispText("\n\n", 0, 0, MODE_NORMAL);
      lcd_BFcolorsB(lcd_BLACK, lcd_WHITE);
      // show station name or admin or maint
		if (row<=8) lcd_DispText(param.station_name[row].name, 0, 0, MODE_NORMAL); // show the station name
      else if (row == ADMIN_PHONE) lcd_DispText("Administrator", 0, 0, MODE_NORMAL);
      else lcd_DispText("Maintenance  ", 0, 0, MODE_NORMAL);
      lcd_DispText("\n", 0, 0, MODE_NORMAL);
      lcd_DispText(param.phoneNum[row], 0, 0, MODE_NORMAL);
      lcd_BFcolorsB(lcd_BLACK, lcd_LGREY);
	   //lcd_Font("12x24");
      col=0;
      keepLooping=TRUE;
	   while ( (keepLooping) && !secTimeout(menu_timeout) )
	   {
	      maintenance();  // watchdog, led activity, UDP commands
	      button = lcd_GetTouch(100);
	      if (button != -1)
	      {  // valid button pressed
	         menu_timeout = SEC_TIMER + 60;
	         switch( button )
	         {
            case BTN_NEXT:  // next message
               keepLooping=FALSE;
               break;
            case BTN_CANCEL:  // finish
               keepLooping=FALSE;
               break;
            case BTN_DEL:  // backspace
               if (col>0)
               {  if (col<8) msgbuf[col]=' ';
               	col--;
               	msgbuf[col]=' ';
                  lcd_DispText(msgbuf, 54, 87, MODE_NORMAL); // show the name
               }
               break;
            default:  // ascii key
					//if ((button >= 0) && (button <= 9)) button += '0';
               if (col<8)
               {  msgbuf[col]=button;
                  col++;
               	lcd_DispText(msgbuf, 54, 87, MODE_NORMAL); // show the name
		            changed=TRUE;
               }
               break;
            }
         }
      }
      // trim spaces
      j=8;
      while (j>0)
      {  // replace _ with null
         if (msgbuf[j]=='_') msgbuf[j]=0;  // doesn't apply anymore but leave incase it comes back
      	// trim spaces
      	if (msgbuf[j]>32)  // first non-space
         { 	msgbuf[j+1]=0;
         	j=0;
         } else --j;
      }

      // put name back into main storage
      if (msgbuf[0]!='_') strcpy(param.phoneNum[row], msgbuf);

      // send name to remote station controller
      message.devAddr=ALL_DEVICES;
      message.command=SET_PHONE_NUMS;
      message.station=row;
      for (j=0; j<2; j++)
      {
	      pos=j*4;
	      message.data[0] = pos;
	      message.data[1] = param.phoneNum[row][pos];
	      message.data[2] = param.phoneNum[row][pos+1];
	      message.data[3] = param.phoneNum[row][pos+2];
	      message.data[4] = param.phoneNum[row][pos+3];

			// send this chunk
	      msDelay(10); send_command(message);
      }

      if (button==BTN_CANCEL) row=99;  // all done
      else row++;  // next item
   }

   if (changed)
   {  // tell slave to finalize update
      message.devAddr=ALL_DEVICES;
      message.command=SET_PHONE_NUMS;
      message.station=69;  // save to eeprom
      msDelay(10); send_command(message);
   }

   return changed;
}
char lcd_editPortMapping(char *title)
{  // Returns TRUE if port mapping changed
   char pos, j, changed, row, col, msgbuf[10];
   char keepLooping;
   char inputLength;
   char label[25]; // shows above edit box
   int button;
   struct iomessage message;  // in order to transmit to slave
   row=1;
   changed=0;
   inputLength=2;  // 0 to 99 max

	lcd_drawScreen(4, title);
   lcd_Font("12x24");

   while ((row <= 7) && !secTimeout(menu_timeout))  // row < number of names (0..SYSTEM_NAME)
   {
      // display this device mapping details
      strcpy(msgbuf, "        ");  // fill with spaces
	   lcd_Font("18BC");
      lcd_BFcolorsB(lcd_BLACK, lcd_WHITE);
      sprintf(label, "Device Addr %d Config", row);
      lcd_DispText(label, 25, 50, MODE_NORMAL);
	   lcd_DispBitmap( BMP_input_box, 45, 80 );
	   lcd_Font("8x16");
      lcd_DispText("You Must Reboot\nAfter Saving Changes!", 20, 160, MODE_NORMAL);

      // white out previous details
      lcd_BFcolorsB(lcd_WHITE, lcd_WHITE);
      lcd_Rectangle(50, 115, 130, 155, 1);
      // show editable device configuration number
      lcd_BFcolorsB(lcd_BLACK, lcd_LGREY);
		lcd_DispText(msgbuf, 54, 87, MODE_NORMAL);
      lcd_DispText("\n\n", 0, 0, MODE_NORMAL);
      // show current config #
      lcd_BFcolorsB(lcd_BLACK, lcd_WHITE);
		//lcd_DispText(param.station_name[row].name, 0, 0, MODE_NORMAL); // show the station name
      //lcd_DispText("\n", 0, 0, MODE_NORMAL);
      sprintf(label, "Config # %d ", param.portMapping[row]);
      lcd_DispText(label, 0, 0, MODE_NORMAL);
      lcd_BFcolorsB(lcd_BLACK, lcd_LGREY);

      col=0;
      keepLooping=TRUE;
	   while ( (keepLooping) && !secTimeout(menu_timeout) )  // go through each device address
	   {
	      maintenance();  // watchdog, led activity, UDP commands
	      button = lcd_GetTouch(100);
	      if (button != -1)
	      {  // valid button pressed
	         menu_timeout = SEC_TIMER + 60;
	         switch( button )
	         {
            case BTN_OK:  // next message
               keepLooping=FALSE;
               break;
            case BTN_CANCEL:  // finish
               keepLooping=FALSE;
               break;
            case BTN_DEL:  // backspace
               if (col>0)
               {  if (col <= inputLength) msgbuf[col]=' ';
               	col--;
               	msgbuf[col]=' ';
                  lcd_DispText(msgbuf, 54, 87, MODE_NORMAL); // show the value
               }
               break;
            default:  // ascii key
					if ((col < inputLength) && (button >= '0') && (button <= '9'))
               {  // add digit to input buffer
                  msgbuf[col]=button;
                  col++;
                  lcd_DispText(msgbuf, 54, 87, MODE_NORMAL); // show the value
                  changed=TRUE;
               }
               break;
            }
         }
      }

      // put value back into main storage
      if ((button != BTN_CANCEL) && (msgbuf[0] != ' '))
      	param.portMapping[row] = (char) atoi(msgbuf);

      if (button==BTN_CANCEL) row=99;  // all done
      else row++;  // next item
   }

   return changed;
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
   case 2: 	strcpy(reqdPW, param.adminPassword); break;
   case 3: 	strcpy(reqdPW, param.maintPassword); break;
   case 4: 	strcpy(reqdPW, param.cfgPassword);
   			strcpy(altPW, "2321"); break;  // 2321 is always available
   default: reqdPW[0]=0;
   }

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
   char j, col, msgbuf[17];
   char inKey;
   int button;

	lcd_drawScreen(5, "SHOW DIGITAL INPUTS");

   strcpy(msgbuf, sys_message[MSG_BLANK].msg);  // buffer to show stations

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
	   lcd_DispText("\nSTAT: ", 0, 0, MODE_NORMAL);
      lcd_DispText(di_priorityRequest ? "ON  " : "OFF", 0, 0, MODE_NORMAL);
	   lcd_DispText("\nReturn Carrier: ", 0, 0, MODE_NORMAL);
      lcd_DispText(di_returnCarrier ? "ON  " : "OFF", 0, 0, MODE_NORMAL);
	   lcd_DispText("\nSend Btns: ", 0, 0, MODE_NORMAL);
	   sprintf(msgbuf, "%X  ", di_requestToSend);
      lcd_DispText(msgbuf, 0, 0, MODE_NORMAL);
      lcd_DispText("\nDiverter Pos: ", 0, 0, MODE_NORMAL);
	   sprintf(msgbuf, "%X  ", di_diverterPos);
      lcd_DispText(msgbuf, 0, 0, MODE_NORMAL);

   }

}

void lcd_clearMiddle(void)
{  // clears the space between the menu header and the buttons
	lcd_BFcolorsB( lcd_WHITE, lcd_WHITE);
	lcd_Rectangle(1, 41, 320, 204, 1);    // fill left side progress
	lcd_BFcolorsB( lcd_BLACK, lcd_WHITE);
}
char * ltrim(char * text)
{
	char i;
   i=0;
   while (text[i]==32) i++;
   return &text[i];
}
nodebug void lcd_drawKeyboard()
{
   lcd_BFcolorsB( lcd_BLACK, lcd_WHITE);
   lcd_Font("24");
	// draw alpha keyboard
	// the button ID for each button is its equivalent ASCII decimal value
	// first row
   lcd_ButtonDef( 49,
		BTN_MOM,
    	BTN_TLXY, 0, 75,
      BTN_SEP, 32, 32,
      BTN_MARGINS, 0, lcd_max_x-32,
      BTN_TYPE, BUTTON_PRESS,
      BTN_TEXT, "1",
      BTN_TXTOFFSET, 8, 5,
      BTN_BMP, BMP_button_up, BMP_button_dn,
      BTN_END );
   lcd_ButtonDef( 50, BTN_TEXT, "2", BTN_END );
   lcd_ButtonDef( 51, BTN_TEXT, "3", BTN_END );
   lcd_ButtonDef( 52, BTN_TEXT, "4", BTN_END );
   lcd_ButtonDef( 53, BTN_TEXT, "5", BTN_END );
   lcd_ButtonDef( 54, BTN_TEXT, "6", BTN_END );
   lcd_ButtonDef( 55, BTN_TEXT, "7", BTN_END );
   lcd_ButtonDef( 56, BTN_TEXT, "8", BTN_END );
   lcd_ButtonDef( 57, BTN_TEXT, "9", BTN_END );
   lcd_ButtonDef( 48, BTN_TEXT, "0", BTN_END );
// second row
   lcd_ButtonDef( 81, BTN_TEXT, "Q", BTN_END );
   lcd_ButtonDef( 87, BTN_TEXT, "W", BTN_END );
   lcd_ButtonDef( 69, BTN_TEXT, "E", BTN_END );
   lcd_ButtonDef( 82, BTN_TEXT, "R", BTN_END );
   lcd_ButtonDef( 84, BTN_TEXT, "T", BTN_END );
   lcd_ButtonDef( 89, BTN_TEXT, "Y", BTN_END );
   lcd_ButtonDef( 85, BTN_TEXT, "U", BTN_END );
   lcd_ButtonDef( 73, BTN_TEXT, "I", BTN_END );
   lcd_ButtonDef( 79, BTN_TEXT, "O", BTN_END );
   lcd_ButtonDef( 80, BTN_TEXT, "P", BTN_END );
// third row
   lcd_ButtonDef( 65, BTN_TEXT, "A", BTN_END );
   lcd_ButtonDef( 83, BTN_TEXT, "S", BTN_END );
   lcd_ButtonDef( 68, BTN_TEXT, "D", BTN_END );
   lcd_ButtonDef( 70, BTN_TEXT, "F", BTN_END );
   lcd_ButtonDef( 71, BTN_TEXT, "G", BTN_END );
   lcd_ButtonDef( 72, BTN_TEXT, "H", BTN_END );
   lcd_ButtonDef( 74, BTN_TEXT, "J", BTN_END );
   lcd_ButtonDef( 75, BTN_TEXT, "K", BTN_END );
   lcd_ButtonDef( 76, BTN_TEXT, "L", BTN_END );
   lcd_ButtonDef( ':', BTN_TEXT, ":", BTN_END );
// fourth row
   lcd_ButtonDef( 90, BTN_TEXT, "Z", BTN_END );
   lcd_ButtonDef( 88, BTN_TEXT, "X", BTN_END );
   lcd_ButtonDef( 67, BTN_TEXT, "C", BTN_END );
   lcd_ButtonDef( 86, BTN_TEXT, "V", BTN_END );
   lcd_ButtonDef( 66, BTN_TEXT, "B", BTN_END );
   lcd_ButtonDef( 78, BTN_TEXT, "N", BTN_END );
   lcd_ButtonDef( 77, BTN_TEXT, "M", BTN_END );
   lcd_ButtonDef( 46, BTN_TEXT, ".", BTN_END );
   lcd_ButtonDef( 45, BTN_TEXT, "-", BTN_END );
   lcd_Font("16B");
   lcd_ButtonDef(  8, BTN_TEXT, "del", BTN_TXTOFFSET, 6, 10, BTN_END );
   lcd_ButtonDef( 32, BTN_TEXT, "Space", BTN_TLXY, param.orientLandscape ? 114 : 74, lcd_max_y - 37,
      BTN_TXTOFFSET, 25, 10,
		BTN_BMP, BMP_92x32_button, BMP_92x32_button_dn, BTN_END );
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
	   lcd_drawScreen(5, "TRANSACTION SUMMARY");
	   lcd_ButtonDef( BTN_CANCEL,
		      BTN_TXTOFFSET, 9, 9,
	         BTN_TLXY, 5, lcd_max_y-35,             // starting x,y for buttons
	         BTN_TEXT, "Reset",
	         BTN_END );

	   lcd_Font("8x12");
	   lcd_DispText(param.station_name[SYSTEM_NAME].name, 10, 50, MODE_NORMAL);
	   lcd_DispText("\n Transactions:", 0, 0, MODE_NORMAL);
	   sprintf(myline, "\n  Incoming:              %ld", statistics.trans_in);
	   lcd_DispText(myline, 0, 0, MODE_NORMAL);
	   sprintf(myline, "\n  Outgoing:              %ld", statistics.trans_out);
	   lcd_DispText(myline, 0, 0, MODE_NORMAL);
	   sprintf(myline, "\n  Total Transactions:    %ld", statistics.trans_in + statistics.trans_out);
	   lcd_DispText(myline, 0, 0, MODE_NORMAL);
	   //sprintf(myline, "\n    Grand Total:           %ld", transactionCount());
	   //lcd_DispText(myline, 0, 0, MODE_NORMAL);
	   lcd_DispText("\n Alarms:", 0, 0, MODE_NORMAL);
	   sprintf(myline, "\n  Incomplete Delivery:   %d", statistics.deliv_alarm);
	   lcd_DispText(myline, 0, 0, MODE_NORMAL);
	   sprintf(myline, "\n  Diverter Timeout:      %d", statistics.divert_alarm);
	   lcd_DispText(myline, 0, 0, MODE_NORMAL);
	   sprintf(myline, "\n  Carrier Lift Timeout:  %d", statistics.cic_lift_alarm);
	   lcd_DispText(myline, 0, 0, MODE_NORMAL);
	   subtot=statistics.deliv_alarm+statistics.divert_alarm+statistics.cic_lift_alarm;
	   sprintf(myline, "\n  Total Alarms:          %ld", subtot);
	   lcd_DispText(myline, 0, 0, MODE_NORMAL);

	   // wait for any button
	   menu_timeout = SEC_TIMER + 60;
	   while(((button=lcd_GetTouch(100)) == -1) && !secTimeout(menu_timeout) ) maintenance();
      if (button==BTN_CANCEL)
      {  // Ask: Are You Sure?
      	lcd_Font("16B");
         lcd_BFcolorsB( lcd_LGREY, lcd_LGREY);  // unfill color
         lcd_Rectangle(30, 100, 230, 160, 1);    // fill white
         lcd_BFcolorsB( lcd_BLACK, lcd_WHITE);
         lcd_Rectangle(30, 100, 230, 160, 0);    // outline
         lcd_DispText("Press Reset to confirm\nor Exit to cancel", 60, 115, MODE_TRANS);

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
#define XMemLogMax 1000
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
   if (OPMODE == 0x80) sf = SF1000Init(); else sf = -3;
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
void checkSerialFlash()
{  // Show serial flash data and allow deep checking of transactions
   long iLong;
	int button;
   int sf;
   int attempts;
   char myline[80];     // line to send to the lcd

	lcd_drawScreen(5, "SERIAL FLASH INFO");
   // add button to reset head/tail
   lcd_ButtonDef( BTN_CANCEL,
      BTN_TXTOFFSET, 9, 9,
      BTN_TLXY, 5, lcd_max_y-35,             // starting x,y for buttons
      BTN_TEXT, "Reset",
      BTN_END );


   lcd_Font("8x12");
   lcd_DispText("Checking Serial Flash", 10, 50, MODE_NORMAL);
   sprintf(myline, "\nOPMODE = %x", OPMODE);
	lcd_DispText(myline, 0, 0, MODE_NORMAL);
   attempts=0;
   if (OPMODE == 0x80)
   {  do
   	{  sf = SF1000Init();
			attempts++;
      } while (sf && attempts < 200);
   }
   else sf = -3;

	switch (sf)
   {
   case 0:
   	lcd_DispText("\nSerial Flash - Initialize OK", 0, 0, MODE_NORMAL);
	   sprintf(myline, "\nDevice Type:        %d", SF1000_Density_Value);
	   lcd_DispText(myline, 0, 0, MODE_NORMAL);
      if (SF1000_Density_Value != DENS_8MB) lcd_DispText(" UNSUPPORTED SIZE", 0, 0, MODE_NORMAL);
	   sprintf(myline, "\nBlock Size:         %d", SF1000_Block_size);
	   lcd_DispText(myline, 0, 0, MODE_NORMAL);
	   sprintf(myline, "\nNumber of Blocks:   %d", SF1000_Nbr_of_Blocks);
	   lcd_DispText(myline, 0, 0, MODE_NORMAL);
	   sprintf(myline, "\nTotal Bytes:        %ld", SF1000_Unit_Bytes);
	   lcd_DispText(myline, 0, 0, MODE_NORMAL);
		iLong = SF1000CheckWrites (0);
		sprintf (myline, "\nBlock 0 has been written %ld times", iLong );
	   lcd_DispText(myline, 0, 0, MODE_NORMAL);
   	break;
   case -1:
   	lcd_DispText("\nSerial Flash - Unknown Density", 0, 0, MODE_NORMAL);
   	break;
   case -2:
   	lcd_DispText("\nSerial Flash - Unknown Response", 0, 0, MODE_NORMAL);
   	break;
   case -3:
   	lcd_DispText("\nSerial Flash - Device Not Found", 0, 0, MODE_NORMAL);
   	break;
	}

   sprintf(myline, "\nEvent Log Start     %ld", logInfo.Tail);
	lcd_DispText(myline, 0, 0, MODE_NORMAL);
   sprintf(myline, "\nEvent Log End       %ld", logInfo.Head);
	lcd_DispText(myline, 0, 0, MODE_NORMAL);

   // wait for any button
   menu_timeout = SEC_TIMER + 60;
   while(((button=lcd_GetTouch(100)) == -1) && !secTimeout(menu_timeout) ) maintenance();
   if (button==BTN_CANCEL)
   {  // Reset the head and tail pointers
   	logInfo.Tail = 0;
      logInfo.Head = 0;

	   // save head/tail back to xmem
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
         BTN_TLXY, 5, lcd_max_y-35,             // starting x,y for buttons
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
void addTransaction(struct trans_log_type trans, char xtra)
{
	long memOffset;
   int sf;
   // Push the transaction into local xmem
   memOffset = logInfo.Head * sizeof(trans);

   if (SFAvailable)
   {  // write to serial flash
	  	while ( (sf=SF1000Write ( memOffset, &trans, sizeof(trans) )) == -3 );
   }
   else
   {  // write to xmem
   	root2xmem( logXaddr+memOffset, &trans, sizeof(trans));
      sf=0;
   }
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
//printf("\nAdded transaction at %ld with status %d and flags %d\n", SEC_TIMER, (int)trans.status, (int)trans.flags);

   // now send out the UDP message
   sendUDPtransaction( trans, xtra );

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
   if (SFAvailable) SF1000Read ( memOffset, trans, sizeof(*trans));
   else xmem2root( trans, logXaddr+memOffset, sizeof(*trans));
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
//#define RS232_MONITOR_F
void uploadTransLog(long start)
{  // open UDP port and transmit the log
   long entry;
   char j;
   struct trans_log_type log;
   struct tm time;
   unsigned long startTime;

   // give some status
 	//lcd_drawScreen(3, "UPLOAD LOG");
   //lcd_Font("24");
   //lcd_DispText("In progress ...", 80, 80, MODE_NORMAL); // show the default name

	for (entry=start; entry<sizeOfTransLog(); entry++)
   {
   	// get entry
		if (getTransaction(entry, &log) == 0)
		{
         // send the transaction to the server
         sendUDPtransaction( log, 0 );

         // wait till the socket is empty to pace the transmission
			startTime = MS_TIMER;
         while ((sock_tbused(&HostSock) > 0) && ((MS_TIMER-startTime) < 500)) maintenance();
      }
      maintenance();
   }

}
void downloadTransLog(int numDays)
{  // open com port and transmit the log
	char buf[120];
   char crlf[3];
   long entry;
   char j;
   struct trans_log_type log;
   struct tm time;
   unsigned long startTime;

   crlf[0]=13;
   crlf[1]=10;
   crlf[2]=0;

   // give some status
 	lcd_drawScreen(3, "DOWNLOAD LOG");
   lcd_Font("24");
   lcd_DispText("In progress ...", 80, 80, MODE_NORMAL); // show the default name

   // open com port
   serFdatabits(PARAM_8BIT);
   serFparity(PARAM_NOPARITY);
   serFopen(19200);

   // write header
   mktm(&time, SEC_TIMER);
   sprintf(buf,"%s Transaction Log Dump at %02d/%02d/%02d %02d:%02d:%02d%s", param.station_name[SYSTEM_NAME].name,
      		time.tm_mon, time.tm_mday, time.tm_year%100, time.tm_hour, time.tm_min, time.tm_sec, crlf);
   serFputs(buf);
   sprintf(buf, "Date,Time,Source,Destination,Duration,Status,Flags%s", crlf);
   serFputs(buf);

	for (entry=0; entry<sizeOfTransLog(); entry++)
   {
   	// get entry
		if (getTransaction(entry, &log) == 0)
      {  mktm(&time, log.start_tm);
         // within date range?
         if (((SEC_TIMER - log.start_tm)/86400) <= numDays)
         {  // within range so download this record
	         if (log.status <= LAST_TRANS_EVENT)
	         {
	            // format entry
	            sprintf(buf, "%02d/%02d/%02d,%02d:%02d:%02d,%s,%s,%d,%d,%d%s",
	               time.tm_mon, time.tm_mday, time.tm_year%100, time.tm_hour, time.tm_min, time.tm_sec,
	               log.source_sta < SYSTEM_NAME ? param.station_name[log.source_sta].name : "??",
	               log.dest_sta < SYSTEM_NAME ? param.station_name[log.dest_sta].name : "??",
	               //(int)(log.duration - log.start_tm),
	               log.duration,
	               (int)log.status, (int)log.flags, crlf );
	         } else if (log.status == ESTS_DOOROPEN)
	         {  // This is a door open event
	            sprintf(buf, "%02d/%02d/%02d,%02d:%02d:%02d,%s,,%d,%d,%d%s",
	            time.tm_mon, time.tm_mday, time.tm_year%100, time.tm_hour, time.tm_min, time.tm_sec,
	            "DOOR OPEN IN TRANS",
	            log.duration,
	            (int) log.status, (int) log.flags, crlf );
	         } else if (log.status == ESTS_MANPURGE)
	         {  // This is a manual purge event
	            sprintf(buf, "%02d/%02d/%02d,%02d:%02d:%02d,%s,%d,%d,%d,%d%s",
	            time.tm_mon, time.tm_mday, time.tm_year%100, time.tm_hour, time.tm_min, time.tm_sec,
	            "MANUAL PURGE",
               log.dest_sta,
	            log.duration,
	            (int) log.status, (int) log.flags, crlf );
	         } else if (log.status == ESTS_AUTOPURGE)
	         {  // This is an automatic purge event
	            sprintf(buf, "%02d/%02d/%02d,%02d:%02d:%02d,%s,,%d,%d,%d%s",
	            time.tm_mon, time.tm_mday, time.tm_year%100, time.tm_hour, time.tm_min, time.tm_sec,
	            "AUTOMATIC PURGE",
	            log.duration,
	            (int) log.status, (int) log.flags, crlf );
	         } else if (log.status == ESTS_SECURE_REMOVAL)
	         {  // This is a secure transaction removal event
	            sprintf(buf, "%02d/%02d/%02d,%02d:%02d:%02d,%s,%s,%ld,%d,%d%s",
	            time.tm_mon, time.tm_mday, time.tm_year%100, time.tm_hour, time.tm_min, time.tm_sec,
	            log.flags < SYSTEM_NAME ? param.station_name[log.flags].name : "??",
	            "SECURE ID",
	            (* (unsigned long*) &log.duration) - CARDBASE,
	            (int) log.status, (int) log.flags, crlf );
	         } else
	         {  // otherwise unknown type
	            sprintf(buf, "%02d/%02d/%02d,%02d:%02d:%02d,%s,,%d,%d,%d%s",
	            time.tm_mon, time.tm_mday, time.tm_year%100, time.tm_hour, time.tm_min, time.tm_sec,
	            "UNKNOWN STATUS",
	            log.duration,
	            (int) log.status, (int) log.flags, crlf );
	        }

	         // make sure there is space in the buffer
	         startTime = MS_TIMER;
	         while ((serFwrFree() < FOUTBUFSIZE) && ((MS_TIMER-startTime) < 500)) maintenance();

	         // write to com port
	         serFputs(buf);
         }
      }
      maintenance();
   }
	startTime = MS_TIMER;
   // wait for buffer to empty
   while ((serFwrUsed() != 0) && ((MS_TIMER-startTime) < 1000)) maintenance();

   // close com port
   msDelay(20); // wait till last character is sent
	serFclose();
}
#define TRANS_PER_PAGE 10
void lcd_showTransactions(char format)
{  // shows the transaction log on the screen
   // format = 1 = only show alarms
   long page;
   long lastPage;
   int button;
   int transPerPage;

   // count how many pages we can show
   page=0;
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
   sprintf(buf, "DATE     TIME     FROM          TO          Sec StFl");
   lcd_DispText(buf, x, y, MODE_NORMAL);
   y+=dy;

   while ( (j < TRANS_PER_PAGE) && (entry >= 0) ) //< sizeOfTransLog()))
   {
		if (getTransaction(entry, &trans) == 0)
      {  mktm(&time, trans.start_tm);
      	lcd_BFcolorsB( lcd_BLACK, lcd_WHITE);
      	if (trans.status <= LAST_TRANS_EVENT)
         {  // This is a transaction
	         sprintf(buf, "%02d/%02d/%02d %02d:%02d:%02d %-11s-> %-11s %3d %2x%2x",
	            time.tm_mon, time.tm_mday, time.tm_year%100, time.tm_hour, time.tm_min, time.tm_sec,
	            trans.source_sta < SYSTEM_NAME ? param.station_name[trans.source_sta].name : "??",
	            trans.dest_sta < SYSTEM_NAME ? param.station_name[trans.dest_sta].name : "??",
	            trans.duration,
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
	   lcd_Rectangle(1, y, 320, lcd_max_y-36, 1);    // fill left side progress
	   lcd_BFcolorsB( lcd_BLACK, lcd_WHITE);
	}
   lcd_showPage(page+1, lastPage+1);
}
void lcd_showPage(long thisPage, long lastPage)
{  // show page x of y
   char buf[25];
   int xpos, ypos;
   if (param.orientLandscape)
   {
	   sprintf(buf, "Page %ld of %ld  ", thisPage, lastPage);
      xpos=145;
      ypos = lcd_max_y-12;
   } else
   {
	   sprintf(buf, "Page %ld\nof %ld  ", thisPage, lastPage);
      xpos = 130;
      ypos = lcd_max_y-23;
   }
   lcd_Font("6x9");
   lcd_BFcolorsB( lcd_BLACK, lcd_WHITE);
   lcd_DispText(buf, xpos, ypos, MODE_NORMAL);
}
/*
void lcd_screenSaver(char brightness)
{  // lower brightness after 60 seconds of dim command
   // works like a 2-state watchdog.  if brightness is not LCD_BRIGHT after 60 sec then make dim
	static char lastBright;
   static unsigned long dimTimer;
   #GLOBAL_INIT
   { lastBright==LCD_BRIGHT;
     dimTimer=MS_TIMER;
   }

   if ((brightness==LCD_BRIGHT) && (lastBright==LCD_DIM))
   {  lcd_Backlight(LCD_DIM); //brightness);
   	dimTimer=MS_TIMER;
   }
   else if (brightness == LCD_DIM)
   {  if (MS_TIMER - dimTimer > 60000)
   	{  lcd_Backlight(LCD_DIM); //brightness);
	   	dimTimer=MS_TIMER;  // retrys again in 60 sec
      }
   }
}
*/
// TCP web server interface
int tcpServerCheck()
{  // Issue HTTP request to system server for status information
   // Process returned information

	// TCPIP buffer is global
   int status;
   int ii;
 	unsigned long tcpWaitTimer;
   unsigned long processTime;
   unsigned long leadTime;
   char ltmsg[12];
   static int usrcount;
   static int tcpretry;
   char postContent[50];
   char msgbuf[30];
   struct tm timeptr;
   int iSee;
   int iRcv;

   #GLOBAL_INIT
   {	// initialize local variables
		usrcount=0;
      tcpretry=0;
   }

   iSee=0;
   iRcv=0;

	// always open the socket
   //printf("\nOpen TCP socket %ld\n", MS_TIMER);
   processTime = MS_TIMER;
   message_add(SYSTEMLCD, MSG_CHECKING_SERVER1, MSG_CHECKING_SERVER2, ONESHOT);

   if (!tcp_open(&tcpsock, 0, resolve(param.serverID), 80, NULL)) {
      printf("TCP Failed to open %ld\n", MS_TIMER - processTime);
      sprintf(msgbuf, "TCP FAILED OPEN   ");
   } else {
      //printf("Wait for established %ld\n", MS_TIMER - processTime);
      while(!sock_established(&tcpsock)) {
         if (!tcp_tick(&tcpsock)) {
            printf("TCP Failed to establish %ld\n", MS_TIMER - processTime);
            sprintf(msgbuf, "TCP NOT ESTABLISHD");
				break;
         }
      }
   }
   if (sock_established(&tcpsock)) {
      //printf("TCP socket ready %ld!\n", MS_TIMER - processTime);
      // build sync timestamp from param.UIDSync
      mktm(&timeptr, param.UIDSync);
		sprintf(postContent,"sysnum=%d&trct=%ld&rno=%d&gao=%d&refr=%04d%02d%02d%02d%02d%02d",
      	param.systemNum, transactionCount(), param.UIDRecNo, UID_GetAddOnly,
      	timeptr.tm_year+1900, timeptr.tm_mon, timeptr.tm_mday, timeptr.tm_hour, timeptr.tm_min, timeptr.tm_sec);
		//sprintf(postContent,"sysnum=%d&trct=%d&refr=%s", 20, 1434, "20131028000000");
      sprintf(tcpBuffer,"GET %s?%s\n HTTP/1.1\nHost: %s\n\n","/pts/web/app.php/sys2321", postContent, param.serverID);
		//printf("%s\n", tcpBuffer);
      sock_puts(&tcpsock,tcpBuffer);

      // If UIDRecNo = 0 and/or UIDSync = 0 then need to reset usrcount
      if (param.UIDRecNo == 0) usrcount=0;

      // wait for bytes to be ready OR TIMEOUT
      ii=-1;
      tcpWaitTimer = MS_TIMER;
      while((ii < 0) && (!Timeout(tcpWaitTimer, 5000))) { ii=sock_bytesready(&tcpsock); tcp_tick(&tcpsock); }
      // printf("sock_bytesready = %d  %ld\n", ii, MS_TIMER - processTime);
      printf("a.Socket state %d  %ld\n", tcp_tick(&tcpsock), MS_TIMER - processTime);
// how many bytes detected
iSee=ii;
leadTime = MS_TIMER - tcpWaitTimer;
sprintf(ltmsg, "%ld ms ", leadTime);
lcd_printMessage(1, ltmsg);
      // get the response message
      if (ii>0) ii=sock_gets(&tcpsock,tcpBuffer,ii < sizeof(tcpBuffer) ? ii : sizeof(tcpBuffer));
		// try to immediately close the socket since we are done communicating
      sock_close(&tcpsock);
      if (ii>0)
      {
	      // how many bytes retrieved
	      iRcv = ii;
         printf("%s\n",tcpBuffer);
         // parse for UID records and add/del as needed
        	lcd_printMessage(3, "CHECKING NEW USERS");
         status = UID_Parse(tcpBuffer); // parse and process UID records
         Sync_Clock(tcpBuffer);         // set date/time from server
         Sync_Trans_Log(tcpBuffer);     // Check for discontinuous transaction record
			msgbuf[0]=0; // no message yet
         if (status == 50) pingTimer = SEC_TIMER - 1; // trigger another server check for next block
         if (status == 0)
         {	usrcount = 0; // all done
            UID_GetAddOnly = 0; // all up to date, ok to accept deletes
			}
         if (status == -1)
         {  // put up a message
            sprintf(msgbuf, " ADD USER FAILED    ");
         } else if (status > 0) {
         	usrcount += status;
            sprintf(msgbuf, "%4d USERS CHANGED  ", usrcount);
         }

         //printf("Done %ld\n", MS_TIMER - processTime);
      } else
      {  printf("No data in sock_gets %ld\n", MS_TIMER - processTime);
         sprintf(msgbuf, "NO SERVER DATA    ");
      }
      //printf("b.Socket state %d %s  %ld\n", tcp_tick(&tcpsock), sockstate(&tcpsock), MS_TIMER - processTime);
      //if (msgbuf[0] > 0) // if we have a message then show it
      //{  lcd_printMessage(3, msgbuf);
      //   msDelay(1000);
      //}
	   //sprintf(msgbuf, "SEE %d RCV %d ", iSee, iRcv);
	   //lcd_printMessage(3, msgbuf);
	   //msDelay(1000);

      // In case there was a previous retry, reset the counter
      tcpretry=0;

   } else
   {  // socket not established, do a few retries
      if (tcpretry<2)
      {  pingTimer = SEC_TIMER - 1; // trigger another server check for next block
         tcpretry++;
      } else // don't retry, but reset counter
      {  tcpretry=0;
      }
   }
   if (msgbuf[0] > 0) // if we have a message then show it
   {  lcd_printMessage(3, msgbuf);
      msDelay(1000);
   }
   // perform check of parameter flash write timer
	Param_Flash_Write(3);

	// ip_print_ifs();
   return 0;
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

	if (ID==NULL)
   {  // erase all UID's
   	for (i=0; i<UIDSize; i++)
      {	memset(param.UID[i], 0, UIDLen);
      }
      param.UIDSync = 0;  // reset sync timestamp
      param.UIDRecNo = 0;
      return 0;
   }
   else
   {  if (UID_Encode(ID, enc) >= 0)
   	{
	      i = UID_Search(enc);
	      if (i >= 0)
         {  memset(param.UID[i], 0, UIDLen);
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
   {	if (memcmp(param.UID[i], ID, UIDLen) == 0) return i;
   }
	return -1;
}
int UID_Get(int UIndex)
{  // return the (6 or 12 byte)? UID at position UIndex

}
int UID_Send(char *UID, char how)
{  // send the 12 byte UID to the remote stations
	// how = 1 to add, 0 to delete
   // if UID[0] > 99 and how = delete then send delete-all
   // returns 0 if successful, 1 if failed
   struct iomessage cmd;
   int status;
   int i;
   char enc[UIDLen];

   status=0; // assume none good
   if (UID_Encode(UID, enc) >= 0)
   {	if (UID[0]=='d') enc[0]='d'; // to trigger delete all
      cmd.station=0;
      for (i=0; i<UIDLen; i++) cmd.data[i]=enc[i];  // byte 1-5
	   cmd.command = how ? UID_ADD : UID_DEL;

      msDelay(100);  // it takes the diverter 95 msec to retransmit to the remotes
	   for (i=1; i<=MAX_DEVADDRS; i++)
	   {  if ( (1 << (i-1)) & availableDevices )  // if i is a remote devAddr
	      { 	hitwd();
         	cmd.devAddr=i;
		      if (send_command(cmd)==1) status |= (1 << (i-1));
	      }
	   }
   }
   return (status == availableDevices) ? 0 : 1;
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
int UID_Parse(char *UIDBuffer)
{  // parse through input string for UID's and add or delete them as needed
	// returns the quantity of user records found (add or delete)
   // or -1 if add failed or send failed
   // sets flash write timer to trigger next event
	char * Ubuf;
   char action;
   char UID[15];  // could be a date code as well
   int UIndex;
   int i;
   int iadd;
   int ii;
   int isend;
   int stop;
   int retval;
   unsigned long tempsync;
   retval=0;
   iadd=0;
   isend=0;

   // specify input key to find in the buffer
   Ubuf = strstr(UIDBuffer, "users[");
   if (Ubuf != NULL)
	{  UIndex = 6; // first user
   	// loop through string until no more
      while (UIndex > 0)
      {  action = Ubuf[UIndex];
      	UIndex++;
         // get the UID
         // next digits, up to 12 or 15
         i=0;
         stop=0;
         while (!stop && i<15)
         {	if (isdigit(Ubuf[UIndex+i])) UID[i]=Ubuf[UIndex+i];
         	else stop=1;
         	i++;
         }
         i--;      // gone one too far
         UID[i]=0; // add null term
         UIndex += i;
			if (i >= 1) // at least 1 digit
         {
	         if (action == '+')
	         {  i=UID_Add(UID);
            	if (i==-1) iadd=-1; // capture add failure
            	ii=UID_Send(UID, 1);  // send add to remotes
	            printf("Add UID %d %d %s\n", i, ii, UID);
               if (ii) {
               	isend=-1;
                  printf("FAILED to send %s\n", UID);
               }
               retval++;
	         }
	         else if (action == '-')
	         {  i=UID_Del(UID);
            	ii=UID_Send(UID, 0);  // send del to remotes
	            printf("Del UID %d %d %s\n", i, ii, UID);
               if (ii) {
               	isend=-1;
               	printf("FAILED to send %s\n", UID);
               }
               retval++;
	         }
	         else if (action == 'e') // Timestamp of last record, sync date
	         {  if (iadd==0 && isend==0) tempsync = cvtTime(UID, 0);
               // will only sync if no more records at this sync
	         }
	         else if (action == 'l') // Last Record, sync record number
	         {  if ((iadd==0) && (isend==0))
            	{  param.UIDRecNo = atoi(UID); // only if no add or send failure
					   if (param.UIDRecNo == 0)
	               {  // only updtae time sync when record sync is 0
                     param.UIDSync = tempsync;
			            printf("Time sync at %s\n", UID);
                  } else
                  {  printf("Rec sync at %d\n", param.UIDRecNo);
                  }
               }
	         }
         }
         // move to next token or end
         // ,+ or ,- or ,d
         if (Ubuf[UIndex] == ',') UIndex ++; // next
         else UIndex = 0; // done
		}
      // setup next flash write trigger whenever new users are found
		if (retval > 0) Param_Flash_Write(2);
   }

   if ((iadd==-1) || (isend==-1)) retval=-1;  // return -1 for any add or send failure
   return retval;
}
int Sync_Clock(char *Buffer)
{  // parse through input string for server clock time and apply here
	char * Ubuf;

   Ubuf = strstr(Buffer, "time[");
   if (Ubuf != NULL)
	{  // date/time starts at Ubuf + 5
	   Ubuf += 5;
		cvtTime(Ubuf,1);  // sync RTC
   }
}
unsigned long cvtTime(char *timestring, char setRTC)
{  // take time packed string and convert to timestamp
	// timestring is YYYYMMDDHHMMSS
   // set identifies if system time should be set (0 = no)
	char Buf[5];
   struct tm timeptr;
   unsigned long timerval;

   // put into time structure for conversion
   strncpy(Buf, &timestring[12], 2);
   timeptr.tm_sec = atoi(Buf);
	strncpy(Buf, &timestring[10], 2);
   timeptr.tm_min = atoi(Buf);
	strncpy(Buf, &timestring[8], 2); timeptr.tm_hour = atoi(Buf);
	strncpy(Buf, &timestring[6], 2); timeptr.tm_mday = atoi(Buf);
	strncpy(Buf, &timestring[4], 2); timeptr.tm_mon = atoi(Buf);
	strncpy(Buf, &timestring[0], 4); timeptr.tm_year = atoi(Buf)-1900;

   timerval = mktime(&timeptr);

   // are we syncing the RTC?
   if (setRTC)
   {  tm_wr(&timeptr);
   	SEC_TIMER = timerval;
      setTimeMessages(MSG_DATE_TIME, timerval);
      syncDateAndTime(); // send to slave
   }

	return timerval;
}
int UID_ResyncAll()
{  // clear all users, reload from server, send to remotes
   // just need to clear the sync-timestamp and the system naturally reconstructs
   char UID[13];
   int i;

	UID_Del(NULL);  	  // delete local list

   UID[0]='d';     	 // trigger remotes to delete all
   for (i=1; i<13; i++) UID[i]='0';
   UID[12]=0; 			// null term just in case
   UID_Send(UID, 0); // delete all at remotes as well
   pingTimer = SEC_TIMER; // force a server check
   UID_GetAddOnly = 1; // only retrieve adds until no more left

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
void UID_Copy(char *Udest, char *Usrc)
{  memcpy(Udest, Usrc, UIDLen);
}
void Param_Flash_Write(char command)
{  // command:
	//  1 = reset/clear timer
   //  2 = set timer
   //  3 = check timer and execute flash write
   static unsigned long PFW_timer;

   if (command == 1)
   {  // reset/clear timer
      PFW_timer=0;
   }
   else if (command == 2)
   {  // set timer (fixed 15 minutes = 900 sec)
   	PFW_timer=SEC_TIMER + 60;
   }
   else if (command == 3)
   {  // check timer and execute flash write
   	if ((PFW_timer > 0) && (PFW_timer <= SEC_TIMER))  // timer is active and triggers
      {  // write local flash and send flash write command to remotes
	      writeParameterSettings();
         syncParametersToRemote();
         // clear timer
         PFW_timer=0;
      }
   }
}
int Sync_Trans_Log(char *Buffer)
{  // Check if server log is consistent and update it if necessary
	char * Ubuf;
   unsigned long STrans; // servers last contiguous transaction
   long entry;

   Ubuf = strstr(Buffer, "trans[");
   if (Ubuf != NULL)
	{  // last contiguous transaction count starts at Ubuf + 6
	   Ubuf += 6;
		// extract trans count
      STrans = atol(Ubuf);
      // compare to actual count
      if (STrans < transactionCount())
      {  // find starting position to transmit
         entry = findTransaction(STrans);
			// retransmit log
         uploadTransLog(entry);
      }

   } // else nothing to do
}
void checkNsendHeartbeat(char sample, char flags)
{
   static unsigned long heartbeat_interval; // control network
   static unsigned long heartbeat_timer;    // control network
   #GLOBAL_INIT
   {
	  	heartbeat_interval=500;
      heartbeat_timer=0;
	}

   // send periodic heartbeat on UDP control network
   if ((MS_TIMER - heartbeat_timer) > heartbeat_interval)
   {  // Send a UDP heartbeat
      sendUDPHeartbeatToCtrl(sample, flags);
      heartbeat_timer=MS_TIMER;
      monitorUDPactivity(0);  // force a refresh of station activity in case all are off
   }

}