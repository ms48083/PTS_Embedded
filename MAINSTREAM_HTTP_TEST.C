/****************************************************************
   MAINSTREAM_HTTP_TEST.C


*****************************************************************/
#define FIRMWARE_VERSION "HTTP TEST V4.20"
#define VERS                            4
#define SUBVERS                         20

#define MYDEBUG nodebug
#define STDIO_DISABLE_FLOATS
#define STDIO_ENABLE_LONG_STRINGS
// define PRINT_ON for additional debug printing via printf
#define PRINT_ON 1
#define USE_TCPIP 1
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
#define NUMDATA   5
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
{  // must be identical in size to trans_log_type
   unsigned long trans_num;     // transaction number
   unsigned long start_tm;      // time of departure
   unsigned long sid;
   char  status;                // secure removal event = 68
   char  flags;                 // transaction flags, probably 0
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
#define ESTS_BLANK        255

#define NUMUDPDATA 293
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
	#define TCPCONFIG 106
	#define TCP_MODE_ASCII 1
	//#define TCP_VERBOSE 1
	#define MAX_UDP_SOCKET_BUFFERS 1
	#define MAX_TCP_SOCKET_BUFFERS 4
	//#define DISABLE_TCP		// Not using TCP
   #define MY_STATIC_IP "192.168.0.9"
   #define MY_BASE_IP "192.168.0.%d"
   char myIPaddress[18];  // string to hold runtime IP address
	#define MY_IP_NETMASK "255.255.255.0"
	#define LOCAL_PORT   1236
	#define REMOTE_IP    "192.168.0.101" //255.255.255.255" /*broadcast*/
	#define REMOTE_PORT  1236
	#use "dcrtcp.lib"
   // #use “http.lib”
	udp_Socket sock;
   tcp_Socket tcpsock;
	char tcpBuffer[4096];
	int sendUDPHeartbeat(char sample);
	int sendUDPcommand(struct UDPmessageType message);
   void sendUDPtransaction(struct trans_log_type);
#endif

// Including parameter block structure
// All parameters here are read to / written from FLASH
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
   char cardL3Digits;     // Constant left 3 digits of valid secure cards
	unsigned long cardR9Min;  // Secure card ID minimum value for the right 9 digits
   unsigned long cardR9Max;  // Secure card ID maximum value for the right 9 digits
   char card2L3Digits;     // Constant left 3 digits of valid secure cards
	unsigned long card2R9Min;  // Secure card ID minimum value for the right 9 digits
   unsigned long card2R9Max;  // Secure card ID maximum value for the right 9 digits
   char version;          // Main version
   char subversion;       // Main subversion
   char mainVisualAlert;  // enable visual alert at main
   char enableTubeDryOpt; // enable menu item selection
   // -->> BE SURE to change NUMUDPDATA if more parameters are added <<--
} param;
#define SYSTEM_NAME  9  // last one is the system name   char station_names[12][8];
#define ADMIN_PHONE  9
#define MAINT_PHONE  10

// ------------------------------
// MISCELLANEOUS DEFINITIONS HERE
// ------------------------------
char  alarm_silenced_flag;
char diverter_map[9];    // to assign diverter positions to stations
unsigned long diverter_start_time;  // these used primarily by diverter functions
int diverter_status;
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
#define LAST_STATION    7   // # of bits from STATION_SET  (DON'T COUNT SLAVE FOR NOW)!!!
char    STATION_SET;        // active stations
//char    activeStations;     // as programmed from the menu
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
void slaveProcessCommand(struct iomessage message);
void slaveProcessIO(void);
void slaveProcessState(void);
void slaveSyncTransCount(char *p);  // for slave to receive transaction count
void slaveEnableCommands(void);
char slaveGetCommand(struct iomessage *message);
void slaveSendResponse(struct iomessage message);
char slaveComActive(void);  // for slave to know if communication is active
void slaveFinishTransaction(void);
void initTransLog(void);
void downloadTransLog(int numDays); // to local serial port
void uploadTransLog();   // to datalog server
void addTransaction(struct trans_log_type trans);
int getTransaction(long entry, struct trans_log_type *trans);
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
#define SYSTEM_EVENT       'V'
#define ESECURE_REMOVAL    'W'
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

// Digital I/O definitions
// INPUTS
int readDigInput(char channel);
// dio_ON|OFF must be used ONLY in primatives readDigInput and setDigOutput
// they reflect the real state of the logic inputs and outputs
#define dio_ON  0
#define dio_OFF 1
// Menu/Next button assigned to input I-0
#define di_carrierArrival    		readDigInput(0)
#define di_carrierInChamber  		readDigInput(1)
#define di_doorClosed        		readDigInput(2)
#define di_priorityRequest   		readDigInput(3)
#define di_returnCarrier     		readDigInput(4)
//#define di_pushButton(value) 	!readDigInput(5+value)
//#define di_P2P_carrierArrival 	readDigInput(5)
//#define di_P2P_carrierInChamber	readDigInput(6)
//#define di_P2P_doorClosed			readDigInput(7)
//#define di_requestToSend     	((~digBankIn(1)) & STATION_SET)
#define di_requestToSend     		((~digBankIn(1)) & 0xFF)
#define di_diverterPos       		(readDigInput(16) | (readDigInput(17) << 1))
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
#define do_closeDoorAlarm(value)   setDigOutput(6+do_shift,value)
#define do_blowerVac(value)        setDigOutput(12+do_shift,value)
#define do_blowerPrs(value)        setDigOutput(13+do_shift,value)
//#define do_blowerALT(value)        setDigOutput(9+do_shift,value)
#define do_blower(value)           setDigOutput(9+do_shift,value)
#define do_blowerDly(value)		  setDigOutput(16+do_shift,value)
//#define do_audibleAlert(value)            setDigOutput(11+do_shift,value & param.mainAudibleAlert)
#define do_audibleAlert(value)            setDigOutput(11+do_shift,value)
#define do_diverter(value)         setDigOutput(8+do_shift,value)
//#define do_alarmLight(value)       setDigOutput(9+do_shift,value)
#define do_alarmSound(value)       setDigOutput(10+do_shift,value)
#define do_priorityLight(value)    setDigOutput(14+do_shift,value)
#define do_CICLight(value)         ((void)0)
#define do_visualAlert(value)      setDigOutput(15+do_shift,value)
void setAlerts(char how);    // sets both audible and visual alerts
//#define do_P2P_CICLight(value) 	  setDigOutput(17+do_shift,value)
//#define do_P2P_alert(value)	 	  setDigOutput(18+do_shift,value)
#define beep(value)                rn_keyBuzzerAct(DevRN1600, value, 0)
MYDEBUG void inUse(char how, char station_b);
MYDEBUG void alarm(char how);
//void alert(char how, char station_b);

MYDEBUG char getKey(void);
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
"Insert Carrier and"," Press Send Button",
" NEXT TRANSACTION ","  STAT PRIORITY   ",
"  LOG TO PRINTER  ","  LOCAL DIVERTER  ",
"  ENTER PASSWORD  "," VERIFY PASSWORD  ",
"  TRANS TIMEOUT   ","  PLEASE WAIT...  ",
"   SYSTEM ALARM   "," SEE MAINTENANCE  ",
" SETTING DIVERTER ","    DOOR OPEN     ",
"CARRIER IN CHAMBER","DELIVERY UNREMOVED",
" DELIVERY OVERDUE ","  CARRIER RETURN  ",
"   MANUAL PURGE   ","TRANSACTN CANCELED",
" NO COMMUNICATION ","  FROM DEVICE #   ",
"   WAITING FOR    "," SYSTEM CONFIG    "," SET TIME & DATE  ",
" PENDING DELIVERY ","     IN ROUTE     "," AT HEAD DIVERTER ",
"     TIMEOUT      ","      ERROR       ","  REMOTE STATION  ","  FUNCTION MODE   ",
"    SET HOURS     ","   SET MINUTES    ","   SET SECONDS    ",
"     SET DAY      ","    SET MONTH     ","     SET YEAR     ",
" (+)=UP (-)=DOWN  "," (+)=ON  (-)=OFF  "," AUTO PURGE TIMER ",
" BEEP ON ARRIVAL  ","  SILENCE ALARM   ","   RESET ALARM    ",
"   SYSTEM RESET   ","  SYSTEM IS BUSY  ",
"   CAN NOT SEND   ","can't send reason ",
" empty 3 xxxxxxx  "," empty 4 xxxxxxx  ",
" empty 5 xxxxx    ","  OPTIC BLOCKED   ",
"        @M        ",
"        @1        ","        @2        ","        @3        ","        @4        ",
"        @5        ","        @6        ","        @7        ","        @8        ",
"        <M        ",
"        <1        ","        <2        ","        <3        ","        <4        ",
"        <5        ","        <6        ","        <7        ","        <8        ",
"        >M        ",
"        >1        ","        >2        ","        >3        ","        >4        ",
"        >5        ","        >6        ","        >7        ","        >8        ",
"SECURE PIN = #### ","  empty 6 xxxxx   ",
" empty 7 xxxxxxx  ","  empty 8 xxxxxx  ","  empty 9 xxxxxx  ",
" --END OF MENU--- ","    NOT READY     ","   ERROR SAVING   ",
"  STACK AT MAIN   "," STACK AT REMOTE  ","  RESET COUNTER   ",
"  TIME: HR:MN     ","  DATE: MO/DY/YR  ","                  ",
"  CURRENTLY Oxx   ","  MO/DY/YR HR:MN  ","    DO NOT USE    ",
"   empty 10 xxx   "," empty 11 xxxxxxx ",
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
 MSG_empty3_xxx,
 MSG_empty4_xxx,
 MSG_empty5_xxx,
 MSG_OPTIC_BLOCKED,
 MSG_AT,                 // points to message before station 1
 // no 53-60
 MSG_FROM        = 61,   //   ..
 // no 62-69
 MSG_TO          = 70,   //   ..
 // no 71-78
 MSG_PIN         = 79,
 MSG_empty6_xxx,
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
 MSG_empty10_xxx,
 MSG_empty11_xxx,
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

// General/Misc definitions
char init_communication(void);
void print_comstats(void);

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
void lcd_resetMsgBackColor(void);
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
char lcd_editPhoneNums(void);
char lcd_editPortMapping(char * title);
void lcd_showCIC(char forceit);
void lcd_setClock(void);
char lcd_defineActiveStations(void);
void lcd_showTransSummary(void);
void lcd_showDirectory(char mode);
void lcd_showPage(long thisPage, long lastPage);
void lcd_showTransPage(long page, long lastPage, char format);
void lcd_showTransactions(char format);
void lcd_show_inputs(void);
char lcd_sendTo(void);
char lcd_returnCarrier(void);
char lcd_autoReturn(void);
//void lcd_screenSaver(char brightness);
//void lcd_smartPrompt(char promptNum);
char * ltrim(char * text);
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
const char blowerStateTable[15][2] = {
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
	blwrIDLE, blwrIDLE };// 0x0E


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

int tcp_server_check(void);

void main()
{
   /* Declare local variables */
   struct iomessage testcmd, testrsp;
   unsigned long testtime, t1, t2, comfail;
   char i;
   char tcpHeartBeat;
   char * genptr;
   auto rn_search newdev;
   int status;
   unsigned long lastHeartBeat;
   unsigned long loopTimer;
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
   }
   else status = rn_digOutConfig(DevRN1100, OUTCONFIG);  //configure safe state

   // show startup screen
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

   hitwd();

   // Initialize other system configuration
   initialize_system();
   initializeMessages();     // Initialize dynamic (RAM) messages
   loadTransactionCount();   // Read transaction counter from EEPROM
   setDiverterConfiguration();     // setup diverter mappings.

#ifdef USE_TCPIP
   // move sock_init up here to avoid unexpected interrupt from calling tcp_tick before sock_init
   if(sock_init()) printf("IP sock_init() failed\n");	// Initialize UDP communications
#endif

   /* Exercise all lighted outputs except on watchdog reset */
   // if (!wderror())
   if (1) // don't have way to check if reset was by watchdog
   {
	   // show missing H/W on LCD
	   if (DevRN1100 == -1)
	   {  //lcd_print(1, 0, "NO RN1100 FOUND...  ");
		   lcd_DispText("NO RN1100 DETECTED\n",0, 0, MODE_NORMAL);
	      //msDelay(1000);
	   }

      // Perform bulb check
	   lcd_DispText("Indicator Check\n",0, 0, MODE_NORMAL);
      exercise_outputs();
      if (param.slaveController == 0)
      {
	      checkRemoteConfiguration();             // how many remotes, etc.

	      // get and show extended data
	      testcmd.devAddr=ALL_DEVICES;
	      testcmd.command=RETURN_EXTENDED;
	      testcmd.station=0;
	      testcmd.data[0]=0;
	      testcmd.data[1]=0;
	      testcmd.data[2]=0;
	      send_n_get(testcmd, &testrsp);
	      // Show extended data
	      show_extended_data();
		   hitwd();

	      // send message for 3 seconds and show statistics
	      testcmd.command=RETURN_INPUTS;
	      for (i=0; i<3; i++)
	      {
	         hitwd();
	         testtime=MS_TIMER;
	         while (MS_TIMER-testtime < 1000)
	         { msDelay(7);
	           send_n_get(testcmd, &testrsp);
	         }
	      }
	      show_comm_test(0); // don't wait

         syncDateAndTime();  // put the clock out on the com bus
         if (syncParametersToRemote())
         {
			   lcd_DispText("REMOTES NOT SYNC'D",0, 0, MODE_REV);
            msDelay(800);
         }
         sendParametersToLogger(); // send to logger
         if (slaveAvailable) setActiveMain(ON); // default to receive here
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
   {
      //lcd_print(SYSTEMLCD, 0, "  SYSTEM RESET   ");
      //lcd_print(SYSTEMLCD, 1, "WATCHDOG TIMEOUT ");
      //msDelay(3000);
   }

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
	sprintf(myIPaddress, MY_BASE_IP, param.systemNum);
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
   if(!udp_open(&sock, LOCAL_PORT, -1/*resolve(REMOTE_IP)*/, REMOTE_PORT, NULL))
   {  printf("udp_open failed!\n");
      //lcd_print(1, 0, "No IP Communications");
      msDelay(1000);
   }

   // wait a little till the interface is up
   while (!ifstatus(IF_ETH0)) {tcp_tick(NULL); msDelay(100); printf("-"); }


	// go into endless loop talking to the server
   loopTimer = SEC_TIMER-5;
   while(1)
   {
   	tcp_tick(NULL);
   	if (SEC_TIMER > loopTimer)
		{  loopTimer = SEC_TIMER + 5;
      	tcp_server_check();
      }
	}


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
			if (SEC_TIMER != lastHeartBeat)
         {	// Send a tcp heartbeat once per second
         	sendUDPHeartbeat(tcpHeartBeat++);
            lastHeartBeat = SEC_TIMER;
         }

	   }
   } else // SLAVE MAIN LOOP
   {
		slaveEnableCommands();
		while(1)
      {
	      maintenance();  // watchdog, led activity, UDP commands, inUse lights
	      //rn_keyProcess(DevRN1600, 0);  // process keypad device
	      showactivity();      // flashes the board led
         slaveProcessIO();
      }
   }

// sock_err:
//   switch(status) {
//      case 1: /* foreign host closed */
//         printf("User closed session\n");
//         break;

//      case -1: /* time-out */
//         printf("\nConnection timed out\n");
//         break;
//   }
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
   if (how)
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
	   lcd_DispText("ERROR LOADING PARAMETERS",0, 0, MODE_NORMAL);
      //msDelay(800);
   }
   // data type mismatch, force two parameters to char
   param.deliveryTimeout &= 0xFF;
   param.autoPurgeTimer &= 0xFF;

   // make sure systemNum is valid
   if (param.systemNum > 20) param.systemNum=1;  // default to 1
   param.version = VERS;
   param.subversion = SUBVERS;

   // Set default for 2nd card range if not yet initialized
   if (param.card2L3Digits == 255)
   {  param.card2L3Digits = 155;
      param.card2R9Min = 437355717;
      param.card2R9Max = 437357715;
   }

   // setup other globals
   statTrans=0;
   autoRtnTrans=0;
   secureTrans=0;
   slaveAvailable = FALSE;  // ASSUME NOT
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
   //param.mainAudibleAlert=TRUE;  make sticky v4.20
   alarm_silenced_flag=FALSE;
   malfunctionActive=FALSE;
   echoLcdToTouchscreen=TRUE;
	latchCarrierArrival=FALSE;  // clear latch
   param.manPurgeTimeout = 5;  // fixed at 5 minutes
   reset_statistics();
   queueDel(0,0);            // initialize request queue

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
   {  secureTransLog[i].sid=0;
   	secureTransLog[i].start_tm=0;
      securePIN[i]=-1;
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

char remote_data[5];
// event log is modified in multiple modules so make global
struct trans_log_type eventlog;
void processSystemIO()
{


}
char checkSecureRemoval(void)
{  // Look into secure tracking structure for any removed secure transactions
}
char autoReturnCarrier()
{  // check if an auto return timer is expired and the return can be started
}

char checkDoorWhileRunning(char calling_state)
{  // Check for main or remote doors opening
}

void processMainArrivalAlert(void)
{
}

void processStatPriority(void)
{  // use btnStatFlag and btnSecureFlag to communicate between lcd screen functions and here
}

const char msgTable[4][2] = {MSG_BLANK, MSG_BLANK,
 									  MSG_BLANK, MSG_DROPEN,
                             MSG_CIC,   MSG_BLANK,
                             MSG_CIC,   MSG_DROPEN};
void update_cic_door_msgs(void)
{  // Add or remove CIC and Door messages from the TRANSLCD

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
}
void blower(char request)
{  // operates both standard blowers and APU blowers
}
char processBlower()
{   // call repeatedly to handle process states of the blower

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
   send_n_get(command, &response);
   //if ( ( (response.data[0] & STATION_SET)==STATION_SET)
   if ( (response.data[0]==availableDevices)
       && ( (diverter_attention==FALSE)
             || (diverter_map[station]==0) ) )
   {   rtnval=TRUE;  }

   // setup the diverter status to find out which device is not ready
   diverter_status = response.data[0];
   if ((diverter_map[station]==0) || (diverter_attention==FALSE)) diverter_status |= 0x100;

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
// define input functions which take master/slave as a parameter
char carrierInChamber(char whichMain)
{	if (whichMain==MASTER) return di_carrierInChamber;
	else return slaveData[0] & 0x01;
}
char carrierArrival(char whichMain)
{	if (whichMain==MASTER) return di_carrierArrival;
	else return slaveData[0] & 0x04;
}
char doorClosed(char whichMain)
{	if (whichMain==MASTER) return di_doorClosed;
	else return slaveData[0] & 0x02;
}
void setAlerts(char how)
{  // sets both audible and visual alerts
	do_audibleAlert(how);
	do_visualAlert(how);
}

//**************************************
//   KEYPAD INPUT FUNCTIONS
//**************************************
char getKey(void)
{  // Return a key from the keypad
   char wKey;
   char gk;
   static char last;

   //rn_keyProcess(DevRN1600, 0);  // process keypad device
   //wKey = rn_keyGet(DevRN1600, 0);
   // reset menu timeout every time a key is pressed
   if (wKey) menu_timeout=SEC_TIMER + 60;
   else
   {  // no keypad key so look for request-to-send key
	   gk = bit2station(di_requestToSend);
      // return the key when the button is released
	   if (gk==0 && last!=0)
	   {  wKey=last;
	      last=0;
	      menu_timeout=SEC_TIMER + 60;
	   } else
	   {  last=gk;
	   }
   }
   return wKey;
}

char valid_send_request(char station_b)
{  // make sure conditions are right to send a carrier
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
      MNU_SECURE_L3DIGITS,
      MNU_SECURE_R9MIN,
      MNU_SECURE_R9MAX,
      MNU_SECURE2_L3DIGITS,
      MNU_SECURE2_R9MIN,
      MNU_SECURE2_R9MAX,
      MNU_SHOW_INPUTS,
      MNU_TUBE_DRYER,
      MNU_ENAB_DRYER_OPT,
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
 MNU_SECURE_L3DIGITS,"CARD1 LEFT 3 DIGITS", MNU_VAL,  &param.cardL3Digits, 154, 154, "ID",
 MNU_SECURE_R9MIN,   "CARD1 RIGHT 9 MIN",   MNU_VAL,  (char *)&param.cardR9Min, 200000000, 618000000, "ID",
 MNU_SECURE_R9MAX,   "CARD1 RIGHT 9 MAX",   MNU_VAL,  (char *)&param.cardR9Max, 200000000, 618000000, "ID",
 MNU_SECURE2_L3DIGITS,"CARD2 LEFT 3 DIGITS", MNU_VAL,  &param.card2L3Digits, 155, 155, "ID",
 MNU_SECURE2_R9MIN,   "CARD2 RIGHT 9 MIN",   MNU_VAL,  (char *)&param.card2R9Min, 000000000, 999999999, "ID",
 MNU_SECURE2_R9MAX,   "CARD2 RIGHT 9 MAX",   MNU_VAL,  (char *)&param.card2R9Max, 000000000, 999999999, "ID",
 MNU_SHOW_INPUTS,    "SHOW INPUTS",        MNU_OTHR, &NOch, 0, 0, "",
 MNU_TUBE_DRYER,     "RUN TUBE DRYER",     MNU_OTHR, &NOch, 0, 0, "",
 MNU_ENAB_DRYER_OPT, "ENABLE TUBE DRYER",  MNU_FLAG, &param.enableTubeDryOpt, 0, 1, "",
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
      menuOrder[i++]=MNU_AUTO_PURGE;
      menuOrder[i++]=MNU_PURGE;
      menuOrder[i++]=MNU_VIEW_LOGS;
      if (param.slaveController == 0)
      {  // these menu items not allowed at slave
			menuOrder[i++]=MNU_ADMIN_FEATURES;
	      menuOrder[i++]=MNU_MAINT_FEATURES;
      }
      menuOrder[i++]=MNU_SYSTEM_CONFIG;
   	break;
   case 2: // Administrator
   	if (param.enableTubeDryOpt) menuOrder[i++]=MNU_TUBE_DRYER;
      menuOrder[i++]=MNU_MARRIVAL_BEEP;
      menuOrder[i++]=MNU_VISUAL_ALERT;
      menuOrder[i++]=MNU_RARRIVAL_BEEP;
      menuOrder[i++]=MNU_ARETURN_BEEP;
      menuOrder[i++]=MNU_MSTACKING;
      menuOrder[i++]=MNU_RSTACKING;
      menuOrder[i++]=MNU_AUTOPUR_TIMER;
      menuOrder[i++]=MNU_AUTO_RETURN;
      menuOrder[i++]=MNU_DNLD_TLOG;
      menuOrder[i++]=MNU_UPLD_TLOG;
      menuOrder[i++]=MNU_SET_CLOCK;
      menuOrder[i++]=MNU_RESET_COUNT;
      menuOrder[i++]=MNU_SET_STANAMES;
      menuOrder[i++]=MNU_SET_PHONENUMS;
      menuOrder[i++]=MNU_SETPASSWORD;
   	break;
   case 3: // Maintenance
      menuOrder[i++]=MNU_SETTIMEOUT;
      menuOrder[i++]=MNU_AUTO_PURGE;
      menuOrder[i++]=MNU_PURGE;
      menuOrder[i++]=MNU_SETPASSWORD;
      menuOrder[i++]=MNU_LCD_CALIBRATE;
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
      menuOrder[i++]=MNU_SETASSLAVE;
      menuOrder[i++]=MNU_SUBSTA_ADDR;
      menuOrder[i++]=MNU_SECURE_SETUP;
      menuOrder[i++]=MNU_PORT_MAPPING;
      menuOrder[i++]=MNU_SHOW_IPADDR;
      menuOrder[i++]=MNU_COMM_STAT;
      menuOrder[i++]=MNU_SETPASSWORD;
      menuOrder[i++]=MNU_CLEARPASSWORDS;
      menuOrder[i++]=MNU_CHECK_SFLASH;
      menuOrder[i++]=MNU_ANALYZE_LOG;
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
      menuOrder[i++]=MNU_SECURE_L3DIGITS;
      menuOrder[i++]=MNU_SECURE_R9MIN;
      menuOrder[i++]=MNU_SECURE_R9MAX;
      menuOrder[i++]=MNU_SECURE2_L3DIGITS;
      menuOrder[i++]=MNU_SECURE2_R9MIN;
      menuOrder[i++]=MNU_SECURE2_R9MAX;
   	break;
	}
   menuOrder[i]=MNU_LAST;

}
#define PAGE_SIZE 5
const char * const title[7] = { "ALARM MENU", "MENU", "ADMIN MENU", "MAINTENANCE MENU", "CONFIGURATION MENU", "LOGS MENU","SECURE TRANS MENU" };
char lcd_ShowMenu(char menuLevel, char page)
{  // returns if a next page is available
	char i, j;
   char rtnval;
   int row, count, button;
   char ttl;
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
	      BTN_TLXY, 40, row,             // starting x,y for buttons
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
      	uploadTransLog();
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
	      lcd_DispText("SYSTEM RESTARTING", 50, 85, MODE_NORMAL);
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

void setActiveMain(char setToMaster)
{
	if (setToMaster) param.activeMain = MASTER;
   else             param.activeMain = SLAVE;

   if (param.slaveController)
   {  // set flag to send new status
      if (param.activeMain==MASTER) slaveReturnStatus |= 0x04;
      else slaveReturnStatus |= 0x02;
   }
   setActiveMainMsg();

}
void setActiveMainMsg()
{  // setup active main message shown on lcd
	char i;
	char mybuf[20];
	char start;

   // Only show Rcv@ if slave detected and sub station addressing is off
   if ((slaveAvailable || param.slaveController) && (param.subStaAddressing==FALSE))
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
	// Left 3 digits
   command.data[0]=0; // Left 3 digits
   command.data[1]=param.cardL3Digits;
   send_command(command);
   msDelay(10);
   // Right 9 minimum
   command.data[0]=1; // Right 9 min
   *(unsigned long *)&command.data[1] = param.cardR9Min;
   send_command(command);
   msDelay(10);
   // Right 9 maximum
   command.data[0]=2; // Right 9 max
   *(unsigned long *)&command.data[1] = param.cardR9Max;
   send_command(command);
   msDelay(10);
   //// SECOND CARD RANGE
	// Left 3 digits
   command.data[0]=3; // Left 3 digits
   command.data[1]=param.card2L3Digits;
   send_command(command);
   msDelay(10);
   // Right 9 minimum
   command.data[0]=4; // Right 9 min
   *(unsigned long *)&command.data[1] = param.card2R9Min;
   send_command(command);
   msDelay(10);
   // Right 9 maximum
   command.data[0]=5; // Right 9 max
   *(unsigned long *)&command.data[1] = param.card2R9Max;
   send_command(command);
   msDelay(10);

	// send command to remote to update and save other parameters
	command.command=SET_PARAMETERS;
	command.station=0;
   status=0; // assume none good
   for (i=FIRST_DEVADDR; i<=LAST_DEVADDR; i++)
   {  if ( (1 << (i-1)) & availableDevices )  // if i is a remote devAddr
		{	command.devAddr=i;
	   	command.data[0]=param.portMapping[i];
			command.data[1]=param.blowerType;
			if (send_command(command)==1) status |= (1 << (i-1));
      }
   }

   return (status == availableDevices) ? 0 : 1;
}
#ifdef USE_TCPIP
int sendUDPcommand(struct UDPmessageType UDPmessage)
{
	// Send the supplied command over UDP
   // Appends some info to the standard message
   //   .device
   //   .timestamp

   auto int    length, retval;
   char * UDPbuffer;  // to index into UDPmessage byte by byte

   // fill the packet with additional data
   UDPmessage.device = param.systemNum;  // system id
   UDPmessage.devType = 1; // main station
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

   sendUDPcommand(HBmessage); // ignore return value, will send again periodically
}
void sendUDPtransaction(struct trans_log_type translog )
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
      TLmessage.station = translog.flags;
      TLmessage.data[sizeof(translog)] = param.cardL3Digits-CARDHBOFFSET; // add on 5th byte of 38 bit card id
   }
   else // otherwise must be some kind of event
	   TLmessage.command = SYSTEM_EVENT;

   // stuff the transaction log data straight into UDP buffer
   TLbuffer = (char *) &translog;
   for (idx = 0; idx < sizeof(translog); idx++)
      TLmessage.data[idx] = TLbuffer[idx];

   sendUDPcommand(TLmessage); // ignore return value, will send again periodically
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
   for (idx = 0; idx < sizeof(param); idx++)
      PBmessage.data[idx] = PBbuffer[idx];

   sendUDPcommand(PBmessage); // ignore return value, will send again periodically

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

   // process flashing inUse lights
   if ((MS_TIMER %500) < 300) digBankOut(0, ~(outputvalue[devInUseFlash] | outputvalue[devInUseSolid]));
   else digBankOut(0, ~outputvalue[devInUseSolid]);

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
#define CMD_LENGTH  11
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
               else if (workmsg.command==SECURE_REMOVAL)
               {  // store secure badge id and delta-time
                  if (secureTransLog[workmsg.station].start_tm)
                  {  // not yet logged
	                  secureTransLog[workmsg.station].sid = *(unsigned long *) &workmsg.data[0];
   	               secureTransLog[workmsg.station].flags = workmsg.data[4];  // save scaled delta-t for now
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

   return good;
}
/********************************************************/
// slave com functions
/********************************************************/
void slaveEnableCommands()
{  // open serial port D
   serDopen(BAUD_RATE);
	ser485Rx();			// enable the receiver
   serDrdFlush(); 	// flush the read buffer
}
char slaveComActive()
{ if (SEC_TIMER - lastComTime > 2) return FALSE;
  else return TRUE;
}
/********************************************************/
void slaveSendResponse(struct iomessage message)
{  /* Transmits a message/command to the main system.
      Command string defined as:
      <STX> <lplc> <Command> <Station> <Data0..4> <ETX> <CHKSUM> */

   char i;
   char cmdstr[CMD_LENGTH];
   char cmdlen;
   char rcvbuf[CMD_LENGTH];
   char rcvlen;
   unsigned long timeout, cmdsent;
   cmdlen=CMD_LENGTH;

   // enable transmitter
   ser485Tx();

   /* formulate the full command string */
   cmdstr[0] = STX;
   cmdstr[1] = SLAVE_DEVADDR;
   cmdstr[2] = message.command;
   cmdstr[3] = message.station;
   for (i=0; i<NUMDATA; i++) cmdstr[i+4]=message.data[i];
   cmdstr[cmdlen-2] = ETX;
   cmdstr[cmdlen-1] = 0;
   for (i=0; i<cmdlen-1; i++) cmdstr[cmdlen-1] += cmdstr[i]; /* checksum */

   // send
   cmdlen = serDwrite(cmdstr, CMD_LENGTH);
   if (cmdlen != CMD_LENGTH) printf("Send command failed, only sent %d chars \n", cmdlen);

   disable485whenDone();

}
/********************************************************/
char slaveGetCommand(struct iomessage *message)
{
/* Repeatedly call this function to process incoming commands efficiently. */
   char charsinbuf, msg_address;
   char rtnval, chksum, i;
	#define rbuflen      30
	#define sbuflen      5
	#define SLAVE_RCV_TIMEOUT 1
	static char rbuf[rbuflen], sbuf[sbuflen];
	static char bufstart;

   rtnval=FALSE;     /* assume none for now */

   // align frame to <STX>
   while ((serDrdUsed() > 0) && (serDpeek() != STX)) serDgetc(); //printf("%ld tossing %d\n",MS_TIMER, serDgetc());

   // if at least 1 command in buffer then get them
   if (serDrdUsed() >= CMD_LENGTH)
	   charsinbuf = serDread(rbuf, CMD_LENGTH, SLAVE_RCV_TIMEOUT);

   bufstart = 0;
   if (charsinbuf==CMD_LENGTH)  /* all characters received */
   {
	   /* verify STX, ETX, checksum then respond */
	   if ((rbuf[bufstart] == STX) && (rbuf[bufstart+CMD_LENGTH-2] == ETX))
	   {
	      /* check checksum */
	      chksum=0;
	      for (i=0; i<CMD_LENGTH-1; i++) chksum+=rbuf[bufstart+i];

	      if (chksum != rbuf[bufstart+CMD_LENGTH-1])
	      {
	         sbuf[0]=NAK;
	         rtnval=FALSE;
	      }
	      else
	      {
	         /* looks good, send ACK */
	         sbuf[0]=ACK;
	         rtnval=TRUE;
	      }
         lastComTime = SEC_TIMER;
	      msg_address = rbuf[bufstart+1];
	      // send response if message is addressed to me only
	      if (msg_address==SLAVE_DEVADDR)
         {
	         // enable transmitter and send
	         ser485Tx();
         	serDwrite(sbuf, 1);
			   disable485whenDone();
         }

	      // If this message for me OR everyone then process it.
	      if ((msg_address==SLAVE_DEVADDR) || (msg_address==ALL_DEVICES))
	      {  // message for me (and possibly others)
	         /* return the command for processing */
	         message->devAddr=msg_address;
	         message->command=rbuf[bufstart+2];
	         message->station=rbuf[bufstart+3];
		      for (i=0; i<NUMDATA; i++) message->data[i]=rbuf[i+bufstart+4];

	      } else rtnval=FALSE;  // command not for me!

         //serDrdFlush(); // flush the read buffer

	   }
   }
   return rtnval;
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
      {  sys_message[MSG_NOCOMM+1].msg[MSG_OFFSET+14]=48+firstBit(comError);  // show device #
         message_add(SYSTEMLCD, MSG_NOCOMM, MSG_NOCOMM+1, NEXT);
      }
      else message_del(SYSTEMLCD, MSG_NOCOMM, MSG_NOCOMM+1);
   }
   // else if (updateclock-2 > SEC_TIMER) updateclock=SEC_TIMER;  // means updateclock is grossly out of whack

   // log to printer on failure and restore
   if ((comError!=0) && (wasComError==0))
   {   // error occured now
   	strcpy(eventMsg, "LOST REMOTE COMMUNICATIONS @ DEV #");
      eventMsg[33]=48+firstBit(comError);
      print_event(eventMsg);
      wasComError=comError;
   } else if ((wasComError!=0) && (comError==0))
   {  // error cleared now
      print_event("REMOTE COMMUNICATIONS RESTORED");
      wasComError=comError;
   }
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
   slaveAvailable = FALSE;  // ASSUME NOT
   FIRST_DEVADDR=0;   // number
   LAST_DEVADDR=0;    // number
   workset=0;
   //ALL_DEVADDRS=((1 << MAX_DEVADDRS) - 1 ) | 0x80; // bits ... including high bit
   availableDevices = 0xFF;  // address all possible devices

   //lcd_print(SYSTEMLCD, 0, " CHECKING REMOTE");
   //lcd_print(SYSTEMLCD, 1, " CONFIGURATION  ");
	lcd_DispText("Detecting Remote Devices\n",0, 0, MODE_NORMAL);
   // hold for a few seconds
   //msDelay(100);

   strcpy(mymsg, "  ");
   STATION_SET = 0;                  // startoff with none
   for (j=1; j<=MAX_DEVADDRS; j++)
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
   command.station=0;
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
void deliverComControl(char *sys_state)
{  // let slave use the comm for a while
   struct iomessage mymsg;
   char waiting;

   param.slaveController=SLAVE; // TEMP ID

   // tell slave ok
   mymsg.command=TAKE_COMMAND;
   mymsg.devAddr=SLAVE_DEVADDR;
   mymsg.data[0]=*sys_state;
   mymsg.data[1]=availableDevices;
   mymsg.data[2]=remote_data[REMOTE_CIC];
   send_command(mymsg);

   // display lcd message
   lcd_printMessage(2, " SLAVE IN USE   ");
   lcd_printMessage(3, " PLEASE WAIT... ");

   // setup diverter at master for slave to purge
   set_diverter(SLAVE);

   // wait for response
   waiting=TRUE;
   slaveEnableCommands();
   lastComTime=SEC_TIMER;
   while (waiting)
   {
      check_diverter();
      maintenance();

      if (slaveGetCommand(&mymsg))
      {  if ( (mymsg.command==TAKE_COMMAND) || (mymsg.command==RESET) )
         {  waiting=FALSE;
            *sys_state=mymsg.data[0];
         }
         lastComTime=SEC_TIMER;
      }
      if (SEC_TIMER - lastComTime > 60) waiting=FALSE;  // timeout if no com for 60 seconds
   }
   param.slaveController=MASTER; // RESET Address

}

void slaveSyncTransCount(char *p)
{  // for slave to sync transaction count from master
   char *myp;
   myp=(char*)&transaction_count;
   *myp++ = *p++;
   *myp++ = *p++;
   *myp++ = *p++;
   *myp++ = *p++;
   setCountMessage();
   root2xmem( transCountXaddr, &transaction_count, 4);
}
void slaveFinishTransaction()
{
   setAlerts(OFF); //, station2bit(systemStation));
   // set arrival alert if the transaction is to here
   if ( ((system_direction == DIR_RETURN) && (mainStation==SLAVE))
       || ((mainStation==MASTER) && (main2main_trans==DIR_SEND)) )
   {
      //inUse(FLASH, station2bit(message.station));  // until no cic
      slave_arrival = SEC_TIMER;                      // to flash inuse
      rts_latch=0;   // don't allow any out-bounds right now
   }

   mainStation=0; systemStation=0;
   system_direction=0;
   arrivalEnable(OFF);

   // force an increment of the transaction counter even though a sync message should come soon
   // for some reason the sync is not getting through
   transaction_count++;
   setCountMessage();
}
int slaveButton;
void slaveProcessIO()
{  // for slave to handle local I/O processing
   struct iomessage message;
   char k;   // temporary work value
   char menu_item;
   char key;
   int button;
   static char lastDirectory;		// for showing stations on the lcd

   #GLOBAL_INIT
   {
   	lastDirectory = 0;
   }

   // check for and process communications
   if (slaveGetCommand(&message)) slaveProcessCommand(message);

   slaveProcessState();      // operate on system_state

   // process local i/o
   check_diverter();
   //processCycleTime();
   //key = getKey(); // processKeyInput();
   // handle button press
   button = lcd_GetTouch(1);
   if (button != -1)
   {  slaveButton = button;  // save this button press
   	if (!slaveComActive()) lcd_processTouchScreen(slaveButton);
   	else slaveReturnStatus |= 0x08; // let master decide if I can do this
   }

   if (slave_arrival)
   {
     // check for 10 second arrival alert signal
     if (system_state != HOLD_TRANSACTION)   // except when holding
     {  if ( (SEC_TIMER - slave_arrival) %20 >= 10 ) setAlerts(ON);
        else setAlerts(OFF);
     }

     // check to clear main arrival alert when door opens or another transaction starts
     if ( (!di_doorClosed && !di_carrierInChamber)
         || ((mainStation==SLAVE) && (system_direction==DIR_SEND)) )
     {  slave_arrival=0;
        arrival_from=0;
        setAlerts(OFF);
     }
   }

   lcd_showCIC(0);  // show CIC

   // latch transaction requests ... only if there is a carrier or carrierReturn
   k=di_requestToSend; // | station2bit(lcd_sendTo());
   if (di_carrierInChamber || di_returnCarrier) { rts_latch |= (k & STATION_SET); }
   else if (system_state==IDLE_STATE)
   {  rts_latch=0;
      // check for directory display
      if (k != lastDirectory)
      {  //systemStationb=k;
         systemStation=bit2station(k); //systemStationb);
         if (systemStation)
	      { //lcd_print(TRANSLCD, 0, sys_message[MSG_REMOTESTA].msg);
           //lcd_print(TRANSLCD, 1, sys_message[MSG_AT+systemStation].msg);
           currentmsg[TRANSLCD][0]=MSG_REMOTESTA;
           currentmsg[TRANSLCD][1]=MSG_AT+systemStation;
           lcd_displayMsgs(&currentmsg[0][0]);
         }
         lastDirectory = k;
         systemStation=0;  // reset this after directory display
      }
   }
}
void slaveProcessCommand(struct iomessage message)
{
}
void slaveProcessState()
{
}

nodebug
void msDelay(unsigned int delay)
{
   auto unsigned long start_time;
   start_time = MS_TIMER;
   //if (delay < 500) while( (MS_TIMER - start_time) <= delay );
   //else
   while( (MS_TIMER - start_time) <= delay )
   {  hitwd();
#ifdef USE_TCPIP
     //if (delay>10) tcp_tick(NULL);
#endif
	}
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
   lcd_DispText(FIRMWARE_VERSION, 230, 5, MODE_NORMAL);
   lcd_DispText(param.station_name[SYSTEM_NAME].name, 10, 5, MODE_NORMAL);
   lcd_DispText("\nTo send a carrier:", 0, 0, MODE_NORMAL);
   lcd_DispText("\nInsert carrier into tube and close the door", 0, 0, MODE_NORMAL);
   lcd_DispText("\nPress the send button for the destination", 0, 0, MODE_NORMAL);
   lcd_DispText("\nOR", 0, 0, MODE_NORMAL);
   lcd_DispText("\nTouch the Send To Directory button", 0, 0, MODE_NORMAL);
   lcd_DispText("\nTouch the send button for the destination", 0, 0, MODE_NORMAL);
   lcd_DispText("\n", 0, 0, MODE_NORMAL);
   lcd_DispText("\nTouch the STAT button for priority dispatch", 0, 0, MODE_NORMAL);
   lcd_DispText("\nTouch the Secure button to enable secure handling", 0, 0, MODE_NORMAL);
   lcd_DispText("\nTouch the Auto Return button to enable automatic return", 0, 0, MODE_NORMAL);
   lcd_DispText("\nTouch the Return Carrier button to return a carrier", 0, 0, MODE_NORMAL);
   lcd_DispText("\nTouch the Menu button to access system features", 0, 0, MODE_NORMAL);
   lcd_DispText("\n", 0, 0, MODE_NORMAL);
   lcd_DispText("\nHelp Phone Numbers", 0, 0, MODE_NORMAL);
   lcd_DispText("\nSystem Administrator: ", 0, 0, MODE_NORMAL);
   lcd_DispText(param.phoneNum[ADMIN_PHONE], 0, 0, MODE_NORMAL);
   lcd_DispText("\nMaintenance: ", 0, 0, MODE_NORMAL);
   lcd_DispText(param.phoneNum[MAINT_PHONE], 0, 0, MODE_NORMAL);
   lcd_DispText("\nColombo Sales & Engineering: 800-547-2820  ", 0, 0, MODE_NORMAL);

	lcd_Font("16B");
   lcd_ButtonDef( BTN_CANCEL,
	   BTN_MOM,                      // momentary operation
      BTN_TYPE, BUTTON_RELEASE,
      BTN_MARGINS, 5, 330,         // set left and right margins
      BTN_SEP, 63, 43,              // set up for button sep
	   BTN_TXTOFFSET, 10, 9,
	   BTN_TLXY, 257, 205,             // starting x,y for buttons
	   BTN_TEXT, "Done",
	   BTN_BMP, BMP_med_button, BMP_med_button_dn,
	   BTN_END );

   // wait for a button press
   menu_timeout = SEC_TIMER + 60;
   while(((button=lcd_GetTouch(100)) == -1) && !secTimeout(menu_timeout) ) maintenance();

}
void lcd_splashScreen(char view)
{  // view 0 for initial startup splash (no wait)
   // view 1 for help->about splash (wait for button)
	int loop_count, i;
   char sbuf[15];

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
	lcd_DispBitmap( BMP_COLOMBO_LOGO, 79, 30 );
   lcd_Font("16");
   lcd_DispText("Colombo Sales & Engineering", 70, 180, MODE_NORMAL);
   lcd_DispText("800-547-2820", 115, 200, MODE_NORMAL);
   if (view)
   {  //lcd_DispText("Software version ", 70, 225, MODE_NORMAL);
      lcd_Font("13");
   	lcd_DispText(FIRMWARE_VERSION, 108, 221, MODE_NORMAL);
      // show phone numbers for admin, maint, and colombo
      msDelay(4000);  // wait 4 seconds
   } else
   {
	   // Please wait message
	   msDelay(1000);
	   lcd_ClearScreen ();
	   lcd_Font("16B");
	   lcd_DispText("Colombo Sales & Engineering\n", 0, 0, MODE_NORMAL);
	   lcd_DispText("Pneumatic Tube System\n", 0, 0, MODE_NORMAL);
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
   int x, dx;
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
	   lcd_Rectangle(5, 5, 315, 40, 1);        // inside paint
	   lcd_BFcolorsB( lcd_BLACK, lcd_WHITE);   // border color
	   lcd_Rectangle(5, 5, 315, 40, 0);        // border paint
	   lcd_Font("24B");
	   lcd_DispText(title, lcd_Center(title, 1), 14, MODE_TRANS);
   }

   // now handle screen specific stuff
   switch (whichScreen)
   {
   case 1:
   	if (title[0] != 0) // Buttons? (any) or not ("")
      {
         // Draw a box for top row of bottom buttons
	      lcd_BFcolorsB( lcd_LBLUE, lcd_LBLUE );   // inside color
	      lcd_Rectangle(5, 139, 315, 197, 1);      // inside paint 158 -> 140
	   	lcd_BFcolorsB( lcd_BLUE, lcd_WHITE);     // border color
	      lcd_Rectangle(5, 139, 315, 197, 0);      // border paint
	      lcd_Font("13B");
		   lcd_BFcolorsB( lcd_BLACK, lcd_WHITE);   // border color
			lcd_DispText("---- Transaction Options ----", 15, 143, MODE_TRANS);

         // Draw a box for Return from
	      lcd_BFcolorsB( lcd_LBLUE, lcd_LBLUE );   // inside color
	      lcd_Rectangle(215, 197, 315, 237, 1);      // inside paint
	   	lcd_BFcolorsB( lcd_BLUE, lcd_WHITE);     // border color
	      lcd_Rectangle(215, 197, 315, 237, 0);      // border paint
         // Draw a box for menu/help
	      lcd_BFcolorsB( lcd_LGREEN, lcd_LGREEN );   // inside color
	      lcd_Rectangle(5, 197, 142, 237, 1);      // inside paint
	   	lcd_BFcolorsB( lcd_BLUE, lcd_WHITE);     // border color
	      lcd_Rectangle(5, 197, 215, 237, 0);      // border paint
	      lcd_Rectangle(5, 197, 142, 237, 0);      // border paint

	      // Draw bottom buttons
	      lcd_BFcolorsD( lcd_BLACK, lcd_VLTGRAY_D );
	      lcd_Font("16B");
	      lcd_ButtonDef( BTN_MENU,
	         BTN_MOM,                      // momentary operation
	         BTN_TLXY, 13, 202, //5, 205,             // starting x,y for buttons
	         BTN_TYPE, BUTTON_RELEASE,
	         BTN_MARGINS, 5, 330,         // set left and right margins
	         BTN_SEP, 63, 43,              // set up for button sep
	         BTN_TEXT, "Menu",
	         BTN_TXTOFFSET, 10, 9,
	         BTN_BMP, BMP_med_button, BMP_med_button_dn,
	         BTN_END );

	      lcd_ButtonDef( BTN_HELP,
	         //BTN_TLXY, 257, 205,
	         BTN_TLXY, 80, 202, //70, 205,
	         BTN_TXTOFFSET, 12, 9,
	         BTN_TEXT, "Help",
	         BTN_BMP, BMP_med_button, BMP_med_button_dn,
	         BTN_END );

	      //lcd_Font("13");
	      //lcd_Font("13B");
	      lcd_ButtonDef( BTN_DIRECTORY,
	         //BTN_TLXY, 114, 198,
	         BTN_TLXY, 220, 153, //135, 198,
	         //BTN_TEXT, "  Station\nDirectory",
	         BTN_TEXT, "Send to /\nDirectory",
	         BTN_TXTOFFSET, 15, 4, //9, //14, 4,
	         //BTN_BMP, BMP_92x40_button, BMP_92x40_button_dn,
	         BTN_BMP, BMP_92x40_button, BMP_92x40_button_dn,
	         BTN_END );
	      //lcd_Font("16B");

	      lcd_Font("13B");
	      lcd_ButtonDef( BTN_CRETURN,
	         //BTN_TLXY, 257, 160,
	         BTN_TLXY, 220, 202, //70, 160,
	         BTN_TXTOFFSET, 15, 3,
	         BTN_TEXT, "Return a\n Carrier",
	         BTN_BMP, BMP_92x32_button, BMP_92x32_button_dn,
	         BTN_END );
//	      lcd_Font("16B");

//	      lcd_Font("13B");
	      lcd_ButtonDef( BTN_ARETURN, BTN_LAT,
	         //BTN_TLXY, 114, 160,
	         BTN_TLXY, 147, 161,
	         BTN_TYPE, autoRtnTrans ? BUTTON_LAT_1 : BUTTON_LAT_0,
	         BTN_TXTOFFSET, 7, 3, 7, 3,
	         BTN_TEXT, " Auto\nReturn", " Auto\nReturn",
            BTN_BMP, BMP_med_button, BMP_med_grn_button_dn, // BMP_92x32_button, BMP_92x32_button_dn,
	         BTN_END );
	      lcd_Font("16B");

	      lcd_ButtonDef( BTN_STAT, BTN_LAT,
	         BTN_TLXY, 13, 161,
	         BTN_TYPE, statTrans ? BUTTON_LAT_1 : BUTTON_LAT_0,
	         BTN_TEXT, "STAT", "STAT",
	         BTN_TXTOFFSET, 12, 9, 12, 9,
	         BTN_BMP, BMP_med_button, BMP_med_grn_button_dn,
	         BTN_END );
         // show secure button only if the feature is enabled
         if (param.secureEnabled)
	         lcd_ButtonDef( BTN_SECURE, BTN_LAT,
	            BTN_TLXY, 80, 161,
	            BTN_TYPE, secureTrans ? BUTTON_LAT_1 : BUTTON_LAT_0,
	            BTN_TEXT, "Secure", "Secure",
	            BTN_TXTOFFSET, 7, 9, 7, 9,
	            BTN_BMP, BMP_med_button, BMP_med_grn_button_dn,
	            BTN_END );
         else
	         lcd_ButtonDef( BTN_FUTURE, BTN_MOM,
	            BTN_TLXY, 80, 161,
               BTN_TYPE, BUTTON_RELEASE,
	            BTN_TEXT, "Future",
	            BTN_TXTOFFSET, 7, 9,
	            BTN_BMP, BMP_med_button, BMP_med_button,
	            BTN_END );


         // Show CIC light on the screen if we are also showing buttons
	      lcd_showCIC(1);
      }

	   // Draw LCD Message boxes
	   //lcd_BFcolorsB( lcd_LGREY, lcd_LGREY);  // inside color
	   //lcd_Rectangle(5, 5, 315, 71, 1);       // inside paint
	   //lcd_Rectangle(5, 71, 315, 137, 1);     // inside paint
	   lcd_BFcolorsB( lcd_BLUE, lcd_LGREY);  // border color
	   lcd_Rectangle(5, 5, 315, 72, 0);       // border paint
	   lcd_Rectangle(5, 72, 315, 139, 0);     // border paint

      // since this screen was just cleared, for a refresh of the lcd messages
      reset_msg_timer(NOW);

      break;

   case 2:

	   // Draw bottom buttons
	   lcd_Font("16B");
	   lcd_ButtonDef( BTN_PREV,
	      BTN_MOM,                      // momentary operation
	      BTN_TLXY, 5, 205,             // starting x,y for buttons
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
	      BTN_TLXY, 257, 205,             // starting x,y for buttons
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
	      BTN_TLXY, 5, 205,             // starting x,y for buttons
		   BTN_TYPE, BUTTON_RELEASE,
	      BTN_TEXT, "OK",
	      BTN_MARGINS, 5, 330,         // set left and right margins
	      BTN_TXTOFFSET, 14, 9,
	      BTN_BMP, BMP_med_button, BMP_med_button_dn,
	      BTN_END );
	   lcd_ButtonDef( BTN_CANCEL,
	      BTN_TXTOFFSET, 5, 9,
	      BTN_TLXY, 257, 205,           // starting x,y for buttons
	      BTN_TEXT, "Cancel",
	      BTN_END );
      break;
   case 5:

	   // Draw bottom buttons
	   lcd_Font("16B");
	   lcd_ButtonDef( BTN_EXIT,
	      BTN_MOM,                      // momentary operation
	      BTN_TLXY, 257, 205,             // starting x,y for buttons
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
	      BTN_TLXY, 5, 205,             // starting x,y for buttons
		   BTN_TYPE, BUTTON_RELEASE,
	      BTN_TEXT, "Next",
	      BTN_MARGINS, 5, 330,         // set left and right margins
	      BTN_TXTOFFSET, 8, 9,
	      BTN_BMP, BMP_med_button, BMP_med_button_dn,
	      BTN_END );
	   lcd_ButtonDef( BTN_CANCEL,
	      BTN_MOM,                      // momentary operation
	      BTN_TLXY, 257, 205,             // starting x,y for buttons
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
	   lcd_DispText(sys_message[MSG_DATE_TIME].msg, 30, 60, MODE_NORMAL);
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

   case 8:  // PgUp, PgDn, Done
	   // Draw bottom buttons
	   lcd_Font("16B");
	   lcd_ButtonDef( BTN_PREV,
	      BTN_MOM,                      // momentary operation
	      BTN_TLXY, 5, 205,             // starting x,y for buttons
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
	      BTN_TLXY, 257, 205,             // starting x,y for buttons
	      BTN_TEXT, "Exit",
	      BTN_END );

   	break;
   case 9:

      lcd_ShowKeypad(1);

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
	      BTN_TLXY, 257, 205,             // starting x,y for buttons
		   BTN_TYPE, BUTTON_RELEASE,
	      BTN_MARGINS, 5, 330,         // set left and right margins
	      BTN_SEP, 63, 43,              // set up for button sep
	      BTN_TEXT, "Exit",
	      BTN_TXTOFFSET, 15, 9,
	      BTN_BMP, BMP_med_button, BMP_med_button_dn,
	      BTN_END );
      break;
   }

}
int lastBkColor;
void lcd_resetMsgBackColor(void) { lastBkColor = -1; }
void lcd_displayMsgs(char * msgNum)
{  // show traditional lcd messages on the color touch screen
//	static int lastBkColor;
	int newColor;
   char myBuf[MSG_LEN+1]; // for date time display
   #GLOBAL_INIT { lastBkColor = -1; }

// DISABLE TOUCH BUTTONS

   lcd_Font("16x32i");
   // TRANS LCD
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
	if (newColor != lastBkColor)
	{  // fill the box
		lcd_BFcolorsB(newColor, newColor);
		lcd_Rectangle(6, 6, 314, 71, 1);       // inside paint
      lastBkColor = newColor;
   }
   if (msgNum[0]==MSG_PIN) refresh_PIN_message(msgNum[0], msgNum[1]);
   if (msgNum[1]==MSG_PIN) refresh_PIN_message(msgNum[1], msgNum[0]);
   //lcd_DispText(sys_message[msgNum[0]].msg, 30, 12, MODE_NORMAL);
   //lcd_DispText(sys_message[msgNum[1]].msg, 30, 42, MODE_NORMAL);
   lcd_BFcolorsB(lcd_BLACK, newColor);
   lcd_DispText(sys_message[msgNum[0]].msg, 16, 8, MODE_NORMAL);
   lcd_DispText(sys_message[msgNum[1]].msg, 16, 38, MODE_NORMAL);

   // SYSTEM LCD
   // check for showing alarm message in red
	if (msgNum[2] == MSG_ALARM) lcd_BFcolorsB(lcd_WHITE, lcd_RED);
   else lcd_BFcolorsB(lcd_BLACK, lcd_WHITE); // inside color
   //lcd_DispText(sys_message[msgNum[2]].msg, 30, 90, MODE_NORMAL);
   //lcd_DispText(sys_message[msgNum[3]].msg, 30, 120, MODE_NORMAL);
   if (msgNum[2]==MSG_PIN) refresh_PIN_message(msgNum[2], msgNum[3]);
   if (msgNum[3]==MSG_PIN) refresh_PIN_message(msgNum[3], msgNum[2]);
   lcd_DispText(sys_message[msgNum[2]].msg, 16, 75, MODE_NORMAL);
   lcd_DispText(sys_message[msgNum[3]].msg, 16, 105, MODE_NORMAL);

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

// ENABLE TOUCH BUTTONS

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
         if (xpos==10) xpos=165;
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

   if (font==1)      temp = 160 - (int)(strlen(string) * 7);  // "24"
   else if (font==2) temp = 160 - (int)(strlen(string) * 4);  // "16"
   if (temp < 5) temp = 5;
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
         lcd_Rectangle(12, 163, 150, 203, 1);  // blank out CIC
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
				lcd_DispText("-->>", 150, y1+pw, MODE_TRANS); // show direction arrow
	      } else
	      {  // progress right to left
	         xmid = x2 - (int)((x2-x1) * (long)progress/100);
	         lcd_Rectangle(xmid, y1, x2, y2, 1);    // fill right side progress
	         lcd_BFcolorsB( lcd_WHITE, lcd_WHITE);  // unfill color
	         lcd_Rectangle(x1, y1, xmid, y2, 1);    // fill left side remaining
		      lcd_BFcolorsB( lcd_BLACK, lcd_WHITE);
            lcd_DispText("<<--", 150, y1+pw, MODE_TRANS); // show direction arrow
	      }
         // show the station names; NOTE: source is really main station and dest is remote station
         lcd_DispText(param.station_name[source].name, 10, y1+pw, MODE_TRANS);
         xmid = 305 - 11*strlen(param.station_name[dest].name);
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

   i=1; // first digit 1
   btn[0]=49;
   btn[1]=0;

   // draw first button
   lcd_Font("16B");
   lcd_ButtonDef( '0'+i,
      BTN_MOM,                      // momentary operation
      BTN_TLXY, 200, 55,               // starting x,y for buttons
      BTN_TYPE, BUTTON_RELEASE,
      BTN_MARGINS, 200, 300,        // set left and right margins
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

	lcd_drawScreen(3, "MENU");
   lcd_Font("24");
   lcd_DispText(label, lcd_Center(label, 1), 50, MODE_NORMAL);
   lcd_DispText(optZero, 150, 85, MODE_NORMAL);
   lcd_DispText(optOne, 150, 115, MODE_NORMAL);

   // show optZero choice
   lcd_ButtonDef( BTN_YES_ON,
   BTN_LAT,                      // latching operation
   BTN_TLXY, 110, 80,             // starting x,y for buttons
   BTN_TYPE, (*choice) ? BUTTON_LAT_1 : BUTTON_LAT_0,
   BTN_BMP, BMP_check_box, BMP_check_box_click,
   BTN_END );
   // show optOne choice
   lcd_ButtonDef( BTN_NO_OFF,
   BTN_TLXY, 110, 110,             // starting x,y for buttons
   BTN_TYPE, (*choice) ? BUTTON_LAT_0 : BUTTON_LAT_1,
   BTN_BMP, BMP_check_box, BMP_check_box_click,
   BTN_END );
   // show Cancel button
   lcd_Font("16B");
   lcd_ButtonDef( BTN_CANCEL,
      BTN_MOM,                      // momentary operation
      BTN_TLXY, 257, 205,             // starting x,y for buttons
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
	   BTN_TLXY, 110, 80,             // starting x,y for buttons
	   BTN_TYPE, *choice ? BUTTON_LAT_1 : BUTTON_LAT_0,
	   BTN_BMP, BMP_check_box, BMP_check_box_click,
	   BTN_END );
	   // show optZero choice
	   lcd_ButtonDef( BTN_NO_OFF,
	   BTN_TLXY, 110, 110,             // starting x,y for buttons
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
   lcd_DispText("Min = ", 50, 120, MODE_NORMAL);
   lcd_DispText(ltoa((long)par_min, number), 0, 0, MODE_NORMAL);
   lcd_DispText("Current = ", 27, 140, MODE_NORMAL);
   lcd_DispText(ltoa((long)*par_value, number), 0, 0, MODE_NORMAL);
   lcd_DispText("Max = ", 47, 160, MODE_NORMAL);
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
	      lcd_Font("16x32i");
		   timedate = mktime(&time);
		   setTimeMessages(MSG_DATE_TIME, timedate);
	      lcd_DispText(sys_message[MSG_DATE_TIME].msg, 30, 60, MODE_NORMAL);
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
char lcd_defineActiveStations()
{  // define the active stations
   // returns TRUE if activestations were changed
   char rtsIn;
   char done;
   char j, col, msgbuf[17];
   char original;
   char inKey;
   char inputStations;
   int button;

	lcd_drawScreen(5, "DEFINE ACTIVE STATIONS");
	lcd_DispText("PRESS ACTIVE SEND BUTTONS", 60, 50, MODE_NORMAL);
   lcd_Font("12x24");
   strcpy(msgbuf, sys_message[MSG_BLANK].msg);  // buffer to show stations

   original = STATION_SET;  // save for later
   STATION_SET = 0xFF;      // to allow requestToSend to return any pushbutton
   inputStations = 0;      // reset
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
      // If station button, include bit and show on display
      rtsIn = firstBit(di_requestToSend);
// P2P WORK TO BE DONE HERE
      if (rtsIn)
      {  inputStations |= station2bit(rtsIn);
         col = (rtsIn * 2) -1;
         msgbuf[col] = 48 + rtsIn;
         //lcd_print(SYSTEMLCD, 1, msgbuf);
			lcd_DispText(msgbuf, 55, 80, MODE_NORMAL);
	      menu_timeout = SEC_TIMER + 60;  // reset menu timeout
      }
   }
   if (STATION_SET==0) STATION_SET = original;  // don't allow zero stations
   param.activeStations = STATION_SET;
   if (STATION_SET == original) return FALSE;
   else return TRUE;
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
      BTN_MARGINS, 0, 340-32,
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
   lcd_ButtonDef( 32, BTN_TEXT, "Space", BTN_TLXY, 114, 203,
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
	         BTN_TLXY, 5, 205,             // starting x,y for buttons
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
         lcd_Rectangle(40, 100, 300, 160, 1);    // fill white
         lcd_BFcolorsB( lcd_BLACK, lcd_WHITE);
         lcd_Rectangle(40, 100, 300, 160, 0);    // outline
         lcd_DispText("Press Reset again to confirm\nor Exit to cancel", 60, 115, MODE_TRANS);

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
      BTN_TLXY, 5, 205,             // starting x,y for buttons
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

void addTransaction(struct trans_log_type trans)
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
   sendUDPtransaction( trans );

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
//#define RS232_MONITOR_F
void uploadTransLog()
{  // open UDP port and transmit the log
   long entry;
   char j;
   struct trans_log_type log;
   struct tm time;
   unsigned long startTime;

   // give some status
 	lcd_drawScreen(3, "UPLOAD LOG");
   lcd_Font("24");
   lcd_DispText("In progress ...", 80, 80, MODE_NORMAL); // show the default name

	for (entry=0; entry<sizeOfTransLog(); entry++)
   {
   	// get entry
		if (getTransaction(entry, &log) == 0)
		{
         // send the transaction to the server
         sendUDPtransaction( log );

         // wait till the socket is empty to pace the transmission
			startTime = MS_TIMER;
         while ((sock_tbused(&sock) > 0) && ((MS_TIMER-startTime) < 500)) maintenance();
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
         	sprintf(buf, "%02d/%02d/%02d %02d:%02d:%02d %-11s %-10s %6ld %2x%2x",
            time.tm_mon, time.tm_mday, time.tm_year%100, time.tm_hour, time.tm_min, time.tm_sec,
	         trans.flags < SYSTEM_NAME ? param.station_name[trans.flags].name : "??",
            "SECURE ID",
            (* (unsigned long*) &trans.duration) - CARDBASE,
            (int) trans.status, (int) trans.flags );
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
void lcd_showPage(long thisPage, long lastPage)
{  // show page x of y
   char buf[25];
   sprintf(buf, "Page %ld of %ld  ", thisPage, lastPage);
   lcd_Font("6x9");
   lcd_BFcolorsB( lcd_BLACK, lcd_WHITE);
   lcd_DispText(buf, 145, 218, MODE_NORMAL);
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
int tcp_server_check()
{
   int status;
   int ii;
 // TCPIP buffer
 	unsigned long tcpWaitTimer;
   unsigned long processTime;
   char postContent[50];

	// always open the socket
   printf("\nOpen TCP socket %ld\n", MS_TIMER);
   processTime = MS_TIMER;
   if (!tcp_open(&tcpsock, 0, resolve(REMOTE_IP), 80, NULL))
      printf("TCP Failed to open %ld\n", MS_TIMER - processTime);
   else {
      printf("Wait for established %ld\n", MS_TIMER - processTime);
      while(!sock_established(&tcpsock)) {
         if (!tcp_tick(&tcpsock)) {
            printf("TCP Failed to establish %ld\n", MS_TIMER - processTime);
				break;
         }
      }
   }
   if (sock_established(&tcpsock)) {
      printf("TCP socket ready %ld!\n", MS_TIMER - processTime);
      // do whatever needs to be done...
      //sock_mode(&tcpsock,TCP_MODE_ASCII);
		sprintf(postContent,"sysnum=%d&trct=%d&vlt=33", 20, 1434);
      sprintf(tcpBuffer,"GET %s?%s\n HTTP/1.1\nHost: %s\n\n","/pts/web/app.php/sys2321", postContent, REMOTE_IP);
      sock_puts(&tcpsock,tcpBuffer);

      // wait for bytes to be ready OR TIMEOUT
      ii=-1;
      tcpWaitTimer = MS_TIMER;
      while((ii < 0) && (!Timeout(tcpWaitTimer, 2000))) { tcp_tick(&tcpsock); ii=sock_bytesready(&tcpsock); }
      printf("sock_bytesready = %d  %ld\n", ii, MS_TIMER - processTime);
      printf("a.Socket state %d  %ld\n", tcp_tick(&tcpsock), MS_TIMER - processTime);

      // get the response message
      if (ii>0) ii=sock_gets(&tcpsock,tcpBuffer,sizeof(tcpBuffer));
      if (ii>0) {
         printf("%s\n",tcpBuffer);
         printf("Done %ld\n", MS_TIMER - processTime);
      } else printf("No data in sock_gets\n");

      printf("b.Socket state %d %s  %ld\n", tcp_tick(&tcpsock), sockstate(&tcpsock), MS_TIMER - processTime);
   }
   return 0;
}