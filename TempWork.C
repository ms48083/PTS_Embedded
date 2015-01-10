struct PayloadType   // data package sent to remotes
{
	char device;
   char devType;  // M for main station
   char command;  // can be heartbeat, event, transaction
   char station;
   unsigned long timestamp;  // MS_TIMER
	char mainArrival;
   char systemState;
   char remoteAlert;
   char transInProgress;
   char mainStation;
   char diverterStation;
   char subStaAddressing;
   char transFlags;
   char secureAckStation;
   int  securePin;
   char transDirection;
   char purgeMode;
   char mainBusy;
   char headDivAlign;
};


 send_payload_to_remotes();

   command.devAddr=ALL_DEVICES;
   command.command=RETURN_INPUTS;
   if (param.subStaAddressing) command.station=mainStation;
   else  command.station=param.activeMain;
   // send the main arrival state to indicate not ready for transaction
   if (main_arrival || slave_arrival) Payload.mainArrival=arrival_from;
   else Payload.mainArrival=0;
   // send the system_state (for headdiverter controller to operate blower)
   command.data[1]=system_state;
   command.data[2]=outputvalue[devInUseSolid] & ~mainFlashAtSlave;		// was system_direction;
   command.data[3]=(outputvalue[devInUseFlash] & ~SLAVE_b) | mainFlashAtSlave;
   command.data[4]=STATION_SET;


 This happens in Retrieve_Remote_Inputs
         for (k=0; k<5; k++)
        {  tempData = response.data[k];
           tempData &= response.station;  // Turn off unused bits
           remote_data[k] |=  (tempData & response.station);  // ON Bits
           remote_data[k] &= ~(tempData ^ response.station); // OFF bits
        }
        // force unused doors closed (on) and other unused inputs off
        remote_data[REMOTE_DOOR] |= ~STATION_SET;
        remote_data[REMOTE_CIC] &= STATION_SET;
        remote_data[REMOTE_RTS] &= STATION_SET;
        remote_data[REMOTE_ARRIVE] &= STATION_SET;
		  if (param.subStaAddressing && slaveAvailable) remote_data[REMOTE_RTS2] &= STATION_SET;
        else remote_data[REMOTE_RTS2] = 0;


 set_remote_diverters(systemStation);
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
         if (received) new_state=WAIT_FOR_DIVERTERS;
         else new_state=CANCEL_STATE;

FROM DIVERTER CONTROLLER         
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

