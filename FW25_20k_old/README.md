# Polymonitor A5 Development Firmware Release Notes

  ## [v0.0.25.0]  [v0.0.25.1] 
  
  **  LED DRIVER [lp55281]
  **  lp55281 production test
  
      H = 7<LF> - for activation
	  
    The calculated voltage (mV) value is the voltage between the selected pin and ground.
    The internal LDO voltage is used as a reference voltage for the conversion. The accuracy of LDO is ± 3%, which
    is defining the overall accuracy. The non-linearity and offset figures are both better than 2LSB.
	

    log example:

	[LP55281] R1[0.000000]   
 	          G1[2620.800049]
			  B1[2620.800049] 
			  R2[2429.699951] 
			  G2[3221.399902]
			  B2[2730.000000] 
			  R3[2511.600098]
			  G3[3248.699951]
			  B3[2730.000000] 
			  R4[2511.600098]
			  G4[3248.699951]
			  B4[2730.000000] <CR><LF>
			  ALED[2457.000000]H 11 = SYS_ERROR_NONE OK<LF>
			  
 ** add Release build configuration	for FOTA
 
	Release vs debug comparison:

    _____________|       Debug      |     Release       |		<br>	  
	Debug info   |      ✅  -g3     |       ❌          |      <br>
	Version      | a5_00.00.0025.00 |  a5_00.00.0025.01 |      <br>
	Degug log    |      ✅  -g      |       ❌          |      <br>
	Macros       |   DEBUG is set   |       ❌          |      <br>
	Optimization |      ❌  -O0     |       -Os         |      <br>



	Image size comparison :

	Debug:   3.43 MB (3,601,036 bytes)
	Release: 346  KB (355,152 bytes)

	text	   data	    bss	    dec	    hex	filename  <br>
	202611	   1296	 682472	 886379	  d866b	a5_00.00.0025.00.elf<br>
	131520	   1240	 680536	 813296	  c68f0	a5_00.00.0025.01.elf<br>

------------------------------------------------------------------------
## [v0.0.24] 

** Fixed HW/SW interface for all Analog channels

   a) Change Channles 7,11,13 to 6,10,12 (6=audio,10=resp,12=scl)
   b) Reverse polarity of audio,resp and scl signals 

** Task 5 Add bio-Z channel 11 to command set : 32 SPS , 80 KHZ

    BIOZ FIFO Data Structure:
	[23:0]=>[20DATA][1RESERV][3BTAG]=> RAWDATA>>8 = 16 bits
	
	/* put in queue and throw 3bits btag data out and 5 LSB bits */
		status = fifo_put32(bioz_fifo, ((int) (buffer[i])) >> (1+3+4));

	Channel Default configuration:
		Sample Rate - 32 Sample/Sec
		Sample Size - 16 bits 
		
    BIOZ is not dependent of other channels state (enable/disable)	
		
	* capture calibration signal for validation	
	* btag field validated

**  changes in Parcel Structure A5 Protocol

   For Detailes: 
   https://docs.google.com/document/d/1KcSqy_oXv8PWxnv7FjHMvxuXXi2mzgAMcOTTlyM18xc/edit#heading=h.o2iefqhuosut 

## [v0.0.14]

1. Channel 2 [3d ACC LSM6DSL] - [full scale changed to 4g][add here link]
2. Json  ["C"] responce modification: "RESP" -> "Respiration"
3. add TMP117 hardware test commands: H 8 ?<LF> H 9 ?<LF> H 10 ?<LF>
4. fixed issue memory issue when all channels are on
5. fixed order in channels 2 and 1 
6. fixed edf header file
7. Adjust ADC ref values for analog channels.
8. MAX30001/3  remove 2 LSB  bits

## [v0.0.15]

1. Add: enable MAX30001 VCM for analog channels
   (in case whenNo other MAXReqiered channela are active in the current session)
2. Add: release notes file in md format to the project tree
3. Add: BT DLE support (244 bytes)


## [v0.0.16]

1. BLE troughput encreased to 7.5 K (channels 0,1,2,3,4,5)
2. FwVer 30.2.3.0  -ATI3
3. AppVer  "4.03"  -Application version (ATI33)
4. LibVer  "5.04"  -Library version  (ATI23)

## [v0.0.17]

1. vesion stucture change:

	<[PROD_NAME]>_<[Castomer facing:MAJOR]>.<[Castomer facing:MINOR]>.<[Castomer facing:MICRO>.<BRANCH]>

	PROD_NAME - product name - "a5"
	'_' - delimiter
	MAJOR - Customer facing 1byte
	PROD_NAME - product name - "a5"
	'.' - delimiter
	MINOR - Customer facing 1byte
	'.' - delimiter
	MICRO - Customer facing 2byte
	'.' - delimiter
	BRANCH - 1byte  0-development 1-release
	
2. add JSON lib to the project [cJSON][ https://github.com/DaveGamble/cJSON/blob/master/cJSON.c ]
3. add A5 cJSON parser/ builder 
   * Get command/s - String Array Example: ["v", "v"] - array with get commnds (see Get multi comand [JSON])
   * Set command/s - Multi Key object Example:  (see GStart [JSON])
   

4. add Json command Handler for following commands:
    https://docs.google.com/document/d/1fMlqM4nJKn7vRaAHkVwwwt3VPaSxfLKQJJXQdzSQ6kU/edit#	

	Get Version Once the cloud gets the version, 
	it can pull the schema from KiSS, so it knows 
	the channels and their available rates.
	
	example:
	
	Request:	["v","k","m","v"]
	Response:	{"v":"a5_00.00.0017","v":"a5_00.00.0017"}
	
	more commands:
	
	Hardwate test related:
	PM will test each channel whenever it’s set to “on”, and report falue value instaed in the response, e.g.

    Example:  "c":[ {"m": 1, "r": 512}]...
    Response: "c":[ {"m": -1, "r": 512}]...

    all supported commands examples:
	
	Get all channels configuration [JSON]
	5B 22 63 22 5D 0A

	Get multi comand [JSON]
	5B 22 76 22 2C 22 63 22 2C 22 6B 22 2C 22 74 22 5D 0A

	Get Version [ JSON ]
	5B 22 76 22 5D 0A

	Get RTC [ JSON ]
	5B 22 74 22 5D 0A

	Set Channels configuration [JSON] - disable all
	7B 22 63 22 3A 5B 7B 22 6D 22 3A 20 30 2C 20 22 72 22 3A 20 35 31 32 7D 2C 7B 22 6D 22 3A 20 30 2C 20 22 72 22 3A 20 31 32 38 7D 2C 7B 22 6D 22 3A 20 30 2C 20 22 72 22 3A 20 33 32 7D 2C 7B 22 6D 22 3A 20 30 7D 2C 7B 22 6D 22 3A 20 30 2C 20 22 72 22 3A 20 33 32 7D 2C 7B 22 6D 22 3A 20 30 2C 20 22 72 22 3A 20 33 32 7D 2C 7B 22 6D 22 3A 20 30 2C 20 22 72 22 3A 20 33 32 7D 2C 7B 22 6D 22 3A 20 30 2C 20 22 72 22 3A 20 33 32 7D 2C 7B 22 6D 22 3A 20 30 7D 5D 7D 0A

	Set Channels configuration [JSON] - enable some
	7B 22 63 22 3A 5B 7B 22 6D 22 3A 20 31 2C 22 72 22 3A 20 35 31 32 7D 2C 7B 22 6D 22 3A 20 31 2C 20 22 72 22 3A 20 31 32 38 7D 2C 7B 22 6D 22 3A 20 31 2C 20 22 72 22 3A 20 33 32 7D 2C 7B 22 6D 22 3A 20 31 7D 2C 7B 22 6D 22 3A 20 31 2C 20 22 72 22 3A 20 33 32 7D 2C 7B 22 6D 22 3A 20 30 2C 20 22 72 22 3A 20 33 32 7D 2C 7B 22 6D 22 3A 20 30 2C 20 22 72 22 3A 20 33 32 7D 2C 7B 22 6D 22 3A 20 30 2C 20 22 72 22 3A 20 33 32 7D 2C 7B 22 6D 22 3A 20 31 2C 22 72 22 3A 20 35 31 32 7D 5D 7D 0A

	Set Channels configuration [JSON] - ecg ch0 mode and rate
	7B 22 63 22 3A 5B 7B 22 6D 22 3A 20 31 2C 22 72 22 3A 20 35 31 32 7D 5D 7D 0A

	Set Channels configuration [JSON] - only rate ecg ch0
	7B 22 63 22 3A 5B 7B 22 72 22 3A 20 35 31 32 7D 5D 7D 0A

	Set RTC [JSON]
	7B 22 74 22 3A 20 31 36 35 33 38 31 37 35 30 31 7D 0A

	Start [JSON]
	7B 22 74 22 3A 20 31 36 35 33 38 31 37 35 30 31 2C 20 22 6C 2E 6D 22 3A 20 31 7D 0A

	Stop [JSON]
	7B 22 6C 2E 6D 22 3A 20 30 7D 0A

5. Set default ODR for CHANNEL 0/8 to 512 samples/sec ECG vert/horiz) 


## [v0.0.18] - "Adasa" Demo Version candidate


** Task 1 Add error recovery without HW reset

	/Solution: Enable WD reset after ~35 seconds of system inacitivity 
	 
	log examples:
		==== NORMAL POWER UP ======<CR><LF>  
	    ==== WARNING!!! POWERED UP after WD reset!!!!!!!======<CR><LF>

** Task 2  Add extended error handler to print file name and line 

log example:
	==== Error_Handler: file ../proto/A5_proto/Parser.c on line 202 ======<CR><LF>
	
** Task 3 Software-Hardware interface, add some missing IO configurations 

** Task 4. Add Hardware validation test for 3 temperature sensors,
        ** with auto sensor selection  (TMP117/AS6221) **
		
		H 8 ?<LF> -> for Skin temperature
		H 9 ?<LF> -> for Control temperature
		H 10 ?<LF> -> for Ambient temperature
		
		log example:
		11:31:17.846 [TX] - H 8 ?<LF>

		11:31:17.858 [RX] - [AS6221][90] detected <CR><LF>
		H 8 = SYS_ERROR_NONE OK<LF>

		11:31:20.087 [TX] - H 9 ?<LF>

		11:31:20.100 [RX] - [AS6221][92] detected <CR><LF>
		H 9 = SYS_ERROR_NONE OK<LF>

		11:31:20.851 [TX] - H 10 ?<LF>

		11:31:20.863 [RX] - [AS6221][94] detected <CR><LF>
		H 10 = SYS_ERROR_NONE OK<LF>
         

** Task 5 Add temperature to command set [channel 9] 

	Default configuration:
		Sample Rate - 1 Sample/Sec
		Sample Size - 16 bits

	Protocol convertions :
		Temperature RESOLUTION = 0.0078125f
		Data order [Skin temp][Control temp][Ambient temp]

** Task 6 Add Gyro to command set [channel 10]

	Gyro resolution 250 dps
	Sample rate is  ~52sps - same rate as accelerometers

** Task 7 Heat enable and temperature allignment

	Add heat_en command to command set (no Json command)
	Integrate "HEAT_EN" control tracking function from ephraim (Anton)

	M = 1<LF> - activation
	M = 0<LF> - de-activation
	M = ?<LF> - get state
	
## [v0.0.19] 

1. Fixed heater enable function (Ephraim)


   
   
