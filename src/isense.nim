# TODO convert DWORD to enums

import unsigned


when defined(linux):
  const dllname = "libisense.so"
elif defined(macos):
  const dllname = "libisense.dylib"
elif defined(windows):
  const dllname = "isense.dll"
else:
  {.error: "io-isense does not support this platform".}


type
  IsdBool*: cint
  IsdHwnd*: cint


const
  islibVersion* = "4.2381"
  isdFalse* = 0
  isdTrue* = 1


type 
  IsdSystemType* {.pure, size: sizeof(cint).} = enum ##  \
    ## Enumerates tracking system types.
    none = 0, ## Not found, or unable to identify 
    precisionSeries, ## InertiaCubes, NavChip, IS-300, IS-600, IS-900 and IS-1200 
    intertraxSeries ## InterTrax 


  IsdSystemModel* {.pure, size: sizeof(cint).} = enum ## \
    ## Enumerates tracking system models.
    unknown = 0,
    is300, ## 3DOF system (unsupported)
    is600, ## 6DOF system (unsupported) 
    is900, ## 6DOF system   
    intertrax, ## InterTrax (Serial) (unsupported)
    intertrax2, ## InterTrax (USB) (unsupported)
    intertraxLS, ## InterTraxLS, verification required (unsupported)
    intertraxLC, ## InterTraxLC (unsupported)
    icube2, ## InertiaCube2 
    icube2pro, ## InertiaCube2 Pro 
    is1200, ## 6DOF system   
    icube3, ## InertiaCube3 
    navchip, ## NavChip
    intertrax3, ## InterTrax3 (unsupported)
    imuk, ## K-Sensor
    icube2bPro, ## InertiaCube2B Pro
    icube2Plus, ## InertiaCube2 Plus
    icubeBT, ## InertiaCube BT
    icube4 ## InertiaCube4


  IsdInterfaceType* {.pure, size: sizeof(cint).} = enum ## \
    ## Enumerates device interface types.
    unknown = 0,
    serial,
    usb, 
    udp,
    tcp, 
    ioCard,
    pcmcia,
    file, 
    pipe


when defined(IsenseLimited):
  const
    isdMaxTrackers* = 2
    isdMaxStations* = 4
else:
  const
    isdMaxTrackers* = 32
    isdMaxStations* = 8


const 
  # Orientation format (3DOF sensors report both regardless of setting;
  # for IS-900, IS-1200).
  isdEuler* = 1
  isdQuaternion* = 2


const
  # Coordinate frame in which position and orientation data is reported.
  isdDefaultFrame* = 1
  isdVsetFrame* = 2


const
  isdMaxButtons* = 8 ## Number of supported stylus buttons.
  isdMaxChannels* = 10 ## Hardware is limited to 10 analog/digital input
    ## channels per station.
  isdMaxAuxInputs* = 4 ## Maximum supported number of bytes for auxiliary input
    ## data
  isdMaxAuxOutputs* = 4 ## Maximum supported number of bytes for auxiliary
    ## output data


type 
  IsdTrackerHandle* = cint


type 
  IsdTrackerInfo* = object 
    # The following items are for information only and should not be changed.
    libVersion*: cfloat ## InterSense Library version 
    trackerType*: IsdSystemType ## See `IsdSystemType <#IsdSystemType>`_
    trackerModel*: IsdSystemModel ## See `IsdSystemModel <#IsdSystemModel>`_
    port*: DWORD # Number of the RS232 port. Starts with 1 (COM1/ttyS0). 
                 # Communications statistics, for information only. 
    recordsPerSec*: uint32
    kBitsPerSec*: cfloat # Following items are used to configure the tracker and can be set in
                         # the isenseX.ini file 
    syncState*: DWORD # 4 states: 0 - OFF, system is in free run 
                      #           1 - ON, hardware genlock frequency is automatically determined
                      #           2 - ON, hardware genlock frequency is specified by the user
                      #           3 - ON, no hardware signal, lock to the user specified frequency  
    syncRate*: cfloat # Sync frequency - number of hardware sync signals per second, 
                      # or, if `SyncState` is 3 - data record output frequency 
    syncPhase*: DWORD       # 0 to 100%    
    Interface*: DWORD       # Hardware interface, read-only 
    ultTimeout*: DWORD      # IS-900 only, ultrasonic timeout (sampling rate)
    ultVolume*: DWORD       # IS-900 only, ultrasonic speaker volume
    dwReserved4*: DWORD
    firmwareRev*: cfloat    # Firmware revision 
    fReserved2*: cfloat
    fReserved3*: cfloat
    fReserved4*: cfloat
    ledEnable*: IsdBool        # IS-900 only, enables flasing blue LEDs on SoniStrips/SoniDiscs if `isdTrue <#isdTrue>`_
    bReserved2*: IsdBool
    bReserved3*: IsdBool
    bReserved4*: IsdBool

  #/////////////////////////////////////////////////////////////////////////////
  # IsdStationInfo can only be used with IS Precision Series tracking devices.
  # If passed to `isdSetStationConfig <#isdSetStationConfig>`_ or `isdGetStationConfig <#isdGetStationConfig>`_ with InterTrax, FALSE is returned. 
  type 
    IsdStationInfo* = object 
      id*: DWORD ## Unique number identifying a station. It is the same as that 
        ## passed to the `isdSetStationConfig` and `isdGetStationConfig`
        ## functions and can be 1 to `isdMaxStations <#isdMaxStations>`_ 
      state*: IsdBool            # `isdTrue <#isdTrue>`_ if ON, FALSE if OFF 
      compass*: IsdBool # 0 (OFF) or 2 (ON).  Setting 1 is no longer used and will
                     # have the same result as 2 (ON).
                     # Compass setting is ignored if station is configured for 
                     # Fusion Mode operation (such as an IS-900 tracker). 
      inertiaCube*: LONG # InertiaCube associated with this station (IS-600). If no InertiaCube is
                         # assigned, this number is -1. Otherwise, it is a positive number
                         # 1 to 4 
      enhancement*: DWORD     # Perceptual enhancement; levels 0, 1, or 2 
      sensitivity*: DWORD     # Sensitivity; levels 1 to 4 (only used with Enhancement = 1 or 2) 
      prediction*: DWORD      # 0 to 50 msec
      angleFormat*: DWORD     # isdEuler or isdQuaternion (only needed for IS-900, IS-1200)
      timeStamped*: IsdBool      # `isdTrue <#isdTrue>`_ if time stamp is requested 
      getInputs*: IsdBool        # `isdTrue <#isdTrue>`_ if button and joystick data is requested 
      getEncoderData*: IsdBool # `isdTrue <#isdTrue>`_ if raw encoder data is requested 
                            # This setting controls how Magnetic Environment Calibration is applied. This calibration
                            # calculates nominal field strength and dip angle for the environment in which the sensor
                            # is used. Based on these values, the system can assign a weight to compass measurements,
                            # allowing it to reject bad measurements. Values from 0 to 3 are accepted.
                            #   If CompassCompensation is set to 0, the calibration is ignored and all compass data is
                            # used. Higher values result in a tighter rejection threshold, resulting in more measurements 
                            # being rejected. If the sensor is used in an environment with significant magnetic interference
                            # this can result in drift due to insufficient compensation from the compass data. Default
                            # setting is 2.
                            #   Note that the sensor must be calibrated in the ISDemo Compass Calibration Tool for this 
                            # setting to have any effect.
                            # 
                            # -----------------------------------------------------------------------------------------
      compassCompensation*: BYTE # This setting controls how the system deals with sharp changes in IMU data that can
                                 # be caused by shock or impact. Sensors may experience momentary rotation rates or
                                 # accelerations that are outside of the specified range, resulting in undesirable 
                                 # behavior. By turning on shock suppression you can have the system filter out
                                 # corrupted data. Values 0 (OFF) to 2 are accepted, with higher values resulting in
                                 # greater filtering.
                                 # 
                                 # -----------------------------------------------------------------------------------------
      imuShockSuppression*: BYTE # This setting controls the rejection threshold for ultrasonic measurements. Currently, it is
                                 # implemented only for the IS-900 PCTracker. Default setting is 4, which results in measurements
                                 # with range errors greater than 4 times the average to be rejected. Please do not change
                                 # this setting without first consulting with InterSense technical support.
                                 # 
                                 # -----------------------------------------------------------------------------------------
      urmRejectionFactor*: BYTE
      bReserved2*: BYTE
      coordFrame*: DWORD # Coordinate frame in which position and orientation data is reported  
                         # AccelSensitivity is used for 3-DOF tracking with InertiaCube products only. It controls how 
                         # fast tilt correction, using accelerometers, is applied. Valid values are 1 to 4, with 2 as default. 
                         #    Level 1 reduces the amount of tilt correction during movement. While it will prevent any effect  
                         # linear accelerations may have on pitch and roll, it will also reduce stability and dynamic accuracy. 
                         # It should only be used in situations when sensor is not expected to experience a lot of movement.
                         #    Level 2 (default) is best for head tracking in static environment, with user seated. 
                         #    Level 3 allows for more aggressive tilt compensation, appropriate when sensor is moved a lot, 
                         # for example, when the user is walking for long periods of time. 
                         #    Level 4 allows for even greater tilt corrections. It will reduce orientation accuracy by 
                         # allowing linear accelerations to effect orientation, but increase stability. This level 
                         # is appropriate for when the user is running, or in other situations where the sensor experiences 
                         # a great deal of movement. 
                         # 
                         # -----------------------------------------------------------------------------------------
      accelSensitivity*: DWORD
      fReserved1*: cfloat
      fReserved2*: cfloat
      tipOffset*: array[3, cfloat] # Coordinates in station frame (relative to tracker origin) of the point being tracked
      fReserved3*: cfloat
      getCameraData*: IsdBool    # `isdTrue <#isdTrue>`_ to get computed FOV, aperture, etc  
      getAuxInputs*: IsdBool
      getCovarianceData*: IsdBool
      getExtendedData*: IsdBool # Retrieving extended data will reduce update rate with even a single tracker
                             # when using serial communications; Ethernet is highly recommended when retrieving
                             # extended data
    
  #/////////////////////////////////////////////////////////////////////////////
  type 
    IsdStationData* = object 
      trackingStatus*: BYTE   # Tracking status, represents "Tracking Quality" (0-255; 0 if lost)
      newData*: BYTE          # `isdTrue <#isdTrue>`_ if data changed since last call to `isdGetTrackingData <#isdGetTrackingData>`_       
      commIntegrity*: BYTE    # Communication integrity (percentage of packets received from tracker, 0-100) 
      batteryState*: BYTE     # Wireless devices only 0=N/A, 1=Low, 2=OK
      euler*: array[3, cfloat] # Orientation in Euler angle format (Yaw, Pitch, Roll)
      Quaternion*: array[4, cfloat] # Orientation in Quaternion format (W,X,Y,Z)
      position*: array[3, cfloat] # Always in meters 
      timeStamp*: cfloat      # Timestamp in seconds, reported only if requested 
      stillTime*: cfloat      # InertiaCube and PC-Tracker products only, whether sensor is still
      batteryLevel*: cfloat   # Battery voltage, if available
      compassYaw*: cfloat # Magnetometer heading, computed based on current orientation.
                          # Available for InertiaCube products only, such as IC2, IC3 and IC2+
      buttonState*: array[isdMaxButtons, IsdBool] # Only if requested 
                                                 # 
                                                 # -----------------------------------------------------------------------------------------
                                                 # Current hardware is limited to 10 channels, with only 2 being used. 
                                                 # The only device using this is the IS-900 wand that has a built-in
                                                 # analog joystick. Channel 1 is X-axis rotation, channel 2 is Y-axis
                                                 # rotation 
      analogData*: array[isdMaxChannels, cshort] # Only if requested 
      auxInputs*: array[isdMaxAuxInputs, BYTE]
      angularVelBodyFrame*: array[3, cfloat] # rad/sec, in sensor body coordinate frame. 
                                             # Reported as rates about X, Y and Z axes, corresponding
                                             # to Roll, Pitch, Yaw order.
                                             # This is the processed angular rate, with current biases 
                                             # removed. This is the angular rate used to produce 
                                             # orientation updates.
      angularVelNavFrame*: array[3, cfloat] # rad/sec, in world coordinate frame, with boresight and other
                                            # transformations applied. 
                                            # Reported as rates about X, Y and Z axes, corresponding
                                            # to Roll, Pitch, Yaw order.
      accelBodyFrame*: array[3, cfloat] # meter/sec^2, in sensor body coordinate frame. These are 
                                        # the accelerometer measurements in the sensor body coordinate 
                                        # frame. Only factory calibration is applied to this data, 
                                        # gravity component is not removed.    
                                        # Reported as accelerations along X, Y and Z axes.
      accelNavFrame*: array[3, cfloat] # meters/sec^2, in the navigation (earth) coordinate frame. 
                                       # This is the accelerometer measurements with calibration,  
                                       # current sensor orientation applied, and gravity  
                                       # subtracted. This is the best available estimate of
                                       # tracker acceleration.
                                       # Reported as accelerations along X, Y and Z axes.
      velocityNavFrame*: array[3, cfloat] # meters/sec, 6-DOF systems only.  
                                          # Reported as velocity along X, Y and Z axes.
      angularVelRaw*: array[3, cfloat] # Raw gyro output, only factory calibration is applied. 
                                       # Some errors due to temperature dependant gyro bias 
                                       # drift will remain.
      measQuality*: BYTE      # Ultrasonic Measurement Quality (IS-900 only, firmware >= 4.26)
      bReserved2*: BYTE
      bReserved3*: BYTE
      bReserved4*: BYTE
      timeStampSeconds*: DWORD # Time Stamp in whole seconds.
      timeStampMicroSec*: DWORD # Fractional part of the Time Stamp in micro-seconds.
      oSTimeStampSeconds*: DWORD # Data record arrival time stamp based on OS time,
      oSTimeStampMicroSec*: DWORD # reserved for future use, not implemented.
      reserved*: array[55, cfloat]
      temperature*: cfloat    # Station temperature in degrees C (3DOF sensors only)
      magBodyFrame*: array[3, cfloat] # 3DOF sensors only
                                      # Magnetometer data along the X, Y, and Z axes
                                      # Units are nominally in Gauss, and factory calibration 
                                      # is applied.  Note, however, that most sensors are not 
                                      # calibrated precisely since the exact field strength is 
                                      # not necessary to for tracking purposes.  Relative 
                                      # magnitudes should be accurate, however.  Fixed metal 
                                      # compass calibration may rescale the values, as well       
    
  #/////////////////////////////////////////////////////////////////////////////


  type 
    IsdCameraEncoderData* = object 
      trackingStatus*: BYTE   # Tracking status byte 
      bReserved1*: BYTE       # Pack to 4 byte boundary 
      bReserved2*: BYTE
      bReserved3*: BYTE
      timecode*: DWORD        # Timecode, not implemented yet 
      apertureEncoder*: LONG  # Aperture encoder counts, relative to last reset or power up 
      focusEncoder*: LONG     # Focus encoder counts 
      zoomEncoder*: LONG      # Zoom encoded counts 
      timecodeUserBits*: DWORD # Time code user bits, not implemented yet 
      aperture*: cfloat       # Computed aperture value 
      focus*: cfloat          # Computed focus value (mm), not implemented yet 
      fOV*: cfloat            # Computed vertical FOV value (degrees) 
      nodalPoint*: cfloat     # Nodal point offset due to zoom and focus (mm) 
      covarianceOrientation*: array[3, cfloat] # Available only for IS-1200
      covariancePosition*: array[3, cfloat]
      dwReserved1*: DWORD
      dwReserved2*: DWORD
      fReserved1*: cfloat
      fReserved2*: cfloat
      fReserved3*: cfloat
      fReserved4*: cfloat

    IsdTrackingData* = object 
      station*: array[isdMaxStations, IsdStationData]

    IsdCameraData* = object 
      camera*: array[isdMaxStations, IsdCameraEncoderData]

    IsdAuxSystemType* {.size: sizeof(cint).} = enum 
      none = 0,
      ultrasonic, 
      optical,
      magnetic,
      rf, 
      gps

  type 
    IsdHardwareCapability* = object 
      position*: IsdBool         # Can track position
      orientation*: IsdBool      # Can track orientation
      encoders*: IsdBool         # Can support lens encoders
      prediction*: IsdBool       # Predictive algorithms are available
      enhancement*: IsdBool      # Enhancement level can be changed
      compass*: IsdBool          # Compass setting can be changed
      selfTest*: IsdBool         # Has the self-test capability
      errorLog*: IsdBool         # Can keep error log
      ultVolume*: IsdBool        # Can control ultrasonic volume via software
      ultGain*: IsdBool          # Can control microphone sensitivity by software
      ultTimeout*: IsdBool       # Can change ultrasonic sampling frequency
      photoDiode*: IsdBool       # SoniDiscs support photodiode
      maxStations*: cint ## Number of supported stations
      maxImus*: cint         # Number of supported IMUs
      maxFPses*: cint        # Maximum number of Fixed Position Sensing Elements (constellation/galaxy)
      maxChannels*: cint     # Maximum number of analog channels supported per station
      maxButtons*: cint      # Maximum number of digital button inputs per station
      measData*: IsdBool         # Can provide measurement data
      diagData*: IsdBool         # Can provide diagnostic data
      pseConfig*: IsdBool        # Supports PSE configuration/reporting tools
      configLock*: IsdBool       # Supports configuration locking     
      ultMaxRange*: cfloat    # Maximum ultrasonic range  
      fReserved2*: cfloat
      fReserved3*: cfloat
      fReserved4*: cfloat
      compassCal*: IsdBool       # Supports dynamic compass calibration     
      bReserved2*: IsdBool
      bReserved3*: IsdBool
      bReserved4*: IsdBool
      dwReserved1*: cint
      dwReserved2*: cint
      dwReserved3*: cint
      dwReserved4*: cint


    IsdHardwareInfo* = object 
      valid*: IsdBool ## Set to `isdTrue <#isdTrue>`_ if
        ## `isdGetSystemHardwareInfo <#isdGetSystemHardwareInfo>`_ succeeded
      trackerType*: IsdSystemType ## See `IsdSystemType <#IsdSystemType>`_
      trackerModel*: IsdSystemModel ## See `IsdSystemModel <#IsdSystemModel>`_
      port*: cint ## Hardware port number (1 for COM1/ttyS0, etc.)
      iftype*: DWORD ## Hardware interface (RS232, USB, etc.)
      onHost*: IsdBool ## `isdTrue <#isdTrue>`_ if tracking algorithms are executed in the library
      auxSystem*: DWORD ## Position tracking hardware
        ## (see `IsdAuxSystemType <#IsdAuxSystemType>`_)
      firmwareRev*: cfloat ## Firmware revision 
      modelName*: array[128, char]
      capability*: IsdHardwareCapability
      bReserved1*: IsdBool
      bReserved2*: IsdBool
      bReserved3*: IsdBool
      bReserved4*: IsdBool
      baudRate*: DWORD ## Serial port baud rate      
      numTestLevels*: DWORD ## Number of self test levels       
      dwReserved3*: DWORD
      dwReserved4*: DWORD
      fReserved1*: cfloat
      fReserved2*: cfloat
      fReserved3*: cfloat
      fReserved4*: cfloat
      cReserved1*: array[128, char]
      cReserved2*: array[128, char]
      cReserved3*: array[128, char]
      cReserved4*: array[128, char]

  
  type 
    IsdStationCapability* = object
      ## Station hardware information. This structure provides detailed
      ## information on station hardware and its capabilities.
      position*: IsdBool ## `isdTrue <#isdTrue>`_ if station can track position
      orientation*: IsdBool ## `isdTrue <#isdTrue>`_ if station can track orientation
      encoders*: cint ## Number of lens encoders, if 0 then none are available
      numChannels*: cint ## Number of analog channels supported by this station,
        ## wand has 2 (joystick axes)
      numButtons*: cint ## Number of digital button inputs supported by this
        ## station
      auxInputs*: cint ## Number of auxiliary input channels (OEM products)
      auxOutputs*: cint ## Number of auxiliary output channels (OEM products)
      compass*: IsdBool ## `isdTrue <#isdTrue>`_ if station has a compass
      bReserved1*: IsdBool
      bReserved2*: IsdBool
      bReserved3*: IsdBool
      bReserved4*: IsdBool
      dwReserved1*: cint
      dwReserved2*: cint
      dwReserved3*: cint
      dwReserved4*: cint

type 
  IsdStationHardwareInfo* = object 
    Valid*: IsdBool ## Set to `isdTrue <#isdTrue>`_ if
      ## `isdGetStationHardwareInfo <#isdGetStationHardwareInfo>`_ succeeded
    id*: DWORD # Unique number identifying a station. It is the same as that 
               # passed to the `isdSetStationConfig` and `isdGetStationConfig`   
               # functions and can be 1 to `isdMaxStations <#isdMaxStations>`_ 
    descVersion*: array[20, char] # Station Descriptor version 
    firmwareRev*: cfloat    # Station firmware revision
    serialNum*: DWORD       # Station serial number 
    calDate*: array[20, char] # Last factory calibration date (mm/dd/yyyy)
    port*: DWORD            # Hardware port number 
    capability*: IsdStationCapability
    bReserved1*: IsdBool
    bReserved2*: IsdBool
    bReserved3*: IsdBool
    bReserved4*: IsdBool
    stationType*: DWORD # Station type        
    deviceId*: DWORD
    dwReserved3*: DWORD
    dwReserved4*: DWORD
    fReserved1*: cfloat
    fReserved2*: cfloat
    fReserved3*: cfloat
    fReserved4*: cfloat
    cReserved1*: array[128, char]
    cReserved2*: array[128, char]
    cReserved3*: array[128, char]
    cReserved4*: array[128, char]

  IsdPortWirelessInfo* = object 
    valid*: IsdBool
    status*: LONG
    wireless*: IsdBool
    channel*: DWORD
    id*: array[4, DWORD]
    radioVersion*: DWORD
    dReserved1*: DWORD
    dReserved2*: DWORD
    dReserved3*: DWORD
    dReserved4*: DWORD


proc isdOpenTracker*(hParent: Hwnd; commPort: DWORD; infoScreen: IsdBool; 
  verbose: IsdBool): IsdTrackerHandle
  {.cdecl, dynlib: dllname, importc: "ISD_OpenTracker".}
  ## Open a single tracker.
  ##
  ## hParent
  ##   Handle to the parent window, or ``nil``. This parameter is optional and
  ##   should only be used if information screen or tracker configuration tools
  ##   are to be used when available in the future releases.
  ## commPort
  ##   If this parameter is a number other than ``0``, program will try to
  ##   locate an InterSense tracker on the specified RS232 port. Otherwise it
  ##   looks for USB device, then for serial port device on all ports at all
  ##   baud rates. Most applications should pass 0 for maximum flexibility. If
  ##   you have more than one InterSense device and would like to have a
  ##   specific tracker, connected to a known port, initialized first, then
  ##   enter the port number instead of ``0``.
  ## infoScreen
  ##   This feature has not been implemented. Its purpose is to display an
  ##   information window to show the tracker detection progress and results.
  ##   Currently, the DLL writes only to the console. Most applications should
  ##   pass `isdFalse <#isdFalse>`_.
  ## verbose
  ##    Pass `isdTrue <#isdTrue>`_ if you would like a more detailed report of
  ##    the DLL activity. Messages are printed to the console
  ## result
  ##   - Handle of the opened tracker on success
  ##   - ``-1`` on failure
  ##
  ## This function is used for opening a single tracker. It may be called
  ## multiple times in order to open multiple trackers, though typically using
  ## `isdOpenAllTrackers <#isdOpenAllTrackers>`_ is recommended instead for
  ## use with multiple trackers.
  ##
  ## Note that wireless 3DOF sensors such as the wireless IC3 are considered
  ## trackers, not stations (even if using a shared receiver), so each tracker
  ## must have a separate handle associated with it. A station is a concept
  ## associated with the IS-900 and IS-1200 systems.


## TODO: convert handle to array
proc isdOpenAllTrackers*(hParent: Hwnd; handle: ptr IsdTrackerHandle; 
  infoScreen: IsdBool; verbose: IsdBool): cint
  {.cdecl, dynlib: dllname, importc: "ISD_OpenAllTrackers".}
  ## Open multiple trackers.
  ##
  ## handle
  ##   Will contain the handles for all detected trackers
  ## infoScreen
  ##   This feature has not been implemented. Its purpose is to display an
  ##   information window to show the tracker detection progress and results.
  ##   Currently, the DLL writes only to the console. Most applications should
  ##   pass `isdFalse <#isdFalse>`_.
  ## verbose
  ##    Pass `isdTrue <#isdTrue>`_ if you would like a more detailed report of
  ##    the DLL activity. Messages are printed to the console
  ## result
  ##   - Number of detected trackers on success
  ##   - ``-1`` on failure
  ##
  ## This is the recommended method for opening multiple trackers.


proc isdCloseTracker*(handle: IsdTrackerHandle): IsdBool
  {.cdecl, dynlib: dllname, importc: "ISD_CloseTracker".}
  ## Close a tracker.
  ##
  ## handle
  ##   Handle to the tracking device. This is a handle returned by
  ##   `isdOpenTracker <#isdOpenTracker>`_ or
  ##   `isdOpenAllTrackers <#isdOpenAllTrackers>`_.
  ## result
  ##   - `isdTrue <#isdTrue>`_ on success
  ##   - `isdFalse <#isdFalse>`_ otherwise
  ##
  ## This function call uninitializes a tracker, closes communications port and 
  ## frees the resources associated with the tracker handle. If ``0`` is passed,
  ## all currently open trackers are closed. When the last tracker is closed,
  ## program frees the library.


proc isdGetTrackerConfig*(handle: IsdTrackerHandle; 
  tracker: ptr IsdTrackerInfo; verbose: IsdBool): IsdBool
  {.cdecl, dynlib: dllname, importc: "ISD_GetTrackerConfig".}
  ## Get general tracker information, such as type, model, port, etc.
  ##
  ## handle
  ##   Handle to the tracking device. This is a handle returned by
  ##   `isdOpenTracker <#isdOpenTracker>`_ or
  ##   `isdOpenAllTrackers <#isdOpenAllTrackers>`_.
  ## tracker
  ##   Pointer to an `IsdTrackerInfo <#IsdTrackerInfo>`_ object.
  ## result
  ##   - `isdTrue <#isdTrue>`_ on success
  ##   - `isdFalse <#isdFalse>`_ otherwise
  ##
  ## Also retrieves genlock synchronization configuration, if available. See the
  ## `IsdTrackerInfo <#IsdTrackerInfo>`_ structure definition for
  ## a complete list

proc isdSetTrackerConfig*(handle: IsdTrackerHandle;
  tracker: ptr IsdTrackerInfo; verbose: IsdBool): IsdBool
  {.cdecl, dynlib: dllname, importc: "ISD_SetTrackerConfig".}
  ## Set tracker configuration.
  ##
  ## handle
  ##   Handle to the tracking device. This is a handle returned by
  ##   `isdOpenTracker <#isdOpenTracker>`_ or
  ##   `isdOpenAllTrackers <#isdOpenAllTrackers>`_.
  ## tracker
  ##   Pointer to an `IsdTrackerInfo <#IsdTrackerInfo>`_ object.
  ## result
  ##   - `isdTrue <#isdTrue>`_ on success
  ##   - `isdFalse <#isdFalse>`_ otherwise
  ##
  ## When used with IS Precision Series (IS-300, IS-600, IS-900, IS-1200)
  ## tracking devices, this function call will set genlock synchronization
  ## parameters; all other fields in the `IsdTrackerInfo <#IsdTrackerInfo>`_
  ## structure are for information purposes only.


proc isdGetCommInfo*(handle: IsdTrackerHandle; tracker: ptr IsdTrackerInfo):
  IsdBool {.cdecl, dynlib: dllname, importc: "ISD_GetCommInfo".}
  ## Get communication data rate information.
  ##
  ## handle
  ##   Handle to the tracking device. This is a handle returned by
  ##   `isdOpenTracker <#isdOpenTracker>`_ or
  ##   `isdOpenAllTrackers <#isdOpenAllTrackers>`_.
  ## tracker
  ##   Pointer to an `IsdTrackerInfo <#IsdTrackerInfo>`_ object.
  ## result
  ##   - `isdTrue <#isdTrue>`_ on success
  ##   - `isdFalse <#isdFalse>`_ otherwise
  ##
  ## This function reads `recordsPerSec` and `kBitsPerSec` without requesting
  ## genlock settings from the tracker. Use this instead of
  ## `isdGetTrackerConfig <#isdGetTrackerConfig>`_ to prevent your program
  ## from stalling while waiting for the tracker response. 


proc isdSetStationConfig*(handle: IsdTrackerHandle;
  station: ptr IsdStationInfo; stationId: WORD; verbose: IsdBool): IsdBool
  {.cdecl, dynlib: dllname, importc: "ISD_SetStationConfig".}
  ## Set station configuration settings.
  ## 
  ## handle
  ##   Handle to the tracking device. This is a handle returned by
  ##   `isdOpenTracker <#isdOpenTracker>`_ or
  ##   `isdOpenAllTrackers <#isdOpenAllTrackers>`_.
  ## station
  ##   Pointer to a structure of type `IsdStationInfo <#IsdStationInfo>`_.
  ##   The structure definition is given below.
  ## stationId
  ##   Number from `1` to `isdMaxStations <#isdMaxStations>`_
  ## result
  ##   - `isdTrue <#isdTrue>`_ on success
  ##   - `isdFalse <#isdFalse>`_ otherwise
  ##
  ## Before this function is called, all elements of the `station` structure
  ## must be assigned a value. This function should only be used with IS
  ## Precision Series tracking devices; not valid for InterTrax.

proc isdGetStationConfig*(handle: IsdTrackerHandle;
  station: ptr IsdStationInfo; stationId: WORD; verbose: IsdBool): IsdBool
  {.cdecl, dynlib: dllname, importc: "ISD_GetStationConfig".}
  ## Get station configuration settings.
  ##
  ## handle
  ##   Handle to the tracking device. This is a handle returned by
  ##   `isdOpenTracker <#isdOpenTracker>`_ or
  ##   `isdOpenAllTrackers <#isdOpenAllTrackers>`_.
  ## station
  ##   Pointer to a structure of type `IsdStationInfo <#IsdStationInfo>`_.
  ##   The structure definition is given below.
  ## stationId
  ##   Number from `1` to `isdMaxStations <#isdMaxStations>`_
  ## result
  ##   - `isdTrue <#isdTrue>`_ on success
  ##   - `isdFalse <#isdFalse>`_ otherwise
  ##
  ## Fills the `IsdStationInfo <#IsdStationInfo>`_ structure with current
  ## settings. Function requests configuration records from the tracker and
  ## waits for the response. If communications are interrupted, it will stall
  ## for several seconds while attempting to recover the settings. Should only
  ## be used with IS Precision Series tracking devices, not valid for InterTrax.


proc isdConfigureFromFile*(handle: IsdTrackerHandle; path: cstring;
  verbose: IsdBool): IsdBool
  {.cdecl, dynlib: dllname, importc: "ISD_ConfigureFromFile".}
  ## Configure a tracker from a configuration file.
  ##
  ## handle
  ##   Handle to the tracking device. This is a handle returned by
  ##   `isdOpenTracker <#isdOpenTracker>`_ or
  ##   `isdOpenAllTrackers <#isdOpenAllTrackers>`_.
  ## path
  ##   Pointer to a string representing the complete path to the file to load.
  ## result
  ##   - `isdTrue <#isdTrue>`_ on success
  ##   - `isdFalse <#isdFalse>`_ otherwise
  ## 
  ## When a tracker is first opened, library automatically looks for a
  ## configuration file in current directory of the application. File name
  ## convention is `isenseX.ini` where `X` is a number, starting at ``1``,
  ## identifying the first tracking system in the order of initialization. This
  ## function provides for a way to manually configure the tracker using an
  ## arbitrary configuration file instead.

proc isdConfigSave*(handle: IsdTrackerHandle): IsdBool
  {.cdecl, dynlib: dllname, importc: "ISD_ConfigSave".}
  ## Save tracker configuration.
  ##
  ## handle
  ##   Handle to the tracking device. This is a handle returned by
  ##   `isdOpenTracker <#isdOpenTracker>`_ or
  ##   `isdOpenAllTrackers <#isdOpenAllTrackers>`_.
  ##
  ## For devices with on-host processing, like the IS-900 PCTracker, this will
  ## write to the isenseX.cfg file. Serial port devices like IS-300, IS-600 and
  ## IS-900 save configuration in the base unit, and this call will just send a
  ## command to commit the changes to permanent storage.


# Get data from all configured stations. Data is placed in the IsdTrackingData
# structure. 
# ----------------------------------------------------------------------------
proc isdGetTrackingData*(handle: IsdTrackerHandle; Data: ptr IsdTrackingData):
  IsdBool {.cdecl, dynlib: dllname, importc: "ISD_GetTrackingData".}
  ## result
  ##   - `isdTrue <#isdTrue>`_ on success
  ##   - `isdFalse <#isdFalse>`_ otherwise


# Get data from all configured stations. Data is placed in the IsdTrackingData
# structure. 
# ----------------------------------------------------------------------------
proc isdGetTrackingDataAtTime*(handle: IsdTrackerHandle; atTime: cdouble;
  maxSyncWait: cdouble): IsdBool
  {.cdecl, dynlib: dllname, importc: "ISD_GetTrackingDataAtTime".}
  ## result
  ##   - `isdTrue <#isdTrue>`_ on success
  ##   - `isdFalse <#isdFalse>`_ otherwise
 
# ----------------------------------------------------------------------------
proc isdGetCameraData*(handle: IsdTrackerHandle; data: ptr IsdCameraData):
  IsdBool {.cdecl, dynlib: dllname, importc: "ISD_GetCameraData".}
  ## Get camera encode and other data for all configured stations.
  ##
  ## handle
  ##   Handle to the tracking device. This is a handle returned by
  ##   `isdOpenTracker <#isdOpenTracker>`_ or
  ##   `isdOpenAllTrackers <#isdOpenAllTrackers>`_.
  ## data
  ##   Pointer to a structure of type `IsdTrackerData <#IsdTrackerData>`_.
  ##   See below for structure definition. Orientation data order is Yaw, Pitch,
  ##   and Roll for Euler angles and W, X, Y, Z for quaternions.
  ## result
  ##   - `isdTrue <#isdTrue>`_ on success
  ##   - `isdFalse <#isdFalse>`_ otherwise
  ##
  ## Data is placed in the `IsdCameraData <#IsdCameraData>`_ structure. This
  ## function does not service the serial port, so
  ## `isdGetTrackingData <#isdGetTrackingData>`_ must be called prior to this. 


proc ISD_RingBufferSetup*(handle: IsdTrackerHandle; stationId: WORD; 
  dataBuffer: ptr IsdStationData; samples: cint): IsdBool
  {.cdecl, dynlib: dllname, importc: "ISD_RingBufferSetup".}
  ## Set up the ring buffer.
  ##
  ## handle
  ##   Handle to the tracking device. This is a handle returned by
  ##   `isdOpenTracker <#isdOpenTracker>`_ or
  ##   `isdOpenAllTrackers <#isdOpenAllTrackers>`_.
  ## stationId
  ##   Number from `1` to `isdMaxStations <#isdMaxStations>`_
  ## dataBuffer
  ##   An array of `IsdStationData <#IsdStationData>`_ structures. Pass in
  ##   ``nil`` if you do not need visibility into the complete buffer (typical).
  ## samples
  ##   The size of the ring buffer.
  ##   `isdGetTrackingData <#isdGetTrackingData>`_ should be called frequently
  ##   enough to avoid buffer overrun.
  ## result
  ##   - `isdTrue <#isdTrue>`_ on success
  ##   - `isdFalse <#isdFalse>`_ otherwise
  ##
  ## By default, `isdGetTrackingData <#isdGetTrackingData>`_ processes all
  ## records available from the tracker and only returns the latest data. As the
  ## result, data samples can be lost. If all the data samples are required, you
  ## can use a ring buffer to store them.
  ##
  ## `isdRingBufferSetup <#isdRingBufferSetup>`_ accepts a pointer to the ring
  ## buffer, and its size. Once activated, all processed data samples are stored
  ## in the buffer for use by the application. 
  ##
  ## isdGetTrackingData can still be used to read the data, but will return the
  ## oldest saved data sample, then remove it from the buffer (first in - first
  ## out). By repeatedly calling `isdGetTrackingData <#isdGetTrackingData>`_,
  ## all samples are retrieved, the latest coming last. All consecutive calls to
  ## `isdGetTrackingData <#isdGetTrackingData>`_ will return the last sample,
  ## but the `NewData` flag will be `isdFalse <#isdFalse>`_ to indicate that the
  ## buffer has been emptied.


proc isdRingBufferStart*(handle: IsdTrackerHandle; stationId: WORD): IsdBool
  {.cdecl, dynlib: dllname, importc: "ISD_RingBufferStart".}
  ## Activate the ring buffer.
  ##
  ## handle
  ##   Handle to the tracking device. This is a handle returned by
  ##   `isdOpenTracker <#isdOpenTracker>`_ or
  ##   `isdOpenAllTrackers <#isdOpenAllTrackers>`_.
  ## stationId
  ##   Number from `1` to `isdMaxStations <#isdMaxStations>`_
  ## result
  ##   - `isdTrue <#isdTrue>`_ on success
  ##   - `isdFalse <#isdFalse>`_ otherwise
  ##
  ## While active, all data samples are stored in the buffer. Because this is a
  ## ring buffer, it will only store the number of samples specified in the call
  ## to `isdRingBufferSetup <#isdRingBufferSetup>`_, so the oldest samples can
  ## be overwritten.


proc isdRingBufferStop*(handle: IsdTrackerHandle; stationId: WORD): IsdBool
  {.cdecl, dynlib: dllname, importc: "ISD_RingBufferStop".}
  ## Stop collection.
  ##
  ## handle
  ##   Handle to the tracking device. This is a handle returned by
  ##   `isdOpenTracker <#isdOpenTracker>`_ or
  ##   `isdOpenAllTrackers <#isdOpenAllTrackers>`_.
  ## stationId
  ##   Number from `1` to `isdMaxStations <#isdMaxStations>`_
  ## result
  ##   - `isdTrue <#isdTrue>`_ on success
  ##   - `isdFalse <#isdFalse>`_ otherwise
  ##
  ## The library will continue to process data, but the contents of the ring
  ## buffer will not be altered.


proc isdRingBufferQuery*(handle: IsdTrackerHandle; stationId: WORD; 
  currentData: ptr IsdStationData; head: ptr DWORD; tail: ptr DWORD): IsdBool
  {.cdecl, dynlib: dllname, importc: "ISD_RingBufferQuery".}
  ## Queries the library for the latest data without removing it from the buffer
  ## or affecting the NewData flag.
  ##
  ## handle
  ##   Handle to the tracking device. This is a handle returned by
  ##   `isdOpenTracker <#isdOpenTracker>`_ or
  ##   `isdOpenAllTrackers <#isdOpenAllTrackers>`_.
  ## stationId
  ##   Number from `1` to `isdMaxStations <#isdMaxStations>`_
  ## currentData
  ##   An array of `IsdStationData <#IsdStationData>`_ used as the buffer.
  ## head
  ##    Pointer to the current head of the ring buffer.
  ## tail
  ##    Pointer to the current tail of the ring buffer.
  ##
  ## This function also returns the indexes of the newest and the oldest samples
  ## in the buffer. These can then be used to parse the buffer.

proc isdResetHeading*(handle: IsdTrackerHandle; stationId: WORD): IsdBool
  {.cdecl, dynlib: dllname, importc: "ISD_ResetHeading".}
  ## Reset heading (yaw) to zero.
  ##
  ## handle
  ##   Handle to the tracking device. This is a handle returned by
  ##   `isdOpenTracker <#isdOpenTracker>`_ or
  ##   `isdOpenAllTrackers <#isdOpenAllTrackers>`_.
  ## stationId
  ##   Number from `1` to `isdMaxStations <#isdMaxStations>`_
  ## result
  ##   - `isdTrue <#isdTrue>`_ on success
  ##   - `isdFalse <#isdFalse>`_ otherwise


proc isdBoresightReferenced*(handle: IsdTrackerHandle; stationId: WORD; 
  yaw: cfloat; pitch: cfloat; roll: cfloat): IsdBool
  {.cdecl, dynlib: dllname, importc: "ISD_BoresightReferenced".}
  ## Boresight station using specific reference angles.
  ##
  ## handle
  ##   Handle to the tracking device. This is a handle returned by
  ##   `isdOpenTracker <#isdOpenTracker>`_ or
  ##   `isdOpenAllTrackers <#isdOpenAllTrackers>`_.
  ## stationId
  ##   Number from `1` to `isdMaxStations <#isdMaxStations>`_
  ## yaw
  ##   Yaw reference angle
  ## pitch
  ##   Pitch reference angle
  ## roll
  ##   Roll reference angle
  ## result
  ##   - `isdTrue <#isdTrue>`_ on success
  ##   - `isdFalse <#isdFalse>`_ otherwise
  ##
  ## This is useful when you need to apply a specific offset to system output.
  ## For example, if a sensor is mounted at 40 degrees pitch relative to an HMD,
  ## you can pass in `0, 40, 0` to make the system output `(0,0,0)` when the HMD
  ## is horizontal.
  ##
  ## This function works with all IS-X00 series products, InterTraxLC and
  ## InertiaCube products. For InterTrax30 and InterTrax2, it behaves like
  ## `isdResetHeading <#isdResetHeading>`_.

proc isdBoresight*(handle: IsdTrackerHandle; stationId: WORD; set: IsdBool):
  IsdBool {.cdecl, dynlib: dllname, importc: "ISD_Boresight".}
  ## Boresight, or unboresight a station.
  ##
  ## handle
  ##   Handle to the tracking device. This is a handle returned by
  ##   `isdOpenTracker <#isdOpenTracker>`_ or
  ##   `isdOpenAllTrackers <#isdOpenAllTrackers>`_.
  ## stationId
  ##   Number from `1` to `isdMaxStations <#isdMaxStations>`_
  ## set
  ##   Whether to reset all angles to zero
  ## result
  ##   - `isdTrue <#isdTrue>`_ on success
  ##   - `isdFalse <#isdFalse>`_ otherwise
  ##
  ## If `set` is `isdTrue <#isdTrue>`_, all angles are reset to zero. Otherwise,
  ## all boresight settings are cleared, including those set by
  ## `isdResetHeading <#isdResetHeading>`_ and
  ## `isdBoresightReferenced <#isdBoresightReferenced>`_.
  ##
  ## Note that the angles are reset relative to the current yaw; if the station
  ## is at 90 degrees yaw and 0 degrees pitch/roll when this function is called, 
  ## rolling the sensor (relative to its current heading) will be considered
  ## pitch, and pitch (relative to its current heading) will be considered roll;
  ## it does not perform a boresight 'relative' to the current orientation
  ## vector.
  ##
  ## This function works with all IS-X00 series products, InterTraxLC and
  ## InertiaCube products. For InterTrax30 and InterTrax2, it behaves like
  ## `isdResetHeading <#isdResetHeading>`_.


proc isdSendScript*(handle: IsdTrackerHandle; script: cstring): IsdBool
  {.cdecl, dynlib: dllname, importc: "ISD_SendScript".}
  ## Send a configuration script to the tracker.
  ##
  ## handle
  ##   Handle to the tracking device. This is a handle returned by
  ##   `isdOpenTracker <#isdOpenTracker>`_ or
  ##   `isdOpenAllTrackers <#isdOpenAllTrackers>`_.
  ## script
  ##   Pointer to a string containing the command script
  ## result
  ##   - `isdTrue <#isdTrue>`_ on success
  ##   - `isdFalse <#isdFalse>`_ otherwise
  ##
  ## The script must consist of valid commands as described in the interface
  ## protocol. Commands in the script should be terminated by the newline
  ## character '\n'. The linefeed character '\r' is added by the function, and
  ## is not required.
  ##
  ## Note that this may not be supported when using the shared memory interface,
  ## such as with sfServer, and is primarily intended for the
  ## IS-300/IS-600/IS-900 system.


proc isdAuxOutput*(handle: IsdTrackerHandle; stationId: WORD; 
  AuxOutput: ptr BYTE; length: WORD): IsdBool
  {.cdecl, dynlib: dllname, importc: "ISD_AuxOutput".}
  ## Sends up to 4 output bytes to the auxiliary interface of the station  
  ## specified. 
  ##
  ## handle
  ##   Handle to the tracking device. This is a handle returned by
  ##   `isdOpenTracker <#isdOpenTracker>`_ or
  ##   `isdOpenAllTrackers <#isdOpenAllTrackers>`_.
  ## stationId
  ##   Number from `1` to `isdMaxStations <#isdMaxStations>`_
  ## auxOutput
  ##   An array of bytes to send
  ## length
  ##   Number of bytes to send
  ## result
  ##   - `isdTrue <#isdTrue>`_ on success
  ##   - `isdFalse <#isdFalse>`_ otherwise
  ##
  ## The number of bytes should match the number the auxiliary outputs the
  ## interface is configured to expect. If too many are specified, extra bytes
  ## are ignored. 


proc isdNumOpenTrackers*(num: ptr WORD): IsdBool
  {.cdecl, dynlib: dllname, importc: "ISD_NumOpenTrackers".}
  ## Get the number of currently opened trackers.
  ##
  ## num
  ##   Will hold the number of trackers
  ## result
  ##   - `isdTrue <#isdTrue>`_ on success
  ##   - `isdFalse <#isdFalse>`_ otherwise


proc isdGetTime*(): cfloat {.cdecl, dynlib: dllname, importc: "ISD_GetTime".}
  ## Platform independent time function.
  ##
  ## result
  ##   Time value


proc isdUdpDataBroadcast*(handle: IsdTrackerHandle; port: DWORD; 
  trackingData: ptr IsdTrackingData; cameraData: ptr IsdCameraData): IsdBool
  {.cdecl, dynlib: dllname, importc: "ISD_UdpDataBroadcast".}
  ## Broadcast tracker data over the network using UDP broadcast.
  ##
  ## handle
  ##   Handle to the tracking device. This is a handle returned by
  ##   `isdOpenTracker <#isdOpenTracker>`_ or
  ##   `isdOpenAllTrackers <#isdOpenAllTrackers>`_.
  ## port
  ##   UDP port (0 to 65535)
  ## trackingData
  ##   An `IsdTrackingData <#IsdTrackingData>`_ structure containing the data to
  ##   send, retrieved with`isdGetTrackingData <#isdGetTrackingData>`_.
  ## cameraData
  ##   Pass ``nil`` to this
  ## result
  ##   - `isdTrue <#isdTrue>`_ on success
  ##   - `isdFalse <#isdFalse>`_ otherwise


proc isdGetSystemHardwareInfo*(handle: IsdTrackerHandle; 
  hwInfo: ptr IsdHardwareInfo): IsdBool
  {.cdecl, dynlib: dllname, importc: "ISD_GetSystemHardwareInfo".}
  ## Retrieve system hardware information.
  ##
  ## handle
  ##   Handle to the tracking device. This is a handle returned by
  ##   `isdOpenTracker <#isdOpenTracker>`_ or
  ##   `isdOpenAllTrackers <#isdOpenAllTrackers>`_.
  ## hwInfo
  ##   An `IsdHardwareInfo <#IsdHardwareInfo>`_ structure
  ##   containing the information.
  ## result
  ##   - `isdTrue <#isdTrue>`_ on success
  ##   - `isdFalse <#isdFalse>`_ otherwise
  ##
  ## Note that the system is a single tracker (and will thus have one handle).
  ## For details on individual stations (such as the devices on each port of an
  ## IS-900), use `isdGetStationHardwareInfo <#isdGetStationHardwareInfo>`_
  ## instead.


proc isdGetStationHardwareInfo*(handle: IsdTrackerHandle;
  info: ptr IsdStationHardwareInfo; stationId: WORD): IsdBool
  {.cdecl, dynlib: dllname, importc: "ISD_GetStationHardwareInfo".}
  ## Retrieve station hardware information.
  ##
  ## handle
  ##   Handle to the tracking device. This is a handle returned by
  ##   `isdOpenTracker <#isdOpenTracker>`_ or
  ##   `isdOpenAllTrackers <#isdOpenAllTrackers>`_.
  ## info
  ##   An `IsdStationHardwareInfo <#IsdStationHardwareInfo>`_ structure
  ##   containing the information.
  ## stationId
  ##   Number from `1` to `isdMaxStations <#isdMaxStations>`_
  ## result
  ##   - `isdTrue <#isdTrue>`_ on success
  ##   - `isdFalse <#isdFalse>`_ otherwise


proc isdEnterHeading*(handle: IsdTrackerHandle; stationId: WORD; 
  yaw: cfloat): IsdBool {.cdecl, dynlib: dllname, importc: "ISD_EnterHeading".}
  ## result
  ##   - `isdTrue <#isdTrue>`_ on success
  ##   - `isdFalse <#isdFalse>`_ otherwise


proc isdGetPortWirelessInfo*(handle: IsdTrackerHandle; port: WORD;
  info: ptr IsdPortWirelessInfo): IsdBool
  {.cdecl, dynlib: dllname, importc: "ISD_GetPortWirelessInfo".}
  ## Retrieve wireless configuration information.
  ##
  ## handle
  ##   Handle to the tracking device. This is a handle returned by
  ##   `isdOpenTracker <#isdOpenTracker>`_ or
  ##   `isdOpenAllTrackers <#isdOpenAllTrackers>`_.
  ## port
  ##   Station or port to get info from, starting at 0 for the first port.
  ## info
  ##   An `IsdPortWirelessInfo <#IsdPortWirelessInfo>`_ structure containing the
  ##   information.
  ## result
  ##   - `isdTrue <#isdTrue>`_ on success
  ##   - `isdFalse <#isdFalse>`_ otherwise
