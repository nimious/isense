## *io-isense* - Nim bindings for the InterSense SDK.
##
## This file is part of the `Nim I/O <http://nimio.us>`_ package collection.
## See the file LICENSE included in this distribution for licensing details.
## GitHub pull requests are encouraged. (c) 2015 Headcrash Industries LLC.

when defined(linux):
  const dllname = "libisense.so"
elif defined(macos):
  const dllname = "libisense.dylib"
elif defined(windows):
  const dllname = "isense.dll"
else:
  {.error: "io-isense does not support this platform".}


const
  islibVersion* = "4.2381"


type
  IsdHwnd* = cint
    ## Window handle.


  IsdBool* {.size: sizeof(cint).} = enum ## \
    ## Boolean return values.
    isdFalse = 0,
    isdTrue = 1


  IsdOnOff* {.size: sizeof(cint).} = enum ## \
    ## On and off states.
    isdOff = 0
    isdOn = 2


  IsdSystemType* {.pure, size: sizeof(cint).} = enum ## \
    ## Supported tracking system types.
    none = 0, ## Not found, unable to identify, or not initialized
    precisionSeries, ## InertiaCube, NavChip, IS-300, IS-600, IS-900 and IS-1200 
    intertraxSeries ## InterTrax 


  IsdSystemModel* {.pure, size: sizeof(cint).} = enum ## \
    ## Supported tracking system models.
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
    ## Supported device interface types.
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
    isdMaxTrackers* = 2 ## Maximum number of trackers
    isdMaxStations* = 4 ## Maximum number of stations
else:
  const
    isdMaxTrackers* = 32 ## Maximum number of trackers
    isdMaxStations* = 8 ## Maximum number of stations


type
  IsdAngleFormat* {.pure, size: sizeof(cint).} = enum ## \
    ## Supported orientation angle formats.
    euler = 1
    quaternion = 2


  IsdCoordFrame* {.pure, size: sizeof(cint).} = enum ## \
    ## Supported coordinate frames.
    defaultFrame = 1
    vsetFrame = 2


  IsdTrackerSyncState* {.pure, size: sizeof(cint).} = enum ## \
    ## Tracker synchronization states.
    ##
    ## Note: used for IS-X devices only.
    off = 0, ## Off (system is in free run)
    autoGenlock, ## On (hardware genlock frequency is automatically determined)
    userGenlock, ## On (hardware genlock frequency is specified by the user)
    userFrequency ## On (no hardware signal, lock to user specified frequency)


  IsdBatteryState* {.pure, size: sizeof(cuchar).} = enum ## \
    ## Battery states
    notAvailable = 0,
    voltageLow,
    voltageOk

const
  isdMaxButtons* = 8 ## Number of supported stylus buttons
  isdMaxChannels* = 10 ## Hardware is limited to 10 analog/digital input
    ## channels per station
  isdMaxAuxInputs* = 4 ## Maximum supported number of bytes for auxiliary input
    ## data
  isdMaxAuxOutputs* = 4 ## Maximum supported number of bytes for auxiliary
    ## output data


type
  IsdTrackerHandle* = cint
  IsdTrackerHandleArray* {.unchecked.} = array[isdMaxTrackers, IsdTrackerHandle]

type 
  IsdTrackerInfo* = object 
    ## Tracking device information.
    ##
    ## The first six fields are for informational purpose only. The remaining
    ## fields can be configured in the isenseX.ini configuration file.
    libVersion*: cfloat ## InterSense Library version (version of DLL or shared
      ## library)
    trackerType*: IsdSystemType ## The type of tracking system
    trackerModel*: IsdSystemModel ## The tracking device model
    port*: cint ## Number of the RS232 port that the tracker is connected to
      ## (starts with 1 for COM1/ttyS0)
    recordsPerSec*: cuint ## Data records per second from tracker
    kBitsPerSec*: cfloat ## kB per second of data from tracker
    syncState*: IsdTrackerSyncState ## Synchronization state
    syncRate*: cfloat ## Number of hardware sync signals per second, or, if
      ## `SyncState` is 3, data record output frequency 
    syncPhase*: cint ## The time within the sync period at which a data record
      ## is transmitted. The phase point is specified as a percentage of the
      ## sync period. 0% (the default) instructs the tracker to output a data
      ## record as soon as possible after the sync period begins. 100% delays
      ## the output of a record as much as possible before the next sync period
      ## begins.
    Interface*: IsdInterfaceType ## Hardware interface type
    ultTimeout*: cint ## Ultrasonic timeout (sampling rate); IS-900 only
    ultVolume*: cint ## Ultrasonic speaker volume; IS-900 only
    dwReserved4*: cint ## Reserved for future use
    firmwareRev*: cfloat ## Firmware revision fopr tracker
    fReserved2*: cfloat ## Reserved for future use
    fReserved3*: cfloat ## Reserved for future use
    fReserved4*: cfloat ## Reserved for future use
    ledEnable*: IsdBool ## Enables flashing blue LEDs on SoniStrips/SoniDiscs if
      ## `isdTrue <#IsdBool>`_; IS-900 only
    bReserved2*: IsdBool ## Reserved for future use
    bReserved3*: IsdBool ## Reserved for future use
    bReserved4*: IsdBool ## Reserved for future use


type 
  IsdStationInfo* = object
    ## Station configuration information.
    ##
    ## This type can only be used with IS Precision Series tracking devices. If
    ## passed to `isdSetStationConfig <#isdSetStationConfig>`_ or
    ## `isdGetStationConfig <#isdGetStationConfig>`_ with InterTrax,
    ## `isdFalse <#IsdBool>`_ is returned.
    id*: cint ## Unique number identifying a station. It is the same as that 
      ## passed to the `isdSetStationConfig` and `isdGetStationConfig` functions
      ## and can be `1` to `isdMaxStations <#isdMaxStations>`_ 
    state*: IsdBool ## Whether the station is turned on or off
      ## (`isdTrue <#IsdBool>`_ = ON, `isdFalse <#IsdBool>`_ = OFF).
      ## InertiaCubes are considered to be a tracking system consisting of one
      ## station, which cannot be turned off, so this field will always be
      ## `true <#IsdBool>`_. The IS-900 may have up to 7 stations
      ## connected.
    compass*: IsdOnOff ## Controls the state of the compass component (only
      ## available for InertiaCube devices). The compass is only used when the
      ## station is is configured GEOS or Dual modes. In Fusion mode, compass
      ## readings are not used, regardless of this setting. When the station is
      ## configured for full compass mode, the readings produced by the
      ## magnetometers inside the InertiaCube are used as absolute reference
      ## orientation for yaw. Compasses can be affected by metallic objects and
      ## electronic equipment in close proximity to an InertiaCube.
    inertiaCube*: cint ## InertiaCube associated with this station. If no
      ## InertiaCube is assigned, this number is `-1`. Otherwise, it is a
      ## positive number between `1` and `isdMaxStations <#isdMaxStations>`_.
      ## Only relevant for IS-300 and IS-600 series devices. For IS-900 systems
      ## it is always the same as the station number, and for InterTrax and
      ## InertiaCubes it is always `1`.
    enhancement*: cint ## Perceptual enhancement level (0, 1, or 2)
    sensitivity*: cint ## Sensitivity level (1 to 4) if `enhancement` is 1 or 2
    prediction*: cint ## Motion prediction (0 to 50 msec); only supported by
      ## IS-300, IS-600, IS-900 and InertiaCubes; not available for InterTrax
    angleFormat*: IsdAngleFormat ## Format of returned orientation angles. Used
      ## only for IS-900 and IS-1200 series devices. 3DOF sensors report both,
      ## regardless of this setting.
    timeStamped*: IsdBool ## Whether data should be timestamped
    getInputs*: IsdBool ## Whether button and joystick data should be included
    getEncoderData*: IsdBool ## Whether raw encoder data should be included.
    compassCompensation*: cuchar ## Controls how Magnetic Environment Calibration
      ## is applied. This calibration calculates nominal field strength and dip
      ## angle for the environment in which the sensor is used. Based on these
      ## values, the system can assign a weight to compass measurements,
      ## allowing it to reject bad measurements.
      ##
      ## If CompassCompensation is set to `0`, the calibration is ignored and
      ## all compass data is used. Higher values result in a tighter rejection
      ## threshold, resulting in more measurements being rejected. If the sensor
      ## is used in an environment with significant magnetic interference this
      ## can result in drift due to insufficient compensation from the compass
      ## data. Default setting is 2, maximum setting is 3.
      ##
      ## Note: the sensor must be calibrated in the ISDemo Compass Calibration
      ## Tool for this setting to have any effect.
    imuShockSuppression*: cuchar ## Controls how the system deals with sharp
      ## changes in IMU data that can be caused by shock or impact. Sensors may
      ## experience momentary rotation rates or accelerations that are outside
      ## of the specified range, resulting in undesirable behavior. By turning
      ## on shock suppression you can have the system filter out corrupted data.
      ##
      ## Values 0 (OFF) to 2 are accepted, with higher values resulting in
      ## greater filtering.
    urmRejectionFactor*: cuchar ## Controls the rejection threshold for
      ## ultrasonic measurements. Currently implemented only for the IS-900
      ## PCTracker. Default setting is 4, which results in measurements with
      ## range errors greater than 4 times the average to be rejected.
      ##
      ## Note: do not change this setting without first consulting with
      ## InterSense technical support.
    bReserved2*: cuchar ## Reserved for future use
    coordFrame*: IsdCoordFrame ## Coordinate frame in which position and
      ## orientation data is reported
    accelSensitivity*: cint ## Acceleration sensitivity for 3-DOF tracking with
      ## InertiaCube products only. It controls how fast tilt correction, using
      ## accelerometers, is applied. Valid values are 1 to 4, with 2 as default.
      ##
      ## - Level 1 reduces the amount of tilt correction during movement. While
      ##   it will prevent any effect linear accelerations may have on pitch and
      ##   roll, it will also reduce stability and dynamic accuracy. It should
      ##   only be used in situations when sensor is not expected to experience
      ##   a lot of movement.
      ## - Level 2 (default) is best for head tracking in static environment,
      ##   with user seated.
      ## - Level 3 allows for more aggressive tilt compensation, appropriate
      ##   when sensor is moved a lot, for example, when the user is walking for
      ##   long periods of time.
      ## - Level 4 allows for even greater tilt corrections. It will reduce
      ##   orientation accuracy by  allowing linear accelerations to effect
      ##   orientation, but increase stability. This level is appropriate for
      ##   when the user is running, or in other situations where the sensor
      ##   experiences a great deal of movement.
    fReserved1*: cfloat ## Reserved for future use
    fReserved2*: cfloat ## Reserved for future use
    tipOffset*: array[3, cfloat] ## Offset of the reported position from the
      ## physical point being tracked. This is only applicable system capable of
      ## tracking position.
    fReserved3*: cfloat ## Reserved for future use
    getCameraData*: IsdBool ## Whether to include computed FOV, aperture and
      ## other camera data in output (default is `isdFalse <#IsdBool>`_)
    getAuxInputs*: IsdBool ## Whether to include values from auxiliary inputs
      ## connected to the I2C port in MicroTrax devices
    getCovarianceData*: IsdBool
    getExtendedData*: IsdBool ## Retrieving extended data will reduce update
      ## rate with even a single tracker when using serial communications;
      ## Ethernet is highly recommended when retrieving extended data


type 
  IsdStationData* = object 
    trackingStatus*: cuchar ## Tracking status, represents "Tracking Quality"
      ## (0-255; 0 if lost)
    newData*: cuchar ## `1` if data changed since last call to
      ## `isdGetTrackingData <#isdGetTrackingData>`_, `0` otherwise
    commIntegrity*: cuchar ## Communication integrity (percentage of packets
      ## received from tracker, 0-100) 
    batteryState*: IsdBatteryState ## Battery state (wireless devices only; not
      ## currently used by MiniTrax and MicroTrax stations)
    euler*: array[3, cfloat] ## Orientation in Euler angles (yaw, pitch, roll)
    Quaternion*: array[4, cfloat] ## Orientation in Quaternion format (W,X,Y,Z)
    position*: array[3, cfloat] ## Position in meters 
    timeStamp*: cfloat ## Timestamp in seconds, reported only if requested 
    stillTime*: cfloat ## InertiaCube and PC-Tracker products only, whether
      ## sensor is still
    batteryLevel*: cfloat ## Battery voltage, if available
    compassYaw*: cfloat ## Magnetometer heading, computed based on current
      ## orientation; available for InertiaCube products only, such as IC2, IC3
      ## and IC2+
    buttonState*: array[isdMaxButtons, IsdBool] ## Button states, if requested 
    analogData*: array[isdMaxChannels, cshort] ## Analog data, if requested
      ## Current hardware is limited to 10 channels, with only 2 being used. The
      ## only device using this is the IS-900 wand that has a built-in analog
      ## joystick. Channel 1 is X-axis rotation, channel 2 is Y-axis rotation
    auxInputs*: array[isdMaxAuxInputs, cuchar] ## Auxiliary data, if requested
    angularVelBodyFrame*: array[3, cfloat] ## rad/sec, in sensor body coordinate
      ## frame. Reported as rates about X, Y and Z axes, corresponding to Roll,
      ## Pitch, Yaw order. This is the processed angular rate, with current
      ## biases removed. This is the angular rate used to produce orientation
      ## updates.
    angularVelNavFrame*: array[3, cfloat] ## rad/sec, in world coordinate frame,
      ## with boresight and other transformations applied. Reported as rates
      ## about X, Y and Z axes, corresponding to Roll, Pitch, Yaw order.
    accelBodyFrame*: array[3, cfloat] ## meter/sec^2, in sensor body coordinate
      ## frame. These are the accelerometer measurements in the sensor body
      ## coordinate frame. Only factory calibration is applied to this data,
      ## gravity component is not removed. Reported as accelerations along X, Y
      ## and Z axes.
    accelNavFrame*: array[3, cfloat] ## meters/sec^2, in the navigation (earth)
      ## coordinate frame. This is the accelerometer measurements with
      ## calibration, current sensor orientation applied, and gravity
      ## subtracted. This is the best available estimate of tracker
      ## acceleration. Reported as accelerations along X, Y and Z axes.
    velocityNavFrame*: array[3, cfloat] ## meters/sec, 6-DOF systems only.
      ## Reported as velocity along X, Y and Z axes.
    angularVelRaw*: array[3, cfloat] ## Raw gyro output, only factory
      ## calibration is applied. Some errors due to temperature dependant gyro
      ## bias drift will remain.
    measQuality*: cuchar ## Ultrasonic Measurement Quality (IS-900 only,
      ## firmware >= 4.26)
    bReserved2*: cuchar ## Reserved for future use
    bReserved3*: cuchar ## Reserved for future use
    bReserved4*: cuchar ## Reserved for future use
    timeStampSeconds*: cuint ## Time Stamp in whole seconds
    timeStampMicroSec*: cuint ## Fractional part of the Time Stamp in
      ## micro-seconds.
    oSTimeStampSeconds*: cuint ## Data record arrival time stamp based on
      ## OS time
    oSTimeStampMicroSec*: cuint ## Reserved for future use; not implemented
    reserved*: array[55, cfloat] ## Reserved for future use
    temperature*: cfloat ## Station temperature in degrees C (3DOF sensors only)
    magBodyFrame*: array[3, cfloat] ## 3DOF sensors only. Magnetometer data
      ## along the X, Y, and Z axes Units are nominally in Gauss, and factory
      ## calibration is applied.  Note, however, that most sensors are not
      ## calibrated precisely since the exact field strength is not necessary
      ## to for tracking purposes.  Relative magnitudes should be accurate,
      ## however. Fixed metal compass calibration may rescale the values, as
      ## well.
  

type 
  IsdCameraEncoderData* = object
    ## Camera encoder data.
    trackingStatus*: cuchar ## Tracking status 
    bReserved1*: cuchar ## Pack to 4 byte boundary 
    bReserved2*: cuchar
    bReserved3*: cuchar
    timecode*: cuint ## Timecode, not implemented yet 
    apertureEncoder*: cint ## Aperture encoder counts, relative to last reset
      ## or power up 
    focusEncoder*: cint ## Focus encoder counts 
    zoomEncoder*: cint ## Zoom encoded counts 
    timecodeUserBits*: cint ## Time code user bits, not implemented yet 
    aperture*: cfloat ## Computed aperture value 
    focus*: cfloat ## Computed focus value (mm), not implemented yet 
    fOV*: cfloat ## Computed vertical FOV value (degrees) 
    nodalPoint*: cfloat ## Nodal point offset due to zoom and focus (mm) 
    covarianceOrientation*: array[3, cfloat] ## Available only for IS-1200
    covariancePosition*: array[3, cfloat]
    dwReserved1*: cint ## Reserved for future use
    dwReserved2*: cint ## Reserved for future use
    fReserved1*: cfloat ## Reserved for future use
    fReserved2*: cfloat ## Reserved for future use
    fReserved3*: cfloat ## Reserved for future use
    fReserved4*: cfloat ## Reserved for future use


  IsdTrackingData* = object
    ## Tracking data for all stations.
    station*: array[isdMaxStations, IsdStationData]


  IsdCameraData* = object
    ## Camera data for all stations.
    camera*: array[isdMaxStations, IsdCameraEncoderData]


  IsdAuxSystem* {.size: sizeof(cint).} = enum ## \
    ## Supported auxiliary systems.
    none = 0,
    ultrasonic, 
    optical,
    magnetic,
    rf, 
    gps


type 
  IsdHardwareCapability* = object 
    ## Hardware capabilities.
    position*: IsdBool ## Can track position
    orientation*: IsdBool ## Can track orientation
    encoders*: IsdBool ## Can support lens encoders
    prediction*: IsdBool ## Predictive algorithms are available
    enhancement*: IsdBool ## Enhancement level can be changed
    compass*: IsdBool ## Compass setting can be changed
    selfTest*: IsdBool ## Has the self-test capability
    errorLog*: IsdBool ## Can keep error log
    ultVolume*: IsdBool ## Can control ultrasonic volume via software
    ultGain*: IsdBool ## Can control microphone sensitivity by software
    ultTimeout*: IsdBool ## Can change ultrasonic sampling frequency
    photoDiode*: IsdBool ## SoniDiscs support photodiode
    maxStations*: cint ## Number of supported stations
    maxImus*: cint ## Number of supported IMUs
    maxFPses*: cint ## Maximum number of Fixed Position Sensing Elements
      ## (constellation/galaxy)
    maxChannels*: cint ## Max. number of analog channels supported per station
    maxButtons*: cint ## Maximum number of digital button inputs per station
    measData*: IsdBool ## Can provide measurement data
    diagData*: IsdBool ## Can provide diagnostic data
    pseConfig*: IsdBool ## Supports PSE configuration/reporting tools
    configLock*: IsdBool ## Supports configuration locking     
    ultMaxRange*: cfloat ## Maximum ultrasonic range  
    fReserved2*: cfloat ## Reserved for future use
    fReserved3*: cfloat ## Reserved for future use
    fReserved4*: cfloat ## Reserved for future use
    compassCal*: IsdBool ## Supports dynamic compass calibration     
    bReserved2*: IsdBool ## Reserved for future use
    bReserved3*: IsdBool ## Reserved for future use
    bReserved4*: IsdBool ## Reserved for future use
    dwReserved1*: cint ## Reserved for future use
    dwReserved2*: cint ## Reserved for future use
    dwReserved3*: cint ## Reserved for future use
    dwReserved4*: cint ## Reserved for future use


  IsdHardwareInfo* = object
    ## System hardware information.
    valid*: IsdBool ## Set to `isdTrue <#IsdBool>`_ if
      ## `isdGetSystemHardwareInfo <#isdGetSystemHardwareInfo>`_ succeeded
    trackerType*: IsdSystemType ## See `IsdSystemType <#IsdSystemType>`_
    trackerModel*: IsdSystemModel ## See `IsdSystemModel <#IsdSystemModel>`_
    port*: cint ## Hardware port number (1 for COM1/ttyS0, etc.)
    iftype*: IsdInterfaceType ## Hardware interface (RS232, USB, etc.)
    onHost*: IsdBool ## `isdTrue <#IsdBool>`_ if tracking algorithms are
      ## executed in the library
    auxSystem*: IsdAuxSystem ## Position tracking hardware
      ## (see `IsdAuxSystemType <#IsdAuxSystemType>`_)
    firmwareRev*: cfloat ## Firmware revision 
    modelName*: array[128, cchar] ## Model name string
    capability*: IsdHardwareCapability ## Hardware capabilities
    bReserved1*: IsdBool ## Reserved for future use
    bReserved2*: IsdBool ## Reserved for future use
    bReserved3*: IsdBool ## Reserved for future use
    bReserved4*: IsdBool ## Reserved for future use
    baudRate*: cuint ## Serial port baud rate      
    numTestLevels*: cint ## Number of self test levels       
    dwReserved3*: cint ## Reserved for future use
    dwReserved4*: cint ## Reserved for future use
    fReserved1*: cfloat ## Reserved for future use
    fReserved2*: cfloat ## Reserved for future use
    fReserved3*: cfloat ## Reserved for future use
    fReserved4*: cfloat ## Reserved for future use
    cReserved1*: array[128, cchar] ## Reserved for future use
    cReserved2*: array[128, cchar] ## Reserved for future use
    cReserved3*: array[128, cchar] ## Reserved for future use
    cReserved4*: array[128, cchar] ## Reserved for future use


type 
  IsdStationCapability* = object
    ## Station capabilities information.
    position*: IsdBool ## Whether the station can track position
    orientation*: IsdBool ## Wether the station can track orientation
    encoders*: cint ## Number of lens encoders (`0` = none are available)
    numChannels*: cint ## Number of analog channels supported by this station,
      ## wand has 2 (joystick axes)
    numButtons*: cint ## Number of digital button inputs supported by this
      ## station
    auxInputs*: cint ## Number of auxiliary input channels (OEM products)
    auxOutputs*: cint ## Number of auxiliary output channels (OEM products)
    compass*: IsdBool ## Whether the station has a compass
    bReserved1*: IsdBool ## Reserved for future use
    bReserved2*: IsdBool ## Reserved for future use
    bReserved3*: IsdBool ## Reserved for future use
    bReserved4*: IsdBool ## Reserved for future use
    dwReserved1*: cint ## Reserved for future use
    dwReserved2*: cint ## Reserved for future use
    dwReserved3*: cint ## Reserved for future use
    dwReserved4*: cint ## Reserved for future use


type 
  IsdStationHardwareInfo* = object
    ## Station hardware information.
    Valid*: IsdBool ## Will be set to `isdTrue <#IsdBool>`_ if
      ## `isdGetStationHardwareInfo <#isdGetStationHardwareInfo>`_ succeeded
    id*: cint ## Unique number identifying a station. It is the same as that 
      ## passed to the `isdSetStationConfig` and `isdGetStationConfig` functions
      ## and can be `1` to `isdMaxStations <#isdMaxStations>`_ 
    descVersion*: array[20, cchar] ## Station Descriptor version 
    firmwareRev*: cfloat ## Station firmware revision
    serialNum*: cint ## Station serial number 
    calDate*: array[20, cchar] ## Last factory calibration date (mm/dd/yyyy)
    port*: cint ## Hardware port number 
    capability*: IsdStationCapability ## Station capabilities
    bReserved1*: IsdBool ## Reserved for future use
    bReserved2*: IsdBool ## Reserved for future use
    bReserved3*: IsdBool ## Reserved for future use
    bReserved4*: IsdBool ## Reserved for future use
    stationType*: cint # Station type        
    deviceId*: cint
    dwReserved3*: cint ## Reserved for future use
    dwReserved4*: cint ## Reserved for future use
    fReserved1*: cfloat ## Reserved for future use
    fReserved2*: cfloat ## Reserved for future use
    fReserved3*: cfloat ## Reserved for future use
    fReserved4*: cfloat ## Reserved for future use
    cReserved1*: array[128, cchar] ## Reserved for future use
    cReserved2*: array[128, cchar] ## Reserved for future use
    cReserved3*: array[128, cchar] ## Reserved for future use
    cReserved4*: array[128, cchar] ## Reserved for future use

  IsdPortWirelessInfo* = object
    ## Wireless hardware information.
    valid*: IsdBool
    status*: cint
    wireless*: IsdBool
    channel*: cint
    id*: array[4, cint]
    radioVersion*: cint ## Type of radio hardware:
      ## - 15 or 31: 2.4 GHz (Aerocomm, radio used with older MiniTrax trackers)
      ## - 128: 2.4 GHz (Chipcon, MicroTrax only)
      ## - 144: 900 MHz (MicroTrax only)
      ## - 160: 868 MHz (MicroTrax only)
    dReserved1*: cint ## Reserved for future use
    dReserved2*: cint ## Reserved for future use
    dReserved3*: cint ## Reserved for future use
    dReserved4*: cint ## Reserved for future use


proc isdOpenTracker*(hParent: IsdHwnd; commPort: cint; infoScreen: IsdBool; 
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
  ##   pass `isdFalse <#IsdBool>`_.
  ## verbose
  ##    Pass `isdTrue <#IsdBool>`_ if you would like a more detailed report of
  ##    the DLL activity. Messages are printed to the console
  ## result
  ##   - Handle of the opened tracker on success
  ##   - `-1` on failure
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


proc isdOpenAllTrackers*(hParent: IsdHwnd; handles: ptr IsdTrackerHandleArray;
  infoScreen: IsdBool; verbose: IsdBool): cint
  {.cdecl, dynlib: dllname, importc: "ISD_OpenAllTrackers".}
  ## Open multiple trackers.
  ##
  ## handles
  ##   Will contain the handles for all detected trackers
  ## infoScreen
  ##   This feature has not been implemented. Its purpose is to display an
  ##   information window to show the tracker detection progress and results.
  ##   Currently, the DLL writes only to the console. Most applications should
  ##   pass `isdFalse <#IsdBool>`_.
  ## verbose
  ##    Pass `isdTrue <#IsdBool>`_ if you would like a more detailed report of
  ##    the DLL activity. Messages are printed to the console
  ## result
  ##   - Number of detected trackers on success
  ##   - `-1` on failure
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
  ##   - `isdTrue <#IsdBool>`_ on success
  ##   - `isdFalse <#IsdBool>`_ otherwise
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
  ##   - `isdTrue <#IsdBool>`_ on success
  ##   - `isdFalse <#IsdBool>`_ otherwise
  ##
  ## Also retrieves genlock synchronization configuration, if available.
  ## See `IsdTrackerInfo <#IsdTrackerInfo>`_ for a complete list

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
  ##   - `isdTrue <#IsdBool>`_ on success
  ##   - `isdFalse <#IsdBool>`_ otherwise
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
  ##   - `isdTrue <#IsdBool>`_ on success
  ##   - `isdFalse <#IsdBool>`_ otherwise
  ##
  ## This function reads `recordsPerSec` and `kBitsPerSec` without requesting
  ## genlock settings from the tracker. Use this instead of
  ## `isdGetTrackerConfig <#isdGetTrackerConfig>`_ to prevent your program
  ## from stalling while waiting for the tracker response. 


proc isdSetStationConfig*(handle: IsdTrackerHandle;
  station: ptr IsdStationInfo; stationId: cshort; verbose: IsdBool): IsdBool
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
  ##   - `isdTrue <#IsdBool>`_ on success
  ##   - `isdFalse <#IsdBool>`_ otherwise
  ##
  ## Before this function is called, all elements of the `station` structure
  ## must be assigned a value. This function should only be used with IS
  ## Precision Series tracking devices; not valid for InterTrax.

proc isdGetStationConfig*(handle: IsdTrackerHandle;
  station: ptr IsdStationInfo; stationId: cshort; verbose: IsdBool): IsdBool
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
  ##   - `isdTrue <#IsdBool>`_ on success
  ##   - `isdFalse <#IsdBool>`_ otherwise
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
  ##   - `isdTrue <#IsdBool>`_ on success
  ##   - `isdFalse <#IsdBool>`_ otherwise
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


proc isdGetTrackingData*(handle: IsdTrackerHandle; data: ptr IsdTrackingData):
  IsdBool {.cdecl, dynlib: dllname, importc: "ISD_GetTrackingData".}
  ## Get data from all configured stations.
  ##
  ## data
  ##   Will contain the tracking data
  ## result
  ##   - `isdTrue <#IsdBool>`_ on success
  ##   - `isdFalse <#IsdBool>`_ otherwise


proc isdGetTrackingDataAtTime*(handle: IsdTrackerHandle; atTime: cdouble;
  maxSyncWait: cdouble): IsdBool
  {.cdecl, dynlib: dllname, importc: "ISD_GetTrackingDataAtTime".}
  ## Get data from all configured stations (at a specified time).
  ##
  ## data
  ##   Will contain the tracking data
  ## atTime
  ##   The time at which the data was generated (?)
  ## result
  ##   - `isdTrue <#IsdBool>`_ on success
  ##   - `isdFalse <#IsdBool>`_ otherwise
 

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
  ##   - `isdTrue <#IsdBool>`_ on success
  ##   - `isdFalse <#IsdBool>`_ otherwise
  ##
  ## Data is placed in the `IsdCameraData <#IsdCameraData>`_ structure. This
  ## function does not service the serial port, so
  ## `isdGetTrackingData <#isdGetTrackingData>`_ must be called prior to this. 


proc ISD_RingBufferSetup*(handle: IsdTrackerHandle; stationId: cshort; 
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
  ##   - `isdTrue <#IsdBool>`_ on success
  ##   - `isdFalse <#IsdBool>`_ otherwise
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
  ## but the `NewData` flag will be `isdFalse <#IsdBool>`_ to
  ## indicate that the buffer has been emptied.


proc isdRingBufferStart*(handle: IsdTrackerHandle; stationId: cshort): IsdBool
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
  ##   - `isdTrue <#IsdBool>`_ on success
  ##   - `isdFalse <#IsdBool>`_ otherwise
  ##
  ## While active, all data samples are stored in the buffer. Because this is a
  ## ring buffer, it will only store the number of samples specified in the call
  ## to `isdRingBufferSetup <#isdRingBufferSetup>`_, so the oldest samples can
  ## be overwritten.


proc isdRingBufferStop*(handle: IsdTrackerHandle; stationId: cshort): IsdBool
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
  ##   - `isdTrue <#IsdBool>`_ on success
  ##   - `isdFalse <#IsdBool>`_ otherwise
  ##
  ## The library will continue to process data, but the contents of the ring
  ## buffer will not be altered.


proc isdRingBufferQuery*(handle: IsdTrackerHandle; stationId: cshort; 
  currentData: ptr IsdStationData; head: ptr cint; tail: ptr cint): IsdBool
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

proc isdResetHeading*(handle: IsdTrackerHandle; stationId: cshort): IsdBool
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
  ##   - `isdTrue <#IsdBool>`_ on success
  ##   - `isdFalse <#IsdBool>`_ otherwise


proc isdBoresightReferenced*(handle: IsdTrackerHandle; stationId: cshort; 
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
  ##   - `isdTrue <#IsdBool>`_ on success
  ##   - `isdFalse <#IsdBool>`_ otherwise
  ##
  ## This is useful when you need to apply a specific offset to system output.
  ## For example, if a sensor is mounted at 40 degrees pitch relative to an HMD,
  ## you can pass in `0, 40, 0` to make the system output `(0,0,0)` when the HMD
  ## is horizontal.
  ##
  ## This function works with all IS-X00 series products, InterTraxLC and
  ## InertiaCube products. For InterTrax30 and InterTrax2, it behaves like
  ## `isdResetHeading <#isdResetHeading>`_.

proc isdBoresight*(handle: IsdTrackerHandle; stationId: cshort; set: IsdBool):
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
  ##   - `isdTrue <#IsdBool>`_ on success
  ##   - `isdFalse <#IsdBool>`_ otherwise
  ##
  ## If `set` is `isdTrue <#IsdBool>`_, all angles are reset to zero. Otherwise,
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
  ##   - `isdTrue <#IsdBool>`_ on success
  ##   - `isdFalse <#IsdBool>`_ otherwise
  ##
  ## The script must consist of valid commands as described in the interface
  ## protocol. Commands in the script should be terminated by the newline
  ## character '\n'. The linefeed character '\r' is added by the function, and
  ## is not required.
  ##
  ## Note that this may not be supported when using the shared memory interface,
  ## such as with sfServer, and is primarily intended for the
  ## IS-300/IS-600/IS-900 system.


proc isdAuxOutput*(handle: IsdTrackerHandle; stationId: cshort; 
  AuxOutput: ptr cuchar; length: cushort): IsdBool
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
  ##   - `isdTrue <#IsdBool>`_ on success
  ##   - `isdFalse <#IsdBool>`_ otherwise
  ##
  ## The number of bytes should match the number the auxiliary outputs the
  ## interface is configured to expect. If too many are specified, extra bytes
  ## are ignored. 


proc isdNumOpenTrackers*(num: ptr cshort): IsdBool
  {.cdecl, dynlib: dllname, importc: "ISD_NumOpenTrackers".}
  ## Get the number of currently opened trackers.
  ##
  ## num
  ##   Will hold the number of trackers
  ## result
  ##   - `isdTrue <#IsdBool>`_ on success
  ##   - `isdFalse <#IsdBool>`_ otherwise


proc isdGetTime*(): cfloat {.cdecl, dynlib: dllname, importc: "ISD_GetTime".}
  ## Platform independent time function.
  ##
  ## result
  ##   Time value


proc isdUdpDataBroadcast*(handle: IsdTrackerHandle; port: cint; 
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
  ##   - `isdTrue <#IsdBool>`_ on success
  ##   - `isdFalse <#IsdBool>`_ otherwise


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
  ##   - `isdTrue <#IsdBool>`_ on success
  ##   - `isdFalse <#IsdBool>`_ otherwise
  ##
  ## Note that the system is a single tracker (and will thus have one handle).
  ## For details on individual stations (such as the devices on each port of an
  ## IS-900), use `isdGetStationHardwareInfo <#isdGetStationHardwareInfo>`_
  ## instead.


proc isdGetStationHardwareInfo*(handle: IsdTrackerHandle;
  info: ptr IsdStationHardwareInfo; stationId: cshort): IsdBool
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
  ##   - `isdTrue <#IsdBool>`_ on success
  ##   - `isdFalse <#IsdBool>`_ otherwise


proc isdEnterHeading*(handle: IsdTrackerHandle; stationId: cshort; 
  yaw: cfloat): IsdBool {.cdecl, dynlib: dllname, importc: "ISD_EnterHeading".}
  ## result
  ##   - `isdTrue <#IsdBool>`_ on success
  ##   - `isdFalse <#IsdBool>`_ otherwise


proc isdGetPortWirelessInfo*(handle: IsdTrackerHandle; port: cshort;
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
  ##   - `isdTrue <#IsdBool>`_ on success
  ##   - `isdFalse <#IsdBool>`_ otherwise
