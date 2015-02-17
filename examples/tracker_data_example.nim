## *io-isense* - Nim bindings for the InterSense SDK.
##
## This file is part of the `Nim I/O <http://nimio.us>`_ package collection.
## See the file LICENSE included in this distribution for licensing details.
## GitHub pull requests are encouraged. (c) 2015 Headcrash Industries LLC.

import isense, os


# The following program is a basic example of using `isense` to connect to a
# tracking device and log out tracking data and some statistics.

var trackerData: IsdTrackingData
var trackerHandle: IsdTrackerHandle
var trackerInfo: IsdTrackerInfo

# connect to tracker
trackerHandle = isdOpenTracker(0, 0, isdFalse, isdFalse)

if trackerHandle == 0:
  echo "Error: Failed to find a tracking device"
else:
  for i in 0..19:
    # log out orientation and position
    if isdGetTrackingData(trackerHandle, addr(trackerData)) == isdFalse:
      echo "Warning: Failed to get tracking data"
    else:
      echo "Az=", trackerData.station[0].euler[0],
        ", El=", trackerData.station[0].euler[1],
        ", Rl=", trackerData.station[0].euler[2],
        ", X=", trackerData.station[0].position[0],
        ", Y=", trackerData.station[0].position[1],
        ", Z=", trackerData.station[0].position[2]

    # log out transfer statistics
    if isdGetCommInfo(trackerHandle, addr(trackerInfo)) == isdFalse:
      echo "Warning: Failed to get tracker info"
    else:
      echo "Kb/s: ", trackerInfo.kBitsPerSec,
        ", Rec/sec: ", trackerInfo.recordsPerSec

    sleep(1000)

  # close tracker
  if isdCloseTracker(trackerHandle) == isdFalse:
    echo "Warning: Failed to close tracker"

echo "Exiting."
