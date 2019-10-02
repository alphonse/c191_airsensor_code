# CHEM 191 Air Sensor Function Descriptions

The following functions are available for programming your sensors.  You will need to use all of them, and you might use some twice.  Combine them according to the flow chart provided.

Remember, most functions take the form of `function(argument);`, where the argument is used in completing the function.  Some functions do not require an argument, and others require multiple.  The arguments are provided below where needed and should be used verbatim.

- `alarm.setRtcTimer(0, 0, 0);`: Set the hibernation interval on the Teensy in Hours, Minutes, and Seconds.

- `createFileName(year(), month(), day()).toCharArray(dataFileName, 16);`: Generate a file name following a standard format of `YYYYMMDD.txt`; this is used to store the data on the SD card.

- `initializeSensors();`: Turn the sensors on and go through initialization procedure; no argument necessary.

- `initializeSDCard();`: Start the SD card reader on the Teensy; no argument necessary.

- `Serial.Begin(9600);`: Start the serial port at 9600 bits per second (baud).

- `Serial.println(dataString);`: Print the data to the serial port of the computer; useful for debugging.

- `Snooze.hibernate(teensySleep);`: Put the Teensy into hibernation mode for the time specified in `alarm.setRtcTimer(H, M, S);`.

- `setSyncProvider(getTeensy3Time);`: Update/set the time on the Teensy's real time clock (RTC).  Must be done after every hibernation.

- `String dataString = makeMeasurement(30);`: Create a string called `dataString`, make measurement for *n* (e.g. 30) seconds, and store the data in `dataString`.

- `turnOffSensors();`: Turn the sensors off before hibernating the Teensy; no argument necessary.

- `wakeSensors();`: Turn on the sensors after hibernating the Teensy; no argument necessary.

- `writeFile(dataFileName, dataString);`: Stores the `dataString` from the sensors to the `dataFileName` file on the SD card.
