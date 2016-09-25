# pysignal
A Python wrapper for "Signal Hound" Spectrum Analyzer, Real Time Spectrum Analyzer and Tracking Generator

Supports the following products:
  - BB60C: Real Time Spectrum Analyzer
  - SA124B: Spectrum Analyzer
  - TG124A: Tracking generator

Works on Ubuntu 16.04, Windows 7, Windows 10

## 1. Real time spectrum analyzer (BB60C)
----------------------------------
User "RealTimeSignalHound" class, in pysignal.py. Look at main() function for example of usage.
The resulting file can be parsed in Matlab by using ParseBinaryIQFile.m.

```
realTimeSignalHound = RealTimeSignalHound();
print( "API version: %s" % realTimeSignalHound.GetAPIVersion() )

realTimeSignalHound.Connect()

try:
    print( "Firmware version: %s" % realTimeSignalHound.GetFirmwareVersion() )

    # Option 1:
    GetRawIQ( realTimeSignalHound ) # Sweep params are in GetRawIQ function

    # or
    # Option 2:
    stepSize 	   = 26.5  * MHZ
    startFrequency = 10  * MHZ
    endFrequency   = 1000 * MHZ
    overlapMargin  = 0.25 * MHZ 
    
    DoConsecutiveSweepOfRawIQ( realTimeSignalHound, startFrequency, endFrequency, stepSize, overlapMargin )
finally:
    realTimeSignalHound.Disconnect()

```

## 1.1 Analyzing continuous scans in Matlab:
------------------------------------
```
% After running the Python code above, the sweeps will be saved into 'consecutive_sweeps' directory, 
% in a timestamp-named directory. The Matlab code will read data from the most recent one.
analyzer = FFTAnalyzer()
analyzer.AnalyzeContiniousScan()
% (Short Time Fourier Transform, implemented in stft.m, written by  M.Sc. Eng. Hristo Zhivomirov)
```
## 1.2 Prequisites:
### 1.2.1 API drivers
Make sure to download API drivers from https://signalhound.com/support/product-downloads/bb60c-bb60a-downloads/

### 1.2.2 For Windows:
If running on windows, make sure to have the ftd2xx.dll file alongside the API dll. This file is exists in the SignalHound installation directory (if you installed the "Spike" application), and also available at
http://www.ftdichip.com/Drivers/D2XX.htm.
You may also need to install Microsoft Visual Studio 2012 Redistributables.
(Windows 10 will just fail to load silently, if you won't install the redistributables)

### 1.3 Signal Hound API Manual
can be found at http://signalhound.com/sigdownloads/BB60C/BB60-API-Manual.pdf


## 2. Using Spectrum Analyzer (SA124B) and Tracking Generator (TG124A):
-------------------------------------------------------------
Use "SignalHound" class, in pysignal.py. See main() for example (commented out).
The resulting file can be parsed in Matlab by using ParseBinarySweepFile.m

```
signalHound = SignalHound()
signalHound.Connect()

DoRegularSweep( signalHound ) # See sweep parameter configuration inside DoRegularSweep function
# or
DoRealTimeSweep( signalHound ) # See sweep parameter configuration inside DoRealTimeSweep function

# Using tracking generator:
frequency = 400 * MHZ
amplitude = -10
signalHound.ConnectToTrackingGenerator()
signalHound.ConfigTrackingGenerator( frequency, amplitude )
```


### 2.1 API drivers:
https://signalhound.com/download/bbsa-application-programming-interface-for-windows-3264-bit/

### 2.2 API Manual:
http://signalhound.com/sigdownloads/SA44B/SA-API-Manual.pdf

### 2.3 For Windows:
------------
If running on windows, make sure to have the ftd2xx.dll file alongside the API dll. This file is exists in the SignalHound installation directory, and also available at
http://www.ftdichip.com/Drivers/D2XX.htm.
You may also need to install Microsoft Visual Studio 2012 Redistributables.
(Windows 10 will just fail to load silently, if you won't install the redistributables)
