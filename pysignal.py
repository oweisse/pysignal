# Author: Ofir Weisse, mail: oweisse (at) umich.edu, www.ofirweisse.com
#
# MIT License
#
# Copyright (c) 2016 oweisse
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from ctypes import cdll, CDLL, c_long, c_int, c_float, c_double, c_char_p, create_string_buffer, byref
import time
import datetime
import os
import sys

MHZ = 1e6
KHZ = 1e3

PRINT_FUNCTION_TRACE = False

def LogCall(func):
    '''Decorator to print function call details - parameters names and effective values'''
    def wrapper(*func_args, **func_kwargs):
        if not PRINT_FUNCTION_TRACE:
            return func(*func_args, **func_kwargs)

        arg_names = func.func_code.co_varnames[:func.func_code.co_argcount]
        args = func_args[:len(arg_names)]
        defaults = func.func_defaults or ()
        args = args + defaults[len(defaults) - (func.func_code.co_argcount - len(args)):]
        params = zip(arg_names, args)
        args = func_args[len(arg_names):]
        if args: params.append(('args', args))
        if func_kwargs: params.append(('kwargs', func_kwargs))
        print "TRACE: " + func.func_name + ' (' + ', '.join('%s = %r' % p for p in params) + ' )'
        return func(*func_args, **func_kwargs)
    return wrapper


class SweepMode:
 	RealTime 	= 1
 	NOTRealTime = 2

class SignalHound:
	def __init__( self ):
		self.sa_api = cdll.sa_api

	def Connect( self ):
		self.deviceHandle = c_int()

		print( "Openning device.." )
		openStatus = self.sa_api.saOpenDevice( byref( self.deviceHandle )  )
		print( "openStatus = %s; handle = %s" % ( openStatus, self.deviceHandle )  )

		self.verifyResult( openStatus )
		print( "Connected successfully!" )

	def Disconnect( self ):
		print( "Disconnecting!")
		self.sa_api.saCloseDevice( self.deviceHandle )

	def SetSweepParams_byCenter( self, centerFrequency, span, sweepResolution, referenceLevel):
		self.ConfigCenterSpan( centerFrequency, span )
		self.ConfigAquisition()
		self.ConfigReferenceLevel( referenceLevel )
		self.ConfigSweepResoloution( sweepResolution )

	def ConfigCenterSpan( self, centerFrequency, span ):
		result = self.sa_api.saConfigCenterSpan( self.deviceHandle, 			\
												 c_double( centerFrequency ), 	\
												 c_double( span ) )
		self.verifyResult( result )

	def ConfigAquisition( self ):
		result = self.sa_api.saConfigAcquisition( self.deviceHandle, SAConstants.SA_MIN_MAX, SAConstants.SA_LOG_SCALE);
		self.verifyResult( result )

	def ConfigReferenceLevel( self, referenceLevel ):
		result = self.sa_api.saConfigLevel(self.deviceHandle, c_double( referenceLevel ) )
		self.verifyResult( result )

	def ConfigSweepResoloution( self, sweepResolution ):
		imageRejection 	= c_int( True )
		rbw 			= c_double( sweepResolution )
		vbw 			= c_double( sweepResolution )
		result = self.sa_api.saConfigSweepCoupling(self.deviceHandle, rbw, vbw, imageRejection )
		self.verifyResult( result )

	def InitiateSweep( self, mode ):
		ALWAYS_0 = 0
		result   = self.sa_api.saInitiate(self.deviceHandle, mode, ALWAYS_0)
		self.verifyResult( result )

	def verifyResult( self, result ):
		if result == SAStatus.saNoError:
			return
		else:
			raise SignalHoundError( "%s" % self.GetErrorString( result ), result )

	def GetErrorString( self, errorNumber ):
	 	errString_char_p = self.sa_api.saGetErrorString( errorNumber )
		errString 		 = c_char_p(errString_char_p).value

		return errString

	def StartSweeping( self ):
		sweepLen, startFrequency, binSize = self.GetSweepParams()

		minArray = ( c_float * sweepLen )()
		maxArray = ( c_float * sweepLen )()

		self.InitiateSweep( SAConstants.SA_SWEEPING )
		result = self.sa_api.saGetSweep_32f( self.deviceHandle, minArray, maxArray )
		self.verifyResult( result )

		print( "========================" )
		for i in range( 10 ):
			print( "frequency[ %d ] = %f; min[ %d ] = %f; max[ %d ] = %f" % (i, startFrequency.value + i*binSize.value, i,minArray[i],i,maxArray[i]))

	def StartRealTimeSweep( self ):
		frameScale = 100.0
		frameRate  = 30
		self.ConfigRealTimeSweep( frameScale, frameRate )

		self.InitiateSweep( SAConstants.SA_REAL_TIME )
		sweepLen, startFrequency, binSize = self.GetSweepParams()
		frameWidth, frameHeight 		  = self.GetRealTimeParams()

		frame    = ( c_float * ( frameWidth * frameHeight ) )()
		maxArray = ( c_float * sweepLen )()

		result 	  = self.sa_api.saGetRealTimeFrame( self.deviceHandle, maxArray, frame )
		self.verifyResult( result )

		print( "========================" )
		for i in range( 10 ):
			print( "frequency[ %d ] = %f;  max[ %d ] = %f" % (i, startFrequency + i*binSize, i,maxArray[i]))


	def GetSweepParams( self ):
		sweepLen 		= c_int(0)
		startFrequency 	= c_double(0)
		binSize 		= c_double( 0 )

		result = self.sa_api.saQuerySweepInfo(  self.deviceHandle, 		 \
												byref( sweepLen ), 		 \
												byref( startFrequency ), \
												byref( binSize ) )
		self.verifyResult( result )

		print( "sweepLen = %s; startFrequency = %s; binSize = %s; endFrequency = %f" % \
					( sweepLen.value, startFrequency.value, binSize.value,  startFrequency.value + (sweepLen.value - 1 )* binSize.value) )

		return sweepLen.value, startFrequency.value, binSize.value

	def ConfigRealTimeSweep( self, frameScale, frameRate ):
		result = self.sa_api.saConfigRealTime(  self.deviceHandle, 		\
												c_double( frameScale ), \
												c_int( frameRate ) )
		self.verifyResult( result )

	def GetRealTimeParams( self ):
		frameWidth  = c_int(0)
		frameHeight = c_int(0)

		result = self.sa_api.saQueryRealTimeFrameInfo( self.deviceHandle,   \
													   byref( frameWidth ), \
								 					   byref( frameHeight ) )
		self.verifyResult( result )

		print( "frameWidth = %d, frameHeight = %d" % ( frameWidth.value, frameHeight.value ))
		return frameWidth.value, frameHeight.value

	def ConnectToTrackingGenerator( self ):
		result = self.sa_api.saAttachTg( self.deviceHandle )
		self.verifyResult( result )

	def ConfigTrackingGenerator( self, frequency, amplitude ):
		result = self.sa_api.saSetTg( 	self.deviceHandle, 		\
										c_double( frequency  ), \
										c_double( amplitude ) )
		self.verifyResult( result )


class SignalHoundError( Exception ):
	def __init__( self, msg, errCode = -99990 ):
		Exception.__init__( self, msg )
		self.errorCode = errCode

class SAStatus:
	saUnknownErr 				= -666

	# Setting specific error codes
	saFrequencyRangeErr 		= -99
	saInvalidDetectorErr 		= -95
	saInvalidScaleErr 			= -94
	saBandwidthErr 				= -91
	saExternalReferenceNotFound = -89

	# Device-specific errors
	saOvenColdErr 				= -20

	# Data errors
	saInternetErr 				= -12
	saUSBCommErr 				= -11

	# General configuration errors
	saTrackingGeneratorNotFound = -10
	saDeviceNotIdleErr 			= -9
	saDeviceNotFoundErr 		= -8
	saInvalidModeErr 			= -7
	saNotConfiguredErr 			= -6
	saTooManyDevicesErr 		= -5
	saInvalidParameterErr 		= -4
	saDeviceNotOpenErr 			= -3
	saInvalidDeviceErr 			= -2
	saNullPtrErr 				= -1

	# No Error
	saNoError 					= 0

	# Warnings
	saNoCorrections 			= 1
	saCompressionWarning 		= 2
	saParameterClamped 			= 3
	saBandwidthClamped 			= 4

class SAConstants:
	SA_MIN_MAX 				= c_int( 0 )
 	SA_LOG_SCALE			= c_int( 0 )
 	SA_SWEEPING 			= c_int( 0 )
 	SA_REAL_TIME			= c_int( 1 )

class Decimation:
	SAMPLE_RATE_40_MS_MAX_BANDWIDTH_27MHZ 	= c_int( 1 )
	SAMPLE_RATE_20_MS_MAX_BANDWIDTH_17_8MHZ = c_int( 2 )
	SAMPLE_RATE_10_MS_MAX_BANDWIDTH_8MHZ 	= c_int( 4 )
	SAMPLE_RATE_5_MS_MAX_BANDWIDTH_3_75MHZ 	= c_int( 8 )
	SAMPLE_RATE_2_5_MS_MAX_BANDWIDTH_2MHZ 	= c_int( 16 )

class RealTimeSignalHound:
	def __init__( self ):
		if sys.platform.startswith( 'win' ):
			self.bb_api = cdll.bb_api
		elif sys.platform.startswith( 'linux' ):
			self.bb_api = CDLL('./libbb_api.so.3.0.5')
		else:
			raise Error( "Unknown operating system " + sys.platform )
	@LogCall
	def Connect( self ):
		self.deviceHandle = c_int()

		print( "Openning device.." )
		openStatus = self.bb_api.bbOpenDevice( byref( self.deviceHandle )  )
		print( "openStatus = %s; handle = %s" % ( openStatus, self.deviceHandle )  )

		self.verifyResult( openStatus )
		print( "Connected successfully!" )

	@LogCall
	def Disconnect( self ):
		print( "Disconnecting!")
		self.bb_api.bbCloseDevice( self.deviceHandle )

	def SetSweepParams_byCenter( self, 								\
								 centerFrequency, 					\
								 span, 								\
								 sweepResolution, 					\
								 referenceLevel,					\
								 sweepMode 			  = SweepMode.NOTRealTime, \
								 sensitivityInSeconds = 0.001):
		self.ConfigAquisition()
		self.ConfigCenterSpan( centerFrequency, span )
		self.ConfigGain()
		self.ConfigReferenceLevel( referenceLevel )
		self.ConfigSweepResoloution( sweepResolution, sensitivityInSeconds, sweepMode )
		self.SetUnits( BBConstants.BB_LOG ) #set units as dBm

	@LogCall
	def SetupRawIQCapture( 	self,								\
							centerFrequency, 					\
						 	span,								\
						 	referenceLevel,						\
						 	decimation = Decimation.SAMPLE_RATE_40_MS_MAX_BANDWIDTH_27MHZ ):
		self.ConfigCenterSpan( centerFrequency, span )
		self.ConfigReferenceLevel( referenceLevel )
		self.ConfigGain()
		self.ConfigureIQ( decimation, span)

	@LogCall
	def ConfigCenterSpan( self, centerFrequency, span ):
		result = self.bb_api.bbConfigureCenterSpan(  self.deviceHandle, 			\
													 c_double( centerFrequency ), 	\
													 c_double( span ) )
		self.verifyResult( result )

		self.centerFrequency = centerFrequency
		self.span 			 = span

	@LogCall
	def ConfigureIQ( self, decimation, span ):
		result = self.bb_api.bbConfigureIQ( self.deviceHandle,  \
											decimation, 		\
											c_double( span ) )

		ignoreList = [ BBStatus.bbClampedToLowerLimit]
		self.verifyResult( result, ignoreList )

	@LogCall
	def ConfigAquisition( self ):
		result = self.bb_api.bbConfigureAcquisition( self.deviceHandle, \
													 BBConstants.BB_MIN_AND_MAX, \
													 BBConstants.BB_LOG_SCALE);
		self.verifyResult( result )

	@LogCall
	def ConfigReferenceLevel( self, referenceLevel ):
		result = self.bb_api.bbConfigureLevel(self.deviceHandle, 		  \
											  c_double( referenceLevel ), \
											  BBConstants.BB_AUTO_ATTEN )
		self.verifyResult( result )

	@LogCall
	def ConfigGain( self ):
		result = self.bb_api.bbConfigureGain( self.deviceHandle, \
											  BBConstants.BB_AUTO_GAIN )
		self.verifyResult( result )

	@LogCall
	def ConfigSweepResoloution( self, sweepResolution, sensitivityInSeconds, sweepMode ):
		rbw 			= c_double( sweepResolution )
		vbw 			= c_double( sweepResolution )
		sweepTime 		= c_double( sensitivityInSeconds ) #0.001 is 1 milliseconds
		imageRejection 	= BBConstants.BB_NO_SPUR_REJECT

		if sweepMode == SweepMode.NOTRealTime:
			rbwShape = BBConstants.BB_RBW_SHAPE_FLATTOP
		elif sweepMode == SweepMode.RealTime:
			rbwShape = BBConstants.BB_RBW_SHAPE_NUTTALL
		else:
			raise SignalHoundError( "Unknown sweep mode: %s" % sweepMode)

		result = self.bb_api.bbConfigureSweepCoupling(self.deviceHandle, \
														rbw, vbw, 		 \
														sweepTime,		 \
														rbwShape, 	 	 \
														imageRejection )
		self.verifyResult( result )

	@LogCall
	def SetUnits( self, unitsEnum ):
		result = self.bb_api.bbConfigureProcUnits(self.deviceHandle, \
														unitsEnum )
		self.verifyResult( result )

	@LogCall
	def InitiateSweep( self, mode, streaming_stamping_flags = 0 ):
		# print( "InitiateSweep called")
		result   = self.bb_api.bbInitiate(  self.deviceHandle,  \
											mode, 				\
											streaming_stamping_flags)
		self.verifyResult( result )

	def verifyResult( self, result, ignoreList = [] ):
		if result == BBStatus.bbNoError:
			return
		elif result in ignoreList:
			print( "WARNING: %s; Error code = %d" % (self.GetErrorString( result ), result) )
			# import traceback
			# import sys
			# exc_type, exc_value, exc_traceback = sys.exc_info()
			# traceback.print_stack()
		else:
			errString = "%s; Error code = %d" % (self.GetErrorString( result ), result)
			raise SignalHoundError( errString, result )

	def GetErrorString( self, errorNumber ):
		self.bb_api.bbGetErrorString.restype = c_char_p
	 	errString = self.bb_api.bbGetErrorString( errorNumber )

		return errString

	def GetMaxEnergy( self ):
		minArrays, maxArrays = self.StartSweeping( numOfSweeps = 1, destFileName = None)

		return max( minArrays[ 0 ] )

	def StartSweeping( self, numOfSweeps = 1, destFileName = None ):
		self.InitiateSweep( BBConstants.BB_SWEEPING )
		sweepLen, startFrequency, binSize = self.GetSweepParams()

		ADCReported = False
		minArrays 	= []
		maxArrays 	= []
		timeStamps 	= (c_float * numOfSweeps)()
		# startTime = time.time()
		for i in range( numOfSweeps ):
			minArrays.append( ( c_float * sweepLen )())
			maxArrays.append( ( c_float * sweepLen )())

		startTime = time.clock()
		for i in range	( numOfSweeps ):
			result = self.bb_api.bbFetchTrace_32f( self.deviceHandle,   \
													c_int( sweepLen ),  \
													minArrays[ i ],		\
													maxArrays[ i ])
			timeStamps[i] = time.clock() - startTime
			# print( timeStamps[i] )
			try:
				self.verifyResult( result )
			except SignalHoundError, e:
				if e.errorCode == BBStatus.bbADCOverflow:
					if not ADCReported:
						sys.stdout.write( "ADC OVERFLOW. ")
						ADCReported = True
				else:
					raise


		endTime = time.clock()
		# print( "%d sweeps took %f seconds" % (numOfSweeps ,endTime - startTime ) )

		# print( "========================" )
		# print( "Sweep took %f seconds" % (endTime - startTime))
		# for i in range( 10 ):
		# 	print( "frequency[ %d ] = %f; min[ %d ] = %f; max[ %d ] = %f" % \
		# 		  (i, startFrequency + i*binSize, i,minArrays[0][i],i,maxArrays[0][i]))

		self.DumpSweepsToFile( sweepLen, 			\
							   startFrequency, 		\
							   binSize, 			\
							   minArrays, 			\
							   maxArrays, 			\
							   numOfSweeps, 		\
							   timeStamps,
							   destFileName )

		return minArrays, maxArrays

	def StartRawCapture( self, numberOfCaptures = 40,  destFileName = None ):
		NULL = 0
		############################################### ??????????????????????????
		self.InitiateSweep( BBConstants.BB_STREAMING, BBConstants.BB_STREAM_IQ )
		bufferSize, bandwidth, samples_per_sec = self.GetRawCaptureParams()

		sys.stdout.write( "bufferSize = %d, bandwidth = %f, samples_per_sec = %d " \
			% (bufferSize.value, bandwidth.value, samples_per_sec.value))

		rawDataBuffers 	= []

		for i in range( numberOfCaptures ):
			rawDataBuffers.append( ( c_float * ( bufferSize.value * 2 ) )())

		# startTime = time.clock()
		for i in range( numberOfCaptures ):
			result = self.bb_api.bbFetchRaw( self.deviceHandle, rawDataBuffers[ i ], NULL )
			self.verifyResult( result, ignoreList = [ BBStatus.bbUncalSweep ] )
		# endTime = time.clock()
		# print( "%d captures took %f seconds" % (numOfcaptures ,endTime - startTime ) )

	 	self.DumpCaptureToFile( rawDataBuffers, bandwidth.value, samples_per_sec.value, destFileName )
	 	return rawDataBuffers

	@LogCall
	def GetRawCaptureParams( self ):
		bufferSize 		= c_int   ( 0 )
		bandwidth 		= c_double( 0 )
		samples_per_sec = c_int   ( 0 )

		result = self.bb_api.bbQueryStreamInfo(  self.deviceHandle, 		 \
												byref( bufferSize ), 		 \
												byref( bandwidth ), \
												byref( samples_per_sec ) )
		self.verifyResult( result )

		return bufferSize, bandwidth, samples_per_sec

	def StartRealTimeSweep( self ):
		frameScale = 100.0
		frameRate  = 30
		self.ConfigRealTimeSweep( frameScale, frameRate )

		self.InitiateSweep( BBConstants.BB_REAL_TIME )
		sweepLen, startFrequency, binSize = self.GetSweepParams()
		frameWidth, frameHeight 		  = self.GetRealTimeParams()

		frame 		= ( c_float * ( frameWidth * frameHeight ) )()
		sweepVals 	= ( c_float * sweepLen )()

		result = self.bb_api.bbFetchRealTimeFrame( self.deviceHandle, \
												   sweepVals, \
												   frame )
		self.verifyResult( result )

		print( "========================" )
		for i in range( 10 ):
			print( "frequency[ %d ] = %f;  max[ %d ] = %f" % (i, startFrequency + i*binSize, i,sweepVals[i]))


	@LogCall
	def GetSweepParams( self ):
		traceLen 		= c_int   ( 0 )
		startFrequency 	= c_double( 0 )
		binSize 		= c_double( 0 )

		result = self.bb_api.bbQueryTraceInfo(  self.deviceHandle, 		 \
												byref( traceLen ), 		 \
												byref( binSize ), \
												byref( startFrequency ) )
		self.verifyResult( result )

		# print( "sweepLen = %s; startFrequency = %s; binSize = %s; endFrequency = %f" % \
		# 	( traceLen.value, startFrequency.value, binSize.value,  startFrequency.value + (traceLen.value - 1 )* binSize.value) )

		return traceLen.value, startFrequency.value, binSize.value

	@LogCall
	def ConfigRealTimeSweep( self, frameScale, frameRate ):
		result = self.bb_api.bbConfigureRealTime(  	self.deviceHandle, 		\
													c_double( frameScale ), \
													c_int   ( frameRate ) )
		self.verifyResult( result )

	def GetFirmwareVersion( self ):
		version  = c_int(0)

		result = self.bb_api.bbGetFirmwareVersion( self.deviceHandle,   \
												   byref( version ) )
		self.verifyResult( result )

		return version.value

	def GetAPIVersion( self ):
		self.bb_api.bbGetAPIVersion.restype = c_char_p
		version = self.bb_api.bbGetAPIVersion()

		return version

	@LogCall
	def GetRealTimeParams( self ):
		frameWidth  = c_int(0)
		frameHeight = c_int(0)

		result = self.bb_api.bbQueryRealTimeInfo( self.deviceHandle,   \
												  byref( frameWidth ), \
								 				  byref( frameHeight ) )
		self.verifyResult( result )

		print( "frameWidth = %d, frameHeight = %d" % ( frameWidth.value, frameHeight.value ))
		return frameWidth.value, frameHeight.value

	@LogCall
	def ConnectToTrackingGenerator( self ):
		result = self.bb_api.saAttachTg( self.deviceHandle )
		self.verifyResult( result )

	def ConfigTrackingGenerator( self, frequency, amplitude ):
		result = self.bb_api.saSetTg( 	self.deviceHandle, 		\
										c_double( frequency  ), \
										c_double( amplitude ) )
		self.verifyResult( result )

	def DumpCaptureToFile( self, rawDataBuffers, bandwidth, samples_per_sec, destFileName ):
		if destFileName is None:
			return

		numberOfCaptures 	= len( rawDataBuffers )
		captureLength 		= len( rawDataBuffers[ 0 ] )
		totalSamples 		= numberOfCaptures * captureLength

		fileObject = open( destFileName, 'wb' )
		try:
			fileObject.write( c_float( bandwidth 			) )
			fileObject.write( c_float( samples_per_sec 		) )
			fileObject.write( c_float( totalSamples 		) )
			fileObject.write( c_float( self.centerFrequency ) )

			for i in range( numberOfCaptures ):
				fileObject.write( rawDataBuffers[ i ] )
		finally:
			fileObject.close()

	def DumpSweepsToFile( self,
						  sweepLen, 			\
						  startFrequency, 		\
						  binSize, 				\
						  minArrays, 			\
						  maxArrays, 			\
						  numOfSweeps, 			\
						  timeStamps,			\
						  destFileName ):

		if destFileName is None:
			return

		fileObject = open( destFileName, 'wb' )
		try:
			fileObject.write( c_float( sweepLen 		) )
			fileObject.write( c_float( startFrequency 	) )
			fileObject.write( c_float( binSize 			) )
			fileObject.write( c_float( numOfSweeps 		) )
			fileObject.write( timeStamps )

			for i in range( numOfSweeps ):
				fileObject.write( minArrays[ i ] )
			for i in range( numOfSweeps ):
				fileObject.write( maxArrays[ i ] )
		finally:
			fileObject.close()

class BBStatus:
	# Configuration Errors
    bbInvalidModeErr             = -112
    bbReferenceLevelErr          = -111
    bbInvalidVideoUnitsErr       = -110
    bbInvalidWindowErr           = -109
    bbInvalidBandwidthTypeErr    = -108
    bbInvalidSweepTimeErr        = -107
    bbBandwidthErr               = -106
    bbInvalidGainErr             = -105
    bbAttenuationErr             = -104
    bbFrequencyRangeErr          = -103
    bbInvalidSpanErr             = -102
    bbInvalidScaleErr            = -101
    bbInvalidDetectorErr         = -100

    # General Errors
    bbLibusbError                = -18
    bbNotSupportedErr            = -17
    bbTrackingGeneratorNotFound  = -16

    bbUSBTimeoutErr              = -15
    bbDeviceConnectionErr        = -14
    bbPacketFramingErr           = -13
    bbGPSErr                     = -12
    bbGainNotSetErr              = -11
    bbDeviceNotIdleErr           = -10
    bbDeviceInvalidErr           = -9
    bbBufferTooSmallErr          = -8
    bbNullPtrErr                 = -7
    bbAllocationLimitErr         = -6
    bbDeviceAlreadyStreamingErr  = -5
    bbInvalidParameterErr        = -4
    bbDeviceNotConfiguredErr     = -3
    bbDeviceNotStreamingErr      = -2
    bbDeviceNotOpenErr           = -1

     # No Error
    bbNoError                    = 0

    # Warnings/Messages
    bbAdjustedParameter          = 1
    bbADCOverflow                = 2
    bbNoTriggerFound             = 3
    bbClampedToUpperLimit        = 4
    bbClampedToLowerLimit        = 5
    bbUncalibratedDevice         = 6
    bbDataBreak                  = 7
    bbUncalSweep                 = 8

class BBConstants:
	BB_MIN_AND_MAX 			= c_int( 0 )
	BB_AVERAGE 				= c_int( 1 )

	BB_LOG_SCALE            = c_int( 0 )
	BB_LIN_SCALE            = c_int( 1 )
	BB_LOG_FULL_SCALE       = c_int( 2 )
	BB_LIN_FULL_SCALE       = c_int( 3 )

	BB_AUTO_ATTEN 			= c_double( -1.0 )
	BB_MAX_REFERENCE        = c_double( 50.0 ) # dBM
	BB_MAX_ATTENUATION      = c_double( 30.0 ) # dB

	BB_AUTO_GAIN            = c_int( -1 )
	BB60_MAX_GAIN           = c_int(  3 )
	BB60C_MAX_GAIN          = c_int(  3 )
	BB124_MAX_GAIN          = c_int(  4 )

	BB_NO_SPUR_REJECT       = c_int( 0 )
	BB_SPUR_REJECT          = c_int( 1 )
	BB_BYPASS_RF            = c_int( 2 )

 	BB_NUTALL               = c_int( 0x0 )
	BB_BLACKMAN             = c_int( 0x1 )
	BB_HAMMING              = c_int( 0x2 )
	BB_FLAT_TOP             = c_int( 0x3 )
	BB_FLAT_TOP_EMC_9KHZ    = c_int( 0x4 )
	BB_FLAT_TOP_EMC_120KHZ  = c_int( 0x5 )

	BB_LOG                  = c_int( 0x0 )
	BB_VOLTAGE              = c_int( 0x1 )
	BB_POWER                = c_int( 0x2 )
	BB_SAMPLE               = c_int( 0x3 )

	BB_NATIVE_RBW           = c_int( 0x0 )
	BB_NON_NATIVE_RBW       = c_int( 0x1 )

	BB_IDLE                 = c_int( -1 )
	BB_SWEEPING             = c_int( 0x0 )
	BB_REAL_TIME            = c_int( 0x1 )
	BB_ZERO_SPAN            = c_int( 0x2 )
	BB_TIME_GATE            = c_int( 0x3 )
	BB_STREAMING            = c_int( 0x4 )
	BB_RAW_PIPE     		= BB_STREAMING # use BB_STREAMING
	BB_RAW_SWEEP            = c_int( 0x5 )
	BB_RAW_SWEEP_LOOP       = c_int( 0x6 )
	BB_AUDIO_DEMOD          = c_int( 0x7 )
	BB_TG_SWEEPING          = c_int( 0x8 )

	BB_RBW_SHAPE_NUTTALL    = c_int( 0x0 )
	BB_RBW_SHAPE_FLATTOP    = c_int( 0x1 )
	BB_RBW_SHAPE_CISPR      = c_int( 0x2 )

 	BB_STREAM_IQ            = c_int( 0x0 )
 	BB_STREAM_IF            = c_int( 0x1 )
	BB_DIRECT_RF            = c_int( 0x2 ) # BB60C only
	BB_TIME_STAMP           = c_int( 0x10)





def GetRawIQ( signalHound ):
	centerFrequency = 450 * MHZ
	span 			= 27  * MHZ
	referenceLevel  = -50

	signalHound.SetupRawIQCapture( 	centerFrequency, 					\
								 	span,								\
								 	referenceLevel,						\
								 	Decimation.SAMPLE_RATE_40_MS_MAX_BANDWIDTH_27MHZ )

	# signalHound.InitiateSweep( BBConstants.BB_STREAMING, BBConstants.BB_STREAM_IQ )
	signalHound.StartRawCapture( destFileName = 'raw_capture.bin' )

def DoRegularSweep( signalHound ):
	centerFrequency = 500 * MHZ
	span 			= 20  * MHZ
	sweepResolution = 1   * KHZ
	referenceLevel  = -10
	numOfSweeps 	= 200
	dataFileName 	= 'sweeps.bin'

	signalHound.SetSweepParams_byCenter( centerFrequency, \
										 span, 			  \
										 sweepResolution, \
										 referenceLevel  )
	maxEnerey 		= signalHound.GetMaxEnergy()
	referenceLevel 	= round( maxEnerey + 10 )
	print( "Max energy = %f; Setting reference level to %f" % ( maxEnerey, referenceLevel ) )
	signalHound.SetSweepParams_byCenter( centerFrequency, \
										 span, 			  \
										 sweepResolution, \
										 referenceLevel  )

	# maxEnerey 		= signalHound.GetMaxEnergy()
	# print( "Max energy = %f; Setting reference level to %f" % ( maxEnerey, referenceLevel ) )

	signalHound.StartSweeping( numOfSweeps, dataFileName )

def DoConsecutiveSweep( signalHound, startFrequency, endFrequency, stepSize, destFileNameBase ):
	NO_PARTIAL_SWEEPS_YET = -1

	sweepString 				 = "from-%dMHz-to-%dMHz_at_steps_of_%dMhz" \
									% (startFrequency / MHZ, endFrequency / MHZ, stepSize / MHZ )
	sweepDirectory 				 = CreateSweepDirectory( sweepString )

	halfStepSize 		 = stepSize / 2
	allCenterFrequencies = range( int( startFrequency + halfStepSize ), \
								  int( endFrequency + 1 ), \
								  int( stepSize ) )

	UpdateIndicatorFile( sweepDirectory, NO_PARTIAL_SWEEPS_YET, doneSweeping = False )
	for frequencyIdx, centerFrequency in enumerate( allCenterFrequencies ):
		DoPartialSweep(  signalHound,		\
						 frequencyIdx, 		\
						 centerFrequency, 	\
						 stepSize, 			\
						 sweepDirectory, 	\
						 destFileNameBase)
		UpdateIndicatorFile( sweepDirectory, frequencyIdx, doneSweeping = False )

	UpdateIndicatorFile( sweepDirectory, frequencyIdx, doneSweeping = True )

def	DoConsecutiveSweepOfRawIQ( realTimeSignalHound, startFrequency, endFrequency, stepSize, overlapMargin ):
	NO_PARTIAL_SWEEPS_YET = -1

	sweepString 		 = "from-%dMHz-to-%dMHz_at_steps_of_%dMhz" \
									% (startFrequency / MHZ, endFrequency / MHZ, ( stepSize ) / MHZ )
	sweepDirectory 		 = CreateSweepDirectory( sweepString )

	if endFrequency - startFrequency < stepSize:
		endFrequency = startFrequency + stepSize
	halfStepSize 		 = stepSize / 2
	allCenterFrequencies = range( int( startFrequency + halfStepSize ), \
								  int( endFrequency + 1 ), \
								  int( stepSize ) )

	# referenceLevel = EstimateReferenceLevel( realTimeSignalHound, startFrequency, endFrequency )

	realTimeSignalHound.ConfigureIQ(   Decimation.SAMPLE_RATE_40_MS_MAX_BANDWIDTH_27MHZ, 27*MHZ )
	realTimeSignalHound.InitiateSweep( BBConstants.BB_STREAMING, BBConstants.BB_STREAM_IQ )

	UpdateIndicatorFile( sweepDirectory, NO_PARTIAL_SWEEPS_YET, doneSweeping = False )
	for frequencyIdx, centerFrequency in enumerate( allCenterFrequencies ):
		referenceLevel = EstimateReferenceLevel( realTimeSignalHound, centerFrequency - halfStepSize, centerFrequency + halfStepSize )

		while True:
			try:
				DoPartialSweepRawIQ( realTimeSignalHound,			\
									 frequencyIdx, 					\
									 centerFrequency, 				\
									 stepSize + 2 * overlapMargin, 	\
									 referenceLevel ,				\
									 sweepDirectory 				\
				)
				break;

			except SignalHoundError as e:
				if e.errorCode == BBStatus.bbADCOverflow:
					print( "ADC overflow, re-adjusting reference level" )
					referenceLevel += 5
				else:
					raise

		UpdateIndicatorFile( sweepDirectory, frequencyIdx, doneSweeping = False )

	UpdateIndicatorFile( sweepDirectory, frequencyIdx, doneSweeping = True )

def UpdateIndicatorFile( sweepDirectory, frequencyIdx, doneSweeping ):
	INDICATOR_FILE_NAME = 'last_partial_sweep_index.txt'

	f = open( sweepDirectory + '/' + INDICATOR_FILE_NAME, 'w' )
	f.write( "%d\n" % frequencyIdx )

	if doneSweeping:
		f.write( "1")
	else:
		f.write( "0" )

	f.close()

def DoPartialSweep(  signalHound, 		\
					 frequencyIdx, 		\
					 centerFrequency, 	\
					 stepSize, 			\
					 sweepDirectory, 	\
					 destFileNameBase):

	sweepResolution = 1 * KHZ
	referenceLevel  = -10
	numOfSweeps 	= 200
	halfStepSize  	= stepSize / 2

	sys.stdout.write( "Sweep %04d: Scanning %.2fMHz - %.2fMHz. " \
		% (frequencyIdx, 						\
		   (centerFrequency - halfStepSize) / MHZ,  	\
		   (centerFrequency + halfStepSize) / MHZ ) )

	signalHound.SetSweepParams_byCenter( 	 centerFrequency, \
											 stepSize, 		  \
											 sweepResolution, \
											 referenceLevel  )
	maxEnerey 		= signalHound.GetMaxEnergy()
	referenceLevel 	= round( maxEnerey + 10 )
	sys.stdout.write( "Max energy = %.2f; ref level <-- %.1f. " % ( maxEnerey, referenceLevel ) )
	signalHound.SetSweepParams_byCenter( centerFrequency, \
										 stepSize, 		  \
										 sweepResolution, \
										 referenceLevel  )

	sweepFilePath = sweepDirectory + "/" 		+ \
					destFileNameBase 	    	+ \
					( "%04d" % frequencyIdx ) 	+ \
					".bin"

	startTime = time.time()

	try:
		signalHound.StartSweeping( numOfSweeps, sweepFilePath )
	except SignalHoundError as e:
		print( "Error code: %d" % e.errorCode )
	sys.stdout.write( "Took %.2f seconds\n" % (time.time() - startTime) )

@LogCall
def EstimateReferenceLevel( signalHound, startFrequency, endFrequency ):
	centerFrequency   = (startFrequency + endFrequency) / 2
	span 		      = endFrequency - startFrequency

	sweepResolution   = 1000 * KHZ
	referenceLevel    = -10
	newReferenceLevel = -100

	while True:
		signalHound.SetSweepParams_byCenter( 	 centerFrequency, \
												 span, 	  		  \
												 sweepResolution, \
												 referenceLevel  )
		maxEnerey 		   = signalHound.GetMaxEnergy()
		newReferenceLevel  = round( maxEnerey )
		# print( "referenceLevel = %d. newReferenceLevel = %d" % (referenceLevel, newReferenceLevel))
		# sys.stdout.write( "Max energy = %.2f; ref level <-- %.1f. \n" % ( maxEnerey, newReferenceLevel ) )

		if abs( newReferenceLevel - referenceLevel ) < 10:
			break

		referenceLevel = newReferenceLevel

	return  newReferenceLevel


def DoPartialSweepRawIQ( 	 signalHound,		\
							 frequencyIdx, 		\
							 centerFrequency, 	\
							 span, 			    \
							 referenceLevel,    \
							 sweepDirectory, 	\
):
	SWEEP_BASE_FILENAME = "rawIQ"
	halfSpanSize  		= span / 2

	sys.stdout.write( "Sweep %04d: Scanning %.2fMHz - %.2fMHz @ reference = %d dBm " \
		% (frequencyIdx, 						\
		   (centerFrequency - halfSpanSize ) / MHZ,  	\
		   (centerFrequency + halfSpanSize ) / MHZ,   \
		   referenceLevel ) )

	sweepFilePath  = sweepDirectory + "/" 		+ \
					 SWEEP_BASE_FILENAME 	    + \
					 ( "%04d" % frequencyIdx ) 	+ \
				 	".bin"

	signalHound.SetupRawIQCapture( 	centerFrequency, 					\
								 	span,								\
								 	referenceLevel,						\
								 	Decimation.SAMPLE_RATE_40_MS_MAX_BANDWIDTH_27MHZ )

	startTime = time.clock()
	signalHound.StartRawCapture( destFileName = sweepFilePath )
	sys.stdout.write( "Took %.2f seconds\n" % (time.clock() - startTime) )

def CreateSweepDirectory( sweepString ):
	BASE_DIRECTORY 				 = "consecutive_sweeps/"
	ts 						   	 = time.time();
	timestamp 				   	 = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d_%H-%M-%S')
	sweepDirectory 				 = BASE_DIRECTORY + timestamp + sweepString

	print( "Creating directory " + sweepDirectory )
	os.makedirs( sweepDirectory )

	return sweepDirectory


def DoRealTimeSweep( signalHound ):
	centerFrequency = 400 * MHZ
	span 			= 20 * MHZ
	sweepResolution = 3   * KHZ
	referenceLevel  = -10

	signalHound.SetSweepParams_byCenter( centerFrequency, \
										 span, 			  \
										 sweepResolution, \
										 referenceLevel,  \
										 SweepMode.RealTime   )
	signalHound.StartRealTimeSweep()

def ConfigTrackingGenerator( signalHound ):
	frequency = 400 * MHZ
	amplitude = -10
	signalHound.ConnectToTrackingGenerator()
	signalHound.ConfigTrackingGenerator( frequency, amplitude )


def main():
	# signalHound = SignalHound()
	# signalHound.Connect()


	realTimeSignalHound = RealTimeSignalHound();
	print( "API version: %s" % realTimeSignalHound.GetAPIVersion() )

	realTimeSignalHound.Connect()

	try:
		pass
		# DoRegularSweep( signalHound )
		# DoRealTimeSweep( signalHound )
		# DoRegularSweep( realTimeSignalHound )
		# DoRealTimeSweep( realTimeSignalHound )
		# ConfigTrackingGenerator( signalHound )

		# startFrequency = 450 * MHZ
		# endFrequency   = 550 * MHZ
		# stepSize 	   = 20  * MHZ
		# DoConsecutiveSweep( realTimeSignalHound, startFrequency, endFrequency, stepSize, "sweep")

		print( "Firmware version: %s" % realTimeSignalHound.GetFirmwareVersion() )
		# print( "API version: %s" % realTimeSignalHound.GetAPIVersion())

		# GetRawIQ( realTimeSignalHound )
		stepSize 	   = 26.5  * MHZ
		startFrequency = 10  * MHZ
		endFrequency   = 1000 * MHZ
		overlapMargin  = 0.25 * MHZ

		DoConsecutiveSweepOfRawIQ( realTimeSignalHound, startFrequency, endFrequency, stepSize, overlapMargin )


	finally:
		# signalHound.Disconnect()
		realTimeSignalHound.Disconnect()

if __name__ == '__main__':
	main()
