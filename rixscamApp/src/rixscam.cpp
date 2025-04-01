/* xcamCamera.cpp
 *
 * This is a driver for an XCAM camera.
 *
 * Author: Phil Atkin
 *         Pixel Analytics Ltd.
 *
 * Created:  12 December 2015
 *
 */

#define _USE_MATH_DEFINES // for C++
#include <cmath>

#include <stddef.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <cstdlib>
#include <fstream>      // std::ifstream

#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsEvent.h>
#include <epicsMutex.h>
#include <epicsString.h>
#include <epicsStdio.h>
#include <epicsMutex.h>
#include <epicsExit.h>
#include <cantProceed.h>
#include <iocsh.h>

#include <vector>
#include <chrono>
#include <thread>
#include <algorithm> // for max, min
using namespace std;

#include "ADDriver.h"
#include <epicsExport.h>
#include "rixscam.h"

// Wrapper class for epicsMutex where destruction releases the mutex
class MutexLocker
{
public:
	MutexLocker(epicsMutex& mutex) :
		_mutex(mutex)
	{
		_mutex.lock();
	}

	~MutexLocker()
	{
		_mutex.unlock();
	}

private:
	epicsMutex& _mutex;
};

// Forward declaration of C-linkage functions
static void c_shutdown(void *arg);
static void c_imageTask(void *drvPvt);
static void c_temperatureTask(void *drvPvt);

const char* xcamCamera::_driverName = "xcamCamera";

#pragma region Constructor

/** Constructor for xcamCamera; most parameters are simply passed to ADDriver::ADDriver.
* After calling the base class constructor this method creates a thread to compute the simulated detector data,
* and sets reasonable default values for parameters defined in this class, asynNDArrayDriver and ADDriver.
* \param[in] portName The name of the asyn port driver to be created.
* \param[in] maxSizeX The maximum X dimension of the images that this driver can create.
* \param[in] maxSizeY The maximum Y dimension of the images that this driver can create.
* \param[in] maxBuffers The maximum number of NDArray buffers that the NDArrayPool for this driver is
*            allowed to allocate. Set this to -1 to allow an unlimited number of buffers.
* \param[in] maxMemory The maximum amount of memory that the NDArrayPool for this driver is
*            allowed to allocate. Set this to -1 to allow an unlimited amount of memory.
* \param[in] priority The thread priority for the asyn port driver thread if ASYN_CANBLOCK is set in asynFlags.
* \param[in] stackSize The stack size for the asyn port driver thread if ASYN_CANBLOCK is set in asynFlags.
*/
xcamCamera::xcamCamera(const char *portName, int maxSizeX, int maxSizeY,
	int maxBuffers, size_t maxMemory, int priority, int stackSize)

	: ADDriver(portName, 1, NUM_XCAM_CAMERA_PARAMS + _parameterCount, maxBuffers, maxMemory,
	0, 0, /* No interfaces beyond those set in ADDriver.cpp */
	ASYN_CANBLOCK, 1, /* ASYN_CANBLOCK=2, ASYN_MULTIDEVICE=0, autoConnect=1 */
	priority, stackSize),
	_newImageRequired(true),
	_sequencerFilenameChanged(true),
	_CCDPowerChanged(true),
	_exiting(false),
	pRaw(NULL),
	_roiParametersChanged(true), // true so that parameters get initialized on first capture
	_voltageParamsChanged(true), // true so that voltages get initialized on first capture
	_tempControllerParamsChanged(true), // so temp controller gets initialized ASAP
	_TriggerModeChanged(true),
	_SequencerParametersChanged(true),
	_acquireTimeChanged(true),
	_adcGainOffsetChanged(true),
	_shutterModeChanged(true),
	_shutterDelayChanged(true),
	_callGrabSetup(true),
	_switchModeCheck(false),
	_grabWaitFlag(false),
	_grabWaitValue(0)
{
	int status = asynSuccess;
	const char *functionName = "xcamCamera";

	/* Create the epicsEvents for signaling to the simulate task when acquisition starts and stops */
	this->startEventId = epicsEventCreate(epicsEventEmpty);
	if (!this->startEventId) {
		asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
			"%s:%s: epicsEventCreate failure for start event\n",
			_driverName, functionName);
		return;
	}
	this->stopEventId = epicsEventCreate(epicsEventEmpty);
	if (!this->stopEventId) {
		asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
			"%s:%s: epicsEventCreate failure for stop event\n",
			_driverName, functionName);
		return;
	}

	/* Set some default values for parameters */
	status = setStringParam(ADManufacturer, "XCAM");
	status |= setStringParam(ADModel, "RIXSCam");
	status |= setIntegerParam(ADMaxSizeX, maxSizeX);
	status |= setIntegerParam(ADMaxSizeY, maxSizeY);
	status |= setIntegerParam(ADMinX, 0);
	status |= setIntegerParam(ADMinY, 0);
	status |= setIntegerParam(ADBinX, 1);
	status |= setIntegerParam(ADBinY, 1);
	status |= setIntegerParam(ADSizeX, maxSizeX);
	status |= setIntegerParam(ADSizeY, maxSizeY);
	status |= setIntegerParam(NDArraySizeX, maxSizeX);
	status |= setIntegerParam(NDArraySizeY, maxSizeY);
	status |= setIntegerParam(NDArraySize, 0);

	// Data type, colour mode and reversals are fixed
	status |= setIntegerParam(NDDataType, NDUInt16);
	status |= setIntegerParam(NDColorMode, NDColorModeMono);
	status |= setIntegerParam(ADReverseX, 0);
	status |= setIntegerParam(ADReverseY, 0);
	// ... as are these
	status |= setIntegerParam(NDArraySizeZ, 0);
	status |= setStringParam(ADStringToServer, "<not used by driver>");
	status |= setStringParam(ADStringFromServer, "<not used by driver>");

	status |= setIntegerParam(ADImageMode, ADImageContinuous);
	status |= setDoubleParam(ADAcquireTime, .001);
	status |= setDoubleParam(ADAcquirePeriod, .005);
	status |= setIntegerParam(ADNumImages, 100);

	// Explicitly create and set string variables, for which we have no Parameter support
	createParam("SOFT_VERSION", asynParamOctet, &SoftVersion);
	setStringParam(SoftVersion, "Unknown");
	createParam("IFS_VERSION", asynParamOctet, &IFSVersion);
	setStringParam(IFSVersion, "Unknown");
	createParam("FPGA_VERSION", asynParamOctet, &FPGAVersion);
	setStringParam(FPGAVersion, "Unknown");
	createParam("CAM_SERIAL", asynParamOctet, &CamSerial);
	setStringParam(CamSerial, "Unknown");
	createParam("SEQ_FILENAME", asynParamOctet, &SeqFilename);

    // Origionally included from file written by PVGenerator.
    // Coppied into file by S.B. Wilkins
    // from "xcamCameraIOC\PVDefinitions.cpp"

    _allParams = {
        &_paramSEQ_ADC_DELAY,
        &_paramSEQ_INT_MINUS_DELAY,
        &_paramSEQ_INT_PLUS_DELAY,
        &_paramSEQ_INT_TIME,
        &_paramSEQ_SERIAL_T,
        &_paramSEQ_PARALLEL_T,
        &_paramSEQ_SERIAL_CLOCK,
        &_paramSEQ_PARALLEL_CLOCK,
        &_paramSEQ_NODE_SELECTION,
        &_paramSEQ_STATUS,
        &_paramVOLT_BIAS_OD,
        &_paramVOLT_BIAS_RD,
        &_paramVOLT_BIAS_DD,
        &_paramVOLT_BIAS_OG,
        &_paramVOLT_BIAS_SS,
        &_paramVOLT_BIAS_HVDC,
        &_paramVOLT_BIAS_PEDESTAL,
        &_paramVOLT_BIAS_HV,
        &_paramVOLT_CLOCK_IMAGE,
        &_paramVOLT_CLOCK_STORE,
        &_paramVOLT_CLOCK_SERIAL,
        &_paramVOLT_CLOCK_RESET,
        &_paramCCD_POWER,
        &_paramRIXS_SIMULATION,
        &_paramRIXS_EVENTSPERFRAME,
        &_paramRIXS_BACKGROUNDLEVEL,
        &_paramRIXS_EVENTHEIGHT,
        &_paramRIXS_EVENTRADIUS,
        &_paramTEMP_PROP_GAIN,
        &_paramTEMP_INT_GAIN,
        &_paramTEMP_DERIV_GAIN,
        &_paramTEMP_PROP_RATE,
        &_paramTEMP_INT_RATE,
        &_paramTEMP_DERIV_RATE,
        &_paramTEMP_ACCUMULATED_ERROR_LIMIT,
        &_paramTEMP_OUTPUT_BIAS,
        &_paramTEMP_MANUAL_MODE,
        &_paramTEMP_ENABLE,
        &_paramTEMP_HEATER_SELECT,
        &_paramTEMP_SENSOR_SELECT,
        &_paramCCD_COUNT,
        &_paramADC_GAIN,
        &_paramADC_OFFSET,
    };

	for (auto param : _allParams)
		status |= param->Initialize(*this);

	if (status) {
		asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
			"%s:%s: unable to set camera parameters (%d)\n",
			_driverName, functionName, status);
		return;
	}

	// Specific collection for the voltage settings
	// N.B. THE ORDER HERE IS THE ORDER IN WHICH THE VOLTAGES ARE APPLIED (i.e. turned on).
	_voltageParams = {
		&_paramVOLT_BIAS_OD,
		&_paramVOLT_BIAS_RD,
		&_paramVOLT_BIAS_DD,
		&_paramVOLT_BIAS_OG,
		&_paramVOLT_BIAS_SS,
		&_paramVOLT_BIAS_HVDC,
		&_paramVOLT_BIAS_PEDESTAL,
		&_paramVOLT_BIAS_HV,
		&_paramVOLT_CLOCK_IMAGE,
		&_paramVOLT_CLOCK_STORE,
		&_paramVOLT_CLOCK_SERIAL,
		&_paramVOLT_CLOCK_RESET,
	};

	// Specific collection for the temperature controller settings
	// (N.B. excludes _paramTEMP_HEATER_SELECT and _paramTEMP_SENSOR_SELECT because they
	// are set using a different mechanism)
	_tempControllerParams = {
		&_paramTEMP_PROP_GAIN,
		&_paramTEMP_INT_GAIN,
		&_paramTEMP_DERIV_GAIN,
		&_paramTEMP_PROP_RATE,
		&_paramTEMP_INT_RATE,
		&_paramTEMP_DERIV_RATE,
		&_paramTEMP_ACCUMULATED_ERROR_LIMIT,
		&_paramTEMP_OUTPUT_BIAS,
		&_paramTEMP_MANUAL_MODE,
		&_paramTEMP_ENABLE
	};

	// Collection of simple sequencer parameters that can be directly set
	_sequencerParams = {
		&_paramSEQ_ADC_DELAY,
		&_paramSEQ_INT_MINUS_DELAY,
		&_paramSEQ_INT_PLUS_DELAY,
		&_paramSEQ_INT_TIME,
		&_paramSEQ_SERIAL_T,
		&_paramSEQ_PARALLEL_T,
		&_paramSEQ_SERIAL_CLOCK,
		&_paramSEQ_PARALLEL_CLOCK,
	};
	// N.B. does not include _paramSEQ_NODE_SELECTION because that has to be set specially

	// Set of parameter indices that, if changed, require a new ROI to be set up
	// N.B. These variables are all currently accessed through writeInt32
	_roiParameterIndices = {
		ADBinX,
		ADBinY,
		ADMinX,
		ADMinY,
		ADSizeX,
		ADSizeY,
		ADMaxSizeX,
		ADMaxSizeY
	};

	// shutdown on exit
	epicsAtExit(c_shutdown, this);

	///////////////////////////////////////////////////////////////////////////////
	// Camera initialization

	// Take the xcmclm mutex, just in case
	{
		MutexLocker lock(_xcmclmMutex); // Destructor releases

#pragma __Note__("Logging turned on!")
		xcm_clm_logging(true);

		char buffer[256];
		xcm_clm_dll_version(buffer);
		setStringParam(SoftVersion, buffer);

		int serialNumbers[MAXCHAN];
		int found;
		int result = xcm_clm_discover(serialNumbers, MAXCHAN, &found);
		if ((result != XE_OK) | (found == 0))
		{
			asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
				"%s:%s: xcm_clm_discover failure (%d)\n",
				_driverName, functionName, result);
			return;
		}

		if (found > _ccdCountMax)
		{
			// We found more CCD interfaces than the maximum permitted.  Continue, but truncate the the maximum
			// and report an error
			asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
				"%s:%s: xcm_clm_discover found too many CCD interfaces (%d); continuing with only %d\n",
				_driverName, functionName, found, _ccdCountMax);

			found = _ccdCountMax;
		}

		// Build the vector of used serial numbers
		_serialNumbers.assign(serialNumbers, serialNumbers + found);
		// .. and the ccd image buffers
		_ccdImages.resize(found, nullptr);

		int located;
		vector<int> errors(found);
		result = xcm_clm_init(&_serialNumbers.front(), &errors.front(), found, &located);
		if ((result != XE_OK) || (located != found))
		{
			asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
				"%s:%s: xcm_clm_init failure (%d)\n",
				_driverName, functionName, result);
			return;
		}

		bool foundError = false;
		for (int i = 0; i < found; ++i)
		{
			if (errors[i] != XE_OK)
			{
				asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
					"%s:%s: xcm_clm_init reports error %i for camera serial %i\n",
					_driverName, functionName, errors[i], _serialNumbers[i]);
				foundError = true;
			}

			if (foundError)
				return;
		}

		asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
			"%s:%s: Discovered %i sensor interfaces and initialized them all\n",
			_driverName, functionName, found);

		// Report the number of CCDs we found
		_paramCCD_COUNT.SetValue(*this, found);

		// Default power-on cache state
		_ccdPowerOn = true;

		// Switch it all off, quick!  (In case IOC crashes and is then restarted)
		// Set CCD power to off (will also disapply voltages)
		SetCCDPower(false, true);

		///////////////////////////////////////////////////////////////////////////////

		// Obtain some information strings
		xcm_clm_get_cam_serial_number(_serialNumbers[0], buffer);
		setStringParam(CamSerial, buffer);
		xcm_clm_ifsver(_serialNumbers[0], buffer);
		setStringParam(IFSVersion, buffer);
		xcm_clm_fpgaver(_serialNumbers[0], buffer);
		setStringParam(FPGAVersion, buffer);

		// Initialize the SPI bus
		xcm_clm_initialise_spi(_serialNumbers[0]);

		// Call the xcm_clm_pulse command to initialise the GPIO on the frame grabber card
		xcm_clm_pulse(_serialNumbers[0], 0, 50, 50);
	} // Release the xcmclm lock

	// The sequencer filename is never initialized at this point, so this is the most we can achieve
	// Sequencer gets loaded (and power applied) when the parameters are set

	// Create the thread that updates the images (from camera or simulation)
	status = (epicsThreadCreate("xcamCameraImageTask",
		epicsThreadPriorityMedium,
		epicsThreadGetStackSize(epicsThreadStackMedium),
		(EPICSTHREADFUNC)c_imageTask,
		this) == NULL);
	if (status) {
		asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
			"%s:%s: epicsThreadCreate failure for image task (%d)\n",
			_driverName, functionName, status);
		return;
	}

	// Create the thread that updates the 'actual' temperatures
	status = (epicsThreadCreate("xcamCameraTemperatureTask",
		epicsThreadPriorityMedium,
		epicsThreadGetStackSize(epicsThreadStackMedium),
		(EPICSTHREADFUNC)c_temperatureTask,
		this) == NULL);
	if (status) {
		asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
			"%s:%s: epicsThreadCreate failure for temperature task (%d)\n",
			_driverName, functionName, status);
		return;
	}
}

#pragma endregion


/** Controls the shutter */
void xcamCamera::setShutter(int open)
{
    int shutterMode;

    getIntegerParam(ADShutterMode, &shutterMode);
    if (shutterMode == ADShutterModeDetector) {
        // Simulate a shutter by just changing the status readback
		// (We can't instantaneously force our shutter open or closed; it happens automatically
		// for the duration of the acquisition)
        setIntegerParam(ADShutterStatus, open);
    } else {
        /* For no shutter or EPICS shutter call the base class method */
        ADDriver::setShutter(open);
    }
}

#pragma region Image update task

// If n is odd, add 1 to n and return true.  Else if n is even, return false
bool RoundUpToEven(int& n)
{
	if ((n & 1) != 0)
	{
		n++;
		return true;
	}
	else
	{
		return false;
	}
}

// Acquire updated image, either by simulation or by acquisition from camera
NDArray* xcamCamera::GetImage()
{
	int status = asynSuccess;
	int binX, binY, minX, minY, sizeX, sizeY;
	int maxSizeX, maxSizeY;
	const int xDim = 0, yDim = 1;
	const int ndims = 2;
	const char* functionName = "UpdateImage";

	/* NOTE: The caller of this function must have taken the mutex */

	status |= getIntegerParam(ADBinX, &binX);
	status |= getIntegerParam(ADBinY, &binY);
	status |= getIntegerParam(ADMinX, &minX);
	status |= getIntegerParam(ADMinY, &minY);
	status |= getIntegerParam(ADSizeX, &sizeX);
	status |= getIntegerParam(ADSizeY, &sizeY);
	status |= getIntegerParam(ADMaxSizeX, &maxSizeX);
	status |= getIntegerParam(ADMaxSizeY, &maxSizeY);

	if (status) asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
		"%s:%s: error getting parameters\n", _driverName, functionName);

	if (_roiParametersChanged)
	{
		/* Make sure parameters are consistent, fix them if they are not */
		if (binX < 1) {
			binX = 1;
			status |= setIntegerParam(ADBinX, binX);
		}
		if (binY < 1) {
			binY = 1;
			status |= setIntegerParam(ADBinY, binY);
		}
		if (minX < 0) {
			minX = 0;
			status |= setIntegerParam(ADMinX, minX);
		}
		if (minY < 0) {
			minY = 0;
			status |= setIntegerParam(ADMinY, minY);
		}
		if (minX > maxSizeX - 1) {
			minX = maxSizeX - 1;
			status |= setIntegerParam(ADMinX, minX);
		}
		if (minY > maxSizeY - 1) {
			minY = maxSizeY - 1;
			status |= setIntegerParam(ADMinY, minY);
		}

		// N.B. the 'size' parameters are in sensor pixels (i.e. unbinned pixels)
		if (minX + sizeX > maxSizeX) {
			sizeX = maxSizeX - minX;
			status |= setIntegerParam(ADSizeX, sizeX);
		}

		// Special case for minY parameter. It also determines the switch point for dual node readout which cannot be windowed
		// Only execute code below if we are reading out of the LS or HR node don't excute code if we are in dual node readout
		if ((short)_paramSEQ_NODE_SELECTION.Value(*this) < 2)
		{
			if (minY + sizeY > maxSizeY) {
				sizeY = maxSizeY - minY;
				status |= setIntegerParam(ADSizeY, sizeY);
			}
		}

#pragma __Note__("It may be beneficial to constrain sizeX/Y to be a multiple of binX/Y")
	}

	NDArray *pImage = nullptr;
	if (_paramRIXS_SIMULATION.Value(*this))
	{
		if (_newImageRequired) {
			/* Free the previous raw buffer */
			if (this->pRaw)
				this->pRaw->release();
			/* Allocate the raw buffer we use to compute images. */
			size_t dims[2];
			dims[xDim] = maxSizeX;
			dims[yDim] = maxSizeY;
			this->pRaw = this->pNDArrayPool->alloc(ndims, dims, NDUInt16, 0, NULL);

			if (!this->pRaw) {
				asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
					"%s:%s: error allocating raw buffer\n", _driverName, functionName);
				return nullptr;
			}

			_newImageRequired = false;
		}

		// Compute the simulation
		status |= computeRIXSArray(maxSizeX, maxSizeY);

		/* Extract the region of interest with binning.
		* If the entire image is being used (no ROI or binning) that's OK because
		* convertImage detects that case and is very efficient */
		NDDimension_t dimsOut[2];
		this->pRaw->initDimension(&dimsOut[xDim], sizeX);
		this->pRaw->initDimension(&dimsOut[yDim], sizeY);
		dimsOut[xDim].binning = binX;
		dimsOut[xDim].offset = minX;
		dimsOut[xDim].reverse = 0;
		dimsOut[yDim].binning = binY;
		dimsOut[yDim].offset = minY;
		dimsOut[yDim].reverse = 0;

		status = this->pNDArrayPool->convert(this->pRaw, &this->pArrays[0], NDUInt16, dimsOut);

		if (status) {
			asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
				"%s:%s: error allocating buffer in convert()\n", _driverName, functionName);
			return nullptr;
		}

		pImage = this->pArrays[0];
	}
	else if (!_serialNumbers.empty())
	{
		// Actually acquire from the camera
		int result;

		vector<int> errors(_serialNumbers.size(), XE_OK);

		int imageSizeX = ((sizeX-16) / binX) + 16;	// Take account of 16 pixel underscan which is always there and not binned
		RoundUpToEven(imageSizeX); // Framegrabber requires that delivered image size is even
		int imageSizeY = sizeY / binY;
		RoundUpToEven(imageSizeY); // Framegrabber requires that delivered image size is even

		size_t dims[2];
		size_t ccdCount = _serialNumbers.size();

		// Get the trigger mode
		int triggerMode;
		getIntegerParam(ADTriggerMode, &triggerMode);

		// Set/get camera parameters
		callParamCallbacks();

		// (The following would be much more efficient if grab_setup would accept a 'stride' parameter,
		// so that all the images could be acquired into the same, final, buffer)

		// Below is the original code that was found on the Register
		for (size_t ccd = 0; ccd < ccdCount; ++ccd)
		{
#pragma __Note__("Force grab_setup, even when ROI unchanged (bug in xcmclm?)")
			if (_callGrabSetup)
			{
				// Release any previous buffer
				if (_ccdImages[ccd] != nullptr)
					_ccdImages[ccd]->release();

				// Set the acquisition timeout to twice the integration time, plus 10 seconds
				double acquireTime;
				getDoubleParam(ADAcquireTime, &acquireTime);
				long timeoutMs = (long)((acquireTime * 2.0 + 10.0) * 1000.0);
				result = xcm_clm_set_timeout(_serialNumbers[ccd], timeoutMs);

				// The image is smaller than sizeX/Y by the factors binX/Y
				dims[xDim] = imageSizeX;
				dims[yDim] = imageSizeY;
				_ccdImages[ccd] = pNDArrayPool->alloc(2, dims, NDUInt16, 0, NULL);

				// Set up the grab
				// Note that sizeX and sizeY have to be passed twice, and reversed the second time!
				// Also that the top/left are passed as zero; non-zero values would invoke a 'software'
				// ROI (we want a 'hardware' ROI, as defined by the sequencer registers)
				result = xcm_clm_grab_setup_1node(_serialNumbers[ccd],
					imageSizeX, imageSizeY, 0, 0, imageSizeY, imageSizeX, (BYTE*)_ccdImages[ccd]->pData,
					(int)_paramSEQ_NODE_SELECTION.Value(*this) + 1);

			}
		}

		if (triggerMode < 3)
			// Call the xcm_clm_pulse command to initialise the GPIO on the frame grabber card
			xcm_clm_pulse(_serialNumbers[0], 0, 50, 50);

		// Grab from _all_ interfaces simultaneously
		result = xcm_clm_grab(&_serialNumbers.front(), &errors.front(), (int)ccdCount);

		// Settings now reflect the ROI parameters
		_roiParametersChanged = false;
		_SequencerParametersChanged = false;
		_acquireTimeChanged = false;

		if (result)
		{
			asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
				"%s:%s: error grabbing image\n", _driverName, functionName);
			return nullptr;
		}

		// Now, assemble the final image from the components
		// It has an image for each ccd, with a one-pixel black (zero) border around each
		dims[xDim] = ccdCount * (imageSizeX + 2);
		dims[yDim] = imageSizeY + 2;
		pImage = pNDArrayPool->alloc(2, dims, NDUInt16, 0, NULL);

		// Copy all the data
		unsigned short* pImageData = (unsigned short*)pImage->pData;
		size_t ccdRowBytes = imageSizeX * sizeof(unsigned short);

		// Write the top black borders
		memset((void *)pImageData, 0, dims[xDim] * sizeof(unsigned short));
		pImageData += dims[xDim];

		for (size_t y = 0; y < imageSizeY; ++y)
		{
			for (size_t ccd = 0; ccd < ccdCount; ++ccd)
			{
				// Initial black border
				*pImageData++ = 0;

				// Transfer real image data
				memcpy((void *)pImageData, (void *)((unsigned short *)_ccdImages[ccd]->pData + y * imageSizeX), ccdRowBytes);
				pImageData += imageSizeX;

				// Final black border
				*pImageData++ = 0;
			}
		}

		// pImageData points to final row
		memset((void *)pImageData, 0, dims[xDim] * sizeof(unsigned short));

		// We do _not_ free the buffers for the individual CCDs here because their addresses have been passed
		// to xcm_clm_grab_setup, and they are expected to stay there unless/until the ROI parameters change
	}
	else
	{
		asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
			"%s:%s: Want to read camera, but none is connected\n", _driverName, functionName);
		return nullptr;
	}

	NDArrayInfo_t arrayInfo;
	pImage->getInfo(&arrayInfo);

	status = asynSuccess;
	status |= setIntegerParam(NDArraySize, (int)arrayInfo.totalBytes);
	status |= setIntegerParam(NDArraySizeX, (int)pImage->dims[xDim].size);
	status |= setIntegerParam(NDArraySizeY, (int)pImage->dims[yDim].size);

	if (status)
	{
		asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
			"%s:%s: error setting parameters\n", _driverName, functionName);
		return nullptr;
	}

	return pImage;
}

static void c_imageTask(void *drvPvt)
{
	xcamCamera *pPvt = (xcamCamera *)drvPvt;

	pPvt->imageTask();
}

// The image acquisition (simulation or camera) task
void xcamCamera::imageTask()
{
	bool acquire = false;
	const char *functionName = "imageTask";

	this->lock();
	/* Loop forever */
	while (!_exiting)
	{
		if (!acquire)
		{
			epicsEventStatus startEventStatus = epicsEventWaitTimeout;
			while (startEventStatus != epicsEventOK)
			{
				startEventStatus = preAcquisition();
			}

			// We saw the semaphore, so should do a real acquisition
			acquire = true;
		}

		setIntegerParam(ADAcquire, acquire);
		setStringParam(ADStatusMessage, "Acquiring data");
		setIntegerParam(ADNumImagesCounter, 0);
		callParamCallbacks();

		bool acquisitionDone = false;

		/* Lock the mutex during the entire acquisition so the performance isn't
		 * affected by concurrent tasks - i.e. temperature reading. */
		{
			MutexLocker lock(_xcmclmMutex);
			do
			{
				doAcquisition(acquisitionDone);
			}
			while(!acquisitionDone);

			acquire = false;
			setIntegerParam(ADAcquire, acquire);
		}
		callParamCallbacks();
	}
}


// If we are not acquiring then update the camera state and wait for a
// semaphore that is given when acquisition is started
// Returns the status of the semaphore waiting operation.
epicsEventStatus xcamCamera::preAcquisition()
{
	const char* functionName = "preAcquisition";
	{
		MutexLocker lock(_xcmclmMutex);
		if (configureCaptureParams() != XE_OK)
		{
			asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
				"%s:%s: error configuring parameters\n", _driverName, functionName);
		}
	}
	/* Release the lock while we wait for an event that says acquire has started, then lock again */
	asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
		"%s:%s: waiting for acquire to start\n", _driverName, functionName);

	this->unlock();
	epicsEventStatus startEventStatus = epicsEventWaitWithTimeout(this->startEventId, _acquireTimeoutSeconds);
	this->lock();

	/* Call the callbacks to update any changes */
	callParamCallbacks();

	return startEventStatus;
}

void xcamCamera::doAcquisition(bool &acquisitionDone)
{
	const char* functionName = "doAcquisition";

	/* Get the current time */
	epicsTimeStamp startTime, endTime;
	double elapsedTime;
	epicsTimeGetCurrent(&startTime);
	int imageMode;
	getIntegerParam(ADImageMode, &imageMode);

	/* Get the exposure parameters */
	double acquireTime, acquirePeriod, delay;
	getDoubleParam(ADAcquireTime, &acquireTime);
	getDoubleParam(ADAcquirePeriod, &acquirePeriod);

	/* We are acquiring. */
	setIntegerParam(ADStatus, ADStatusAcquire);

	/* Open the shutter */
	setShutter(ADShutterOpen);

	callParamCallbacks();

	epicsEventStatus stopEventStatus;

	if (_paramRIXS_SIMULATION.Value(*this))
	{
		/* Simulate being busy during the exposure time.  Use epicsEventWaitWithTimeout so that
		 * manually stopping the acquisition will work */
		this->unlock();
		stopEventStatus = epicsEventWaitWithTimeout(this->stopEventId, acquireTime);
		this->lock();

		if (stopEventStatus == epicsEventWaitOK)
		{
			acquisitionDone = true;
			setIntegerParam(ADStatus, ADStatusAborted);
			callParamCallbacks();
			return;
		}
	}
	else
	{
		if (configureCaptureParams() != XE_OK)
		{
			asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
				"%s:%s: error configuring parameters\n", _driverName, functionName);
		}
	}

	setIntegerParam(ADStatus, ADStatusReadout);

	/* Update the image */
	NDArray* pImage = GetImage();

	if (pImage == nullptr) return;

	/* Close the shutter */
	setShutter(ADShutterClosed);

	/* Call the callbacks to update any changes */
	callParamCallbacks();

	/* Get the current parameters */
	int imageCounter;
	int numImages, numImagesCounter;
	int arrayCallbacks;
	getIntegerParam(NDArrayCounter, &imageCounter);
	getIntegerParam(ADNumImages, &numImages);
	getIntegerParam(ADNumImagesCounter, &numImagesCounter);
	getIntegerParam(NDArrayCallbacks, &arrayCallbacks);
	imageCounter++;
	numImagesCounter++;
	setIntegerParam(NDArrayCounter, imageCounter);
	setIntegerParam(ADNumImagesCounter, numImagesCounter);

	/* Put the frame number and time stamp into the buffer */
	pImage->uniqueId = imageCounter;
	pImage->timeStamp = startTime.secPastEpoch + startTime.nsec / 1.e9;
	updateTimeStamp(&pImage->epicsTS);

	/* Get any attributes that have been defined for this driver */
	this->getAttributes(pImage->pAttributeList);

	if (arrayCallbacks)
	{
		/* Call the NDArray callback */
		/* Must release the lock here, or we can get into a deadlock, because we can
		* block on the plugin lock, and the plugin can be calling us */
		this->unlock();
		asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
			"%s:%s: calling imageData callback\n", _driverName, functionName);
		doCallbacksGenericPointer(pImage, NDArrayData, 0);
		this->lock();
	}

	pImage->release();

	/* Call the callbacks to update any changes */
	callParamCallbacks();

	/* If we are still acquiring then sleep for the acquire period minus elapsed time. */
	epicsTimeGetCurrent(&endTime);
	elapsedTime = epicsTimeDiffInSeconds(&endTime, &startTime);
	delay = acquirePeriod - elapsedTime;
	asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
		"%s:%s: delay=%f\n", _driverName, functionName, delay);

	/* The delay can't be a negative value */
	if(delay < 0) delay = 0;

	/* We set the status to waiting to indicate we are in the period delay */
	setIntegerParam(ADStatus, ADStatusWaiting);
	callParamCallbacks();

	this->unlock();
	stopEventStatus = epicsEventWaitWithTimeout(this->stopEventId, delay);
	this->lock();

	acquisitionDone = (stopEventStatus == epicsEventWaitOK) ||
					   (imageMode == ADImageSingle) ||
					   (imageMode == ADImageMultiple && numImagesCounter >= numImages);

	if(acquisitionDone)
	{
		if (stopEventStatus == epicsEventWaitOK && imageMode == ADImageMultiple)
		{
			setIntegerParam(ADStatus, ADStatusAborted);
		}
		else
		{
			setIntegerParam(ADStatus, ADStatusIdle);
			setStringParam(ADStatusMessage, "Waiting for acquisition");
		}
		callParamCallbacks();
	}
}

// This function must be run while holding the mutex.
int xcamCamera::configureCaptureParams()
{
	int result = XE_OK, status = asynSuccess;
	int binX, binY, minX, minY, sizeX, sizeY;
	int maxSizeX, maxSizeY;
	const char* functionName = "configureCaptureParams";

	status |= getIntegerParam(ADBinX, &binX);
	status |= getIntegerParam(ADBinY, &binY);
	status |= getIntegerParam(ADMinX, &minX);
	status |= getIntegerParam(ADMinY, &minY);
	status |= getIntegerParam(ADSizeX, &sizeX);
	status |= getIntegerParam(ADSizeY, &sizeY);
	status |= getIntegerParam(ADMaxSizeX, &maxSizeX);
	status |= getIntegerParam(ADMaxSizeY, &maxSizeY);

	if(status)
	{
		asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
			"%s:%s: error getting parameters\n", _driverName, functionName);
		return XE_IOFAIL;
	}

	int imageSizeX = ((sizeX - 16) / binX) + 16;	// Take account of 16 pixel underscan which is always there and not binned
	RoundUpToEven(imageSizeX); // Framegrabber requires that delivered image size is even
	int imageSizeY = sizeY / binY;
	RoundUpToEven(imageSizeY); // Framegrabber requires that delivered image size is even

	if (_sequencerFilenameChanged)
	{
		// Read the new sequencer file
		asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
			"%s:%s: New sequencer filename setting detected\n", _driverName, functionName);

		LoadSequencer();

		// The new sequencer filename has been responded to; don't do so again
		_sequencerFilenameChanged = false;
		// Download all parameters if Sequencer file has been uploaded
		_roiParametersChanged = true;
		_SequencerParametersChanged = true;
		_acquireTimeChanged = true;
		_shutterDelayChanged = true;
		_shutterModeChanged = true;
		_callGrabSetup = true;
		_switchModeCheck = true;
	}

	if (_CCDPowerChanged)
	{
		asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
			"%s:%s: New CCD power setting detected\n", _driverName, functionName);

		SetCCDPower();
	}

	if (_voltageParamsChanged)
	{
		SetCCDVoltages();
	}

	if (_TriggerModeChanged)
	{
		// Set the trigger mode
		int triggerMode;
		getIntegerParam(ADTriggerMode, &triggerMode);
		result |= xcm_clm_set_param(_serialNumbers[0], 65, (short)triggerMode);
		_TriggerModeChanged = false;
		_switchModeCheck = true;
	}

	// Set the (global) gain and offset
	if (_adcGainOffsetChanged)
	{
		result |= xcm_clm_cds_gain(_serialNumbers[0], (short)_paramADC_GAIN.ScaledValue(*this));
		result |= xcm_clm_cds_offset(_serialNumbers[0], (short)_paramADC_OFFSET.ScaledValue(*this));
		_adcGainOffsetChanged = false;
	}

	// Node select handled specially because value to set differs from combo box selection
	if (_roiParametersChanged)
	{
		short node;
		switch ((short)_paramSEQ_NODE_SELECTION.Value(*this))
		{
		case 0:
			node = 1;
			break;
		case 1:
			node = 2;
			break;
		case 2:
			node = 4;
			break;
		case 3:
			node = 8;
			break;
		default:
			node = 1;
		}

		result |= xcm_clm_set_param(_serialNumbers[0], _paramSEQ_NODE_SELECTION.InternalIndex(), node);
		_adcGainOffsetChanged = true;
		_callGrabSetup = true;
		_switchModeCheck = true;

	}

	// Set all the sequencer parameters
	// Need to sequencer parameters if the node has been changed hence including '_roiParametersChanged' as an option
	if ((_SequencerParametersChanged) || (_roiParametersChanged))
	{
		for (auto param : _sequencerParams)
		{
			result |= xcm_clm_set_param(_serialNumbers[0], param->InternalIndex(), (short)param->Value(*this));
		}

		_SequencerParametersChanged = false;
		_callGrabSetup = true;
		_switchModeCheck = true;
	}

	if (_roiParametersChanged)
	{
		// Set the registers as required (global calls)
		result |= xcm_clm_set_param(_serialNumbers[0], 10, imageSizeX);
		result |= xcm_clm_set_param(_serialNumbers[0], 11, imageSizeY);
		result |= xcm_clm_set_param(_serialNumbers[0], 60, minX);
		result |= xcm_clm_set_param(_serialNumbers[0], 61, minY);
		result |= xcm_clm_set_param(_serialNumbers[0], 62, maxSizeX);
		result |= xcm_clm_set_param(_serialNumbers[0], 63, maxSizeY);
		result |= xcm_clm_set_param(_serialNumbers[0], 12, binX);
		result |= xcm_clm_set_param(_serialNumbers[0], 9, binY);
		_callGrabSetup = true;
		_switchModeCheck = true;

	}

	if ((_acquireTimeChanged) || (_roiParametersChanged))
	{
		SetExposureTime();
		_acquireTimeChanged = false;
		_callGrabSetup = true;
		_switchModeCheck = true;
	}

	// Enable the shutter if shutter mode is 'detector'
	// Parameter 67 set to 15 to enable, 0 to disable
	if (_shutterModeChanged)
	{
		int shutterMode;
		getIntegerParam(ADShutterMode, &shutterMode);
		result |= xcm_clm_set_param(_serialNumbers[0], 67, (short)(shutterMode == 2 ? 15 : 0));
		_shutterModeChanged = false;
	}
	// Set the shutter close delay from ADShutterCloseDelay (convert to ms)
	if (_shutterDelayChanged)
	{
		double shutterCloseDelay;
		getDoubleParam(ADShutterCloseDelay, &shutterCloseDelay);
		// We can't impose limits on this in the database since in EPICS shutter mode the limits
		// may be different.  Instead we constrain here and (if necessary) set the readback value
		if (shutterCloseDelay < 0)
		{
			shutterCloseDelay = 0;
			setDoubleParam(ADShutterCloseDelay, shutterCloseDelay);
		}
		else if (shutterCloseDelay > 65.535)
		{
			shutterCloseDelay = 65.535;
			setDoubleParam(ADShutterCloseDelay, shutterCloseDelay);
		}

		short shutterCloseDelay_ms = (short)(shutterCloseDelay * 1000.0 + 0.5);
		result |= xcm_clm_set_param(_serialNumbers[0], 66, shutterCloseDelay_ms);
		_shutterDelayChanged = false;
	}

	if (_callGrabSetup)
	{
		epicsThreadSleep(0.5);
		_roiParametersChanged = false;
	}

	if (_switchModeCheck)
	{
		// Set the trigger mode
		int triggerMode;
		getIntegerParam(ADTriggerMode, &triggerMode);
		// Check to see if we have been updating parameters whilst in an external trigger mode
		if ((triggerMode < 3) || (triggerMode==5))
		{
			short node;
			switch ((short)_paramSEQ_NODE_SELECTION.Value(*this))
			{
				case 0:
					if (imageSizeX < 1654)
						_grabWaitValue = 1.5;
					else
						_grabWaitValue = 2.5;
					break;
				case 1:
					if (imageSizeX < 1654)
						_grabWaitValue = 3;
					else
						_grabWaitValue = 6;
					break;
				default:
					if (imageSizeX < 1654)
						_grabWaitValue = 3;
					else
						_grabWaitValue = 6;
					break;
			}

			_grabWaitFlag = true;

			// Briefly release the external trigger mode and then set back to original state
			result |= xcm_clm_set_param(_serialNumbers[0], 65, 3);
			epicsThreadSleep(0.3);
			result |= xcm_clm_set_param(_serialNumbers[0], 65, (short)triggerMode);
			epicsThreadSleep(1);
		}
		_switchModeCheck = false;
	}
	callParamCallbacks();

	return result;
}

#pragma region RIXS simulation

// Compute array for RIXS simulation
int xcamCamera::computeRIXSArray(int sizeX, int sizeY)
{
	epicsUInt16 *pMono = (epicsUInt16 *)this->pRaw->pData;

	// Refresh the parameters we need
	bool simulationMode = (_paramRIXS_SIMULATION.Value(*this)) != 0;
	int eventsPerFrame = _paramRIXS_EVENTSPERFRAME.Value(*this);
	int backgroundLevel = _paramRIXS_BACKGROUNDLEVEL.Value(*this);
	float eventHeight = (float)_paramRIXS_EVENTHEIGHT.Value(*this);
	float eventRadius = (float)_paramRIXS_EVENTRADIUS.Value(*this);

	// Clear the Image
	epicsUInt16 *pMono2 = pMono;
	for (int i = 0; i < sizeY; i++) {
		for (int j = 0; j < sizeX; j++) {
			(*pMono2++) = (epicsUInt16)backgroundLevel;
		}
	}

	float centreX = (float)sizeX * 0.5f;
	float centreY = (float)sizeY * 0.5f;
	float radius = (centreX < centreY ? centreX : centreY);
	radius *= 0.75f;
	float eventRadius2 = eventRadius * eventRadius;

	for (int ev = 0; ev < eventsPerFrame; ++ev)
	{
		// Random angle -> random point on a circle
		double theta = M_PI * 2.0 * (double)rand() / (double)RAND_MAX;
		// The location of the centre of the event
		float x = centreX + (float)cos(theta) * radius;
		float y = centreY + (float)sin(theta) * radius;
		int ix = (int)(x + 0.5f);
		int iy = (int)(y + 0.5f);

		// Add a Gaussian profile at the event location
		for (int ey = iy - 1; ey <= iy + 1; ++ey)
		{
			for (int ex = ix - 1; ex <= ix + 1; ++ex)
			{
				float dx = (float)ex - x;
				float dy = (float)ey - y;
				float r2 = dx * dx + dy * dy;

				pMono[ey * sizeX + ex] += (epicsUInt16)(eventHeight * exp(-r2 / eventRadius2));
			}
		}
	}

	return asynSuccess;
}


#pragma endregion

#pragma endregion

#pragma region Temperature task

static void c_temperatureTask(void *drvPvt)
{
	xcamCamera *pPvt = (xcamCamera *)drvPvt;

	pPvt->temperatureTask();
}

void xcamCamera::temperatureTask(void)
{
	static const char *functionName = "temperatureTask";
	static double tSim = 0.0;

	while (!_exiting)
	{
		// Read the temperature once every 30 seconds
		// Wait at the start, to give saveRestore time to establish the parameters
		epicsThreadSleep(30.0);

		// Encoded temperature set to simulation (in case of hardware emulation)
		int encodedTemp = EncodeTemperatureCelsius(-50.0 + 5.0 * sin(tSim));

		{
			// Prevent camera thread from accessing xcmclm
			MutexLocker lock(_xcmclmMutex);

			if (_tempControllerParamsChanged)
			{
#pragma __Note__("This might not be thread-safe")
				_tempControllerParamsChanged = false;

				SetTemperatureController();
			}

			xcm_clm_send_temp_cntrl_command(_serialNumbers[0], CMD_TEMP_GET_RAW_PLANT_VALUE, 0, &encodedTemp);
		} // Release the mutex, so camera thread can access xcmclm

		double actualTempCelcius = DecodeTemperatureCelsius(encodedTemp);

		this->lock();
		setDoubleParam(ADTemperatureActual, actualTempCelcius);

		callParamCallbacks();
		this->unlock();

		tSim += 0.1;
	}
}


#pragma endregion

#pragma region Parameter write handlers

/** Called when asyn clients call pasynInt32->write().
  * This function performs actions for some parameters, including ADAcquire, ADColorMode, etc.
  * For all parameters it sets the value in the parameter library and calls any registered callbacks..
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Value to write. */
asynStatus xcamCamera::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
	int function = pasynUser->reason;
	int adstatus;
	int acquiring;
	int imageMode;

	/* Ensure that ADStatus is set correctly before we set ADAcquire.*/
	getIntegerParam(ADStatus, &adstatus);
	getIntegerParam(ADAcquire, &acquiring);
	getIntegerParam(ADImageMode, &imageMode);
	if (function == ADAcquire)
	{
		if (value && !acquiring)
		{
			if (_grabWaitFlag)
			{
				epicsThreadSleep(_grabWaitValue);
				_grabWaitFlag = false;
			}
			setStringParam(ADStatusMessage, "Acquiring data");
		}
		if (!value && acquiring)
		{
			setStringParam(ADStatusMessage, "Acquisition stopped");
			if (imageMode == ADImageContinuous)
			{
				setIntegerParam(ADStatus, ADStatusIdle);
			}
			else
			{
				setIntegerParam(ADStatus, ADStatusAborted);
			}
		}
	}
	else if (_paramCCD_POWER.HasParameterIndex(function))
	{
		_CCDPowerChanged = true;
	}
	else if (function == ADTriggerMode)
	{
		_TriggerModeChanged = true;
	}
	else if (function == ADShutterMode)
	{
		_shutterModeChanged = true;
	}
	else
	{
		for (auto tempParam : _tempControllerParams)
		{
			if (tempParam->HasParameterIndex(function))
			{
				_tempControllerParamsChanged = true;
				break;
			}
		}

		// Deal with awkward cases
		if (_paramTEMP_HEATER_SELECT.HasParameterIndex(function) ||
			_paramTEMP_SENSOR_SELECT.HasParameterIndex(function))
			_tempControllerParamsChanged = true;;
	}

	callParamCallbacks();

	// Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
	// status at the end, but that's OK
	asynStatus status = setIntegerParam(function, value);

	// If the function refers to an ROI parameter, note it
	_roiParametersChanged |= (_roiParameterIndices.find(function) != _roiParameterIndices.end());
	// If node has changed force sequencer to be re-loaded
	_roiParametersChanged |= _paramSEQ_NODE_SELECTION.HasParameterIndex(function);
	_SequencerParametersChanged |= _paramSEQ_ADC_DELAY.HasParameterIndex(function);
	_SequencerParametersChanged |= _paramSEQ_INT_MINUS_DELAY.HasParameterIndex(function);
	_SequencerParametersChanged |= _paramSEQ_INT_PLUS_DELAY.HasParameterIndex(function);
	_SequencerParametersChanged |= _paramSEQ_INT_TIME.HasParameterIndex(function);
	_SequencerParametersChanged |= _paramSEQ_PARALLEL_T.HasParameterIndex(function);
	_SequencerParametersChanged |= _paramSEQ_SERIAL_T.HasParameterIndex(function);

	_adcGainOffsetChanged |= _paramADC_GAIN.HasParameterIndex(function);
	_adcGainOffsetChanged |= _paramADC_OFFSET.HasParameterIndex(function);

	if (function == ADAcquire)
	{
		if (value && !acquiring)
		{
			/* Send an event to wake up the simulation task.
			 * It won't actually start generating new images until we release the lock below */
			epicsEventSignal(this->startEventId);
		}
		if (!value && acquiring)
		{
			/* This was a command to stop acquisition */
			/* Send the stop event */
			epicsEventSignal(this->stopEventId);
		}
	}
	else
	{
		/* If this parameter belongs to a base class call its method */
		if (function < FIRST_XCAM_CAMERA_PARAM) status = ADDriver::writeInt32(pasynUser, value);
	}

	/* Do callbacks so higher layers see any changes */
	callParamCallbacks();
	ReportWriteStatus(pasynUser, status, "writeInt32");
	return status;
}


/** Called when asyn clients call pasynFloat64->write().
  * This function performs actions for some parameters, including ADAcquireTime, ADGain, etc.
  * For all parameters it sets the value in the parameter library and calls any registered callbacks..
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Value to write. */
asynStatus xcamCamera::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
	int function = pasynUser->reason;
	asynStatus status = asynSuccess;

	for (auto voltageParam : _voltageParams)
	{
		if (voltageParam->HasParameterIndex(function))
		{
			_voltageParamsChanged = true;
			break;
		}
	}

	/* Check if the target temperature has changed */
	if (function == ADTemperature)
	{
		_tempControllerParamsChanged = true;
	}
	else if (function == ADAcquireTime)
	{
		_acquireTimeChanged = true;
	}
	else if (function == ADGain)
	{
		_adcGainOffsetChanged = true;
	}

	else if (function == ADShutterCloseDelay)
	{
		_shutterDelayChanged = true;
	}

	/* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
	 * status at the end, but that's OK */
	status = setDoubleParam(function, value);

	/* If this parameter belongs to a base class call its method */
	if (function < FIRST_XCAM_CAMERA_PARAM) status = ADDriver::writeFloat64(pasynUser, value);

	/* Do callbacks so higher layers see any changes */
	callParamCallbacks();
	ReportWriteStatus(pasynUser, status, "writeFloat64");
	return status;
}

asynStatus xcamCamera::writeOctet(asynUser *pasynUser, const char *value, size_t maxChars,
	size_t *nActual)
{
	int function = pasynUser->reason;
	asynStatus status = asynSuccess;

	if (function == SeqFilename)
	{
		// Sequencer filename is being changed.  Invalidate the correction values here;
		// the file will be re-read when it is required.
		_sequencerFilenameChanged = true;
		status = setStringParam(SeqFilename, value);
	}
	else
	{
		if (function < FIRST_XCAM_CAMERA_PARAM)
			status = ADDriver::writeOctet(pasynUser, value, maxChars, nActual);
	}

	ReportWriteStatus(pasynUser, status, "writeOctet");
	if (status != asynSuccess) return (status);

	// Do callbacks so higher layers see any changes
	callParamCallbacks();

	*nActual = maxChars;
	return status;
}


#pragma endregion

#pragma region iocsh interface

/** Configuration command, called directly or from iocsh */
extern "C" int RIXSCamConfig(const char *portName, int maxSizeX, int maxSizeY,
	int maxBuffers, int maxMemory, int priority, int stackSize)
{
	new xcamCamera(portName, maxSizeX, maxSizeY,
		(maxBuffers < 0) ? 0 : maxBuffers,
		(maxMemory < 0) ? 0 : maxMemory,
		priority, stackSize);
	return(asynSuccess);
}

/** Code for iocsh registration */
static const iocshArg xcamCameraConfigArg0 = { "Port name", iocshArgString };
static const iocshArg xcamCameraConfigArg1 = { "Max X size", iocshArgInt };
static const iocshArg xcamCameraConfigArg2 = { "Max Y size", iocshArgInt };
static const iocshArg xcamCameraConfigArg3 = { "maxBuffers", iocshArgInt };
static const iocshArg xcamCameraConfigArg4 = { "maxMemory", iocshArgInt };
static const iocshArg xcamCameraConfigArg5 = { "priority", iocshArgInt };
static const iocshArg xcamCameraConfigArg6 = { "stackSize", iocshArgInt };
static const iocshArg * const xcamCameraConfigArgs[] = {
	&xcamCameraConfigArg0,
	&xcamCameraConfigArg1,
	&xcamCameraConfigArg2,
	&xcamCameraConfigArg3,
	&xcamCameraConfigArg4,
	&xcamCameraConfigArg5,
	&xcamCameraConfigArg6
};

static const iocshFuncDef configxcamCamera = { "RIXSCamConfig", 7, xcamCameraConfigArgs };
static void configRIXSCamCallFunc(const iocshArgBuf *args)
{
	RIXSCamConfig(
		args[0].sval, args[1].ival, args[2].ival, args[3].ival,
		args[4].ival, args[5].ival, args[6].ival);
}

static void RIXSCamRegister(void)
{
	iocshRegister(&configxcamCamera, configRIXSCamCallFunc);
}

extern "C" {
	epicsExportRegistrar(RIXSCamRegister);
}

#pragma endregion

#pragma region Camera control

bool xcamCamera::LoadSequencer()
{
	char seqFilename[MAX_FILENAME_LEN];
	int status = getStringParam(SeqFilename, sizeof(seqFilename), seqFilename);

	// We proceed only if we can open the file
	bool found = false;
	{
		ifstream file(seqFilename);
		if (file)
			found = true;
		// (file closed on block exit)
	}

	if (!found)
		return false;

	// Disapply power before loading the sequencer
	SetCCDPower(false);

	asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
		"%s:%s: Loading sequencer file %s for camera serial %i\n",
		_driverName, "LoadSequencer", seqFilename, _serialNumbers[0]);

	// Only the first serial number needs to be loaded
	int result = xcm_clm_load_seq(_serialNumbers[0], seqFilename);

	if (result != XE_OK)
	{
		asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
			"%s:%s: xcm_clm_load_seq reports error %i loading %s for camera serial %i\n",
			_driverName, "LoadSequencer", result, seqFilename, _serialNumbers[0]);

		return false;
	}

	_sequencerFilenameChanged = false;

	// Restore setting as specified by parameter
	return SetCCDPower();
}

bool xcamCamera::SetCCDPower()
{
	return SetCCDPower(_paramCCD_POWER.Value(*this) != 0);
}

bool xcamCamera::SetCCDPower(bool on, bool force)
{
	if (force || _ccdPowerOn != on)
	{
		asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
			"%s:%s: Required CCD power state for camera changing from %i to %i; disapplying all voltages\n",
			_driverName, "SetCCDPower", _ccdPowerOn, on);

		// We ALWAYS set the voltages to zero before changing the state of the power
		// Set voltages off (in reverse order)
		for (auto voltageParamIt = _voltageParams.rbegin(); voltageParamIt != _voltageParams.rend(); ++voltageParamIt)
		{
			for (size_t ccd = 0; ccd < CCDCount(); ++ccd)
				xcm_clm_set_voltage(_serialNumbers[ccd], (*voltageParamIt)->InternalIndex(ccd, 0), 0);
			// (We use an index step of zero because xcmclm adds it according to which serial number is specified.)
		}

		// Turn CCD power on or off
		xcm_clm_ccd_power(_serialNumbers[0], (int)on);

		// One second delay required after the power is applied
		if (on)
			epicsThreadSleep(1.0);

		// Record current state
		_ccdPowerOn = on;

		_CCDPowerChanged = false;

		SetCCDVoltages();
	}

	return true;
}

bool xcamCamera::SetCCDVoltages()
{
	if (_ccdPowerOn)
	{
		// Make sure the voltages satisfy the constraints
		// (Note: can't do this when they are set, because we can't control the order in which they are set)
		if (ApplyVoltageConstraints())
		{
			// Do callbacks so higher layers see any changes
			callParamCallbacks();
		}

		asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
			"%s:%s: Applying voltages to camera\n",
			_driverName, "SetCCDPower");

		// Set voltages on (in forward order)
		for (auto voltageParam : _voltageParams)
		{
			for (size_t ccd = 0; ccd < CCDCount(); ++ccd)
			{
				short v = (short)(voltageParam->ScaledValue(*this, ccd));
				// (Voltage - in volts - should already be an exact multiple of the scale step)

				// Send the value to the camera
				xcm_clm_set_voltage(_serialNumbers[ccd], voltageParam->InternalIndex(ccd, 0), v);
				// (We use an index step of zero because xcmclm adds it according to which serial number is specified.)
			}
		}

		_voltageParamsChanged = false;
	}

	return true;
}

bool xcamCamera::ApplyVoltageConstraints()
{
	bool modified = false;

	for (size_t ccd = 0; ccd < CCDCount(); ++ccd)
	{
		// Normalize all the voltages, in case they are not multiples of their scale setting
		for (auto voltageParam : _voltageParams)
		{
			modified |= voltageParam->Normalize(*this, ccd);
		}

		epicsFloat64 Vss = _paramVOLT_BIAS_SS.Value(*this, ccd);

		modified |= _paramVOLT_BIAS_OD.SetValueAtMinimum(*this, ccd, Vss);
		modified |= _paramVOLT_BIAS_DD.SetValueAtMinimum(*this, ccd, Vss);
		modified |= _paramVOLT_BIAS_RD.SetValueAtMinimum(*this, ccd, Vss);
	}

	return modified;
}

// Set the exposure time registers from the AcquireTime parameter, and update the readback
void xcamCamera::SetExposureTime()
{
	// Get the requested exposure time in seconds
	double acquireTime;
	getDoubleParam(ADAcquireTime, &acquireTime);

	vector<double> units = { 0.001, 0.01};

	// We choose the smallest unit such that the requested exposure time is <= 65535 multiples of it
	int unitIndex = 2;
	int count = 65535;
	for (int i = 0; i < units.size(); ++i)
	{
		if (acquireTime < units[i] * 65535.5)
		{
			unitIndex = i;
			// Find the number of units closest to the requested exposure time
			count = (int)((acquireTime + units[i] * 0.5) / units[i]);

			if (count < 1)
				count = 1;
			else if (count > 65535)
				count = 65535;

			break;
		}
	}

	// Register 64 is for the 'units'
	xcm_clm_set_param(_serialNumbers[0], 64, unitIndex);
	// Register 15 is for the 'count'
	xcm_clm_set_param(_serialNumbers[0], 15, count);

	// Update the value that will actually be used
	setDoubleParam(ADAcquireTime, (double)count * units[unitIndex]);
}

void xcamCamera::SetTemperatureController()
{
	for (auto param : _tempControllerParams)
	{
		// Set the parameter
		int currentValue = param->Value(*this);
		int newValue = currentValue;

		// Set the controller parameter - the internal indeces are the 'set' variants
		xcm_clm_send_temp_cntrl_command(_serialNumbers[0], param->InternalIndex(), currentValue, &newValue);
		// No need to read the values back - the controller does not modify them
	}

	// Change the target temperature
	double SetTemperature;
	getDoubleParam(ADTemperature, &SetTemperature);
	xcm_clm_set_target_temperature(_serialNumbers[0], SetTemperature);

	// Configure I/O select bits for the temperature controller (Bit 0 : Temp Sensor Input; Bit 1 Heater Output)
	short selectBits = 0;
	if (_paramTEMP_HEATER_SELECT.Value(*this) > 0)
		selectBits |= 2;
	if (_paramTEMP_SENSOR_SELECT.Value(*this) > 0)
		selectBits |= 1;

	xcm_clm_temp_cntrl_select_IO(_serialNumbers[0], selectBits);
}

#pragma endregion

#pragma region Shutdown

// This (C) function should be called on shutdown with an argument set to a pointer to the xcamCamera object.
// However, I'm not confident it will often get called in practice
static void c_shutdown(void *arg)
{
	xcamCamera *p = (xcamCamera *)arg;
	p->Shutdown();
}

void xcamCamera::Shutdown()
{
	_exiting = true;

	// We don't try to acquire the mutex, in case we enter a lock

	// Cancel any in-progress capture
	for (auto serial : _serialNumbers)
		xcm_clm_cancel_grab(serial);

	// Set CCD power to off (will also disapply voltages)
	SetCCDPower(false);

	// Call xcm_clm_terminate()
	for (auto serial : _serialNumbers)
		xcm_clm_terminate();

	_serialNumbers.clear();
}

#pragma endregion

#pragma region Helpers

/** Report status of the driver.
  * Prints details about the driver if details>0.
  * It then calls the ADDriver::report() method.
  * \param[in] fp File pointed passed by caller where the output is written to.
  * \param[in] details If >0 then driver details are printed.
  */
void xcamCamera::report(FILE *fp, int details)
{

    fprintf(fp, "XCAM camera %s\n", this->portName);
    if (details > 0) {
        int nx, ny, dataType;
        getIntegerParam(ADSizeX, &nx);
        getIntegerParam(ADSizeY, &ny);
        getIntegerParam(NDDataType, &dataType);
        fprintf(fp, "  NX, NY:            %d  %d\n", nx, ny);
        fprintf(fp, "  Data type:         %d\n", dataType);
    }
    /* Invoke the base class method */
    ADDriver::report(fp, details);
}

void xcamCamera::ReportWriteStatus(asynUser *pasynUser, const asynStatus status, const char * methodName)
{
	if (status)
		asynPrint(pasynUser, ASYN_TRACE_ERROR,
		"%s:%s: error, status=%d function=%d\n",
		_driverName, methodName, status, pasynUser->reason);
	else
		asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
		"%s:%s: function=%d\n",
		_driverName, methodName, pasynUser->reason);
}


#pragma endregion
