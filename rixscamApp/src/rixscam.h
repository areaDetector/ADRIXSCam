#include <epicsEvent.h>
#include "ADDriver.h"
#include "Parameter.tlh"
#include <vector>
#include <set>

#define BYTE unsigned char
#include "xcmclm.h"

#define _STRING(x) #x
#define STRING(x) _STRING(x)
#define __Note__(x) message(__FILE__ "(" STRING(__LINE__) ") : Note >>>> " x )

/** Simulation detector driver; demonstrates most of the features that areaDetector drivers can support. */
class epicsShareClass xcamCamera : public ADDriver {
public:
    xcamCamera(const char *portName, int maxSizeX, int maxSizeY,
                int maxBuffers, size_t maxMemory,
                int priority, int stackSize);

    /* These are the methods that we override from ADDriver */
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
	virtual asynStatus writeOctet(asynUser *pasynUser, const char *value, size_t maxChars, size_t *nActual);
	virtual void setShutter(int open);
    virtual void report(FILE *fp, int details);

	// The following should be private, but get called from C, so must be public
	void imageTask(); 
	void temperatureTask(void);
	void Shutdown();

protected:
    int SoftVersion;
#define FIRST_XCAM_CAMERA_PARAM SoftVersion
	int IFSVersion;
	int FPGAVersion;
	int CamSerial;
	int SeqFilename;
#define LAST_XCAM_CAMERA_PARAM SeqFilename


private:
	epicsMutex _xcmclmMutex;

	const size_t _sensorSizeX = 1600;
	const size_t _sensorSizeY = 1600;

	// We check at these time intervals to see if any of the major settings have changed
	const double _acquireTimeoutSeconds = 0.2;

	// These are private and therefore not accessible outside the class - let alone outside the dll.
	// We can therefore ignore warnings about dll linkages
#pragma warning (push)
#pragma warning (disable: 4251)

	bool _newImageRequired;

	static const char* _driverName;

	bool _exiting;
	std::vector<int> _serialNumbers;
	vector<NDArray*> _ccdImages;

#include "\Development\Clients\XCAM\EPICS\synApps\synApps_5_8\support\areaDetector-R2-4\ADxcamProjects\xcamCameraIOC\xcamCameraIOC\PVDeclarations.h"

	bool _ccdPowerOn;
	bool _CCDPowerChanged;
	bool _sequencerFilenameChanged;

	bool _TriggerModeChanged;

	const short _voltageIndexStep = 16; // Add this * <ccd number> to the internal index, to get register number
	std::vector<MultiParameter<_ccdCountMax, epicsFloat64>*> _voltageParams;
	bool _voltageParamsChanged;

	std::vector<Parameter<epicsInt32>*> _tempControllerParams;
	bool _tempControllerParamsChanged;

	std::vector<Parameter<epicsInt32>*> _sequencerParams;
	bool _SequencerParametersChanged;
    
	bool _acquireTimeChanged;
	bool _adcGainOffsetChanged;
	bool _shutterModeChanged;
	bool _shutterDelayChanged;
	bool _grabWaitFlag;
	double _grabWaitValue;

	//int _node;

	// Mechanism to detect changes to roi parameters, so scan does not have to be set up
	// if no changes have been made
	bool _roiParametersChanged;
	bool _callGrabSetup;
	bool _switchModeCheck;

	std::set<int> _roiParameterIndices;

#pragma warning (pop)

	size_t CCDCount() const { return _serialNumbers.size(); };

	// Load the sequencer from the file specified by the parameter
	bool xcamCamera::LoadSequencer();
	// Set the CCD power according to the parameter setting
	bool SetCCDPower();
	// Set the CCD power according to the boolean parameter.  If force is true, ignore the cached state
	bool SetCCDPower(bool on, bool force = false);
	// Apply voltages, if the power is on
	bool SetCCDVoltages();
	bool ApplyVoltageConstraints();
	// Set the exposure time registers from the AcquisitionTime parameter, and update the readback
	void SetExposureTime();
	// Set the temperature controller configuration
	void SetTemperatureController();
	// Configure the ROI and binning etc.
	void ConfigureROIBinning();
	// Configure the ROI and binning etc. for a particular axis (i.e. X or Y)
	void ConfigureROIBinningAxis(const int binID, const int minID, const int sizeID, const int maxID,
		int& bin, int& origin, int& count);
	void ConstrainToSensorAxis(const int globalOrigin, const int globalSize, const int globalBin,
		const int localOrigin, const int localSize,
		int& originOut, int& countOut);

    int computeRIXSArray(int sizeX, int sizeY);
    NDArray* GetImage();

	enum temperatureControllerCommands
	{
		CMD_TEMP_GET_RAW_PLANT_VALUE = 0,
		CMD_TEMP_GET_SETPOINT = 1,
		CMD_TEMP_SET_SETPOINT = 2,
		CMD_TEMP_GET_PROPORTIONAL_GAIN = 3,
		CMD_TEMP_SET_PROPORTIONAL_GAIN = 4,
		CMD_TEMP_GET_INTEGRAL_GAIN = 5,
		CMD_TEMP_SET_INTEGRAL_GAIN = 6,
		CMD_TEMP_GET_DERIVATIVE_GAIN = 7,
		CMD_TEMP_SET_DERIVATIVE_GAIN = 8,
		CMD_TEMP_GET_PROPORTIONAL_TIME = 9,
		CMD_TEMP_SET_PROPORTIONAL_TIME = 10,
		CMD_TEMP_GET_INTEGRAL_RATE = 11,
		CMD_TEMP_SET_INTEGRAL_RATE = 12,
		CMD_TEMP_GET_DERIVATIVE_RATE = 13,
		CMD_TEMP_SET_DERIVATIVE_RATE = 14,
		CMD_TEMP_GET_ACCUMULATED_ERROR_LIMIT = 15,
		CMD_TEMP_SET_ACCUMULATED_ERROR_LIMIT = 16,
		CMD_TEMP_GET_OUTPUT_BIAS_VALUE = 17,
		CMD_TEMP_SET_OUTPUT_BIAS_VALUE = 18,
		CMD_TEMP_SET_MANUAL_MODE = 19,
		CMD_TEMP_SET_ENABLE_CONTROL = 20
	};

	int EncodeTemperatureCelsius(double tempDegreesC) { return (int)(0.5 + (tempDegreesC + 260.0) / 0.0000277); };
	double DecodeTemperatureCelsius(int encoded) { return ((double)encoded * 0.0000277) - 260.0; }

	void ReportWriteStatus(asynUser *pasynUser, const asynStatus status, const char * methodName);

    /* Our data */
    epicsEventId startEventId;
    epicsEventId stopEventId;
    NDArray *pRaw;
};

#define NUM_XCAM_CAMERA_PARAMS ((int)(&LAST_XCAM_CAMERA_PARAM - &FIRST_XCAM_CAMERA_PARAM + 1))

