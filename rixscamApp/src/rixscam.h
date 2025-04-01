#include <epicsEvent.h>
#include "ADDriver.h"
#include <iostream>
#include <string>
#include <vector>
#include <set>
#include <sstream>
#include <assert.h>

#define BYTE unsigned char
#include "xcmclm.h"

#define _STRING(x) #x
#define STRING(x) _STRING(x)
#define __Note__(x) message(__FILE__ "(" STRING(__LINE__) ") : Note >>>> " x )

// The following was included from Parameter.tlh

class ParameterBase
{
public:
	virtual asynStatus Initialize(asynPortDriver& driver) = 0;

protected:
	ParameterBase(const std::string& name) :
		_name(name)
	{};

	asynStatus ParameterBase::SetParameter(asynPortDriver& driver, const int parameterIndex, const epicsInt32 value)
	{
		return driver.setIntegerParam(parameterIndex, value);
	}

	asynStatus ParameterBase::SetParameter(asynPortDriver& driver, const int parameterIndex, const epicsFloat64 value)
	{
		return driver.setDoubleParam(parameterIndex, value);
	}

	asynStatus ParameterBase::GetParameter(asynPortDriver& driver, const int parameterIndex, epicsInt32& value)
	{
		return driver.getIntegerParam(parameterIndex, &value);
	}

	asynStatus ParameterBase::GetParameter(asynPortDriver& driver, const int parameterIndex, epicsFloat64& value)
	{
		return driver.getDoubleParam(parameterIndex, &value);
	}

	const std::string _name;
};




template <typename T>
class ReadOnlyParameter : public ParameterBase
{
public:
	ReadOnlyParameter(const std::string& name, const short internalIndex, const T& defaultValue, const T& scale) :
		ParameterBase(name),
		_internalIndex(internalIndex),
		_parameterIndex(0),
		_value(defaultValue),
		_scale(scale)
	{
	};

	ReadOnlyParameter(const std::string& name, const short internalIndex, const T& defaultValue) :
		ParameterBase(name),
		_internalIndex(internalIndex),
		_parameterIndex(0),
		_value(defaultValue),
		_scale((T)1)
	{
	};

	ReadOnlyParameter(const std::string& name, const short internalIndex) :
		ParameterBase(name),
		_internalIndex(internalIndex),
		_parameterIndex(0),
		_value((T)0),
		_scale((T)1)
	{
	};

	virtual asynStatus Initialize(asynPortDriver& driver)
	{
		asynStatus status = Create(driver, _name, _parameterIndex);
		if (status == asynSuccess)
		{
			status = SetParameter(driver, _parameterIndex, _value);
		}

		return status;
	}

	virtual T Value(asynPortDriver& driver)
	{
		// The local value is definitive for a read-only parameter
		return _value;
	}

	// We implement SetValue even for read-only parameters, because the 'read only' is from
	// the point of view of the outside world; we still have to set the parameter value
	virtual asynStatus SetValue(asynPortDriver& driver, T newValue)
	{
		if (newValue == _value)
			return asynSuccess;

		_value = newValue;
		return SetParameter(driver, _parameterIndex, _value);
	}

	virtual int ScaledValue(asynPortDriver& driver)
	{
		return (int)((Value(driver) + _scale / (T)2) / _scale);
	}

	virtual bool SetValueAtMinimum(asynPortDriver& driver, T minimumValue)
	{
		T currentValue = Value(driver);
		if (currentValue >= minimumValue)
			return false; // The value is already >= the required level

		// Calculate the minimum multiple of _scale that's >= minimumVaue
		int scaled = (int)ceil(minimumValue / _scale);
		T newValue = scaled * _scale;

		assert(newValue >= minimumValue);

		SetValue(driver, newValue);

		return true;
	}

	virtual asynStatus SetScaledValue(asynPortDriver& driver, int newValue)
	{
		T internalValue = (T)newValue * _scale;
		return SetValue(driver, internalValue);
	}

	virtual bool Normalize(asynPortDriver& driver)
	{
		int scaled = ScaledValue(driver);
		if ((scaled * _scale) != Value(driver))
		{
			SetScaledValue(driver, scaled);
			return true;
		}
		else
			return false;
	}

	virtual bool HasParameterIndex(int indexToMatch) const
	{
		return _parameterIndex == indexToMatch;
	}

	// Return an index value associated with the parameter
	virtual short InternalIndex() const
	{
		return _internalIndex;
	}

protected:
	virtual asynStatus Create(asynPortDriver& driver, const std::string& name, int& parameterIndex)
	{
		return asynError; // Only executed if <T> is unknown
	}

	T _value; // Local value
	int _parameterIndex; // The numeric index by which the variable is known to asynPortDriver
	const int _internalIndex; // A numeric index by which the parameter is known to the user (e.g. a voltage index)
	const T _scale; // Scale factor: <EPICS value> = <Internal value> * _scale
};


template<> asynStatus ReadOnlyParameter<epicsInt32>::Create(asynPortDriver& driver, const std::string& name, int& parameterIndex)
{
	return driver.createParam(name.c_str(), asynParamInt32, &parameterIndex);
}

template<> asynStatus ReadOnlyParameter<epicsFloat64>::Create(asynPortDriver& driver, const std::string& name, int& parameterIndex)
{
	return driver.createParam(name.c_str(), asynParamFloat64, &parameterIndex);
}


template <typename T>
class Parameter : public ReadOnlyParameter<T>
{
public:
	Parameter(const std::string& name, const short internalIndex,
		const T& minimumValue, const T& maximumValue, const T& defaultValue) :
		ReadOnlyParameter(name, internalIndex, defaultValue),
		_minValue(minimumValue),
		_maxValue(maximumValue)
	{
		assert(minimumValue <= maximumValue);
		assert(minimumValue <= defaultValue);
		assert(defaultValue <= maximumValue);
	};

	Parameter(const std::string& name, const short internalIndex,
		const T& minimumValue, const T& maximumValue, const T& defaultValue, const T& scale) :
		ReadOnlyParameter(name, internalIndex, defaultValue, scale),
		_minValue(minimumValue),
		_maxValue(maximumValue)
	{
		assert(minimumValue <= maximumValue);
		assert(minimumValue <= defaultValue);
		assert(defaultValue <= maximumValue);
	};

	virtual T Value(asynPortDriver& driver)
	{
		GetParameter(driver, _parameterIndex, _value);

		if (_value < _minValue)
		{
			_value = _minValue;
			SetParameter(driver, _parameterIndex, _value);
		}
		else if (_value > _maxValue)
		{
			_value = _maxValue;
			SetParameter(driver, _parameterIndex, _value);
		}

		return _value;
	}

private:
	const T _minValue; // Minimum valid value
	const T _maxValue; // Maximum valid value
};

template <int valueCount, typename T>
class MultiParameter : public ParameterBase
{
public:
	MultiParameter(const std::string& rootName, const short internalIndex,
		const T& minimumValue, const T& maximumValue, const T& defaultValue, const T& scale) :
		ParameterBase(rootName),
		_internalIndex(internalIndex),
		_minValue(minimumValue),
		_maxValue(maximumValue),
		_defaultValue(defaultValue)
	{
		assert(minimumValue <= maximumValue);
		assert(minimumValue <= defaultValue);
		assert(defaultValue <= maximumValue);

		for (size_t i = 0; i < valueCount; ++i)
		{
			std::stringstream fullName;
			fullName << _name << "_" << (i + 1);

			_parameters.push_back(Parameter<T>(fullName.str(), _internalIndex, _minValue, _maxValue, _defaultValue, scale));
		}
	};

	asynStatus Initialize(asynPortDriver& driver)
	{
		asynStatus status;
		for (auto& parameter : _parameters)
		{
			status = parameter.Initialize(driver);
			if (status != asynSuccess)
				return status;
		}

		return status;
	}

	T Value(asynPortDriver& driver, size_t index)
	{
		return _parameters[index].Value(driver);
	}

	int ScaledValue(asynPortDriver& driver, size_t index)
	{
		return _parameters[index].ScaledValue(driver);
	}

	asynStatus SetValue(asynPortDriver& driver, size_t index, T newValue)
	{
		return _parameters[index].SetValue(driver, newValue);
	}

	bool SetValueAtMinimum(asynPortDriver& driver, size_t index, T minimumValue)
	{
		return _parameters[index].SetValueAtMinimum(driver, minimumValue);
	}

	asynStatus SetScaledValue(asynPortDriver& driver, size_t index, int newValue)
	{
		return _parameters[index].SetScaledValue(driver, newValue);
	}

	bool Normalize(asynPortDriver& driver, size_t index)
	{
		return _parameters[index].Normalize(driver);
	}

	bool HasParameterIndex(int indexToMatch) const
	{
		for (auto& parameter : _parameters)
		{
			if (parameter.HasParameterIndex(indexToMatch))
				return true;
		}

		return false;
	}

	short InternalIndex(size_t index, short indexStep)
	{
		return _parameters[index].InternalIndex() + (short)(index + 1) * indexStep;
	}

protected:
	const short _internalIndex;
	const T _defaultValue; // Default, and initial, value
	const T _minValue; // Minimum valid value
	const T _maxValue; // Maximum valid value
	std::vector<Parameter<T>> _parameters;
};

// RIXSCAM

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
	// The acquisition task doesn't ensure ordenation between this mutex lock
	// and the port lock, so, always ensure to use one or another to prevent
	// dead lock.
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

    // Included from PVDeclarations.h
    const static int _ccdCountMax = 3;
    Parameter<epicsInt32> _paramSEQ_ADC_DELAY{"SEQ_ADC_DELAY", 0, 1, 65535, 144};
    Parameter<epicsInt32> _paramSEQ_INT_MINUS_DELAY{"SEQ_INT_MINUS_DELAY", 1, 1, 65535, 3};
    Parameter<epicsInt32> _paramSEQ_INT_PLUS_DELAY{"SEQ_INT_PLUS_DELAY", 2, 1, 65535, 3};
    Parameter<epicsInt32> _paramSEQ_INT_TIME{"SEQ_INT_TIME", 3, 1, 65535, 50};
    Parameter<epicsInt32> _paramSEQ_SERIAL_T{"SEQ_SERIAL_T", 4, 1, 65535, 1};
    Parameter<epicsInt32> _paramSEQ_PARALLEL_T{"SEQ_PARALLEL_T", 5, 1, 65535, 255};
    Parameter<epicsInt32> _paramSEQ_SERIAL_CLOCK{"SEQ_SERIAL_CLOCK", 7, 0, 0, 0};
    Parameter<epicsInt32> _paramSEQ_PARALLEL_CLOCK{"SEQ_PARALLEL_CLOCK", 8, 0, 1, 0};
    Parameter<epicsInt32> _paramSEQ_NODE_SELECTION{"SEQ_NODE_SELECTION", 13, 0, 3, 0};
    ReadOnlyParameter<epicsInt32> _paramSEQ_STATUS{"SEQ_STATUS", 68};
    MultiParameter<_ccdCountMax, epicsFloat64> _paramVOLT_BIAS_OD{"VOLT_BIAS_OD", 4, 0, 33.915, 0, 0.133};
    MultiParameter<_ccdCountMax, epicsFloat64> _paramVOLT_BIAS_RD{"VOLT_BIAS_RD", 5, 0, 23.97, 0, 0.094};
    MultiParameter<_ccdCountMax, epicsFloat64> _paramVOLT_BIAS_DD{"VOLT_BIAS_DD", 6, 0, 23.97, 0, 0.094};
    MultiParameter<_ccdCountMax, epicsFloat64> _paramVOLT_BIAS_OG{"VOLT_BIAS_OG", 7, 0, 23.97, 0, 0.094};
    MultiParameter<_ccdCountMax, epicsFloat64> _paramVOLT_BIAS_SS{"VOLT_BIAS_SS", 9, 0, 23.97, 0, 0.094};
    MultiParameter<_ccdCountMax, epicsFloat64> _paramVOLT_BIAS_HVDC{"VOLT_BIAS_HVDC", 8, 0, 23.97, 0, 0.094};
    MultiParameter<_ccdCountMax, epicsFloat64> _paramVOLT_BIAS_PEDESTAL{"VOLT_BIAS_PEDESTAL", 10, 0, 14.025, 0, 0.055};
    MultiParameter<_ccdCountMax, epicsFloat64> _paramVOLT_BIAS_HV{"VOLT_BIAS_HV", 11, 0, 49.98, 0, 0.196};
    MultiParameter<_ccdCountMax, epicsFloat64> _paramVOLT_CLOCK_IMAGE{"VOLT_CLOCK_IMAGE", 0, 0, 14.025, 0, 0.055};
    MultiParameter<_ccdCountMax, epicsFloat64> _paramVOLT_CLOCK_STORE{"VOLT_CLOCK_STORE", 1, 0, 14.025, 0, 0.055};
    MultiParameter<_ccdCountMax, epicsFloat64> _paramVOLT_CLOCK_SERIAL{"VOLT_CLOCK_SERIAL", 2, 0, 14.025, 0, 0.055};
    MultiParameter<_ccdCountMax, epicsFloat64> _paramVOLT_CLOCK_RESET{"VOLT_CLOCK_RESET", 3, 0, 14.025, 0, 0.055};
    Parameter<epicsInt32> _paramCCD_POWER{"CCD_POWER", 0, 0, 1, 0};
    Parameter<epicsInt32> _paramRIXS_SIMULATION{"RIXS_SIMULATION", 0, 0, 1, 0};
    Parameter<epicsInt32> _paramRIXS_EVENTSPERFRAME{"RIXS_EVENTSPERFRAME", 0, 0, 10000, 100};
    Parameter<epicsInt32> _paramRIXS_BACKGROUNDLEVEL{"RIXS_BACKGROUNDLEVEL", 0, 0, 65535, 0};
    Parameter<epicsFloat64> _paramRIXS_EVENTHEIGHT{"RIXS_EVENTHEIGHT", 0, 0, 65535, 200};
    Parameter<epicsFloat64> _paramRIXS_EVENTRADIUS{"RIXS_EVENTRADIUS", 0, 0.001, 2, 1};
    Parameter<epicsInt32> _paramTEMP_PROP_GAIN{"TEMP_PROP_GAIN", 4, 0, 255, 0};
    Parameter<epicsInt32> _paramTEMP_INT_GAIN{"TEMP_INT_GAIN", 6, 0, 255, 0};
    Parameter<epicsInt32> _paramTEMP_DERIV_GAIN{"TEMP_DERIV_GAIN", 8, 0, 255, 0};
    Parameter<epicsInt32> _paramTEMP_PROP_RATE{"TEMP_PROP_RATE", 10, 0, 65535, 0};
    Parameter<epicsInt32> _paramTEMP_INT_RATE{"TEMP_INT_RATE", 12, 0, 65535, 0};
    Parameter<epicsInt32> _paramTEMP_DERIV_RATE{"TEMP_DERIV_RATE", 14, 0, 65535, 0};
    Parameter<epicsInt32> _paramTEMP_ACCUMULATED_ERROR_LIMIT{"TEMP_ACCUMULATED_ERROR_LIMIT", 16, 0, 16777215, 0};
    Parameter<epicsInt32> _paramTEMP_OUTPUT_BIAS{"TEMP_OUTPUT_BIAS", 18, 0, 65535, 0};
    Parameter<epicsInt32> _paramTEMP_MANUAL_MODE{"TEMP_MANUAL_MODE", 19, 0, 1, 1};
    Parameter<epicsInt32> _paramTEMP_ENABLE{"TEMP_ENABLE", 20, 0, 1, 1};
    Parameter<epicsInt32> _paramTEMP_HEATER_SELECT{"TEMP_HEATER_SELECT", 0, 0, 1, 0};
    Parameter<epicsInt32> _paramTEMP_SENSOR_SELECT{"TEMP_SENSOR_SELECT", 0, 0, 1, 0};
    ReadOnlyParameter<epicsInt32> _paramCCD_COUNT{"CCD_COUNT", 0};
    Parameter<epicsFloat64> _paramADC_GAIN{"ADC_GAIN", 0, 0, 36.1119, 10, 0.0353};
    Parameter<epicsInt32> _paramADC_OFFSET{"ADC_OFFSET", 0, 0, 1020, 40, 4};

    const static int _parameterContainerCount = 43;
    const static int _parameterCount = 67;

    std::vector<ParameterBase*> _allParams;

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
    int configureCaptureParams();
    void doAcquisition(bool &acquisition_done);
    epicsEventStatus preAcquisition();

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

