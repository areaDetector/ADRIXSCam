/*
 * xcmclm.h - header file for applications wanting to use xcmclm.dll
 */

#ifndef XCMCLM_H
#define XCMCLM_H

/*
 * Users must not #define XCAM_EXPORT - this is reserved for exporting
 * the functions from the DLL.
 */
#ifdef XCAM_EXPORT
#define fntype __declspec(dllexport)
#else
#define fntype __declspec(dllimport)
#endif

#ifdef __cplusplus
extern "C" {
#endif

/*
 * error codes returned in errorcodes array for multi-module functions and
 * by the function for single-module calls
 */
#define XE_OK		0		/* no error */
#define XE_NOTFOUND	1		/* interface serial number not found */
#define XE_NOTINIT	2		/* interface not initialised */
#define XE_ARRAYLEN	3		/* array length too large */
#define XE_IOFAIL	4		/* interface communication failed */
#define XE_ERRORIND	5		/* error code array contains errors */
#define XE_TIMEOUT	6		/* transaction timed out */
#define XE_NOFILE	7		/* file name not found */
#define XE_ARGERROR	8		/* argument value in error */
#define XE_MEMALLOC	9		/* failed to allocate memory */
#define XE_NUMNODES	10		/* number of nodes not supported */
#define XE_CDSTYPE	11		/* type of CDS unknown */
#define XE_NULLBUFF	12		/* null buffer argument passed */
#define XE_PROT_NAK	13		/* comms protocol returned NAK */


/* this defines the number of CameraLink channels that the dll will support */
#define MAXCHAN 8

/* turn disk logging on if arg is non-zero - always returns E_OK */
fntype int __stdcall xcm_clm_logging(int enabled);

/*
 * Discover any XCAM interface modules connected to the PC, and return their serial 
 * numbers in the array whose length is arraylen. The number of modules found
 * is written to the int pointed to by the found argument.
 *
 * Return values:
 *
 *  XE_OK			success
 *  XE_ARRAYLEN		array length > MAXCHAN
 */
fntype int __stdcall xcm_clm_discover(int *array,int arraylen,int *found);

/*
 * Initialise interface modules from an array of serial numbers found by a previous
 * call to xcm_clm_disvover(). Note that the array may contain any serial numbers, 
 * regardless of whether they were found by the discovery function.
 *
 * If found, modules will be initialised so as to be suitable for controlling
 * an XCAM rack for setting up and grabbing frames using the other functions
 * in the DLL.
 *
 * The number of devices listed in the array that have actually been
 * found and initialised is returned to the caller in the located variable.
 *
 * Error codes in the errorcodes array are associated with the corresponding element
 * in the input array.
 *
 * return values:
 *
 *  XE_OK		success
 *	XE_ARRAYLEN	array length > MAXCHAN
 *	XE_ERRORIND	errorcodes array contains some error codes
 *
 * errorcodes values:
 *
 *	XE_OK		success
 *  XE_NOTFOUND	serial invalid
 *	XE_NOTINIT	could not initialise module
 */
fntype int __stdcall xcm_clm_init(int *array,int *errorcodes,int arraylen,int *located);

/*
 * xcm_dll_version(): copies dllversion to caller's buffer
 *
 * Return value:
 *
 *	XE_OK		success
 */
fntype int __stdcall xcm_clm_dll_version(char *version);

/*-----------------------------------------------------------------------------------
 *
 * All of the following functions (except xcm_clm_grab) require module serial number
 * to be passed as the first parameter, indicating which channel to work on. See
 * the notes for xcm_clm_grab() for argument details, which are similar to the serial
 * number arrays passed to discover() and init() functions.
 *
 */

/*
 * Read the ID string identifying the version of software in the rack's i/o card.
 * Data is written to the buffer, which must have a minimum length of 256
 * bytes.
 *
 * Return value:
 *
 *  XE_OK		success
 *  XE_NOTFOUND	serial invalid
 *	XE_NOTINIT	module not initialised
 *	XE_IOFAIL	module i/o failed
 *	XE_TIMEOUT	module read timed out
 */
fntype int __stdcall xcm_clm_ifsver(int serial,char *buffer);

/*
 * Read the ID string identifying the version of hardware in the rack's i/o card.
 * Data is written to the buffer, which must have a minimum length of 256
 * bytes.
 *
 * Return value:
 *
 *  XE_OK		success
 *  XE_NOTFOUND	serial invalid
 *	XE_NOTINIT	module not initialised
 *	XE_IOFAIL	module i/o failed
 *	XE_TIMEOUT	module read timed out
 */
fntype int __stdcall xcm_clm_fpgaver(int serial,char *buffer);

/*
 * Set CDS gain
 *
 * Return value:
 *
 *  XE_OK		success
 *  XE_NOTFOUND	serial invalid
 *	XE_NOTINIT	module not initialised
 *	XE_IOFAIL	module i/o failed
 */
fntype int __stdcall  xcm_clm_cds_gain(int serial,short int gain);

/*
 * Set CDS offset
 *
 * Return value:
 *
 *  XE_OK		success
 *  XE_NOTFOUND	serial invalid
 *	XE_NOTINIT	module not initialised
 *	XE_IOFAIL	module i/o failed
 */
fntype int __stdcall  xcm_clm_cds_offset(int serial,short int offset);

/*
 * Turn on the control signals from the rack to the CCD camera
 *
 * Return value:
 *
 *  XE_OK		success
 *  XE_NOTFOUND	serial invalid
 *	XE_NOTINIT	module not initialised
 *	XE_IOFAIL	module i/o failed
 */
fntype int __stdcall xcm_clm_ccd_power(int serial,short int onoff);

/*
 * Execute an echo request from the interface card. Sends a 16-byte packet to the
 * interface card consisting of ASCII characters 'A to 'P', which the interface card
 * should return without modification. The echoed packet is written to the buffer
 * passed by the user. The buffer should be a minimum of 20 bytes long.
 *
 * Return value:
 *
 *  XE_OK		success
 *  XE_NOTFOUND	serial invalid
 *	XE_NOTINIT	module not initialised
 *	XE_IOFAIL	module i/o failed
 */
fntype int __stdcall xcm_clm_echoreq(int serial,char *buffer);

/*
 * Set an individual bias/clock signal voltage; vindex indicates which signal voltage
 * to set (range 0 .. 11), and value is the value to write (range 0 .. 255)
 *
 * Return value:
 *
 *  XE_OK		success
 *  XE_NOTFOUND	serial invalid
 *	XE_NOTINIT	module not initialised
 *	XE_IOFAIL	module i/o failed
 *	XE_ARGERROR	argument error - vindex out of range
 */
fntype int __stdcall xcm_clm_set_voltage(int serial,short vindex,short value);

/*
 * Get an individual bias/clock signal voltage; vindex indicates which signal voltage
 * to set (range 0 .. 11), and value is the address of the result
 *
 * Return value:
 *
 *  XE_OK		success
 *  XE_NOTFOUND	serial invalid
 *	XE_NOTINIT	module not initialised
 *	XE_IOFAIL	module i/o failed
 *	XE_ARGERROR	argument error - vindex out of range
 */
fntype int __stdcall xcm_clm_get_voltage(int serial,short vindex,short *value);

/*
 * Load the named sequencer program file into the rack.
 *
 * Return value:
 *
 *  XE_OK		success
 *  XE_NOTFOUND	serial invalid
 *	XE_NOTINIT	module not initialised
 *	XE_IOFAIL	module i/o failed
 *	XE_NOFILE	file could not be opened
 *	XE_ARGERROR	argument error - file error encountered
 */
fntype int __stdcall xcm_clm_load_seq(int serial,const char *seqfile);

/*
 * Read a single sequencer program word of 24 bits. The index parameter indicates which
 * word to fetch in the range 0 .. 16383; the value parameter is the address to write it to.
 *
 * Data is returned as received from the interface card, in HM-L format
 *
 * Return value:
 *
 *  XE_OK		success
 *  XE_NOTFOUND	serial invalid
 *	XE_NOTINIT	module not initialised
 *	XE_IOFAIL	module i/o failed
 *	XE_ARGERROR	argument error - index out of range
 */
fntype int __stdcall xcm_clm_read_seq(int serial,int index,unsigned int *value);

/*
 * Set a single sequencer timing parameter. The index parameter indicates which
 * parameter to set to value, and must lie in the range 0 .. 99
 *
 * Return value:
 *
 *  XE_OK		success
 *  XE_NOTFOUND	serial invalid
 *	XE_NOTINIT	module not initialised
 *	XE_IOFAIL	module i/o failed
 *	XE_ARGERROR	argument error - index out of range
 */
fntype int __stdcall xcm_clm_set_param(int serial,short index,short value);

/*
 * Get a single sequencer timing parameter. The index parameter indicates which
 * parameter to fetch in the range 0 .. 99; the value parameter is the address to write it to
 *
 * Return value:
 *
 *  XE_OK		success
 *  XE_NOTFOUND	serial invalid
 *	XE_NOTINIT	module not initialised
 *	XE_IOFAIL	module i/o failed
 *	XE_ARGERROR	argument error - index out of range
 */
fntype int __stdcall xcm_clm_get_param(int serial,short index,short *value);

/*
 * Write a byte to the interface card eeprom
 *
 * Return value:
 *
 *	XE_OK		success
 *	XE_NOTFOUND	serial invalid
 *	XE_NOTINIT	module not initialised
 *	XE_ARGERR	argument error - address invalid
 *	XE_IOFAIL	module i/o failed
 */
fntype int __stdcall xcm_clm_eepwrite(int serial,int address,int data);

/*
 * Read a byte from the interface card eeprom
 *
 * Return value:
 *
 *	XE_OK		success
 *	XE_NOTFOUND	serial invalid
 *	XE_NOTINIT	module not initialised
 *	XE_ARGERR	argument error - address invalid
 *	XE_IOFAIL	module i/o failed
 */
fntype int __stdcall xcm_clm_eepread(int serial,int address,int *data);

/*
 * Set the read timeout for channel, where timeout is in millseconds
 *
 * Return value:
 *
 *  XE_OK		success
 *  XE_NOTFOUND	serial invalid
 *	XE_NOTINIT	module not initialised
 */
fntype int __stdcall xcm_clm_set_timeout(int serial,long timeout);

/*
 * Reset the interface - primarily the sequencer, but this could be separated out later
 * if appropriate
 *
 * Return value:
 *	
 *	XE_OK		success
 *  XE_NOTFOUND	serial invalid
 *	XE_NOTINIT	module not initialised
 *	XE_IOFAIL	module i/o failed
 */
fntype int __stdcall xcm_clm_interface_reset(int serial);

/*
 * Generate a pulse on one of the NI card's TTL i/o lines. The pulse will be active low
 * because the i/o lines default to 10k pullup inputs. The delay and width arguments are
 * in milliseconds.
 *
 * Return value:
 *	
 *	XE_OK		success
 *  XE_NOTFOUND	serial invalid
 *	XE_NOTINIT	module not initialised
 *	XE_ARGERROR	argument error - trigger line invalid (must be 0..3)
 */
fntype int __stdcall xcm_clm_pulse(int serial,int trigline,unsigned int delay,unsigned int width);

/*
 * Set up channel grab parameters. Note that numcols, numrows, and the region
 * of interest top, left, height and width must all be even, in order to avoid
 * possible grab failure in the CameraLink frame grabber interface.
 *
 * Return value:
 *
 *  XE_OK		success
 *  XE_NOTFOUND	serial invalid
 *	XE_NOTINIT	module not initialised
 *	XE_ARGERROR	argument error - row, cols, roi args must all be even
 */
fntype int __stdcall xcm_clm_grab_setup(int serial,int numcols,int numrows,
										int top,int left,int height,int width,
										BYTE *databuffer);

fntype int __stdcall xcm_clm_grab_setup_1node(int serial,int numcols,int numrows,
										int top,int left,int height,int width,
										BYTE *databuffer,int node);

fntype int __stdcall xcm_clm_grab_setup_deint4(int serial,int numcols,int numrows,
										int top,int left,int height,int width,
										BYTE *databuffer);

/*
 * The overscan param indicates the overscan to be added per row, where the
 * overscan is removed from the image before returning to the user. This quirky
 * behaviour allows correct black level estimates to maintained.
 */
fntype int __stdcall xcm_clm_set_overscan(int overscan);


/*
 * cdstype definitions for grab setup function
 */
#define XCM_CDS16_1		0x0000		/* 16 bit 1 channel 200kHz card */
#define XCM_CDS14_2		0x0100		/* 14 bit 2 channel 1MHz card */
#define XCM_CDS16_4		0x0200		/* 16 bit 4 channel 1MHz card */



/*
 * Grab data from channels indicated by the array of serial numbers passed as
 * the first argument.
 * 
 * The errorcodes parameter points to an array of integers which receives the
 * error codes from each channel's grab operation.
 *
 * Return value:
 *
 *  XE_OK		success
 *	XE_ARRAYLEN	array length too large
 *	XE_ERRORIND	errors indicated in errorcodes array
 *
 * Errorcodes value:
 *
 *  XE_OK		success
 *	XE_IOFAIL	module i/o failure
 *	XE_TIMEOUT	module read timed out
 */
fntype int __stdcall xcm_clm_grab(int *array,int *errorcodes,int arraylen);

/*
 * Cancel a grab operation
 *
 * Return value:
 *	
 *	XE_OK		success
 *  XE_NOTFOUND	serial invalid
 *	XE_NOTINIT	module not initialised
 *	XE_IOFAIL	module i/o failed
 */
fntype int __stdcall xcm_clm_cancel_grab(int serial);

/*
 * Terminate any open sessions with the frame grabbers
 *
 * Return values:
 *	
 *	XE_OK		success
 */
fntype int __stdcall xcm_clm_terminate(void);

/*
 * Load the named FPGA HEX file to the rack/camera head.
 *
 * Return value:
 *
 *  XE_OK		success
 *  XE_NOTFOUND	serial invalid
 *	XE_NOTINIT	module not initialised
 *	XE_IOFAIL	module i/o failed
 *	XE_NOFILE	file could not be opened
 *	XE_ARGERROR	argument error - file error encountered
 *	XE_MEMALLOC	malloc failure
 */
fntype int __stdcall xcm_clm_load_fpga(int serial, const char *filename, int destfpga);

/* destination FPGA definitions for this call and xcm_clm_read_fpga_sector() */
#define INTERFACE_CARD_FPGA			0
#define CAMERA_HEAD_FPGA			1
#define SEQ_FIBRE_OPTIC_FPGA		2
#define INTERFACE_FIBRE_OPTIC_FPGA	3	
#define CAMERA_HEAD_ANALOGUE_FPGA	4

/*
 * Read a sector from the selected FPGA
 *
 * Return value:
 *
 *  XE_OK		success
 *  XE_NOTFOUND	serial invalid
 *	XE_NOTINIT	module not initialised
 *	XE_IOFAIL	module i/o failed
 *	XE_ARGERROR	argument error
 */
fntype int __stdcall xcm_clm_read_fpga_sector(int serial, int destfpga, int secnum,unsigned char *buffer);

/*
 * Get camera serial number string from specified rack
 *
 * Return value:
 *
 *  XE_OK		success
 *  XE_NOTFOUND	serial invalid
 *	XE_NOTINIT	module not initialised
 *	XE_IOFAIL	module i/o failed
 */
fntype int __stdcall xcm_clm_get_cam_serial_number(int serial, char *value);

/*
 * Get interface card status
 *
 * Return value:
 *
 *  XE_OK		success
 *  XE_NOTFOUND	serial invalid
 *	XE_NOTINIT	module not initialised
 *	XE_IOFAIL	module i/o failed
 */
fntype int __stdcall xcm_clm_get_interface_status(int serial, short *status);

/*
 * Grab command with pointer passed into function
 *
 * Return value:
 *
 *  XE_OK		success
 *  XE_NOTFOUND	serial invalid
 *	XE_NOTINIT	module not initialised
 *	XE_IOFAIL	module i/o failed
 *	XE_TIMEOUT	module read timed out
 */
fntype int __stdcall xcm_clm_grab_pointer(int serial, BYTE *databuffer);

/*
 * Turn on offline mode if arg is non-zero - always returns E_OK
 *
 * Return value:
 *
 *  XE_OK		success
 */
fntype int __stdcall xcm_clm_offline(int flag);

/*
 * initialise camera head SPI & DACs
 *
 * Return value:
 *
 *  XE_OK		success
 *  XE_NOTFOUND	serial invalid
 *	XE_NOTINIT	module not initialised
 *	XE_IOFAIL	module i/o failed
 */
fntype int __stdcall xcm_clm_initialise_spi(int serial);

/*
 * start production (2nd) EPCS image
 *
 * Return value:
 *
 *  XE_OK		success
 *  XE_NOTFOUND	serial invalid
 *	XE_NOTINIT	module not initialised
 *	XE_IOFAIL	module i/o failed
 */
fntype int __stdcall xcm_clm_start_production_image(int serial,int destfpga);

/*
 * get timing estimates based on exposure time and known line readout and frame transfer times and 
 * configured rows and cols
 *
 * Return value:
 *
 *  XE_OK		success
 *  XE_NOTFOUND	serial invalid
 *	XE_NOTINIT	module not initialised
 *	XE_NULLBUFF	pointer to result is NULL
 */
fntype int __stdcall xcm_clm_get_timings(int serial,int texposure,int *treadout,int *tbusy,int *tcycle,int *buffsize);

/*
 * read fpga status
 *
 * Return value:
 *
 *  XE_OK		success
 *  XE_NOTFOUND	serial invalid
 *	XE_NOTINIT	module not initialised
 *	XE_NULLBUFF	pointer to buffer is NULL
 */
fntype int __stdcall xcm_clm_get_fpga_status(int serial,int destfpga);

/*
 * set grab speed
 *
 * Return value:
 *
 *  XE_OK		success
 *  XE_NOTFOUND	serial invalid
 *	XE_NOTINIT	module not initialised
 *	XE_IOFAIL	module i/o failed
 */
fntype int __stdcall xcm_clm_set_grab_speed(int serial, short int speed);

/*
 * send adc spi command
 *
 * Return value:
 *
 *  XE_OK		success
 *  XE_NOTFOUND	serial invalid
 *	XE_NOTINIT	module not initialised
 *	XE_IOFAIL	module i/o failed
 */
fntype int __stdcall xcm_clm_send_adc_spi_cmd(int serial,short addr,short value);

/*
* send dac spi command
*
* Return value:
*
*  XE_OK		success
*  XE_NOTFOUND	serial invalid
*  XE_NOTINIT	module not initialised
*  XE_IOFAIL	module i/o failed
*/
fntype int __stdcall xcm_clm_send_dac_spi_cmd(int serial, short addr, short value, short channel);

#define GETPLANTVAL	   0         	// Get current raw plant value (24 bit)
#define GETSETPOINT	   1         	// Get Temperature Setpoint (24 bit)
#define SETSETPOINT	   2         	// Set Temperature Setpoint (24 bit)
#define GETPROPGAIN	   3         	// Get Proportional Gain (8 bit)
#define SETPROPGAIN	   4         	// Set Proportional Gain (8 bit)
#define GETINTGAIN	   5         	// Get Integral Gain (8 bit)
#define SETINTGAIN	   6         	// Set Integral Gain (8 bit)
#define GETDERGAIN	   7         	// Get Derivative Gain (8 bit)
#define SETDERGAIN	   8         	// Set Derivative Gain (8 bit)
#define GETPROPTIME	   9         	// Get Proportional Cycle Rate (16 bit)
#define SETPROPTIME	   10			// Set Proportional Cycle Rate (16 bit)
#define GETIFACT	   11          	// Get Integral Rate (16 bit)
#define SETIFACT       12          	// Set integral Rate (16 bit)
#define GETDFACT       13          	// Get Derivative Rate (16 bit)
#define SETDFACT	   14          	// Set Derivative Rate (16 bit)
#define GETACCLIM	   15          	// Get accumulated error limit (24 bit)
#define SETACCLIM      16          	// Set accumulated error limit (24 bit)
#define GETOPBIAS	   17          	// Get Output Bias value (16 bit)
#define SETOPBIAS	   18          	// Get Output Bias value (16 bit)
#define MANUALMODE	   19          	// Switch from manual to auto with bias (1/0)

/*
 * send temperature controller command
 *
 * Return value:
 *
 *  XE_OK		success
 *  XE_NOTFOUND	serial invalid
 *	XE_NOTINIT	module not initialised
 *	XE_IOFAIL	module i/o failed
 */
fntype int __stdcall xcm_clm_send_temp_cntrl_command(int serial,short command,int data,int *retValue);

/*
* Configure I/O select bits for the temperature controller (Bit 0 : Temp Sensor Input; Bit 1 Heater Output)
*
* Return value:
*
*  XE_OK		success
*  XE_NOTFOUND	serial invalid
*  XE_NOTINIT	module not initialised
*  XE_IOFAIL	module i/o failed
*/
fntype int __stdcall xcm_clm_temp_cntrl_select_IO(int serial, short int selectbits);

/*
* Get the current temperature from the temperature controller (in DegC)
*
* Return value:
*
*  XE_OK		success
*  XE_NOTFOUND	serial invalid
*  XE_NOTINIT	module not initialised
*  XE_IOFAIL	module i/o failed
*/
fntype int __stdcall xcm_clm_get_current_temperature(int serial, double *currenttemp);

/*
* Set the target temperature for the temperature controller (in DegC)
*
* Return value:
*
*  XE_OK		success
*  XE_NOTFOUND	serial invalid
*  XE_NOTINIT	module not initialised
*  XE_IOFAIL	module i/o failed
*/
fntype int __stdcall xcm_clm_set_target_temperature(int serial, double targettemp);

/*
* Get the target temperature from the temperature controller (in DegC)
*
* Return value:
*
*  XE_OK		success
*  XE_NOTFOUND	serial invalid
*  XE_NOTINIT	module not initialised
*  XE_IOFAIL	module i/o failed
*/
fntype int __stdcall xcm_clm_get_target_temperature(int serial, double *settemp);


#ifdef __cplusplus
}
#endif

#endif /* XCMCLM_H */
