/**************************************************************************

    This is the device code. This file contains the child class where
    custom functionality can be added to the device. Custom
    functionality to the base class can be extended here. Access to
    the ports can also be done from this class

**************************************************************************/

#include "PlutoSDR.h"

PREPARE_LOGGING(PlutoSDR_i)

PlutoSDR_i::PlutoSDR_i(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl) :
    PlutoSDR_base(devMgr_ior, id, lbl, sftwrPrfl)
{
}

PlutoSDR_i::PlutoSDR_i(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, char *compDev) :
    PlutoSDR_base(devMgr_ior, id, lbl, sftwrPrfl, compDev)
{
}

PlutoSDR_i::PlutoSDR_i(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, CF::Properties capacities) :
    PlutoSDR_base(devMgr_ior, id, lbl, sftwrPrfl, capacities)
{
}

PlutoSDR_i::PlutoSDR_i(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, CF::Properties capacities, char *compDev) :
    PlutoSDR_base(devMgr_ior, id, lbl, sftwrPrfl, capacities, compDev)
{
}

PlutoSDR_i::~PlutoSDR_i()
{
}

void PlutoSDR_i::constructor()
{
    /***********************************************************************************
     This is the RH constructor. All properties are properly initialized before this function is called 

     For a tuner device, the structure frontend_tuner_status needs to match the number
     of tuners that this device controls and what kind of device it is.
     The options for devices are: TX, RX, RX_DIGITIZER, CHANNELIZER, DDC, RC_DIGITIZER_CHANNELIZER
     
     For example, if this device has 5 physical
     tuners, 3 RX_DIGITIZER and 2 CHANNELIZER, then the code in the construct function 
     should look like this:

     this->addChannels(3, "RX_DIGITIZER");
     this->addChannels(2, "CHANNELIZER");
     
     The incoming request for tuning contains a string describing the requested tuner
     type. The string for the request must match the string in the tuner status.
    ***********************************************************************************/
    try {
    	// Find PlutoSDR URI (address)
    	if (uri.empty()) {
			RH_INFO(this->_baseLog, "No URI specified, scanning for one.");
			uri = findPlutoSDR();
    	}

    	// Create PlutoSDR context
    	global_ctx = iio_create_context_from_uri(uri.c_str());
    	RH_INFO(this->_baseLog, "Created context for uri " << uri);

    	// Get PlutoSDR RX/TX info and populate device_channels property
    	getAD9361PHYDeviceInfo(global_ctx);

        this->addChannels(1, "RX_DIGITIZER");
        this->addChannels(1, "TX");

    	// Until you call start on the Redhawk Device, the ServiceFunction won't be run.
    	start();
    } catch(...) {
    	RH_ERROR(this->_baseLog, "Caught exception when initializing PlutoSDR");
    }
}

/* Grab all the device info from a specified URI, or find one dynamically.
 * Puts the device info into the property struct device_channels
 */
std::string PlutoSDR_i::findPlutoSDR() {
	struct iio_scan_context* scan_ctx = iio_create_scan_context(NULL, 0);
	if (!scan_ctx) {
		RH_ERROR(this->_baseLog, "Unable to create scan context.");
		exit(1);
	}

	struct iio_context_info **info;
	ssize_t ret = iio_scan_context_get_info_list(scan_ctx, &info);
	if (ret < 0) {
		RH_ERROR(this->_baseLog, "Unable to scan, returned  " << (long)ret);
		iio_scan_context_destroy(scan_ctx);
		exit(1);
	}
	if (ret == 0) {
		RH_ERROR(this->_baseLog, "No contexts found.");
		iio_context_info_list_free(info);
		iio_scan_context_destroy(scan_ctx);
		exit(1);
	}

	RH_INFO(this->_baseLog, "Available contexts:");
	std::string uri;
	unsigned int i = 0;
	for (i = 0; i < (size_t)ret; i++) {
		uri = iio_context_info_get_uri(info[i]);
		RH_INFO(this->_baseLog, "	" << uri);
	}
	return uri;
}

// Print all channels for this device
void PlutoSDR_i::printDeviceChannels(struct iio_device* dev) {
	int num_channels = iio_device_get_channels_count(dev);
	int j = 0;
	for (j=0; j<num_channels; j++) {
		struct iio_channel* chn;
		chn =  iio_device_get_channel(dev, j);
		int num_attrs = iio_channel_get_attrs_count(chn);
		printf("\n    Channel %i, Attributes %i\n", j, num_attrs);
		const char* id = iio_channel_get_id(chn);
		const char* name = iio_channel_get_name(chn);
		bool isoutput = iio_channel_is_output(chn);
		bool isscanel = iio_channel_is_scan_element(chn);
		printf("      %-30s: %s\n", "id", id);
		printf("      %-30s: %s\n", "name", name);
		if (isoutput) { printf("      %-30s: %s\n", "I/O", "output"); }
		else { printf("      %-30s: %s\n", "I/O", "input"); }
		printf("      %-30s: %i\n", "scan_element", isscanel);

		// Get all attributes for this channel
		int k = 0;
		for (k=0; k<num_attrs; k++) {
			const char* attr = iio_channel_get_attr(chn, k);

			char buf[2048];
			int ret = iio_channel_attr_read(chn, attr, buf, sizeof(buf));
			if (ret > 0) {
				printf("      %-30s: %s\n", attr, buf);
			} else {
				iio_strerror(-ret, buf, sizeof(buf));
				fprintf(stderr, "[ERROR]  Unable to read attribute %s: %s\n", attr, buf);
			}
		}
	}
}

// Print all attributes for this device
void PlutoSDR_i::printDeviceAttributes(struct iio_device* dev) {
	int num_attrs = iio_device_get_attrs_count(dev);
	int j = 0;
	for (j=0; j<num_attrs; j++) {
		const char* attr = iio_device_get_attr(dev, j);
		char buf[2048];
		int ret = iio_device_attr_read(dev, attr, buf, sizeof(buf));
		if (ret > 0) {
			printf("    %-30s: %s\n", attr, buf);
		} else {
			iio_strerror(-ret, buf, sizeof(buf));
			fprintf(stderr, "[ERROR]  Unable to read attribute %s: %s\n", attr, buf);
		}
	}
}

// Print 1 devices info
void PlutoSDR_i::printDeviceInfo(struct iio_device* dev) {
	const char* id = iio_device_get_id(dev);
	const char* name = iio_device_get_name(dev);
	printf("    %-30s: %s\n", "id", id);
	printf("    %-30s: %s\n", "name", name);
	printDeviceAttributes(dev);
	printDeviceChannels(dev);
}

// Print info for all devices in a given context
void PlutoSDR_i::printDevicesInfo(struct iio_context* ctx) {
	int num_devs = iio_context_get_devices_count(ctx);
	printf("\nDevices: %i\n",num_devs);
	int i = 0;
	for (i=0; i<num_devs; i++) {
		struct iio_device* dev = iio_context_get_device(ctx, i);
		printf("\n  Device %i\n", i);
		printDeviceInfo(dev);
	}
}

/* Helper function that parses PlutoSDR ranges
 * works for hardwaregain_available, sampling_frequency_available, rf_bandwidth_available, frequency_available
 */
int PlutoSDR_i::getAttributeRange(struct iio_channel* chn, const char* attr, double* min, double* max) {
	const char* id = iio_channel_get_id(chn);

	char buf[2048];
	int ret = iio_channel_attr_read(chn, attr, buf, sizeof(buf));
	if (ret > 0) {
		std::vector<std::string> strs;
		boost::split(strs, buf, boost::is_any_of(" "));
		std::vector<std::string> strs2;
		std::vector<std::string> strs3;
		boost::split(strs2, strs[0], boost::is_any_of("["));
		boost::split(strs3, strs[2], boost::is_any_of("]"));
		std::istringstream convert(strs2[1]);
		if ( !(convert >> *min) ) {
			RH_ERROR(this->_baseLog, "Could not convert string to double");
			return 1;
		}
		std::istringstream convert2(strs3[0]);
		if ( !(convert2 >> *max) ) {
			RH_ERROR(this->_baseLog, "Could not convert string to double");
			return 1;
		}
		RH_INFO(this->_baseLog, "Channel " << id << " " << attr << ", range: " << *min << " - " << *max)
		return 0;

	} else {
		RH_ERROR(this->_baseLog, "PlutoSDR iio_channel_attr_read return error code " << ret);
		return 1;
	}
}

// Populate device_channels property with the PlutoSDR info
void PlutoSDR_i::getAD9361PHYDeviceInfo(struct iio_context* ctx) {
	// NOTE: Because getting all the desired info for a PlutoSDR is confusing (located in multiple places, channel naming is weird, and has to be accessed by index)
	//       I'm just going to hardcode the fact that there are 2 channels per device, an RX and TX. And we can't map directly to a PlutoSDR "channel" or "device" because
	//       it isn't a one-to-one mapping. I honestly don't understand how they organized it.
	struct iio_device* dev = iio_context_find_device (ctx, "ad9361-phy");
	RH_INFO(this->_baseLog, "Pulling info from ad9361-phy");

	// Create new RX channel in device_channels
	channel_struct rx_channel;
	rx_channel.tuner_type = "RX_DIGITIZER";
	rx_channel.channel_name = "RX0";
	rx_channel.channel_num = 0;
	device_channels.push_back(rx_channel);

	// channel #=1, name=voltage0, I/O=input has most of the ADC values
	// channel #=3, name=altvoltage0, I/O=output has the ADC RF frequencies

	struct iio_channel* rx_chn;
	rx_chn =  iio_device_get_channel(dev, 1);
	RH_INFO(this->_baseLog, "Device 1, RX")

	// Get gain range
	double min, max;
	int ret = getAttributeRange(rx_chn, "hardwaregain_available", &min, &max);
	device_channels[0].gain_min = min;
	device_channels[0].gain_max = max;

	// Get sample range
	ret = getAttributeRange(rx_chn, "sampling_frequency_available", &min, &max);
	device_channels[0].samplerate_min = min;
	device_channels[0].samplerate_max = max;

	// Get bandwidth range
	ret = getAttributeRange(rx_chn, "rf_bandwidth_available", &min, &max);
	device_channels[0].bandwidth_min = min;
	device_channels[0].bandwidth_max = max;

	rx_chn =  iio_device_get_channel(dev, 3);
	RH_INFO(this->_baseLog, "Device 3, RX")

	// Get center frequency range
	ret = getAttributeRange(rx_chn, "frequency_available", &min, &max);
	device_channels[0].frequency_min = min;
	device_channels[0].frequency_max = max;


	// Create new TX channel in device_channels
	channel_struct tx_channel;
	tx_channel.tuner_type = "TX";
	tx_channel.channel_name = "TX0";
	tx_channel.channel_num = 1;
	device_channels.push_back(tx_channel);

	// channel #=6, name=voltage0, I/O=output has most of the DAC values
	// channel #=0, name=altvoltage1, I/O=output has the DAC RF frequencies

	struct iio_channel* tx_chn;
	tx_chn =  iio_device_get_channel(dev, 6);
	RH_INFO(this->_baseLog, "Device 6, TX")

	// Get gain range
	ret = getAttributeRange(tx_chn, "hardwaregain_available", &min, &max);
	device_channels[1].gain_min = min;
	device_channels[1].gain_max = max;

	// Get sample range
	ret = getAttributeRange(tx_chn, "sampling_frequency_available", &min, &max);
	device_channels[1].samplerate_min = min;
	device_channels[1].samplerate_max = max;

	// Get bandwidth range
	ret = getAttributeRange(tx_chn, "rf_bandwidth_available", &min, &max);
	device_channels[1].bandwidth_min = min;
	device_channels[1].bandwidth_max = max;

	tx_chn =  iio_device_get_channel(dev, 0);
	RH_INFO(this->_baseLog, "Device 0, TX")

	// Get center frequency range
	ret = getAttributeRange(tx_chn, "frequency_available", &min, &max);
	device_channels[1].frequency_min = min;
	device_channels[1].frequency_max = max;
}

// Get info for all devices in a given context
//void PlutoSDR_i::getDevicesInfo(struct iio_context* ctx) {
//	int num_devs = iio_context_get_devices_count(ctx);
//	int i = 0;
//	for (i=0; i<num_devs; i++) {
//		struct iio_device* dev = iio_context_get_device(ctx, i);
//		const char* name = iio_device_get_name(dev);
//
//		//---------------------------------------------------------
//		// TODO looking for specific things
//		std::string str_name(name);
//		if (str_name.compare("ad9361-phy") == 0) {		// RX?
//			RH_INFO(this->_baseLog, "Pulling info from device: " << name);
//			getAD9361PHYDeviceInfo(dev);
//
//		} else {
//			RH_INFO(this->_baseLog, "Skipping device: " << name);
//		}
//	}
//}

// Helper to print errors and old/new values of attributes
int PlutoSDR_i::setAttribute(struct iio_channel* chn, const char* attr, double value, double* retvalue) {
	const char* id = iio_channel_get_id(chn);
	// read current value
	long long val;
	int ret = iio_channel_attr_read_longlong(chn, attr, &val);
	if (ret != 0) {
		RH_INFO(this->_baseLog, "Failed to read " << attr << ", return code: " << ret);
	} else {
		RH_INFO(this->_baseLog, "Channel " << id << " " << attr << ", current value: " << val);
	}

	// set new value
	ret = iio_channel_attr_write_longlong(chn, attr, (long long)value); /* RX baseband rate 5 MSPS */
	if (ret != 0) {
		RH_INFO(this->_baseLog, "Failed to write " << attr << ", return code: " << ret);
		return 1;
	} else {
		ret = iio_channel_attr_read_longlong(chn, attr, &val);
		if (ret != 0) {
			RH_INFO(this->_baseLog, "Failed to read " << attr << ", return code: " << ret);
			return 1;
		} else {
			RH_INFO(this->_baseLog, "Channel " << id << " " << attr << ", new value: " << val);
			double dval = (double)val;
			*retvalue = dval;
			return 0;
		}
	}
	return 1;
}

/***********************************************************************************************

    Basic functionality:

        The service function is called by the serviceThread object (of type ProcessThread).
        This call happens immediately after the previous call if the return value for
        the previous call was NORMAL.
        If the return value for the previous call was NOOP, then the serviceThread waits
        an amount of time defined in the serviceThread's constructor.
        
    SRI:
        To create a StreamSRI object, use the following code:
                std::string stream_id = "testStream";
                BULKIO::StreamSRI sri = bulkio::sri::create(stream_id);

        To create a StreamSRI object based on tuner status structure index 'idx' and collector center frequency of 100:
                std::string stream_id = "my_stream_id";
                BULKIO::StreamSRI sri = this->create(stream_id, this->frontend_tuner_status[idx], 100);

    Time:
        To create a PrecisionUTCTime object, use the following code:
                BULKIO::PrecisionUTCTime tstamp = bulkio::time::utils::now();

        
    Ports:

        Data is passed to the serviceFunction through by reading from input streams
        (BulkIO only). The input stream class is a port-specific class, so each port
        implementing the BulkIO interface will have its own type-specific input stream.
        UDP multicast (dataSDDS and dataVITA49) ports do not support streams.

        The input stream from which to read can be requested with the getCurrentStream()
        method. The optional argument to getCurrentStream() is a floating point number that
        specifies the time to wait in seconds. A zero value is non-blocking. A negative value
        is blocking.  Constants have been defined for these values, bulkio::Const::BLOCKING and
        bulkio::Const::NON_BLOCKING.

        More advanced uses of input streams are possible; refer to the REDHAWK documentation
        for more details.

        Input streams return data blocks that automatically manage the memory for the data
        and include the SRI that was in effect at the time the data was received. It is not
        necessary to delete the block; it will be cleaned up when it goes out of scope.

        To send data using a BulkIO interface, create an output stream and write the
        data to it. When done with the output stream, the close() method sends and end-of-
        stream flag and cleans up.

        NOTE: If you have a BULKIO dataSDDS or dataVITA49  port, you must manually call 
              "port->updateStats()" to update the port statistics when appropriate.

        Example:
            // This example assumes that the device has two ports:
            //  An input (provides) port of type bulkio::InShortPort called dataShort_in
            //  An output (uses) port of type bulkio::OutFloatPort called dataFloat_out
            // The mapping between the port and the class is found
            // in the device base class header file

            bulkio::InShortStream inputStream = dataShort_in->getCurrentStream();
            if (!inputStream) { // No streams are available
                return NOOP;
            }

            // Get the output stream, creating it if it doesn't exist yet
            bulkio::OutFloatStream outputStream = dataFloat_out->getStream(inputStream.streamID());
            if (!outputStream) {
                outputStream = dataFloat_out->createStream(inputStream.sri());
            }

            bulkio::ShortDataBlock block = inputStream.read();
            if (!block) { // No data available
                // Propagate end-of-stream
                if (inputStream.eos()) {
                   outputStream.close();
                }
                return NOOP;
            }

            if (block.sriChanged()) {
                // Update output SRI
                outputStream.sri(block.sri());
            }

            // Get read-only access to the input data
            redhawk::shared_buffer<short> inputData = block.buffer();

            // Acquire a new buffer to hold the output data
            redhawk::buffer<float> outputData(inputData.size());

            // Transform input data into output data
            for (size_t index = 0; index < inputData.size(); ++index) {
                outputData[index] = (float) inputData[index];
            }

            // Write to the output stream; outputData must not be modified after
            // this method call
            outputStream.write(outputData, block.getStartTime());

            return NORMAL;

        If working with complex data (i.e., the "mode" on the SRI is set to
        true), the data block's complex() method will return true. Data blocks
        provide a cxbuffer() method that returns a complex interpretation of the
        buffer without making a copy:

            if (block.complex()) {
                redhawk::shared_buffer<std::complex<short> > inData = block.cxbuffer();
                redhawk::buffer<std::complex<float> > outData(inData.size());
                for (size_t index = 0; index < inData.size(); ++index) {
                    outData[index] = inData[index];
                }
                outputStream.write(outData, block.getStartTime());
            }

        Interactions with non-BULKIO ports are left up to the device developer's discretion
        
    Messages:
    
        To receive a message, you need (1) an input port of type MessageEvent, (2) a message prototype described
        as a structure property of kind message, (3) a callback to service the message, and (4) to register the callback
        with the input port.
        
        Assuming a property of type message is declared called "my_msg", an input port called "msg_input" is declared of
        type MessageEvent, create the following code:
        
        void PlutoSDR_i::my_message_callback(const std::string& id, const my_msg_struct &msg){
        }
        
        Register the message callback onto the input port with the following form:
        this->msg_input->registerMessage("my_msg", this, &PlutoSDR_i::my_message_callback);
        
        To send a message, you need to (1) create a message structure, (2) a message prototype described
        as a structure property of kind message, and (3) send the message over the port.
        
        Assuming a property of type message is declared called "my_msg", an output port called "msg_output" is declared of
        type MessageEvent, create the following code:
        
        ::my_msg_struct msg_out;
        this->msg_output->sendMessage(msg_out);

    Accessing the Device Manager and Domain Manager:
    
        Both the Device Manager hosting this Device and the Domain Manager hosting
        the Device Manager are available to the Device.
        
        To access the Domain Manager:
            CF::DomainManager_ptr dommgr = this->getDomainManager()->getRef();
        To access the Device Manager:
            CF::DeviceManager_ptr devmgr = this->getDeviceManager()->getRef();
    
    Properties:
        
        Properties are accessed directly as member variables. For example, if the
        property name is "baudRate", it may be accessed within member functions as
        "baudRate". Unnamed properties are given the property id as its name.
        Property types are mapped to the nearest C++ type, (e.g. "string" becomes
        "std::string"). All generated properties are declared in the base class
        (PlutoSDR_base).
    
        Simple sequence properties are mapped to "std::vector" of the simple type.
        Struct properties, if used, are mapped to C++ structs defined in the
        generated file "struct_props.h". Field names are taken from the name in
        the properties file; if no name is given, a generated name of the form
        "field_n" is used, where "n" is the ordinal number of the field.
        
        Example:
            // This example makes use of the following Properties:
            //  - A float value called scaleValue
            //  - A boolean called scaleInput
              
            if (scaleInput) {
                dataOut[i] = dataIn[i] * scaleValue;
            } else {
                dataOut[i] = dataIn[i];
            }
            
        Callback methods can be associated with a property so that the methods are
        called each time the property value changes.  This is done by calling 
        addPropertyListener(<property>, this, &PlutoSDR_i::<callback method>)
        in the constructor.

        The callback method receives two arguments, the old and new values, and
        should return nothing (void). The arguments can be passed by value,
        receiving a copy (preferred for primitive types), or by const reference
        (preferred for strings, structs and vectors).

        Example:
            // This example makes use of the following Properties:
            //  - A float value called scaleValue
            //  - A struct property called status
            
        //Add to PlutoSDR.cpp
        PlutoSDR_i::PlutoSDR_i(const char *uuid, const char *label) :
            PlutoSDR_base(uuid, label)
        {
            addPropertyListener(scaleValue, this, &PlutoSDR_i::scaleChanged);
            addPropertyListener(status, this, &PlutoSDR_i::statusChanged);
        }

        void PlutoSDR_i::scaleChanged(float oldValue, float newValue)
        {
            RH_DEBUG(this->_baseLog, "scaleValue changed from" << oldValue << " to " << newValue);
        }
            
        void PlutoSDR_i::statusChanged(const status_struct& oldValue, const status_struct& newValue)
        {
            RH_DEBUG(this->_baseLog, "status changed");
        }
            
        //Add to PlutoSDR.h
        void scaleChanged(float oldValue, float newValue);
        void statusChanged(const status_struct& oldValue, const status_struct& newValue);

    Logging:

        The member _baseLog is a logger whose base name is the component (or device) instance name.
        New logs should be created based on this logger name.

        To create a new logger,
            rh_logger::LoggerPtr my_logger = this->_baseLog->getChildLogger("foo");

        Assuming component instance name abc_1, my_logger will then be created with the 
        name "abc_1.user.foo".

    Allocation:
    
        Allocation callbacks are available to customize the Device's response to 
        allocation requests. For example, if the Device contains the allocation 
        property "my_alloc" of type string, the allocation and deallocation
        callbacks follow the pattern (with arbitrary function names
        my_alloc_fn and my_dealloc_fn):
        
        bool PlutoSDR_i::my_alloc_fn(const std::string &value)
        {
            // perform logic
            return true; // successful allocation
        }
        void PlutoSDR_i::my_dealloc_fn(const std::string &value)
        {
            // perform logic
        }
        
        The allocation and deallocation functions are then registered with the Device
        base class with the setAllocationImpl call. Note that the variable for the property is used rather
        than its id:
        
        this->setAllocationImpl(my_alloc, this, &PlutoSDR_i::my_alloc_fn, &PlutoSDR_i::my_dealloc_fn);
        
        

************************************************************************************************/
// RX thread
// TODO create separate TX/RX threads
int PlutoSDR_i::serviceFunction()
{
	// Redhawk Devices can be started with start() before the channels are set up so check for that.
	// TODO doesn't support simultaneous RX/TX
	if (frontend_tuner_status[0].enabled && frontend_tuner_status[1].enabled) {
		RH_ERROR(this->_baseLog, "PlutoSDR Redhawk Device does not support simultaneous RX/TX, disable one.");
		return NOOP;

	// RX operations
	} else if (frontend_tuner_status[0].enabled) {
		// Create a new buffer to hold the output samples
		redhawk::buffer< std::complex<short> > outputData(rxbuf_size);

		// Read input samples from PlutoSDR
		iio_buffer_refill(rxbuf);

		int idx = 0;
		for (rxp_dat = iio_buffer_first(rxbuf, rx0_i); rxp_dat < rxp_end; rxp_dat += rxp_inc, rxt_dat += rxp_inc) {
			//const int16_t i = ((int16_t*)p_dat)[0]; // Real (I)
			//const int16_t q = ((int16_t*)p_dat)[1]; // Imag (Q)

			// Copy samples from input buffer to output buffer
			outputData[idx] = std::complex<short>(((int16_t*)rxp_dat)[0], ((int16_t*)rxp_dat)[1]);
			idx++;
		}

		if (!outputStream) {
			RH_INFO(this->_baseLog, "Creating outputStream");
			outputStream = dataShort_out->createStream("PlutoSDR_RX0");
			outputStream.complex(true); // complex, interleaved I/Q
			outputStream.xdelta(1.0 / frontend_tuner_status[0].sample_rate); // TODO frontend_tuner_status[0] isn't guarenteed to be RX
			outputStream.setKeyword("CHAN_RF", frontend_tuner_status[0].center_frequency);
		}

		// To update values if calls to setTunerCenterFrequency() or setTunerOutputSampleRate() were made
		if (sriChanged) {
			// TODO what about bandwidth changes, whats the connection between that and sample rate for the PlutoSDR?
			outputStream.xdelta(1.0 / frontend_tuner_status[0].sample_rate); // TODO frontend_tuner_status[0] isn't guarenteed to be RX
			outputStream.setKeyword("CHAN_RF", frontend_tuner_status[0].center_frequency);
			sriChanged = false;
		}
		// Since samples aren't given a GPS timestamp in the hardware just use system time now
		// Write to the output stream; outputData must not be modified after this method call
		outputStream.write(outputData, bulkio::time::utils::now());
		//RH_INFO(this->_baseLog, "wrote data to output");
		return NORMAL;

	// TX operations
	} else if (frontend_tuner_status[1].enabled) {
		RH_INFO(this->_baseLog, "TX not implemented yet.");
		return NOOP;

	} else {
		return NOOP;
	}
}

/*************************************************************
Functions supporting tuning allocation
*************************************************************/
void PlutoSDR_i::deviceEnable(frontend_tuner_status_struct_struct &fts, size_t tuner_id){
    /************************************************************
    modify fts, which corresponds to this->frontend_tuner_status[tuner_id]
    Make sure to set the 'enabled' member of fts to indicate that tuner as enabled
    ************************************************************/
	RH_INFO(this->_baseLog, "deviceEnable() >>");
	struct iio_device* dev = iio_context_find_device(global_ctx, "cf-ad9361-lpc");

	if (fts.tuner_type == "RX_DIGITIZER") {
		rx0_i = iio_device_find_channel(dev, "voltage0", false);
		rx0_q = iio_device_find_channel(dev, "voltage1", false);

		iio_channel_enable(rx0_i);
		iio_channel_enable(rx0_q);

		rxbuf_size = 4096;
		rxbuf = iio_device_create_buffer(dev, rxbuf_size, false);
		if (!rxbuf) {
			RH_ERROR(this->_baseLog, "Could not create PlutoSDR RX buffer");
			exit(-1);
		}

		rxp_inc = iio_buffer_step(rxbuf);
		rxp_end = iio_buffer_end(rxbuf);

	} else if (fts.tuner_type == "TX") {
		tx0_i = iio_device_find_channel(dev, "voltage0", true);
		tx0_q = iio_device_find_channel(dev, "voltage1", true);

		iio_channel_enable(tx0_i);
		iio_channel_enable(tx0_q);

		txbuf_size = 4096;
		txbuf = iio_device_create_buffer(dev, txbuf_size, false);
		if (!txbuf) {
			RH_ERROR(this->_baseLog, "Could not create PlutoSDR TX buffer");
			exit(-1);
		}

		txp_inc = iio_buffer_step(txbuf);
		txp_end = iio_buffer_end(txbuf);

	} else {
		RH_ERROR(this->_baseLog, "deviceDisable() called with unknown tuner type: " << fts.tuner_type);
	}

    fts.enabled = true;
	RH_INFO(this->_baseLog, "deviceEnable() <<");
    return;
}
void PlutoSDR_i::deviceDisable(frontend_tuner_status_struct_struct &fts, size_t tuner_id){
    /************************************************************
    modify fts, which corresponds to this->frontend_tuner_status[tuner_id]
    Make sure to reset the 'enabled' member of fts to indicate that tuner as disabled
    ************************************************************/
	RH_INFO(this->_baseLog, "deviceDisable() >>");
	fts.enabled = false;
	RH_INFO(this->_baseLog, "deviceDisable() <<");
    return;
}
bool PlutoSDR_i::deviceSetTuning(const frontend::frontend_tuner_allocation_struct &request, frontend_tuner_status_struct_struct &fts, size_t tuner_id){
    /************************************************************
    modify fts, which corresponds to this->frontend_tuner_status[tuner_id]
      At a minimum, bandwidth, center frequency, and sample_rate have to be set
      If the device is tuned to exactly what the request was, the code should be:
        fts.bandwidth = request.bandwidth;
        fts.center_frequency = request.center_frequency;
        fts.sample_rate = request.sample_rate;

    return true if the tuning succeeded, and false if it failed
    ************************************************************/
	RH_INFO(this->_baseLog, "deviceSetTuning() >>");

	struct iio_device* dev = iio_context_find_device(global_ctx, "ad9361-phy");

	if (request.tuner_type == "RX_DIGITIZER") {
		struct iio_channel* chn = iio_device_find_channel(dev, "voltage0", false);
		double retvalue = 0;

		// set bandwidth
		int ret = setAttribute(chn, "rf_bandwidth", request.bandwidth, &retvalue);
		if (ret != 0) { return false; }
		fts.bandwidth = retvalue;

		// get the available bandwidth
		double min, max;
		ret = getAttributeRange(chn, "rf_bandwidth_available", &min, &max);
		if (ret != 0) { return false; }
		std::ostringstream s;
		s << min << "-" << max;
		fts.available_bandwidth = s.str();

		// set sample rate
		ret = setAttribute(chn, "sampling_frequency", request.sample_rate, &retvalue);
		if (ret != 0) { return false; }
		fts.sample_rate = retvalue+1;	// TODO common case where return BW is '1' less than the request and the tolerance in Redhawk only works for >= the request.... dumb.

		// get the available sample_rate
		ret = getAttributeRange(chn, "sampling_frequency_available", &min, &max);
		if (ret != 0) { return false; }
		s.str(""); // clears s
		s << min << "-" << max;
		fts.available_sample_rate = s.str();

		// get the current gain
		long long val;
		ret = iio_channel_attr_read_longlong(chn, "hardwaregain", &val);
		if (ret != 0) { RH_INFO(this->_baseLog, "Failed to read " << "hardwaregain" << ", return code: " << ret); }
		fts.gain = (double)val;

		// get the available gain
		ret = getAttributeRange(chn, "hardwaregain_available", &min, &max);
		if (ret != 0) { return false; }
		s.str(""); // clears s
		s << min << "-" << max;
		fts.available_gain = s.str();

		// set center frequency
		chn = iio_device_find_channel(dev, "altvoltage0", true); // center freq is set on the RX_LO channel which is an output type
		ret = setAttribute(chn, "frequency", request.center_frequency, &retvalue);
		if (ret != 0) { return false; }
		fts.center_frequency = retvalue;

		// get the available frequency
		ret = getAttributeRange(chn, "frequency_available", &min, &max);
		if (ret != 0) { return false; }
		s.str(""); // clears s
		s << min << "-" << max;
		fts.available_frequency = s.str();

		fts.bandwidth_tolerance = request.bandwidth_tolerance;
		fts.sample_rate_tolerance = request.sample_rate_tolerance;
		fts.group_id = request.group_id;
		fts.rf_flow_id = request.rf_flow_id;
		fts.reference_source = 0;
		fts.agc = false;
		fts.complex = false;
		fts.decimation = 0;
		fts.tuner_number = 0;

	} else if (request.tuner_type == "TX") {
		struct iio_channel* chn = iio_device_find_channel(dev, "voltage0", true);
		double retvalue = 0;

		int ret = setAttribute(chn, "rf_bandwidth", request.bandwidth, &retvalue);
		if (ret != 0) { return false; }
		RH_INFO(this->_baseLog, "PlutoSDR returned bandwidth of " << retvalue);
		fts.bandwidth = retvalue;

		ret = setAttribute(chn, "sampling_frequency", request.sample_rate, &retvalue);
		if (ret != 0) { return false; }
		RH_INFO(this->_baseLog, "PlutoSDR returned sample_rate of " << retvalue);
		fts.sample_rate = retvalue+1;	// TODO common case where return BW is '1' less than the request and the tolerance in Redhawk only works for >= the request.... dumb.

		chn = iio_device_find_channel(dev, "altvoltage1", true);
		ret = setAttribute(chn, "frequency", request.center_frequency, &retvalue);
		if (ret != 0) { return false; }
		RH_INFO(this->_baseLog, "PlutoSDR returned center_frequency of " << retvalue);
		fts.center_frequency = retvalue;

	} else {
		RH_ERROR(this->_baseLog, "Allocation request for unrecognized tuner type: " << request.tuner_type);
		return false;
	}
	RH_INFO(this->_baseLog, "deviceSetTuning() <<");
    return true;
}
bool PlutoSDR_i::deviceDeleteTuning(frontend_tuner_status_struct_struct &fts, size_t tuner_id) {
    /************************************************************
    modify fts, which corresponds to this->frontend_tuner_status[tuner_id]
    return true if the tune deletion succeeded, and false if it failed
    ************************************************************/
	RH_INFO(this->_baseLog, "deviceDeleteTuning() >>");
	// TODO This gets called on a deallocate after the deviceDisable() gets called... what else should I do here vs there?
	fts.enabled = false; // TODO does deviceDeleteTuning() ever called without having already called deviceDisable()?
	try {

		if (fts.tuner_type == "RX_DIGITIZER") {
			if (outputStream) {
				outputStream.close();				// Destroy the Redhawk FEI output buffer
			}
			if (rxbuf) {
				iio_buffer_destroy(rxbuf);			// Destroy PlutoSDR receive buffer
			}
		} else if (fts.tuner_type == "TX") {
			if (txbuf) {
				iio_buffer_destroy(txbuf); 			// Destroy PlutoSDR transmit buffer
			}
		} else {
			RH_ERROR(this->_baseLog, "deviceDisable() called with unknown tuner type: " << fts.tuner_type);
		}

	} catch(...) {
		RH_ERROR(this->_baseLog, "Caught exception trying to close outputStream");
	}
	RH_INFO(this->_baseLog, "deviceDeleteTuning() <<");
    return true;
}

/*************************************************************
Functions servicing the tuner control port
*************************************************************/
std::string PlutoSDR_i::getTunerType(const std::string& allocation_id) {
    long idx = getTunerMapping(allocation_id);
    if (idx < 0) throw FRONTEND::FrontendException("Invalid allocation id");
    return frontend_tuner_status[idx].tuner_type;
}

bool PlutoSDR_i::getTunerDeviceControl(const std::string& allocation_id) {
    long idx = getTunerMapping(allocation_id);
    if (idx < 0) throw FRONTEND::FrontendException("Invalid allocation id");
    if (getControlAllocationId(idx) == allocation_id)
        return true;
    return false;
}

std::string PlutoSDR_i::getTunerGroupId(const std::string& allocation_id) {
    long idx = getTunerMapping(allocation_id);
    if (idx < 0) throw FRONTEND::FrontendException("Invalid allocation id");
    return frontend_tuner_status[idx].group_id;
}

std::string PlutoSDR_i::getTunerRfFlowId(const std::string& allocation_id) {
    long idx = getTunerMapping(allocation_id);
    if (idx < 0) throw FRONTEND::FrontendException("Invalid allocation id");
    return frontend_tuner_status[idx].rf_flow_id;
}

void PlutoSDR_i::setTunerCenterFrequency(const std::string& allocation_id, double freq) {
	RH_INFO(this->_baseLog, "setTunerCenterFrequency() >>");
	long idx = getTunerMapping(allocation_id);
    if (idx < 0) throw FRONTEND::FrontendException("Invalid allocation id");
    if(allocation_id != getControlAllocationId(idx))
        throw FRONTEND::FrontendException(("ID "+allocation_id+" does not have authorization to modify the tuner").c_str());
    if (freq<0) throw FRONTEND::BadParameterException("Center frequency cannot be less than 0");

    // set hardware to new value. Raise an exception if it's not possible
	struct iio_device* dev = iio_context_find_device(global_ctx, "ad9361-phy");
	struct iio_channel* chn;
	if (this->frontend_tuner_status[idx].tuner_type == "RX_DIGITIZER") {
		chn = iio_device_find_channel(dev, "altvoltage0", true);
	} else if (this->frontend_tuner_status[idx].tuner_type == "TX") {
		chn = iio_device_find_channel(dev, "altvoltage1", true);
	} else {
		throw FRONTEND::BadParameterException("setTunerCenterFrequency() for unrecognized tuner type.");
	}
	double retvalue = 0;
	int ret = setAttribute(chn, "frequency", freq, &retvalue);
	if (ret != 0) { throw FRONTEND::BadParameterException("PlutoSDR could not set freq, check other logs."); }
	if (retvalue > (freq+2) || retvalue < (freq-2)) { throw FRONTEND::BadParameterException("PlutoSDR returned freq different than the value requested by more than 2 Hz"); }

	sriChanged = true;
    this->frontend_tuner_status[idx].center_frequency = retvalue;
	RH_INFO(this->_baseLog, "setTunerCenterFrequency() <<");
}

double PlutoSDR_i::getTunerCenterFrequency(const std::string& allocation_id) {
    long idx = getTunerMapping(allocation_id);
    if (idx < 0) throw FRONTEND::FrontendException("Invalid allocation id");
    return frontend_tuner_status[idx].center_frequency;
}

void PlutoSDR_i::setTunerBandwidth(const std::string& allocation_id, double bw) {
	RH_INFO(this->_baseLog, "setTunerBandwidth() >>");
	long idx = getTunerMapping(allocation_id);
    if (idx < 0) throw FRONTEND::FrontendException("Invalid allocation id");
    if(allocation_id != getControlAllocationId(idx))
        throw FRONTEND::FrontendException(("ID "+allocation_id+" does not have authorization to modify the tuner").c_str());
    if (bw<0) throw FRONTEND::BadParameterException("Bandwidth cannot be less than 0");

    // set hardware to new value. Raise an exception if it's not possible
	struct iio_device* dev = iio_context_find_device(global_ctx, "ad9361-phy");
	struct iio_channel* chn;
	if (this->frontend_tuner_status[idx].tuner_type == "RX_DIGITIZER") {
		chn = iio_device_find_channel(dev, "voltage0", false);
	} else if (this->frontend_tuner_status[idx].tuner_type == "TX") {
		chn = iio_device_find_channel(dev, "voltage0", true);
	} else {
		throw FRONTEND::BadParameterException("setTunerBandwidth() for unrecognized tuner type.");
	}
	double retvalue = 0;
	int ret = setAttribute(chn, "rf_bandwidth", bw, &retvalue);
	if (ret != 0) { throw FRONTEND::BadParameterException("PlutoSDR could not set bw, check other logs."); }
	double tol = this->frontend_tuner_status[idx].bandwidth_tolerance;
	if (retvalue > (bw+(bw*tol)) || retvalue < (bw-(bw*tol))) { throw FRONTEND::BadParameterException("PlutoSDR set bw to different value than the exact request"); }

	sriChanged = true;
    this->frontend_tuner_status[idx].bandwidth = retvalue;
	RH_INFO(this->_baseLog, "setTunerBandwidth() <<");
}

double PlutoSDR_i::getTunerBandwidth(const std::string& allocation_id) {
    long idx = getTunerMapping(allocation_id);
    if (idx < 0) throw FRONTEND::FrontendException("Invalid allocation id");
    return frontend_tuner_status[idx].bandwidth;
}

void PlutoSDR_i::setTunerAgcEnable(const std::string& allocation_id, bool enable)
{
    throw FRONTEND::NotSupportedException("setTunerAgcEnable not supported");
}

bool PlutoSDR_i::getTunerAgcEnable(const std::string& allocation_id)
{
    throw FRONTEND::NotSupportedException("getTunerAgcEnable not supported");
}

void PlutoSDR_i::setTunerGain(const std::string& allocation_id, float gain) {
	RH_INFO(this->_baseLog, "setTunerGain() >>");
	long idx = getTunerMapping(allocation_id);
    if (idx < 0) throw FRONTEND::FrontendException("Invalid allocation id");
    if(allocation_id != getControlAllocationId(idx))
        throw FRONTEND::FrontendException(("ID "+allocation_id+" does not have authorization to modify the tuner").c_str());

    // set hardware to new value. Raise an exception if it's not possible
	struct iio_device* dev = iio_context_find_device(global_ctx, "ad9361-phy");
	struct iio_channel* chn;
	if (this->frontend_tuner_status[idx].tuner_type == "RX_DIGITIZER") {
		chn = iio_device_find_channel(dev, "voltage0", false);
	} else if (this->frontend_tuner_status[idx].tuner_type == "TX") {
		chn = iio_device_find_channel(dev, "voltage0", true);
	} else {
		throw FRONTEND::BadParameterException("setTunerGain() for unrecognized tuner type.");
	}
	double retvalue = 0;
	int ret = setAttribute(chn, "hardware_gain", gain, &retvalue);
	if (ret != 0) { throw FRONTEND::BadParameterException("PlutoSDR could not set gain, check other logs."); }
	if (retvalue != gain) { throw FRONTEND::BadParameterException("PlutoSDR set gain to different value than the exact request"); }

	sriChanged = true;
	this->frontend_tuner_status[idx].gain = retvalue;
	RH_INFO(this->_baseLog, "setTunerGain() <<");
}

float PlutoSDR_i::getTunerGain(const std::string& allocation_id) {
    long idx = getTunerMapping(allocation_id);
    if (idx < 0) throw FRONTEND::FrontendException("Invalid allocation id");
    return frontend_tuner_status[idx].gain;
}

void PlutoSDR_i::setTunerReferenceSource(const std::string& allocation_id, long source)
{
    throw FRONTEND::NotSupportedException("setTunerReferenceSource not supported");
}

long PlutoSDR_i::getTunerReferenceSource(const std::string& allocation_id)
{
    throw FRONTEND::NotSupportedException("getTunerReferenceSource not supported");
}

void PlutoSDR_i::setTunerEnable(const std::string& allocation_id, bool enable) {
    long idx = getTunerMapping(allocation_id);
    if (idx < 0) throw FRONTEND::FrontendException("Invalid allocation id");
    if(allocation_id != getControlAllocationId(idx))
        throw FRONTEND::FrontendException(("ID "+allocation_id+" does not have authorization to modify the tuner").c_str());
    // set hardware to new value. Raise an exception if it's not possible
    deviceEnable(this->frontend_tuner_status[idx], idx);
    this->frontend_tuner_status[idx].enabled = enable;
}

bool PlutoSDR_i::getTunerEnable(const std::string& allocation_id) {
    long idx = getTunerMapping(allocation_id);
    if (idx < 0) throw FRONTEND::FrontendException("Invalid allocation id");
    return frontend_tuner_status[idx].enabled;
}

void PlutoSDR_i::setTunerOutputSampleRate(const std::string& allocation_id, double sr) {
	RH_INFO(this->_baseLog, "setTunerOutputSampleRate() >>");
    long idx = getTunerMapping(allocation_id);
    if (idx < 0) throw FRONTEND::FrontendException("Invalid allocation id");
    if(allocation_id != getControlAllocationId(idx))
        throw FRONTEND::FrontendException(("ID "+allocation_id+" does not have authorization to modify the tuner").c_str());
    if (sr<0) throw FRONTEND::BadParameterException("Sample rate cannot be less than 0");

    // set hardware to new value. Raise an exception if it's not possible
	struct iio_device* dev = iio_context_find_device(global_ctx, "ad9361-phy");
	struct iio_channel* chn;
	if (this->frontend_tuner_status[idx].tuner_type == "RX_DIGITIZER") {
		chn = iio_device_find_channel(dev, "voltage0", false);
	} else if (this->frontend_tuner_status[idx].tuner_type == "TX") {
		chn = iio_device_find_channel(dev, "voltage0", true);
	} else {
		throw FRONTEND::BadParameterException("setTunerOutputSampleRate() for unrecognized tuner type.");
	}
	double retvalue = 0;
	int ret = setAttribute(chn, "sampling_frequency", sr, &retvalue);
	if (ret != 0) { throw FRONTEND::BadParameterException("PlutoSDR could not set sr, check other logs."); }
	double tol = this->frontend_tuner_status[idx].sample_rate_tolerance;
	if (retvalue > (sr+(sr*tol)) || retvalue < (sr-(sr*tol))) { throw FRONTEND::BadParameterException("PlutoSDR set sr to different value than the exact request"); }

	sriChanged = true;
    this->frontend_tuner_status[idx].sample_rate = retvalue;
	RH_INFO(this->_baseLog, "setTunerOutputSampleRate() <<");
}

double PlutoSDR_i::getTunerOutputSampleRate(const std::string& allocation_id){
    long idx = getTunerMapping(allocation_id);
    if (idx < 0) throw FRONTEND::FrontendException("Invalid allocation id");
    return frontend_tuner_status[idx].sample_rate;
}

/*************************************************************
Functions servicing the RFInfo port(s)
- port_name is the port over which the call was received
*************************************************************/
std::string PlutoSDR_i::get_rf_flow_id(const std::string& port_name)
{
    return std::string("none");
}

void PlutoSDR_i::set_rf_flow_id(const std::string& port_name, const std::string& id)
{
}

frontend::RFInfoPkt PlutoSDR_i::get_rfinfo_pkt(const std::string& port_name)
{
    frontend::RFInfoPkt pkt;
    return pkt;
}

void PlutoSDR_i::set_rfinfo_pkt(const std::string& port_name, const frontend::RFInfoPkt &pkt)
{
}

