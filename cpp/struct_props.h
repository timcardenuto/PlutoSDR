#ifndef STRUCTPROPS_H
#define STRUCTPROPS_H

/*******************************************************************************************

    AUTO-GENERATED CODE. DO NOT MODIFY

*******************************************************************************************/

#include <ossie/CorbaUtils.h>
#include <CF/cf.h>
#include <ossie/PropertyMap.h>
#include <bulkio/bulkio.h>
typedef bulkio::connection_descriptor_struct connection_descriptor_struct;

#include <frontend/fe_tuner_struct_props.h>

namespace enums {
    // Enumerated values for FRONTEND::tuner_status_struct
    namespace frontend_tuner_status_struct {
        // Enumerated values for FRONTEND::tuner_status::reference_source
        namespace reference_source {
            static const CORBA::Long INTERNAL = 0;
            static const CORBA::Long EXTERNAL = 1;
        }
    }
}

struct frontend_tuner_status_struct_struct : public frontend::default_frontend_tuner_status_struct_struct {
    frontend_tuner_status_struct_struct () : frontend::default_frontend_tuner_status_struct_struct()
    {
    }

    static std::string getId() {
        return std::string("FRONTEND::tuner_status_struct");
    }

    static const char* getFormat() {
        return "bsssssdddbibdsisddhsb";
    }

    bool agc;
    std::string available_bandwidth;
    std::string available_frequency;
    std::string available_gain;
    std::string available_sample_rate;
    double bandwidth_tolerance;
    bool complex;
    CORBA::Long decimation;
    double gain;
    CORBA::Long reference_source;
    double sample_rate_tolerance;
    short tuner_number;
    bool valid;
};

inline bool operator>>= (const CORBA::Any& a, frontend_tuner_status_struct_struct& s) {
    CF::Properties* temp;
    if (!(a >>= temp)) return false;
    const redhawk::PropertyMap& props = redhawk::PropertyMap::cast(*temp);
    if (props.contains("FRONTEND::tuner_status::agc")) {
        if (!(props["FRONTEND::tuner_status::agc"] >>= s.agc)) return false;
    }
    if (props.contains("FRONTEND::tuner_status::allocation_id_csv")) {
        if (!(props["FRONTEND::tuner_status::allocation_id_csv"] >>= s.allocation_id_csv)) return false;
    }
    if (props.contains("FRONTEND::tuner_status::available_bandwidth")) {
        if (!(props["FRONTEND::tuner_status::available_bandwidth"] >>= s.available_bandwidth)) return false;
    }
    if (props.contains("FRONTEND::tuner_status::available_frequency")) {
        if (!(props["FRONTEND::tuner_status::available_frequency"] >>= s.available_frequency)) return false;
    }
    if (props.contains("FRONTEND::tuner_status::available_gain")) {
        if (!(props["FRONTEND::tuner_status::available_gain"] >>= s.available_gain)) return false;
    }
    if (props.contains("FRONTEND::tuner_status::available_sample_rate")) {
        if (!(props["FRONTEND::tuner_status::available_sample_rate"] >>= s.available_sample_rate)) return false;
    }
    if (props.contains("FRONTEND::tuner_status::bandwidth")) {
        if (!(props["FRONTEND::tuner_status::bandwidth"] >>= s.bandwidth)) return false;
    }
    if (props.contains("FRONTEND::tuner_status::bandwidth_tolerance")) {
        if (!(props["FRONTEND::tuner_status::bandwidth_tolerance"] >>= s.bandwidth_tolerance)) return false;
    }
    if (props.contains("FRONTEND::tuner_status::center_frequency")) {
        if (!(props["FRONTEND::tuner_status::center_frequency"] >>= s.center_frequency)) return false;
    }
    if (props.contains("FRONTEND::tuner_status::complex")) {
        if (!(props["FRONTEND::tuner_status::complex"] >>= s.complex)) return false;
    }
    if (props.contains("FRONTEND::tuner_status::decimation")) {
        if (!(props["FRONTEND::tuner_status::decimation"] >>= s.decimation)) return false;
    }
    if (props.contains("FRONTEND::tuner_status::enabled")) {
        if (!(props["FRONTEND::tuner_status::enabled"] >>= s.enabled)) return false;
    }
    if (props.contains("FRONTEND::tuner_status::gain")) {
        if (!(props["FRONTEND::tuner_status::gain"] >>= s.gain)) return false;
    }
    if (props.contains("FRONTEND::tuner_status::group_id")) {
        if (!(props["FRONTEND::tuner_status::group_id"] >>= s.group_id)) return false;
    }
    if (props.contains("FRONTEND::tuner_status::reference_source")) {
        if (!(props["FRONTEND::tuner_status::reference_source"] >>= s.reference_source)) return false;
    }
    if (props.contains("FRONTEND::tuner_status::rf_flow_id")) {
        if (!(props["FRONTEND::tuner_status::rf_flow_id"] >>= s.rf_flow_id)) return false;
    }
    if (props.contains("FRONTEND::tuner_status::sample_rate")) {
        if (!(props["FRONTEND::tuner_status::sample_rate"] >>= s.sample_rate)) return false;
    }
    if (props.contains("FRONTEND::tuner_status::sample_rate_tolerance")) {
        if (!(props["FRONTEND::tuner_status::sample_rate_tolerance"] >>= s.sample_rate_tolerance)) return false;
    }
    if (props.contains("FRONTEND::tuner_status::tuner_number")) {
        if (!(props["FRONTEND::tuner_status::tuner_number"] >>= s.tuner_number)) return false;
    }
    if (props.contains("FRONTEND::tuner_status::tuner_type")) {
        if (!(props["FRONTEND::tuner_status::tuner_type"] >>= s.tuner_type)) return false;
    }
    if (props.contains("FRONTEND::tuner_status::valid")) {
        if (!(props["FRONTEND::tuner_status::valid"] >>= s.valid)) return false;
    }
    return true;
}

inline void operator<<= (CORBA::Any& a, const frontend_tuner_status_struct_struct& s) {
    redhawk::PropertyMap props;
 
    props["FRONTEND::tuner_status::agc"] = s.agc;
 
    props["FRONTEND::tuner_status::allocation_id_csv"] = s.allocation_id_csv;
 
    props["FRONTEND::tuner_status::available_bandwidth"] = s.available_bandwidth;
 
    props["FRONTEND::tuner_status::available_frequency"] = s.available_frequency;
 
    props["FRONTEND::tuner_status::available_gain"] = s.available_gain;
 
    props["FRONTEND::tuner_status::available_sample_rate"] = s.available_sample_rate;
 
    props["FRONTEND::tuner_status::bandwidth"] = s.bandwidth;
 
    props["FRONTEND::tuner_status::bandwidth_tolerance"] = s.bandwidth_tolerance;
 
    props["FRONTEND::tuner_status::center_frequency"] = s.center_frequency;
 
    props["FRONTEND::tuner_status::complex"] = s.complex;
 
    props["FRONTEND::tuner_status::decimation"] = s.decimation;
 
    props["FRONTEND::tuner_status::enabled"] = s.enabled;
 
    props["FRONTEND::tuner_status::gain"] = s.gain;
 
    props["FRONTEND::tuner_status::group_id"] = s.group_id;
 
    props["FRONTEND::tuner_status::reference_source"] = s.reference_source;
 
    props["FRONTEND::tuner_status::rf_flow_id"] = s.rf_flow_id;
 
    props["FRONTEND::tuner_status::sample_rate"] = s.sample_rate;
 
    props["FRONTEND::tuner_status::sample_rate_tolerance"] = s.sample_rate_tolerance;
 
    props["FRONTEND::tuner_status::tuner_number"] = s.tuner_number;
 
    props["FRONTEND::tuner_status::tuner_type"] = s.tuner_type;
 
    props["FRONTEND::tuner_status::valid"] = s.valid;
    a <<= props;
}

inline bool operator== (const frontend_tuner_status_struct_struct& s1, const frontend_tuner_status_struct_struct& s2) {
    if (s1.agc!=s2.agc)
        return false;
    if (s1.allocation_id_csv!=s2.allocation_id_csv)
        return false;
    if (s1.available_bandwidth!=s2.available_bandwidth)
        return false;
    if (s1.available_frequency!=s2.available_frequency)
        return false;
    if (s1.available_gain!=s2.available_gain)
        return false;
    if (s1.available_sample_rate!=s2.available_sample_rate)
        return false;
    if (s1.bandwidth!=s2.bandwidth)
        return false;
    if (s1.bandwidth_tolerance!=s2.bandwidth_tolerance)
        return false;
    if (s1.center_frequency!=s2.center_frequency)
        return false;
    if (s1.complex!=s2.complex)
        return false;
    if (s1.decimation!=s2.decimation)
        return false;
    if (s1.enabled!=s2.enabled)
        return false;
    if (s1.gain!=s2.gain)
        return false;
    if (s1.group_id!=s2.group_id)
        return false;
    if (s1.reference_source!=s2.reference_source)
        return false;
    if (s1.rf_flow_id!=s2.rf_flow_id)
        return false;
    if (s1.sample_rate!=s2.sample_rate)
        return false;
    if (s1.sample_rate_tolerance!=s2.sample_rate_tolerance)
        return false;
    if (s1.tuner_number!=s2.tuner_number)
        return false;
    if (s1.tuner_type!=s2.tuner_type)
        return false;
    if (s1.valid!=s2.valid)
        return false;
    return true;
}

inline bool operator!= (const frontend_tuner_status_struct_struct& s1, const frontend_tuner_status_struct_struct& s2) {
    return !(s1==s2);
}

struct channel_struct {
    channel_struct ()
    {
        channel_name = "";
        channel_num = 0;
        tuner_type = "";
        bandwidth_min = 0;
        bandwidth_max = 0;
        samplerate_min = 0;
        samplerate_max = 0;
        frequency_min = 0;
        frequency_max = 0;
        gain_min = 0;
        gain_max = 0;
        clock_min = 0;
        clock_max = 0;
    }

    static std::string getId() {
        return std::string("device_channels::channel");
    }

    static const char* getFormat() {
        return "shsdddddddddd[s]";
    }

    std::string channel_name;
    short channel_num;
    std::string tuner_type;
    double bandwidth_min;
    double bandwidth_max;
    double samplerate_min;
    double samplerate_max;
    double frequency_min;
    double frequency_max;
    double gain_min;
    double gain_max;
    double clock_min;
    double clock_max;
    std::vector<std::string> available_antennas;
};

inline bool operator>>= (const CORBA::Any& a, channel_struct& s) {
    CF::Properties* temp;
    if (!(a >>= temp)) return false;
    const redhawk::PropertyMap& props = redhawk::PropertyMap::cast(*temp);
    if (props.contains("device_channels::channel_name")) {
        if (!(props["device_channels::channel_name"] >>= s.channel_name)) return false;
    }
    if (props.contains("device_channels::channel_num")) {
        if (!(props["device_channels::channel_num"] >>= s.channel_num)) return false;
    }
    if (props.contains("device_channels::tuner_type")) {
        if (!(props["device_channels::tuner_type"] >>= s.tuner_type)) return false;
    }
    if (props.contains("device_channels::bandwidth_min")) {
        if (!(props["device_channels::bandwidth_min"] >>= s.bandwidth_min)) return false;
    }
    if (props.contains("device_channels::bandwidth_max")) {
        if (!(props["device_channels::bandwidth_max"] >>= s.bandwidth_max)) return false;
    }
    if (props.contains("device_channels::samplerate_min")) {
        if (!(props["device_channels::samplerate_min"] >>= s.samplerate_min)) return false;
    }
    if (props.contains("device_channels::samplerate_max")) {
        if (!(props["device_channels::samplerate_max"] >>= s.samplerate_max)) return false;
    }
    if (props.contains("device_channels::frequency_min")) {
        if (!(props["device_channels::frequency_min"] >>= s.frequency_min)) return false;
    }
    if (props.contains("device_channels::frequency_max")) {
        if (!(props["device_channels::frequency_max"] >>= s.frequency_max)) return false;
    }
    if (props.contains("device_channels::gain_min")) {
        if (!(props["device_channels::gain_min"] >>= s.gain_min)) return false;
    }
    if (props.contains("device_channels::gain_max")) {
        if (!(props["device_channels::gain_max"] >>= s.gain_max)) return false;
    }
    if (props.contains("device_channels::clock_min")) {
        if (!(props["device_channels::clock_min"] >>= s.clock_min)) return false;
    }
    if (props.contains("device_channels::clock_max")) {
        if (!(props["device_channels::clock_max"] >>= s.clock_max)) return false;
    }
    if (props.contains("device_channels::available_antennas")) {
        if (!(props["device_channels::available_antennas"] >>= s.available_antennas)) return false;
    }
    return true;
}

inline void operator<<= (CORBA::Any& a, const channel_struct& s) {
    redhawk::PropertyMap props;
 
    props["device_channels::channel_name"] = s.channel_name;
 
    props["device_channels::channel_num"] = s.channel_num;
 
    props["device_channels::tuner_type"] = s.tuner_type;
 
    props["device_channels::bandwidth_min"] = s.bandwidth_min;
 
    props["device_channels::bandwidth_max"] = s.bandwidth_max;
 
    props["device_channels::samplerate_min"] = s.samplerate_min;
 
    props["device_channels::samplerate_max"] = s.samplerate_max;
 
    props["device_channels::frequency_min"] = s.frequency_min;
 
    props["device_channels::frequency_max"] = s.frequency_max;
 
    props["device_channels::gain_min"] = s.gain_min;
 
    props["device_channels::gain_max"] = s.gain_max;
 
    props["device_channels::clock_min"] = s.clock_min;
 
    props["device_channels::clock_max"] = s.clock_max;
 
    props["device_channels::available_antennas"] = s.available_antennas;
    a <<= props;
}

inline bool operator== (const channel_struct& s1, const channel_struct& s2) {
    if (s1.channel_name!=s2.channel_name)
        return false;
    if (s1.channel_num!=s2.channel_num)
        return false;
    if (s1.tuner_type!=s2.tuner_type)
        return false;
    if (s1.bandwidth_min!=s2.bandwidth_min)
        return false;
    if (s1.bandwidth_max!=s2.bandwidth_max)
        return false;
    if (s1.samplerate_min!=s2.samplerate_min)
        return false;
    if (s1.samplerate_max!=s2.samplerate_max)
        return false;
    if (s1.frequency_min!=s2.frequency_min)
        return false;
    if (s1.frequency_max!=s2.frequency_max)
        return false;
    if (s1.gain_min!=s2.gain_min)
        return false;
    if (s1.gain_max!=s2.gain_max)
        return false;
    if (s1.clock_min!=s2.clock_min)
        return false;
    if (s1.clock_max!=s2.clock_max)
        return false;
    if (s1.available_antennas!=s2.available_antennas)
        return false;
    return true;
}

inline bool operator!= (const channel_struct& s1, const channel_struct& s2) {
    return !(s1==s2);
}

#endif // STRUCTPROPS_H
