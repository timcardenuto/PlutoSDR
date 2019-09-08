#ifndef PLUTOSDR_I_IMPL_H
#define PLUTOSDR_I_IMPL_H

#include "PlutoSDR_base.h"
#include <boost/algorithm/string.hpp>
#include <iostream>
#include <iio.h>

class PlutoSDR_i : public PlutoSDR_base
{
    ENABLE_LOGGING
    public:
        PlutoSDR_i(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl);
        PlutoSDR_i(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, char *compDev);
        PlutoSDR_i(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, CF::Properties capacities);
        PlutoSDR_i(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, CF::Properties capacities, char *compDev);
        ~PlutoSDR_i();

        void constructor();

        int serviceFunction();

    protected:
        std::string getTunerType(const std::string& allocation_id);
        bool getTunerDeviceControl(const std::string& allocation_id);
        std::string getTunerGroupId(const std::string& allocation_id);
        std::string getTunerRfFlowId(const std::string& allocation_id);
        double getTunerCenterFrequency(const std::string& allocation_id);
        void setTunerCenterFrequency(const std::string& allocation_id, double freq);
        double getTunerBandwidth(const std::string& allocation_id);
        void setTunerBandwidth(const std::string& allocation_id, double bw);
        bool getTunerAgcEnable(const std::string& allocation_id);
        void setTunerAgcEnable(const std::string& allocation_id, bool enable);
        float getTunerGain(const std::string& allocation_id);
        void setTunerGain(const std::string& allocation_id, float gain);
        long getTunerReferenceSource(const std::string& allocation_id);
        void setTunerReferenceSource(const std::string& allocation_id, long source);
        bool getTunerEnable(const std::string& allocation_id);
        void setTunerEnable(const std::string& allocation_id, bool enable);
        double getTunerOutputSampleRate(const std::string& allocation_id);
        void setTunerOutputSampleRate(const std::string& allocation_id, double sr);
        std::string get_rf_flow_id(const std::string& port_name);
        void set_rf_flow_id(const std::string& port_name, const std::string& id);
        frontend::RFInfoPkt get_rfinfo_pkt(const std::string& port_name);
        void set_rfinfo_pkt(const std::string& port_name, const frontend::RFInfoPkt& pkt);

        // Manually added functions
        std::string findPlutoSDR();
        void printDeviceChannels(struct iio_device* dev);
        void printDeviceAttributes(struct iio_device* dev);
        void printDeviceInfo(struct iio_device* dev);
        void printDevicesInfo(struct iio_context* ctx);
        int getAttributeRange(struct iio_channel* chn, const char* attr, double* min, double* max);
		void getAD9361PHYDeviceInfo(struct iio_context* ctx);
//        void getAD9361PHYDeviceInfo(struct iio_device* dev);
//        void getDevicesInfo(struct iio_context* ctx);
        int setAttribute(struct iio_channel* chn, const char* attr, double value, double* retvalue);
        struct iio_context *global_ctx;
        struct iio_channel *rx0_i, *rx0_q, *tx0_i, *tx0_q;
        int rxbuf_size, txbuf_size;
        struct iio_buffer *rxbuf, *txbuf;
        void *rxp_end, *rxp_dat, *rxt_dat, *txp_end, *txp_dat, *txt_dat;
        ptrdiff_t rxp_inc, txp_inc;
    	bool sriChanged = false;

    	bulkio::OutShortStream outputStream;
    	bulkio::InShortStream inputStream;


    private:
        ////////////////////////////////////////
        // Required device specific functions // -- to be implemented by device developer
        ////////////////////////////////////////

        // these are pure virtual, must be implemented here
        void deviceEnable(frontend_tuner_status_struct_struct &fts, size_t tuner_id);
        void deviceDisable(frontend_tuner_status_struct_struct &fts, size_t tuner_id);
        bool deviceSetTuning(const frontend::frontend_tuner_allocation_struct &request, frontend_tuner_status_struct_struct &fts, size_t tuner_id);
        bool deviceDeleteTuning(frontend_tuner_status_struct_struct &fts, size_t tuner_id);

};

#endif // PLUTOSDR_I_IMPL_H
