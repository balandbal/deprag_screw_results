#include "cppitasc/coordination/script_rosservice.hpp"

#include <deprag_downloader/download_request.h>


class ScriptRosServiceDownloadCurveRequest : public ScriptRosService<deprag_downloader::download_request, deprag_downloader::download_request::Request, deprag_downloader::download_request::Response>
{
public:
    ScriptRosServiceDownloadCurveRequest(const string& name)
        : ScriptRosService<deprag_downloader::download_request, deprag_downloader::download_request::Request, deprag_downloader::download_request::Response>(
            name)
    {
    }

    bool init(Dict& params) override {
        extract(params["retries"], retries_);
        extract(params["download_target"], download_target_);

        service_.request.iTarget = download_target_;
        return ScriptRosService::init(params);
    }

    bool call() override {
        pi_info_named("ScriptRosServiceDownloadCurveRequest::call", "Calling service: '{}'", serviceName_);
        bool result = false;

        for (int tries=0; tries <= retries_; tries++) {
            result = client_.call(service_.request, service_.response);

            pi_info_named("ScriptRosServiceDownloadCurveRequest::call", "Service response: '{}'", service_.response);
            pi_info_named("ScriptRosServiceDownloadCurveRequest::call", "Service result: '{}'", result);
            
            if (result) {
                break;
            }
        };

        return result;
    }

private:
    int download_target_{1};
    int retries_{0};
};

RUNTIME_COMPONENT(ScriptRosServiceDownloadCurveRequest)