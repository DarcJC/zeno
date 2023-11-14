
#include <utility>
#if __has_include(<print>)
#include <print>
#endif
#include <format>
#include <filesystem>
#include <iostream>
#include <fstream>
#include "roads/thirdparty/httplib.h"

#include "roads/server.h"
#define ZENO_LOAD_PATH
#include "roads/shared_server.h"
#undef ZENO_LOAD_PATH

namespace roads {

    void GetOutOfMyHouse() {
        std::string ZenoHome = std::format("{}/.zeno", service::GetUserHomeDirectory());
        std::filesystem::create_directories(ZenoHome);
        std::string LockFilePath = std::format("{}/.road_lock", ZenoHome);
        if (std::filesystem::exists(LockFilePath)) {
            std::ifstream InputStream { LockFilePath };
            if (InputStream) {
                uint32_t WorkingThreadId;
                InputStream >> WorkingThreadId;
                service::KillThread(WorkingThreadId);
            }
            InputStream.close();
            std::filesystem::remove(LockFilePath);
        }
    }

    void StartRoadServer() {
        using namespace std::chrono_literals;
        // Get and check lock
        GetOutOfMyHouse();

        // Start server in new thread
        static std::optional<std::thread> ServerThread;
        ServerThread = std::thread([] () {
            httplib::Server Server;

            Server.Post("/v0/zeno", [] (const httplib::Request& Request, httplib::Response& Response) {
                Response.status = 404;

                using namespace roads::service;
                std::string RPCInputBuf = Request.body;
                zpp::bits::in RPCInput {RPCInputBuf};
                zpp::bits::out RPCOutput { Response.body };
                rpc::server RPCServer { RPCInput, RPCOutput, FRemoteSubsystem::Get() };
                auto ResultServe = RPCServer.serve();
                if (zpp::bits::success(ResultServe)) {
                    Response.status = 200;
                    Response.set_header("Content-Type", "application/x-zeno");
                }
            });

            Server.listen("0.0.0.0", 19990);
            return 0;
        });
    }

    std::string service::GetUserHomeDirectory() {
#if WIN32
        if (const char* USERPROFILE = std::getenv("USERPROFILE")) {
            return { USERPROFILE };
        } else if (const char* HOMEDRIVE = std::getenv("HOMEDRIVE")) {
            if (const char* HOMEPATH = std::getenv("HOMEPATH")) {
                return std::format("{}\\{}", HOMEDRIVE, HOMEPATH);
            }
        }
#else
        if (const char* HOME = std::getenv("HOME")) {
            return { HOME };
        }
#endif
        throw std::runtime_error("Could not found user home.");
    }

    bool service::KillThread(uint32_t ThreadId) {
#if WIN32
#include "processthreadsapi.h"
        HANDLE Handle = OpenThread(THREAD_TERMINATE, false, ThreadId);
        if (Handle) return TerminateProcess(Handle, 10);
        return true;
#else
        throw std::runtime_error("Kill Thread doesn't implemented on this platform.");
#endif
    }

};

namespace roads::service {
    int32_t FRemoteSubsystem::TestFunc(int32_t i) {
        return IState += (i + 1);
    }

    FRemoteSubsystem &FRemoteSubsystem::Get() {
        static FRemoteSubsystem SRemoteSubsystem;
        return SRemoteSubsystem;
    }
}
