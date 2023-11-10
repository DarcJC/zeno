
#include <utility>
#if __has_include(<print>)
#include <print>
#endif
#include <format>
#include <filesystem>
#include <fstream>

#include "roads/server.h"
#define ZENO_LOAD_PATH
#include "roads/shared_server.h"
#undef ZENO_LOAD_PATH

namespace roads {
    void StartRoadServer() {
        using namespace std::chrono_literals;
        // Get and check lock
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

        // Start server in new thread
        static service::EventQueue Queue;
        auto Handler = [] (std::istream& InStream) {
            Queue.Enqueue(service::ParseRequest(InStream));
        };
        static service::Server Server(19990, Handler);

        static std::vector<std::thread> LocalPool;
        size_t NumWorkers = 4;
        for (size_t i = 0; i < NumWorkers; ++i) {
            LocalPool.emplace_back([i] { service::ServerWorker(std::format("No.{}", i), Queue, 10s); } );
        }
        LocalPool.emplace_back([] { Server.RunFor(service::Clock::duration(std::numeric_limits<int64_t>::infinity())); });

        std::ofstream OutputStream { LockFilePath };
        OutputStream << LocalPool.back().get_id() << std::endl;
        OutputStream.close();
    }

    service::Session::Session(boost::asio::ip::tcp::socket &&SocketToMove, service::PostRequest Poster)
        : m_Socket(std::move(SocketToMove))
        , m_Poster(std::move(Poster))
    {}

    void service::Session::Process() {
        std::shared_ptr<Session> Self = shared_from_this();
        boost::asio::async_read(m_Socket, m_ReqBuffer, [this, Self] (boost::system::error_code ErrorCode, size_t) {
            if (!ErrorCode || ErrorCode == boost::asio::error::eof) {
                std::istream Reader(&m_ReqBuffer);
                m_Poster(Reader);
            }
        });
    }

    service::Server::Server(uint16_t Port, service::PostRequest Poster)
        : m_Port(Port)
        , m_Poster(std::move(Poster))
    {}

    void service::Server::RunFor(service::Clock::duration Duration) {
        m_Timer.expires_from_now(Duration);
        m_Timer.async_wait([this] (boost::system::error_code ErrorCode) {
            if (!ErrorCode) m_IOService.post([this] { m_Acceptor.close(); });
        });
        m_Acceptor.listen();

        DoAccept();

        m_IOService.run();
    }

    void service::Server::DoAccept() {
        m_Acceptor.async_accept(m_Socket, [this] (boost::system::error_code ErrorCode) {
            if (!ErrorCode) {
                std::make_shared<Session>(std::move(m_Socket), m_Poster)->Process();
            }
        });
    }

    service::Request service::ParseRequest(std::istream &is) {
        Request NewRequest;
        NewRequest.Data.assign(std::istream_iterator<uint8_t>(is), {});
        return NewRequest;
    }

    void service::ServerWorker(std::string Name, service::EventQueue &Queue, service::Clock::duration Duration) {
        auto const Deadline = Clock::now() + Duration;

        while (true) try {
            auto NewRequest = Queue.Dequeue(Deadline);
            std::puts(std::format("Worker {} received request\n", Name).c_str());
        } catch (std::exception const& Exception) {
            std::puts(std::format("Worker {} got {}\n", Name, Exception.what()).c_str());
        }
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

    service::EventQueue::EventQueue(size_t MaxSize)
        : m_MaxSize(MaxSize)
    {}

    void service::EventQueue::Enqueue(service::Request Req) {
        std::unique_lock<std::mutex> Lock(m_Mutex);
        m_ConditionVariable.wait(Lock, [this] { return size() < m_MaxSize; });
        push_back(std::move(Req));
        m_ConditionVariable.notify_one();
    }

    service::Request service::EventQueue::Dequeue(service::Clock::time_point Deadline) {
        Request OutRequest;

        {
            std::unique_lock<std::mutex> Lock(m_Mutex);
            m_Peak = std::max(m_Peak, size());
            if (m_ConditionVariable.wait_until(Lock, Deadline, [this] { return !empty(); } )) {
                OutRequest = std::move(front());
                pop_front();
                m_ConditionVariable.notify_one();
            } else {
                throw std::range_error("Failed to dequeue from queue within deadline.");
            }
        }

        return OutRequest;
    }

    size_t service::EventQueue::PeakDepth() const {
        std::lock_guard<std::mutex> Lock(m_Mutex);
        return m_Peak;
    }
};
