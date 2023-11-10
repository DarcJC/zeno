
#pragma once
#include <functional>
#include <memory>
#include <chrono>
#include <queue>
#include "boost/asio.hpp"
#include "boost/asio/ip/tcp.hpp"

#if not defined(DISABLE_ZPP_BIT_LIBRARY)
#include "roads/thirdparty/zpp_bits.h"
#endif

namespace roads::service {
    using PostRequest = std::function<void(std::istream&)>;
    using boost::asio::ip::tcp;
    using Clock = std::chrono::high_resolution_clock;
    using namespace std::chrono_literals;

    struct Request {
        std::vector<uint8_t> Data;
    };

    Request ParseRequest(std::istream& is);

    /**
     * Also named Connection.
     * <br/>
     * It held an socket and could perform async read on the socket.
     * Data read from the socket will store in m_ReqBuffer and pass to the Poster function.
     */
    struct Session : std::enable_shared_from_this<Session> {
        Session(tcp::socket&& SocketToMove, PostRequest Poster);

        void Process();

    private:
        tcp::socket m_Socket;
        boost::asio::streambuf m_ReqBuffer;
        PostRequest m_Poster;
    };

    class Server {
    public:
        Server(uint16_t Port, PostRequest Poster);

        void RunFor(Clock::duration Duration = 30s);

    private:
        void DoAccept();

        uint16_t m_Port;
        PostRequest m_Poster;

        boost::asio::io_service m_IOService;
        boost::asio::high_resolution_timer m_Timer { m_IOService };
        tcp::acceptor m_Acceptor { m_IOService, tcp::endpoint {{}, m_Port} };
        tcp::socket m_Socket { m_IOService };
    };

    class EventQueue : protected std::deque<Request> {
    public:
        explicit EventQueue(size_t MaxSize = 50);

        void Enqueue(Request Req);

        Request Dequeue(Clock::time_point Deadline);

        size_t PeakDepth() const;

    private:
        mutable std::mutex m_Mutex;
        mutable std::condition_variable m_ConditionVariable;

        size_t m_MaxSize = 50;
        size_t m_Peak = 0;
    };

    void ServerWorker(std::string Name, service::EventQueue& Queue, service::Clock::duration Duration = 30s);

    std::string GetUserHomeDirectory();

    bool KillThread(uint32_t ThreadId);
}

namespace roads {
    void StartRoadServer();
}
