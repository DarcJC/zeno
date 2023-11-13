
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

    std::string GetUserHomeDirectory();

    bool KillThread(uint32_t ThreadId);
}

namespace roads {
    void StartRoadServer();
}
