#include "roads/shared_client.h"

namespace roads::client {
    FRPCClient &FRPCClient::Get() {
        static FRPCClient SClient;
        return SClient;
    }
}
