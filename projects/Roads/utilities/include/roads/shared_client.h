
#define ZENO_LOAD_PATH
#include "roads/shared_server.h"
#undef ZENO_LOAD_PATH

#include "roads/thirdparty/httplib.h"

namespace roads::client {
    using roads::service::rpc;
    using namespace zpp::bits::literals;

    class FRPCClient {
    public:
        template <auto Id>
        constexpr auto Request(auto &&... arguments) {
            auto [data, in, out] = zpp::bits::data_in_out();
            rpc::client Client {in, out};
            auto Result = Client.request<Id>(arguments...);
            std::string Converted;
            Converted.resize(data.size());
            std::transform(std::begin(data), std::end(data), std::begin(Converted), [] (std::byte b) {
                return static_cast<char>(b);
            });

            auto Response = HttpClient.Post("/v0/zeno", Converted, "application/x-zeno");
            if (Response && Response->status == 200) {
                data.resize(Response->body.size());
                std::transform(std::begin(Response->body), std::end(Response->body), std::begin(data), [] (char c) {
                    return static_cast<std::byte>(c);
                });
                return std::make_tuple(Result, data);
            }

            zpp::bits::errc err( std::errc { 5 } );
            return std::make_tuple(err, std::vector<std::byte>{});
        }

        template <auto Id>
        constexpr auto Response(auto& InByteView) {
            auto [in, out] = zpp::bits::in_out(InByteView);
            rpc::client Client { in, out };
            return Client.template response<Id>();
        }

        static FRPCClient& Get();

    private:
        httplib::Client HttpClient { "http://localhost:19990" };
    };

}
