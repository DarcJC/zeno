#include <format>
#include <print>

#define ZENO_LOAD_PATH

#include "roads/shared_client.h"

int main() {
    using roads::service::rpc;
    using namespace zpp::bits::literals;
//
//    auto [data, in, out] = zpp::bits::data_in_out();
//    rpc::client c { in, out};
//    c.request<"TestFunc"_sha256_int>(100).or_throw();
//    std::string converted;
//    converted.resize(data.size());
//    std::transform(std::begin(data), std::end(data), std::begin(converted), [] (std::byte b) {
//        return static_cast<char>(b);
//    });
//
//    httplib::Client Cli("http://localhost:19990");
//    auto res = Cli.Post("/v0/zeno", converted, "application/x-zeno");
//    auto c2 =  rpc::client { zpp::bits::in {res->body}, zpp::bits::out {res->body} };
//    int32_t result = c2.response<"TestFunc"_sha256_int>().or_throw();
//    std::print("result: {}", result);
    auto& Client = roads::client::FRPCClient::Get();
    auto [Result, Data] = Client.Request<"TestFunc"_sha256_int>(123);
    if (zpp::bits::success(Result)) {
        int32_t i = Client.Response<"TestFunc"_sha256_int>(Data).or_throw();
        std::print("result: {}", i);
    }

    return 0;
}
