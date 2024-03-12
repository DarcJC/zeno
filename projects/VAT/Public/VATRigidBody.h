//
// Created by DarcJC on 2024/3/8.
//
#pragma once
#include <iostream>
#include <type_traits>
#include <vector>
#include <ostream>
#include <istream>

namespace zeno::vat {

    template <typename T>
    struct array3 {
        T x;
        T y;
        T z;
    };

    template <typename T>
    struct array4 {
        T x;
        T y;
        T z;
        T w;
    };

    struct TriangleMesh {
        std::vector<array3<float>> vertices;
        std::vector<array3<uint32_t>> triangles;
    };

    struct RigidBodyTransform {
        array3<float> translation;
        array4<float> rotation;
    };

    struct RigidBodyData {
        std::vector<TriangleMesh> reference_mesh;
        std::vector<std::vector<RigidBodyTransform>> transform;
    };

    namespace trait {

        template <typename T>
        struct is_vector {
            using type = T;
            static constexpr bool value = false;
        };

        template <typename T>
        struct is_vector<std::vector<T>> {
            using type = std::vector<T>;
            using value_type = T;
            static constexpr bool value = true;
        };

        template <typename T>
        struct dumper {
            static_assert(std::is_copy_constructible_v<T>);
            using type = T;

            static void dump(std::ostream& stream, const type& value) {
                stream.write(reinterpret_cast<const char*>(&value), sizeof(type));
            }

            static type read(std::istream& stream) {
                char buf[sizeof(type)];
                if (!stream.read(buf, sizeof(type))) {
                    throw std::runtime_error("read: ");
                }
                return type(*(reinterpret_cast<type*>(buf))); // copy construct
            }
        };

        template <typename ValueType>
        struct dumper<std::vector<ValueType>> {
            using type = std::vector<ValueType>;
            using value_type = ValueType;

            static void dump(std::ostream& stream, const type& value) {
                const uint64_t vec_size = value.size();
                stream.write(reinterpret_cast<const char*>(&vec_size), sizeof(vec_size));

                if constexpr (is_vector<value_type>::value) {
                    for (const auto & item : value) {
                        dumper<typename is_vector<value_type>::type>::dump(stream, item);
                    }
                } else {
                    for (const auto & item : value) {
                        dumper<decltype(item)>::dump(stream, item);
                    }
                }
            }

            static type read(std::istream& stream) {
                uint64_t vec_size = 0;
                if (!stream.read(reinterpret_cast<char*>(&vec_size), sizeof(vec_size))) {
                    throw std::runtime_error("read: vec size");
                }

                type result{};
                for (uint64_t i = 0; i < vec_size; ++i) {
                    if constexpr (is_vector<value_type>::value) {
                        auto val = dumper<typename is_vector<value_type>::type>::read(stream);
                        result.push_back(std::move(val));
                    } else {
                        value_type val = dumper<value_type>::read(stream);
                        result.push_back(std::move(val));
                    }
                }

                return result;
            }
        };

        template <>
        struct dumper<TriangleMesh> {
            using type = TriangleMesh;

            static void dump(std::ostream& stream, const type& value) {
                dumper<decltype(value.vertices)>::dump(stream, value.vertices);
                dumper<decltype(value.triangles)>::dump(stream, value.triangles);
            }

            static type read(std::istream& stream) {
                type result{};
                result.vertices = dumper<decltype(type::vertices)>::read(stream);
                result.triangles = dumper<decltype(type::triangles)>::read(stream);
                return result;
            }
        };

        template <>
        struct dumper<RigidBodyTransform> {
            using type = RigidBodyTransform;

            static void dump(std::ostream& stream, type& value) {
                dumper<decltype(value.rotation)>::dump(stream, value.rotation);
                dumper<decltype(value.translation)>::dump(stream, value.translation);
            }

            static type read(std::istream& stream) {
                type result{};
                result.rotation = dumper<decltype(result.rotation)>::read(stream);
                result.translation = dumper<decltype(result.translation)>::read(stream);
                return result;
            }
        };

        template <>
        struct dumper<RigidBodyData> {
            using type = RigidBodyData;

            static void dump(std::ostream& stream, const type& value) {
                dumper<decltype(value.reference_mesh)>::dump(stream, value.reference_mesh);
                dumper<decltype(value.transform)>::dump(stream, value.transform);
            }

            static type read(std::istream& stream) {
                type result{};
                result.reference_mesh = dumper<decltype(result.reference_mesh)>::read(stream);
                result.transform = dumper<decltype(result.transform)>::read(stream);
                return result;
            }
        };
    }

}
