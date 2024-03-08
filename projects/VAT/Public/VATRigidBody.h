//
// Created by DarcJC on 2024/3/8.
//
#pragma once
#include <type_traits>
#include <vector>
#include <array>
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
        TriangleMesh reference_mesh;
        std::vector<std::vector<RigidBodyTransform>> transform;
    };

    namespace trait {
        template <typename T>
        struct simple_dummper {
            static_assert(std::is_arithmetic_v<T> || std::is_standard_layout_v<T>);

            using Type = T;
            static constexpr uint32_t ValueSize = sizeof(T);

            static void dump(std::ostream& out_stream, const Type& value) {
                out_stream.write(&value, ValueSize);
            }

            static Type read(std::istream& in_stream) {
                Type temp;

                if (!in_stream.read(&temp, ValueSize)) {
                    throw std::runtime_error("simple_dummper: failed to read a number");
                }

                return temp;
            }
        };

        template <typename T>
        struct is_vector {
            using type = T;
            static constexpr bool value = false;
        };

        template <typename T>
        struct is_vector<std::vector<T>> {
            using type = T;
            static constexpr bool value = true;
        };

        template <typename ValueType>
        struct simple_dummper<std::vector<ValueType>> {
            static_assert((std::is_trivial_v<ValueType> && std::is_standard_layout_v<ValueType>) || (is_vector<ValueType>::value && std::is_standard_layout_v<typename is_vector<ValueType>::type>), "simple dummper only support pod value types in STL container");

            using Type = std::vector<ValueType>;
            static constexpr int32_t PerValueSize = sizeof(ValueType);

            static void dump(std::ostream& out_stream, const Type& in_vector) {
                // 写出元数据
                const auto size = static_cast<uint32_t>(in_vector.size());
                out_stream.write(reinterpret_cast<const char*>(&size), sizeof(size));

                // 写出数据
                for (const ValueType& value : in_vector) {
                    out_stream.write(reinterpret_cast<const char*>(&value), PerValueSize);
                }
            }

            static Type read(std::istream& in_stream) {
                Type result;

                // 读入元素数量的元数据
                uint32_t size;
                in_stream.read(reinterpret_cast<char*>(&size), sizeof(size));

                // 读入指定数量的元数据
                for (uint32_t i = 0; i < size; ++i) {
                    ValueType value{};
                    if (in_stream.read(reinterpret_cast<char*>(&value), PerValueSize)) {
                        result.push_back(value);
                    } else {
                        // 如果在期望读取的元素数量之前就达到了流的末尾，或发生了读取错误，跳出循环
                        throw std::runtime_error("simple_dumpper: read file error");
                    }
                }

                return result;
            }
        };

        template <>
        struct simple_dummper<TriangleMesh> {
            using Type = TriangleMesh;

            static void dump(std::ostream& out_stream, const Type& in_mesh) {
                simple_dummper<decltype(in_mesh.vertices)>::dump(out_stream, in_mesh.vertices);
                simple_dummper<decltype(in_mesh.triangles)>::dump(out_stream, in_mesh.triangles);
            }

            static Type read(std::istream& in_stream) {
                Type result{};
                result.vertices = simple_dummper<decltype(result.vertices)>::read(in_stream);
                result.triangles = simple_dummper<decltype(result.triangles)>::read(in_stream);
                return result;
            }
        };

        template <>
        struct simple_dummper<RigidBodyData> {
            using Type = RigidBodyData;

            static void dump(std::ostream& out_stream, const Type& in_mesh) {
                simple_dummper<decltype(in_mesh.reference_mesh)>::dump(out_stream, in_mesh.reference_mesh);
                simple_dummper<decltype(in_mesh.transform)>::dump(out_stream, in_mesh.transform);
            }

            static Type read(std::istream& in_stream) {
                Type result{};
                simple_dummper<decltype(result.reference_mesh)>::read(in_stream);
                simple_dummper<decltype(result.transform)>::read(in_stream);
                return result;
            }
        };
    }

}
