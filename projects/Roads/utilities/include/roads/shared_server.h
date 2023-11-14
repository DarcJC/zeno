#pragma once

#pragma warning(disable:4146)

#include <type_traits>
#include <vector>
#include <map>

#if defined(ZENO_LOAD_PATH)
#include "roads/thirdparty/zpp_bits.h"
#define UENUM(...)
#define UPROPERTY(...)
#define USTRUCT(...)
#define UCLASS(...)
#define GENERATED_USTRUCT_BODY()
#define GENERATED_BODY()
#define FORCEINLINE inline
using int32 = std::int32_t;
#else
#include "zpp_bits.h"
#include "ZenoSharedServer.generated.h"
#endif

#if !defined(ZENO_LOAD_PATH)
template <typename ValueType>
void StdToUnrealArray(const std::vector<ValueType>& StdVector, TArray<ValueType>& UEVector) {
    UEVector.Empty();
    UEVector.AddUninitialized(StdVector.size());
    FMemory::Memcpy(UEVector.GetData(), StdVector.data(), UEVector.Num());
}

template <typename ValueType>
void UnrealToStdArray(const TArray<ValueType>& UEVector, std::vector<ValueType>& StdVector) {
    StdVector.resize(UEVector.Num());
    FMemory::Memcpy(StdVector.data(), UEVector.GetData(), StdVector.size());
}
#endif

UENUM()
enum class EZenoAssetType {
    Invalid = 0,
    Terrain = 1,
    Max,
};

template <typename T>
concept TZenoAssetData = requires(T t) {
    { t.AssetType } -> std::same_as<EZenoAssetType>;
};

#ifdef ZENO_LOAD_PATH
struct FArchive {
    template <typename T>
    FArchive& operator<<(T& Value) {}

    bool IsLoading();
    bool IsSaving();
};
#endif

template <typename T>
FORCEINLINE FArchive& operator<<(FArchive& Ar, std::vector<T>& Self)
{
    int32 Size = Self.size();
    Ar << Size;
    if (Ar.IsLoading())
    {
        Self.resize(Size);
    }
    for (int32 i = 0; i < Self.size(); ++i)
    {
        Ar << Self.at(i);
    }
    return Ar;
}

USTRUCT()
struct FStdFloatVector {
    GENERATED_BODY()

    std::vector<float> InternalData;

    FORCEINLINE friend FArchive& operator<<(FArchive& Ar, FStdFloatVector& Self)
    {
        return Ar << Self.InternalData;
    }
};

USTRUCT()
struct FZenoVector {
    GENERATED_BODY();

    float X;
    float Y;
    float Z;
};

USTRUCT()
struct FZenoTransformInfo {
    GENERATED_BODY()

    UPROPERTY()
    FZenoVector Location;
    UPROPERTY()
    FZenoVector Rotation;
    UPROPERTY()
    FZenoVector Scale;
};

USTRUCT()
struct FZenoTerrainData {
    GENERATED_BODY()

    UPROPERTY()
    int32 Row;

    UPROPERTY()
    int32 Column;

    UPROPERTY()
    FStdFloatVector Data;

    UPROPERTY()
    FZenoTransformInfo TransformInfo;

    UPROPERTY()
    EZenoAssetType AssetType = EZenoAssetType::Terrain;

    inline bool IsValid() const {
        return Row * Column < Data.InternalData.size();
    }
};

namespace roads::service {
    using namespace zpp::bits::literals;

    struct FRemoteSubsystem {
    public:
        static FRemoteSubsystem& Get();

        FZenoTerrainData TestFunc(int32_t i);

    private:
        int32_t IState = 123;
    };

    using rpc = zpp::bits::rpc<
            zpp::bits::bind<&FRemoteSubsystem::TestFunc, "TestFunc"_sha256_int>
    >;
}
