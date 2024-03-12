//
// Created by DarcJC on 2024/3/12.
//
#pragma once
#include "VATRigidBody.h"
#include "zeno/core/IObject.h"

namespace zeno::vat {

    struct RigidBodyDataWarpper final : public IObject {
        RigidBodyData data;
    };

}
