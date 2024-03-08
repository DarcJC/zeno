//
// Created by DarcJC on 2024/3/8.
//

#include <fstream>

#include "VATRigidBody.h"
#include "zeno/core/defNode.h"
#include "zeno/core/INode.h"
#include "zeno/types/ListObject.h"
#include "zeno/types/PrimitiveObject.h"
#include "zeno/utils/log.h"

struct RigidBodyVAT final : zeno::INode {
    void apply() override;
    void complete() override;

    int use_reference_index = -1;
    zeno::vat::RigidBodyData data{};
    std::shared_ptr<zeno::ListObject> reference_meshes;
};

ZENDEFNODE(RigidBodyVAT, {
    {
        {"reference_list"},
        {"int", "use_reference_index", "0"},
        {"transform_list"},
        {"translation_list"},
    },
    {
        "data",
    },
    {},
    {"VAT"}
});

void RigidBodyVAT::apply() {
    if (-1 == use_reference_index) {
        use_reference_index = get_input2<int>("use_reference_index");
    }
    if (const int frame_id = getGlobalState()->frameid; frame_id == use_reference_index) {
        reference_meshes = get_input<zeno::ListObject>("reference_list");
    }

    std::ofstream stream("D:/test.zrigid", std::ios::binary);
    zeno::vat::trait::simple_dummper<decltype(data)>::dump(stream, data);

    set_output("data", std::make_shared<zeno::NumericObject>(use_reference_index));
}

void RigidBodyVAT::complete() {
}
