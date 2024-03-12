//
// Created by DarcJC on 2024/3/8.
//

#include <fstream>

#include "VAT.h"
#include "VATRigidBody.h"
#include "zeno/core/defNode.h"
#include "zeno/core/INode.h"
#include "zeno/types/ListObject.h"
#include "zeno/types/PrimitiveObject.h"
#include "zeno/utils/log.h"

struct RigidBodyVAT final : zeno::INode {
    void apply() override;
    void end();

    int use_reference_index = -1;
    zeno::vat::RigidBodyData data{};
    std::shared_ptr<zeno::ListObject> reference_meshes;
};

ZENDEFNODE(RigidBodyVAT, {
    {
        {"reference_list"},
        {"int", "use_reference_index", "0"},
        {"rotation_list"},
        {"translation_list"},
        {"int", "frame_end", "0"},
        {"path", "output_path", ""},
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

    data.transform.emplace_back();
    auto& current_trans = data.transform.back();

    auto rotation_list = get_input<zeno::ListObject>("rotation_list");
    auto translation_list = get_input<zeno::ListObject>("translation_list");

    if (rotation_list->arr.size() != translation_list->arr.size()) {
        zeno::log_error("RightBodyVAT: size of rotation_list({}) must meet size of translation_size({})", rotation_list->arr.size(), translation_list->arr.size());
        return;
    }

    const auto pieces_count = static_cast<int32_t>(rotation_list->arr.size());
    current_trans.resize(pieces_count, {});

    for (int32_t i = 0; i < pieces_count; ++i) {
        const auto & rot_object = rotation_list->arr[i];
        const auto rotation = std::dynamic_pointer_cast<zeno::NumericObject>(rot_object);
        if (nullptr != rotation) {
            auto temp_rot = rotation->get<zeno::vec4f>();
            current_trans[i].rotation = { temp_rot[0], temp_rot[1], temp_rot[2], temp_rot[3] };
        }

        const auto & tran_object = translation_list->arr[i];
        const auto translation = std::dynamic_pointer_cast<zeno::NumericObject>(tran_object);
        if (nullptr != translation) {
            auto temp_trans = translation->get<zeno::vec3f>();
            current_trans[i].translation = { temp_trans[0], temp_trans[1], temp_trans[2] };
        }
    }

    const auto data_output = std::make_shared<zeno::vat::RigidBodyDataWarpper>();
    data_output->data = data;
    set_output("data", data_output);

    if (getGlobalState()->frameid == get_input2<int>("frame_end")) {
        end();
    }
}

void RigidBodyVAT::end() {
    // 写入reference mesh
    if (!reference_meshes) {
        zeno::log_error("RigidBodyVAT: Invalid reference meshes");
        return;
    }
    for (auto & object : reference_meshes->arr) {
        zeno::vat::TriangleMesh mesh{};
        if (auto primitive = std::dynamic_pointer_cast<zeno::PrimitiveObject>(object); nullptr != primitive) {
            for (const auto & vert : primitive->verts) {
                mesh.vertices.push_back({ vert[0], vert[1], vert[2] });
            }
            for (const auto & tri : primitive->tris) {
                mesh.triangles.push_back({ static_cast<uint32_t>(tri[0]), static_cast<uint32_t>(tri[1]), static_cast<uint32_t>(tri[2]) });
            }
        }
        data.reference_mesh.push_back(mesh);
    }

    // 检查数据完整性
    int64_t rot_arr_size = -1;
    int64_t trans_arr_size = -1;
    for (int32_t i = 0; i < data.transform.size(); ++i) {
        if (rot_arr_size != -1) {
            if (rot_arr_size != data.transform[i].size()) {
                zeno::log_error("RightBodyVAT: Data Size doesn't match");
                return;
            }
        }
        if (trans_arr_size != -1) {
            if (trans_arr_size != data.transform[i].size()) {
                zeno::log_error("RightBodyVAT: Data Size doesn't match");
                return;
            }
        }

        rot_arr_size = static_cast<int64_t>(data.transform[i].size());
        trans_arr_size = static_cast<int64_t>(data.transform[i].size());
        if (rot_arr_size != trans_arr_size) [[unlikely]] {
            zeno::log_error("RightBodyVAT: Internal Error");
            return;
        }
    }

    const auto output_path = get_input<zeno::StringObject>("output_path");
    if (!output_path->value.empty()) {
        std::ofstream stream(output_path->value.c_str(), std::ios::binary);
        zeno::vat::trait::dumper<decltype(data)>::dump(stream, data);
    }
}
