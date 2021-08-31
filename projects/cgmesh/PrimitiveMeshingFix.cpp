#include <zeno/zeno.h>
#include <zeno/utils/vec.h>
#include <zeno/types/NumericObject.h>
#include <zeno/types/PrimitiveObject.h>
#define MESHFIX_WITH_EIGEN
#include "meshfix/meshfix.h"
#include "EigenUtils.h"

namespace {

using namespace zeno;


struct PrimitiveMeshingFix : INode {
    virtual void apply() override {
        auto primA = get_input<PrimitiveObject>("prim");
        auto op_type = get_param<std::string>("op_type");

        auto [VA, FA] = prim_to_eigen(primA.get());
        Eigen::MatrixXd VB;
        Eigen::MatrixXi FB;

        meshfix(VA, FA, VB, FB);

        auto primFixed = std::make_shared<PrimitiveObject>();
        eigen_to_prim(VB, FB, primFixed.get());

        set_output("primFixed", std::move(primFixed));
    }
};

ZENO_DEFNODE(PrimitiveMeshingFix)({
    {
    "prim",
    },
    {
    "primFixed",
    },
    {
    },
    {"cgmesh"},
});

}
