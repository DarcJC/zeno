
set(VAT_SOURCE_FILES
        Private/RigidBody.cpp
        Private/RigidBodyNode.cpp
)
set(VAT_HEADER_FILES
        Public/VATRigidBody.h
)

target_sources(zeno PRIVATE ${VAT_SOURCE_FILES} ${VAT_HEADER_FILES})
target_include_directories(zeno PRIVATE Public)
