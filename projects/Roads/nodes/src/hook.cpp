
#define DISABLE_ZPP_BIT_LIBRARY
#include "roads/server.h"
#undef DISABLE_ZPP_BIT_LIBRARY
#include <zeno/extra/EventCallbacks.h>
#include <zeno/zeno.h>
#include <zeno/utils/logger.h>

[[maybe_unused]]
int _DefRunServer = zeno::getSession().eventCallbacks->hookEvent("editorConstructed", &roads::StartRoadServer);
