#ifndef __ZENOEDIT_HELPER__
#define __ZENOEDIT_HELPER__

#include <zenomodel/include/igraphsmodel.h>
#include "zenoapplication.h"
#include <zenomodel/include/graphsmanagment.h>
#include <zenomodel/include/igraphsmodel.h>
#include "zenomainwindow.h"
#include <zenovis/ObjectsManager.h>
#include <zeno/types/UserData.h>
#include <zenoui/comctrl/gv/zveceditoritem.h>
#include <viewport/viewportwidget.h>
#include "launch/corelaunch.h"
#include "settings/zsettings.h"
#include "viewport/recordvideomgr.h"
#include "panel/zenospreadsheet.h"

class AppHelper
{
public:
    static QModelIndexList getSubInOutNode(IGraphsModel* pModel, const QModelIndex& subgIdx, const QString& sockName, bool bInput);
    static QLinearGradient colorString2Grad(const QString& colorStr);
    static INPUT_SOCKET getInputSocket(const QPersistentModelIndex& index, const QString& inSock, bool& exist);
    static void ensureSRCDSTlastKey(INPUT_SOCKETS& inputs, OUTPUT_SOCKETS& outputs);
    static QString nativeWindowTitle(const QString& currentFilePath);
    static void socketEditFinished(QVariant newValue, QPersistentModelIndex nodeIdx, QPersistentModelIndex paramIdx);
    static void modifyOptixObjDirectly(QVariant newValue, QPersistentModelIndex nodeIdx, QPersistentModelIndex paramIdx, bool editByPropPanel = false);
    static void modifyOptixCameraPropDirectly(QVariant newValue, QPersistentModelIndex nodeIdx, QPersistentModelIndex paramIdx);
    static void modifyLightData(QVariant newValue, QPersistentModelIndex nodeIdx, QPersistentModelIndex paramIdx);
    static VideoRecInfo getRecordInfo(const ZENO_RECORD_RUN_INITPARAM& param);
    static void initLaunchCacheParam(LAUNCH_PARAM& param);
    static bool openZsgAndRun(const ZENO_RECORD_RUN_INITPARAM& param, LAUNCH_PARAM launchParam);
    static QVector<QString> getKeyFrameProperty(const QVariant &val);
    static bool getCurveValue(QVariant & val);
    static bool updateCurve(QVariant oldVal, QVariant& val);
    static void dumpTabsToZsg(QDockWidget* dockWidget, RAPIDJSON_WRITER& writer);
};


#endif