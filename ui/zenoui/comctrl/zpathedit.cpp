#include "zpathedit.h"
#include "zlineedit.h"
#include <zenomodel/include/modeldata.h>
#include <zenoedit/zenoapplication.h>
#include <zenoedit/zenomainwindow.h>



ZPathEdit::ZPathEdit(QWidget *parent)
    : ZLineEdit(parent)
{
    initUI();
}

ZPathEdit::ZPathEdit(const QString &text, QWidget *parent)
    : ZLineEdit(text, parent)
{
    initUI();
}

void ZPathEdit::initUI()
{
    setFocusPolicy(Qt::ClickFocus);
    setIcons(":/icons/file-loader.svg", ":/icons/file-loader-on.svg");
    setProperty("cssClass", "path_edit");

    CALLBACK_SWITCH cbSwitch = [=](bool bOn) {
        zenoApp->getMainWindow()->setInDlgEventLoop(bOn); //deal with ubuntu dialog slow problem when update viewport.
    };
    QObject::connect(this, &ZLineEdit::btnClicked, [=]() {
        int ctrl = this->property("control").toInt();
        QString path;
        cbSwitch(true);
        if (ctrl == CONTROL_READPATH) {
            path = QFileDialog::getOpenFileName(nullptr, "File to Open", "", "All Files(*);;");
        } else if (ctrl == CONTROL_WRITEPATH) {
            path = QFileDialog::getSaveFileName(nullptr, "Path to Save", "", "All Files(*);;");
        } else {
            path = QFileDialog::getExistingDirectory(nullptr, "Path to Save", "");
        }
        if (path.isEmpty()) {
            cbSwitch(false);
            return;
        }
        setText(path);
        emit textEditFinished();
        clearFocus();
        cbSwitch(false);
    });
}
