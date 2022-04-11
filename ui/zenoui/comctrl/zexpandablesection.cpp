#include "zexpandablesection.h"
#include "zlabel.h"
#include "../style/zenostyle.h"


ZExpandableSection::ZExpandableSection(const QString& title, QWidget* parent)
	: QWidget(parent)
	, m_mainLayout(nullptr)
	, m_contentArea(nullptr)
	, m_contentWidget(nullptr)
{
	m_contentArea = new QScrollArea(this);
	m_mainLayout = new QGridLayout(this);

	QLabel* plblTitle = new QLabel(title);
	plblTitle->setProperty("cssClass", "proppanel-sectionname");

	m_collaspBtn = new ZIconLabel;
	m_collaspBtn->setIcons(ZenoStyle::dpiScaledSize(QSize(24, 24)), ":/icons/ic_parameter_fold.svg", "", ":/icons/ic_parameter_unfold.svg");

	m_contentArea->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
	m_contentArea->setMinimumHeight(0);
	m_contentArea->setProperty("cssClass", "proppanel");
	m_contentArea->setFrameShape(QFrame::NoFrame);

	m_mainLayout->setVerticalSpacing(0);
	m_mainLayout->setContentsMargins(15, 15, 15, 15);

	int row = 0;
	m_mainLayout->addWidget(m_collaspBtn, 0, 0);
	m_mainLayout->addWidget(plblTitle, 0, 1);
	m_mainLayout->addWidget(m_contentArea, 1, 1);

	setLayout(m_mainLayout);

	connect(m_collaspBtn, &ZIconLabel::toggled, this, &ZExpandableSection::toggle);
}

void ZExpandableSection::setContentLayout(QLayout* contentLayout)
{
    QWidget* contentWidget = new QWidget;
    contentWidget->setLayout(contentLayout);
    contentWidget->setAutoFillBackground(true);
    QPalette pal = this->palette();
    pal.setColor(QPalette::Window, QColor(42, 42, 42));
    contentWidget->setPalette(pal);

    m_contentArea->setWidget(contentWidget);
	update();
}

void ZExpandableSection::toggle(bool collasped)
{
    m_contentArea->setVisible(!m_contentArea->isVisible());
    update();
}