/*
 *  DialogPreferences.cpp
 *  GaitSym2017
 *
 *  Created by Bill Sellers on 12/02/2017.
 *  Copyright 2017 Bill Sellers. All rights reserved.
 *
 */

#include <QColorDialog>
#include <QLabel>
#include <QGridLayout>
#include <QVBoxLayout>
#include <QDialogButtonBox>
#include <QLineEdit>
#include <QCheckBox>
#include <QPushButton>
#include <QMultiMap>
#include <QScrollArea>
#include <QFileDialog>
#include <QMenu>
#include <QAction>

#include "Preferences.h"

#include "DialogPreferences.h"

DialogPreferences::DialogPreferences(QWidget *parent)
    : QDialog(parent)
{
    setWindowTitle(tr("Preferences Dialog"));
#ifdef Q_OS_MACOS
    // on MacOS High Sierra (and possibly others), the Qt::Dialog flag disables resizing
    // if I clear the Qt::Dialog flag and make sure the Qt::Window flag is set (it should be already)
    // then it seems to let me have resizing without any other side effects
    setWindowFlags(windowFlags() & (~Qt::Dialog) | Qt::Window);
#endif
}

DialogPreferences::~DialogPreferences()
{
}

void DialogPreferences::setPreferences(const Preferences &preferences)
{
    const QString COLOUR_STYLE("QPushButton { background-color : %1; color : %2; border: 4px solid %3; }");
    m_Preferences = preferences;

    QVBoxLayout *verticalLayout;
    QScrollArea *scrollArea;
    QWidget *scrollAreaWidgetContents;
    QGridLayout *gridLayout;
    QDialogButtonBox *buttonBox;

    this->resize(800, 800);
    verticalLayout = new QVBoxLayout(this);
    verticalLayout->setSpacing(6);
    verticalLayout->setContentsMargins(11, 11, 11, 11);
    verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
    scrollArea = new QScrollArea(this);
    scrollArea->setObjectName(QStringLiteral("scrollArea"));
    scrollArea->setWidgetResizable(true);
    scrollAreaWidgetContents = new QWidget();
    scrollAreaWidgetContents->setObjectName(QStringLiteral("scrollAreaWidgetContents"));
    // scrollAreaWidgetContents->setGeometry(QRect(0, 0, 245, 154));
    gridLayout = new QGridLayout(scrollAreaWidgetContents);
    gridLayout->setSpacing(6);
    gridLayout->setContentsMargins(11, 11, 11, 11);
    gridLayout->setObjectName(QStringLiteral("gridLayout"));

    QStringList keys = m_Preferences.keys();
    keys.sort(Qt::CaseInsensitive);
    QMultiMap<int, SettingsItem> sortedItems;
    for (int i = 0; i < keys.size(); i++)
    {
        SettingsItem item = m_Preferences.settingsItem(keys[i]);
        if (item.display) sortedItems.insert(item.order, item);
    }

    int row = 0;
    SettingsWidget settingsWidget;
    for (QMultiMap<int, SettingsItem>::const_iterator i = sortedItems.constBegin(); i != sortedItems.constEnd(); i++)
    {
        SettingsItem item = i.value();

        QLabel *label = new QLabel();
        label->setText(item.label);
        gridLayout->addWidget(label, row, 0);

        settingsWidget.item = item;
        QLineEdit *lineEdit;
        if (item.value.type() == QMetaType::Int)
        {
            lineEdit = new QLineEdit();
            lineEdit->setMinimumWidth(200);
            lineEdit->setText(QString("%1").arg(item.value.toInt()));
            gridLayout->addWidget(lineEdit, row, 1);
            settingsWidget.widget = lineEdit;
            m_SettingsWidgetList.append(settingsWidget);
        }

        if (item.value.type() == QMetaType::Double)
        {
            lineEdit = new QLineEdit();
            lineEdit->setMinimumWidth(200);
            lineEdit->setText(QString("%1").arg(item.value.toDouble()));
            gridLayout->addWidget(lineEdit, row, 1);
            settingsWidget.widget = lineEdit;
            m_SettingsWidgetList.append(settingsWidget);
        }

        if (item.value.type() == QMetaType::Float)
        {
            lineEdit = new QLineEdit();
            lineEdit->setMinimumWidth(200);
            lineEdit->setText(QString("%1").arg(item.value.toFloat()));
            gridLayout->addWidget(lineEdit, row, 1);
            settingsWidget.widget = lineEdit;
            m_SettingsWidgetList.append(settingsWidget);
        }

        if (item.value.type() == QMetaType::QString)
        {
            lineEdit = new QLineEdit();
            lineEdit->setMinimumWidth(200);
            lineEdit->setText(QString("%1").arg(item.value.toString()));
            gridLayout->addWidget(lineEdit, row, 1);
            settingsWidget.widget = lineEdit;
            m_SettingsWidgetList.append(settingsWidget);
            if (item.path)
            {
                lineEdit->setContextMenuPolicy(Qt::CustomContextMenu);
                QObject::connect(lineEdit, SIGNAL(customContextMenuRequested(QPoint)), this, SLOT(menuRequestPath(QPoint)));
            }
        }

        if (item.value.type() == QMetaType::Bool)
        {
            QCheckBox *checkBox = new QCheckBox();
            checkBox->setChecked(item.value.toBool());
            gridLayout->addWidget(checkBox, row, 1);
            settingsWidget.widget = checkBox;
            m_SettingsWidgetList.append(settingsWidget);
        }

        if (item.value.type() == QMetaType::QColor)
        {
            QPushButton *pushButton = new QPushButton();
            pushButton->setText("Colour");
            QColor color = qvariant_cast<QColor>(item.value);
            pushButton->setStyleSheet(COLOUR_STYLE.arg(color.name()).arg(getIdealTextColour(color).name()).arg(getAlphaColourHint(color).name()));
            connect(pushButton, SIGNAL(clicked()), this, SLOT(colourButtonClicked()));
            gridLayout->addWidget(pushButton, row, 1);
            settingsWidget.widget = pushButton;
            m_SettingsWidgetList.append(settingsWidget);
        }

        row++;

    }

    scrollArea->setWidget(scrollAreaWidgetContents);
    verticalLayout->addWidget(scrollArea);

    buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    buttonBox->setObjectName(QStringLiteral("buttonBox"));
    QPushButton *importButton = buttonBox->addButton(tr("&Import..."), QDialogButtonBox::ActionRole);
    QPushButton *exportButton = buttonBox->addButton(tr("&Export..."), QDialogButtonBox::ActionRole);
    QPushButton *defaultsButton = buttonBox->addButton(tr("&Defaults"), QDialogButtonBox::ActionRole);
    verticalLayout->addWidget(buttonBox);

    connect(buttonBox, SIGNAL(accepted()), this, SLOT(acceptButtonClicked()));
    connect(buttonBox, SIGNAL(rejected()), this, SLOT(rejectButtonClicked()));
    connect(importButton, SIGNAL(clicked()), this, SLOT(importButtonClicked()));
    connect(exportButton, SIGNAL(clicked()), this, SLOT(exportButtonClicked()));
    connect(defaultsButton, SIGNAL(clicked()), this, SLOT(defaultsButtonClicked()));

    restoreGeometry(m_Preferences.valueQByteArray("DialogPreferencesGeometry"));

}

Preferences DialogPreferences::preferences()
{
    for (int i = 0; i < m_SettingsWidgetList.size(); i++)
    {
        SettingsItem item = m_SettingsWidgetList[i].item;
        QWidget *widget = m_SettingsWidgetList[i].widget;

        if (item.value.type() == QMetaType::Int) item.value = dynamic_cast<QLineEdit *>(widget)->text().toInt();
        if (item.value.type() == QMetaType::Double) item.value = dynamic_cast<QLineEdit *>(widget)->text().toDouble();
        if (item.value.type() == QMetaType::Float) item.value = dynamic_cast<QLineEdit *>(widget)->text().toFloat();
        if (item.value.type() == QMetaType::QString) item.value = dynamic_cast<QLineEdit *>(widget)->text();
        if (item.value.type() == QMetaType::Bool) item.value = dynamic_cast<QCheckBox *>(widget)->isChecked();

        m_Preferences.insert(item);
    }
    return m_Preferences;
}

void DialogPreferences::colourButtonClicked()
{
    const QString COLOUR_STYLE("QPushButton { background-color : %1; color : %2; border: 4px solid %3; }");
    int i;
    QPushButton *pushButton = dynamic_cast<QPushButton *>(QObject::sender());
    for (i = 0; i < m_SettingsWidgetList.size(); i++)
        if (m_SettingsWidgetList[i].widget == pushButton) break;
    if (i >= m_SettingsWidgetList.size()) return;

    QColor colour = QColorDialog::getColor(qvariant_cast<QColor>(m_SettingsWidgetList[i].item.value), this, "Select Color", QColorDialog::ShowAlphaChannel /*| QColorDialog::DontUseNativeDialog*/);
    if (colour.isValid())
    {
        pushButton->setStyleSheet(COLOUR_STYLE.arg(colour.name()).arg(getIdealTextColour(colour).name()).arg(getAlphaColourHint(colour).name()));
        m_SettingsWidgetList[i].item.value = colour;
    }
}


// return an ideal label colour, based on the given background colour.
// Based on http://www.codeproject.com/cs/media/IdealTextColor.asp
QColor DialogPreferences::getIdealTextColour(const QColor& rBackgroundColour)
{
    const int THRESHOLD = 105;
    int BackgroundDelta = (rBackgroundColour.red() * 0.299) + (rBackgroundColour.green() * 0.587) + (rBackgroundColour.blue() * 0.114);
    return QColor((255- BackgroundDelta < THRESHOLD) ? Qt::black : Qt::white);
}

QColor DialogPreferences::getAlphaColourHint(const QColor& colour)
{
    // (source * Blend.SourceAlpha) + (background * Blend.InvSourceAlpha)
    QColor background;
    background.setRgbF(1.0, 1.0, 1.0);
    QColor hint;
    hint.setRedF((colour.redF() * colour.alphaF()) + (background.redF() * (1 - colour.alphaF())));
    hint.setGreenF((colour.greenF() * colour.alphaF()) + (background.greenF() * (1 - colour.alphaF())));
    hint.setBlueF((colour.blueF() * colour.alphaF()) + (background.blueF() * (1 - colour.alphaF())));
    return hint;
}

void DialogPreferences::importButtonClicked()
{
    QString lastImportedFile = m_Preferences.valueQString(tr("LastImportedFile"));
    QString fileName = QFileDialog::getOpenFileName(this, tr("Import Settings File"), lastImportedFile, tr("Exported Settings Files (*.xml)"), 0);
    if (fileName.isNull() == false)
    {
        m_Preferences.Import(fileName);
        m_Preferences.insert("LastImportedFile", fileName);
        for (int i = 0; i < m_SettingsWidgetList.size(); i++)
        {
            SettingsItem item = m_SettingsWidgetList[i].item;
            QWidget *widget = m_SettingsWidgetList[i].widget;
            if (item.value.type() == QMetaType::Int) dynamic_cast<QLineEdit *>(widget)->setText(m_Preferences.valueQString(item.key));
            if (item.value.type() == QMetaType::Double) dynamic_cast<QLineEdit *>(widget)->setText(m_Preferences.valueQString(item.key));
            if (item.value.type() == QMetaType::QString) dynamic_cast<QLineEdit *>(widget)->setText(m_Preferences.valueQString(item.key));
            if (item.value.type() == QMetaType::Bool) dynamic_cast<QCheckBox *>(widget)->setChecked(m_Preferences.valueBool(item.key));
        }
    }
}

void DialogPreferences::exportButtonClicked()
{
    QString lastExportedFile = m_Preferences.valueQString(tr("LastExportedFile"));
    QString fileName = QFileDialog::getSaveFileName(this, tr("Export Settings File"), lastExportedFile, tr("Exported Settings Files (*.xml)"), 0);
    if (fileName.isNull() == false)
    {
        m_Preferences.insert("LastExportedFile", fileName);
        m_Preferences.Export(fileName);
    }
}

void DialogPreferences::defaultsButtonClicked()
{
    m_Preferences.LoadDefaults();
    for (int i = 0; i < m_SettingsWidgetList.size(); i++)
    {
        SettingsItem item = m_SettingsWidgetList[i].item;
        QWidget *widget = m_SettingsWidgetList[i].widget;
        if (item.value.type() == QMetaType::Int) dynamic_cast<QLineEdit *>(widget)->setText(m_Preferences.valueQString(item.key));
        if (item.value.type() == QMetaType::Double) dynamic_cast<QLineEdit *>(widget)->setText(m_Preferences.valueQString(item.key));
        if (item.value.type() == QMetaType::QString) dynamic_cast<QLineEdit *>(widget)->setText(m_Preferences.valueQString(item.key));
        if (item.value.type() == QMetaType::Bool) dynamic_cast<QCheckBox *>(widget)->setChecked(m_Preferences.valueBool(item.key));
    }
}

void DialogPreferences::acceptButtonClicked()
{
    m_Preferences.insert("DialogPreferencesGeometry", saveGeometry());
    accept();
}

void DialogPreferences::rejectButtonClicked()
{
    m_Preferences.insert("DialogPreferencesGeometry", saveGeometry());
    reject();
}

void DialogPreferences::menuRequestPath(QPoint pos)
{
    // this should always come from a QLineEdit
    QLineEdit *lineEdit = qobject_cast<QLineEdit*>(sender());
    if (lineEdit == 0) return;

    QMenu *menu = lineEdit->createStandardContextMenu();
    menu->addSeparator();
    menu->addAction(tr("Select Folder..."));


    QPoint gp = lineEdit->mapToGlobal(pos);

    QAction *action = menu->exec(gp);
    if (action)
    {
        if (action->text() == tr("Select Folder..."))
        {
            QString dir = QFileDialog::getExistingDirectory(this, "Select required folder", lineEdit->text());
            if (!dir.isEmpty())
            {
                lineEdit->setText(dir);
            }
        }
    }
    delete menu;
}

