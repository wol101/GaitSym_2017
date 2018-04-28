/*
 *  Preferences.cpp
 *  GaitSym2017
 *
 *  Created by Bill Sellers on 12/02/2017.
 *  Copyright 2017 Bill Sellers. All rights reserved.
 *
 */

#include <QSettings>
#include <QFile>
#include <QTextStream>
#include <QString>
#include <QColor>
#include <QDebug>
#include <QDomDocument>

#include "Preferences.h"

Preferences::Preferences()
{
}

Preferences::~Preferences()
{
}

void Preferences::Write()
{
    qDebug() << "Preferences::Write() Organisation " << valueQString("Organisation") << " Application " << valueQString("Application");
    QSettings settings(valueQString("Organisation"), valueQString("Application"));
    settings.clear();
    for (QMap<QString, SettingsItem>::const_iterator i = m_settings.constBegin(); i != m_settings.constEnd(); i++)
    {
        qDebug("%s: %s", qUtf8Printable(i.key()), qUtf8Printable(i.value().value.toString()));
        settings.setValue(i.key(), i.value().value);
    }
    settings.sync();
}

void Preferences::Read()
{
    LoadDefaults();
    qDebug() << "Preferences::Read() Organisation " << valueQString("Organisation") << " Application " << valueQString("Application");
    QSettings settings(valueQString("Organisation"), valueQString("Application"));
    // check whether the settings are the right ones
    if (settings.value("SettingsCode") != m_settings["SettingsCode"].value)
    {
        settings.clear();
        settings.sync();
    }
    else
    {
        QStringList keys = settings.allKeys();
        for (int i = 0; i < keys.size(); i++) insert(keys[i], settings.value(keys[i]));
    }
}

void Preferences::Export(const QString &filename)
{
    QDomDocument doc("GaitSym2017Preferences");
    QDomElement root = doc.createElement("PREFERENCES");
    doc.appendChild(root);

    // this bit of code gets the SettingsItems sorted by order
    QStringList keys = m_settings.keys();
    keys.sort(Qt::CaseInsensitive);
    QMultiMap<int, SettingsItem> sortedItems;
    for (int i = 0; i < keys.size(); i++)
    {
        SettingsItem item = m_settings[keys[i]];
        sortedItems.insert(item.order, item);
    }

    for (QMultiMap<int, SettingsItem>::const_iterator i = sortedItems.constBegin(); i != sortedItems.constEnd(); i++)
    {
        qDebug("%s: %s", qUtf8Printable(i.key()), qUtf8Printable(i.value().value.toString()));
        QDomElement setting = doc.createElement("SETTING");
        root.appendChild(setting);
        setting.setAttribute("key", i.value().key);
        setting.setAttribute("type", i.value().value.typeName());
        setting.setAttribute("display", QString::number(i.value().display));
        setting.setAttribute("path", QString::number(i.value().path));
        setting.setAttribute("label", i.value().label);
        setting.setAttribute("order", QString::number(i.value().order));
        switch (i.value().value.type())
        {
        case QMetaType::QByteArray:
            setting.setAttribute("defaultValue", QString::fromUtf8(i.value().defaultValue.toByteArray().toBase64(QByteArray::Base64UrlEncoding)));
            setting.setAttribute("value", QString::fromUtf8(i.value().value.toByteArray().toBase64(QByteArray::Base64UrlEncoding)));
            break;
        case QMetaType::QColor:
            setting.setAttribute("defaultValue", qvariant_cast<QColor>(i.value().defaultValue).name(QColor::HexArgb));
            setting.setAttribute("value", qvariant_cast<QColor>(i.value().value).name(QColor::HexArgb));
            break;
        default:
            setting.setAttribute("defaultValue", i.value().defaultValue.toString());
            setting.setAttribute("value", i.value().value.toString());
        }
    }

    QFile file(filename);
    if (!file.open(QIODevice::WriteOnly))
    {
        qWarning("Unable to open settings export file: %s", qPrintable(filename));
        return;
    }

    // we need to add the processing instruction at the beginning of the document
    QString encoding("<?xml version=\"1.0\" encoding=\"utf-8\"?>\n");
    file.write(encoding.toUtf8());

    // and now the actual xml doc
    QString xmlString = doc.toString();
    QByteArray xmlData = xmlString.toUtf8();
    qint64 bytesWritten = file.write(xmlData);
    if (bytesWritten != xmlData.size()) qWarning("Unable to write to settings export file: %s", qPrintable(filename));
    file.close();
}

void Preferences::Import(const QString &filename)
{
    m_settings.clear();

    QDomDocument doc("GaitSym2017Preferences");
    QFile file(filename);
    if (!file.open(QIODevice::ReadOnly))
    {
        qWarning("Unable to open settings export file: %s", qPrintable(filename));
        return;
    }
    if (!doc.setContent(&file))
    {
        qWarning("Unable to read settings export file: %s", qPrintable(filename));
        return;
    }
    file.close();

    QDomElement docElem = doc.documentElement();   
    ParseQDomElement(docElem);
}

void Preferences::ParseQDomElement(const QDomElement &docElem)
{
    qDebug() << qPrintable(docElem.tagName()) << "\n";
    if (docElem.tagName() != "PREFERENCES")
    {
        qWarning("Unable to find tag PREFERENCES: %s", qPrintable(docElem.tagName()));
        return;
    }

    SettingsItem item;
    QString key;
    QDomNode n = docElem.firstChild();
    while(!n.isNull())
    {
        QDomElement e = n.toElement(); // try to convert the node to an element.
        qDebug() << e.tagName() << "\n";
        if (!e.isNull())
        {
            if (e.tagName() == "SETTING")
            {
//                QDomNamedNodeMap attrs = e.attributes();
//                for (int i =0; i < attrs.length(); i++)
//                    qDebug() << attrs.item(i).toAttr().name() << "\n";
                item.key = e.attribute("key");
                QVariant::Type type = QVariant::nameToType(e.attribute("type").toUtf8());
                item.display = toBool(e.attribute("display"));
                item.path = toBool(e.attribute("path"));
                item.label = e.attribute("label");
                item.order = e.attribute("order").toInt();
                switch (type)
                {
                case QMetaType::QByteArray:
                    item.value = QByteArray::fromBase64(QString(e.attribute("value")).toUtf8(), QByteArray::Base64UrlEncoding);
                    item.defaultValue = QByteArray::fromBase64(QString(e.attribute("defaultValue")).toUtf8(), QByteArray::Base64UrlEncoding);
                    break;
                case QMetaType::QColor:
                    item.value = QColor(e.attribute("value"));
                    item.defaultValue = QColor(e.attribute("defaultValue"));
                    break;
                default:
                    item.value = e.attribute("value");
                    item.defaultValue = e.attribute("defaultValue");
                    item.value.convert(type);
                    item.defaultValue.convert(type);
                }
                m_settings[item.key] = item;
            }
        }
        n = n.nextSibling();
    }
}

void Preferences::LoadDefaults()
{
    Import(":/preferences/default_values.xml");

    for (QMap<QString, SettingsItem>::const_iterator i = m_settings.constBegin(); i != m_settings.constEnd(); i++) insert(i.key(), i.value().defaultValue);
}

bool Preferences::toBool(const QString &string)
{
    if (string.trimmed().compare("true", Qt::CaseInsensitive) == 0 || string.toInt() != 0) return true;
    return false;
}

const SettingsItem Preferences::settingsItem(const QString &key) const
{
    return m_settings.value(key);
}

void Preferences::value(const QString &key, QVariant *value) const
{
    if (m_settings.contains(key)) *value = m_settings.value(key).value;
}

void Preferences::value(const QString &key, QString *value) const
{
    if (m_settings.contains(key)) *value = m_settings.value(key).value.toString();
}

void Preferences::value(const QString &key, QColor *value) const
{
    if (m_settings.contains(key)) *value = qvariant_cast<QColor>(m_settings.value(key).value);
}

void Preferences::value(const QString &key, QByteArray *value) const
{
    if (m_settings.contains(key)) *value = qvariant_cast<QByteArray>(m_settings.value(key).value);
}

void Preferences::value(const QString &key, double *value) const
{
    if (m_settings.contains(key)) *value = m_settings.value(key).value.toDouble();
}

void Preferences::value(const QString &key, float *value) const
{
    if (m_settings.contains(key)) *value = static_cast<float>(m_settings.value(key).value.toDouble());
}

void Preferences::value(const QString &key, int *value) const
{
    if (m_settings.contains(key)) *value = m_settings.value(key).value.toInt();
}

void Preferences::value(const QString &key, bool *value) const
{
    if (m_settings.contains(key)) *value = m_settings.value(key).value.toBool();
}

QVariant Preferences::valueQVariant(const QString &key) const
{
    QVariant v;
    value(key, &v);
    return v;
}

QString Preferences::valueQString(const QString &key) const
{
    QString v;
    value(key, &v);
    return v;
}

QColor Preferences::valueQColor(const QString &key) const
{
    QColor v;
    value(key, &v);
    return v;
}

QByteArray Preferences::valueQByteArray(const QString &key) const
{
    QByteArray v;
    value(key, &v);
    return v;
}

double Preferences::valueDouble(const QString &key) const
{
    double v;
    value(key, &v);
    return v;
}

float Preferences::valueFloat(const QString &key) const
{
    float v;
    value(key, &v);
    return v;
}

int Preferences::valueInt(const QString &key) const
{
    int v;
    value(key, &v);
    return v;
}

bool Preferences::valueBool(const QString &key) const
{
    bool v;
    value(key, &v);
    return v;
}


void Preferences::insert(const SettingsItem &item)
{
    m_settings.insert(item.key, item);
}

bool Preferences::contains(const QString &key) const
{
    return m_settings.contains(key);
}

QStringList Preferences::keys() const
{
    return m_settings.keys();
}

void Preferences::insert(const QString &key, const QVariant &value)
{
    SettingsItem item;
    if (m_settings.contains(key))
    {
        item = m_settings.value(key);
        item.value = value;
        m_settings.insert(key, item);
    }
    else
    {
        item.key = key;
        item.display = false;
        item.label = "";
        item.order = -1;
        item.value = value;
        item.defaultValue = QVariant(value.type());
        m_settings.insert(key, item);

    }
}


