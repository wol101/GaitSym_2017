#include "Settings.h"

#include <QSettings>

const QString Settings::applicationName("GaitSym2017");
const QString Settings::organizationName("AnimalSimulationLaboratory");
QSettings *Settings::m_settings = new QSettings(QSettings::IniFormat, QSettings::UserScope, Settings::organizationName, Settings::applicationName);

Settings::Settings()
{
}

void Settings::setValue(const QString &key, const QVariant &value)
{
    Q_ASSERT(m_settings);
    QString newKey1 = QString("%1_key_%2").arg(applicationName).arg(key);
    QString newKey2 = QString("%1_type_%2").arg(applicationName).arg(key);
    m_settings->setValue(newKey1, value);
    m_settings->setValue(newKey2, value.typeName()); // we have to do this because some settings formats lose the explicit type
}

QVariant Settings::value(const QString &key, const QVariant &defaultValue)
{
    Q_ASSERT(m_settings);
    QVariant variant = defaultValue;
    QString newKey1 = QString("%1_key_%2").arg(applicationName).arg(key);
    QString newKey2 = QString("%1_type_%2").arg(applicationName).arg(key);
    if (m_settings->contains(newKey1))
    {
        variant = m_settings->value(newKey1);
        if (m_settings->contains(newKey2))
        {
            QString typeName = m_settings->value(newKey2).toString();
            variant.convert(QVariant::nameToType(typeName.toUtf8())); // convert to the type stored in the settings
        }
        else
        {
            m_settings->setValue(newKey2, defaultValue.typeName());
        }
    }
    else // if the setting does not exist, create it and set it to the default value
    {
        setValue(key, defaultValue);
    }
    return variant;
}

void Settings::clear()
{
    Q_ASSERT(m_settings);
    m_settings->clear();
}

void Settings::sync()
{
    Q_ASSERT(m_settings);
    m_settings->sync();
}

QStringList Settings::allKeys()
{
    Q_ASSERT(m_settings);
    QStringList keys = m_settings->allKeys();
    QStringList newKeys;
    newKeys.reserve(keys.size());
    QString prefix = QString("%1_key_").arg(applicationName);
    for (int i = 0; i < keys.size(); i++)
    {
        if (keys.at(i).startsWith(prefix) == false) continue;
        QString newKey = keys.at(i).mid(prefix.size());
        newKeys.append(newKey);
    }
    return newKeys;
}

QString Settings::fileName()
{
    Q_ASSERT(m_settings);
    return m_settings->fileName();
}




