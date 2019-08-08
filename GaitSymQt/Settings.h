#ifndef SETTINGS_H
#define SETTINGS_H

#include <QVariant>

class QSettings;

class Settings
{
public:

    static const QString applicationName;
    static const QString organizationName;

    Settings();

    static void setValue(const QString &key, const QVariant &value);
    static QVariant value(const QString &key, const QVariant &defaultValue);

    static void clear();
    static void sync();

    static QStringList allKeys();
    static QString fileName();

private:
    static QSettings *m_settings;
};

#endif // SETTINGS_H
