/*
 *  Preferences.cpp
 *  GaitSym2017
 *
 *  Created by Bill Sellers on 12/02/2017.
 *  Copyright 2017 Bill Sellers. All rights reserved.
 *
 */

#ifndef PREFERENCES_H
#define PREFERENCES_H

#include <QColor>
#include <QString>
#include <QByteArray>
#include <QVariantMap>
#include <QVariant>
#include <QMap>
#include <QDomElement>

struct SettingsItem
{
    SettingsItem() { order = -1; display = false; path = false; }
    QString key;
    QString label;
    QVariant defaultValue;
    QVariant value;
    int order;
    bool display;
    bool path;
};

class Preferences
{
public:
    Preferences();
    ~Preferences();

    void Read();
    void Write();
    void Export(const QString &filename);
    void Import(const QString &filename);
    void ParseQDomElement(const QDomElement &docElem);
    void LoadDefaults();

    const SettingsItem settingsItem(const QString &key) const;
    void value(const QString &key, QVariant *value) const;
    void value(const QString &key, QString *value) const;
    void value(const QString &key, QColor *value) const;
    void value(const QString &key, QByteArray *value) const;
    void value(const QString &key, double *value) const;
    void value(const QString &key, float *value) const;
    void value(const QString &key, int *value) const;
    void value(const QString &key, bool *value) const;
    QVariant valueQVariant(const QString &key) const;
    QString valueQString(const QString &key) const;
    QColor valueQColor(const QString &key) const;
    QByteArray valueQByteArray(const QString &key) const;
    double valueDouble(const QString &key) const;
    float valueFloat(const QString &key) const;
    int valueInt(const QString &key) const;
    bool valueBool(const QString &key) const;
    void insert(const SettingsItem &item);
    void insert(const QString &key, const QVariant &value);
    bool contains(const QString &key) const;
    QStringList keys() const;

    static bool toBool(const QString &string);

protected:

    QMap<QString, SettingsItem> m_settings;
};

#endif // PREFERENCES_H
