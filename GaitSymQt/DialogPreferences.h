/*
 *  DialogPreferences.h
 *  GaitSym2017
 *
 *  Created by Bill Sellers on 12/02/2017.
 *  Copyright 2017 Bill Sellers. All rights reserved.
 *
 */

#ifndef DIALOGPREFERENCES_H
#define DIALOGPREFERENCES_H

#include <QDialog>
#include <QColor>

#include "Preferences.h"

struct SettingsWidget
{
    QWidget *widget;
    SettingsItem item;
};

class DialogPreferences : public QDialog
{
    Q_OBJECT

public:
    explicit DialogPreferences(QWidget *parent = 0);
    ~DialogPreferences();

    void setPreferences(const Preferences &preferences);
    Preferences preferences();

    QColor getIdealTextColour(const QColor& rBackgroundColour);
    QColor getAlphaColourHint(const QColor& colour);

public slots:
    void colourButtonClicked();
    void importButtonClicked();
    void exportButtonClicked();
    void defaultsButtonClicked();
    void acceptButtonClicked();
    void rejectButtonClicked();
    void menuRequestPath(QPoint pos);

private:
    QList<SettingsWidget> m_SettingsWidgetList;
    Preferences m_Preferences;
};

#endif // DIALOGPREFERENCES_H
