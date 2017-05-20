/*
    JosluFocus
    Copyright (C) 2006 Markus Wildi (markus.wildi@datacomm.ch)
                  2011 Jasem Mutlaq (mutlaqja@ikarustech.com)

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

*/

#ifndef JosluFocus_H
#define JosluFocus_H

#include <indifocuser.h>
#include "joslufocus.h"

class JosluFocus : public INDI::Focuser
{
public:
    JosluFocus();
    ~JosluFocus();

    virtual bool Handshake();
    const char * getDefaultName();
    virtual bool initProperties();
    virtual bool updateProperties();
    virtual bool ISNewNumber (const char *dev, const char *name, double values[], char *names[], int n);
    virtual bool ISNewSwitch (const char *dev, const char *name, ISState *states, char *names[], int n);
    virtual IPState MoveAbsFocuser(uint32_t ticks);
    virtual IPState MoveRelFocuser(FocusDirection dir, uint32_t ticks);
    virtual bool AbortFocuser();
    virtual void TimerHit();

protected:
    bool saveConfigItems(FILE *fp);

private:

    int timerID;
    double targetPos;
    double simulatedTemperature;
    double simulatedHumidity;
    double simulatedPosition;

    unsigned char CheckSum(char *rf_cmd);
    unsigned char CalculateSum(char *rf_cmd);
    int SendCommand(char *rf_cmd);
    int ReadResponse(char *buf);
    void GetFocusParams();

    int updateRFPosition(double *value);
    int updateRFTempHum(double *temp, double *hum) ;
    int updateRFBacklash(double *value);
    int updateRFFirmware(char *rf_cmd) ;
    int updateRFMotorSettings(double *duty, double *delay, double *ticks);
    int updateRFPositionRelativeInward(double value);
    int updateRFPositionRelativeOutward(double value);
    int updateRFPositionAbsolute(double value);
    int updateRFPowerSwitches(int s, int  new_sn, int *cur_s1LL, int *cur_s2LR, int *cur_s3RL, int *cur_s4RR) ;
    int updateRFMaxPosition(double *value);
    int updateRFSetPosition(double *value);
    int updateRFHeatersSwitches(int s, int  new_sn, int *cur_s1LL, int *cur_s2LR, int *cur_s3RL) ;
    int updateRFHeatersPWM(unsigned char *pwm1, unsigned char *pwm2, unsigned char *pwm3) ;
    int updateRFHeatersTemp(double *temp1, double *temp2, double *temp3) ;

    int ReadUntilComplete(char *buf, int timeout);


    INumber TempHumN[2];
    INumberVectorProperty TempHumNP;

    INumber SettingsN[3];
    INumberVectorProperty SettingsNP;

    ISwitch PowerSwitchesS[4];
    ISwitchVectorProperty PowerSwitchesSP;

    ISwitch HeatersSwitchesS[3];
    ISwitchVectorProperty HeatersSwitchesSP;

    INumber HeatersPWMN[3];
    INumberVectorProperty HeatersPWMNP;
    
    INumber HeatersTempN[3];
    INumberVectorProperty HeatersTempNP;

    INumber MinMaxPositionN[2];
    INumberVectorProperty MinMaxPositionNP;

    INumber MaxTravelN[1];
    INumberVectorProperty MaxTravelNP;

    INumber SetRegisterPositionN[1];
    INumberVectorProperty SetRegisterPositionNP;

    INumber RelMovementN[1];
    INumberVectorProperty RelMovementNP;

    INumber AbsMovementN[1];
    INumberVectorProperty AbsMovementNP;

    INumber SetBacklashN[1];
    INumberVectorProperty SetBacklashNP;

};

#endif // JosluFocus_H
