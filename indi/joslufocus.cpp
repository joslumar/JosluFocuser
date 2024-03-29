/*
    JosluFocus

    RoboFocus
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

#include "joslufocus.h"
#include "indicom.h"

#include <termios.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>
#include <memory>

#define RF_MAX_CMD   9
#define RF_TIMEOUT   15
#define RF_STEP_RES  5

#define RF_MAX_TRIES    3
#define RF_MAX_DELAY    100000

#define BACKLASH_READOUT 99999
#define MAXTRAVEL_READOUT 99999

#define currentSpeed            SpeedN[0].value
#define currentPosition         FocusAbsPosN[0].value
#define currentTemperature      TempHumN[0].value
#define currentHumidity         TempHumN[1].value
#define currentBacklash         SetBacklashN[0].value
#define currentDuty             SettingsN[0].value
#define currentDelay            SettingsN[1].value
#define currentTicks            SettingsN[2].value
#define currentRelativeMovement FocusRelPosN[0].value
#define currentAbsoluteMovement FocusAbsPosN[0].value
#define currentSetBacklash      SetBacklashN[0].value
#define currentMinPosition      MinMaxPositionN[0].value
#define currentMaxPosition      MinMaxPositionN[1].value
#define currentMaxTravel        MaxTravelN[0].value

#define POLLMS  1000

#define SETTINGS_TAB            "Settings"
#define HEATERS_TAB             "Heaters Control"

std::unique_ptr<JosluFocus> josluFocus(new JosluFocus());

void ISGetProperties(const char *dev)
{
    josluFocus->ISGetProperties(dev);
}

void ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int num)
{
    josluFocus->ISNewSwitch(dev, name, states, names, num);
}

void ISNewText(	const char *dev, const char *name, char *texts[], char *names[], int num)
{
    josluFocus->ISNewText(dev, name, texts, names, num);
}

void ISNewNumber(const char *dev, const char *name, double values[], char *names[], int num)
{
    josluFocus->ISNewNumber(dev, name, values, names, num);
}

void ISNewBLOB (const char *dev, const char *name, int sizes[], int blobsizes[], char *blobs[], char *formats[], char *names[], int n)
{
    INDI_UNUSED(dev);
    INDI_UNUSED(name);
    INDI_UNUSED(sizes);
    INDI_UNUSED(blobsizes);
    INDI_UNUSED(blobs);
    INDI_UNUSED(formats);
    INDI_UNUSED(names);
    INDI_UNUSED(n);
}

void ISSnoopDevice (XMLEle *root)
{
    josluFocus->ISSnoopDevice(root);
}

JosluFocus::JosluFocus()
{       
    timerID = -1;
    SetFocuserCapability(FOCUSER_CAN_ABS_MOVE | FOCUSER_CAN_REL_MOVE | FOCUSER_CAN_ABORT);
}

JosluFocus::~JosluFocus()
{

}

bool JosluFocus::initProperties()
{
    INDI::Focuser::initProperties();

    /* Focuser temperature */
    IUFillNumber(&TempHumN[0], "TEMPERATURE", "Celsius", "%6.2f", 0, 65000., 0., 10000.);
    IUFillNumber(&TempHumN[1], "HUMIDITY", "%", "%6.2f", 0, 65000., 0., 10000.);
    IUFillNumberVector(&TempHumNP, TempHumN, 2, getDeviceName(), "FOCUS_TEMPERATURE", "Temperature", MAIN_CONTROL_TAB, IP_RO, 0, IPS_IDLE);

    /* Settings of the JosluFocus */
    IUFillNumber(&SettingsN[0], "Duty cycle", "Duty cycle", "%6.0f", 0., 255., 0., 1.0);
    IUFillNumber(&SettingsN[1], "Step Delay", "Step delay", "%6.0f", 0., 255., 0., 1.0);
    IUFillNumber(&SettingsN[2], "Motor Steps", "Motor steps per tick", "%6.0f", 0., 255., 0., 1.0);
    IUFillNumberVector(&SettingsNP, SettingsN, 3, getDeviceName(), "FOCUS_SETTINGS", "Settings", SETTINGS_TAB, IP_RW, 0, IPS_IDLE);

    /* Power Switches of the JosluFocus */
    IUFillSwitch(&PowerSwitchesS[0], "1", "Switch 1", ISS_OFF);
    IUFillSwitch(&PowerSwitchesS[1], "2", "Switch 2", ISS_OFF);
    IUFillSwitch(&PowerSwitchesS[2], "3", "Switch 3", ISS_OFF);
    IUFillSwitch(&PowerSwitchesS[3], "4", "Switch 4", ISS_ON);
    IUFillSwitchVector(&PowerSwitchesSP, PowerSwitchesS, 4, getDeviceName(), "SWTICHES", "Power", SETTINGS_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

    /* JosluFocus should stay within these limits */
    IUFillNumber(&MinMaxPositionN[0], "MINPOS", "Minimum Tick", "%6.0f", 1., 65000., 0., 100. );
    IUFillNumber(&MinMaxPositionN[1], "MAXPOS", "Maximum Tick", "%6.0f", 1., 65000., 0., 55000.);
    IUFillNumberVector(&MinMaxPositionNP, MinMaxPositionN, 2, getDeviceName(), "FOCUS_MINMAXPOSITION", "Extrema", SETTINGS_TAB, IP_RW, 0, IPS_IDLE);

    IUFillNumber(&MaxTravelN[0], "MAXTRAVEL", "Maximum travel", "%6.0f", 1., 64000., 0., 10000.);
    IUFillNumberVector(&MaxTravelNP, MaxTravelN, 1, getDeviceName(), "FOCUS_MAXTRAVEL", "Max. travel", SETTINGS_TAB, IP_RW, 0, IPS_IDLE );

    /* Set JosluFocus position register to this position */
    IUFillNumber(&SetRegisterPositionN[0], "SETPOS", "Position", "%6.0f", 0, 64000., 0., 0. );
    IUFillNumberVector(&SetRegisterPositionNP, SetRegisterPositionN, 1, getDeviceName(), "FOCUS_REGISTERPOSITION", "Sync", SETTINGS_TAB, IP_RW, 0, IPS_IDLE);

    /* Backlash */
    IUFillNumber(&SetBacklashN[0], "SETBACKLASH", "Backlash", "%6.0f", -255., 255., 0., 0.);
    IUFillNumberVector(&SetBacklashNP, SetBacklashN, 1, getDeviceName(), "FOCUS_BACKLASH", "Set Register", SETTINGS_TAB, IP_RW, 0, IPS_IDLE);

    /* Heaters Switches of the JosluFocus */
    IUFillSwitch(&HeatersSwitchesS[0], "1", "Heater 1", ISS_OFF);
    IUFillSwitch(&HeatersSwitchesS[1], "2", "Heater 2", ISS_OFF);
    IUFillSwitch(&HeatersSwitchesS[2], "3", "Heater 3", ISS_OFF);
    IUFillSwitchVector(&HeatersSwitchesSP, HeatersSwitchesS, 3, getDeviceName(), "HEATERS_SWTICHES", "Heaters", HEATERS_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

    /* Heaters PWM of the JosluFocus */
    IUFillNumber(&HeatersPWMN[0], "1", "PWM Heater 1", "%6.0f", 0., 100., 0., 1.0);
    IUFillNumber(&HeatersPWMN[1], "2", "PWM Heater 2", "%6.0f", 0., 100., 0., 1.0);
    IUFillNumber(&HeatersPWMN[2], "3", "PWM Heater 3", "%6.0f", 0., 100., 0., 1.0);
    IUFillNumberVector(&HeatersPWMNP, HeatersPWMN, 3, getDeviceName(), "HEATERS_PWM", "Heaters Power", HEATERS_TAB, IP_RW, 0, IPS_IDLE);

    /* Heaters temperature of the JosluFocus */
    IUFillNumber(&HeatersTempN[0], "1", "Temperature Heater 1", "%6.0f", 0., 255., 0., 1.0);
    IUFillNumber(&HeatersTempN[1], "2", "Temperature Heater 2", "%6.0f", 0., 255., 0., 1.0);
    IUFillNumber(&HeatersTempN[2], "3", "Temperature Heater 3", "%6.0f", 0., 255., 0., 1.0);
    IUFillNumberVector(&HeatersPWMNP, HeatersTempN, 3, getDeviceName(), "HEATERS_TEMPERATURE", "Heaters Temperature", HEATERS_TAB, IP_RO, 0, IPS_IDLE);

    /* Relative and absolute movement */
    FocusRelPosN[0].min = -5000.;
    FocusRelPosN[0].max = 5000.;
    FocusRelPosN[0].value = 100;
    FocusRelPosN[0].step = 100;

    FocusAbsPosN[0].min = 0.;
    FocusAbsPosN[0].max = 64000.;
    FocusAbsPosN[0].value = 0;
    FocusAbsPosN[0].step = 1000;

    simulatedTemperature=600.0;
    simulatedHumidity=60.0;
    simulatedPosition=20000;

    addDebugControl();
    addSimulationControl();

    return true;

}

bool JosluFocus::updateProperties()
{
    INDI::Focuser::updateProperties();

    if (isConnected())
    {
        defineNumber(&TempHumNP);
        defineSwitch(&PowerSwitchesSP);
        defineNumber(&SettingsNP);
        defineNumber(&MinMaxPositionNP);
        defineNumber(&MaxTravelNP);
        defineNumber(&SetRegisterPositionNP);
        defineNumber(&SetBacklashNP);
        defineNumber(&FocusRelPosNP);
        defineNumber(&FocusAbsPosNP);
        defineSwitch(&HeatersSwitchesSP);
        defineNumber(&HeatersPWMNP);
        defineNumber(&HeatersTempNP);

        GetFocusParams();

        DEBUG(INDI::Logger::DBG_DEBUG, "JosluFocus paramaters readout complete, focuser ready for use.");
    }
    else
    {

        deleteProperty(TempHumNP.name);
        deleteProperty(SettingsNP.name);
        deleteProperty(PowerSwitchesSP.name);
        deleteProperty(MinMaxPositionNP.name);
        deleteProperty(MaxTravelNP.name);
        deleteProperty(SetRegisterPositionNP.name);
        deleteProperty(SetBacklashNP.name);
        deleteProperty(FocusRelPosNP.name);
        deleteProperty(FocusAbsPosNP.name);
        deleteProperty(HeatersSwitchesSP.name);
        deleteProperty(HeatersPWMNP.name);
        deleteProperty(HeatersTempNP.name);

    }

    return true;

}

bool JosluFocus::Handshake()
{
    char firmeware[]="FV0000000";

    if (isSimulation())
    {
        timerID = SetTimer(POLLMS);
        DEBUG(INDI::Logger::DBG_SESSION, "Simulated JosluFocus is online. Getting focus parameters...");
        FocusAbsPosN[0].value = simulatedPosition;
        updateRFFirmware(firmeware);
        return true;
    }

    if((updateRFFirmware(firmeware)) < 0)
    {
        /* This would be the end*/
        DEBUG(INDI::Logger::DBG_ERROR, "Unknown error while reading JosluFocus firmware.");
        return false;
    }

    return true;
}

const char * JosluFocus::getDefaultName()
{
    return "JosluFocus";
}

unsigned char JosluFocus::CheckSum(char *rf_cmd)
{
    char substr[255] ;
    unsigned char val= 0 ;
    int i= 0 ;

    for(i=0; i < 8; i++)
        substr[i]=rf_cmd[i] ;

    val = CalculateSum( substr) ;

    if( val !=  (unsigned char) rf_cmd[8])
        DEBUGF(INDI::Logger::DBG_WARNING, "Checksum: Wrong (%s,%ld), %x != %x",  rf_cmd, strlen(rf_cmd), val, (unsigned char) rf_cmd[8]) ;

    return val ;
}

unsigned char JosluFocus::CalculateSum(char *rf_cmd)
{
    unsigned char val= 0 ;
    int i=0 ;

    for(i=0; i < 8; i++)
        val = val + (unsigned char) rf_cmd[i];

    return val % 256 ;
}

int JosluFocus::SendCommand(char *rf_cmd)
{
    int nbytes_written=0, nbytes_read=0, check_ret=0, err_code=0;
    char rf_cmd_cks[32],JosluFocus_error[MAXRBUF];

    unsigned char val= 0 ;

    val = CalculateSum( rf_cmd );

    for(int i=0; i < 8; i++)
        rf_cmd_cks[i]= rf_cmd[i] ;

    rf_cmd_cks[8]=  (unsigned char) val ;
    rf_cmd_cks[9]= 0 ;

    if (isSimulation())
        return 0;

    tcflush(PortFD, TCIOFLUSH);

    DEBUGF(INDI::Logger::DBG_DEBUG, "CMD (%#02X %#02X %#02X %#02X %#02X %#02X %#02X %#02X %#02X)", rf_cmd_cks[0], rf_cmd_cks[1], rf_cmd_cks[2], rf_cmd_cks[3], rf_cmd_cks[4], rf_cmd_cks[5], rf_cmd_cks[6], rf_cmd_cks[7], rf_cmd_cks[8]);

    if  ( (err_code = tty_write(PortFD, rf_cmd_cks, RF_MAX_CMD, &nbytes_written) != TTY_OK))
    {
        tty_error_msg(err_code, JosluFocus_error, MAXRBUF);
        DEBUGF(INDI::Logger::DBG_ERROR, "TTY error detected: %s", JosluFocus_error);
        return -1;
    }

    return nbytes_written;
}

int JosluFocus::ReadResponse(char *buf)
{    
    char JosluFocus_error[MAXRBUF];
    char JosluFocus_char[1];
    int bytesRead = 0;
    int err_code;

    char motion = 0;
    bool externalMotion=false;

    if (isSimulation())
        return RF_MAX_CMD;

    while (1)
    {
        if ( (err_code = tty_read(PortFD, JosluFocus_char, 1, RF_TIMEOUT, &bytesRead)) != TTY_OK)
        {
            tty_error_msg(err_code, JosluFocus_error, MAXRBUF);
            DEBUGF(INDI::Logger::DBG_ERROR, "TTY error detected: %s", JosluFocus_error);
            return -1;
        }

        switch (JosluFocus_char[0])
        {
        // Catch 'I'
        case 0x49:
            if (motion != 0x49)
            {
                motion = 0x49;
                DEBUG(INDI::Logger::DBG_SESSION, "Moving inward...");

                if (FocusAbsPosNP.s != IPS_BUSY)
                {
                    externalMotion = true;
                    FocusAbsPosNP.s = IPS_BUSY;
                    IDSetNumber(&FocusAbsPosNP, NULL);
                }
            }
            //usleep(100000);
            break;

            // catch 'O'
        case 0x4F:
            if (motion != 0x4F)
            {
                motion = 0x4F;
                DEBUG(INDI::Logger::DBG_SESSION, "Moving outward...");

                if (FocusAbsPosNP.s != IPS_BUSY)
                {
                    externalMotion = true;
                    FocusAbsPosNP.s = IPS_BUSY;
                    IDSetNumber(&FocusAbsPosNP, NULL);
                }
            }
            //usleep(100000);
            break;

            // Start of frame
        case 0x46:
            buf[0]=0x46;
            // Read rest of frame
            if ( (err_code = tty_read(PortFD, buf+1, RF_MAX_CMD-1, RF_TIMEOUT, &bytesRead)) != TTY_OK)
            {
                tty_error_msg(err_code, JosluFocus_error, MAXRBUF);
                DEBUGF(INDI::Logger::DBG_ERROR, "TTY error detected: %s", JosluFocus_error);
                return -1;
            }

            if (motion != 0)
            {
                DEBUG(INDI::Logger::DBG_SESSION, "Stopped.");

                // If we set it busy due to external motion, let's set it to OK
                if (externalMotion)
                {
                    FocusAbsPosNP.s = IPS_OK;
                    IDSetNumber(&FocusAbsPosNP, NULL);
                }
            }

            tcflush(PortFD, TCIOFLUSH);
            return (bytesRead+1);
            break;

        default:
            break;
        }
    }

    return -1;
}

int JosluFocus::updateRFPosition(double *value)
{
    float temp ;
    char rf_cmd[RF_MAX_CMD] ;
    int JosluFocus_rc ;

    DEBUG(INDI::Logger::DBG_DEBUG, "Querying Position...");

    if (isSimulation())
    {
        *value = simulatedPosition;
        return 0;
    }

    strcpy(rf_cmd, "FG000000" ) ;

    if ((JosluFocus_rc= SendCommand(rf_cmd)) < 0)
        return JosluFocus_rc;

    if ((JosluFocus_rc= ReadResponse(rf_cmd)) < 0)
        return JosluFocus_rc;

    if (sscanf(rf_cmd, "FD%6f", &temp) < 1)
        return -1;

    *value = (double) temp;

    DEBUGF(INDI::Logger::DBG_DEBUG, "Position: %g", *value);

    return 0;

}

int JosluFocus::updateRFTempHum(double *temperature, double *humidity)
{
    DEBUGF(INDI::Logger::DBG_DEBUG, "Update Temperature: %g", temperature);
    DEBUGF(INDI::Logger::DBG_DEBUG, "Update Humidity: %g", humidity);

    float temp ;
    char rf_cmd[32] ;
    int JosluFocus_rc ;

    strcpy(rf_cmd, "FT000000" ) ;

    if ((JosluFocus_rc= SendCommand( rf_cmd)) < 0)
        return JosluFocus_rc;

    if (isSimulation())
        snprintf(rf_cmd, 32, "FT%6g", simulatedTemperature);
    else  if ((JosluFocus_rc= ReadResponse(rf_cmd)) < 0)
            return JosluFocus_rc;

    if (sscanf(rf_cmd, "FT%6f", &temp) < 1)
        return -1;

    *temperature = (double) temp/2.- 273.15;

    strcpy(rf_cmd, "FTH00000" ) ;

    if ((JosluFocus_rc= SendCommand( rf_cmd)) < 0)
        return JosluFocus_rc;

    if (isSimulation())
        snprintf(rf_cmd, 32, "FTH%5g", simulatedHumidity);
    else  if ((JosluFocus_rc= ReadResponse(rf_cmd)) < 0)
            return JosluFocus_rc;

    if (sscanf(rf_cmd, "FTH%5f", &temp) < 1)
        return -1;

    *humidity = (double) temp;

    return 0;
}

int JosluFocus::updateRFBacklash(double *value)
{
    DEBUGF(INDI::Logger::DBG_DEBUG, "Update Backlash: %g", value);

    float temp ;
    char rf_cmd[32] ;
    char vl_tmp[4] ;
    int JosluFocus_rc ;
    int sign= 0 ;

    if (isSimulation())
        return 0;

    if(*value== BACKLASH_READOUT)
    {

        strcpy(rf_cmd, "FB000000" ) ;

    }
    else
    {

        rf_cmd[0]=  'F' ;
        rf_cmd[1]=  'B' ;

        if( *value > 0) {

            rf_cmd[2]= '3' ;

        } else {
            *value= - *value ;
            rf_cmd[2]= '2' ;
        }
        rf_cmd[3]= '0' ;
        rf_cmd[4]= '0' ;

        if(*value > 99) {
            sprintf( vl_tmp, "%3d", (int) *value) ;

        } else if(*value > 9) {
            sprintf( vl_tmp, "0%2d", (int) *value) ;
        } else {
            sprintf( vl_tmp, "00%1d", (int) *value) ;
        }
        rf_cmd[5]= vl_tmp[0] ;
        rf_cmd[6]= vl_tmp[1] ;
        rf_cmd[7]= vl_tmp[2] ;
    }

    if ((JosluFocus_rc= SendCommand( rf_cmd)) < 0)
        return JosluFocus_rc;

    if ((JosluFocus_rc= ReadResponse(rf_cmd)) < 0)
        return JosluFocus_rc;

    if (sscanf(rf_cmd, "FB%1d%5f", &sign, &temp) < 1)
        return -1;

    *value = (double) temp  ;

    if(( sign== 2) && ( *value > 0))
    {
        *value = - (*value) ;
    }

    return 0;
}

int JosluFocus::updateRFFirmware(char *rf_cmd)
{
    int JosluFocus_rc ;

    DEBUG(INDI::Logger::DBG_DEBUG, "Querying JosluFocus Firmware...");

    strcpy(rf_cmd, "FV000000" ) ;

    if ((JosluFocus_rc= SendCommand(rf_cmd)) < 0)
        return JosluFocus_rc;

    if (isSimulation())
        strcpy(rf_cmd, "SIM");
    else if ((JosluFocus_rc= ReadResponse(rf_cmd)) < 0)
        return JosluFocus_rc;

    return 0;
}

int JosluFocus::updateRFMotorSettings(double *duty, double *delay, double *ticks)
{

    DEBUGF(INDI::Logger::DBG_DEBUG, "Update Motor Settings: Duty (%g), Delay (%g), Ticks(%g)", *duty, *delay, *ticks);

    char rf_cmd[32] ;
    int JosluFocus_rc ;

    if (isSimulation())
    {
        *duty = 100;
        *delay = 0;
        *ticks = 0;
        return 0;
    }

    if(( *duty== 0 ) && (*delay== 0) && (*ticks== 0) ){

        strcpy(rf_cmd, "FC000000" ) ;

    } else {

        rf_cmd[0]=  'F' ;
        rf_cmd[1]=  'C' ;
        rf_cmd[2]= (char) *duty ;
        rf_cmd[3]= (char) *delay ;
        rf_cmd[4]= (char) *ticks ;
        rf_cmd[5]= '0' ;
        rf_cmd[6]= '0' ;
        rf_cmd[7]= '0' ;
        rf_cmd[8]=  0 ;

    }

    if ((JosluFocus_rc= SendCommand( rf_cmd)) < 0)
        return JosluFocus_rc;

    if ((JosluFocus_rc= ReadResponse(rf_cmd)) < 0)
            return JosluFocus_rc;

    *duty=  (float) rf_cmd[2] ;
    *delay= (float) rf_cmd[3] ;
    *ticks= (float) rf_cmd[4] ;

    return 0;
}

int JosluFocus::updateRFPositionRelativeInward(double value)
{

    char rf_cmd[32] ;
    int JosluFocus_rc ;
    //float temp ;
    rf_cmd[0]= 0 ;

    DEBUGF(INDI::Logger::DBG_DEBUG, "Update Relative Position Inward: %g", value);

    if (isSimulation())
    {
        simulatedPosition+= value;
        //value = simulatedPosition;
        return 0;
    }

    if(value > 9999) {
        sprintf( rf_cmd, "FI0%5d", (int) value) ;
    } else if(value > 999) {
        sprintf( rf_cmd, "FI00%4d", (int) value) ;
    } else if(value > 99) {
        sprintf( rf_cmd, "FI000%3d", (int) value) ;
    } else if(value > 9) {
        sprintf( rf_cmd, "FI0000%2d", (int) value) ;
    } else {
        sprintf( rf_cmd, "FI00000%1d", (int) value) ;
    }

    if ((JosluFocus_rc= SendCommand( rf_cmd)) < 0)
        return JosluFocus_rc;

    return 0;
}

int JosluFocus::updateRFPositionRelativeOutward(double value)
{

    char rf_cmd[32] ;
    int JosluFocus_rc ;
    //float temp ;

    DEBUGF(INDI::Logger::DBG_DEBUG, "Update Relative Position Outward: %g", value);

    if (isSimulation())
    {
        simulatedPosition-= value;
        //value = simulatedPosition;
        return 0;
    }

    rf_cmd[0]= 0 ;

    if(value > 9999) {
        sprintf( rf_cmd, "FO0%5d", (int) value) ;
    } else if(value > 999) {
        sprintf( rf_cmd, "FO00%4d", (int) value) ;
    } else if(value > 99) {
        sprintf( rf_cmd, "FO000%3d", (int) value) ;
    } else if(value > 9) {
        sprintf( rf_cmd, "FO0000%2d", (int) value) ;
    } else {
        sprintf( rf_cmd, "FO00000%1d", (int) value) ;
    }

    if ((JosluFocus_rc= SendCommand( rf_cmd)) < 0)
        return JosluFocus_rc;

    return 0;
}

int JosluFocus::updateRFPositionAbsolute(double value)
{

    char rf_cmd[32] ;
    int JosluFocus_rc ;

    DEBUGF(INDI::Logger::DBG_DEBUG, "Moving Absolute Position: %g", value);

    if (isSimulation())
    {
        simulatedPosition = value;
        return 0;
    }

    rf_cmd[0]= 0 ;

    if(value > 9999) {
        sprintf( rf_cmd, "FG0%5d", (int) value) ;
    } else if(value > 999) {
        sprintf( rf_cmd, "FG00%4d", (int) value) ;
    } else if(value > 99) {
        sprintf( rf_cmd, "FG000%3d", (int) value) ;
    } else if(value > 9) {
        sprintf( rf_cmd, "FG0000%2d", (int) value) ;
    } else {
        sprintf( rf_cmd, "FG00000%1d", (int) value) ;
    }

    if ((JosluFocus_rc= SendCommand( rf_cmd)) < 0)
        return JosluFocus_rc;

    return 0;
}

int JosluFocus::updateRFPowerSwitches(int s, int  new_sn, int *cur_s1LL, int *cur_s2LR, int *cur_s3RL, int *cur_s4RR)
{    

    char rf_cmd[32] ;
    char rf_cmd_tmp[32] ;
    int JosluFocus_rc ;
    int i = 0 ;


    if (isSimulation())
    {
        return 0;
    }

    DEBUG(INDI::Logger::DBG_DEBUG, "Get switch status...");

    /* Get first the status */
    strcpy(rf_cmd_tmp, "FP000000" ) ;

    if ((JosluFocus_rc= SendCommand( rf_cmd_tmp)) < 0)
        return JosluFocus_rc ;

    if ((JosluFocus_rc= ReadResponse(rf_cmd_tmp)) < 0)
            return JosluFocus_rc;

    for(i= 0; i < 9; i++)
    {
        rf_cmd[i]= rf_cmd_tmp[i] ;
    }


    if( rf_cmd[new_sn + 4]== '2')
    {
        rf_cmd[new_sn + 4]= '1' ;
    }
    else
    {
        rf_cmd[new_sn + 4]= '2' ;
    }


    rf_cmd[8]= 0 ;

    if ((JosluFocus_rc= SendCommand( rf_cmd)) < 0)
        return JosluFocus_rc ;

    if ((JosluFocus_rc= ReadResponse(rf_cmd)) < 0)
            return JosluFocus_rc;

    *cur_s1LL= *cur_s2LR= *cur_s3RL= *cur_s4RR= ISS_OFF ;

    if(rf_cmd[4]== '2' ) {
        *cur_s1LL= ISS_ON ;
    }

    if(rf_cmd[5]== '2' ) {
        *cur_s2LR= ISS_ON ;
    }

    if(rf_cmd[6]== '2' ) {
        *cur_s3RL= ISS_ON ;
    }

    if(rf_cmd[7]== '2' ) {
        *cur_s4RR= ISS_ON ;
    }
    return 0 ;
}


int JosluFocus::updateRFMaxPosition(double *value)
{

    DEBUG(INDI::Logger::DBG_DEBUG, "Query max position...");

    float temp ;
    char rf_cmd[32] ;
    char vl_tmp[6] ;
    int JosluFocus_rc ;
    char waste[1] ;

    if (isSimulation())
    {
        return 0;
    }

    if(*value== MAXTRAVEL_READOUT) {

        strcpy(rf_cmd, "FL000000" ) ;

    } else {

        rf_cmd[0]=  'F' ;
        rf_cmd[1]=  'L' ;
        rf_cmd[2]=  '0' ;

        if(*value > 9999) {
            sprintf( vl_tmp, "%5d", (int) *value) ;

        } else if(*value > 999) {

            sprintf( vl_tmp, "0%4d", (int) *value) ;

        } else if(*value > 99) {
            sprintf( vl_tmp, "00%3d", (int) *value) ;

        } else if(*value > 9) {
            sprintf( vl_tmp, "000%2d", (int) *value) ;
        } else {
            sprintf( vl_tmp, "0000%1d", (int) *value) ;
        }
        rf_cmd[3]= vl_tmp[0] ;
        rf_cmd[4]= vl_tmp[1] ;
        rf_cmd[5]= vl_tmp[2] ;
        rf_cmd[6]= vl_tmp[3] ;
        rf_cmd[7]= vl_tmp[4] ;
        rf_cmd[8]= 0 ;
    }

    if ((JosluFocus_rc= SendCommand( rf_cmd)) < 0)
        return JosluFocus_rc;

    if ((JosluFocus_rc= ReadResponse(rf_cmd)) < 0)
            return JosluFocus_rc;

    if (sscanf(rf_cmd, "FL%1c%5f", waste, &temp) < 1)
        return -1;

    *value = (double) temp;

    DEBUGF(INDI::Logger::DBG_DEBUG, "Max position: %g", *value);

    return 0;
}

int JosluFocus::updateRFSetPosition(double *value)
{
    DEBUGF(INDI::Logger::DBG_DEBUG, "Set Max position: %g", *value);

    char rf_cmd[32] ;
    char vl_tmp[6] ;
    int JosluFocus_rc ;

    if (isSimulation())
    {
        simulatedPosition= *value;
        return 0;
    }

    rf_cmd[0]=  'F' ;
    rf_cmd[1]=  'S' ;
    rf_cmd[2]=  '0' ;

    if(*value > 9999) {
        sprintf( vl_tmp, "%5d", (int) *value) ;
    } else if(*value > 999) {
        sprintf( vl_tmp, "0%4d", (int) *value) ;
    } else if(*value > 99) {
        sprintf( vl_tmp, "00%3d", (int) *value) ;
    } else if(*value > 9) {
        sprintf( vl_tmp, "000%2d", (int) *value) ;
    } else {
        sprintf( vl_tmp, "0000%1d", (int) *value) ;
    }
    rf_cmd[3]= vl_tmp[0] ;
    rf_cmd[4]= vl_tmp[1] ;
    rf_cmd[5]= vl_tmp[2] ;
    rf_cmd[6]= vl_tmp[3] ;
    rf_cmd[7]= vl_tmp[4] ;
    rf_cmd[8]= 0 ;

    if ((JosluFocus_rc= SendCommand( rf_cmd)) < 0)
        return JosluFocus_rc;

    return 0;
}

bool JosluFocus::ISNewSwitch (const char *dev, const char *name, ISState *states, char *names[], int n)
{
    if(strcmp(dev,getDeviceName())==0)
    {
        if (!strcmp (name, PowerSwitchesSP.name))
        {
            int ret= -1 ;
            int nset= 0 ;
            int i= 0 ;
            int new_s= -1 ;
            int new_sn= -1 ;
            int cur_s1LL=0 ;
            int cur_s2LR=0 ;
            int cur_s3RL=0 ;
            int cur_s4RR=0 ;

            ISwitch *sp ;

            PowerSwitchesSP.s = IPS_BUSY ;
            IDSetSwitch(&PowerSwitchesSP, NULL) ;


            for( nset = i = 0; i < n; i++) {
                /* Find numbers with the passed names in the SettingsNP property */
                sp = IUFindSwitch (&PowerSwitchesSP, names[i]) ;

                /* If the state found is  (PowerSwitchesS[0]) then process it */

                if( sp == &PowerSwitchesS[0]){


                    new_s = (states[i]) ;
                    new_sn= 0;
                    nset++ ;
                } else if( sp == &PowerSwitchesS[1]){

                    new_s = (states[i]) ;
                    new_sn= 1;
                    nset++ ;
                } else if( sp == &PowerSwitchesS[2]){

                    new_s = (states[i]) ;
                    new_sn= 2;
                    nset++ ;
                } else if( sp == &PowerSwitchesS[3]){

                    new_s = (states[i]) ;
                    new_sn= 3;
                    nset++ ;
                }
            }
            if (nset == 1)
            {
                cur_s1LL= cur_s2LR= cur_s3RL= cur_s4RR= 0 ;

                if(( ret= updateRFPowerSwitches(new_s, new_sn, &cur_s1LL, &cur_s2LR, &cur_s3RL, &cur_s4RR)) < 0)
                {

                    PowerSwitchesSP.s = IPS_ALERT;
                    IDSetSwitch(&PowerSwitchesSP, "Unknown error while reading  JosluFocus power swicht settings");
                    return true;
                }
            }
            else
            {
                /* Set property state to idle */
                PowerSwitchesSP.s = IPS_IDLE ;

                IDSetNumber(&SettingsNP, "Power switch settings absent or bogus.");
                return true ;
            }

        }
    }


    return INDI::Focuser::ISNewSwitch(dev, name, states, names, n);
}

bool JosluFocus::ISNewNumber (const char *dev, const char *name, double values[], char *names[], int n)
{
    int nset=0,i=0;

    if(strcmp(dev,getDeviceName())==0)
    {

        if (!strcmp (name, SettingsNP.name))
        {
            /* new speed */
            double new_duty = 0 ;
            double new_delay = 0 ;
            double new_ticks = 0 ;
            int ret = -1 ;

            for (nset = i = 0; i < n; i++)
            {
                /* Find numbers with the passed names in the SettingsNP property */
                INumber *eqp = IUFindNumber (&SettingsNP, names[i]);

                /* If the number found is  (SettingsN[0]) then process it */
                if (eqp == &SettingsN[0])
                {

                    new_duty = (values[i]);
                    nset += new_duty >= 0 && new_duty <= 255;
                } else if  (eqp == &SettingsN[1])
                {

                    new_delay = (values[i]);
                    nset += new_delay >= 0 && new_delay <= 255;
                } else if  (eqp == &SettingsN[2])
                {

                    new_ticks = (values[i]);
                    nset += new_ticks >= 0 && new_ticks <= 255;
                }
            }

            /* Did we process the three numbers? */
            if (nset == 3)
            {

                /* Set the JosluFocus state to BUSY */
                SettingsNP.s = IPS_BUSY;


                IDSetNumber(&SettingsNP, NULL);

                if(( ret= updateRFMotorSettings(&new_duty, &new_delay, &new_ticks))< 0)
                {

                    IDSetNumber(&SettingsNP, "Changing to new settings failed");
                    return false;
                }

                currentDuty = new_duty ;
                currentDelay= new_delay ;
                currentTicks= new_ticks ;

                SettingsNP.s = IPS_OK;
                IDSetNumber(&SettingsNP, "Motor settings are now  %3.0f %3.0f %3.0f", currentDuty, currentDelay, currentTicks);
                return true;

            } else
            {
                /* Set property state to idle */
                SettingsNP.s = IPS_IDLE;

                IDSetNumber(&SettingsNP, "Settings absent or bogus.");
                return false ;
            }
        }

        if (!strcmp (name, SetBacklashNP.name))
        {

            double new_back = 0 ;
            int nset = 0;
            int ret= -1 ;

            for (nset = i = 0; i < n; i++)
            {
                /* Find numbers with the passed names in the SetBacklashNP property */
                INumber *eqp = IUFindNumber (&SetBacklashNP, names[i]);

                /* If the number found is SetBacklash (SetBacklashN[0]) then process it */
                if (eqp == &SetBacklashN[0]){

                    new_back = (values[i]);

                    /* limits */
                    nset += new_back >= -0xff && new_back <= 0xff;
                }

                if (nset == 1) {

                    /* Set the JosluFocus state to BUSY */
                    SetBacklashNP.s = IPS_BUSY;
                    IDSetNumber(&SetBacklashNP, NULL);

                    if(( ret= updateRFBacklash(&new_back)) < 0) {

                        SetBacklashNP.s = IPS_IDLE;
                        IDSetNumber(&SetBacklashNP, "Setting new backlash failed.");

                        return false ;
                    }

                    currentSetBacklash=  new_back ;
                    SetBacklashNP.s = IPS_OK;
                    IDSetNumber(&SetBacklashNP, "Backlash is now  %3.0f", currentSetBacklash) ;
                    return true;
                } else {

                    SetBacklashNP.s = IPS_IDLE;
                    IDSetNumber(&SetBacklashNP, "Need exactly one parameter.");

                    return false ;
                }
            }
        }

        if (!strcmp (name, MinMaxPositionNP.name))
        {
            /* new positions */
            double new_min = 0 ;
            double new_max = 0 ;

            for (nset = i = 0; i < n; i++)
            {
                /* Find numbers with the passed names in the MinMaxPositionNP property */
                INumber *mmpp = IUFindNumber (&MinMaxPositionNP, names[i]);

                /* If the number found is  (MinMaxPositionN[0]) then process it */
                if (mmpp == &MinMaxPositionN[0])
                {

                    new_min = (values[i]);
                    nset += new_min >= 1 && new_min <= 65000;
                } else if  (mmpp == &MinMaxPositionN[1])
                {

                    new_max = (values[i]);
                    nset += new_max >= 1 && new_max <= 65000;
                }
            }

            /* Did we process the two numbers? */
            if (nset == 2)
            {

                /* Set the JosluFocus state to BUSY */
                MinMaxPositionNP.s = IPS_BUSY;

                currentMinPosition = new_min ;
                currentMaxPosition= new_max ;


                MinMaxPositionNP.s = IPS_OK;
                IDSetNumber(&MinMaxPositionNP, "Minimum and Maximum settings are now  %3.0f %3.0f", currentMinPosition, currentMaxPosition);
                return true;

            } else {
                /* Set property state to idle */
                MinMaxPositionNP.s = IPS_IDLE;

                IDSetNumber(&MinMaxPositionNP, "Minimum and maximum limits absent or bogus.");

                return false;
            }
        }


        if (!strcmp (name, MaxTravelNP.name))
        {

            double new_maxt = 0 ;
            int ret = -1 ;

            for (nset = i = 0; i < n; i++)
            {
                /* Find numbers with the passed names in the MinMaxPositionNP property */
                INumber *mmpp = IUFindNumber (&MaxTravelNP, names[i]);

                /* If the number found is  (MaxTravelN[0]) then process it */
                if (mmpp == &MaxTravelN[0])
                {

                    new_maxt = (values[i]);
                    nset += new_maxt >= 1 && new_maxt <= 64000;
                }
            }
            /* Did we process the one number? */
            if (nset == 1) {

                IDSetNumber(&MinMaxPositionNP, NULL);

                if(( ret= updateRFMaxPosition(&new_maxt))< 0 )
                {
                    MaxTravelNP.s = IPS_IDLE;
                    IDSetNumber(&MaxTravelNP, "Changing to new maximum travel failed");
                    return false ;
                }

                currentMaxTravel=  new_maxt ;
                MaxTravelNP.s = IPS_OK;
                IDSetNumber(&MaxTravelNP, "Maximum travel is now  %3.0f", currentMaxTravel) ;
                return true;

            } else {
                /* Set property state to idle */

                MaxTravelNP.s = IPS_IDLE;
                IDSetNumber(&MaxTravelNP, "Maximum travel absent or bogus.");

                return false ;
            }
        }


        if (!strcmp (name, SetRegisterPositionNP.name))
        {

            double new_apos = 0 ;
            int nset = 0;
            int ret= -1 ;

            for (nset = i = 0; i < n; i++)
            {
                /* Find numbers with the passed names in the SetRegisterPositionNP property */
                INumber *srpp = IUFindNumber (&SetRegisterPositionNP, names[i]);

                /* If the number found is SetRegisterPosition (SetRegisterPositionN[0]) then process it */
                if (srpp == &SetRegisterPositionN[0])
                {

                    new_apos = (values[i]);

                    /* limits are absolute */
                    nset += new_apos >= 0 && new_apos <= 64000;
                }

                if (nset == 1)
                {

                    if((new_apos < currentMinPosition) || (new_apos > currentMaxPosition))
                    {

                        SetRegisterPositionNP.s = IPS_ALERT ;
                        IDSetNumber(&SetRegisterPositionNP, "Value out of limits  %5.0f", new_apos);
                        return false ;
                    }

                    /* Set the JosluFocus state to BUSY */
                    SetRegisterPositionNP.s = IPS_BUSY;
                    IDSetNumber(&SetRegisterPositionNP, NULL);

                    if(( ret= updateRFSetPosition(&new_apos)) < 0)
                    {

                        SetRegisterPositionNP.s = IPS_OK;
                        IDSetNumber(&SetRegisterPositionNP, "Read out of the set position to %3d failed. Trying to recover the position", ret);

                        if((ret= updateRFPosition( &currentPosition)) < 0)
                        {

                            FocusAbsPosNP.s = IPS_ALERT;
                            IDSetNumber(&FocusAbsPosNP, "Unknown error while reading  JosluFocus position: %d", ret);

                            SetRegisterPositionNP.s = IPS_IDLE;
                            IDSetNumber(&SetRegisterPositionNP, "Relative movement failed.");
                        }

                        SetRegisterPositionNP.s = IPS_OK;
                        IDSetNumber(&SetRegisterPositionNP, NULL);

                        FocusAbsPosNP.s = IPS_OK;
                        IDSetNumber(&FocusAbsPosNP, "JosluFocus position recovered %5.0f", currentPosition);
                        DEBUG(INDI::Logger::DBG_DEBUG, "JosluFocus position recovered resuming normal operation");
                        /* We have to leave here, because new_apos is not set */
                        return true ;
                    }
                    currentPosition= new_apos ;
                    SetRegisterPositionNP.s = IPS_OK;
                    IDSetNumber(&SetRegisterPositionNP, "JosluFocus register set to %5.0f", currentPosition);

                    FocusAbsPosNP.s = IPS_OK;
                    IDSetNumber(&FocusAbsPosNP, "JosluFocus position is now %5.0f", currentPosition);

                    return true ;

                } else
                {

                    SetRegisterPositionNP.s = IPS_IDLE;
                    IDSetNumber(&SetRegisterPositionNP, "Need exactly one parameter.");

                    return false;
                }

                if((ret= updateRFPosition(&currentPosition)) < 0)
                {

                    FocusAbsPosNP.s = IPS_ALERT;
                    DEBUGF(INDI::Logger::DBG_ERROR, "Unknown error while reading  JosluFocus position: %d", ret);
                    IDSetNumber(&FocusAbsPosNP, NULL);

                    return false ;
                }

                SetRegisterPositionNP.s = IPS_OK;
                SetRegisterPositionN[0].value = currentPosition;
                IDSetNumber(&SetRegisterPositionNP, "JosluFocus has accepted new register setting" ) ;

                FocusAbsPosNP.s = IPS_OK;
                DEBUGF(INDI::Logger::DBG_SESSION, "JosluFocus new position %5.0f", currentPosition);
                IDSetNumber(&FocusAbsPosNP, NULL);

                return true;
            }
        }
    }

    return INDI::Focuser::ISNewNumber(dev, name, values, names, n);

}

void JosluFocus::GetFocusParams ()
{
    int ret = -1 ;
    int cur_s1LL=0 ;
    int cur_s2LR=0 ;
    int cur_s3RL=0 ;
    int cur_s4RR=0 ;

    if((ret= updateRFPosition(&currentPosition)) < 0)
    {
        FocusAbsPosNP.s = IPS_ALERT;
        DEBUGF(INDI::Logger::DBG_ERROR, "Unknown error while reading  JosluFocus position: %d", ret);
        IDSetNumber(&FocusAbsPosNP, NULL);
        return;
    }

    FocusAbsPosNP.s = IPS_OK;
    IDSetNumber(&FocusAbsPosNP, NULL);

    if(( ret= updateRFTempHum(&currentTemperature,&currentHumidity)) < 0)
    {
        TempHumNP.s = IPS_ALERT;
        DEBUG(INDI::Logger::DBG_ERROR, "Unknown error while reading JosluFocus temperature.");
        IDSetNumber(&TempHumNP, NULL);
        return;
    }

    TempHumNP.s = IPS_OK;
    IDSetNumber(&TempHumNP, NULL);

    currentBacklash= BACKLASH_READOUT ;
    if(( ret= updateRFBacklash(&currentBacklash)) < 0)
    {
        SetBacklashNP.s = IPS_ALERT;
        DEBUG(INDI::Logger::DBG_ERROR, "Unknown error while reading JosluFocus backlash.");
        IDSetNumber(&SetBacklashNP, NULL);
        return;
    }
    SetBacklashNP.s = IPS_OK;
    IDSetNumber(&SetBacklashNP, NULL);

    currentDuty= currentDelay= currentTicks=0 ;

    if(( ret= updateRFMotorSettings(&currentDuty, &currentDelay, &currentTicks )) < 0)
    {
        SettingsNP.s = IPS_ALERT;
        DEBUG(INDI::Logger::DBG_ERROR, "Unknown error while reading JosluFocus motor settings.");
        IDSetNumber(&SettingsNP, NULL);
        return;
    }

    SettingsNP.s = IPS_OK;
    IDSetNumber(&SettingsNP, NULL);

    if(( ret= updateRFPowerSwitches(-1, -1,  &cur_s1LL, &cur_s2LR, &cur_s3RL, &cur_s4RR)) < 0)
    {
        PowerSwitchesSP.s = IPS_ALERT;
        DEBUG(INDI::Logger::DBG_ERROR, "Unknown error while reading JosluFocus power switch settings.");
        IDSetSwitch(&PowerSwitchesSP, NULL);
        return;
    }

    PowerSwitchesS[0].s= PowerSwitchesS[1].s= PowerSwitchesS[2].s= PowerSwitchesS[3].s= ISS_OFF ;

    if(cur_s1LL== ISS_ON)
    {

        PowerSwitchesS[0].s= ISS_ON ;
    }
    if(cur_s2LR== ISS_ON)
    {

        PowerSwitchesS[1].s= ISS_ON ;
    }
    if(cur_s3RL== ISS_ON)
    {

        PowerSwitchesS[2].s= ISS_ON ;
    }
    if(cur_s4RR== ISS_ON)
    {

        PowerSwitchesS[3].s= ISS_ON ;
    }
    PowerSwitchesSP.s = IPS_OK ;
    IDSetSwitch(&PowerSwitchesSP, NULL);


    currentMaxTravel= MAXTRAVEL_READOUT;
    if(( ret= updateRFMaxPosition(&currentMaxTravel)) < 0)
    {
        MaxTravelNP.s = IPS_ALERT;
        DEBUG(INDI::Logger::DBG_ERROR, "Unknown error while reading JosluFocus maximum travel");
        IDSetNumber(&MaxTravelNP, NULL);
        return;
    }
    MaxTravelNP.s = IPS_OK;
    IDSetNumber(&MaxTravelNP, NULL);

}

IPState JosluFocus::MoveAbsFocuser(uint32_t targetTicks)
{
    int ret= -1 ;
    targetPos = targetTicks;

    if (targetTicks < FocusAbsPosN[0].min || targetTicks > FocusAbsPosN[0].max)
    {
        DEBUG(INDI::Logger::DBG_DEBUG, "Error, requested position is out of range.");
        return IPS_ALERT;
    }

    if(( ret= updateRFPositionAbsolute(targetPos)) < 0)
    {

        DEBUGF(INDI::Logger::DBG_DEBUG, "Read out of the absolute movement failed %3d", ret);
        return IPS_ALERT;
    }

    RemoveTimer(timerID);
    timerID = SetTimer(250);
    return IPS_BUSY;
}

IPState JosluFocus::MoveRelFocuser(FocusDirection dir, uint32_t ticks)
{
    return MoveAbsFocuser(FocusAbsPosN[0].value + ((int)ticks * (dir == FOCUS_INWARD ? -1 : 1)));
}

bool JosluFocus::saveConfigItems(FILE *fp)
{
    IUSaveConfigNumber(fp, &SettingsNP);
    IUSaveConfigNumber(fp, &SetBacklashNP);

    return INDI::Focuser::saveConfigItems(fp);
}

void JosluFocus::TimerHit()
{
    if (isConnected() == false)
        return;

    double prevPos=currentPosition;
    double newPos=0;

    if (FocusAbsPosNP.s == IPS_OK || FocusAbsPosNP.s == IPS_IDLE)
    {
        int rc = updateRFPosition(&newPos);
        if (rc >= 0)
        {
            currentPosition = newPos;
            if (prevPos != currentPosition)
                IDSetNumber(&FocusAbsPosNP, NULL);
        }
    }
    else if (FocusAbsPosNP.s == IPS_BUSY)
    {
        float newPos=0;
        int nbytes_read=0;
        char rf_cmd[RF_MAX_CMD] = {0};


        //nbytes_read= ReadUntilComplete(rf_cmd, RF_TIMEOUT) ;

        nbytes_read= ReadResponse(rf_cmd);

        rf_cmd[ nbytes_read - 1] = 0 ;

        if (nbytes_read != 9 || (sscanf(rf_cmd, "FD0%5f", &newPos) <= 0))
        {
            DEBUGF(INDI::Logger::DBG_WARNING, "Bogus position: (%#02X %#02X %#02X %#02X %#02X %#02X %#02X %#02X %#02X) - Bytes read: %d", rf_cmd[0], rf_cmd[1], rf_cmd[2], rf_cmd[3],
                    rf_cmd[4], rf_cmd[5], rf_cmd[6], rf_cmd[7], rf_cmd[8], nbytes_read);
            timerID = SetTimer(POLLMS);
            return;
        }
        else if (nbytes_read < 0)
        {
            FocusAbsPosNP.s == IPS_ALERT;
            DEBUG(INDI::Logger::DBG_ERROR, "Read error! Reconnect and try again.");
            IDSetNumber(&FocusAbsPosNP, NULL);
            return;
        }

        currentPosition = newPos;

        if (currentPosition == targetPos)
        {
            FocusAbsPosNP.s = IPS_OK;

            if (FocusRelPosNP.s == IPS_BUSY)
            {
                FocusRelPosNP.s = IPS_OK;
                IDSetNumber(&FocusRelPosNP, NULL);
            }
        }

        IDSetNumber(&FocusAbsPosNP, NULL);
        if (FocusAbsPosNP.s == IPS_BUSY)
        {
            timerID = SetTimer(250);
            return;
        }
    }

    timerID = SetTimer(POLLMS);
}

bool JosluFocus::AbortFocuser()
{
    DEBUG(INDI::Logger::DBG_DEBUG, "Aborting focuser...");

    int nbytes_written;
    const char *buf = "\r";
    if (tty_write(PortFD, buf, strlen(buf), &nbytes_written) == TTY_OK)
        return true;
    else
        return false;
}




