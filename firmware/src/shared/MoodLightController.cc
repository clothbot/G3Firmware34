/*
 * Mood Light Controller
 *
 * Copyright Dec, 2011 by Jetty840
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */

#include "MoodLightController.hh"

#include "Configuration.hh"
#include "EepromMap.hh"
#include "Eeprom.hh"
#include "EepromDefaults.hh"
#include <util/delay.h>


volatile uint8_t		lastScriptPlayed = 0;
volatile enum moodLightStatus   lastStatus=MOOD_LIGHT_STATUS_NOT_SET;



MoodLightController::MoodLightController(Pin sda, Pin scl)
{
    eStopTriggered = false;
    lastScriptPlayed = 0;
    lastStatus = MOOD_LIGHT_STATUS_IDLE;

    _sda = sda;
    _scl = scl;
}


//Returns true if BlinkM found, otherwise false

bool MoodLightController::start()
{
    blinkM.init(_sda, _scl);

    //BlinkM's by default boot up running a script, stop it
    blinkM.stopScript();

#ifdef HAS_MOOD_LIGHT
    playScript(eeprom::getEeprom8(eeprom::MOOD_LIGHT_SCRIPT, EEPROM_DEFAULT_MOOD_LIGHT_SCRIPT));
#endif

    return blinkM.blinkMIsPresent;
}


//There has to be a better way, however PROGMEM precludes it

const static unsigned char PROGMEM script0 [] = "Bot Status";
const static unsigned char PROGMEM script1 [] = "Custom RGB";
const static unsigned char PROGMEM script2 [] = "Off";
const static unsigned char PROGMEM script3 [] = "White";
const static unsigned char PROGMEM script4 [] = "Red";
const static unsigned char PROGMEM script5 [] = "Green";
const static unsigned char PROGMEM script6 [] = "Blue";
const static unsigned char PROGMEM script7 [] = "Cyan";
const static unsigned char PROGMEM script8 [] = "Magenta";
const static unsigned char PROGMEM script9 [] = "Yellow";
const static unsigned char PROGMEM script10[] = "Purple";
const static unsigned char PROGMEM script11[] = "Orange";
const static unsigned char PROGMEM script12[] = "Almond";
const static unsigned char PROGMEM script13[] = "Peach Puff";
const static unsigned char PROGMEM script14[] = "Mint Cream";
const static unsigned char PROGMEM script15[] = "Alice Blue";
const static unsigned char PROGMEM script16[] = "Lavender";
const static unsigned char PROGMEM script17[] = "Misty Rose";
const static unsigned char PROGMEM script18[] = "Slate Gray";
const static unsigned char PROGMEM script19[] = "Gray";
const static unsigned char PROGMEM script20[] = "Light Gray";
const static unsigned char PROGMEM script21[] = "MidnghtBlu";
const static unsigned char PROGMEM script22[] = "DeepSkyBlu";
const static unsigned char PROGMEM script23[] = "OliveGreen";
const static unsigned char PROGMEM script24[] = "ForstGreen";
const static unsigned char PROGMEM script25[] = "Gold";
const static unsigned char PROGMEM script26[] = "Hot Pink";
const static unsigned char PROGMEM script27[] = "Linen";
const static unsigned char PROGMEM script100[] = "WRGB";
const static unsigned char PROGMEM script101[] = "RGB";
const static unsigned char PROGMEM script102[] = "WhiteFlash";
const static unsigned char PROGMEM script103[] = "Red Flash";
const static unsigned char PROGMEM script104[] = "GreenFlash";
const static unsigned char PROGMEM script105[] = "Blue Flash";
const static unsigned char PROGMEM script106[] = "Cyan Flash";
const static unsigned char PROGMEM script107[] = "MgntaFlash";
const static unsigned char PROGMEM script108[] = "Yell Flash";
const static unsigned char PROGMEM script109[] = "Black";
const static unsigned char PROGMEM script110[] = "Hue Cycle";
const static unsigned char PROGMEM script111[] = "Mood Light";
const static unsigned char PROGMEM script112[] = "Candle";
const static unsigned char PROGMEM script113[] = "Water";
const static unsigned char PROGMEM script114[] = "Old Neon";
const static unsigned char PROGMEM script115[] = "Seasons";
const static unsigned char PROGMEM script116[] = "Thundrstrm";
const static unsigned char PROGMEM script117[] = "Stop Light";
const static unsigned char PROGMEM script118[] = "S. O. S.";
const static unsigned char PROGMEM script_unknown[] = "UNKNOWN";

//There has to be a better way, however PROGMEM precludes it

const unsigned char PROGMEM *MoodLightController::scriptIdToStr(uint8_t scriptId)
{
    switch(scriptId)
    {
    case 0:
        return script0;
    case 1:
        return script1;
    case 2:
        return script2;
    case 3:
        return script3;
    case 4:
        return script4;
    case 5:
        return script5;
    case 6:
        return script6;
    case 7:
        return script7;
    case 8:
        return script8;
    case 9:
        return script9;
    case 10:
        return script10;
    case 11:
        return script11;
    case 12:
        return script12;
    case 13:
        return script13;
    case 14:
        return script14;
    case 15:
        return script15;
    case 16:
        return script16;
    case 17:
        return script17;
    case 18:
        return script18;
    case 19:
        return script19;
    case 20:
        return script20;
    case 21:
        return script21;
    case 22:
        return script22;
    case 23:
        return script23;
    case 24:
        return script24;
    case 25:
        return script25;
    case 26:
        return script26;
    case 27:
        return script27;
    case 100:
        return script100;
    case 101:
        return script101;
    case 102:
        return script102;
    case 103:
        return script103;
    case 104:
        return script104;
    case 105:
        return script105;
    case 106:
        return script106;
    case 107:
        return script107;
    case 108:
        return script108;
    case 109:
        return script109;
    case 110:
        return script110;
    case 111:
        return script111;
    case 112:
        return script112;
    case 113:
        return script113;
    case 114:
        return script114;
    case 115:
        return script115;
    case 116:
        return script116;
    case 117:
        return script117;
    case 118:
        return script118;
    default:
        return script_unknown;
    }
}


#define kBlinkMStart 100
#define kBlinkMEnd   118

#define kDefinedColors 26
typedef uint8_t ColorLookup[kDefinedColors][3];

const ColorLookup clut PROGMEM =
{
    {0,   0,   0  }, //Off
    {255, 255, 255}, //White
    {255, 0,   0  }, //Red
    {0,   255, 0  }, //Green
    {0,   0,   255}, //Blue
    {0,   255, 255}, //Cyan
    {255, 0,   255}, //Magenta
    {255, 255, 0  }, //Yellow
    {160, 32,  240}, //Purple
    {255, 165, 0  }, //Orange
    {255, 195, 165}, //Almond
    {255, 178, 145}, //Peach Puff
    {205, 255, 210}, //Mint Cream
    {210, 208, 255}, //Alice Blue
    {180, 180, 255}, //Lavender
    {255, 208, 205}, //Misty Rose
    {112, 138, 144}, //Slate Gray
    {190, 190, 190}, //Gray
    {211, 211, 211}, //Light Gray
    {25,  25,  112}, //Midnight Blue
    {0,   191, 255}, //Deep Sky Blue
    {85,  137, 47 }, //Olive Green
    {34,  139, 34 }, //Forest Green
    {255, 215, 0  }, //Gold
    {255, 105, 180}, //Hot Pink
    {240, 240, 230}  //Linen
};



//Given a scriptId, return the next scriptId
//There has to be a better way, however PROGMEM precludes it

uint8_t MoodLightController::nextScriptId(uint8_t currentScriptId)
{
    if ( currentScriptId <= kDefinedColors )	return currentScriptId + 1;
    if ( currentScriptId == (kDefinedColors + 1) )	return kBlinkMStart;
    if (( currentScriptId >= kBlinkMStart ) && ( currentScriptId < kBlinkMEnd))	return currentScriptId + 1;
    return 0;
}


//Given a scriptId, return the previous scriptId
//There has to be a better way, however PROGMEM precludes it

uint8_t MoodLightController::prevScriptId(uint8_t currentScriptId)
{
    if (( currentScriptId <= (kDefinedColors + 1) ) && ( currentScriptId >= 1 ))	return currentScriptId - 1;
    if ( currentScriptId == kBlinkMStart )	return kDefinedColors + 1;
    if (( currentScriptId > kBlinkMStart ) && ( currentScriptId <= kBlinkMEnd))	return currentScriptId - 1;
    return kBlinkMEnd;
}


void MoodLightController::playScript(uint8_t scriptId)
{
    lastScriptPlayed = scriptId;

    blinkM.stopScript();

    blinkM.setFadeSpeed(8);
    blinkM.setTimeAdjust(50);

    if      ( scriptId == 0 )  						//Bot Status
    {
        blinkM.fadeToRGB(255, 255, 255);
    }
    else if ( scriptId == 1 )  						//Custom RGB (User Defined)
    {
#ifdef HAS_MOOD_LIGHT
        blinkM.fadeToRGB(eeprom::getEeprom8(eeprom::MOOD_LIGHT_CUSTOM_RED,	EEPROM_DEFAULT_MOOD_LIGHT_CUSTOM_RED),
                         eeprom::getEeprom8(eeprom::MOOD_LIGHT_CUSTOM_GREEN,	EEPROM_DEFAULT_MOOD_LIGHT_CUSTOM_GREEN),
                         eeprom::getEeprom8(eeprom::MOOD_LIGHT_CUSTOM_BLUE,	EEPROM_DEFAULT_MOOD_LIGHT_CUSTOM_BLUE));
#endif
    }
    else if (( scriptId >= 2 ) && ( scriptId < (2 + kDefinedColors )))  	//Regular solid colors
    {
        uint8_t r, g, b;

        //Copy from PROGMEM
        memcpy_P(&r, (const void*)&(clut[scriptId-2][0]), sizeof(uint8_t));
        memcpy_P(&g, (const void*)&(clut[scriptId-2][1]), sizeof(uint8_t));
        memcpy_P(&b, (const void*)&(clut[scriptId-2][2]), sizeof(uint8_t));

        blinkM.fadeToRGB(r, g, b);
    }

    else if (( scriptId >= kBlinkMStart ) && (scriptId <= kBlinkMEnd ))	//BlinkM Predefined Scripts
        blinkM.playScript(scriptId - (uint8_t)100, 0);
    else	playScript(3);							//Default to white for an unknown color
}


//Should only be used with ERR_ESTOP as most other errors occur to frequently
//Only displays when the script playing is "Bot Status", i.e. 0

void MoodLightController::debugLightSetValue(bool value)
{
#ifdef HAS_MOOD_LIGHT
    if ( eeprom::getEeprom8(eeprom::MOOD_LIGHT_SCRIPT, EEPROM_DEFAULT_MOOD_LIGHT_SCRIPT) == (uint8_t) 0 )
    {
        eStopTriggered = true;
        if ( value )	blinkM.setRGB(255, 0, 0);
        else		blinkM.setRGB(0, 0, 0);
    }
#endif
}


void MoodLightController::displayStatus(enum moodLightStatus status, float percentHot)
{
    if ( status == lastStatus )
    {
        if (( status != MOOD_LIGHT_STATUS_HEATING ) &&
                ( status != MOOD_LIGHT_STATUS_COOLING )) return;
    }

    if ( lastScriptPlayed == (uint8_t)0 )  	//Bot Status Script
    {
        if ( eStopTriggered )	status = MOOD_LIGHT_STATUS_ERROR;

        uint8_t tempColor = (uint8_t)(percentHot/100.0 * 255.0);

        switch(status)
        {
        case MOOD_LIGHT_STATUS_IDLE:
        case MOOD_LIGHT_STATUS_PRINTING:
            blinkM.fadeToRGB(255, 255, 255);
            break;
        case MOOD_LIGHT_STATUS_HEATING:
        case MOOD_LIGHT_STATUS_COOLING:
            blinkM.fadeToRGB(tempColor, 0, 255 - tempColor);
            break;

        case MOOD_LIGHT_STATUS_ERROR:
            //Error condition.  This is being handled by
            //debugLightSetValue, so we do nothing.
            break;
        }
    }

    lastStatus = status;
}


enum moodLightStatus MoodLightController::getLastStatus()
{
    return lastStatus;
}

uint8_t MoodLightController::getLastScriptPlayed()
{
    return lastScriptPlayed;
}
