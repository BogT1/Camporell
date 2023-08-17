/*
*   ---Weather Station WH24C (OTTA version)---
*   
*   Dev board used: Heltec CubeCell HTCC-AB01 (V2).
*/
/*******************************************************************************************
 * @file      main.cpp
 * @author    Bogdan Turcu
 * @version   v1.0
 * @date      13/06/2023
 * @brief     Programa principal.
 * *****************************************************************************************
 * 
 *  TODO:   Deixar el dispositiu funcionant durant molt temps per mirar si en algun moment
 *          l'estació llegeix malament o tot zeros. O si hi ha algun CRC error.
 *          Si resulta algun error anterior mirar d'arreglar-ho
 * 
 * @details Actualment no s'ha pujat el codi en cap dispositiu vendut (26-06-2023).
 *          Si no troba l'estació s'adorm i no envia res.
 *          Si no pilla conexió amb el gateway no es queda penjat. Això juntament amb el punt
 *          anterior permet no tenir que ficar botó d'encendido a l'estació.
 * 
 *          S'ha de comprovar el consum del dispositiu. Mirar quant temps dura sense carrega
 *          solar quant l'estació no esta conectada.
 * 
 */


/* Includes ------------------------------------------------------------------------------ */
#include <LoRaWan_APP.h>
#include <Arduino.h>
#include "SoftSerial.h"
#include "hexToBinary.h"
#include "Utils.h"
#include "WH24CP.h"
#include "EEPROM.h"

/* Defines ------------------------------------------------------------------------------- */
#define TEMPS_ENVIAMENT_MILISEGONS                          (1000*(4*16))               // (Temps d'enviament per defecte)
#define ADDRESS_SEND_TIME                                   0
#define TEMPS_BUSQUEDA_ESTACIO                              16000                       // Temps en que el dispositiu buscara l'estació abans d'anar a dormir


/* Variables LoRaWAN --------------------------------------------------------------------- */

/* OTAA para*/
uint8_t devEui[8];
uint8_t appEui[8];
uint8_t appKey[16];

/* ABP para*/
uint8_t nwkSKey[16];
uint8_t appSKey[16];
uint32_t devAddr =  ( uint32_t )0x007e6ae1;

/*LoraWan channelsmask, default channels 0-7*/ 
uint16_t userChannelsMask[6]={ 0x00FF,0x0000,0x0000,0x0000,0x0000,0x0000 };

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t  loraWanClass = LORAWAN_CLASS;

/*the application data transmission duty cycle.  value in [ms].*/
uint32_t appTxDutyCycle = TEMPS_ENVIAMENT_MILISEGONS;

/*OTAA or ABP*/
bool overTheAirActivation = LORAWAN_NETMODE;

/*ADR enable*/
bool loraWanAdr = LORAWAN_ADR;

/* set LORAWAN_Net_Reserve ON, the node could save the network info to flash, when node reset not need to join again */
bool keepNet = LORAWAN_NET_RESERVE;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = LORAWAN_UPLINKMODE;

/* Application port */
uint8_t appPort = 2;

uint8_t confirmedNbTrials = 4;


/* Variables compostes ----------------------------------------------------------------------------- */
union u64_bytes {
    uint64_t u64;
    uint8_t u8[sizeof(uint64_t)];
};

typedef enum 
{
    RESPOSTA_OK = 0,
    RESPOSTA_MAL = 1,
    ESTACIO_NO_TROBADA =2
} RespostaEstacioTypeDef;

RespostaEstacioTypeDef Resposta_Estacio = RESPOSTA_OK;

/*Variables-----------------------------------------------------------------------------------------*/
uint8_t data_buf[17];
__IO bool sendData = false;
uint8_t stationResponse[18] = {0};


/*Classes-------------------------------------------------------------------------------------------*/

/* Important:
    - En el cas de la comunicació amb l'estació initialitzem com a Tx el GPIO5, quan el verdades Tx es el GPIO1. Això perque
      si inicialitzem el GPIO1 com Tx, no podem ficar Vext a HIGH degut a alguna interrupció. Com no hem de transmetre res a la
      estació i unicament esperam que ens envi coses es indiferent el pin Tx que inicialitzem ja que no l'utilitzem. */
softSerial Station(GPIO5, GPIO2); // Tx y Rx respectivly

WH24CP wh24c;

/*Functions declarations----------------------------------------------------------------------------*/
void generateDeveuiByChipID( void );
void downLinkDataHandle(McpsIndication_t *mcpsIndication);
void lectura_estacio( void );

void eeprom_write(uint32_t t_ms);
uint32_t eeprom_read( void );

uint32_t hexToInt(String str);

/* Prepares the payload of the frame */
static void prepareTxFrame(uint8_t port) 
{
    sendData = true;
    appDataSize = wh24c.getOutput(appData);
}



/* Main code ------------------------------------------------------------------ */

void setup() {

	Serial.begin(115200);
    EEPROM.begin(512);

	enableAt();

	Station.begin(9600);

	pinMode(GPIO3, OUTPUT);
	pinMode(Vext, OUTPUT);
    digitalWrite(Vext, HIGH);

    eeprom_write( 0 );


	deviceState = DEVICE_STATE_INIT;
	LoRaWAN.ifskipjoin();
}

void loop()
{
	
	switch( deviceState )
	{
		case DEVICE_STATE_INIT:
		{	
			generateDeveuiByChipID();
			//LoRaWAN.generateDeveuiByChipID();
			
			getDevParam();

			printDevParam();
			LoRaWAN.init(loraWanClass,loraWanRegion);
			deviceState = DEVICE_STATE_JOIN;
			break;
		}
		case DEVICE_STATE_JOIN:
		{
			LoRaWAN.join();
			break;
		}
		case DEVICE_STATE_SEND:
		{
            /* Llegim l'estació */
            lectura_estacio();

            if (Resposta_Estacio == RESPOSTA_OK)
            {
                /* Llegim el TxDutyCycle guardat a la EEPROM */
                appTxDutyCycle = eeprom_read();
                prepareTxFrame( appPort );
			    LoRaWAN.send(); 
                
            }
            else if (Resposta_Estacio == ESTACIO_NO_TROBADA)    // En el cas de que no trobem l'estacio
            {
                /* Si no trobem l'estació preparem el valor de la bateria */
                uint16_t v_bateria = getBatteryVoltage();
                appDataSize = 2;
                appData[0] = (v_bateria >> 8) & 0xFF;
                appData[1] = v_bateria & 0xFF;

                /* Enviem el valor de la bateria unicament ja que no hem trobat l'estació. */
                LoRaWAN.send();

                /* Ja que no hem trobat l'estació, fem que s'adormi 20 min */
                // Schedule next packet transmission
                appTxDutyCycle = 1200000;   // 1200000 ms = 20 min
                Serial.print("");
            }
			
            /* Reasignem el valor per defecte */
            Resposta_Estacio = RESPOSTA_OK;

			deviceState = DEVICE_STATE_CYCLE;
			break;
		}
		case DEVICE_STATE_CYCLE:
		{
            Serial.print("AppTxDutyCycle: ");Serial.println(appTxDutyCycle); delay(50);
			txDutyCycleTime = appTxDutyCycle + randr( 0, APP_TX_DUTYCYCLE_RND );
			LoRaWAN.cycle(txDutyCycleTime);

			deviceState = DEVICE_STATE_SLEEP;
			break;
		}
		case DEVICE_STATE_SLEEP:
		{
            /* Dispositiu s'en va a dormir */
			LoRaWAN.sleep();
			break;
		}
		default:
		{
			deviceState = DEVICE_STATE_INIT;
			break;
		}
	}	
}




/* Functions declarations ------------------------------------------------------------------------------ */

/**
 * @brief   Funció que s'encarrega de llegir l'informació que arriba de l'estació.
 * @param   None.
 * @retval  None.
 * */
void lectura_estacio( void )
{
    digitalWrite(Vext, LOW);
    delay(200);

    // Netejem el buffer de la estació
    Station.flush();


    bool cnv = true;
    uint32_t t_inicial = millis();
    while (Station.available() == 0)
    {
        if (cnv)
        {
            Serial.print("Buscant l'estacio.");
            cnv = false;
        }
        else
        {
            Serial.print(".");
        }

        if (millis() - t_inicial > TEMPS_BUSQUEDA_ESTACIO)
        {
            Serial.print("\nNO s'ha trobat l'estacio!\n");
            digitalWrite(Vext, HIGH);
            delay(100);

            Resposta_Estacio = ESTACIO_NO_TROBADA;

            return;
        }

        delay(500);
    }

    // Luego ya podemos leer
    if (Station.available() > 15) {

        // Reading the 17 bytes that the station send. The station send it automatically every 16 seconds.
        for (int i = 0; i < 17; i++) {
            stationResponse[i] = (uint8_t)Station.read();
            // Serial.print(stationResponse[i], HEX);
        }
        stationResponse[18] = '\0';

        // CRC or cyclic redundancy check in order to know if the data received is not corrupted
        if (gencrc(stationResponse, 15) != stationResponse[15]) {
            Serial.printf("- PAYLOAD 1: CRC Error: CRC calculated is %02X and CRC read is %02X\n", gencrc(stationResponse, 15), stationResponse[15]);

            /* Si tenim algun error amb el CRC, tornem a llegir l'estacio. */
            lectura_estacio();
        }
    }

    // Passing the station response in order to process it (17 bytes)
    wh24c.init(stationResponse, 17);

    // Get the battery of the device
    wh24c.addBattery(getBatteryVoltage() / 10);
    // The wind speed of the actual reading
    wh24c.setWindSpeed(wh24c.getWindSpeed());
    // The maximum velocity of the wind of the actual reading
    wh24c.setWindBurst(wh24c.getWindBurst());

    Serial.printf("\nData readed fron WH24C:\n\n%s\n", wh24c.humanReadable().c_str());
    delay(100);
    digitalWrite(Vext, HIGH);
    delay(100);
}



/**
 * @brief   Funció que genera el devEui, appEui, appKey i nwkSKey sengons el ID del chip. Aquests parametres es guardel en els vectord devEui, appEui, appKey, nwSKey i appSKey.
 * @param   None.
 * @retval  None.
 * */
void generateDeveuiByChipID() {
    Serial.print(F("Device ID: vk-"));
    Serial.println(((uint32_t)getID()) & 0xFFFFFF);

    u64_bytes idNumber;
    idNumber.u64 = (__builtin_bswap64(getID()));

    devEui[0] = 'V';
    appEui[0] = 'V'; 

    devEui[1] = 'K';
    appEui[1] = 'K';

    for (int i = 2; i < (sizeof(uint64_t)); i++) {
        devEui[i] = (uint8_t)idNumber.u8[i];
        appEui[9 - i] = (uint8_t)((idNumber.u8[i]) * 2);
    }

    for (int i = 0; i < 8; i++) {
        appKey[i] = (uint8_t)((devEui[i] * 2) + appEui[8 - i]);
        appKey[i + 8] = (uint8_t)(devEui[(i + 8) - 8] + (appEui[16 - (i + 8)] * 2));
        nwkSKey[i] = (uint8_t)((devEui[i] * 3) + appEui[8 - i]);
        nwkSKey[i + 8] = (uint8_t)(devEui[(i + 8) - 8] + (appEui[16 - (i + 8)] * 3));
        appSKey[i] = (uint8_t)((devEui[i] * 2) - appEui[8 - i]);
        appSKey[i + 8] = (uint8_t)(devEui[(i + 8) - 8] - (appEui[16 - (i + 8)] * 2));
    }
}


/**
 * @brief   Funció que s'encarrega de manejar les comandes que li arriba al dispositiu des del gateway.
 * @param   mcpsIndication  Manejador del downlink comunication.
 * @retval  None.
 * */
void downLinkDataHandle(McpsIndication_t *mcpsIndication) {
    Serial.printf("+REV DATA:%s,RXSIZE %d,PORT %d\r\n", mcpsIndication->RxSlot ? "RXWIN2" : "RXWIN1",
                  mcpsIndication->BufferSize, mcpsIndication->Port);
    Serial.print("+REV DATA:");
    for (uint8_t i = 0; i < mcpsIndication->BufferSize; i++) {
        Serial.printf("%02X", mcpsIndication->Buffer[i]);
    }
    Serial.println();
    switch (mcpsIndication->Buffer[0]) {
        case 0x01:
            if (mcpsIndication->BufferSize == 4) {
                String hexstring;
                for (uint8_t i = 1; i < 4; i++) {
                    hexstring += String(mcpsIndication->Buffer[i], HEX);
                }
                uint32_t value = hexToInt(hexstring) * 1000;
                eeprom_write(value);
                delay(100);
            }
            break;
        case 0xFF:
            delay(100);

            /* Resetegem el dispositiu */
            HW_Reset(0);

            break;
        default:
            Serial.printf("Received Unknown Command 0x%02X\n", (int)(mcpsIndication->Buffer[0]));
    }
}

/**
 * @brief   Funcio que escriu un uint32_t en la memoria del dispositiu.
 * @param   t_ms    Es la variable que ha de ser guardada en la memoria FLASH.
 * @retval  None.
 * */
void eeprom_write(uint32_t t_ms)
{
    Serial.print("\ntemps appTxDutyCycle:");
    Serial.println(t_ms);
    delay(50);

    /* Guardem en la EEPROM el temps en ms */
    EEPROM.write(ADDRESS_SEND_TIME    , t_ms >> 24);
    EEPROM.write(ADDRESS_SEND_TIME + 1, t_ms >> 16);
    EEPROM.write(ADDRESS_SEND_TIME + 2, t_ms >> 8 );
    EEPROM.write(ADDRESS_SEND_TIME + 3, t_ms);

    if (EEPROM.commit())
    {
        Serial.println("EEPROM successfully committed");
    }
    else
    {
        Serial.println("ERROR! EEPROM commit failed");
    }

    delay(100);

}

/**
 * @brief   Llegim el temps d'enviament que hi ha guardat en la EEPROM. Si aquest temps es menor que 29000 ms
 *          per defecte asignarem un temps d'enviament de 63000 ms.
 * @param   None.
 * @retval  t_ms    El temps d'enviament en ms.
 * */
uint32_t eeprom_read( void )
{
    uint32_t t_ms = 0;
    t_ms = EEPROM.read(ADDRESS_SEND_TIME) << 24     | \
           EEPROM.read(ADDRESS_SEND_TIME + 1) << 16 | \
           EEPROM.read(ADDRESS_SEND_TIME + 2) <<  8 | \
           EEPROM.read(ADDRESS_SEND_TIME + 3) ;

    /* Si no hi ha cap valor en la eeprom es mes petit que 29 segons, fem que el temp de envio sigui  */
    if (t_ms < (TEMPS_ENVIAMENT_MILISEGONS/2))
    {
        t_ms = TEMPS_ENVIAMENT_MILISEGONS;
    }

    return t_ms;
}


/**
 * @brief   Funció que passa una cadena de caracters a enters.
 * */
uint32_t hexToInt(String str) {
    uint32_t i;
    sscanf(str.c_str(), "%X", &i);
    return i;
}

/*TODO: 
    1.- Fer que si no detecta lestació que s'adormi 5 min el dispositiu i despres tornar a intentar llegir.
    2.- Fer que si llegeix malament l'estació que la torni a llegir.
    3.- Mirar lo del CRC error. Si tenim una lectura mal que fa? Ja que s'ha borrat el goto:a_dromir
    4.- Mirar que mes fa falta.
*/  



