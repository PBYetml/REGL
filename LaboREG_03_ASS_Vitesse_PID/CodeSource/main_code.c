//----------------------------------------------------------------------------//
// Nom du projet 		:	Reglage Encodeur
// Nom du fichier 		:   main_code.c
// Date de création 	:   04.12.2019
// Date de modification : 	25.01.2022
// 
// Auteur 				: 	Philou (Ph. Bovey) 
//                      :   Michel Bonzon 
//                      :   Kevin Bougnon 
//
// Description 			: 	Canevas pour le laboratoire de réglage avec la carte
//                          1601_HalfBridge 
//                         
// Remarques			: 
// 	    chemin pour trouver le headerfile 
//		C:\Program Files\Microchip\MPLAB C30\support\dsPIC33F\h
//
//      Hardware de la carte => ATTENTION PIC MONTER SUR LA CARTE DSPIC33FJ128MC802
//      K:\ES\PROJETS\SLO\1601x_HalfBridge3x\doc 
//
//	    doc pour le DSP : 
//		K:\ES\PROJETS\SLO\1601x_HalfBridge3x\doc\datasheets
//
//      doc pour la configuration du PWM : 
//      https://ww1.microchip.com/downloads/en/DeviceDoc/70187E.pdf
//
//      doc pour la configuration du module de quadrature 
//      https://ww1.microchip.com/downloads/en/DeviceDoc/70208C.pdf
//
//----------------------------------------------------------------------------//

//--- librairie à inclure ---// 
#if defined(__dsPIC33F__)
#include "p33Fxxxx.h"
#endif

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <xc.h>                             // appel de directive du compilateur C pour le Dspic 
#include "math.h"

#include "configuration_bitsSettings.h"     // configuration des bits settings pour l'oscillateur 
#include "configOscillateur.h"              // configuration de l'oscillateur interne du Dspic
#include "configTimer.h"                    // configuration des timers
#include "ModuleConfiPWM.h"                 // configuration PWM 

#define NBR_PAS_MAX_ECHANTILLONER 1024

//------------------------------------------------------------------------------
//-- variables globales utilisées dans plusieurs fichiers 
//-- !!! A NE PAS TOUCHER - SVP => ligne 52 à 93 --// 
//-- initialisation des variables globale à zéro --// 
int8_t flagInterrupt = 0;                   

//-- variable représentant des variations de temps (timing) -- 
long temps_1ms = 0;         // initialiser à zéro pour le début du programmme
long tempPID = 0;           // variable d'échantillonage (temps) => pour déterminer le saut réel 
long TempsautConsigne = 0;  // variable d'échantillonage (temps) => pour déterminer la consigne  

long periodesaut = 50000;      // => représnet un saut de 5s : 5s / 100us = 50000 => constante lié au Timer1 
long periodePID = 10;          // => le PWM est régler sur une période de 100us + l'échantillonage a une période de 1ms 
                               // => T_echantillo / T_PWM = 1ms / 100us = 10
                                
//-- variables utilisée pour lire l'encodeur en quadrature (4 flancs) //
long retenu = 0;
long posk = 0;          // position actuelle 
long posk_1 = 0;        // position passée

//-- variable pour représenter la notion de vitesse (tours / seconde) --//
float vitessek;         // représentant le calcul de la vitesse actuelle
float vitessek_1;       // représentant le calcul de la vitesse passée 

//-- variables pour représenter l'erreur entre la consigne et la position  
float erreurk;          // représentant l'erreur actuel entre la position et la consigne 
float erreurk_1;        // représentant l'erreur passée entre la position et la consigne 

//-- variable pour réprésenter soit la consigne position ou en vitesse --// 
float consigneV;        // réprésente la consigne en tour / seconde 
float consigneP;        // représente la consigne en nombre de pas 
float saut = 262.144;   // représente le saut en nombre de micropas =>  
                        // 512*128*4 / 1ms = 262.144  => représente le nombre upas pour 1 tours  

//-- variable pour gérer les PWM de la carte DSPIC --// 
float pourcentPWM1 = 100, pourcentPWM2 = 0;     // rapport cyclique 

//-- variable liée aux différents correcteur PID --// 
float Up_k = 0;        // facteur proportionnelle; 
float Ui_k = 0;        // facteur intégrateur => actuel 
float Ui_k_1 = 0;      // facteur intégrateur => passé 
float Ud_k = 0;        // facteur dérivateur => actuel 
float Ud_k_1 = 0; ;    // facteur dérivateur => passé  
float Uk;              // facteur globale

//-- Prototypes -- 
void CalculerVitesse(void);             // en fonction de la position permet de déterminer la vitesse 
void CalculerErreur(void);              // en fonction de la consigne actuelle et la vitesse actuelle => détermine l'erreur 
void RaffraichirPWM(void);              // en fonction de Uk (facteur de correction) => mise à jour des PWM moteur 
void MAJVariables(void);                // une fois le cycle de correction passé => mise à jours de valeurs actuelles à passées 
void LectureImagePosition(void);        // lecture de l'encodeur A/B => système en quadradrture => 4 flanc => lecture micropas 
void CalculerPID(void);                 // en fonction des constantes Kp / Ki / Kd, va mettre à jour le facteur Uk 
void CalculerConsigne(void);            // représente la gestion des sauts sous forme de machine d'état 
//------------------------------------------------------------------------------

//----------- VARIABLES ou CONSTANTES AUTORISES A ETRE MODIFIER ---------------- 
//----------------------  CONSTANTES PID ---------------------------------------
float Kp = 2;       // Kp (proportionnel)  
float Ki = 0;       // Ki (intégrateur)
float Kd = 0;       // Kd (dérivateur)

//---------------------- FONCTION A MODIFIER  ---------------------------------- 
//----------------------------------------------------------------------------//
//-- nom 				: CalculerPID
//-- entrée - sortie 	: - / - 
//-- description 		: Permet de metre en place les différents correcteur 
//                        P / I / D 
//-- remarque 			: 
//-- modification       : 25.01.2022
//----------------------------------------------------------------------------//
void CalculerPID(void) 
{
    //-- proportionnel --//
    Up_k = Kp * erreurk; 
    
    //-- intégrateur --// 
    //Ui_k = Ki * erreurk + Ui_k_1; 
    
    //-- dérivateur --// 
    //Ud_k = Kd * (erreurk - erreurk_1); 
            
    //-- final --// 
    Uk = Up_k + Ui_k + Ud_k; 
} 

//----------------------------------------------------------------------------//
//-- nom 				: CalculerConsigne
//-- entrée - sortie 	: - / - 
//-- description 		: Machine d'état définissant la gestion de saut sur 5s
//-- remarque 			: 
//-- modification       : 28.01.2021
//----------------------------------------------------------------------------//
void CalculerConsigne(void)
{
    //-- déclaration de variables --//  
    static int states = 1;            // gère différents état 
    
    //-- état 1 => mettre la consigne à jour avec saut => 1 tour 
    if(states == 1)
    {
        consigneV = saut;    // la consigne correspond à 1 tours/s 
    }
    //-- état 2 => mettre la consigne à jour avec saut => moteur à l'arret 
    else if(states == 2)
    {
       consigneV = 0;       // la consigne correspond au moteur bloqué 
       states = 0;
    }
    else if(states == 3)
    {
        states = 0;
    }
    
    //-- Mise à jour de l'état --// 
    states++; 
}
//------------------------------------------------------------------------------
//------------------ A NE PAS MODIFIER -----------------------------------------
int main(int argc, char** argv) 
{
    //-- déclaration de variable --//
    
    //-- appel de fonction d'initialisation --//
    InitOscillateur();      // configuration de l'oscillateur 
    Timer1Init();           // initialisation du Timer 1 
    Timer2Init();           // initialisation du Timer 2 
    
    //-- gestion des entrées / sorties --//
    TRISA = 0x0000;
    TRISBbits.TRISB8 = 0; 
    
    //-- gestion du PWM 1--//     
    P1TCONbits.PTMOD = 0;   // free Running mode
    P1TCONbits.PTCKPS = 0;  // prescelaire configurer à 1 : 1 / fcyc = tcyc 
    P1TCONbits.PTOPS = 0;   // postscaler 1 : 1 
    
    // registre de 16bits pour la mise à jour de la péridode du PWM 
    // période : 100kHz même chose que le timer 1 => 10us (T_PWM)
    // tcyc = 1 / Fcyc = 1 / 36.85 Mhz = 27ns 
    // N-Tic_PWM = T_PWM / Tcyc = 10us / 27ns = 370 tic 
    P1TPER = 370; 
    
    //ConfigTimerPWM(100); //insérer valeur PWM 
    PWM1CON1 = 0;           // pour un clear du regitstre
    
    //-- activation 
    PWM1CON1bits.PEN1H = 1; 
    PWM1CON1bits.PEN1L = 0; 
    PWM1CON1bits.PMOD1 = 1; 
    
    //PWM1CON1 = 0;           // pour un clear du regitstre
    //-- activation 
    PWM1CON1bits.PEN2H = 1; 
    PWM1CON1bits.PEN2L = 0; 
    PWM1CON1bits.PMOD2 = 1; 

    // MAJ du Timer Counter lié au PWM à zéro + activation mode PWM  
    P1TMR = 0;
    P1TCONbits.PTEN = 1;        // activer le PWM 
    
    //-- activation du pont H au niveau hardware BTN8982TA --//
    //-- PIN RB8 en sortie à 1 --//
    LATBbits.LATB8 = 1; 
    
    //-- configuration du module de quatrature --// 
    //-- configuration des entres A et B encodeur 
    RPINR14bits.QEA1R = 6;      // correspond à la pin RP6 => A
    RPINR14bits.QEB1R = 5;      // correspond à la pin RP5 => B 
    
    // QEIxCON 
    QEI1CONbits.QEIM = 7;   // Quadrature mode 4 
    QEI1CONbits.PCDOUT = 1; 
    QEI1CONbits.POSRES = 0; 
    //QEI1CONbits.UPDN_SRC = 1; 
    
    DFLT1CONbits.QEOUT = 0; 
    
    MAX1CNT = 65535;                // ??? 
    
    //-sortie en PWM vitesse --//
    //-- OC1 => vitesse angulaire
    OC1CONbits.OCM = 0;         // désactivation  
    
    OC1R = 100;
    OC1RS = 200; 
    
    OC1CONbits.OCTSEL = 0;      // timer2
    OC1R = 100;
    
    OC1CONbits.OCM = 6;         // Activation 
    
    //T2CONbits.TON = 1;      // Enable Timer2 and start the counter
    
    //-- OC2 => vitesse angulaire consigne
    OC2CONbits.OCM = 0;         // désactivation
    OC2R = 100;
    OC2RS = 200; 
    
    OC2CONbits.OCTSEL = 0;      // timer2
    OC2R = 100;
    
    OC2CONbits.OCM = 6;         // activation 
    
    //-- configuration des pins 15 et 13 pour mesurer des PWM qui commanderont le moteur --// 
    RPOR7bits.RP15R = 0x12;    // configuration de la pin 15 en sortie OC1
    RPOR6bits.RP13R = 0x13;    // configuration de la pin 13 en sortie Oc2
    
    //-- activation Timer 2 --// 
    T2CONbits.TON = 1;      // Enable Timer2 and start the counter
    
    //-- boucle sans fin --// 
    while(1)
    {
        //-- temps d'échantillonage doit être plus grand que le temps PID --// 
        //-- GESTION DE LA VITESSE REEL --//
        if (temps_1ms > tempPID)
        {
            //-- appel de fonction pour la mise à jour de la vitese --//
            //-- lecture de l'encodeur A/B pour le calculs des microPas --//
            LectureImagePosition(); 
            //-- calcul de la vitesse en fonction des microPas --// 
            CalculerVitesse(); 
            //-- calcui de l'erreur entre la consigne et la vitesse actuelle
            CalculerErreur();
            //-- mise à jour du facteur uK en fonction des constantes Kp / Ki / Kd
            CalculerPID(); 
            //-- mise à jour des PWM moteur pour commander les moteurs en fonction de uK 
            RaffraichirPWM(); 
            //-- mise à jours de valeur passée => actuel à passée
            MAJVariables(); 
            //-- mise à jour du temps PID par step de 10us 
            tempPID = tempPID + periodePID; 
        }
        
        //-- temps d'échantillonage doit être plus grand que le temps consigne --// 
        //-- GESTION DE LA CONSIGNE--//
        if(temps_1ms > TempsautConsigne)
        {
            //-- gestion des saut de consignes => commande moteur 
            CalculerConsigne();
            //-- mise à jour du tempsConsigne avec des saut de 5s
            TempsautConsigne = TempsautConsigne + periodesaut; 
        }
        
        //-- représentation de la valeur réel => Connecter filtre 2x RC <= IMPORTANT sur la sortie X1 => pin4
        // la valeur 3685 => 100% Timer1 
        // un tour correspond à 262.144 micropas => la constante 1048.4 correspond à +/- 2tours  
        //-- représentation de la vitesse vitesse reelle --//
        OC1RS = (((vitessek / 1048.44) * 3685) + 1842);                    
        //OC1RS = (((vitessek / (10484.0/10.0)) * 3685) + 1842); 
        //- Pulse 
        OC2RS = (((consigneV / 1048.44) * 3685) + 1842); 
        
        //flagInterrupt = 0; 
        
        //-- lecture du flag d'interruption 
        //if(flagInterrupt == 1)
        //{
            /*posk = POS1CNT; 
            
            if (posk - posk_1 >= 32768)
            { 
                retenu -= 1;  
            }
            else if(posk - posk_1 <= -32768)
            {
                retenu += 1; 
            }
            
            position = retenu * pow(2, 16) + posk; */

            //posk_1 = posk; 
            
       //     flagInterrupt = 0; 
       // }
        
        
        
    }

    return (EXIT_SUCCESS);
}

//--------------------------- NE PAS TOUCHER SVP !!! ---------------------------
//----------------------------------------------------------------------------//
//-- nom 				: CalculerVitesse
//-- entrée - sortie 	: - / - 
//-- description 		: Met à jour la vitesse en fonction de la position
//-- remarque 			: 
//-- modification       : 25.01.2022
//----------------------------------------------------------------------------//
void CalculerVitesse(void)
{   
    vitessek = posk - posk_1; 
}

//----------------------------------------------------------------------------//
//-- nom 				: CalculerErreur
//-- entrée - sortie 	: - / - 
//-- description 		: calcul l'erreur en fonction de la consigne et la 
//                        vitesse réel 
//-- remarque 			: 
//-- modification       : 25.01.2022
//----------------------------------------------------------------------------//
void CalculerErreur()
{
    erreurk = consigneV - vitessek; 
}

//----------------------------------------------------------------------------//
//-- nom 				: CalculerErreur
//-- entrée - sortie 	: - / - 
//-- description 		: Mets à jour les PWM pour commander les ponth => 
//                        cmd moteur 
//-- remarque 			: 
//-- modification       : 25.01.2022
//----------------------------------------------------------------------------//
void RaffraichirPWM(void) 
{
    ConfigTimerPWM(Uk); //insérer valeur PWM 
} 

//----------------------------------------------------------------------------//
//-- nom 				: CalculerErreur
//-- entrée - sortie 	: - / - 
//-- description 		: Mise à jour après la gestion des correcteurs P / I / D  
//-- remarque 			: 
//-- modification       : 25.01.2022
//----------------------------------------------------------------------------//
void MAJVariables(void) 
{
    //tempPID = tempPID + periodePID; 
    vitessek_1 = vitessek; 
    posk_1 = posk; 
    
    Ud_k_1 = Ud_k; 
    Ui_k_1 = Ui_k; 
    //consigne_P_k_1 = consigneP; 
    erreurk_1 = erreurk;  
}

//----------------------------------------------------------------------------//
//-- nom 				: LectureImagePosition
//-- entrée - sortie 	: - / - 
//-- description 		: lecteur de l'encodeur A et B => module en quatrature 
//                        activer 
//-- remarque 			: 
//-- modification       : 25.01.2022
//----------------------------------------------------------------------------/
void LectureImagePosition(void)
{
    long cptQadratureK; 
    static long cptQadratureK_1 = 0; 
    
    cptQadratureK = POS1CNT;         // lire compteur quadrature 
            
    if (cptQadratureK - cptQadratureK_1 >= 32768)
    { 
        retenu -= 1;  
    }
    else if(cptQadratureK - cptQadratureK_1 <= -32768)
    {
        retenu += 1; 
    }
    
    posk = retenu * pow(2, 16) + cptQadratureK; 
    cptQadratureK_1 = cptQadratureK; 
   
}

