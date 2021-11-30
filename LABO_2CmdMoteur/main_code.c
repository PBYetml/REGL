//----------------------------------------------------------------------------//
// Nom du projet 		:	Reglage Encodeur
// Nom du fichier 		:   main_code.c
// Date de création 	:   04.12.2019
// Date de modification : 	10.02.2021
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

//-- variables globales utilisées dans plusieurs fichiers --// 
int8_t flagInterrupt = 0;                   

long temps_1ms = 0; 




long periodesaut = 5000;                   // 5s 
long  periodePID = 10;                       // 1ms 

long tempPID = 0, TempsautConsigne = 0; 

//-- position 
long retenu = 0;
long posk = 0, posk_1 = 0; 

//-- vitesse 
float vitessek, vitessek_1; 

//-- erreur 
float erreurk, erreurk_1; 

//-- consigne 
float consineV, consigneP; 

//-- constante PID
float Kp = 0.1, Ki, Kd;  
float Up_k, Ui_k, Ui_k_1,  Ud_k; 
float Uk; 

//-- consigne --//
float saut = 2621;                       // upas = 512*128*4 / 10ms   => 4 tous les flancs 


void CalculerVitesse(void); 
void CalculerErreur(void);
void CalculerPID(void); 
void RaffraichirPWM(void); 
void MAJVariables(void); 
void CalculerConsigne(void); 
void LectureImagePosition(void); 



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
    
    MAX1CNT = 65535;    
    //RPOR7bits.
    
    //-sortie en PWM vitesse
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
    
    
    
    RPOR7bits.RP15R = 0x12;             //configuration de la pin 15 en sortie OC1
    RPOR6bits.RP13R = 0x13;             // configuration de la pin 13 en sortie Oc2
    
    T2CONbits.TON = 1;      // Enable Timer2 and start the counter
    
    
    //-- boucle sans fin --// 
    while(1)
    {
        if (temps_1ms > tempPID)
        {
            LectureImagePosition(); 
            CalculerVitesse(); 
            CalculerErreur();
            CalculerPID(); 
            RaffraichirPWM(); 
            MAJVariables(); 
            tempPID = tempPID + periodePID; 
        }
        
        if(temps_1ms > TempsautConsigne)
        {
            CalculerConsigne(); 
            TempsautConsigne = TempsautConsigne + periodesaut; 
        }
        
        // 3685 = 100% Timer1
        
        OC1RS = (((vitessek / 10484.0) * 3685) + 1842); 
        OC2RS = (((consineV / 10484.0) * 3685) + 1842); 
        
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


void CalculerVitesse(void)
{   
    vitessek = posk - posk_1; 
}

void CalculerErreur()
{
    erreurk = consineV - vitessek; 
}

void CalculerPID(void) 
{
    
    //-- proportionnel 
    Up_k = Kp * erreurk; 
    Uk = Up_k; 
} 


void RaffraichirPWM(void) 
{
    ConfigTimerPWM(Uk); //insérer valeur PWM 
} 


void MAJVariables(void) 
{
    //tempPID = tempPID + periodePID; 
    vitessek_1 = vitessek; 
    posk_1 = posk; 
    
}

void CalculerConsigne(void)
{
    // variable 
    static int states = 0;
    
    states++; 
    
    if(states == 1)
        consineV = 0; 
    else if(states == 2)
    {
       consineV = saut;
       states = 0;
    }
    else if(states == 3)
    {
        consineV = -saut; 
        states = 0;
    }
}

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

