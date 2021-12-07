//----------------------------------------------------------------------------//
// Nom du projet 		:	Reglage Encodeur
// Nom du fichier 		:   PWMConfiguration.c
// Date de création 	:   28.01.2021
// Date de modification : 	xx.xx.2021
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
//----------------------------------------------------------------------------//
//--- librairie à inclure ---// 
#if defined(__dsPIC33F__)
#include "p33Fxxxx.h"
#endif

//-- librairie standart --// 
#include <stdint.h>
#include <xc.h>
//-- librairie personnelle --// 

#include "ModuleConfiPWM.h"


//----------------------------------------------------------------------------//
//-- nom 				: ConfigTimerPWM
//-- entrée - sortie 	: - / - 
//-- description 		: configuration du Timer pour le PWM 
//-- remarque 			: datasheet DSPIC -> timer section 16 -> p213
//                        configuration de la période du PWM
//                        Fcy : 36.85 [MHz]
//                        nb_tic = Fcyc / Ft1 
//                        36.85 x 10^6 / 100 x 10^3 = 36.85
//-- modification       : 28.01.2021
//----------------------------------------------------------------------------//
void ConfigTimerPWM(float pourcentPWM) 
{
    //-- déclaration de varaible --// 
    float var1, var2; 
    
    //-- gestion limte valeur PWM --//
    if(pourcentPWM > 100)
        pourcentPWM = 100; 
    else if (pourcentPWM < -100)
        pourcentPWM = -100; 
    
    // PWM à 50% (la mise à jour ce fait à 2 x la valeur de la période)
    // ex : (2 x P1TPER + 2) * rapport cyclique (60/100)
    var1 = OFFSET + (pourcentPWM * PENTE) ; 
    P1DC1 = (2 * P1TPER + 2) * (var1/100.0); 
    
    // PWM à 50% (la mise à jour ce fait à 2 x la valeur de la période)
    // ex : (2 x P1TPER + 2) * rapport cyclique (60/100)
    var2 = OFFSET - (pourcentPWM * PENTE) ; 
    P1DC2 = (2 * P1TPER + 2) * (var2/100.0); 
    
    //PWM1CON1 = 0;           // pour un clear du regitstre
    
    //-- activation 
    /*PWM1CON1bits.PEN1H = 1; 
    PWM1CON1bits.PEN1L = 0; 
    PWM1CON1bits.PMOD1 = 1; 
    
    //PWM1CON1 = 0;           // pour un clear du regitstre
    //-- activation 
    PWM1CON1bits.PEN2H = 1; 
    PWM1CON1bits.PEN2L = 0; 
    PWM1CON1bits.PMOD2 = 1; */
    
    // PWM à 50% (la mise à jour ce fait à 2 x la valeur de la période)
    // ex : (2 x P1TPER + 2) * rapport cyclique (60/100)
 
    //P1DC1 = (P1TPER  * 50)/100;
    
    
    //-- gestion du PWM 2--//     
    //P1TCONbits.PTMOD = 0;   // free Running mode
    //P1TCONbits.PTCKPS = 0;  // prescelaire configurer à 1 : 1 / fcyc = tcyc 
    //P1TCONbits.PTOPS = 0;   // postscaler 1 : 1 
    
    //PWM1CON1 = 0;           // pour un clear du regitstre
    //-- activation 
    //PWM1CON1bits.PEN2H = 1; 
    //PWM1CON1bits.PEN2L = 0; 
    //PWM1CON1bits.PMOD2 = 1; 
        
    // PWM à 50% (la mise à jour ce fait à 2 x la valeur de la période)
    // ex : (2 x P1TPER + 2) * rapport cyclique (60/100)
    //P1DC2 = ((2 * P1TPER * 12)/100) + 2; 
    //P1DC2 = (P1TPER  * 50)/100;
    
    // MAJ du Timer Counter lié au PWM à zéro + activation mode PWM  
    //P1TMR = 0; 
    //P1TCONbits.PTEN = 1;        // activer le PWM 
}
/*
//-- gestion du PWM 1--//     
    P1TCONbits.PTMOD = 0;   // free Running mode
    P1TCONbits.PTCKPS = 0;  // prescelaire configurer à 1 : 1 / fcyc = tcyc 
    P1TCONbits.PTOPS = 0;   // postscaler 1 : 1 
    
    PWM1CON1 = 0;           // pour un clear du regitstre
    //-- activation 
    PWM1CON1bits.PEN1H = 1; 
    PWM1CON1bits.PEN1L = 0; 
    PWM1CON1bits.PMOD1 = 1; 
    
    
    // registre de 16bits pour la mise à jour de la péridode du PWM 
    // période : 100kHz même chose que le timer 1 => 10us (T_PWM)
    // tcyc = 1 / Fcyc = 1 / 36.85 Mhz = 27ns 
    // N-Tic_PWM = T_PWM / Tcyc = 10us / 27ns = 370 tic 
    P1TPER = 370; 
    
    // PWM à 50% (la mise à jour ce fait à 2 x la valeur de la période)
    // ex : (2 x P1TPER + 2) * rapport cyclique (60/100)
    P1DC1 = ((2 * P1TPER * 88)/100) + 2; 
    //P1DC1 = (P1TPER  * 50)/100;
    
    
    //-- gestion du PWM 2--//     
    //P1TCONbits.PTMOD = 0;   // free Running mode
    //P1TCONbits.PTCKPS = 0;  // prescelaire configurer à 1 : 1 / fcyc = tcyc 
    //P1TCONbits.PTOPS = 0;   // postscaler 1 : 1 
    
    //PWM1CON1 = 0;           // pour un clear du regitstre
    //-- activation 
    PWM1CON1bits.PEN2H = 1; 
    PWM1CON1bits.PEN2L = 0; 
    PWM1CON1bits.PMOD2 = 1; 
        
    // PWM à 50% (la mise à jour ce fait à 2 x la valeur de la période)
    // ex : (2 x P1TPER + 2) * rapport cyclique (60/100)
    P1DC2 = ((2 * P1TPER * 12)/100) + 2; 
    //P1DC2 = (P1TPER  * 50)/100;
    
    // MAJ du Timer Counter lié au PWM à zéro + activation mode PWM  
    P1TMR = 0; 
    P1TCONbits.PTEN = 1;        // activer le PWM
 * 
 * */