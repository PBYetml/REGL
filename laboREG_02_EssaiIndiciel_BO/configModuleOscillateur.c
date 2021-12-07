//----------------------------------------------------------------------------//
// Nom du projet 		:	Reglage Encodeur
// Nom du fichier 		:   configModuleOscillateur.c
// Date de création 	:   02.12.2020
// Date de modification : 	27.01.2021
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
//      Hardware de la carte 
//      K:\ES\PROJETS\SLO\1601x_HalfBridge3x\doc 
//
//	    doc pour le DSP : 
//		K:\ES\PROJETS\SLO\1601x_HalfBridge3x\doc\datasheets
// 
//----------------------------------------------------------------------------//

//--- librairie à inclure ---// 
#if defined(__dsPIC33F__)
#include "p33Fxxxx.h"
#endif

#include "xc.h"                     // appel de directive du compilateur C pour le Dspic 
#include "configOscillateur.h"      // configuration de l'oscillateur interne du Dspic

//----------------------------------------------------------------------------//
//-- nom 				: InitOscillateur
//-- entrée - sortie 	: - / - 
//-- description 		: configuration oscillateur | clock à choix  
//-- remarque 			: datasheet DSPIC -> section 9 -> p.145
//                        On aimerait Fosc <= 80MHz 
//                        choix oscillateur FRC (7.37MHz)
//                        Fosc = Fin * (M / (N1 * N2))
//                        7.37*10^6 *(160 / (7 * 2)) = 73.7 [MHz]
//                        Fcy = Fosc / 2 => 73.7 x 10^6 / 2 = 36.85[MHz]
//-- modification       : 03.12.2020
//----------------------------------------------------------------------------//
void InitOscillateur(void)
{
    RCONbits.SWDTEN=0;                      /* Disable Watch Dog Timer*/
	    
    PLLFBDbits.PLLDIV 	= 158;			// M = 158 => 158+2 => 160  -> voir remarque 
    CLKDIVbits.PLLPRE 	= 5; 			// N1 = 5 => 5+2 = 7        -> voir remarque 
    CLKDIVbits.PLLPOST 	= 0; 			// N2 = 2                   -> voir remarque 
    
    // Clock switching to incorporate PLL
	__builtin_write_OSCCONH(0x01);		// Initiate Clock Switch to Primary
                                        // Oscillator with PLL (NOSC=0b011)
	__builtin_write_OSCCONL(0x01);		// Start clock switching
}
