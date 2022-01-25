//----------------------------------------------------------------------------//
// Nom du projet 		:	Reglage Encodeur
// Nom du fichier 		:   configOscillateur.h 
// Date de création 	:   03.12.2020
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
//----------------------------------------------------------------------------//
//-- librairie standart --// 
#include <stdint.h>

#ifndef CONFIGTIMER_H
#define	CONFIGTIMER_H

//-- variables globales --// 

//-- prototypes --// 
void Timer1Init(void); 
void Timer2Init(void); 
inline void InterruptTimer1Init(void); 
inline void InterruptTimer2Init(void); 

//-- prototypes -> interruption --// 
void __attribute__((interrupt, no_auto_psv)) _T1Interrupt( void ); 
//void __attribute__((interrupt, no_auto_psv)) _T2Interrupt( void ); 

#endif	/* CONFIGTIMER_H */

