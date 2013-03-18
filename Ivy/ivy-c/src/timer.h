/*
 *	Ivy, C interface
 *
 *	Copyright (C) 1997-2000
 *	Centre d'�tudes de la Navigation A�rienne
 *
 *	Timers for select-based main loop
 *
 *	Authors: Fran�ois-R�gis Colin <fcolin@cena.dgac.fr>
 *
 *	$Id: timer.h 3431 2010-11-22 12:21:04Z fcolin $
 * 
 *	Please refer to file version.h for the
 *	copyright notice regarding this software
 */
#ifndef IVYTIMER_H
#define IVYTIMER_H


#ifdef __cplusplus
extern "C" {
#endif
	
/* Module de gestion des timers autour d'un select */

typedef struct _timer *TimerId;
typedef void (*TimerCb)( TimerId id , void *user_data, unsigned long delta );

/* API  le temps est en millisecondes */
#define TIMER_LOOP -1			/* timer en boucle infinie */
TimerId TimerRepeatAfter( int count, long timeout, TimerCb cb, void *user_data );

void TimerModify( TimerId id, long timeout );

void TimerRemove( TimerId id );

/* Interface avec select */

struct timeval *TimerGetSmallestTimeout();

void TimerScan();
#ifdef __cplusplus
}
#endif
#endif

