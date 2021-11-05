/*
  Copyright (c) 2017, COMAU S.p.A.
  All rights reserved.
  
  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
  
  1. Redistributions of source code must retain the above copyright notice, this
     list of conditions and the following disclaimer.
  2. Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation
     and/or other materials provided with the distribution.
  
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  
  The views and conclusions contained in the software and documentation are those
  of the authors and should not be interpreted as representing official policies,
  either expressed or implied, of the FreeBSD Project.
*/
/*
 * EdoMsgType.h
 *
 *  Created on: Jun 29, 2017
 *      Author: comau
 */

#ifndef EDO_MSG_TYPE_H_
#define EDO_MSG_TYPE_H_

  enum E_MOVE_COMMAND {
    E_MOVE_COMMAND_MOVE       = 'M', /* Execute a command statement */
    E_MOVE_COMMAND_CANCEL     = 'C', /* cancella la move in esecuzione (se presente) */
    E_MOVE_COMMAND_PAUSE      = 'P', /* mette in pausa la move in esecuzione (se presente) */
    E_MOVE_COMMAND_RESUME     = 'R', /* avvia la move precedentemente messa in pausa (se presente) */
    E_MOVE_COMMAND_JOGMOVE    = 'J', /* Execute a JOG move */
    E_MOVE_COMMAND_JOGSTOP    = 'S'  /* Stop a JOG move */
  };
  enum E_MOVE_TYPE {
    E_MOVE_TYPE_JOINT    = 'J',
    E_MOVE_TYPE_LINEAR   = 'L',
    E_MOVE_TYPE_CIRCULAR = 'C'
  };
  enum E_MOVE_DEST_POINT {
    E_MOVE_POINT_JOINT     = 'J',
    E_MOVE_POINT_POSITION  = 'P',
    E_MOVE_POINT_XTND_POS  = 'X'
  };

#define SSM_CONFIG_FLAG_STRLEN_MAX 80
#if 0
=================================================================================================
enum MOVE_MESSAGE_TYPE {
	MOVE_TRJNT_J =  00, /* esegue la move in giunti, con INPUT il comando in giunti */
	MOVE_TRJNT_C =  01, /* esegue la move in giunti, con INPUT la posizione Cartesiana */
	MOVE_CARLIN_J = 10, /* esegue la move cartesiana lineare, con INPUT il comando in giunti */
	MOVE_CARLIN_C = 11, /* esegue la move cartesiana lineare, con INPUT la posizione Cartesiana */
	MOVE_CARCIR_J = 20, /* esegue la move cartesiana circolare, con INPUT due comandi (ultimo e intermedio) in giunti */
	MOVE_CARCIR_C = 21, /* esegue la move cartesiana circolare, con INPUT due posizioni Cartesiane (ultima e intermedia) */
	MOVE_PAUSE = 6, /* mette in pausa la move in esecuzione (se presente) */
	MOVE_RESUME = 7, /* avvia la move precedentemente messa in pausa (se presente) */
	MOVE_CANCEL = 8 /* cancella la move in esecuzione (se presente) */
};

enum JOG_MESSAGE_TYPE {
	JOG_TRJNT = 0, /* esegue la jog in giunti, al comando il movimento inizia, finchè non viene settato il comando di STOP */
	JOG_CARLIN = 1, /* esegue la jog cartesiana lineare, al comando il movimento inizia, finchè non viene settato il comando di STOP */
	JOG_STOP = 2 /* ferma il movimento JOG */
};
==================================================================================================
#endif

enum MESSAGE_FEEDBACK {
	COMMAND_RECEIVED = 0, /* acknowledgement, comando ricevuto */
	F_NEED_DATA = 1, /* si chiede il punto successivo */
	COMMAND_EXECUTED = 2, /* movimento completato */
	ERROR = -1, /* l’esecuzione della move è stata interrotta per un errore */
	COMMAND_REJECTED = -2, /* comando scartato */
	BUFFER_FULL = -3
};

// TODO V_FON TBD
enum ERROR_FEEDBACK {
	ERROR_1 = 0,
	ERROR_2 = 1
};

#endif /* EDO_MSG_TYPE_H_ */