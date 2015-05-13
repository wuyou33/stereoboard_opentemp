/*
 * cameras.h
 *
 *  Created on: May 11, 2015
 *      Author: mavlab
 */

#ifndef CAMERAS_H_
#define TUNNELS_H_


#define CAMERA1   Usart2
#define CAMERA2   Usart5
#define CAMERA3   Usart4
#define CAMERA4   Usart1
#define CAMERA5   Usart6
#define CAMERA6   Usart3


#define Cam1Tx  Usart2Tx
#define Cam1Ch  Usart2Ch
#define Cam1Rx  Usart2Rx

#define Cam2Tx  Usart5Tx
#define Cam2Ch  Usart5Ch
#define Cam2Rx  Usart5Rx

#define Cam3Tx  Usart4Tx
#define Cam3Ch  Usart4Ch
#define Cam3Rx  Usart4Rx

#define Cam4Tx  Usart1Tx
#define Cam4Ch  Usart1Ch
#define Cam4Rx  Usart1Rx

#define Cam5Tx  Usart6Tx
#define Cam5Ch  Usart6Ch
#define Cam5Rx  Usart6Rx

#define Cam6Tx  Usart3Tx
#define Cam6Ch  Usart3Ch
#define Cam6Rx  Usart3Rx


void tunnel_init(void);
void tunnel_run(void);

#endif /* CAMERAS_H_ */
