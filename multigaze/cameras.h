/*
 * cameras.h
 *
 *  Created on: May 11, 2015
 *      Author: mavlab
 */

#ifndef CAMERAS_H_
#define CAMERAS_H_


#define CAMERA1   Usart2
#define CAMERA2   Usart5
#define CAMERA3   Usart4
#define CAMERA4   Usart1
#define CAMERA5   Usart6
#define CAMERA6   Usart3


void cameras_init(void);
void cameras_tunnel(void);

#endif /* CAMERAS_H_ */
