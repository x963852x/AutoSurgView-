/**************************************************************************

Copyright:BUAA Biologically Inspired Mobile Robot Labratory

Author: Zhuo Yijiang

Date:2022-05-29

Description:Provide  functions  of IQmath

**************************************************************************/

#pragma once

#include <math.h>
#include <stdio.h>
#include <iostream>


double IQ24toD(int data);	//IQ24转换为double
int DtoIQ24(double data);	//double转换为IQ24

double IQ12toD(int16_t data);	//IQ12转换为double
int16_t DtoIQ12(double data);	//double转换为IQ12