/**************************************************************************

Copyright:BUAA Biologically Inspired Mobile Robot Labratory

Author: Zhuo Yijiang

Date:2022-05-29

Description:Provide  functions  of IQmath

**************************************************************************/

#include "IQmath.h"
using namespace std;

//IQ24转换为double
//@param data IQ24数据
//@return res double数据
double IQ24toD(int data)
{
	double res = 0;
	if (data < 0)
	{
		data = -data;
		res = data >> 24;
		res += double(data >> 16 & 0xFF) / pow(2, 8);
		res += double(data >> 8 & 0xFF) / pow(2, 16);
		res += double(data & 0xFF) / pow(2, 24);
		res = -res;
	}
	else
	{
		res = data >> 24;
		res += double(data >> 16 & 0xFF) / pow(2, 8);
		res += double(data >> 8 & 0xFF) / pow(2, 16);
		res += double(data & 0xFF) / pow(2, 24);
	}
	return res;
}

//double转换为IQ24
//@param data double数据
//@return res IQ24数据
int DtoIQ24(double data)
{
	int res = 0;
	if (data < 0)
	{
		data = -data;
		res = (int(data * pow(2, 24)) & 0xFF);
		res += (int(data * pow(2, 16)) & 0xFF) << 8;
		res += (int(data * pow(2, 8)) & 0xFF) << 16;
		res += int(data) << 24;
		res = -res;
	}
	else
	{
		res = (int(data * pow(2, 24)) & 0xFF);
		res += (int(data * pow(2, 16)) & 0xFF) << 8;
		res += (int(data * pow(2, 8)) & 0xFF) << 16;
		res += int(data) << 24;
	}
	return res;
}

//IQ12转换为double
//@param data IQ12数据
//@return res double数据
double IQ12toD(int16_t data)
{
	double res = 0;
	if (data < 0)
	{
		data = -data;
		res += double(data >> 8 & 0xFF) / pow(2, 4);
		res += double(data & 0xFF) / pow(2, 12);
		res = -res;
	}
	else
	{
		res += double(data >> 8 & 0xFF) / pow(2, 4);
		res += double(data & 0xFF) / pow(2, 12);
	}
	return res;
}

//double转换为IQ12
//@param data double数据
//@return res IQ12数据
int16_t DtoIQ12(double data)
{
	int16_t res = 0;
	if (data < 0)
	{
		data = -data;
		res = (int16_t(data * pow(2, 12)) & 0xFF);
		res += (int16_t(data * pow(2, 4)) & 0xFF) << 8;
		res = -res;
	}
	else
	{
		res = (int16_t(data * pow(2, 12)) & 0xFF);
		res += (int16_t(data * pow(2, 4)) & 0xFF) << 8;
	}
	return res;
}