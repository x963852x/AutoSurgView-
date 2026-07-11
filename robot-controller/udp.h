#pragma once

#include <atomic>
#include <thread>
#include <Windows.h>
#include <iostream>
#include <mutex>
#include <map>
#include "json/json.h"

//UDP发送状态
#define NEWSEND_FREE 0;
#define NEWSEND_READY 1;

void udp_init();	//UDP初始化