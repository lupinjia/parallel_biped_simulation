#pragma once

#include <iostream>

using namespace std;

#define FOOT_NUM 2

class BaseContactSensor
{
	virtual void update() = 0;
	virtual bool getContact(int id) = 0;
	virtual bool* getContact() = 0; //all
};
