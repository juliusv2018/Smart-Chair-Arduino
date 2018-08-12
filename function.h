/*
 * Created in 2018/1/10
 * Modified in 2018/1/14 16:02  
 * Structure utility functions
 * Author - Butterfly
 */

#include "structure.h"

uint16_t _bv(int pos)
{
	return (1 << pos);
}

uint16_t set_busy_status(uint16_t busy_info, int idx)
{
	uint16_t result = busy_info + _bv(idx);
	return result;
}

int get_busy_status(uint16_t busy_info, int idx)
{
	int state = busy_info & _bv(idx);
	if (state > 0) return 1;
	return 0;
}

Rotation set_rotation(float angle)
{
	Rotation result;
	uint16_t tmp = (uint16_t)(angle * 100);
	result.left = (uint8_t)(tmp / 100);
	result.right = (uint8_t)(tmp % 100);
	return result;
}

float get_rotation(Rotation rot)
{
	return (rot.left + 0.01f * rot.right);
}

