/*	Floating point PID control loop for Microcontrollers
	Copyright (C) 2015 Jesus Ruben Santa Anna Zamudio.
	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.
	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.
	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
	Author website: http://www.geekfactory.mx
	Author e-mail: ruben at geekfactory dot mx
 */
#include "main.h"
#include "PID.h"
#define TICK_SECOND 1000

bool pid_enabled = true;

pid_t pid_create(pid_t pid, float* in, float* out, float* set, float kp, float ki, float kd)
{
	pid->input = in;
	pid->output = out;
	pid->setpoint = set;
	pid->automode = false;

	pid_limits(pid, 0, 255); //widget motor speed max????  set to 150  // before 255

	// Set default sample time to 10 ms
	pid->sampletime = 10 * (TICK_SECOND / 1000);

	pid_direction(pid, E_PID_DIRECT);
	pid_tune(pid, kp, ki, kd);

	pid->lasttime = HAL_GetTick() - pid->sampletime;
	pid_auto(pid);
	pid->circulaire = 0;
	
	return pid;
}

bool pid_need_compute(pid_t pid)
{
	// Check if the PID period has elapsed
	if(pid->Kp == 0) return false;
	if(!pid_enabled) return false;
	return(HAL_GetTick() - pid->lasttime >= pid->sampletime) ? true : false;
}

void pid_compute(pid_t pid)
{
	if(!pid_enabled) return;
	// Check if control is enabled
	if (!pid->automode)
		return;
	
	float in = *(pid->input);
	// Compute error
	float error = (*(pid->setpoint)) - in;
	
	if(pid->circulaire > 0){
		float half = pid->circulaire / 2;
		if(error > half) 		error -= pid->circulaire;
		if(error < -half) 	error += pid->circulaire;
	}
	
	// Compute integral
	pid->iterm += (pid->Ki * error);
	if (pid->iterm > pid->omax)
		pid->iterm = pid->omax;
	else if (pid->iterm < pid->omin)
		pid->iterm = pid->omin;
	// Compute differential on input
	float dinput = in - pid->lastin;
	// Compute PID output
	float out = pid->Kp * error + pid->iterm - pid->Kd * dinput;
	// Apply limit to output value
	if (out > pid->omax)
		out = pid->omax;
	else if (out < pid->omin)
		out = pid->omin;
	// Output to pointed variable
	
	(*pid->output) = out;
	
	// Keep track of some variables for next execution
	pid->lastin = in;
	pid->lasttime = HAL_GetTick();
}

void pid_enable(bool isEnabled){
	pid_enabled = isEnabled;
}

void pid_tune(pid_t pid, float kp, float ki, float kd)
{
	// Check for validity
	if (kp < 0 || ki < 0 || kd < 0)
		return;
	
	//Compute sample time in seconds
	float ssec = ((float) pid->sampletime) / ((float) TICK_SECOND);

	pid->Kp = kp;
	pid->Ki = ki * ssec;
	pid->Kd = kd / ssec;

	if (pid->direction == E_PID_REVERSE) {
		pid->Kp = 0 - pid->Kp;
		pid->Ki = 0 - pid->Ki;
		pid->Kd = 0 - pid->Kd;
	}
}

void pid_sample(pid_t pid, uint32_t time)
{
	if (time > 0) {
		float ratio = (float) (time * (TICK_SECOND / 1000)) / (float) pid->sampletime;
		pid->Ki *= ratio;
		pid->Kd /= ratio;
		pid->sampletime = time * (TICK_SECOND / 1000);
	}
}

void pid_limits(pid_t pid, float min, float max)
{
	if (min >= max) return;
	pid->omin = min;
	pid->omax = max;
	//Adjust output to new limits
	if (pid->automode) {
		if (*(pid->output) > pid->omax)
			*(pid->output) = pid->omax;
		else if (*(pid->output) < pid->omin)
			*(pid->output) = pid->omin;

		if (pid->iterm > pid->omax)
			pid->iterm = pid->omax;
		else if (pid->iterm < pid->omin)
			pid->iterm = pid->omin;
	}
}

void pid_auto(pid_t pid)
{
	// If going from manual to auto
	if (!pid->automode) {
		pid->iterm = *(pid->output);
		pid->lastin = *(pid->input);
		if (pid->iterm > pid->omax)
			pid->iterm = pid->omax;
		else if (pid->iterm < pid->omin)
			pid->iterm = pid->omin;
		pid->automode = true;
	}
}

void pid_manual(pid_t pid)
{
	pid->automode = false;
}

void pid_direction(pid_t pid, enum pid_control_directions dir)
{
	if (pid->automode && pid->direction != dir) {
		pid->Kp = (0 - pid->Kp);
		pid->Ki = (0 - pid->Ki);
		pid->Kd = (0 - pid->Kd);
	}
	pid->direction = dir;
}

void pid_circulaire(pid_t pid, float limit)
{
	pid->circulaire = limit;
}

void pid_debug_uart(pid_t pid){
	uart_debug_printf("%d,%f,%f,%f\r\n", HAL_GetTick(), *(pid->setpoint), *(pid->input), *(pid->output));
}
