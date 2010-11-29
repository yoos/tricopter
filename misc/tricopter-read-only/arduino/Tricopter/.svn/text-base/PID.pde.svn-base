/*
  AeroQuad v1.2 - June 2009
 www.AeroQuad.info
 Copyright (c) 2009 Ted Carancho.  All rights reserved.
 An Open Source Arduino based quadrocopter.
 
 This program is free software: you can redistribute it and/or modify 
 it under the terms of the GNU General Public License as published by 
 the Free Software Foundation, either version 3 of the License, or 
 (at your option) any later version. 
 
 This program is distributed in the hope that it will be useful, 
 but WITHOUT ANY WARRANTY; without even the implied warranty of 
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
 GNU General Public License for more details. 
 
 You should have received a copy of the GNU General Public License 
 along with this program. If not, see <http://www.gnu.org/licenses/>. 
 */

// Modified from http://www.arduino.cc/playground/Main/BarebonesPIDForEspresso
float updatePID(float targetPosition, float currentPosition, struct PIDdata *p)
{
  float error = targetPosition - currentPosition;

  p->integratedError += error;
  p->integratedError = limitRange(p->integratedError, -windupGaurd, windupGaurd);

  p->dTerm = p->D_GAIN * (currentPosition - p->lastPosition); // Assumes constant time between updates
  p->iTerm = p->I_GAIN * p->integratedError;
  p->pTerm = p->P_GAIN * error;
  p->lastPosition = currentPosition;
  return p->pTerm + p->iTerm + p->dTerm;
}

void zeroIntegralError(struct PIDdata *p) {
  p->integratedError = 0;
}

