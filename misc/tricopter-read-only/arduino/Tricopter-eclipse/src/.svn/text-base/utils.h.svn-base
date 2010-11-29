/*
 * utils.h
 *
 *  Created on: Mar 23, 2010
 *      Author: ben
 */

#ifndef UTILS_H_
#define UTILS_H_

#include <WProgram.h>

#define DEBUG(x,y)  Serial.print("D "); Serial.print(x); Serial.println(y);

/* Find the sum of an int array */
int sumArray(int* array, int start, int end);

/* Find the mode of an int array
 * The mode is the value that occurs most frequently
 */
int findMode(int *data, int arraySize);

/* Works faster and is smaller than the constrain() function */
int limitIntRange(int data, int minLimit, int maxLimit);

float limitFloatRange(float data, float minLimit, float maxLimit);


#endif /* UTILS_H_ */
