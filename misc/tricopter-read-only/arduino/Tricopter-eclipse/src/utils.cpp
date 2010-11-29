#include <WProgram.h>
#include "utils.h"

/* Find the sum of an int array */
int sumArray(int* array, int start, int end) {
  int s = 0;
  for(int i = start; i < end; i++) {
    s += array[i];
  }
  return s;
}

/* Find the mode of an int array
 * The mode is the value that occurs most frequently
 */
int findMode(int *data, int arraySize) {
  boolean done = 0;
  byte i;
  int temp, maxData, frequency, maxFrequency;

  // Sorts numbers from lowest to highest
  while (done != 1) {        
    done = 1;
    for (i=0; i<(arraySize-1); i++) {
      if (data[i] > data[i+1]) {     // numbers are out of order - swap
        temp = data[i+1];
        data[i+1] = data[i];
        data[i] = temp;
        done = 0;
      }
    }
  }

  temp = 0;
  frequency = 0;
  maxFrequency = 0;

  // Count number of times a value occurs in sorted array
  for (i=0; i<arraySize; i++) {
    if (data[i] > temp) {
      frequency = 0;
      temp = data[i];
      frequency++;
    } 
    else if (data[i] == temp) frequency++;
    if (frequency > maxFrequency) {
      maxFrequency = frequency;
      maxData = data[i];
    }
  }
  return maxData;
}

/* Works faster and is smaller than the constrain() function */
int limitIntRange(int data, int minLimit, int maxLimit) {
  if (data < minLimit) return minLimit;
  else if (data > maxLimit) return maxLimit;
  else return data;
}

float limitFloatRange(float data, float minLimit, float maxLimit) {
  if (data < minLimit) return minLimit;
  else if (data > maxLimit) return maxLimit;
  else return data;
}
