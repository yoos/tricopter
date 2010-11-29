//Convention
//matrix[row][column]


void MatrixAdd(float** input1, float** input2, int height, int width, float** output) {
  for (int i = 0; i < height; i++) {
    for (int j = 0; j < width; j++) {
      output[i][j] = input1[i][j] + input2[i][j];
    }
  }
}

void MatrixMultiply(float** input1, float** input2, int height1, int width1, int width2, float** output) {
  for (int i = 0; i < height1; i++) {
    for (int j = 0; j < width2; j++) {
      output[i][j] = 0;
      for (int k = 0; k < width1; k++) {
        output[i][j] += input1[i][k] * input2[k][j];
      }
    }
  }
}

void MatrixTranspose(float** input, int rows, int columns, float** output) {
  for (int i = 0; i < rows; i++) {
    for (int j = 0; j < columns; j++) {
      output[i][j] = input[j][i];
    }
  }
}
/*
void AntiSemetricMatrix(float * input, int length, float** output) {
  for (int i = 0; i < length; i++) {
    for (int j = 0; i < length; i++) {
      if (i == j) {
        output[i][j] = 0;
      } else {
        
  
*/

float DotProduct(float* vector1, float* vector2, int length) {
  float result = 0;
  for (int i = 0; i < length; i++) {
    result += vector1[i] * vector2[i];
  }
  return result;
}
