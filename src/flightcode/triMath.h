#ifndef TRIMATH_H
#define TRIMATH_H

// Some non-general vector math.

// Dot product
void vDotP (float v1[3], float v2[3], float vOut[3]) {
    for (int i=0; i<3; i++) {
        vOut[i] += v1[i] * v2[i];
    }
}

// Cross product
void vCrossP (float v1[3], float v2[3], float vOut[3]) {
    vOut[0] = (v1[1]*v2[2]) - (v1[2]*v2[1]);
    vOut[1] = (v1[2]*v2[0]) - (v1[0]*v2[2]);
    vOut[2] = (v1[0]*v2[1]) - (v1[1]*v2[0]);
}

// Scalar multiplication
void vScale (float v[3], float scale, float vOut[3]) {
    for (int i=0; i<3; i++) {
        vOut[i] = v[i] * scale; 
    }
}

// Addition
void vAdd (float v1[3], float v2[3], float vOut[3]) {
    for (int i=0; i<3; i++) {
         vOut[i] = v1[i] + v2[i];
    }
}


// 3x3 matrix multiplication
void mProduct (float m1[3][3], float m2[3][3], float mOut[3][3]) {
    float tmp[3];
    for (int i=0; i<3; i++) {
        for(int j=0; j<3; j++) {
            for(int k=0; k<3; k++) {
                tmp[k] = m1[i][k] * m2[k][j];
            }
            mOut[i][j] = tmp[0] + tmp[1] + tmp[2];
        }
    }
}

#endif // TRIMATH_H

