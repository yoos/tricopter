#ifndef TRIMATH_H
#define TRIMATH_H

// Some non-general vector math.

// Dot product
void vDotP (float v1[3], float v2[3], float dotOut) {
    dotOut += v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2];
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

// Calculate modulus of vector = sqrt(x^2 + y^2 + z^2)
void vMod (float v[3], float modOut) {
    float tmp;
    tmp = v[0] * v[0];
    tmp += v[1] * v[1];
    tmp += v[2] * v[2];
    modOut = tmp;
}

// Normalize vector to a vector with same direction, mod 1
void vNorm (float v[3]) {
    float tmp;
    vMod(v, tmp);
    v[0] /= tmp;
    v[1] /= tmp;
    v[2] /= tmp;
}

// Create equivalent skew symmetric matrix plus identity TODO: what does this do?
// for v = {x,y,z} returns
// m = {{1,-z,y}
//      {z,1,-x}
//      {-y,x,1}}
void vSkew(float v[3], float mOut[3][3]) {
    mOut[0][0] = 1;
    mOut[0][1] = -v[2];
    mOut[0][2] = v[1];
    mOut[1][0] = v[2];
    mOut[1][1] = 1;
    mOut[1][2] = -v[0];
    mOut[2][0] = -v[1];
    mOut[2][1] = v[0];
    mOut[2][2] = 1;
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

