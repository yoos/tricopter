''
''
''     QuadRotor - Full Stability
'' -- Jason Dorie --               
''               
''

{
  TODO:
        Implement DCM based control algorithm

  DONE:
        Temperature drift compensation in gyro code
        Convert ITG-3200 / ADXL-345 code to PASM
        Add ADXL345 readings to ITG code
        Implementation of rotating frame of reference (DCM)
}


CON
  _clkmode = xtal1 + pll16x
  _xinfreq = 5_000_000

  OUT_F = 0                ' Front ESC                       
  OUT_R = 1                ' Right ESC
  OUT_B = 2                ' Back  ESC                        
  OUT_L = 3                ' Left  ESC


  FixedBits = 15
  Fixed_One = 1 << FixedBits



OBJ
  Gyro  : "ITG-3200-pasm.spin"                          ' 1 cog
  Dbg   : "FullDuplexSerial.spin"
  'XBee  : "FullDuplexSerial.spin"

VAR
  long Output[4]                                        ' Temp output flight controls array, copied into 'servo' when finished                        
  long counter
  long LastTime, CurTime

  long Gx, Gy, Gz                                       ' Gyro readings (instant)
  long Ax, Ay, Az

  long RotX, RotY, RotZ
  long SinX, SinY, SinZ
  long Ref[9], Temp[9]

  long CorrectionScaled[3]
  long XErr[3], YErr[3]
  long AccelLevel[3], Gravity[3]


PUB Main | start, end
 
  Dbg.Start( 31, 30, 0, 115200 )
  'XBee.Start( 24, 25, 0, 115200 )
  Gyro.Start( 16, 17 )

  counter := 0
  LastTime := cnt

  'Initialize the reference matrix to identity, fixed point format, standard identity matrix (X = left, Y = up, Z = forward)
  Ref[0] := Fixed_One 
  Ref[1] := 0 
  Ref[2] := 0

  Ref[3] := 0 
  Ref[4] := Fixed_One 
  Ref[5] := 0 

  Ref[6] := 0 
  Ref[7] := 0 
  Ref[8] := Fixed_One

  CorrectionScaled[0] := 0
  CorrectionScaled[1] := 0
  CorrectionScaled[2] := 0   

  'Note that because the gyro and accelerometer use different meanings for x,y,z, the axis get moved.  I consider a Z
  'rotation a rotation around the Z (forward) axis.
  
  AccelLevel[0] :=  Gyro.GetAX << (FixedBits-8)
  AccelLevel[1] := (Gyro.GetAZ << (FixedBits-8)) - Fixed_One
  AccelLevel[2] :=  Gyro.GetAY << (FixedBits-8)                 


  repeat
    Gx := Gyro.GetRX
    Gy := Gyro.GetRY
    Gz := Gyro.GetRZ
    Ax := Gyro.GetAX
    Ay := Gyro.GetAY
    Az := Gyro.GetAZ

    {
    'Use this code and remove the update routines when computing in LocalMode on the PC viewer
    'Just sends the gyro and accelerometer readings as signed 16 bit values to the PC and the PC runs the DCM.
    Dbg.tx( $77 )
    Dbg.tx( $77 )
    Dbg.tx( Gx >> 8 )
    Dbg.tx( Gx & $ff )
    Dbg.tx( Gy >> 8 )
    Dbg.tx( Gy & $ff )
    Dbg.tx( Gz >> 8 )
    Dbg.tx( Gz & $ff )
     
    Dbg.tx( Ax >> 8 )
    Dbg.tx( Ax & $ff )
    Dbg.tx( Ay >> 8 )
    Dbg.tx( Ay & $ff )
    Dbg.tx( Az >> 8 )
    Dbg.tx( Az & $ff )
    }

    'Use this code with the update routine when NOT running localmode on the PC viewer - just
    'transmits the 9 matrix values as signed bytes for display
    Dbg.tx( $78 )
    Dbg.tx( $78 )
    Dbg.tx( Ref[0] ~> (FixedBits-8+2) )
    Dbg.tx( Ref[1] ~> (FixedBits-8+2) )
    Dbg.tx( Ref[2] ~> (FixedBits-8+2) )
    Dbg.tx( Ref[3] ~> (FixedBits-8+2) )
    Dbg.tx( Ref[4] ~> (FixedBits-8+2) )
    Dbg.tx( Ref[5] ~> (FixedBits-8+2) )
    Dbg.tx( Ref[6] ~> (FixedBits-8+2) )
    Dbg.tx( Ref[7] ~> (FixedBits-8+2) )
    Dbg.tx( Ref[8] ~> (FixedBits-8+2) )

    'Magic happens here
    UpdateLoop
    
    waitcnt( constant(80_000_000 / 200) + LastTime )
    LastTime += constant(80_000_000 / 200)


PUB GetAccelAsVector( Vect )
  long[Vect][0] := Ax << (FixedBits-8)
  long[Vect][1] := Az << (FixedBits-8)
  long[Vect][2] := Ay << (FixedBits-8)                 


PUB UpdateLoop | iGx, iGy, iGz, ErrorAxis[3]

  'Convert the gyro readings into proper range for the DCM inputs.  Also, swap the axis
  'around and adjust signs to coincide with my frame of reference
  iGz := -Gx / 5
  iGx :=  Gy / 5
  iGy := -Gz / 5
   
  iGx += CorrectionScaled[0]
  iGy += CorrectionScaled[1]
  iGz += CorrectionScaled[2]

  'Rotate the current reference matrix into the temporary
  RotateMatrix( @Ref , iGx, iGy, iGz , @Temp ) 

  'Normalize & orthonormalize the result
  RenormalizeMatrix( @Temp )

  'Copy the result back into the reference matrix
  longmove( @Ref, @Temp, 9 )  
   
   
  'Gravity (or some approximation thereof) is the accelerometer

  GetAccelAsVector( @Gravity )  'Swaps the axis, scales the values
  Gravity[0] -= AccelLevel[0]   'Offset the accelerometer, assuming it was level on startup (HACK)
  Gravity[1] -= AccelLevel[1]
  Gravity[2] -= AccelLevel[2]

  'GravityNorm = Gravity.Normalized();
  NormalizeVector( @Gravity )
  
  'FixedVector WorldUp = new FixedVector(BodyEstCorr.m[1, 0], BodyEstCorr.m[1, 1], BodyEstCorr.m[1, 2]);
  'ErrorAxis = GravityNorm.Cross(WorldUp)
  CrossVector( @Gravity, @Ref[3], @ErrorAxis[0] )    

  'Scale the error value down enough to feed back into the update loop.  Should convert this to a PID (without D)
  ErrorAxis[0] ~>= 8   
  ErrorAxis[1] ~>= 8   
  ErrorAxis[2] ~>= 8   
   
  CorrectionScaled[0] := ErrorAxis[0]
  CorrectionScaled[1] := ErrorAxis[1]
  CorrectionScaled[2] := ErrorAxis[2]   



PUB RotateMatrix( Mat , Rx, Ry, Rz , Dest ) | F1

  { Shown here for clarity - The F1 terms (fixed point one) are removed from the real code for speed
    F1 := Fixed_One  
     
    long[Dest][0] := (long[Mat][0] *  F1 + long[Mat][1] *  Rz + long[Mat][2] * -Ry) ~> FixedBits
    long[Dest][1] := (long[Mat][0] * -Rz + long[Mat][1] *  F1 + long[Mat][2] *  Rx) ~> FixedBits
    long[Dest][2] := (long[Mat][0] *  Ry + long[Mat][1] * -Rx + long[Mat][2] *  F1) ~> FixedBits
     
    long[Dest][3] := (long[Mat][3] *  F1 + long[Mat][4] *  Rz + long[Mat][5] * -Ry) ~> FixedBits
    long[Dest][4] := (long[Mat][3] * -Rz + long[Mat][4] *  F1 + long[Mat][5] *  Rx) ~> FixedBits
    long[Dest][5] := (long[Mat][3] *  Ry + long[Mat][4] * -Rx + long[Mat][5] *  F1) ~> FixedBits
     
    long[Dest][6] := (long[Mat][6] *  F1 + long[Mat][7] *  Rz + long[Mat][8] * -Ry) ~> FixedBits
    long[Dest][7] := (long[Mat][6] * -Rz + long[Mat][7] *  F1 + long[Mat][8] *  Rx) ~> FixedBits
    long[Dest][8] := (long[Mat][6] *  Ry + long[Mat][7] * -Rx + long[Mat][8] *  F1) ~> FixedBits
    }

  long[Dest][0] := long[Mat][0] + (long[Mat][1] *  Rz + long[Mat][2] * -Ry) / Fixed_One
  long[Dest][1] := long[Mat][1] + (long[Mat][0] * -Rz + long[Mat][2] *  Rx) / Fixed_One
  long[Dest][2] := long[Mat][2] + (long[Mat][0] *  Ry + long[Mat][1] * -Rx) / Fixed_One
  
  long[Dest][3] := long[Mat][3] + (long[Mat][4] *  Rz + long[Mat][5] * -Ry) / Fixed_One
  long[Dest][4] := long[Mat][4] + (long[Mat][3] * -Rz + long[Mat][5] *  Rx) / Fixed_One
  long[Dest][5] := long[Mat][5] + (long[Mat][3] *  Ry + long[Mat][4] * -Rx) / Fixed_One

  long[Dest][6] := long[Mat][6] + (long[Mat][7] *  Rz + long[Mat][8] * -Ry) / Fixed_One
  long[Dest][7] := long[Mat][7] + (long[Mat][6] * -Rz + long[Mat][8] *  Rx) / Fixed_One
  long[Dest][8] := long[Mat][8] + (long[Mat][6] *  Ry + long[Mat][7] * -Rx) / Fixed_One


PUB RenormalizeMatrix( Mat ) | OrthoError

  'FixedVector xRow = new FixedVector(m[0, 0], m[0, 1], m[0, 2]);
  'FixedVector yRow = new FixedVector(m[1, 0], m[1, 1], m[1, 2]);
   
  OrthoError := -DotVector( Mat , Mat+12 )            'Will be zero if there's no error
   
  'Scale the error term by half
  OrthoError /= 2
   
  'Scale rows x and y by the error
  'FixedVector xErr = xRow.Scale( error );
  XErr[0] := (long[Mat][0] * OrthoError) ~> FixedBits
  XErr[1] := (long[Mat][1] * OrthoError) ~> FixedBits
  XErr[2] := (long[Mat][2] * OrthoError) ~> FixedBits

  'FixedVector yErr = yRow.Scale( error );
  YErr[0] := (long[Mat][3] * OrthoError) ~> FixedBits
  YErr[1] := (long[Mat][4] * OrthoError) ~> FixedBits
  YErr[2] := (long[Mat][5] * OrthoError) ~> FixedBits
   
  'Add the error corrections to the opposite row vector
  'xRow = xRow.Add(yErr)
  long[Mat][0] += YErr[0]
  long[Mat][1] += YErr[1]
  long[Mat][2] += YErr[2]
  
  'yRow = yRow.Add(xErr)
  long[Mat][3] += XErr[0]
  long[Mat][4] += XErr[1]
  long[Mat][5] += XErr[2]

  'Normalize the X and Y rows   
  'xRow = xRow.Normalized()
  NormalizeVector( Mat + 0 )

  'yRow = yRow.Normalized()
  NormalizeVector( Mat + 12 )
   
  'FixedVector zRow = xRow.Cross( yRow );
  CrossVector( Mat+0 , Mat+12 , Mat+24 ) 
  


'In the following code I use divide instead of shift right in some cases because it's more accurate for negative
'numbers.  -1 arithmetic shift right 1 is still -1, which isn't true for positive numbers.  Divide corrects that
'even though it's a bit slower.

PUB NormalizeVector( Vect ) | Scale , SquareLen

  SquareLen := DotVector( Vect, Vect ) 
  Scale := (constant(3 << FixedBits) - SquareLen) ~> 1
  long[Vect][0] := (long[Vect][0] * Scale) / Fixed_One
  long[Vect][1] := (long[Vect][1] * Scale) / Fixed_One
  long[Vect][2] := (long[Vect][2] * Scale) / Fixed_One


PUB DotVector( V1, V2 )
  result := ((long[V1][0] * long[V2][0]) + (long[V1][1] * long[V2][1]) + (long[V1][2] * long[V2][2])) / Fixed_One


PUB CrossVector( V1, V2, Dest )

  'res.v[0] = (lv[1] * rv[2] - lv[2] * rv[1]) >> FixedMatrix.Bits;
  'res.v[1] = (lv[2] * rv[0] - lv[0] * rv[2]) >> FixedMatrix.Bits;
  'res.v[2] = (lv[0] * rv[1] - lv[1] * rv[0]) >> FixedMatrix.Bits;

  long[Dest][0] := (long[V1][1] * long[V2][2] - long[V1][2] * long[V2][1]) ~> FixedBits
  long[Dest][1] := (long[V1][2] * long[V2][0] - long[V1][0] * long[V2][2]) ~> FixedBits
  long[Dest][2] := (long[V1][0] * long[V2][1] - long[V1][1] * long[V2][0]) ~> FixedBits
   