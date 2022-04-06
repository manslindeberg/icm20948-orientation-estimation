/* MIT License

Copyright (c) 2022 Lindeberg, M & Hansson, L

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE. */

#ifndef SRC_INTERPOLATION_H_
#define SRC_INTERPOLATION_H_

/* Madgwick Calibration Interpolation from MATLAB script*/
#define MG00P -1.774
#define MG10P 0.007119
#define MG01P 0.00967
#define MG20P -5.38e-06
#define MG11P -7.878e-06
#define MG02P 0.0001361
#define MG30P -1.487e-07
#define MG21P 6.914e-08
#define MG12P -3.358e-07

#define MG00T 0.8559
#define MG10T 1.768e-06
#define MG01T 0.04034
#define MG20T 3.295e-05
#define MG11T 3.532e-06
#define MG02T -0.0001032
#define MG21T 2.714e-07
#define MG12T 7.007e-08
#define MG03T -4.74e-06

struct poly {
	float p00;
	float p01;
	float p02;
	float p03;
	float p30;
	float p20;
	float p10;
	float p11;
	float p12;
	float p13;
	float p31;
	float p21;
	float p22;
	float p23;
	float p32;
	float p33;
};

float CalcPoly23Error(float x, float y);
float CalcPoly32Error(float x, float y);

#endif /* SRC_INTERPOLATION_H_ */
