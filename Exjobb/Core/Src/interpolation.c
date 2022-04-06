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


#include "interpolation.h"

struct poly c23 = {MG00T,MG01T,MG02T,MG03T,0,MG20T,MG10T,MG11T,MG12T,0,0,MG21T,0,0,0,0};
struct poly c32 = {MG00P,MG01P,MG02P,0,MG30P,MG20P,MG10P,MG11P,MG12P,0,0,MG21P,0,0,0,0};

float CalcPoly23Error(float x, float y)
{
	return  (c23.p00 + c23.p10*x + c23.p01*y + c23.p20*x*x + c23.p11*x*y + c23.p02*y*y +
			c23.p21*x*x*y + c23.p12*x*y*y + c23.p03*y*y*y);
}

float CalcPoly32Error(float x, float y)
{
	return (c32.p00 + c32.p10*x + c32.p01*y + c32.p20*x*x + c32.p11*x*y + c32.p02*y*y +
			c32.p30*x*x*x + c32.p21*x*x*y + c32.p12*x*y*y);
}
