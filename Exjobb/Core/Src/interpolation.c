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


float CalcPoly23Error(float x, float y, struct poly c)
{
	return  (c.p00 + c.p10*x + c.p01*y + c.p20*x*x + c.p11*x*y + c.p02*y*y + c.p21*x*x*y
	                    + c.p12*x*y*y + c.p03*y*y*y);
}

float CalcPoly32Error(float x, float y, struct poly c)
{
	return (c.p00 + c.p10*x + c.p01*y + c.p20*x*x + c.p11*x*y + c.p02*y*y + c.p30*x*x*x +
	                    c.p21*x*x*y + c.p12*x*y*y);
}
