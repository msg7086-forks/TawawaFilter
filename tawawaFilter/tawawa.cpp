/*
Copyright (c) 2016, sorayuki
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of this program nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <windows.h>
#include <Avisynth.h>

class TawawaFilter : public GenericVideoFilter
{
public:
	TawawaFilter(PClip child, IScriptEnvironment* env)
		: GenericVideoFilter(child)
	{
		if (!vi.IsYV12())
			env->ThrowError("TawawaFilter: Only YV12 input is supported.");
		if (vi.width & 15)
			env->ThrowError("TawawaFilter: Width must be mod16");
		if (vi.height & 1)
			env->ThrowError("TawawaFilter: Height must be even");
	}

	PVideoFrame __stdcall GetFrame(int n, IScriptEnvironment* env) override
	{
		PVideoFrame frame = child->GetFrame(n, env);
		PVideoFrame newFrame = env->NewVideoFrame(vi);
		
		const unsigned char* pSrc = frame->GetReadPtr(PLANAR_Y);
		unsigned char* pDstY = newFrame->GetWritePtr(PLANAR_Y);
		unsigned char* pDstU = newFrame->GetWritePtr(PLANAR_U);
		unsigned char* pDstV = newFrame->GetWritePtr(PLANAR_V);

		int srcPitch = frame->GetPitch(PLANAR_Y);
		int dstPitchY = newFrame->GetPitch(PLANAR_Y);
		int dstPitchU = newFrame->GetPitch(PLANAR_U);
		int dstPitchV = newFrame->GetPitch(PLANAR_V);

		for (int i = 0; i < vi.height; i += 2)
		{
			auto cpSrc = pSrc;
			auto cpDstY = pDstY;
			auto cpDstU = pDstU;
			auto cpDstV = pDstV;
			for (int j = 0; j < vi.width; j += 2)
			{
				double u1, u2, u3, u4;
				double v1, v2, v3, v4;
				ComputePixel(cpSrc[0], cpDstY[0], u1, v1);
				ComputePixel(cpSrc[1], cpDstY[1], u2, v2);
				ComputePixel(cpSrc[srcPitch], cpDstY[dstPitchY], u3, v3);
				ComputePixel(cpSrc[srcPitch + 1], cpDstY[dstPitchY + 1], u4, v4);

				cpDstU[0] = (u1 + u2 + u3 + u4) / 4;
				cpDstV[0] = (v1 + v2 + v3 + v4) / 4;

				cpSrc += 2;
				cpDstY += 2;
				cpDstU += 1;
				cpDstV += 1;
			}
			pSrc += srcPitch << 1;
			pDstY += dstPitchY << 1;
			pDstU += dstPitchU;
			pDstV += dstPitchV;
		}

		return newFrame;
	}

	void ComputePixel(double sy, unsigned char& dy, double& du, double& dv)
	{
		double y = (sy - 16) * 200 / 219 + 55;
		if (y > 255) y = 255;
		double r = (y - 85) * 340 / 255;
		if (r < 0) r = 0;
		double b = y + 120;
		if (b > 255) b = 255;
		dy = (0.299 * r + 0.587 * y + 0.114 * b);
		if (dy < 0) dy = 0;
		if (dy > 255) dy = 255;
		du = (-0.14713 * r - 0.28886 * y + 0.436 * b + 128);
		if (du < 0) du = 0;
		if (du > 255) du = 255;
		dv = (0.615 * r - 0.51499 * y - 0.10001 * b + 128);
		if (dv < 0) dv = 0;
		if (dv > 255) dv = 255;
	}
};

AVSValue __cdecl CreateTawawaFilter(AVSValue args, void* user_data, IScriptEnvironment* env)
{
	return new TawawaFilter(args[0].AsClip(), env);
}

const AVS_Linkage *AVS_linkage = NULL;

extern "C" __declspec(dllexport) const char* __stdcall AvisynthPluginInit3(IScriptEnvironment* env, AVS_Linkage* vectors)
{
	AVS_linkage = vectors;
	env->AddFunction("Tawawa", "c", CreateTawawaFilter, 0);
	return "TawawaFilter";
}
