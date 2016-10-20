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
#include <intrin.h>

class TawawaFilter : public GenericVideoFilter
{
	int m_y[256];
	int m_u[256]; // = u * 4
	int m_v[256]; // = v * 4
public:
	TawawaFilter(PClip child, IScriptEnvironment* env)
		: GenericVideoFilter(child)
	{
		if (!vi.IsYV12())
			env->ThrowError("TawawaFilter: Only YV12 input is supported.");
		if (vi.width & 31)
			env->ThrowError("TawawaFilter: Width must be mod32");
		if (vi.height & 1)
			env->ThrowError("TawawaFilter: Height must be even");
		for (int i = 0; i < 256; i++)
		{
			ComputePixel(i, m_y[i], m_u[i], m_v[i]);
		}
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
			for (int j = 0; j < vi.width; j += 16)
			{
				auto row1_raw = _mm_load_si128((__m128i const*)cpSrc);
				auto row2_raw = _mm_load_si128((__m128i const*)(cpSrc + srcPitch));
				auto zero = _mm_setzero_si128();
				__m128i idx1, idx2, idx3, idx4;
				__m128i u1, u2, u3, u4;
				__m128i v1, v2, v3, v4;
				// Row1 Y
				_mm_spread(row1_raw, zero, idx1, idx2, idx3, idx4);
				idx1 = _mm_luty(idx1);
				idx2 = _mm_luty(idx2);
				idx3 = _mm_luty(idx3);
				idx4 = _mm_luty(idx4);
				u1 = _mm_lutuv(idx1, PLANAR_U);
				u2 = _mm_lutuv(idx2, PLANAR_U);
				u3 = _mm_lutuv(idx3, PLANAR_U);
				u4 = _mm_lutuv(idx4, PLANAR_U);
				v1 = _mm_lutuv(idx1, PLANAR_V);
				v2 = _mm_lutuv(idx2, PLANAR_V);
				v3 = _mm_lutuv(idx3, PLANAR_V);
				v4 = _mm_lutuv(idx4, PLANAR_V);
				auto u1s = _mm_hadd_epi32(u1, u2);
				auto u2s = _mm_hadd_epi32(u3, u4);
				auto v1s = _mm_hadd_epi32(v1, v2);
				auto v2s = _mm_hadd_epi32(v3, v4);
				row1_raw = _mm_mergey(idx1, idx2, idx3, idx4);
				_mm_store_si128((__m128i*)cpDstY, row1_raw);
				// Row2 Y
				_mm_spread(row2_raw, zero, idx1, idx2, idx3, idx4);
				idx1 = _mm_luty(idx1);
				idx2 = _mm_luty(idx2);
				idx3 = _mm_luty(idx3);
				idx4 = _mm_luty(idx4);
				u1 = _mm_lutuv(idx1, PLANAR_U);
				u2 = _mm_lutuv(idx2, PLANAR_U);
				u3 = _mm_lutuv(idx3, PLANAR_U);
				u4 = _mm_lutuv(idx4, PLANAR_U);
				v1 = _mm_lutuv(idx1, PLANAR_V);
				v2 = _mm_lutuv(idx2, PLANAR_V);
				v3 = _mm_lutuv(idx3, PLANAR_V);
				v4 = _mm_lutuv(idx4, PLANAR_V);
				u1s = _mm_add_epi32(u1s, _mm_hadd_epi32(u1, u2));
				u2s = _mm_add_epi32(u2s, _mm_hadd_epi32(u3, u4));
				v1s = _mm_add_epi32(v1s, _mm_hadd_epi32(v1, v2));
				v2s = _mm_add_epi32(v2s, _mm_hadd_epi32(v3, v4));
				row2_raw = _mm_mergey(idx1, idx2, idx3, idx4);
				_mm_store_si128((__m128i*)(cpDstY + dstPitchY), row2_raw);
				// UV
				auto u_raw = _mm_mergeuv(u1s, u2s, zero);
				_mm_storel_epi64((__m128i*)cpDstU, u_raw);
				// V
				auto v_raw = _mm_mergeuv(v1s, v2s, zero);
				_mm_storel_epi64((__m128i*)cpDstV, v_raw);

				cpSrc += 16;
				cpDstY += 16;
				cpDstU += 8;
				cpDstV += 8;
			}
			pSrc += srcPitch << 1;
			pDstY += dstPitchY << 1;
			pDstU += dstPitchU;
			pDstV += dstPitchV;
		}

		return newFrame;
	}

	void ComputePixel(int sy, int& dy, int& du, int& dv)
	{
		double y = (sy - 16) * 200 / 219 + 55;
		if (y > 255) y = 255;
		double r = (y - 85) * 340 / 255;
		if (r < 0) r = 0;
		double b = y + 120;
		if (b > 255) b = 255;
		dy = (0.2126 * r + 0.7152 * y + 0.0722 * b);
		if (dy < 0) dy = 0;
		if (dy > 255) dy = 255;
		du = 4.0 * (-0.09991 * r - 0.33609 * y + 0.436 * b + 128);
		if (du < 0) du = 0;
		if (du > 1023) du = 1023;
		dv = 4.0 * (0.615 * r - 0.55861 * y - 0.05639 * b + 128);
		if (dv < 0) dv = 0;
		if (dv > 1023) dv = 1023;
	}

	void _mm_spread(__m128i& raw, __m128i& zero, __m128i& idx1, __m128i& idx2, __m128i& idx3, __m128i& idx4)
	{
		auto data_16_1 = _mm_unpacklo_epi8(raw, zero);
		auto data_16_2 = _mm_unpackhi_epi8(raw, zero);
		idx1 = _mm_unpacklo_epi16(data_16_1, zero);
		idx2 = _mm_unpackhi_epi16(data_16_1, zero);
		idx3 = _mm_unpacklo_epi16(data_16_2, zero);
		idx4 = _mm_unpackhi_epi16(data_16_2, zero);
	}
	__m128i _mm_luty(__m128i& idx)
	{
		// __m128i _mm_i32gather_epi32 (int const* base_addr, __m128i vindex, const int scale)
		return _mm_i32gather_epi32(m_y, idx, sizeof(int));
	}
	__m128i _mm_lutuv(__m128i& idx, int plane)
	{
		return _mm_i32gather_epi32(plane == PLANAR_U ? m_u : m_v, idx, sizeof(int));
	}
	__m128i _mm_mergey(__m128i& data1, __m128i& data2, __m128i& data3, __m128i& data4)
	{
		auto data_16_1 = _mm_packus_epi32(data1, data2);
		auto data_16_2 = _mm_packus_epi32(data3, data4);
		return _mm_packus_epi16(data_16_1, data_16_2);
	}
	__m128i _mm_mergeuv(__m128i& data1, __m128i& data2, __m128i& zero)
	{
		auto data_16_1 = _mm_packus_epi32(data1, data2);
		data_16_1 = _mm_srli_epi16(data_16_1, 4);
		return _mm_packus_epi16(data_16_1, zero);
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
