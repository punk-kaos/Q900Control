#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
REF="d28164b39ba4d91ad5948ff22707937f8944f70f"
DVM_REF="435ff45b367887ffa94c89cb1a2779628e0f7bfb"
WORK="${TMPDIR:-/tmp}/q900-opendmr-fixed"
DVM_WORK="${TMPDIR:-/tmp}/q900-dvmhost-vocoder"

rm -rf "$WORK" "$DVM_WORK"
git clone --quiet https://github.com/MW0MWZ/OpenDMR.git "$WORK"
git -C "$WORK" checkout --quiet "$REF"
git clone --quiet https://github.com/DVMProject/dvmhost.git "$DVM_WORK"
git -C "$DVM_WORK" checkout --quiet "$DVM_REF"

cp "$DVM_WORK/src/vocoder/ambe3600x2250.c" "$WORK/decoder/ambe3600x2250_q900.c"
sed -i.bak 's#"vocoder/mbe.h"#"mbelib.h"#' "$WORK/decoder/ambe3600x2250_q900.c"
rm -f "$WORK/decoder/ambe3600x2250_q900.c.bak"

python3 - "$WORK" <<'PY'
from pathlib import Path
import sys

root = Path(sys.argv[1])
mbe = root / "encoder" / "mbeenc.cpp"
api = root / "opendmr.cpp"
hdr = root / "decoder" / "mbelib.h"
makefile = root / "Makefile"
state_src = root / "decoder" / "ambe3600x2250_q900.c"

# DVMHost's file also contains tone helpers that depend on its older public
# mbe_tone type. Q900Control only needs the DMR predictor dequantizer; keep that
# translation unit deliberately minimal so it is compiled against mbelib-neo's
# current mbe_parms ABI.
t = state_src.read_text()
if '#include "mbelib.h"' not in t:
    raise SystemExit("adapted DVMHost 2250 source is missing mbelib include")
t = t.replace('#include "mbelib.h"', '#include "mbelib.h"\n#include "ambe3600x2450_const.h"', 1)
# mbelib-neo keeps the DMR quantizer tables translation-unit local in this
# internal header, so remove DVMHost's external declarations and compile a
# private copy of the same constants into the predictor helper.
t = "\n".join(
    line for line in t.splitlines()
    if not line.lstrip().startswith("extern const ")
) + "\n"
marker = "int mbe_dequantizeAmbeTone"
pos = t.find(marker)
if pos < 0:
    raise SystemExit("DVMHost 2250 source no longer contains expected tone helper marker")
state_src.write_text(t[:pos])

s = hdr.read_text()
anchor = "MBE_API int mbe_decodeAmbe2450Parms(char* ambe_d, mbe_parms* cur_mp, mbe_parms* prev_mp);"
if s.count(anchor) != 1:
    raise SystemExit("OpenDMR mbelib header no longer matches pinned source")
s = s.replace(anchor, anchor + "\nMBE_API int mbe_dequantizeAmbe2250Parms(mbe_parms* cur_mp, mbe_parms* prev_mp, const int* b);")
hdr.write_text(s)

s = makefile.read_text()
anchor = "               decoder/ambe3600x2450.c \\\n"
if s.count(anchor) != 1:
    raise SystemExit("OpenDMR Makefile decoder list no longer matches pinned source")
s = s.replace(anchor, anchor + "               decoder/ambe3600x2250_q900.c \\\n")
makefile.write_text(s)

s = mbe.read_text()
old = """	/* Encode voice/unvoiced (b[1]) */
	float en_min = 0;
	b[1] = 0;
	for (int n = 0; n < 17; n++) {
		float En = 0;
		for (int l = 1; l <= L; l++) {
			int jl = (int)((float)l * 16.0f * AmbeW0table[b[0]]);
			if (jl > 7) jl = 7;
			if (imbe_param->v_uv_dsn[l-1] != AmbeVuv[n][jl])
				En += m_float2[l-1];
		}
		if (n == 0)
			en_min = En;
		else if (En < en_min) {
			b[1] = n;
			en_min = En;
		}
	}
"""
new = """	/* Encode voice/unvoiced (b[1]).
	 * Keep the DMR harmonic grouping from the OP25 source.  IMBE's
	 * v_uv_dsn analysis vector is grouped in threes here; indexing it
	 * directly by harmonic number destroys the AMBE voicing pattern. */
	float en_min = 0;
	b[1] = 0;
	for (int n = 0; n < 17; n++) {
		float En = 0;
		for (int l = 1; l <= L; l++) {
			int jl = (int)((float)l * 16.0f * AmbeW0table[b[0]]);
			if (jl > 7) jl = 7;
			int kl = 12;
			if (l <= 36)
				kl = (l + 2) / 3;
			if (imbe_param->v_uv_dsn[(kl - 1) * 3] != AmbeVuv[n][jl])
				En += m_float2[l-1];
		}
		if (n == 0)
			en_min = En;
		else if (En < en_min) {
			b[1] = n;
			en_min = En;
		}
	}
"""
if s.count(old) != 1:
    raise SystemExit("OpenDMR voiced/unvoiced block no longer matches pinned source")
s = s.replace(old, new)
if s.count(": d_gain_adjust(1.0f)") != 1:
    raise SystemExit("OpenDMR encoder default gain no longer matches pinned source")
s = s.replace(": d_gain_adjust(1.0f)", ": d_gain_adjust(2.5f)")

old_prng = "unsigned int p = PRNG_TABLE[aOrig] >> 1;"
new_prng = """/* DMR scrambles the 23-bit B codeword with an LCG mask selected by
	 * the full 12-bit A payload. The pinned OpenDMR table has only 248 entries,
	 * so indexing it with aOrig reads out of bounds for most voice frames. */
	uint16_t pr = (uint16_t)(16U * aOrig);
	unsigned int p = 0U;
	for (unsigned int i = 0U; i < 23U; i++) {
		pr = (uint16_t)((173U * (unsigned int)pr + 13849U) & 0xFFFFU);
		p = (p << 1) | (pr >> 15);
	}"""
if s.count(old_prng) != 1:
    raise SystemExit("OpenDMR B-block PRNG code no longer matches pinned source")
s = s.replace(old_prng, new_prng)

old_state = """	/* Update decoder state with quantized values */
	uint8_t ambe_49[49];
	encode_49bit(ambe_49, b);

	char ambe_d[49];
	for (int i = 0; i < 49; i++) {
		ambe_d[i] = ambe_49[i];
	}

	mbe_decodeAmbe2450Parms(ambe_d, cur_mp, prev_mp);
	mbe_moveMbeParms(cur_mp, prev_mp);
"""
new_state = """	/* Update the encoder predictor with the same DMR dequantizer used by
	 * OP25/Dudestar/DVMHost. The 2450 decoder path is not an equivalent
	 * predictor-state update and causes spectral prediction to drift. */
	mbe_dequantizeAmbe2250Parms(cur_mp, prev_mp, b);
	mbe_moveMbeParms(cur_mp, prev_mp);
"""
if s.count(old_state) != 1:
    raise SystemExit("OpenDMR predictor-state block no longer matches pinned source")
s = s.replace(old_state, new_state)
mbe.write_text(s)

s = api.read_text()
if s.count('static const char *version_string = "1.0.0";') != 1:
    raise SystemExit("OpenDMR version string no longer matches pinned source")
s = s.replace('static const char *version_string = "1.0.0";',
              'static const char *version_string = "1.0.0-q900fix5";')

# The upstream public encoder claims to return DVSI/canonical A+B+C frames, but
# it serializes b[0]..b[8] consecutively instead of using DMR's 49-bit parameter
# permutation, and it does not drop the extended Golay parity bit from B. Patch
# that public path so Q900Control can use opendmr_encode() directly and stop
# depending on private/mangled MBEEncoder C++ symbols.
old_pack = """    /* Pack b[9] into 49 bits */
    static const int b_lengths[9] = {7, 5, 5, 9, 7, 5, 4, 4, 3};
    uint8_t bits49[49];
    int pos = 0;
    for (int i = 0; i < 9; i++) {
        int val = b[i];
        for (int j = b_lengths[i] - 1; j >= 0; j--) {
            bits49[pos++] = (val >> j) & 1;
        }
    }
"""
new_pack = """    /* DMR AMBE 3600x2450 parameter permutation (OP25/OpenDMR encode_49bit). */
    uint8_t bits49[49] = {0};
    bits49[0] = (b[0] >> 6) & 1; bits49[1] = (b[0] >> 5) & 1;
    bits49[2] = (b[0] >> 4) & 1; bits49[3] = (b[0] >> 3) & 1;
    bits49[4] = (b[1] >> 4) & 1; bits49[5] = (b[1] >> 3) & 1;
    bits49[6] = (b[1] >> 2) & 1; bits49[7] = (b[1] >> 1) & 1;
    bits49[8] = (b[2] >> 4) & 1; bits49[9] = (b[2] >> 3) & 1;
    bits49[10] = (b[2] >> 2) & 1; bits49[11] = (b[2] >> 1) & 1;
    bits49[12] = (b[3] >> 8) & 1; bits49[13] = (b[3] >> 7) & 1;
    bits49[14] = (b[3] >> 6) & 1; bits49[15] = (b[3] >> 5) & 1;
    bits49[16] = (b[3] >> 4) & 1; bits49[17] = (b[3] >> 3) & 1;
    bits49[18] = (b[3] >> 2) & 1; bits49[19] = (b[3] >> 1) & 1;
    bits49[20] = (b[4] >> 6) & 1; bits49[21] = (b[4] >> 5) & 1;
    bits49[22] = (b[4] >> 4) & 1; bits49[23] = (b[4] >> 3) & 1;
    bits49[24] = (b[5] >> 4) & 1; bits49[25] = (b[5] >> 3) & 1;
    bits49[26] = (b[5] >> 2) & 1; bits49[27] = (b[5] >> 1) & 1;
    bits49[28] = (b[6] >> 3) & 1; bits49[29] = (b[6] >> 2) & 1;
    bits49[30] = (b[6] >> 1) & 1; bits49[31] = (b[7] >> 3) & 1;
    bits49[32] = (b[7] >> 2) & 1; bits49[33] = (b[7] >> 1) & 1;
    bits49[34] = (b[8] >> 2) & 1; bits49[35] = b[1] & 1;
    bits49[36] = b[2] & 1; bits49[37] = (b[0] >> 2) & 1;
    bits49[38] = (b[0] >> 1) & 1; bits49[39] = b[0] & 1;
    bits49[40] = b[3] & 1; bits49[41] = (b[4] >> 2) & 1;
    bits49[42] = (b[4] >> 1) & 1; bits49[43] = b[4] & 1;
    bits49[44] = b[5] & 1; bits49[45] = b[6] & 1;
    bits49[46] = b[7] & 1; bits49[47] = (b[8] >> 1) & 1;
    bits49[48] = b[8] & 1;
"""
if s.count(old_pack) != 1:
    raise SystemExit("OpenDMR public AMBE packer no longer matches pinned source")
s = s.replace(old_pack, new_pack)

old_b = "    uint32_t b_codeword = CGolay24128::encode23127(c1);"
new_b = "    uint32_t b_codeword = CGolay24128::encode23127(c1) >> 1;"
if s.count(old_b) != 1:
    raise SystemExit("OpenDMR public B Golay code no longer matches pinned source")
s = s.replace(old_b, new_b)

# Stable test hook for verifying canonical frame construction without relying on
# private C++ ABI. Runtime TX uses opendmr_encode(); this helper is CI-only.
hook_anchor = """bool opendmr_encode(opendmr_encoder_t *enc,
                    const int16_t pcm[OPENDMR_PCM_SAMPLES],
                    uint8_t ambe[OPENDMR_AMBE_FRAME_BYTES])
"""
hook = """extern "C" bool opendmr_q900_encode_params(const int b[9], uint8_t ambe[OPENDMR_AMBE_FRAME_BYTES])
{
    if (!b || !ambe)
        return false;
    encode_ambe_frame(b, ambe);
    return true;
}

""" + hook_anchor
if s.count(hook_anchor) != 1:
    raise SystemExit("OpenDMR public encode function anchor no longer matches pinned source")
s = s.replace(hook_anchor, hook)

# USRP2DMR, a long-running OP25-derived DMR bridge, explicitly uses a 2.5
# AMBE model gain adjustment. Keep that operating point for normal Q900 TX,
# including after opendmr_encoder_reset(); DmrVoiceTransmitter resets at every
# call start. Non-zero diagnostic values retain the existing q900 dB sweep
# conversion so prior recordings remain directly comparable.
pairs = [
    ("enc->enc->set_gain_adjust(1.0f);", "enc->enc->set_gain_adjust(2.5f);"),
    ("enc->enc->set_gain_adjust(powf(10.0f, enc->gain_db / 20.0f));",
     "enc->enc->set_gain_adjust(enc->gain_db == 0 ? 2.5f : -((float)enc->gain_db / 6.020599913f));"),
]
for old, new in pairs:
    count = s.count(old)
    if count < 1:
        raise SystemExit(f"OpenDMR gain block no longer matches pinned source: {old}")
    s = s.replace(old, new)
api.write_text(s)
PY

JOBS=2
if command -v getconf >/dev/null 2>&1; then
  JOBS="$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 2)"
fi
make -C "$WORK" -j"$JOBS" >/dev/null

case "$(uname -s)" in
  Darwin) EXT=dylib ;;
  Linux)  EXT=so ;;
  *) echo "Unsupported platform: $(uname -s)" >&2; exit 1 ;;
esac

OUT="$ROOT/libopendmr-q900fix.$EXT"
cp "$WORK/libopendmr.$EXT" "$OUT"
echo "Built $OUT"
echo "Q900Control will prefer this fixed library automatically."
