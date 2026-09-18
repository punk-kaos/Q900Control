#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
REF="d28164b39ba4d91ad5948ff22707937f8944f70f"
WORK="${TMPDIR:-/tmp}/q900-opendmr-fixed"

rm -rf "$WORK"
git clone --quiet https://github.com/MW0MWZ/OpenDMR.git "$WORK"
git -C "$WORK" checkout --quiet "$REF"

python3 - "$WORK" <<'PY'
from pathlib import Path
import sys

root = Path(sys.argv[1])
mbe = root / "encoder" / "mbeenc.cpp"
api = root / "opendmr.cpp"

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
s = s.replace(": d_gain_adjust(1.0f)", ": d_gain_adjust(0.0f)")
mbe.write_text(s)

s = api.read_text()
if s.count('static const char *version_string = "1.0.0";') != 1:
    raise SystemExit("OpenDMR version string no longer matches pinned source")
s = s.replace('static const char *version_string = "1.0.0";',
              'static const char *version_string = "1.0.0-q900fix1";')
pairs = [
    ("enc->enc->set_gain_adjust(1.0f);", "enc->enc->set_gain_adjust(0.0f);"),
    ("enc->enc->set_gain_adjust(powf(10.0f, enc->gain_db / 20.0f));",
     "enc->enc->set_gain_adjust(-((float)enc->gain_db / 6.020599913f));"),
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
