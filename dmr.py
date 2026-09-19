"""DMR Tier II air-interface support for Q900Control SDR mode."""
from __future__ import annotations
from collections import deque
from dataclasses import dataclass
import ctypes, json, os, wave
from pathlib import Path
from typing import Callable, Iterable
import numpy as np

SAMPLE_RATE=48000; SYMBOL_RATE=4800; SPS=10
SYMBOL_DEVIATION_HZ=648.0; BURST_BITS=264; BURST_SYMBOLS=132
BURST_SAMPLES=1320; SLOT_SAMPLES=2880; AMBE_BITS=72; AMBE_BYTES=9
DMR_TX_PREAMBLE_MS=int(os.getenv("Q900_DMR_TX_PREAMBLE_MS","200") or 200)
if not 0<=DMR_TX_PREAMBLE_MS<=1000: raise ValueError("Q900_DMR_TX_PREAMBLE_MS must be 0..1000")
SYNC_WORDS={
 "BS_VOICE":0x755FD7DF75F7,"BS_DATA":0xDFF57D75DF5D,
 "MS_VOICE":0x7F7D5DD57DFD,"MS_DATA":0xD5D7F77FD757,
 "DIRECT1_VOICE":0x5D577F7757FF,"DIRECT1_DATA":0xF7FDD5DDFD55,
 "DIRECT2_VOICE":0x7DFFD5F55D5F,"DIRECT2_DATA":0xD7557F5FF7F5,
}
VOICE_SYNCS={k:v for k,v in SYNC_WORDS.items() if k.endswith("VOICE")}
DATA_SYNCS={k:v for k,v in SYNC_WORDS.items() if k.endswith("DATA")}
DT_VOICE_LC_HEADER=1; DT_TERMINATOR_WITH_LC=2; FLCO_GROUP=0; FLCO_PRIVATE=3
DMR_A=(0,4,8,12,16,20,24,28,32,36,40,44,48,52,56,60,64,68,1,5,9,13,17,21)
DMR_B=(25,29,33,37,41,45,49,53,57,61,65,69,2,6,10,14,18,22,26,30,34,38,42)
DMR_C=(46,50,54,58,62,66,70,3,7,11,15,19,23,27,31,35,39,43,47,51,55,59,63,67,71)
AMBE_TABLE=DMR_A+DMR_B+DMR_C
DMO_FILL=bytes.fromhex("63 EA 00 76 6C 76 C4 52 C8 78 09 2D B8 79 27 57 9B 31 BC 3E EA 45 C3 30 49 17 93 AE 8B 6D A4 A5 AD A2 F1 35 B5 3C 1E")
G2087=np.asarray([
[1,0,0,0,0,0,0,0,0,0,1,1,1,1,0,1,1,0,1,0],[0,1,0,0,0,0,0,0,1,1,0,1,1,0,0,1,1,0,0,1],
[0,0,1,0,0,0,0,0,0,1,1,0,1,1,0,0,1,1,0,1],[0,0,0,1,0,0,0,0,0,0,1,1,0,1,1,0,0,1,1,1],
[0,0,0,0,1,0,0,0,1,1,0,1,1,1,0,0,0,1,1,0],[0,0,0,0,0,1,0,0,1,0,1,0,1,0,0,1,0,1,1,1],
[0,0,0,0,0,0,1,0,1,0,0,1,0,0,1,1,1,1,1,0],[0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,0,1,0,1,1]],dtype=np.uint8)
QR1676=np.asarray([
[1,0,0,0,0,0,0,0,0,1,0,0,1,1,1,1],[0,1,0,0,0,0,0,1,0,0,0,1,1,1,1,0],
[0,0,1,0,0,0,0,1,1,0,1,1,0,1,1,1],[0,0,0,1,0,0,0,1,1,1,1,0,0,0,1,0],
[0,0,0,0,1,0,0,1,1,1,0,0,1,0,0,1],[0,0,0,0,0,1,0,0,1,1,1,0,0,1,0,1],
[0,0,0,0,0,0,1,0,0,1,1,1,0,0,1,1]],dtype=np.uint8)

def int_bits(v,n): return np.asarray([(v>>(n-1-i))&1 for i in range(n)],dtype=np.uint8)
def bytes_bits(b): return np.unpackbits(np.frombuffer(b,dtype=np.uint8),bitorder="big")
def bits_bytes(b):
 b=np.asarray(b,dtype=np.uint8).reshape(-1)
 if len(b)%8: b=np.pad(b,(0,8-len(b)%8))
 return np.packbits(b,bitorder="big").tobytes()
def sync_bits(v): return int_bits(v,48)
def _menc(v,m,n): return (int_bits(v,n)@m&1).astype(np.uint8)
def golay_encode(v): return _menc(v&255,G2087,8)
_GOLAY=np.stack([golay_encode(i) for i in range(256)])
def golay_decode(b):
 d=np.count_nonzero(_GOLAY!=np.asarray(b,dtype=np.uint8).reshape(20),axis=1); v=int(np.argmin(d)); e=int(d[v])
 if e>3: raise ValueError("uncorrectable Golay(20,8)")
 return v,e
def qr_encode(v): return _menc(v&127,QR1676,7)
_QR=np.stack([qr_encode(i) for i in range(128)])
def qr_decode(b):
 d=np.count_nonzero(_QR!=np.asarray(b,dtype=np.uint8).reshape(16),axis=1); v=int(np.argmin(d)); e=int(d[v])
 if e>2: raise ValueError("uncorrectable QR(16,7)")
 return v,e

def _h1511(d): return np.asarray([d[0]^d[1]^d[2]^d[3]^d[5]^d[7]^d[8],d[1]^d[2]^d[3]^d[4]^d[6]^d[8]^d[9],d[2]^d[3]^d[4]^d[5]^d[7]^d[9]^d[10],d[0]^d[1]^d[2]^d[4]^d[6]^d[7]^d[10]],dtype=np.uint8)
def _h139(d): return np.asarray([d[0]^d[1]^d[3]^d[5]^d[6],d[0]^d[1]^d[2]^d[4]^d[6]^d[7],d[0]^d[1]^d[2]^d[3]^d[5]^d[7]^d[8],d[0]^d[2]^d[4]^d[5]^d[8]],dtype=np.uint8)
def _h1611(d): return np.r_[_h1511(d[:11]),d[0]^d[2]^d[5]^d[6]^d[8]^d[9]^d[10]]
def _correct(w,n,fn):
 s=fn(w[:n])^w[n:]
 if not np.any(s): return 0
 sig=[]
 for i in range(n):
  p=np.zeros(n,dtype=np.uint8);p[i]=1;sig.append(fn(p))
 sig.extend(np.eye(len(w)-n,dtype=np.uint8))
 for i,x in enumerate(sig):
  if np.array_equal(s,x): w[i]^=1; return 1
 return -1
_BPOS=list(range(4,12))+list(range(16,27))+list(range(31,42))+list(range(46,57))+list(range(61,72))+list(range(76,87))+list(range(91,102))+list(range(106,117))+list(range(121,132))
def bptc_encode(payload):
 de=np.zeros(196,dtype=np.uint8);de[np.asarray(_BPOS)]=bytes_bits(payload)
 for r in range(9):
  p=r*15+1;de[p+11:p+15]=_h1511(de[p:p+11])
 for c in range(15):
  ix=c+1+15*np.arange(13);de[ix[9:13]]=_h139(de[ix[:9]])
 raw=np.empty(196,dtype=np.uint8)
 for a in range(196): raw[(a*181)%196]=de[a]
 return raw
def bptc_decode(raw):
 de=np.empty(196,dtype=np.uint8);raw=np.asarray(raw,dtype=np.uint8).reshape(196)
 for a in range(196): de[a]=raw[(a*181)%196]
 corr=0
 for _ in range(4):
  ch=0
  for c in range(15):
   ix=c+1+15*np.arange(13);q=_correct(de[ix],9,_h139);ch+=max(q,0)
  for r in range(9):
   p=r*15+1;q=_correct(de[p:p+15],11,_h1511);ch+=max(q,0)
  corr+=ch
  if not ch: break
 return bits_bytes(de[np.asarray(_BPOS)])[:12],corr

def _gf():
 e=[0]*512;l=[0]*256;x=1
 for i in range(255):
  e[i]=x;l[x]=i;x<<=1
  if x&256:x^=0x11D
 for i in range(255,512):e[i]=e[i-255]
 return e,l
_GE,_GL=_gf()
def _gm(a,b): return 0 if not a or not b else _GE[_GL[a]+_GL[b]]
def rs129(msg):
 p=[0,0,0]
 for b in msg:
  d=b^p[2];p[2]=p[1]^_gm(14,d);p[1]=p[0]^_gm(56,d);p[0]=_gm(64,d)
 return bytes(p)

@dataclass(slots=True)
class LinkControl:
 source:int; destination:int; group:bool=True; service_options:int=0; fid:int=0
 def bytes9(self):
  flco=FLCO_GROUP if self.group else FLCO_PRIVATE
  return bytes((flco,self.fid&255,self.service_options&255))+int(self.destination&0xFFFFFF).to_bytes(3,"big")+int(self.source&0xFFFFFF).to_bytes(3,"big")
 @classmethod
 def from_bytes(cls,d):
  return cls(int.from_bytes(d[6:9],"big"),int.from_bytes(d[3:6],"big"),(d[0]&0x3F)!=FLCO_PRIVATE,d[2],d[1])
def full_lc_payload(lc,typ):
 p=rs129(lc.bytes9());m=0x96 if typ==DT_VOICE_LC_HEADER else 0x99
 return lc.bytes9()+bytes((p[2]^m,p[1]^m,p[0]^m))
def full_lc_decode(p,typ):
 m=0x96 if typ==DT_VOICE_LC_HEADER else 0x99; got=bytes((p[11]^m,p[10]^m,p[9]^m))
 return LinkControl.from_bytes(p[:9]),got==rs129(p[:9])
def build_data_burst(payload,cc,typ,sync):
 info=bptc_encode(payload);slot=golay_encode(((cc&15)<<4)|(typ&15))
 return np.r_[info[:98],slot[:10],sync_bits(sync),slot[10:],info[98:]]
def parse_data_burst(bits):
 bits=np.asarray(bits,dtype=np.uint8).reshape(264);sv,se=golay_decode(np.r_[bits[98:108],bits[156:166]])
 raw=np.r_[bits[:98],bits[166:]];payload,be=bptc_decode(raw);r={"color_code":sv>>4,"data_type":sv&15,"slot_errors":se,"bptc_corrected":be,"payload":payload}
 if r["data_type"] in (1,2):
  lc,ok=full_lc_decode(payload,r["data_type"]);r.update(lc=lc,lc_valid=ok)
 return r

def _can_to_ota(frame):
 c=bytes_bits(frame);o=np.zeros(72,dtype=np.uint8);o[np.asarray(AMBE_TABLE)]=c;return o
def _ota_to_can(ota): return bits_bytes(np.asarray(ota,dtype=np.uint8)[np.asarray(AMBE_TABLE)])[:9]
def ambe_to_ota(frames,center):
 p=np.concatenate([_can_to_ota(x) for x in frames]);return np.r_[p[:108],np.asarray(center,dtype=np.uint8),p[108:]]
def ota_to_ambe(bits):
 p=np.r_[bits[:108],bits[156:]];return tuple(_ota_to_can(p[i:i+72]) for i in range(0,216,72))
def _embedded(lc):
 b=bytes_bits(lc.bytes9());crc=sum(lc.bytes9())%31;d=np.zeros(128,dtype=np.uint8)
 for pos,k in ((106,0),(90,1),(74,2),(58,3),(42,4)):d[pos]=(crc>>k)&1
 src=0
 for st,n in ((0,11),(16,11),(32,10),(48,10),(64,10),(80,10),(96,10)):d[st:st+n]=b[src:src+n];src+=n
 for st in range(0,112,16):d[st+11:st+16]=_h1611(d[st:st+11])
 for c in range(16):d[112+c]=np.bitwise_xor.reduce(d[c:112:16])
 raw=np.zeros(128,dtype=np.uint8);j=0
 for i in range(128):raw[i]=d[j];j+=16;j-=127 if j>127 else 0
 return raw
def embedded_center(lc,cc,idx):
 raw=_embedded(lc);chunk=raw[(idx-1)*32:idx*32] if idx<=4 else np.zeros(32,dtype=np.uint8);lcss=(1,3,3,2,0)[idx-1];q=qr_encode(((cc&15)<<3)|lcss);return np.r_[q[:8],chunk,q[8:]]
def build_voice_burst(frames,lc,cc,idx,slot=1,voice_sync=None):
 if idx==0:
  sync=SYNC_WORDS[f"DIRECT{slot}_VOICE"] if voice_sync is None else int(voice_sync)
  center=sync_bits(sync)
 else:center=embedded_center(lc,cc,idx)
 return ambe_to_ota(frames,center)
def voice_center_info(bits):
 c=np.asarray(bits)[108:156]
 try:v,e=qr_decode(np.r_[c[:8],c[40:]])
 except ValueError:return {"valid":False}
 return {"valid":True,"color_code":(v>>3)&15,"lcss":v&3,"qr_errors":e}

def dibit_levels(bits):
 p=np.asarray(bits,dtype=np.uint8).reshape(-1,2);c=(p[:,0]<<1)|p[:,1];return np.take(np.asarray([-1.,-3.,1.,3.]),c)
def levels_bits(x,center,scale):
 n=(np.asarray(x)-center)/scale;c=np.where(n<-2,1,np.where(n<0,0,np.where(n<2,2,3))).astype(np.uint8);o=np.empty(2*len(c),dtype=np.uint8);o[::2]=c>>1;o[1::2]=c&1;return o
def _rrc(beta=.2,span=8):
 n=np.arange(-span*SPS//2,span*SPS//2+1,dtype=float);t=n/SPS;h=np.empty_like(t)
 for i,x in enumerate(t):
  if abs(x)<1e-12:h[i]=1+beta*(4/np.pi-1)
  elif abs(abs(x)-1/(4*beta))<1e-12:h[i]=(beta/np.sqrt(2))*((1+2/np.pi)*np.sin(np.pi/(4*beta))+(1-2/np.pi)*np.cos(np.pi/(4*beta)))
  else:h[i]=(np.sin(np.pi*x*(1-beta))+4*beta*x*np.cos(np.pi*x*(1+beta)))/(np.pi*x*(1-(4*beta*x)**2))
 return h/h.sum()
RRC=_rrc()
class Dmr4FskModulator:
 def __init__(self,offset_hz=12000,q900_orientation=True):self.offset_hz=offset_hz;self.q900_orientation=q900_orientation;self.phase=0.;self.n=0;self.state=np.zeros(len(RRC)-1)
 def modulate(self,bits):
  lev=dibit_levels(bits);imp=np.zeros(len(lev)*SPS);imp[::SPS]=lev*SPS;c=np.r_[self.state,imp];sh=np.convolve(c,RRC,mode="valid");self.state=c[-(len(RRC)-1):]
  ph=self.phase+np.cumsum(2*np.pi*sh*SYMBOL_DEVIATION_HZ/SAMPLE_RATE);self.phase=float(ph[-1]%(2*np.pi));ix=np.arange(self.n,self.n+len(ph));self.n+=len(ph);z=np.exp(1j*ph)*np.exp(1j*2*np.pi*self.offset_hz*ix/SAMPLE_RATE)
  return np.conj(z).astype(np.complex64) if self.q900_orientation else z.astype(np.complex64)
def dmo_cycle_bits(burst):return np.r_[np.asarray(burst,dtype=np.uint8).reshape(264),bytes_bits(DMO_FILL)]

@dataclass(slots=True)
class DmrConfig:
 source_id:int=0;destination_id:int=0;color_code:int=1;slot:int=1;group:bool=True;tdma_direct:bool=False
 @classmethod
 def from_env(cls):
  return cls(int(os.getenv("Q900_DMR_ID","0")),int(os.getenv("Q900_DMR_TG","0")),int(os.getenv("Q900_DMR_CC","1")),int(os.getenv("Q900_DMR_SLOT","1")),os.getenv("Q900_DMR_PRIVATE","0").lower() not in ("1","true","yes"),os.getenv("Q900_DMR_TDMA_DIRECT","0").lower() in ("1","true","yes"))
 def validate_tx(self):
  if not 1<=self.source_id<=0xFFFFFF:raise ValueError("DMR TX needs Q900_DMR_ID")
  if not 1<=self.destination_id<=0xFFFFFF:raise ValueError("DMR TX needs Q900_DMR_TG")
  if not 0<=self.color_code<=15 or self.slot not in (1,2):raise ValueError("invalid DMR CC/slot")
 def data_sync(self):return SYNC_WORDS[f"DIRECT{self.slot}_DATA"] if self.tdma_direct else SYNC_WORDS["MS_DATA"]
 def voice_sync(self):return SYNC_WORDS[f"DIRECT{self.slot}_VOICE"] if self.tdma_direct else SYNC_WORDS["MS_VOICE"]
 def tx_label(self):return f"TDMA-direct TS{self.slot}" if self.tdma_direct else "MS simplex"

def ambe_params_bits(params):
 p=[int(x) for x in params]
 if len(p)!=9:raise ValueError("AMBE parameter vector must contain 9 values")
 o=np.zeros(49,dtype=np.uint8)
 # DMR AMBE+2 does not serialize b[0]..b[8] consecutively. This is the
 # established OP25/OpenDMR encode_49bit() layout used before Golay/FEC.
 o[0:4]=[(p[0]>>x)&1 for x in (6,5,4,3)]
 o[4:8]=[(p[1]>>x)&1 for x in (4,3,2,1)]
 o[8:12]=[(p[2]>>x)&1 for x in (4,3,2,1)]
 o[12:20]=[(p[3]>>x)&1 for x in (8,7,6,5,4,3,2,1)]
 o[20:24]=[(p[4]>>x)&1 for x in (6,5,4,3)]
 o[24:28]=[(p[5]>>x)&1 for x in (4,3,2,1)]
 o[28:31]=[(p[6]>>x)&1 for x in (3,2,1)]
 o[31:34]=[(p[7]>>x)&1 for x in (3,2,1)]
 o[34]=(p[8]>>2)&1;o[35]=p[1]&1;o[36]=p[2]&1
 o[37:40]=[(p[0]>>x)&1 for x in (2,1,0)];o[40]=p[3]&1
 o[41:44]=[(p[4]>>x)&1 for x in (2,1,0)]
 o[44]=p[5]&1;o[45]=p[6]&1;o[46]=p[7]&1
 o[47]=(p[8]>>1)&1;o[48]=p[8]&1
 return o

def dmr_prng_mask(seed):
 p=(16*int(seed))&0xFFFF;mask=0
 for _ in range(23):p=(173*p+13849)&0xFFFF;mask=(mask<<1)|(p>>15)
 return mask

OPENDMR_TX_VERSION="1.0.0-q900fix5"

class OpenDmrCodec:
 def __init__(self,enc=False,dec=False):
  paths=[os.getenv("Q900_OPENDMR_LIB"),str(Path(__file__).with_name("libopendmr-q900fix.dylib")),str(Path(__file__).with_name("libopendmr-q900fix.so")),str(Path(__file__).with_name("libopendmr.dylib")),str(Path(__file__).with_name("libopendmr.so")),"/usr/local/lib/libopendmr.dylib","/usr/local/lib/libopendmr.so"];self.lib=None
  for p in filter(None,paths):
   try:self.lib=ctypes.CDLL(p);break
   except OSError:pass
  if self.lib is None:raise RuntimeError("OpenDMR library not found; set Q900_OPENDMR_LIB")
  L=self.lib;L.opendmr_version.restype=ctypes.c_char_p;self.version=(L.opendmr_version() or b"").decode("ascii","replace")
  L.opendmr_decoder_create.restype=ctypes.c_void_p;L.opendmr_encoder_create.restype=ctypes.c_void_p
  L.opendmr_decoder_destroy.argtypes=(ctypes.c_void_p,);L.opendmr_encoder_destroy.argtypes=(ctypes.c_void_p,)
  allow_stock=os.getenv("Q900_DMR_ALLOW_STOCK_OPENDMR","0").lower() in ("1","true","yes")
  if enc and self.version!=OPENDMR_TX_VERSION and not allow_stock:
   loaded=getattr(L,"_name","OpenDMR")
   raise RuntimeError(f"DMR TX needs {OPENDMR_TX_VERSION}; loaded {self.version or 'unknown'} from {loaded}. Run: bash tools/build_opendmr_fixed.sh")
  L.opendmr_decode.argtypes=(ctypes.c_void_p,ctypes.POINTER(ctypes.c_uint8),ctypes.POINTER(ctypes.c_int16),ctypes.POINTER(ctypes.c_int));L.opendmr_decode.restype=ctypes.c_bool
  L.opendmr_encode.argtypes=(ctypes.c_void_p,ctypes.POINTER(ctypes.c_int16),ctypes.POINTER(ctypes.c_uint8));L.opendmr_encode.restype=ctypes.c_bool
  if hasattr(L,"opendmr_encoder_set_gain"):
   L.opendmr_encoder_set_gain.argtypes=(ctypes.c_void_p,ctypes.c_int);L.opendmr_encoder_set_gain.restype=None
  if hasattr(L,"opendmr_encoder_reset"):L.opendmr_encoder_reset.argtypes=(ctypes.c_void_p,)
  if hasattr(L,"opendmr_decoder_reset"):L.opendmr_decoder_reset.argtypes=(ctypes.c_void_p,)
  self._encode_params_test=getattr(L,"opendmr_q900_encode_params",None)
  if self._encode_params_test is not None:
   self._encode_params_test.argtypes=(ctypes.POINTER(ctypes.c_int),ctypes.POINTER(ctypes.c_uint8));self._encode_params_test.restype=ctypes.c_bool
  self.decoder=L.opendmr_decoder_create() if dec else None;self.encoder=L.opendmr_encoder_create() if enc else None
 def set_gain_db(self,gain_db):
  if not self.encoder:raise RuntimeError("OpenDMR encoder is not open")
  fn=getattr(self.lib,"opendmr_encoder_set_gain",None)
  if fn is None:raise RuntimeError("OpenDMR encoder gain API unavailable")
  fn(self.encoder,int(round(gain_db)))
 def decode(self,frame):
  inp=(ctypes.c_uint8*9).from_buffer_copy(frame);out=(ctypes.c_int16*160)();err=ctypes.c_int()
  if not self.lib.opendmr_decode(self.decoder,inp,out,ctypes.byref(err)):raise RuntimeError("OpenDMR decode failed")
  return np.ctypeslib.as_array(out).copy(),err.value
 def encode(self,pcm):
  # q900fix4 repairs OpenDMR's public encoder so it returns exactly the documented
  # DVSI/canonical 72-bit frame: A(24) + B(23) + C(25). Keep that canonical frame
  # here; DmrVoiceTransmitter performs the one and only OTA interleave later.
  words=np.ascontiguousarray(pcm,dtype="<i2").reshape(-1)
  if len(words)!=160:raise ValueError("OpenDMR encode needs exactly 160 PCM samples")
  inp=(ctypes.c_int16*160).from_buffer_copy(words.tobytes());out=(ctypes.c_uint8*9)()
  if not self.lib.opendmr_encode(self.encoder,inp,out):raise RuntimeError("OpenDMR encode failed")
  return bytes(out)
 def _encode_params_frame(self,params):
  if self._encode_params_test is None:raise RuntimeError("q900fix parameter test hook unavailable")
  values=np.ascontiguousarray(params,dtype=np.int32).reshape(-1)
  if len(values)!=9:raise ValueError("AMBE parameter test needs 9 values")
  inp=(ctypes.c_int*9)(*[int(x) for x in values]);out=(ctypes.c_uint8*9)()
  if not self._encode_params_test(inp,out):raise RuntimeError("OpenDMR parameter test encode failed")
  return bytes(out)
 def close(self):
  if self.decoder:self.lib.opendmr_decoder_destroy(self.decoder);self.decoder=None
  if self.encoder:self.lib.opendmr_encoder_destroy(self.encoder);self.encoder=None

class DmrVocoderRecorder:
 def __init__(self,prefix):
  self.prefix=str(prefix);self.frames=0;self.decode_errors=0;self.decode_failures=0
  self.mic_sum_sq=0.;self.round_sum_sq=0.;self.mic_samples=0;self.round_samples=0
  self.mic_peak=0;self.round_peak=0;self.mic_clipped=0
  self.decoder=OpenDmrCodec(dec=True)
  self.mic=wave.open(f"{self.prefix}.dmr.mic.wav","wb");self.mic.setnchannels(1);self.mic.setsampwidth(2);self.mic.setframerate(8000)
  self.round=wave.open(f"{self.prefix}.dmr.roundtrip.wav","wb");self.round.setnchannels(1);self.round.setsampwidth(2);self.round.setframerate(8000)
  self.ambe=open(f"{self.prefix}.dmr.ambe.raw","wb")
 def capture(self,pcm,frame):
  p=np.ascontiguousarray(pcm,dtype="<i2").reshape(-1)
  if len(p)!=160:raise ValueError("DMR diagnostic expects 160 PCM samples")
  if len(frame)!=9:raise ValueError("DMR diagnostic expects a 9-byte AMBE frame")
  self.mic.writeframesraw(p.tobytes());self.ambe.write(frame);self.frames+=1
  pf=p.astype(np.float64);self.mic_sum_sq+=float(np.dot(pf,pf));self.mic_samples+=len(p)
  self.mic_peak=max(self.mic_peak,int(np.max(np.abs(p.astype(np.int32)))))
  self.mic_clipped+=int(np.count_nonzero(np.abs(p.astype(np.int32))>=32767))
  try:
   out,errs=self.decoder.decode(frame);self.decode_errors+=int(errs)
  except Exception:
   self.decode_failures+=1;out=np.zeros(160,dtype=np.int16)
  o=np.ascontiguousarray(out,dtype="<i2").reshape(-1)
  self.round.writeframesraw(o.tobytes());of=o.astype(np.float64)
  self.round_sum_sq+=float(np.dot(of,of));self.round_samples+=len(o)
  self.round_peak=max(self.round_peak,int(np.max(np.abs(o.astype(np.int32)))))
 def close(self):
  for h in (getattr(self,"mic",None),getattr(self,"round",None),getattr(self,"ambe",None)):
   try:
    if h:h.close()
   except Exception:pass
  try:self.decoder.close()
  except Exception:pass
  def dbfs(sum_sq,count):
   return 20*np.log10(max(np.sqrt(sum_sq/max(count,1))/32768.0,1e-12))
  meta={
   "version":1,"frames":self.frames,"duration_s":self.frames*0.020,
   "codec_version":getattr(self.decoder,"version",""),
   "decode_errors":self.decode_errors,"decode_failures":self.decode_failures,
   "mic_rms_dbfs":float(dbfs(self.mic_sum_sq,self.mic_samples)),
   "roundtrip_rms_dbfs":float(dbfs(self.round_sum_sq,self.round_samples)),
   "mic_peak":self.mic_peak,"roundtrip_peak":self.round_peak,
   "mic_clipped_samples":self.mic_clipped,
   "mic_wav":f"{self.prefix}.dmr.mic.wav",
   "roundtrip_wav":f"{self.prefix}.dmr.roundtrip.wav",
   "ambe_raw":f"{self.prefix}.dmr.ambe.raw",
  }
  try:
   with open(f"{self.prefix}.dmr.vocoder.json","w") as h:json.dump(meta,h,indent=2)
  except OSError:pass
  return meta

class DmrVoiceTransmitter:
 def __init__(self,config,offset_hz=12000,codec=None,q900_orientation=True,preamble_ms=DMR_TX_PREAMBLE_MS,diagnostic_prefix=None):
  config.validate_tx();self.c=config;self.lc=LinkControl(config.source_id,config.destination_id,config.group);self.codec=codec or OpenDmrCodec(enc=True);self.mod=Dmr4FskModulator(offset_hz,q900_orientation);self.pcm=np.empty(0,dtype=np.float32);self.ambe=deque();self.idx=0;self.started=False;self.preamble_ms=max(0,int(preamble_ms));self.diag=DmrVocoderRecorder(diagnostic_prefix) if diagnostic_prefix else None
  n=63;t=np.arange(n)-(n-1)/2;self._audio_taps=2*3400/SAMPLE_RATE*np.sinc(2*3400*t/SAMPLE_RATE)*np.hamming(n);self._audio_taps/=self._audio_taps.sum();self._audio_hist=np.zeros(n-1,dtype=np.float64)
 def _cycle(self,b):return self.mod.modulate(dmo_cycle_bits(b))
 def _preamble(self):
  # MMDVM DMO keys with repeated 0x5F (+3,+3,-3,-3) before the first burst.
  # 4 symbols/byte at 4800 sym/s => 1.2 bytes per millisecond.
  count=round(self.preamble_ms*SYMBOL_RATE/4000)
  return self.mod.modulate(bytes_bits(bytes([0x5F])*count)) if count else np.empty(0,dtype=np.complex64)
 def start_iq(self):
  self.started=True
  if hasattr(self.codec,"lib") and getattr(self.codec,"encoder",None) and hasattr(self.codec.lib,"opendmr_encoder_reset"):self.codec.lib.opendmr_encoder_reset(self.codec.encoder)
  pre=self._preamble()
  header_bits=build_data_burst(full_lc_payload(self.lc,DT_VOICE_LC_HEADER),self.c.color_code,DT_VOICE_LC_HEADER,self.c.data_sync())
  # Match the established MMDVM DMR call start: repeat the Voice LC Header
  # three times before voice. Modulate each cycle separately so CPM phase and
  # the RRC filter state remain continuous across all three 60 ms slots.
  headers=np.concatenate(tuple(self._cycle(header_bits) for _ in range(3)))
  return np.concatenate((pre,headers)) if len(pre) else headers
 def _pcm8(self,frame48):
  combined=np.r_[self._audio_hist,np.asarray(frame48,dtype=np.float64)]
  filtered=np.convolve(combined,self._audio_taps,mode="valid")
  self._audio_hist=combined[-(len(self._audio_taps)-1):]
  return np.clip(np.rint(filtered[::6]*32767),-32768,32767).astype(np.int16)
 def _queue_audio_frame(self,frame48):
  pcm8=self._pcm8(frame48);frame=self.codec.encode(pcm8)
  if self.diag:self.diag.capture(pcm8,frame)
  self.ambe.append(frame)
 def _emit_ready_voice(self,out):
  while len(self.ambe)>=3:
   frames=[self.ambe.popleft() for _ in range(3)]
   burst=build_voice_burst(frames,self.lc,self.c.color_code,self.idx,self.c.slot,self.c.voice_sync())
   out.append(self._cycle(burst));self.idx=(self.idx+1)%6
 def feed_pcm(self,x):
  self.pcm=np.r_[self.pcm,np.asarray(x,dtype=np.float32).reshape(-1)];out=[]
  if not self.started:out.append(self.start_iq())
  while len(self.pcm)>=960:
   f=self.pcm[:960];self.pcm=self.pcm[960:];self._queue_audio_frame(f)
   self._emit_ready_voice(out)
  return np.concatenate(out) if out else np.empty(0,dtype=np.complex64)
 def finish_iq(self):
  out=[]
  if not self.started:out.append(self.start_iq())
  if len(self.pcm):
   final=np.zeros(960,dtype=np.float32);final[:len(self.pcm)]=self.pcm
   self.pcm=np.empty(0,dtype=np.float32);self._queue_audio_frame(final)
  while self.ambe and len(self.ambe)<3:self._queue_audio_frame(np.zeros(960,dtype=np.float32))
  self._emit_ready_voice(out)
  out.append(self._cycle(build_data_burst(full_lc_payload(self.lc,DT_TERMINATOR_WITH_LC),self.c.color_code,DT_TERMINATOR_WITH_LC,self.c.data_sync())))
  return np.concatenate(out) if len(out)>1 else out[0]
 def close(self):
  if self.diag:self.diag.close();self.diag=None
  if hasattr(self.codec,"close"):self.codec.close()

@dataclass(slots=True)
class DmrStatus:
 sync:str="";slot:int|None=None;color_code:int|None=None;source:int|None=None;destination:int|None=None;group:bool|None=None;data_type:int|None=None;sync_quality:float=0.;corrected:int=0;ambe_frames:int=0;vocoder_errors:int=0;message:str="searching";input_dbfs:float=-120.;acquisition_quality:float=0.;sync_polarity:int=0;carrier_hz:float=0.
 def summary(self):
  p=["DMR"]
  if self.color_code is not None:p.append(f"CC{self.color_code}")
  if self.slot is not None:p.append(f"TS{self.slot}")
  if self.destination is not None:p.append(f"{'TG' if self.group else 'ID'} {self.destination}")
  if self.source is not None:p.append(f"SRC {self.source}")
  if len(p)==1:
   p.append(f"search {self.input_dbfs:.0f}dBFS q{self.acquisition_quality:.2f}")
  return "  ".join(p)

class DmrAirReceiver:
 def __init__(self,audio_output=None,status_output=None):
  self.audio_output=audio_output;self.status_output=status_output;self.status=DmrStatus();self.codec=None;self.codec_error=""
  try:self.codec=OpenDmrCodec(dec=True)
  except RuntimeError as e:self.codec_error=str(e)
  self.prev=1+0j;self.count=0;self.fs=np.zeros(len(RRC)-1);self.samples=np.empty(0);self.base=0;self.done=deque(maxlen=128);self.tracked=deque()
 def reset(self):self.prev=1+0j;self.count=0;self.fs.fill(0);self.samples=np.empty(0);self.base=0;self.done.clear();self.tracked.clear();self.status=DmrStatus()
 def close(self):
  if self.codec:self.codec.close()
 def _slot(self,n):return 1 if n.startswith("DIRECT1") else 2 if n.startswith("DIRECT2") else None
 def _used(self,s):return any(abs(s-x)<20 for x in self.done)
 def _symbols(self,start):
  i=start-self.base
  return None if i<0 or i+1320>len(self.samples) else self.samples[i:i+1320:10][:132]
 def feed(self,iq,offset_hz):
  z=np.asarray(iq,dtype=np.complex64).reshape(-1)
  if len(z):
   rms=float(np.sqrt(np.mean(np.abs(z.astype(np.complex128))**2)))
   self.status.input_dbfs=20*np.log10(max(rms,1e-6))
  # Do not pre-mix DMR by the analog SDR offset. 4FSK detection only needs
  # phase *differences*, so a constant RF offset becomes a constant discriminator
  # centre that the sync fit removes. Pre-mixing can instead push a channel on
  # the opposite side of the Q900 IQ passband through +/-Fs/2 and destroy it.
  self.count+=len(z);pr=np.r_[self.prev,z[:-1]];self.prev=z[-1] if len(z) else self.prev;d=np.angle(z*np.conj(pr))*48000/(2*np.pi);c=np.r_[self.fs,d];f=np.convolve(c,RRC,mode="valid");self.fs=c[-(len(RRC)-1):];self.samples=np.r_[self.samples,f];self._track();self._find()
  if len(self.samples)>48000:q=len(self.samples)-48000;self.samples=self.samples[q:];self.base+=q
 def _track(self):
  q=deque()
  while self.tracked:
   st,ce,sc,idx,sl=self.tracked.popleft();sy=self._symbols(st)
   if sy is None:
    if st>=self.base:q.append((st,ce,sc,idx,sl))
   elif not self._used(st):self._voice(levels_bits(sy,ce,sc),"TRACKED",sl,1.,idx);self.done.append(st)
  self.tracked=q
 def _find(self):
  if len(self.samples)<1560:return
  candidates=[];best_seen=0.0
  window=24;ones=np.ones(window,dtype=np.float64)
  for phase in range(10):
   sy=self.samples[phase::10]
   if len(sy)<window:continue
   # Rank sync candidates by *normalized* correlation. Raw dot product is
   # dominated by the very large discriminator transient when a TDMA handheld
   # keys on/off, which can prevent a nearly perfect real sync from ever being
   # examined. Normalizing by local energy makes the score about shape instead
   # of amplitude.
   sumy=np.convolve(sy,ones,mode="valid")
   sumy2=np.convolve(sy*sy,ones,mode="valid")
   local_energy=np.maximum(sumy2-(sumy*sumy/window),1e-12)
   for name,word in {**VOICE_SYNCS,**DATA_SYNCS}.items():
    x=dibit_levels(sync_bits(word));xc=x-x.mean()
    numerator=np.correlate(sy,xc,mode="valid")
    corr=numerator/np.sqrt(local_energy*np.dot(xc,xc))
    if not len(corr):continue
    for pos in np.argpartition(np.abs(corr),-min(4,len(corr)))[-4:]:
     corr_quality=float(abs(corr[pos]))
     best_seen=max(best_seen,corr_quality)
     if corr_quality<.70:continue
     y=sy[pos:pos+window];sc=float(np.dot(y-y.mean(),xc)/np.dot(xc,xc))
     if abs(sc)<80:continue
     ce=float(y.mean()-sc*x.mean());res=y-(ce+sc*x)
     qu=max(0.,1.-float(np.sqrt(np.mean(res*res)))/(abs(sc)*2))
     st=self.base+phase+(int(pos)-54)*10
     if qu>=.65 and st>=self.base and not self._used(st):
      # Sort primarily by normalized sync shape, then by the fitted-level
      # residual quality used by the slicer.
      candidates.append((corr_quality,qu,name,st,ce,sc))
  self.status.acquisition_quality=max(self.status.acquisition_quality*0.85,best_seen)
  if not candidates:return

  # Data and voice sync are complements, so an inverted discriminator makes a
  # true data sync correlate just as well with positive-polarity voice sync (and
  # vice versa). Resolve that ambiguity with the burst structure/FEC instead of
  # trusting correlation alone. Valid data candidates get first refusal.
  candidates.sort(reverse=True,key=lambda c:(c[0],c[1]))
  for corr_quality,qu,name,st,ce,sc in candidates:
   if not name.endswith("DATA"):continue
   sy=self._symbols(st)
   if sy is None:continue
   bits=levels_bits(sy,ce,sc)
   try:
    parsed=parse_data_burst(bits)
   except ValueError:
    continue
   dtype=parsed["data_type"]
   # LC-bearing bursts have an independent RS check; require it during initial
   # acquisition so complementary voice sync cannot win on a random FEC decode.
   if dtype in (DT_VOICE_LC_HEADER,DT_TERMINATOR_WITH_LC) and not parsed.get("lc_valid",False):
    continue
   self.status.sync_polarity=1 if sc>=0 else -1;self.status.carrier_hz=ce
   self._data(bits,name,self._slot(name),qu)
   self.done.append(st)
   return

  # No structurally valid data burst: accept the strongest voice candidate.
  corr_quality,qu,name,st,ce,sc=next((c for c in candidates if c[2].endswith("VOICE")),candidates[0])
  self.status.sync_polarity=1 if sc>=0 else -1;self.status.carrier_hz=ce;sy=self._symbols(st)
  if sy is None:return
  bits=levels_bits(sy,ce,sc);sl=self._slot(name)
  self._voice(bits,name,sl,qu,0)
  for i in range(1,6):self.tracked.append((st+i*2880,ce,sc,i,sl))
  self.done.append(st)
 def _emit(self):
  if self.status_output:self.status_output(self.status)
 def _data(self,bits,name,sl,q):
  try:r=parse_data_burst(bits)
  except ValueError:return
  s=self.status;s.sync=name;s.slot=sl;s.color_code=r["color_code"];s.data_type=r["data_type"];s.sync_quality=q;s.corrected=r["slot_errors"]+r["bptc_corrected"];lc=r.get("lc")
  if lc:s.source=lc.source;s.destination=lc.destination;s.group=lc.group;s.message="voice LC" if r["data_type"]==1 else "terminator";s.message+= "" if r.get("lc_valid") else " (RS bad)"
  self._emit()
 def _voice(self,bits,name,sl,q,idx):
  frames=ota_to_ambe(bits);s=self.status;s.sync=name;s.slot=sl;s.sync_quality=q;s.ambe_frames+=3;s.message=f"voice {chr(65+idx)}"
  if idx and s.color_code is None:
   i=voice_center_info(bits)
   if i.get("valid"):s.color_code=i["color_code"]
  if self.codec and self.audio_output:
   for fr in frames:
    pcm,e=self.codec.decode(fr);s.vocoder_errors+=e;src=pcm.astype(np.float32)/32768.;audio=np.interp(np.arange(960)/6.,np.arange(160),src,left=src[0],right=src[-1]).astype(np.float32);self.audio_output(audio)
  elif self.codec_error:s.message+=" (AMBE codec unavailable)"
  self._emit()

def self_test():
 assert dmr_prng_mask(0)==0x216623 and dmr_prng_mask(1)==0x0CEB7F and dmr_prng_mask(4095)==0x0B3F09
 # Independent vector for MW0MWZ/OpenDMR encoder/mbeenc.cpp encode_49bit().
 # The old public encoder incorrectly serialized these fields consecutively
 # (which would produce ab26d56d569a80 for this vector).
 params=(0x55,0x12,0x0D,0x155,0x5A,0x15,0xA,0x6,0x5)
 assert bits_bytes(ambe_params_bits(params)).hex()=="a96aabaaeda880"
 # External MMDVMHost silence vector. B9E881526173002A6B is the standard
 # on-air 72-bit AMBE silence codeword used in DMR_SILENCE_DATA. Pin both
 # directions so our canonical A+B+C <-> OTA interleave cannot validate itself.
 silence_ota=bytes.fromhex("B9 E8 81 52 61 73 00 2A 6B")
 silence_can=bytes.fromhex("F8 01 48 A8 2A 1B 3F 19 C1")
 assert _ota_to_can(bytes_bits(silence_ota))==silence_can
 assert bits_bytes(_can_to_ota(silence_can))[:9]==silence_ota
 for v in (0,1,0x5A,0xFF):
  c=golay_encode(v);assert golay_decode(c)==(v,0);n=c.copy();n[[0,7,19]]^=1;assert golay_decode(n)==(v,3)
 p=bytes(range(12));c=bptc_encode(p);assert bptc_decode(c)[0]==p
 lc=LinkControl(1234567,91);burst=build_data_burst(full_lc_payload(lc,1),1,1,SYNC_WORDS["DIRECT1_DATA"]);r=parse_data_burst(burst);assert r["lc_valid"] and r["lc"]==lc
 a=(bytes(range(9)),bytes(range(9,18)),bytes(range(18,27)))
 for i in range(6):assert ota_to_ambe(build_voice_burst(a,lc,1,i,1))==a
 # Normal/simplex TX uses MS-sourced sync, matching MMDVMHost duplex=false
 # and the OTA handheld capture used to bring up this decoder.
 cfg=DmrConfig(1234567,91,1,1,True,False)
 assert cfg.data_sync()==SYNC_WORDS["MS_DATA"] and cfg.voice_sync()==SYNC_WORDS["MS_VOICE"]
 direct=DmrConfig(1234567,91,1,2,True,True)
 assert direct.data_sync()==SYNC_WORDS["DIRECT2_DATA"] and direct.voice_sync()==SYNC_WORDS["DIRECT2_VOICE"]
 # Regression for real TDMA captures: a high-energy key-up transient must
 # not outrank a lower-amplitude but correctly shaped sync sequence.
 ideal=dibit_levels(sync_bits(SYNC_WORDS["MS_VOICE"]))
 noisy=np.zeros(140,dtype=np.float64);noisy[8:32]=12000*np.sign(np.sin(np.arange(24)*1.7))
 noisy[80:104]=12000+650*ideal
 xc=ideal-ideal.mean();raw=np.correlate(noisy,xc,mode="valid")
 assert int(np.argmax(np.abs(raw)))!=80
 sy=noisy;sumy=np.convolve(sy,np.ones(24),mode="valid");sumy2=np.convolve(sy*sy,np.ones(24),mode="valid")
 energy=np.maximum(sumy2-sumy*sumy/24,1e-12);corr=np.correlate(sy,xc,mode="valid")/np.sqrt(energy*np.dot(xc,xc))
 assert int(np.argmax(np.abs(corr)))==80 and abs(corr[80])>.999
 class FakeCodec:
  def __init__(self):self.n=0
  def encode(self,pcm):
   self.n+=1;return bytes(((self.n+i*17)&255) for i in range(9))
 fake=FakeCodec();tx=DmrVoiceTransmitter(cfg,12000,fake,q900_orientation=False,preamble_ms=0)
 start=tx.start_iq()
 assert len(start)==3*SLOT_SAMPLES,len(start)
 call=[start]
 # Six voice bursts = one complete superframe. PCM contents are irrelevant to
 # FakeCodec but exercise the streaming 48->8 kHz frame cadence.
 for k in range(18):call.append(tx.feed_pcm(np.sin(2*np.pi*700*np.arange(960)/48000).astype(np.float32)))
 call.append(tx.finish_iq());call=np.concatenate([x for x in call if len(x)])
 statuses=[];rxcall=DmrAirReceiver(status_output=lambda st:statuses.append(DmrStatus(**{f:getattr(st,f) for f in st.__dataclass_fields__})))
 for i in range(0,len(call),173):rxcall.feed(call[i:i+173],12000)
 assert any(st.sync=="MS_DATA" and st.source==cfg.source_id and st.destination==cfg.destination_id for st in statuses),[st.summary() for st in statuses]
 assert any(st.sync=="MS_VOICE" and st.ambe_frames>=3 for st in statuses),[(st.sync,st.ambe_frames) for st in statuses]
 assert any(st.message.startswith("terminator") for st in statuses),[(st.sync,st.message) for st in statuses]
 rxcall.close()
 # PTT release must not discard a partial 20 ms PCM frame or 1-2 AMBE frames.
 # Four full frames plus half a frame become six AMBE frames: the partial PCM
 # is zero-padded once, then one silence AMBE frame completes the 60 ms burst.
 tail_fake=FakeCodec();tail_tx=DmrVoiceTransmitter(cfg,12000,tail_fake,q900_orientation=False,preamble_ms=0)
 tail_tx.start_iq();tail_tx.feed_pcm(np.ones(960*4+480,dtype=np.float32)*.1)
 tail_end=tail_tx.finish_iq()
 assert tail_fake.n==6 and len(tail_tx.pcm)==0 and len(tail_tx.ambe)==0
 assert len(tail_end)==2*SLOT_SAMPLES,len(tail_end)

 got=[];rx=DmrAirReceiver(status_output=lambda s:got.append(DmrStatus(**{f:getattr(s,f) for f in s.__dataclass_fields__})));m=Dmr4FskModulator(12000,False);z=m.modulate(dmo_cycle_bits(burst))
 for i in range(0,len(z),173):rx.feed(z[i:i+173],12000)
 assert any(s.source==lc.source and s.destination==lc.destination for s in got),[s.summary() for s in got]
 assert any(abs(s.carrier_hz-12000)<500 for s in got if s.source==lc.source),[(s.carrier_hz,s.summary()) for s in got]
 rx.close()
 # Repeat with the DMR deviation polarity inverted while keeping the +12 kHz
 # carrier in place. Real receivers/mixers may present either sign.
 n=np.arange(len(z));carrier=np.exp(1j*2*np.pi*12000*n/SAMPLE_RATE);zinv=carrier*np.conj(z/carrier)
 got=[];rx=DmrAirReceiver(status_output=lambda s:got.append(DmrStatus(**{f:getattr(s,f) for f in s.__dataclass_fields__})))
 for i in range(0,len(zinv),173):rx.feed(zinv[i:i+173],12000)
 assert any(s.source==lc.source and s.destination==lc.destination and s.sync_polarity==-1 for s in got),[s.summary() for s in got]
 rx.close()
