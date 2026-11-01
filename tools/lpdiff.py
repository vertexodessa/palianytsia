#!/usr/bin/env python3
"""First differences between two LP files (u32 len | bytes), e.g. ref vs rust
`--sends` or `--aus` dumps. Usage: tools/lpdiff.py a.sends b.sends"""
import sys,struct,collections
def rd(p):
    b=open(p,'rb').read(); i=0; out=[]
    while i<len(b):
        n=struct.unpack_from('<I',b,i)[0]; out.append(b[i+4:i+4+n]); i+=4+n
    return out
a,b=rd(sys.argv[1]),rd(sys.argv[2])
print(len(a),len(b))
bad=[k for k in range(min(len(a),len(b))) if a[k]!=b[k]]
print("differ",len(bad))
types=collections.Counter((x[3] if len(x)>3 else -1) for x in a)
print("ref types",types)
for k in bad[:4]:
    print(k,"ref ",a[k].hex()); print(k,"rust",b[k].hex())
