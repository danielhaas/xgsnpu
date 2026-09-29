#!/usr/bin/python3
# Read the NPU barmap from BAR2 (read-only). Checks config space first.
import mmap, os, struct, subprocess, sys
D='/sys/bus/pci/devices/0000:01:00.0'
if open(D+'/vendor').read().strip()!='0x11ab': sys.exit('no device')
cfg=open(D+'/config','rb').read(4)
if cfg==b'\xff\xff\xff\xff': sys.exit('endpoint not answering config space - NOT touching BARs')
bar=struct.unpack('<I',open(D+'/config','rb').read()[0x18:0x1c])[0]
if bar & ~0xf == 0: sys.exit('BAR2 cleared in config space (0x%x) - rescan first'%bar)
subprocess.run(['setpci','-s','01:00.0','COMMAND=0002:0002'])
f=os.open(D+'/resource2',os.O_RDONLY|os.O_SYNC); sz=os.fstat(f).st_size
m=mmap.mmap(f,sz,mmap.MAP_SHARED,mmap.PROT_READ)
off=sz-0x2000
w=lambda o: struct.unpack('<I',m[o:o+4])[0]
print('BAR2 size 0x%x barmap @0x%x version %d cookie 0x%08x'%(sz,off,w(off),w(off+4)))
names={0:'ctrl',1:'mvmgmt',2:'nwa',3:'rpc',4:'giu'}
for i in range(5):
    b,t,o,s=struct.unpack('<4I',m[off+8+16*i:off+24+16*i])
    print('  fac[%d] bar=%s type=%d(%s) off=0x%x size=0x%x'%(i,'BAR0' if b==0 else 'BAR2',t,names.get(t,'?'),o,s))
win=sz-0x104000
print('ctrl cookie @0x%x = 0x%08x'%(win,w(win)))
