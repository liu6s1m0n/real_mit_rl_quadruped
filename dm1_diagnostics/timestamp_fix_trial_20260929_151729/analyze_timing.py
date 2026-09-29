import re
from collections import defaultdict
p = re.compile(r'\((\d+\.\d+)\) (can[01]) ([0-9A-F]{3})#([0-9A-F]{16})')
frames=[]
for line in open('can.log', encoding='ascii'):
    m=p.match(line)
    if m: frames.append((float(m[1]),m[2],m[3],m[4]))
enable=min(t for t,b,i,d in frames if d=='FFFFFFFFFFFFFFFC')
disable=max(t for t,b,i,d in frames if d=='FFFFFFFFFFFFFFFD')
cmd=[t for t,b,i,d in frames if b=='can0' and i=='004' and not d.startswith('FFFFFFFF') and enable<t<disable]
gaps=sorted(((b-a,a,b) for a,b in zip(cmd,cmd[1:])), reverse=True)
print(f'enable={enable:.6f} disable={disable:.6f} enabled_duration={disable-enable:.3f}s command_count={len(cmd)}')
print('largest command gaps:')
for gap,a,b in gaps[:15]: print(f' gap={gap*1000:.3f}ms at +{a-enable:.3f}s')
master={'011':'001','012':'002','013':'003','014':'004','015':'005','016':'006'}
statuses=defaultdict(list)
for t,b,i,d in frames:
    if i in master and enable<t<disable:
        statuses[(b,master[i])].append((t,int(d[0],16)))
print('status transitions after enable:')
for key,items in sorted(statuses.items()):
    transitions=[]
    prev=items[0][1]
    for t,s in items[1:]:
        if s!=prev:
            transitions.append((t-enable,prev,s))
            prev=s
    print(key, transitions[:10], 'last=',items[-1][1])
