import sys, csv, math
from ortools.sat.python import cp_model
inst, carpeta = sys.argv[1], sys.argv[2]
COVER = len(sys.argv)>3 and sys.argv[3]=='cover'
# instancia
mapa={}; dep=None; est=set()
with open(f'/home/claude/evrptw/instancias/{inst}.txt') as f:
    for line in f:
        p=line.split()
        if len(p)>=7 and p[1] in 'dfc':
            i=len(mapa); mapa[p[0]]=i
            if p[1]=='d': dep=i
            if p[1]=='f': est.add(i)
clientes=[i for i in range(len(mapa)) if i!=dep and i not in est]
rutas=[]
with open(f'{carpeta}/pool_{inst}.csv') as f:
    for r in csv.DictReader(f):
        seq=[int(x) for x in r['secuencia'].split()]
        cl=frozenset(c for c in seq if c in clientes)
        if cl: rutas.append((float(r['distancia']), cl, seq))
# quedarse con la ruta mas corta por conjunto de clientes
best={}
for d,cl,seq in rutas:
    if cl not in best or d<best[cl][0]: best[cl]=(d,seq)
rutas=[(d,cl,seq) for cl,(d,seq) in best.items()]
print(f'{inst}: pool {len(rutas)} rutas distintas')
def resolver(max_rutas=None, tlim=60):
    m=cp_model.CpModel(); x=[m.NewBoolVar(f'x{j}') for j in range(len(rutas))]
    for c in clientes:
        cov=[x[j] for j in range(len(rutas)) if c in rutas[j][1]]
        (m.AddAtLeastOne if COVER else m.AddExactlyOne)(cov)
    if max_rutas is None: m.Minimize(sum(x))
    else:
        m.Add(sum(x)<=max_rutas); m.Minimize(sum(int(round(rutas[j][0]*1000))*x[j] for j in range(len(rutas))))
    s=cp_model.CpSolver(); s.parameters.max_time_in_seconds=tlim; s.parameters.num_workers=1
    st=s.Solve(m)
    if st not in (cp_model.OPTIMAL, cp_model.FEASIBLE): return None
    sel=[j for j in range(len(rutas)) if s.Value(x[j])]
    return sel, s.StatusName(st)
r1=resolver(tlim=150); 
if r1 is None: print('fase 1 sin solucion'); sys.exit()
sel,stn=r1; nveh=len(sel); print(f'fase 1: {nveh} vehiculos ({stn})')
r2=resolver(nveh, 90); sel,stn=r2
print(f'fase 2: {len(sel)} vehiculos, distancia {sum(rutas[j][0] for j in sel):.1f} ({stn})')
