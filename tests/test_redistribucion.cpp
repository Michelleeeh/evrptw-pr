// Verifica la politica de redistribucion de carga: (a) toda ruta que pasa a
// factible cumple ventanas y bateria al releer las paradas; (b) ninguna ruta
// factible sin redistribucion deja de serlo; (c) real y virtual coinciden.
#include "../Ruta.h"
#include "../InstanciaReader.h"
#include <random>
#include <iostream>
#include <cmath>
using namespace std;
int main(int argc, char** argv) {
    string archivo = argc > 1 ? argv[1] : "instancias/rc104_21.txt";
    vector<Nodo> mapa; double cc, cb, rc, rcons; leerInstancia(archivo, mapa, cc, cb, rc, rcons);
    int n = mapa.size(), dep = -1; vector<int> cli, est;
    for (int i = 0; i < n; ++i) { if (mapa[i].es_deposito) dep = i; else if (mapa[i].es_estacion) est.push_back(i); else cli.push_back(i); }
    vector<vector<double>> D(n, vector<double>(n));
    for (int i = 0; i < n; ++i) for (int j = 0; j < n; ++j) D[i][j] = hypot(mapa[i].x - mapa[j].x, mapa[i].y - mapa[j].y);
    mt19937 rng(5); int fallos = 0, rescatadas = 0, total = 0, virt_ok = 0;
    for (int t = 0; t < 200000; ++t) {
        int nc = 3 + rng() % 10, ne = 2 + rng() % 2; vector<int> cuerpo;
        int base = rng() % cli.size(); for (int i = 0; i < nc; ++i) cuerpo.push_back(cli[(base + rng() % 15) % cli.size()]);
        for (int i = 0; i < ne; ++i) cuerpo.insert(cuerpo.begin() + 1 + rng() % (cuerpo.size() - 1), est[rng() % est.size()]);
        Ruta r; r.secuencia.push_back({ dep,0,0,0,0 }); for (int id : cuerpo) r.secuencia.push_back({ id,0,0,0,0 }); r.secuencia.push_back({ dep,0,0,0,0 });
        Ruta::redistribuir_carga = false; Ruta a = r; int sa = a.calcular_metricas(mapa, D, cb, cc, rc, rcons, dep);
        Ruta::redistribuir_carga = true;  Ruta b = r; int sb = b.calcular_metricas(mapa, D, cb, cc, rc, rcons, dep);
        total++;
        if (sa == 0 && sb != 0) { fallos++; if (fallos < 5) cerr << "ruta factible dejo de serlo\n"; }
        if (sb == 0) {
            if (sa != 0) rescatadas++;
            // releer paradas
            double t_ = 0, bat = cb;
            for (size_t i = 1; i < b.secuencia.size(); ++i) {
                const Parada& p = b.secuencia[i]; const Nodo& nd = mapa[p.nodo_id];
                double d = D[b.secuencia[i - 1].nodo_id][p.nodo_id];
                double lleg = t_ + d; bat -= d * rcons;
                bool ok = lleg <= nd.due_date + 1e-6 && bat >= -1e-6 && fabs(bat - p.llegada_bateria) < 1e-6 && p.salida_bateria <= cb + 1e-6 && p.salida_bateria >= p.llegada_bateria - 1e-6;
                if (nd.es_estacion && !nd.es_deposito) { double ini = max(lleg, nd.ready_time); double tc = (p.salida_bateria - p.llegada_bateria) * rc; ok = ok && p.salida_tiempo <= nd.due_date + 1e-6 && fabs(p.salida_tiempo - (ini + tc)) < 1e-6; t_ = p.salida_tiempo; bat = p.salida_bateria; }
                else { t_ = max(lleg, nd.ready_time) + nd.service_time; if (i + 1 == b.secuencia.size() && nd.es_deposito) bat = max(bat, 0.0); }
                if (i + 1 == b.secuencia.size()) ok = ok && (lleg <= nd.due_date + 1e-6);
                if (!ok && !(i + 1 == b.secuencia.size())) { fallos++; if (fallos < 5) cerr << "paradas inconsistentes en ruta rescatada, parada " << i << "\n"; break; }
            }
            if (fabs(a.distancia_total - b.distancia_total) > 1e-9 && sa == 0) { fallos++; }
        }
        // virtual vs real con redistribucion: insertar un cliente y comparar
        int u = cli[rng() % cli.size()], pos = 1 + rng() % (r.secuencia.size() - 1);
        double dv; int fv; int sv = b.calcular_metricas_con_insercion_virtual(mapa, D, cb, cc, rc, rcons, dep, pos, u, -1, -1, dv, fv);
        Ruta real = b; real.secuencia.insert(real.secuencia.begin() + pos, { u,0,0,0,0 }); int sr = real.calcular_metricas(mapa, D, cb, cc, rc, rcons, dep);
        if (sv == sr && fabs(dv - real.distancia_total) <= 1e-9) virt_ok++; else { fallos++; if (fallos < 5) cerr << "virtual != real\n"; }
    }
    cout << archivo << ": " << total << " rutas, rescatadas por redistribucion: " << rescatadas << ", virtual==real: " << virt_ok << "/" << total << ", fallos: " << fallos << endl;
    return fallos ? 1 : 0;
}
