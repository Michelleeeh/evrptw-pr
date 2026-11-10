// ============================================================================
//  test_equivalencia.cpp
//  Verifica que los cambios 4 y 2 (version con sumas en orden original) son EXACTOS:
//    (a) calcular_metricas nuevo == calcular_metricas original, parada por
//        parada (status, distancia, violaciones, baterias y tiempos).
//    (b) calcular_metricas_con_insercion_virtual == insertar de verdad y
//        evaluar, para cliente solo y para cliente + estacion.
//  Uso: ./test_equivalencia instancias/rc104_21.txt [n_rutas] [semilla]
// ============================================================================
#include "../Ruta.h"
#include "RutaOrig.h"
#include "../InstanciaReader.h"
#include <random>
#include <iostream>
#include <cmath>
#include <cstdlib>
using namespace std;

static int fallos = 0;
static void check(bool ok, const string& msg) { if (!ok) { ++fallos; if (fallos <= 20) cerr << "  FALLO: " << msg << endl; } }

int main(int argc, char** argv) {
    string archivo = argc > 1 ? argv[1] : "instancias/rc104_21.txt";
    int n_rutas = argc > 2 ? atoi(argv[2]) : 20000;
    unsigned semilla = argc > 3 ? atoi(argv[3]) : 7;
    vector<Nodo> mapa; double cc, cb, rc, rcons;
    if (!leerInstancia(archivo, mapa, cc, cb, rc, rcons)) { cerr << "no se pudo leer " << archivo << endl; return 1; }
    int n = mapa.size(), dep = -1;
    vector<int> clientes, estaciones;
    for (int i = 0; i < n; ++i) { if (mapa[i].es_deposito) dep = i; else if (mapa[i].es_estacion) estaciones.push_back(i); else clientes.push_back(i); }
    vector<vector<double>> D(n, vector<double>(n));
    for (int i = 0; i < n; ++i) for (int j = 0; j < n; ++j) D[i][j] = hypot(mapa[i].x - mapa[j].x, mapa[i].y - mapa[j].y);

    mt19937 rng(semilla);
    const double TOL = 1e-9;
    long comp_metricas = 0, comp_virtual = 0, comp_virtual_est = 0;

    for (int t = 0; t < n_rutas; ++t) {
        // Ruta aleatoria: 1..14 clientes y 0..3 estaciones intercaladas; mitad de
        // las veces se usan solo clientes cercanos entre si para que haya rutas
        // factibles (y no solo fallos tempranos).
        int nc = 1 + rng() % 14, ne = rng() % 4;
        vector<int> cuerpo;
        if (rng() % 2) { for (int i = 0; i < nc; ++i) cuerpo.push_back(clientes[rng() % clientes.size()]); }
        else { int base = rng() % clientes.size(); for (int i = 0; i < nc; ++i) cuerpo.push_back(clientes[(base + rng() % 12) % clientes.size()]); }
        for (int i = 0; i < ne; ++i) cuerpo.insert(cuerpo.begin() + rng() % (cuerpo.size() + 1), estaciones[rng() % estaciones.size()]);

        Ruta nueva; RutaOrig vieja;
        nueva.secuencia.push_back({ dep,0,0,0,0 }); vieja.secuencia.push_back({ dep,0,0,0,0 });
        for (int id : cuerpo) { nueva.secuencia.push_back({ id,0,0,0,0 }); vieja.secuencia.push_back({ id,0,0,0,0 }); }
        nueva.secuencia.push_back({ dep,0,0,0,0 }); vieja.secuencia.push_back({ dep,0,0,0,0 });

        // (a) calcular_metricas
        int st_n = nueva.calcular_metricas(mapa, D, cb, cc, rc, rcons, dep);
        int st_o = vieja.calcular_metricas(mapa, D, cb, cc, rc, rcons, dep);
        ++comp_metricas;
        check(st_n == st_o, "status distinto en calcular_metricas");
        check(fabs(nueva.distancia_total - vieja.distancia_total) <= TOL, "distancia distinta");
        check(fabs(nueva.violacion_tiempo - vieja.violacion_tiempo) <= TOL, "viol_t distinta");
        check(fabs(nueva.violacion_bateria - vieja.violacion_bateria) <= TOL, "viol_b distinta");
        check(fabs(nueva.violacion_capacidad - vieja.violacion_capacidad) <= TOL, "viol_c distinta");
        check(fabs(nueva.carga_total - vieja.carga_total) <= TOL, "carga distinta");
        // Hasta la parada de fallo (o toda la ruta) las paradas deben coincidir.
        for (size_t i = 0; i < nueva.secuencia.size(); ++i) {
            const auto& a = nueva.secuencia[i]; const auto& b = vieja.secuencia[i];
            bool ok = fabs(a.llegada_bateria - b.llegada_bateria) <= TOL && fabs(a.salida_bateria - b.salida_bateria) <= TOL
                && fabs(a.llegada_tiempo - b.llegada_tiempo) <= TOL && fabs(a.salida_tiempo - b.salida_tiempo) <= TOL;
            check(ok, "parada " + to_string(i) + " distinta");
            // Si hubo fallo temprano, las paradas posteriores no se escriben: cortar.
            if (st_n != 0 && (a.llegada_bateria < -1e-9 || a.llegada_tiempo > mapa[a.nodo_id].due_date + 1e-9 || a.salida_tiempo > mapa[a.nodo_id].due_date + 1e-9)) break;
        }

        // (c) insercion virtual vs. insercion real. Base: la ruta sin modificar.
        Ruta base = nueva; base.calcular_metricas(mapa, D, cb, cc, rc, rcons, dep);
        for (int rep = 0; rep < 3; ++rep) {
            int u = clientes[rng() % clientes.size()];
            int pos = 1 + rng() % (base.secuencia.size() - 1);
            double dv; int fv;
            int sv = base.calcular_metricas_con_insercion_virtual(mapa, D, cb, cc, rc, rcons, dep, pos, u, -1, -1, dv, fv);
            Ruta real = base; real.secuencia.insert(real.secuencia.begin() + pos, { u,0,0,0,0 });
            int sr = real.calcular_metricas(mapa, D, cb, cc, rc, rcons, dep);
            int fr = -1; if (sr == 1) for (int i = 1; i < (int)real.secuencia.size(); ++i) if (real.secuencia[i].llegada_bateria < -1e-9) { fr = i; break; }
            ++comp_virtual;
            check(sv == sr, "virtual(u): status distinto"); check(fabs(dv - real.distancia_total) <= TOL, "virtual(u): distancia distinta"); check(fv == fr, "virtual(u): idx_falla distinto");
            // Cliente + estacion en una posicion anterior (como en probar_insercion).
            if (real.secuencia.size() > 2) {
                int p = 1 + rng() % (real.secuencia.size() - 1);
                int s = estaciones[rng() % estaciones.size()];
                double dv2; int fv2;
                int sv2 = base.calcular_metricas_con_insercion_virtual(mapa, D, cb, cc, rc, rcons, dep, pos, u, p, s, dv2, fv2);
                Ruta real2 = real; real2.secuencia.insert(real2.secuencia.begin() + p, { s,0,0,0,0 });
                int sr2 = real2.calcular_metricas(mapa, D, cb, cc, rc, rcons, dep);
                int fr2 = -1; if (sr2 == 1) for (int i = 1; i < (int)real2.secuencia.size(); ++i) if (real2.secuencia[i].llegada_bateria < -1e-9) { fr2 = i; break; }
                ++comp_virtual_est;
                check(sv2 == sr2, "virtual(u,s): status distinto"); check(fabs(dv2 - real2.distancia_total) <= TOL, "virtual(u,s): distancia distinta"); check(fv2 == fr2, "virtual(u,s): idx_falla distinto");
            }
        }
    }
    cout << "Instancia " << archivo << ": " << comp_metricas << " rutas, "
         << comp_virtual << " inserciones virtuales de cliente, " << comp_virtual_est << " de cliente+estacion. Fallos: " << fallos << endl;
    return fallos == 0 ? 0 : 1;
}
