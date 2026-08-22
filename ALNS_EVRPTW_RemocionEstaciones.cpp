// ============================================================================
//  ALNS_EVRPTW_RemocionEstaciones.cpp
//  Operadores de remocion de estaciones de recarga.
// ============================================================================

#include "ALNS_EVRPTW.h"
#include <algorithm>
#include <cmath>

using namespace std;

void ALNS_EVRPTW::random_station_removal(vector<int>& fuera, int q) {
    vector<pair<int, int>> pos;
    for (int r = 0; r < (int)solucion_actual.size(); r++) {
        for (int i = 1; i < (int)solucion_actual[r].secuencia.size() - 1; i++) {
            if (mapa[solucion_actual[r].secuencia[i].nodo_id].es_estacion) pos.push_back({ r,i });
        }
    }
    shuffle(pos.begin(), pos.end(), rng);
    for (int i = 0; i < min(q, (int)pos.size()); i++) fuera.push_back(solucion_actual[pos[i].first].secuencia[pos[i].second].nodo_id);
    actualizar_rutas_post_remocion(fuera);
}

void ALNS_EVRPTW::worst_distance_station(vector<int>& fuera, int q) {
    vector<pair<double, int>> costos;
    for (int r = 0; r < (int)solucion_actual.size(); ++r) {
        const auto& seq = solucion_actual[r].secuencia;
        for (int n = 1; n < (int)seq.size() - 1; ++n) {
            int id_act = seq[n].nodo_id;
            if (mapa[id_act].es_estacion && !mapa[id_act].es_deposito) {
                int id_ant = seq[n - 1].nodo_id;
                int id_sig = seq[n + 1].nodo_id;
                double ahorro = matriz_dist[id_ant][id_act] + matriz_dist[id_act][id_sig] - matriz_dist[id_ant][id_sig];
                costos.push_back({ ahorro, id_act });
            }
        }
    }
    if (costos.empty()) return;
    sort(costos.rbegin(), costos.rend());
    int agregados = 0;
    int kappa = 4;
    while (agregados < q && !costos.empty()) {
        double lambda = dist_01(rng);
        int idx = pow(lambda, kappa) * costos.size();
        if (idx >= costos.size()) idx = costos.size() - 1;
        fuera.push_back(costos[idx].second);
        costos.erase(costos.begin() + idx);
        agregados++;
    }
    actualizar_rutas_post_remocion(fuera);
}

void ALNS_EVRPTW::worst_charge_station(vector<int>& fuera, int q) {
    vector<pair<double, int>> niveles_bateria;
    for (int r = 0; r < (int)solucion_actual.size(); ++r) {
        for (int n = 1; n < (int)solucion_actual[r].secuencia.size() - 1; ++n) {
            int id_act = solucion_actual[r].secuencia[n].nodo_id;
            if (mapa[id_act].es_estacion && !mapa[id_act].es_deposito) {
                niveles_bateria.push_back({ solucion_actual[r].secuencia[n].llegada_bateria, id_act });
            }
        }
    }
    if (niveles_bateria.empty()) return;
    sort(niveles_bateria.rbegin(), niveles_bateria.rend());
    int num = min((int)niveles_bateria.size(), q);
    for (int i = 0; i < num; ++i) fuera.push_back(niveles_bateria[i].second);
    actualizar_rutas_post_remocion(fuera);
}

void ALNS_EVRPTW::full_charge_station_removal(vector<int>& fuera, int q) {
    vector<int> estaciones_cargadas;
    for (int r = 0; r < (int)solucion_actual.size(); ++r) {
        for (int n = 1; n < (int)solucion_actual[r].secuencia.size() - 1; ++n) {
            int id_act = solucion_actual[r].secuencia[n].nodo_id;
            if (mapa[id_act].es_estacion && !mapa[id_act].es_deposito) {
                if (solucion_actual[r].secuencia[n].salida_bateria >= CAP_BATT - 1e-5) estaciones_cargadas.push_back(id_act);
            }
        }
    }
    if (estaciones_cargadas.empty()) return;
    shuffle(estaciones_cargadas.begin(), estaciones_cargadas.end(), rng);
    int num = min((int)estaciones_cargadas.size(), q);
    for (int i = 0; i < num; ++i) fuera.push_back(estaciones_cargadas[i]);
    actualizar_rutas_post_remocion(fuera);
}
