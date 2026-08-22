// ============================================================================
//  ALNS_EVRPTW_RemocionClientes.cpp
//  Operadores de remocion (destruccion) de clientes individuales.
// ============================================================================

#include "ALNS_EVRPTW.h"
#include <algorithm>
#include <cmath>
#include <map>

using namespace std;

void ALNS_EVRPTW::random_removal(vector<int>& fuera, int q) {
    vector<pair<int, int>> todas_pos;
    for (int r = 0; r < (int)solucion_actual.size(); ++r) {
        const auto& seq = solucion_actual[r].secuencia;
        for (int n = 1; n < (int)seq.size() - 1; ++n) {
            if (!mapa[seq[n].nodo_id].es_estacion) {
                todas_pos.push_back({ r, n });
            }
        }
    }
    if (todas_pos.empty()) return;

    shuffle(todas_pos.begin(), todas_pos.end(), rng);
    int num = min((int)todas_pos.size(), q);
    for (int i = 0; i < num; ++i) {
        fuera.push_back(solucion_actual[todas_pos[i].first].secuencia[todas_pos[i].second].nodo_id);
    }
    actualizar_rutas_post_remocion(fuera);
}

void ALNS_EVRPTW::worst_distance_removal(vector<int>& fuera, int q) {
    vector<pair<double, pair<int, int>>> costos;
    for (int r = 0; r < (int)solucion_actual.size(); ++r) {
        const auto& seq = solucion_actual[r].secuencia;
        for (int n = 1; n < (int)seq.size() - 1; ++n) {
            int id_act = seq[n].nodo_id;
            if (mapa[id_act].es_estacion) continue;
            int id_ant = seq[n - 1].nodo_id;
            int id_sig = seq[n + 1].nodo_id;
            double ahorro = matriz_dist[id_ant][id_act] + matriz_dist[id_act][id_sig] - matriz_dist[id_ant][id_sig];
            costos.push_back({ ahorro, {r, n} });
        }
    }
    sort(costos.rbegin(), costos.rend());
    int agregados = 0;
    int kappa = 4;
    while (agregados < q && !costos.empty()) {
        double lambda = dist_01(rng);
        int idx = pow(lambda, kappa) * costos.size();
        if (idx >= costos.size()) idx = costos.size() - 1;
        fuera.push_back(solucion_actual[costos[idx].second.first].secuencia[costos[idx].second.second].nodo_id);
        costos.erase(costos.begin() + idx);
        agregados++;
    }
    actualizar_rutas_post_remocion(fuera);
}

void ALNS_EVRPTW::worst_time_removal(vector<int>& fuera, int q) {
    vector<pair<double, int>> costos;
    for (int r = 0; r < (int)solucion_actual.size(); ++r) {
        const auto& seq = solucion_actual[r].secuencia;
        for (int n = 1; n < (int)seq.size() - 1; ++n) {
            int id_act = seq[n].nodo_id;
            if (mapa[id_act].es_estacion || mapa[id_act].es_deposito) continue;
            costos.push_back({ abs(seq[n].llegada_tiempo - mapa[id_act].ready_time), id_act });
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

void ALNS_EVRPTW::shaw_removal(vector<int>& fuera, int q) {
    vector<int> clientes_actuales;
    map<int, int> ruta_de_cliente;

    for (int r = 0; r < (int)solucion_actual.size(); ++r) {
        for (size_t i = 1; i < solucion_actual[r].secuencia.size() - 1; ++i) {
            int id_act = solucion_actual[r].secuencia[i].nodo_id;
            if (!mapa[id_act].es_estacion && !mapa[id_act].es_deposito) {
                clientes_actuales.push_back(id_act);
                ruta_de_cliente[id_act] = r;
            }
        }
    }
    if (clientes_actuales.empty()) return;

    double phi_1 = 5.0, phi_2 = 13.0, phi_3 = 0.15, phi_4 = 4.0, eta = 12.0;
    int idx_semilla = rng() % clientes_actuales.size();
    fuera.push_back(clientes_actuales[idx_semilla]);
    clientes_actuales.erase(clientes_actuales.begin() + idx_semilla);

    while ((int)fuera.size() < q && !clientes_actuales.empty()) {
        int i = fuera[rng() % fuera.size()];
        vector<pair<double, int>> similitudes;
        for (int j : clientes_actuales) {
            double d_ij = matriz_dist[i][j];
            double tiempo_diff = abs(mapa[i].ready_time - mapa[j].ready_time);
            double l_ij = (ruta_de_cliente[i] == ruta_de_cliente[j]) ? -1.0 : 1.0;
            double demanda_diff = abs(mapa[i].demanda - mapa[j].demanda);
            double R_ij = (phi_1 * d_ij) + (phi_2 * tiempo_diff) + (phi_3 * l_ij) + (phi_4 * demanda_diff);
            similitudes.push_back({ R_ij, j });
        }
        sort(similitudes.begin(), similitudes.end());
        double lambda = dist_01(rng);
        int idx = pow(lambda, eta) * similitudes.size();
        if (idx >= similitudes.size()) idx = similitudes.size() - 1;

        int cliente_elegido = similitudes[idx].second;
        fuera.push_back(cliente_elegido);
        clientes_actuales.erase(remove(clientes_actuales.begin(), clientes_actuales.end(), cliente_elegido), clientes_actuales.end());
    }
    actualizar_rutas_post_remocion(fuera);
}
