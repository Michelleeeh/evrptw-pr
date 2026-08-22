// ============================================================================
//  ALNS_EVRPTW_RemocionRutas.cpp
//  Operadores de remocion de rutas completas y remocion por zona geografica.
// ============================================================================

#include "ALNS_EVRPTW.h"
#include <algorithm>
#include <cmath>

using namespace std;

void ALNS_EVRPTW::random_route_removal(vector<int>& fuera) {
    if (solucion_actual.size() <= 1) return;
    double proporcion = 0.1 + dist_01(rng) * 0.3;
    int rutas_a_remover = max(1, (int)(solucion_actual.size() * proporcion));
    for (int i = 0; i < rutas_a_remover; i++) {
        if (solucion_actual.size() <= 1) break;
        int r_elegida = rng() % solucion_actual.size();
        const auto& seq = solucion_actual[r_elegida].secuencia;
        for (int n = 1; n < (int)seq.size() - 1; ++n) {
            int n_id = seq[n].nodo_id;
            if (!mapa[n_id].es_estacion) fuera.push_back(n_id);
        }
        solucion_actual.erase(solucion_actual.begin() + r_elegida);
    }
}

void ALNS_EVRPTW::greedy_route_removal(vector<int>& fuera) {
    if (solucion_actual.size() <= 1) return;
    double proporcion = 0.1 + dist_01(rng) * 0.3;
    int rutas_a_remover = max(1, (int)(solucion_actual.size() * proporcion));

    vector<pair<int, int>> conteo_rutas;
    for (int r = 0; r < (int)solucion_actual.size(); ++r) {
        int count = 0;
        for (const auto& p : solucion_actual[r].secuencia) {
            if (!mapa[p.nodo_id].es_estacion && !mapa[p.nodo_id].es_deposito) count++;
        }
        conteo_rutas.push_back({ count, r });
    }
    sort(conteo_rutas.begin(), conteo_rutas.end());

    vector<int> indices_a_borrar;
    for (int i = 0; i < rutas_a_remover; i++) {
        int r_elegida = conteo_rutas[i].second;
        indices_a_borrar.push_back(r_elegida);
        for (int n = 1; n < (int)solucion_actual[r_elegida].secuencia.size() - 1; ++n) {
            int n_id = solucion_actual[r_elegida].secuencia[n].nodo_id;
            if (!mapa[n_id].es_estacion && !mapa[n_id].es_deposito) fuera.push_back(n_id);
        }
    }
    sort(indices_a_borrar.rbegin(), indices_a_borrar.rend());
    for (int r_idx : indices_a_borrar) solucion_actual.erase(solucion_actual.begin() + r_idx);
}

void ALNS_EVRPTW::customer_preceding_station(vector<int>& fuera, int q) {
    vector<pair<int, int>> pos_candidatas, pos_resto;
    for (int r = 0; r < (int)solucion_actual.size(); ++r) {
        for (int n = 1; n < (int)solucion_actual[r].secuencia.size() - 1; ++n) {
            int id_act = solucion_actual[r].secuencia[n].nodo_id;
            if (!mapa[id_act].es_estacion && !mapa[id_act].es_deposito) {
                int id_prev = solucion_actual[r].secuencia[n - 1].nodo_id;
                if (mapa[id_prev].es_estacion && !mapa[id_prev].es_deposito) pos_candidatas.push_back({ r, n });
                else pos_resto.push_back({ r, n });
            }
        }
    }
    shuffle(pos_candidatas.begin(), pos_candidatas.end(), rng);
    shuffle(pos_resto.begin(), pos_resto.end(), rng);
    vector<pair<int, int>> seleccionados;
    int agregados = 0;

    for (int i = 0; i < (int)pos_candidatas.size() && agregados < q; ++i) {
        seleccionados.push_back(pos_candidatas[i]);
        fuera.push_back(solucion_actual[pos_candidatas[i].first].secuencia[pos_candidatas[i].second].nodo_id);
        agregados++;
    }
    for (int i = 0; i < (int)pos_resto.size() && agregados < q; ++i) {
        fuera.push_back(solucion_actual[pos_resto[i].first].secuencia[pos_resto[i].second].nodo_id);
        agregados++;
    }
    sort(seleccionados.begin(), seleccionados.end(), [](const pair<int, int>& a, const pair<int, int>& b) {
        if (a.first != b.first) return a.first < b.first;
        return a.second > b.second;
        });

    for (auto& p : seleccionados) {
        int r = p.first;
        int n_estacion = p.second - 1;
        solucion_actual[r].secuencia.erase(solucion_actual[r].secuencia.begin() + n_estacion);
    }
    actualizar_rutas_post_remocion(fuera);
}

void ALNS_EVRPTW::customer_succeding_station(vector<int>& fuera, int q) {
    vector<pair<int, int>> pos_candidatas, pos_resto;
    for (int r = 0; r < (int)solucion_actual.size(); ++r) {
        for (int n = 1; n < (int)solucion_actual[r].secuencia.size() - 1; ++n) {
            int id_act = solucion_actual[r].secuencia[n].nodo_id;
            if (!mapa[id_act].es_estacion && !mapa[id_act].es_deposito) {
                int id_sig = solucion_actual[r].secuencia[n + 1].nodo_id;
                if (mapa[id_sig].es_estacion && !mapa[id_sig].es_deposito) pos_candidatas.push_back({ r, n });
                else pos_resto.push_back({ r, n });
            }
        }
    }
    shuffle(pos_candidatas.begin(), pos_candidatas.end(), rng);
    shuffle(pos_resto.begin(), pos_resto.end(), rng);
    vector<pair<int, int>> seleccionados;
    int agregados = 0;

    for (int i = 0; i < (int)pos_candidatas.size() && agregados < q; ++i) {
        seleccionados.push_back(pos_candidatas[i]);
        fuera.push_back(solucion_actual[pos_candidatas[i].first].secuencia[pos_candidatas[i].second].nodo_id);
        agregados++;
    }
    for (int i = 0; i < (int)pos_resto.size() && agregados < q; ++i) {
        fuera.push_back(solucion_actual[pos_resto[i].first].secuencia[pos_resto[i].second].nodo_id);
        agregados++;
    }
    sort(seleccionados.begin(), seleccionados.end(), [](const pair<int, int>& a, const pair<int, int>& b) {
        if (a.first != b.first) return a.first < b.first;
        return a.second > b.second;
        });

    for (auto& p : seleccionados) {
        int r = p.first;
        int n_estacion = p.second + 1;
        solucion_actual[r].secuencia.erase(solucion_actual[r].secuencia.begin() + n_estacion);
    }
    actualizar_rutas_post_remocion(fuera);
}

void ALNS_EVRPTW::zone_removal(vector<int>& fuera, int q) {
    if (solucion_actual.empty()) return;
    vector<pair<double, double>> centroides(solucion_actual.size(), { 0.0, 0.0 });
    vector<int> num_clientes_ruta(solucion_actual.size(), 0);

    for (int r = 0; r < (int)solucion_actual.size(); ++r) {
        for (const auto& p : solucion_actual[r].secuencia) {
            if (!mapa[p.nodo_id].es_estacion && !mapa[p.nodo_id].es_deposito) {
                centroides[r].first += mapa[p.nodo_id].x;
                centroides[r].second += mapa[p.nodo_id].y;
                num_clientes_ruta[r]++;
            }
        }
        if (num_clientes_ruta[r] > 0) {
            centroides[r].first /= num_clientes_ruta[r];
            centroides[r].second /= num_clientes_ruta[r];
        }
        else centroides[r] = { mapa[ID_DEPOSITO].x, mapa[ID_DEPOSITO].y };
    }

    int ruta_semilla = rng() % solucion_actual.size();
    vector<pair<double, int>> dist_rutas;
    for (int r = 0; r < (int)solucion_actual.size(); ++r) {
        double dx = centroides[r].first - centroides[ruta_semilla].first;
        double dy = centroides[r].second - centroides[ruta_semilla].second;
        dist_rutas.push_back({ sqrt(dx * dx + dy * dy), r });
    }
    sort(dist_rutas.begin(), dist_rutas.end());

    int n_z = min(15, (int)solucion_actual.size());
    vector<int> rutas_zona;
    for (int i = 0; i < n_z; ++i) rutas_zona.push_back(dist_rutas[i].second);

    vector<pair<int, int>> clientes_candidatos;
    for (int r : rutas_zona) {
        for (int n = 1; n < (int)solucion_actual[r].secuencia.size() - 1; ++n) {
            int id_nodo = solucion_actual[r].secuencia[n].nodo_id;
            if (!mapa[id_nodo].es_estacion && !mapa[id_nodo].es_deposito) {
                clientes_candidatos.push_back({ r, n });
            }
        }
    }
    shuffle(clientes_candidatos.begin(), clientes_candidatos.end(), rng);
    int num = min(q, (int)clientes_candidatos.size());
    for (int i = 0; i < num; ++i) {
        fuera.push_back(solucion_actual[clientes_candidatos[i].first].secuencia[clientes_candidatos[i].second].nodo_id);
    }
    actualizar_rutas_post_remocion(fuera);
}
