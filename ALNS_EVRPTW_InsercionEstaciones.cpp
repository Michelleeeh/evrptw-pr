// ============================================================================
//  ALNS_EVRPTW_InsercionEstaciones.cpp
//  Operadores de insercion de estaciones de recarga.
// ============================================================================

#include "ALNS_EVRPTW.h"
#include <limits>

using namespace std;

void ALNS_EVRPTW::greedy_station_insertion() {
    for (auto& ruta : solucion_actual) {
        auto& seq = ruta.secuencia;
        for (int i = 1; i < (int)seq.size(); ++i) {
            if (seq[i].llegada_bateria >= 0) continue;
            double best = 1e18;
            int best_s = -1;
            for (int s : estaciones) {
                seq.insert(seq.begin() + i, { s,0,0,0,0 });
                if (ruta.calcular_metricas(mapa, matriz_dist, CAP_BATT, CAP_CARGA, R_CARGA, R_CONSUMO, ID_DEPOSITO) == 0) {
                    if (ruta.distancia_total < best) { best = ruta.distancia_total; best_s = s; }
                }
                seq.erase(seq.begin() + i);
            }
            if (best_s != -1) {
                seq.insert(seq.begin() + i, { best_s,0,0,0,0 });
                ruta.calcular_metricas(mapa, matriz_dist, CAP_BATT, CAP_CARGA, R_CARGA, R_CONSUMO, ID_DEPOSITO);
            }
        }
    }
}

void ALNS_EVRPTW::greedy_station_insertion_comparison() {
    for (auto& ruta : solucion_actual) {
        auto& seq = ruta.secuencia;
        while (true) {
            ruta.calcular_metricas(mapa, matriz_dist, CAP_BATT, CAP_CARGA, R_CARGA, R_CONSUMO, ID_DEPOSITO);
            if (ruta.violacion_bateria <= 1e-5) break;
            int idx_falla = -1;
            for (int i = 1; i < (int)seq.size(); ++i) {
                if (seq[i].llegada_bateria < 0) { idx_falla = i; break; }
            }
            if (idx_falla == -1) break;
            double mejor_dist = numeric_limits<double>::max();
            int mejor_estacion = -1, mejor_pos = -1;
            for (int pos = idx_falla; pos >= 1; --pos) {
                for (int s : estaciones) {
                    seq.insert(seq.begin() + pos, { s,0,0,0,0 });
                    if (ruta.calcular_metricas(mapa, matriz_dist, CAP_BATT, CAP_CARGA, R_CARGA, R_CONSUMO, ID_DEPOSITO) == 0 && ruta.distancia_total < mejor_dist) {
                        mejor_dist = ruta.distancia_total; mejor_estacion = s; mejor_pos = pos;
                    }
                    seq.erase(seq.begin() + pos);
                }
            }
            if (mejor_estacion != -1) seq.insert(seq.begin() + mejor_pos, { mejor_estacion,0,0,0,0 });
            else break;
        }
    }
}

void ALNS_EVRPTW::best_station_insertion() {
    for (auto& ruta : solucion_actual) {
        auto& seq = ruta.secuencia;
        while (true) {
            ruta.calcular_metricas(mapa, matriz_dist, CAP_BATT, CAP_CARGA, R_CARGA, R_CONSUMO, ID_DEPOSITO);
            if (ruta.violacion_bateria <= 1e-5) break;
            int idx_falla = -1, idx_ultimo_punto_carga = 0;
            for (int i = 1; i < (int)seq.size(); ++i) {
                int id_previo = seq[i - 1].nodo_id;
                if (mapa[id_previo].es_estacion || mapa[id_previo].es_deposito) idx_ultimo_punto_carga = i - 1;
                if (seq[i].llegada_bateria < 0) { idx_falla = i; break; }
            }
            if (idx_falla == -1) break;
            double best_dist = numeric_limits<double>::max();
            int best_s = -1, best_pos = -1;
            for (int pos = idx_ultimo_punto_carga + 1; pos <= idx_falla; ++pos) {
                for (int s : estaciones) {
                    seq.insert(seq.begin() + pos, { s,0,0,0,0 });
                    if (ruta.calcular_metricas(mapa, matriz_dist, CAP_BATT, CAP_CARGA, R_CARGA, R_CONSUMO, ID_DEPOSITO) == 0) {
                        if (ruta.distancia_total < best_dist) {
                            best_dist = ruta.distancia_total; best_s = s; best_pos = pos;
                        }
                    }
                    seq.erase(seq.begin() + pos);
                }
            }
            if (best_s != -1) seq.insert(seq.begin() + best_pos, { best_s,0,0,0,0 });
            else break;
        }
    }
}
