// ============================================================================
//  ALNS_EVRPTW_Auxiliares.cpp
//  Funciones auxiliares internas (privadas) usadas por los operadores y por
//  el ciclo principal del ALNS: construccion inicial, prueba/aplicacion de
//  inserciones, mantenimiento de indices, criterio de aceptacion, limpieza
//  de rutas vacias, seleccion de operadores y actualizacion de pesos.
// ============================================================================

#include "ALNS_EVRPTW.h"
#include <limits>
#include <algorithm>

using namespace std;

void ALNS_EVRPTW::inicializar_solucion() {
    vector<int> no_asignados;
    no_asignados.reserve(mapa.size());
    for (int i = 0; i < (int)mapa.size(); ++i) {
        if (!mapa[i].es_estacion && !mapa[i].es_deposito) no_asignados.push_back(i);
    }
    solucion_actual.clear();
    solucion_actual.reserve(50);
    greedy_insertion(no_asignados, false);
}

void ALNS_EVRPTW::crear_nueva_ruta(int nodo_id) {
    Ruta nueva;
    nueva.secuencia.reserve(100);
    nueva.secuencia = { {ID_DEPOSITO, 0, CAP_BATT, 0, 0}, {nodo_id, 0, 0, 0, 0}, {ID_DEPOSITO, 0, 0, 0, 0} };
    if (nueva.calcular_metricas(mapa, matriz_dist, CAP_BATT, CAP_CARGA, R_CARGA, R_CONSUMO, ID_DEPOSITO) == 1) {
        double mejor_costo_est = numeric_limits<double>::max();
        int mejor_est = -1;
        for (int s : estaciones) {
            nueva.secuencia.insert(nueva.secuencia.begin() + 1, { s, 0, 0, 0, 0 });
            if (nueva.calcular_metricas(mapa, matriz_dist, CAP_BATT, CAP_CARGA, R_CARGA, R_CONSUMO, ID_DEPOSITO) == 0) {
                if (nueva.distancia_total < mejor_costo_est) { mejor_costo_est = nueva.distancia_total; mejor_est = s; }
            }
            nueva.secuencia.erase(nueva.secuencia.begin() + 1);
        }
        if (mejor_est != -1) {
            nueva.secuencia.insert(nueva.secuencia.begin() + 1, { mejor_est, 0, 0, 0, 0 });
            nueva.calcular_metricas(mapa, matriz_dist, CAP_BATT, CAP_CARGA, R_CARGA, R_CONSUMO, ID_DEPOSITO);
        }
    }
    solucion_actual.push_back(nueva);
}

double ALNS_EVRPTW::probar_insercion(int r_idx, int pos, int u) {
    buffer_insercion.secuencia = solucion_actual[r_idx].secuencia;
    double dist_original = solucion_actual[r_idx].distancia_total;

    buffer_insercion.secuencia.insert(buffer_insercion.secuencia.begin() + pos, { u, 0, 0, 0, 0 });
    int status = buffer_insercion.calcular_metricas(mapa, matriz_dist, CAP_BATT, CAP_CARGA, R_CARGA, R_CONSUMO, ID_DEPOSITO);

    if (status == 0) return buffer_insercion.distancia_total - dist_original;

    if (status == 1) {
        int idx_falla = -1;
        for (int i = 1; i < (int)buffer_insercion.secuencia.size(); ++i) {
            if (buffer_insercion.secuencia[i].llegada_bateria < -1e-9) { idx_falla = i; break; }
        }

        if (idx_falla != -1) {
            double mejor_costo = numeric_limits<double>::max();
            for (int p = idx_falla - 1; p >= 1; --p) {
                for (int s : estaciones) {
                    buffer_insercion.secuencia.insert(buffer_insercion.secuencia.begin() + p, { s, 0, 0, 0, 0 });
                    if (buffer_insercion.calcular_metricas(mapa, matriz_dist, CAP_BATT, CAP_CARGA, R_CARGA, R_CONSUMO, ID_DEPOSITO) == 0) {
                        double costo = buffer_insercion.distancia_total - dist_original;
                        if (costo < mejor_costo) mejor_costo = costo;
                    }
                    buffer_insercion.secuencia.erase(buffer_insercion.secuencia.begin() + p);
                }
            }
            return mejor_costo;
        }
    }
    return numeric_limits<double>::max();
}

void ALNS_EVRPTW::reconstruir_indice_rutas() {
    fill(ruta_de_nodo.begin(), ruta_de_nodo.end(), -1);
    for (int r = 0; r < (int)solucion_actual.size(); ++r) {
        for (const auto& p : solucion_actual[r].secuencia) {
            ruta_de_nodo[p.nodo_id] = r;
        }
    }
}

void ALNS_EVRPTW::registrar_rutas_en_pool(const vector<Ruta>& sol) {
    for (const auto& r : sol) {
        if (r.violacion_tiempo > 1e-5 || r.violacion_bateria > 1e-5 || r.violacion_capacidad > 1e-5) continue;
        vector<int> clientes;
        for (const auto& p : r.secuencia) {
            if (!mapa[p.nodo_id].es_estacion && !mapa[p.nodo_id].es_deposito) clientes.push_back(p.nodo_id);
        }
        if (clientes.empty()) continue;
        sort(clientes.begin(), clientes.end());
        string firma;
        for (int c : clientes) { firma += to_string(c); firma += ','; }

        // Evitar guardar una secuencia identica repetida (mismo orden,
        // mismas estaciones) para no inflar el pool con copias exactas.
        bool ya_existe_identica = false;
        auto it_firma = firmas_pool.find(firma);
        if (it_firma != firmas_pool.end()) {
            for (int idx : it_firma->second) {
                if (pool_rutas[idx].secuencia_completa.size() == r.secuencia.size()) {
                    bool igual = true;
                    for (size_t i = 0; i < r.secuencia.size(); ++i) {
                        if (pool_rutas[idx].secuencia_completa[i].nodo_id != r.secuencia[i].nodo_id) { igual = false; break; }
                    }
                    if (igual) { ya_existe_identica = true; break; }
                }
            }
        }
        if (ya_existe_identica) continue;

        // Agregamos la nueva variante como candidata.
        int nuevo_idx = (int)pool_rutas.size();
        pool_rutas.push_back({ clientes, r.distancia_total, r.secuencia });
        firmas_pool[firma].push_back(nuevo_idx);

        // Si superamos el limite de variantes para este grupo de clientes,
        // descartamos la peor (mayor distancia). No la borramos fisicamente
        // de pool_rutas (para no reindexar todo), solo la sacamos de la
        // lista de indices activos de esta firma y la marcamos invalida.
        auto& indices = firmas_pool[firma];
        if ((int)indices.size() > K_VARIANTES_POR_GRUPO) {
            int peor_pos = 0;
            double peor_dist = pool_rutas[indices[0]].distancia;
            for (int k = 1; k < (int)indices.size(); ++k) {
                if (pool_rutas[indices[k]].distancia > peor_dist) {
                    peor_dist = pool_rutas[indices[k]].distancia;
                    peor_pos = k;
                }
            }
            pool_rutas[indices[peor_pos]].clientes.clear(); // marca "invalida": sin clientes no cubre nada
            indices.erase(indices.begin() + peor_pos);
        }
    }
}

vector<int> ALNS_EVRPTW::rutas_candidatas_para(int u) {
    static vector<bool> marca; // Reutilizable entre llamadas.
    if ((int)marca.size() < (int)solucion_actual.size()) marca.resize(solucion_actual.size(), false);

    vector<int> resultado;
    for (int v : vecinos_cercanos[u]) {
        int r = ruta_de_nodo[v];
        if (r != -1 && r < (int)marca.size() && !marca[r]) {
            marca[r] = true;
            resultado.push_back(r);
        }
    }
    for (int r : resultado) marca[r] = false;

    if (resultado.empty()) {
        resultado.reserve(solucion_actual.size());
        for (int r = 0; r < (int)solucion_actual.size(); ++r) resultado.push_back(r);
    }
    return resultado;
}

bool ALNS_EVRPTW::aplicar_insercion(int r_idx, int pos, int nodo_id) {
    solucion_actual[r_idx].secuencia.insert(solucion_actual[r_idx].secuencia.begin() + pos, { nodo_id, 0, 0, 0, 0 });
    int status = solucion_actual[r_idx].calcular_metricas(mapa, matriz_dist, CAP_BATT, CAP_CARGA, R_CARGA, R_CONSUMO, ID_DEPOSITO);

    if (status == 0) return true;

    if (status == 1) {
        int idx_falla = -1;
        for (int i = 1; i < (int)solucion_actual[r_idx].secuencia.size(); i++) {
            if (solucion_actual[r_idx].secuencia[i].llegada_bateria < 0) { idx_falla = i; break; }
        }

        if (idx_falla != -1) {
            double mejor_dist = numeric_limits<double>::max();
            int mejor_estacion = -1, mejor_pos = -1;

            for (int p = idx_falla - 1; p >= 1; --p) {
                for (int s : estaciones) {
                    solucion_actual[r_idx].secuencia.insert(solucion_actual[r_idx].secuencia.begin() + p, { s, 0, 0, 0, 0 });
                    if (solucion_actual[r_idx].calcular_metricas(mapa, matriz_dist, CAP_BATT, CAP_CARGA, R_CARGA, R_CONSUMO, ID_DEPOSITO) == 0) {
                        if (solucion_actual[r_idx].distancia_total < mejor_dist) {
                            mejor_dist = solucion_actual[r_idx].distancia_total;
                            mejor_estacion = s; mejor_pos = p;
                        }
                    }
                    solucion_actual[r_idx].secuencia.erase(solucion_actual[r_idx].secuencia.begin() + p);
                }
            }

            if (mejor_estacion != -1) {
                solucion_actual[r_idx].secuencia.insert(solucion_actual[r_idx].secuencia.begin() + mejor_pos, { mejor_estacion, 0, 0, 0, 0 });
                solucion_actual[r_idx].calcular_metricas(mapa, matriz_dist, CAP_BATT, CAP_CARGA, R_CARGA, R_CONSUMO, ID_DEPOSITO);
                return true;
            }
        }
    }
    solucion_actual[r_idx].secuencia.erase(solucion_actual[r_idx].secuencia.begin() + pos);
    solucion_actual[r_idx].calcular_metricas(mapa, matriz_dist, CAP_BATT, CAP_CARGA, R_CARGA, R_CONSUMO, ID_DEPOSITO);
    return false;
}

void ALNS_EVRPTW::evaluar_y_aceptar(vector<Ruta>& backup, int op_rem, int op_ins, int op_sr, int op_si, int iteracion_actual) {
    int tipo = 0;
    if (mejor(solucion_actual, mejor_solucion)) {
        mejor_solucion = solucion_actual; ultima_mejora = iteracion_actual; tipo = 1;
    }
    else if (config.criterio == ConfigALNS::Criterio::SOLO_MEJORA) {
        if (mejor(solucion_actual, backup)) tipo = 2;
    }
    else {
        if (mejor(solucion_actual, backup)) tipo = 2;
        else {
            double delta = calcular_costo(solucion_actual) - calcular_costo(backup);
            if (dist_01(rng) < exp(-delta / T)) tipo = 3;
        }
    }
    if (tipo == 0) solucion_actual = backup;
    if (tipo > 0) {
        registrar_rutas_en_pool(solucion_actual);
        double premio = (tipo == 1) ? 25 : (tipo == 2) ? 20 : 9;
        if (op_rem != -1) scores_rem[op_rem] += premio;
        if (op_ins != -1) scores_ins[op_ins] += premio;
        if (op_sr != -1) scores_sr[op_sr] += premio;
        if (op_si != -1) scores_si[op_si] += premio;
    }
}

void ALNS_EVRPTW::actualizar_rutas_post_remocion(const vector<int>& fuera) {
    vector<bool> remover(mapa.size(), false);
    for (int id : fuera) remover[id] = true;

    for (auto& ruta : solucion_actual) {
        auto& seq = ruta.secuencia;
        bool hubo_cambio = false;

        seq.erase(remove_if(seq.begin() + 1, seq.end() - 1, [&](const Parada& p) {
            bool rm = remover[p.nodo_id];
            if (rm) hubo_cambio = true;
            return rm;
            }), seq.end() - 1);

        if (!hubo_cambio) continue;

        ruta.calcular_metricas(mapa, matriz_dist, CAP_BATT, CAP_CARGA, R_CARGA, R_CONSUMO, ID_DEPOSITO);

        for (int i = 1; i < (int)ruta.secuencia.size() - 1; ) {
            if (mapa[ruta.secuencia[i].nodo_id].es_estacion) {
                int id_estacion = ruta.secuencia[i].nodo_id;
                ruta.secuencia.erase(ruta.secuencia.begin() + i);
                if (ruta.calcular_metricas(mapa, matriz_dist, CAP_BATT, CAP_CARGA, R_CARGA, R_CONSUMO, ID_DEPOSITO) != 0) {
                    ruta.secuencia.insert(ruta.secuencia.begin() + i, { id_estacion, 0, 0, 0, 0 });
                    ruta.calcular_metricas(mapa, matriz_dist, CAP_BATT, CAP_CARGA, R_CARGA, R_CONSUMO, ID_DEPOSITO);
                    ++i;
                }
            }
            else ++i;
        }
    }
}

void ALNS_EVRPTW::limpiar_rutas_vacias() {
    solucion_actual.erase(remove_if(solucion_actual.begin(), solucion_actual.end(), [&](const Ruta& r) {
        for (const auto& p : r.secuencia) {
            if (!mapa[p.nodo_id].es_estacion && !mapa[p.nodo_id].es_deposito) return false;
        }
        return true;
        }), solucion_actual.end());
}

int ALNS_EVRPTW::seleccionar_operador(const vector<double>& pesos) {
    double sum = 0; for (double d : pesos) sum += d;
    if (sum == 0) return 0;
    double r = dist_01(rng) * sum;
    double acc = 0;
    for (int i = 0; i < (int)pesos.size(); ++i) {
        acc += pesos[i];
        if (r <= acc) return i;
    }
    return pesos.size() - 1;
}

void ALNS_EVRPTW::actualizar_pesos_clientes() {
    double rho = 0.25;
    for (int i = 0; i < (int)pesos_rem.size(); ++i) {
        if (usos_rem[i] > 0) pesos_rem[i] = pesos_rem[i] * (1.0 - rho) + rho * (scores_rem[i] / usos_rem[i]);
        scores_rem[i] = 0; usos_rem[i] = 0;
    }
    for (int i = 0; i < (int)pesos_ins.size(); ++i) {
        if (usos_ins[i] > 0) pesos_ins[i] = pesos_ins[i] * (1.0 - rho) + rho * (scores_ins[i] / usos_ins[i]);
        scores_ins[i] = 0; usos_ins[i] = 0;
    }
}

void ALNS_EVRPTW::actualizar_pesos_estaciones() {
    double rho = 0.25;
    for (int i = 0; i < (int)pesos_sr.size(); ++i) {
        if (usos_sr[i] > 0) pesos_sr[i] = pesos_sr[i] * (1.0 - rho) + rho * (scores_sr[i] / usos_sr[i]);
        scores_sr[i] = 0; usos_sr[i] = 0;
    }
    for (int i = 0; i < (int)pesos_si.size(); ++i) {
        if (usos_si[i] > 0) pesos_si[i] = pesos_si[i] * (1.0 - rho) + rho * (scores_si[i] / usos_si[i]);
        scores_si[i] = 0; usos_si[i] = 0;
    }
}

bool ALNS_EVRPTW::mejor(const vector<Ruta>& A, const vector<Ruta>& B) const {
    auto es_factible = [](const vector<Ruta>& sol) -> bool {
        for (const auto& r : sol) {
            if (r.violacion_tiempo > 1e-5 || r.violacion_bateria > 1e-5 || r.violacion_capacidad > 1e-5) return false;
        }
        return true;
        };
    bool fact_A = es_factible(A);
    bool fact_B = es_factible(B);
    if (fact_A != fact_B) return fact_A;
    if (A.size() != B.size()) return A.size() < B.size();
    return obtener_distancia(A) < obtener_distancia(B);
}
