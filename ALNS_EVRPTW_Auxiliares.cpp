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
#include <iostream>
#include <unordered_set>
#include <unordered_map>
#include <cstdlib>

using namespace std;

#ifdef VERIFICAR_PREFILTRO
// Contador de candidatos descartados y comprobados (modo de verificacion).
static long verificaciones_prefiltro = 0;
struct ReportePrefiltro { ~ReportePrefiltro() { cerr << "[VERIFICAR_PREFILTRO] candidatos descartados verificados: " << verificaciones_prefiltro << " (ninguno era factible)" << endl; } } reporte_prefiltro_;
#endif

void ALNS_EVRPTW::inicializar_solucion() {
    vector<int> no_asignados;
    no_asignados.reserve(mapa.size());
    for (int i = 0; i < (int)mapa.size(); ++i) {
        if (!mapa[i].es_estacion && !mapa[i].es_deposito) no_asignados.push_back(i);
    }
    solucion_actual.clear();
    solucion_actual.reserve(50);
    if (!config.solucion_inicial_secuencial) { greedy_insertion(no_asignados, false); return; }

    // Algoritmo 1 del paper (construccion secuencial).
    vector<int>& pend = no_asignados;
    while (!pend.empty()) {
        // 1/5: nueva ruta con el cliente no servido mas cercano al deposito
        int bi = 0;
        for (int i = 1; i < (int)pend.size(); ++i)
            if (matriz_dist[ID_DEPOSITO][pend[i]] < matriz_dist[ID_DEPOSITO][pend[bi]]) bi = i;
        crear_nueva_ruta(pend[bi]);
        pend.erase(pend.begin() + bi);
        int r = (int)solucion_actual.size() - 1;
        vector<char> fallido(mapa.size(), 0);
        // 3/7: insertar el cliente que menos aumenta la distancia hasta que no quepa ninguno
        while (true) {
            double mejor = numeric_limits<double>::max(); int mejor_i = -1, mejor_p = -1;
            const Ruta& ruta = solucion_actual[r];
            for (int i = 0; i < (int)pend.size(); ++i) {
                int u = pend[i];
                if (fallido[u] || ruta.carga_total + mapa[u].demanda > CAP_CARGA) continue;
                for (int p = 1; p < (int)ruta.secuencia.size(); ++p) {
                    double inc = probar_insercion(r, p, u);
                    if (inc < mejor) { mejor = inc; mejor_i = i; mejor_p = p; }
                }
            }
            if (mejor_i == -1) break;
            int u = pend[mejor_i];
            if (aplicar_insercion(r, mejor_p, u)) pend.erase(pend.begin() + mejor_i);
            else fallido[u] = 1;
        }
    }
    reconstruir_indice_rutas();
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

bool ALNS_EVRPTW::insertar_en_cualquier_ruta(vector<int>& fuera) {
    while (true) {
        double mejor_costo = numeric_limits<double>::max();
        int mejor_i = -1, mejor_r = -1, mejor_p = -1;
        for (int i = 0; i < (int)fuera.size(); ++i) {
            int u = fuera[i];
            for (int r = 0; r < (int)solucion_actual.size(); ++r) {
                int p = -1;
                double c = mejor_insercion_en_ruta(u, r, p);
                if (c < mejor_costo) { mejor_costo = c; mejor_i = i; mejor_r = r; mejor_p = p; }
            }
        }
        if (mejor_i == -1) return false;
        if (aplicar_insercion(mejor_r, mejor_p, fuera[mejor_i])) {
            ruta_de_nodo[fuera[mejor_i]] = mejor_r;
            invalidar_cache_ruta(fuera, mejor_r);
            fuera.erase(fuera.begin() + mejor_i);
            return true;
        }
        cache_costos[fuera[mejor_i]][mejor_r] = -2.0; // fallo al aplicar: seguir buscando
    }
}

double ALNS_EVRPTW::probar_insercion(int r_idx, int pos, int u) {
    const Ruta& ruta = solucion_actual[r_idx];
    const double dist_original = ruta.distancia_total;
    // Time Based Insertion (paper): el costo es el incremento del tiempo
    // total de la ruta (llegada al deposito) en vez de la distancia.
    const bool por_tiempo = modo_costo_tiempo;
    const double tiempo_original = ruta.secuencia.empty() ? 0.0 : ruta.secuencia.back().salida_tiempo;
    auto costo_de = [&](double dist_nueva, double tiempo_nuevo) {
        return por_tiempo ? (tiempo_nuevo - tiempo_original) : (dist_nueva - dist_original);
    };

    // Cambio 4: evaluar sin copiar la ruta ni insertar en el vector. Ademas
    // se recogen, parada a parada, las cotas que usa el prefiltro de abajo.
    static thread_local EstadoVirtual estado;
    double dist = 0.0, tfin = 0.0;
    int idx_falla = -1;
    int status = ruta.calcular_metricas_con_insercion_virtual(mapa, matriz_dist, CAP_BATT, CAP_CARGA,
        R_CARGA, R_CONSUMO, ID_DEPOSITO, pos, u, -1, -1, dist, idx_falla,
        config.usar_prefiltro_exacto ? &estado : nullptr, &tfin);

    if (status == 0) return costo_de(dist, tfin);
    if (status != 1 || idx_falla < 2) return numeric_limits<double>::max();

    // Fallo de bateria: buscar una estacion de reparacion en las MAX_POS
    // posiciones anteriores al punto de fallo (misma politica que el original).
    static const int N_EST = 21;   // numero de estaciones mas cercanas a probar (busqueda exhaustiva)
    const int n_virt = (int)ruta.secuencia.size() + 1;
    const int id_falla_nodo = (idx_falla == pos) ? u
        : ruta.secuencia[idx_falla > pos ? idx_falla - 1 : idx_falla].nodo_id;

    // Nodo en la posicion k de la secuencia virtual "ruta con u insertado en pos".
    auto nodo_en = [&](int k) -> int {
        if (k == pos) return u;
        return ruta.secuencia[k > pos ? k - 1 : k].nodo_id;
    };

    // Primera posicion donde se prueba la estacion: MAX_POS hacia atras, o
    // (paper) justo despues del ultimo punto de carga anterior al fallo.
    int pos_min;
    if (config.max_posiciones_estacion > 0) pos_min = max(1, idx_falla - config.max_posiciones_estacion);
    else {
        pos_min = 1;
        for (int k = idx_falla - 1; k >= 1; --k) {
            int id = nodo_en(k);
            if (mapa[id].es_estacion || mapa[id].es_deposito) { pos_min = k + 1; break; }
        }
    }

    // ------------------------------------------------------------------
    //  Prefiltro EXACTO (sustituye a la poda heuristica top-k del informe).
    //  Medido en rc104_21: el 99 % de las evaluaciones (posicion, estacion)
    //  fallan, y casi todas por una de estas tres causas, que se detectan
    //  con cotas en O(1) sin recorrer la ruta:
    //    (a) la estacion s no se alcanza ni con bateria llena desde el
    //        ultimo punto de carga anterior a p;
    //    (b) desde s, ni con bateria llena se llega al siguiente punto de
    //        carga (o al deposito) pasando por el nodo siguiente;
    //    (c) aun sin cargar nada en la ruta, el desvio por s hace llegar
    //        tarde a s o al nodo siguiente.
    //  Cada cota es valida para cualquier politica de carga, y se aplica
    //  con un margen (MARGEN) muy superior al error de redondeo, de modo
    //  que NUNCA descarta un candidato que calcular_metricas aceptaria:
    //  el resultado es identico al de la busqueda exhaustiva.
    // ------------------------------------------------------------------
    static thread_local vector<double> energia_hasta_prox; // desde A[i] hasta el proximo punto de carga
    const bool prefiltro = config.usar_prefiltro_exacto;
    if (prefiltro) {
        if ((int)energia_hasta_prox.size() < n_virt) energia_hasta_prox.resize(n_virt);
        double acum = 0.0;
        int ultimo = nodo_en(n_virt - 1);
        if (!mapa[ultimo].es_deposito) acum = matriz_dist[ultimo][ID_DEPOSITO] * R_CONSUMO;
        energia_hasta_prox[n_virt - 1] = acum;
        int id_sig = ultimo;
        for (int i = n_virt - 2; i >= pos_min; --i) {
            int id_i = nodo_en(i);
            double tramo = matriz_dist[id_i][id_sig] * R_CONSUMO;
            acum = (mapa[id_sig].es_estacion || mapa[id_sig].es_deposito) ? tramo : tramo + acum;
            energia_hasta_prox[i] = acum;
            id_sig = id_i;
        }
    }
    static const double MARGEN = 1e-6;

    double mejor_costo = numeric_limits<double>::max();
    auto buscar_estacion = [&](bool solo_topk) {
        for (int p = idx_falla - 1; p >= pos_min; --p) {
            const int id_prev = nodo_en(p - 1);
            const int id_next = nodo_en(p);
            const vector<int>* cands;
            int n_cands;
            if (solo_topk) {
                // Cambio 3 (opcional): solo las k estaciones con menor desvio
                // para el arco (id_prev, id_next) donde se insertaria la estacion.
                cands = &estaciones_candidatas_arco[id_prev][id_next];
                n_cands = (int)cands->size();
            }
            else {
                cands = &estaciones_cercanas[id_falla_nodo];
                n_cands = min(N_EST, (int)cands->size());
            }
            for (int k = 0; k < n_cands; ++k) {
                int s = (*cands)[k];
                if (prefiltro) {
                    const Nodo& ns = mapa[s];
                    const Nodo& nn = mapa[id_next];
                    double d_prev_s = matriz_dist[id_prev][s];
                    double d_s_next = matriz_dist[s][id_next];
                    bool descartar = false;
                    // (a) bateria para llegar a s
                    if (estado.energia_desde_ultimo_punto_carga[p - 1] + d_prev_s * R_CONSUMO > CAP_BATT + MARGEN) descartar = true;
                    // (b) bateria desde s hasta el proximo punto de carga
                    else if (d_s_next * R_CONSUMO + energia_hasta_prox[p] > CAP_BATT + MARGEN) descartar = true;
                    else {
                        // (c) ventanas de tiempo con tiempo minimo (sin cargar)
                        double t_s = estado.tiempo_salida_minimo[p - 1] + d_prev_s;
                        if (t_s > ns.due_date + 1e-9 + MARGEN) descartar = true;
                        else if (max(t_s, ns.ready_time) + d_s_next > nn.due_date + 1e-9 + MARGEN) descartar = true;
                    }
                    if (descartar) {
#ifdef VERIFICAR_PREFILTRO
                        // Modo de verificacion: evaluar de todos modos y abortar si el
                        // candidato descartado era factible (nunca deberia ocurrir).
                        double dv = 0.0; int fv = -1;
                        if (ruta.calcular_metricas_con_insercion_virtual(mapa, matriz_dist, CAP_BATT, CAP_CARGA,
                            R_CARGA, R_CONSUMO, ID_DEPOSITO, pos, u, p, s, dv, fv) == 0) {
                            cerr << "[VERIFICAR_PREFILTRO] candidato factible descartado: ruta " << r_idx
                                 << " u=" << u << " pos=" << pos << " p=" << p << " s=" << s << endl;
                            abort();
                        }
                        verificaciones_prefiltro++;
#endif
                        continue;
                    }
                }
                double dist_s = 0.0, tfin_s = 0.0;
                int dummy = -1;
                if (ruta.calcular_metricas_con_insercion_virtual(mapa, matriz_dist, CAP_BATT, CAP_CARGA,
                    R_CARGA, R_CONSUMO, ID_DEPOSITO, pos, u, p, s, dist_s, dummy, nullptr, &tfin_s) == 0) {
                    double costo = costo_de(dist_s, tfin_s);
                    if (costo < mejor_costo) mejor_costo = costo;
                }
            }
        }
    };

    if (config.usar_topk_en_probar_insercion) {
        buscar_estacion(true);
        // Red de seguridad: si la poda no encontro reparacion, confirmar con
        // la busqueda exhaustiva antes de declarar la insercion infactible.
        if (mejor_costo == numeric_limits<double>::max() && config.topk_fallback_exhaustivo) {
            buscar_estacion(false);
        }
    }
    else {
        buscar_estacion(false);
    }
    return mejor_costo;
}

double ALNS_EVRPTW::mejor_insercion_en_ruta(int u, int r, int& mejor_pos, bool con_ruido) {
    mejor_pos = -1;

    // Cambio 1: reutilizar la cache si la entrada (cliente, ruta) es valida.
    double cacheado = cache_costos[u][r];
    if (cacheado == -2.0) return numeric_limits<double>::max();

    if (cacheado == -1.0) {
        // Evaluar de verdad y guardar costo, posicion e incrementos por posicion.
        const auto& ruta = solucion_actual[r];
        auto& incs = cache_incs[u][r];
        incs.clear();
        double costo_ruta = numeric_limits<double>::max();
        if (ruta.carga_total + mapa[u].demanda <= CAP_CARGA) {
            int n_pos = (int)ruta.secuencia.size() - 1;
            incs.resize(max(0, n_pos), numeric_limits<double>::max());
            for (int p = 1; p < (int)ruta.secuencia.size(); ++p) {
                double inc = probar_insercion(r, p, u);
                incs[p - 1] = inc;
                if (inc < costo_ruta) { costo_ruta = inc; mejor_pos = p; }
            }
        }
        if (costo_ruta == numeric_limits<double>::max()) {
            cache_costos[u][r] = -2.0; cache_pos[u][r] = -1;
            return costo_ruta;
        }
        cache_costos[u][r] = costo_ruta; cache_pos[u][r] = mejor_pos;
        cacheado = costo_ruta;
    }

    if (!con_ruido) {
        mejor_pos = cache_pos[u][r];
        return cacheado;
    }

    // Greedy con ruido: ruido fresco por posicion sobre los incrementos
    // cacheados, en el mismo orden y con las mismas extracciones del RNG que
    // el codigo original (asi la trayectoria es identica, solo mas rapida).
    const auto& incs = cache_incs[u][r];
    double costo_ruta = numeric_limits<double>::max();
    mejor_pos = -1;
    for (int p = 1; p <= (int)incs.size(); ++p) {
        double inc = incs[p - 1];
        if (inc != numeric_limits<double>::max()) inc *= (0.8 + 0.4 * dist_01(rng));
        if (inc < costo_ruta) { costo_ruta = inc; mejor_pos = p; }
    }
    return costo_ruta;
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
    if (config.evaluar_todas_las_rutas) {
        vector<int> todas(solucion_actual.size());
        for (int r = 0; r < (int)todas.size(); ++r) todas[r] = r;
        return todas;
    }
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
            if (solucion_actual[r_idx].secuencia[i].llegada_bateria < -EPS_BATERIA) { idx_falla = i; break; }
        }
        if (idx_falla != -1 && idx_falla >= 2) {
            const int MAX_POS = config.max_posiciones_estacion;
            static const int N_EST = 21;
            int id_falla = solucion_actual[r_idx].secuencia[idx_falla].nodo_id;
            const auto& ests_ord = estaciones_cercanas[id_falla];
            int n_est = min(N_EST, (int)ests_ord.size());
            double mejor_dist = numeric_limits<double>::max();
            int mejor_estacion = -1, mejor_pos = -1;
            int pos_min = (MAX_POS > 0) ? max(1, idx_falla - MAX_POS) : 1;
            if (MAX_POS <= 0)
                for (int k = idx_falla - 1; k >= 1; --k) {
                    int id = solucion_actual[r_idx].secuencia[k].nodo_id;
                    if (mapa[id].es_estacion || mapa[id].es_deposito) { pos_min = k + 1; break; }
                }

            for (int p = idx_falla - 1; p >= pos_min; --p) {
                for (int k = 0; k < n_est; ++k) {
                    int s = ests_ord[k];
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
        double premio = (tipo == 1) ? config.sigma1 : (tipo == 2) ? config.sigma2 : config.sigma3;
        if (op_rem != -1) scores_rem[op_rem] += premio;
        if (op_ins != -1) scores_ins[op_ins] += premio;
        if (op_sr != -1) scores_sr[op_sr] += premio;
        if (op_si != -1) scores_si[op_si] += premio;
    }
}

void ALNS_EVRPTW::eliminar_clientes_de_ruta(Ruta& ruta, const vector<int>& nodos_a_quitar) {
    if (nodos_a_quitar.empty()) return;
    unordered_set<int> quitar(nodos_a_quitar.begin(), nodos_a_quitar.end());
    auto& seq = ruta.secuencia;
    if (seq.size() < 2) return;

    bool hubo_cambio = false;
    seq.erase(remove_if(seq.begin() + 1, seq.end() - 1, [&](const Parada& p) {
        bool rm = quitar.count(p.nodo_id) > 0;
        if (rm) hubo_cambio = true;
        return rm;
        }), seq.end() - 1);

    if (!hubo_cambio) return;

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

void ALNS_EVRPTW::actualizar_rutas_post_remocion(const vector<int>& fuera) {
    if (fuera.empty()) return;
    for (auto& ruta : solucion_actual) {
        eliminar_clientes_de_ruta(ruta, fuera);
    }
}

void ALNS_EVRPTW::reparar_solapes_covering(vector<Ruta>& sol) {
    unordered_map<int, vector<int>> ocurrencias;
    for (int r = 0; r < (int)sol.size(); ++r) {
        for (const auto& p : sol[r].secuencia) {
            if (!mapa[p.nodo_id].es_estacion && !mapa[p.nodo_id].es_deposito) {
                ocurrencias[p.nodo_id].push_back(r);
            }
        }
    }

    vector<vector<int>> clientes_a_quitar(sol.size());
    int duplicados = 0;

    for (auto& [cliente, rutas] : ocurrencias) {
        if (rutas.size() <= 1) continue;
        duplicados++;

        int mejor_r = rutas[0];
        double mejor_ahorro = -numeric_limits<double>::max();
        for (int r : rutas) {
            const auto& seq = sol[r].secuencia;
            for (int n = 1; n < (int)seq.size() - 1; ++n) {
                if (seq[n].nodo_id != cliente) continue;
                int id_ant = seq[n - 1].nodo_id;
                int id_sig = seq[n + 1].nodo_id;
                double ahorro = matriz_dist[id_ant][cliente] + matriz_dist[cliente][id_sig] - matriz_dist[id_ant][id_sig];
                if (ahorro > mejor_ahorro) { mejor_ahorro = ahorro; mejor_r = r; }
                break;
            }
        }
        for (int r : rutas) if (r != mejor_r) clientes_a_quitar[r].push_back(cliente);
    }

    if (duplicados == 0) return;
    cout << "  [reparacion] " << duplicados << " cliente(s) cubiertos por mas de una ruta; reparando solapes." << endl;

    for (int r = 0; r < (int)sol.size(); ++r) {
        if (!clientes_a_quitar[r].empty()) eliminar_clientes_de_ruta(sol[r], clientes_a_quitar[r]);
    }

    sol.erase(remove_if(sol.begin(), sol.end(), [&](const Ruta& r) {
        for (const auto& p : r.secuencia) {
            if (!mapa[p.nodo_id].es_estacion && !mapa[p.nodo_id].es_deposito) return false;
        }
        return true;
        }), sol.end());
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
