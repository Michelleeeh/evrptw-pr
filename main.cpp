// ============================================================================
//  main.cpp
//  Punto de entrada del programa. Orquesta la ejecucion batch del ALNS
//  sobre todas las instancias EVRPTW encontradas en `carpeta_instancias`:
//    1. Crea (si no existe) la carpeta de salida para los resultados.
//    2. Define las variantes de configuracion del ALNS a correr (por
//       defecto solo "baseline") y las semillas aleatorias a probar con
//       cada variante.
//    3. Para cada archivo .txt de instancia en `carpeta_instancias`:
//         - Parsea la instancia con `leerInstancia`.
//         - Abre un CSV de resultados propio de esa instancia.
//         - Para cada combinacion (variante, semilla), construye un
//           `ALNS_EVRPTW`, ejecuta la metaheuristica (25000 iteraciones)
//           y registra en el CSV: vehiculos, distancia, iteracion de
//           ultima mejora y tiempo de computo.
//    4. Informa por consola el progreso y la ubicacion de los resultados.
// ============================================================================

#include <iostream>
#include <vector>
#include <fstream>
#include <chrono>
#include <filesystem>

#include "Nodo.h"
#include "ConfigALNS.h"
#include "InstanciaReader.h"
#include "ALNS_EVRPTW.h"

using namespace std;
namespace fs = std::filesystem;

int main() {
    // Carpetas de entrada (instancias) y salida (resultados en CSV).
    string carpeta_instancias = "./instancias2/";
    string carpeta_salida = "./resultados/";

    // Crea la carpeta de salida si aun no existe.
    if (!fs::exists(carpeta_salida)) fs::create_directory(carpeta_salida);

    // Lista de variantes de configuracion del ALNS a ejecutar. Cada
    // variante puede activar/desactivar operadores o cambiar parametros
    // (ver ConfigALNS); aqui solo se define la variante "baseline" con
    // los valores por defecto.
    vector<ConfigALNS> variantes;

    ConfigALNS baseline;
    baseline.nombre_variante = "baseline";
    variantes.push_back(baseline);

    // Semillas aleatorias con las que se repetira cada variante, para
    // poder promediar/comparar resultados entre corridas independientes.
    vector<unsigned int> semillas = { 222 };

    // Si no existe la carpeta de instancias, no hay nada que procesar.
    if (!fs::exists(carpeta_instancias)) {
        cerr << "[!] No existe la carpeta de instancias: "
            << carpeta_instancias << endl;
        return 1;
    }

    // Recorre cada archivo .txt de la carpeta de instancias.
    for (const auto& entry : fs::directory_iterator(carpeta_instancias)) {
        if (entry.path().extension() != ".txt") continue;
        string ruta_archivo = entry.path().string();
        string nombre_instancia = entry.path().stem().string();

        // Parsea la instancia: nodos (deposito/clientes/estaciones) y
        // parametros del vehiculo electrico (capacidad de carga, bateria,
        // tasa de recarga y de consumo).
        vector<Nodo> mapa;
        double cc, cb, rc, rcons;
        if (!leerInstancia(ruta_archivo, mapa, cc, cb, rc, rcons)) {
            cout << "[!] Error al leer: " << ruta_archivo << endl;
            continue;
        }

        // Archivo CSV de resultados especifico de esta instancia, con una
        // fila por cada combinacion (variante, semilla) ejecutada.
        string archivo_salida_instancia = carpeta_salida + "resultados_" + nombre_instancia + ".csv";
        ofstream out(archivo_salida_instancia);
        out << "Instancia,Variante,Semilla,Vehiculos,Distancia,Ultima_Mejora,Tiempo_Segundos\n";

        // Ejecuta el ALNS para cada variante de configuracion y cada
        // semilla, registrando el resultado y el tiempo de computo.
        for (auto config : variantes) {
            for (unsigned int semilla : semillas) {
                config.semilla = semilla;
                cout << "\n=== " << nombre_instancia << " | " << config.nombre_variante
                    << " | semilla " << semilla << " ===" << endl;

                ALNS_EVRPTW alns(mapa, cc, cb, rc, rcons, config);
                auto start_time = chrono::high_resolution_clock::now();
                alns.ejecutar(25000, nombre_instancia);
                auto end_time = chrono::high_resolution_clock::now();
                chrono::duration<double> duration = end_time - start_time;

                // Registra la fila de resultados de esta corrida.
                out << nombre_instancia << "," << config.nombre_variante << ","
                    << semilla << "," << alns.mejor_solucion.size() << ","
                    << alns.obtener_distancia(alns.mejor_solucion) << ","
                    << alns.ultima_mejora << "," << duration.count() << "\n";
                out.flush();
            }
        }

        out.close();
        cout << "Resultados de " << nombre_instancia << " guardados en: " << archivo_salida_instancia << endl;
    }

    cout << "\nProceso completado. Resultados individuales en carpeta: " << carpeta_salida << endl;
    return 0;
}
