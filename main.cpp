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
#include <algorithm>
#include <limits>

#include "Nodo.h"
#include "ConfigALNS.h"
#include "InstanciaReader.h"
#include "ALNS_EVRPTW.h"

using namespace std;
namespace fs = std::filesystem;

int main() {
    // Carpetas de entrada (instancias) y salida (resultados en CSV).
    string carpeta_instancias = "./instancias/";
    string carpeta_salida = "./resultados/";

    // Crea la carpeta de salida si aun no existe.
    if (!fs::exists(carpeta_salida)) fs::create_directory(carpeta_salida);

    // Lista de variantes de configuracion del ALNS a ejecutar. Cada
    // variante puede activar/desactivar operadores o cambiar parametros
    // (ver ConfigALNS); aqui solo se define la variante "baseline" con
    // los valores por defecto.
    vector<ConfigALNS> variantes;

    ConfigALNS sin_frvcp;
    sin_frvcp.nombre_variante = "sin_frvcp";
    sin_frvcp.usar_frvcp_en_final = false;
    sin_frvcp.usar_frvcp_en_pool = false;
    variantes.push_back(sin_frvcp);

    //ConfigALNS con_frvcp;
    //con_frvcp.nombre_variante = "con_frvcp";
    //con_frvcp.usar_frvcp_en_final = true;
    //con_frvcp.usar_frvcp_en_pool = true;
    //variantes.push_back(con_frvcp);

    // Semillas aleatorias con las que se repetira cada variante, para
    // poder promediar/comparar resultados entre corridas independientes.
    vector<unsigned int> semillas = { 111 };

    // Si no existe la carpeta de instancias, no hay nada que procesar.
    if (!fs::exists(carpeta_instancias)) {
        cerr << "[!] No existe la carpeta de instancias: "
            << carpeta_instancias << endl;
        return 1;
    }

    // Listar todos los archivos .txt disponibles y ordenarlos alfabeticamente.
    vector<fs::path> archivos_disponibles;
    for (const auto& entry : fs::directory_iterator(carpeta_instancias)) {
        if (entry.path().extension() == ".txt") {
            archivos_disponibles.push_back(entry.path());
        }
    }
    sort(archivos_disponibles.begin(), archivos_disponibles.end());

    if (archivos_disponibles.empty()) {
        cerr << "[!] No hay archivos .txt en: " << carpeta_instancias << endl;
        return 1;
    }

    // Mostrar menu de seleccion.
    cout << "\n=== Instancias disponibles ===" << endl;
    for (size_t i = 0; i < archivos_disponibles.size(); ++i) {
        cout << "  " << (i + 1) << ". " << archivos_disponibles[i].stem().string() << endl;
    }

    int seleccion = 0;
    while (true) {
        cout << "\nSeleccione el numero de la instancia a ejecutar (1-"
             << archivos_disponibles.size() << "): ";
        if (cin >> seleccion && seleccion >= 1 && seleccion <= (int)archivos_disponibles.size()) {
            break;
        }
        cout << "[!] Seleccion invalida. Intente de nuevo." << endl;
        cin.clear();
        cin.ignore(numeric_limits<streamsize>::max(), '\n');
    }

    // Procesar solo la instancia seleccionada.
    {
        const auto& ruta_path = archivos_disponibles[seleccion - 1];
        string ruta_archivo = ruta_path.string();
        string nombre_instancia = ruta_path.stem().string();

        // Parsea la instancia: nodos (deposito/clientes/estaciones) y
        // parametros del vehiculo electrico (capacidad de carga, bateria,
        // tasa de recarga y de consumo).
        vector<Nodo> mapa;
        double cc, cb, rc, rcons;
        if (!leerInstancia(ruta_archivo, mapa, cc, cb, rc, rcons)) {
            cout << "[!] Error al leer: " << ruta_archivo << endl;
            return 1;
        }

        // Crear subcarpeta propia para esta instancia: ./resultados/r107_21/
        string carpeta_instancia = carpeta_salida + nombre_instancia + "/";
        if (!fs::exists(carpeta_instancia)) fs::create_directory(carpeta_instancia);

        // Archivo CSV de resultados especifico de esta instancia.
        string archivo_salida_instancia = carpeta_instancia + "resultados_" + nombre_instancia + ".csv";
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
                alns.ejecutar(25000, nombre_instancia, carpeta_instancia);
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
        cout << "Resultados de " << nombre_instancia << " guardados en: " << carpeta_instancia << endl;
    }

    cout << "\nProceso completado. Resultados individuales en carpeta: " << carpeta_salida << endl;
    return 0;
}